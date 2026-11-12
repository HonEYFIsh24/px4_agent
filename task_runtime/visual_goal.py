"""Visual decisions for relative goals; geometry supplies execution, not semantics."""
import copy
import math
from dataclasses import asdict

from stereo_localization import finite, vector, matvec, quaternion_rotation, iris_left_optical_to_body
from target_confirmation import observations_for
from viewpoint_planner import map_revision
from .contracts import GoalSpec, GoalPoseCandidate
from .goal_planner import camera_facing_yaw
from .memory import SharedTargetMemory


def image_ground_point(snapshot, normalized_uv, ground_z):
    """Intersect an observed image ray with the registered static ground plane."""
    uv = vector(normalized_uv, 2, 'normalized ground pixel')
    if any(not 0 <= v <= 1 for v in uv):
        raise ValueError('Ground pixel must be within the current image, normalized to 0..1')
    pose, info = snapshot['body_pose_enu'], snapshot['camera_intrinsics']
    if pose.get('frame_id') not in ('map', 'local_enu') or pose.get('body_reference') != 'base_link':
        raise ValueError('Ground projection requires the saved ENU camera pose')
    k = vector(info['K'], 9, 'camera intrinsics')
    if k[0] <= 0 or k[4] <= 0:
        raise ValueError('Ground projection requires actual positive focal lengths')
    rotation = quaternion_rotation(pose['orientation_xyzw']);mount = iris_left_optical_to_body()
    origin = [a+b for a,b in zip(pose['position_m'], matvec(rotation, mount['translation_m']))]
    ray = matvec(rotation, matvec(mount['rotation'],
        [(uv[0]*info['width']-k[2])/k[0], (uv[1]*info['height']-k[5])/k[4], 1.]))
    if ray[2] >= 0:
        raise ValueError('Chosen pixel does not look down onto the supported ground plane')
    distance = (finite(ground_z, 'registered ground height')-origin[2])/ray[2]
    point = [a+distance*b for a,b in zip(origin, ray)]
    if distance <= 0 or math.dist(origin, point) > 20.:
        raise ValueError('Chosen ground pixel is beyond the supported camera range')
    return point


class VisualGoalSkill:
    def __init__(self, runtime):
        self.runtime = runtime

    def choose(self, owner, capture_id, decision, description, ground_point_uv=None):
        r = self.runtime;r.require_live(owner)
        s = r.sessions[owner];stage = r.stage(owner);goal = s['goal_state']
        if goal.get('phase') != 'choose_goal' or capture_id != goal['view_capture_id']:
            raise ValueError('Visual goal choice must use the currently delivered goal image')
        if decision not in ('here', 'move', 'observe') or not description.strip():
            raise ValueError('Describe the visual decision: here, move, or observe')
        frame = r.frame_store.get(owner, capture_id, require_delivered=True)
        semantic = r.skills.search.semantic_maps.maps[owner];child = r.skills.search.sessions[owner]
        if frame['snapshot']['coordinate_epoch'] != semantic.coordinate_epoch:
            raise ValueError('Visual goal image belongs to another coordinate epoch')
        goal['visual_decision'] = {'capture_id': capture_id, 'decision': decision, 'description': description,
            'ground_point_uv': copy.deepcopy(ground_point_uv),
            'image_sha256': copy.deepcopy(frame['snapshot']['stereo_image_sha256'])}
        if decision == 'observe' and ground_point_uv is None:
            goal['phase'] = 'request_observation'
        else:
            state = r.client.state();pose = state['body_pose_enu'];point = list(pose['position_m'])
            if stage.altitude_m is not None:point[2] = stage.altitude_m
            try:
                if ground_point_uv is not None:
                    ground = image_ground_point(frame['snapshot'], ground_point_uv, semantic.grid['ground_enu_z_m'])
                    point[:2] = ground[:2]
                elif decision != 'here':
                    raise ValueError('Choose a visible ground pixel for a move, or request another observation')
            except ValueError as exc:
                return {'status': 'not_executed', 'reason': str(exc), 'next_tool': 'choose_task_goal'}
            reference = s['targets'][stage.target_ref]
            matches = observations_for(semantic, child['query'], capture_id)
            observed = next((o for l,o in matches if l.id == reference['target_id']),
                            matches[0][1] if len(matches) == 1 else None)
            yaw = state['yaw_enu_rad']
            if ground_point_uv is not None and observed and observed.get('position_enu_m') is not None:
                yaw = camera_facing_yaw(point, observed['position_enu_m'])
            spec = GoalSpec(reference['target_id'], stage.relation, stage.kind)
            revision = map_revision(semantic)
            version = SharedTargetMemory(semantic).view(reference['target_id'])['target_revision']
            candidate = GoalPoseCandidate('goal_001', point, yaw, asdict(spec), [copy.deepcopy(goal['visual_decision'])],
                version, revision, copy.deepcopy(semantic.coordinate_epoch),
                {'identity': True, 'visual_proposal': True, 'permission_volume': True, 'path': 'unknown'}, {'relation': 1.})
            generated = {'status': 'candidates', 'candidates': [asdict(candidate)], 'target_revision': version,
                'target_anchor_enu_m': None, 'map_revision': revision, 'requires_target_visibility': False,
                'goal_context_version': 2, 'scoring': 'model_selected_visual_goal'}
            prepared = r.goal_execution.prepare(owner, stage, s['plan'].plan_revision, generated)
            if prepared['status'] != 'ready':
                return {'status': 'not_executed', 'reason': 'Chosen position has no executable safe path; choose another visible area',
                    'next_tool': 'choose_task_goal'}
            goal.update(prepared=prepared, phase='ready', source_capture_id=capture_id)
        s['revision'] += 1;s['issued_calls'].clear()
        return r.emit(owner, 'visual_goal_selected' if goal['phase'] == 'ready' else 'visual_goal_observation_requested')

    def advance(self, owner, stage):
        r = self.runtime;s = r.sessions[owner];goal = s['goal_state'];child = r.skills.search.sessions[owner]
        if goal['phase'] in ('verification', 'observing'):
            capture = goal.get('verification_capture_id')
            if capture and any(o['capture_id'] == capture for o in child['observations']):
                goal.update(view_capture_id=capture, phase='review_visual_goal')
                r.emit(owner, 'visual_goal_awaiting_review')
            elif goal['phase'] == 'observing' and child['observations'][-1]['capture_id'] != goal['view_capture_id']:
                goal.update(view_capture_id=child['observations'][-1]['capture_id'], phase='choose_goal')
            return False
        if goal['phase'] == 'request_observation':
            if len(child['observations']) >= s['limits'].max_observations-1:
                stage.reason_code = 'VISUAL_GOAL_UNCERTAIN';r.finish(owner, False);return False
            semantic = r.skills.search.semantic_maps.maps[owner]
            reference = s['targets'][stage.target_ref]
            observed = SharedTargetMemory(semantic).last_reliable_observation(reference['target_id'], child['query'])
            r.frame_store.get(owner, observed['capture_id'], require_delivered=True)
            r.frame_store.pin_confirmation_source(owner, observed['capture_id'])
            s['revision'] += 1
            child.update(status='searching', activity_closed=False, arrived=False, pending=None,
                         search_phase='confirming', target_confirmation_mode='route_observation',
                         handoff_evidence_profile='route_observation', target_focus_state='reacquiring',
                         confirmation={'target_id': reference['target_id'], 'source_capture_id': observed['capture_id'],
                                       'purpose': 'recovery'},
                         nbv_awaiting_selection=False, program_verification=True)
            child['observation_ceiling'] = s['limits'].max_observations-1
            child.pop('front_relation_quality', None)
            goal.update(phase='observing', verification_capture_id=None)
            r.skills.search.next_view(owner, 'auto', goal['visual_decision']['description'], preparation_source='visual_goal')
            if r.skills.search.summary(owner)['search_status'] in ('no_reachable_view', 'coverage_exhausted', 'budget_exhausted'):
                stage.reason_code = 'NO_REACHABLE_OBSERVATION';r.finish(owner, False)
            return False
        if goal['phase'] == 'authorize_landing':
            landing = r.landing_skill.prepare(owner, stage, s['plan'].plan_revision, goal, s['targets'][stage.target_ref])
            if landing['status'] == 'not_executed':
                goal.update(phase='choose_goal', execution_feedback=landing.get('reason', landing.get('reason_code')))
                return False
            if landing['status'] != 'authorized':
                raise ValueError('Landing executor returned an invalid authorization')
            goal.update(landing=landing, phase='landing_ready')
            stage.conditions['landing_support_and_corridor'] = 'true'
        return False

    def review(self, owner, capture_id, status, same_target, description):
        r = self.runtime;r.require_live(owner)
        s = r.sessions[owner];stage = r.stage(owner);goal = s['goal_state']
        if goal['phase'] != 'review_visual_goal' or capture_id != goal['verification_capture_id']:
            raise ValueError('Goal review requires the actual fresh post-arrival image')
        if status not in ('satisfied', 'unsatisfied', 'uncertain') or not description.strip():
            raise ValueError('Goal review requires a visual conclusion and its reason')
        frame = r.frame_store.get(owner, capture_id, require_delivered=True)
        source = r.frame_store.get(owner, goal['source_capture_id'], require_delivered=True)
        if (frame['snapshot']['stamp_sim_s'] <= source['snapshot']['stamp_sim_s']
                or frame['snapshot']['coordinate_epoch'] != source['snapshot']['coordinate_epoch']):
            raise ValueError('Goal review image must be new and in the same coordinate epoch')
        review = {'status': 'verified' if status == 'satisfied' and same_target else status,
            'model_status': status, 'same_target': same_target, 'description': description, 'capture_id': capture_id,
            'image_sha256': copy.deepcopy(frame['snapshot']['stereo_image_sha256']),
            'coordinate_epoch': copy.deepcopy(frame['snapshot']['coordinate_epoch']), 'basis': 'model_fresh_visual_relation_review'}
        goal['relation_review'] = review
        s['revision'] += 1;s['issued_calls'].clear()
        if status != 'satisfied' or not same_target:
            goal.update(phase='choose_goal', execution_feedback=description)
            return r.emit(owner, 'visual_goal_not_yet_satisfied')
        stage.conditions.update(visual_relation='true', goal_region='true', fresh_target_verification='true', facing_target='true')
        child = r.skills.search.sessions[owner]
        child['target_visual_confirmed'] = True
        reference = s['targets'][stage.target_ref]
        reference.update(confirmation_level='visual_review', visual_verification_capture_id=capture_id)
        if stage.kind != 'relative_land':
            child.update(status='route_completed', search_phase='exploring', confirmation=None,
                         pending=None, program_verification=False, skill_handoff=copy.deepcopy(reference),
                         nbv_private=None, nbv_public=None, nbv_awaiting_selection=False)
            s.pop('goal_state', None)
            r.complete_stage(owner, [review, {'mission_id': goal['prepared']['execution']['mission_id']}])
            return r.emit(owner, 'visual_goal_verified')
        goal['phase'] = 'authorize_landing'
        return r.emit(owner, 'visual_goal_verified')
