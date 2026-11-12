"""Normal descent after a fresh visual task review and static ground checks.

All lifecycle state lives in the parent goal record. The executor independently
proves support/descent; a visual assertion alone never authorizes landing.
"""
import copy
import math
import time
import uuid

from search_tools import coordinate_epoch
from target_confirmation import observation_for
from viewpoint_planner import map_identity, map_revision
from .goal_planner import GoalPosePlanner
from .memory import SharedTargetMemory


class LandingSkill:
    def __init__(self, search):
        self.search, self.client = search, search.client

    def prepare(self, owner, stage, plan_revision, goal, reference):
        child = self.search.sessions[owner]
        semantic = self.search.semantic_maps.maps[owner]
        self.search.validate_state(owner, self.client.state(), allow_skill_handoff=True)
        review = goal.get('relation_review') or {}
        visual = goal.get('visual_goal') is True
        if (stage.kind != 'relative_land' or not visual and stage.relation != 'object_front'
                or review.get('status') != 'verified'
                or review.get('capture_id') != (goal.get('verification_capture_id') if visual else reference['capture_id'])
                or review.get('coordinate_epoch') != semantic.coordinate_epoch):
            raise ValueError('Landing needs the actual fresh same-target object-front review')
        state = self.client.state()
        execution = goal['prepared']['execution']
        selected = next(c for c in goal['prepared']['candidates'] if c['id'] == execution['candidate_id'])
        if (execution['status'] != 'arrived' or (state.get('mission') or {}).get('id') != execution['mission_id']
                or not GoalPosePlanner.verify_arrival(selected, state['position_enu_m'], state['yaw_enu_rad'], coordinate_epoch(state))):
            raise ValueError('Landing requires the actual owned verified goal-hover mission')
        version = SharedTargetMemory(semantic).view(reference['target_id'])['target_revision']
        context = {'owner': list(owner), 'batch_id': uuid.uuid4().hex,
                   'map_revision': self.search.semantic_maps.checkpoint(owner)}
        goal_context = {'schema_version': 2 if visual else 1, 'stage_id': stage.stage_id,
            'stage_revision': child['authority_stage_revision'], 'plan_revision': plan_revision,
            'target_id': reference['target_id'], 'target_revision': version, 'relation': stage.relation,
            'terminal_action': stage.kind}
        probes = None if visual else [observation_for(semantic, reference['target_id'], reference['capture_id'])[1]['position_enu_m']]
        self.client.request('/navigation/goal-poses', {'operation': 'context', 'pair_id': child['last_stereo_pair_id'],
            'origin_xy_enu_m': child['boundary_origin'][:2], 'expected_identity': map_identity(semantic),
            'search_context': context, 'goal_context': goal_context, 'candidates': [],
            'probe_points_enu_m': probes})
        result = self.client.request('/navigation/authorize-goal-land', {'owner': list(owner),
            'expected_mission_id': execution['mission_id'], 'search_context': context, 'goal_context': goal_context,
            'verification_pair_id': child['last_stereo_pair_id']})
        return {**result, 'context': context, 'goal_context': goal_context,
                'expected_mission_id': execution['mission_id'], 'execution': None,
                'visual_review': copy.deepcopy(review) if visual else None}

    def execute(self, owner, landing):
        semantic = self.search.semantic_maps.maps[owner]
        before = self.client.state()
        self.search.validate_state(owner, before, allow_skill_handoff=True)
        before_ev = before.get('external_vision') or {}
        before_ekf = before_ev.get('ekf') or {}
        velocity = before_ekf.get('local_velocity_reset_counters')
        if before_ev.get('required') is True and (type(before_ekf.get('primary_instance')) is not int
                or not isinstance(velocity, dict) or set(velocity) != {'vxy', 'vz'}
                or any(type(v) is not int or not 0 <= v <= 255 for v in velocity.values())):
            raise ValueError('Landing requires the actual EKF instance and velocity reset identity')
        if (landing.get('status') != 'authorized' or landing.get('execution') is not None
                or map_revision(semantic) != landing['context']['map_revision']):
            raise ValueError('Landing authorization already submitted or evidence/map changed')
        current = SharedTargetMemory(semantic).view(landing['goal_context']['target_id'])
        if current['target_revision'] != landing['goal_context']['target_revision']:
            raise ValueError('Target changed after landing authorization')
        record = {'status': 'submitting', 'started_wall_s': time.time(), 'mission_id': None}
        landing['execution'] = record
        try:
            accepted = self.client.request('/navigation/execute-goal-land', {
                'owner': list(owner), 'ticket': landing['ticket'], 'goal_context': landing['goal_context'],
                'search_context': landing['context'], 'expected_mission_id': landing['expected_mission_id']})
            record.update(status='accepted', mission_id=accepted['mission']['id'])
            result = (self.client.wait_for_photo_action(accepted) if hasattr(self.client, 'wait_for_photo_action')
                      else self.client.wait_for(accepted))
            if ((result.get('mission') or {}).get('status') != 'succeeded'
                    or result.get('armed') is not False or result.get('landed_state') != 1):
                raise ValueError('Target landing has no actual successful ground/disarmed evidence')
            state_available = True
            try:
                state = self.client.state()
                final_epoch = coordinate_epoch(state)
                state_available = bool(final_epoch and final_epoch.get('vio_epoch') is not None
                                       and final_epoch.get('ekf_counters') is not None)
            except (ValueError, KeyError, RuntimeError, OSError):
                state, final_epoch, state_available = result, None, False
            external = state.get('external_vision') or {}
            ekf = external.get('ekf') or {}
            reference_continuous = (state_available and external.get('healthy') is not False and
                (before_ev.get('required') is not True or external.get('healthy') is True)
                and ekf.get('primary_instance') == before_ekf.get('primary_instance')
                and ekf.get('local_velocity_reset_counters') == before_ekf.get('local_velocity_reset_counters'))
            pose = state.get('body_pose_enu') or {}
            centre = landing['proof']['centre_enu_m']
            position = state.get('position_enu_m')
            verified = (reference_continuous and final_epoch == semantic.coordinate_epoch and position is not None
                and state.get('ready') is True and 0 <= pose.get('age_seconds', 100.) <= 1.
                and math.dist(position[:2], centre[:2]) <= .35)
            if landing.get('visual_review'):
                # The fresh visual conclusion and successful owned descent
                # establish this semantic task. A reset after touchdown is a
                # navigation diagnostic, not evidence that landing failed.
                verified = landing['visual_review']['status'] == 'verified'
            elif not reference_continuous or final_epoch != semantic.coordinate_epoch:
                semantic.invalidate('coordinate reset during target landing; final relative location unverified')
            record.update(status='grounded_verified' if verified else 'grounded_relation_unverified')
            return {'actual_ground_disarmed': True, 'relation_verified': bool(verified),
                    'mission_id': record['mission_id'], 'position_enu_m': position,
                    'coordinate_epoch': final_epoch, 'landing_proof': landing['proof'],
                    'post_landing_navigation_available': state_available,
                    'navigation_reference_continuous': bool(reference_continuous),
                    'relation_verification_basis': ('model_fresh_review_and_completed_owned_descent'
                        if landing.get('visual_review') else 'metric_pose_continuity'),
                    'actual_mission': copy.deepcopy(result['mission']), 'actual_state': copy.deepcopy(result)}
        except Exception:
            record['status'] = 'failed_or_unknown_effect'
            raise
