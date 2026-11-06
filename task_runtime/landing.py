"""Fresh target-front review and narrow static flat-ground landing adapter.

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
from .object_frame import measured_part, FRONT_FACE_BASIS, estimate_front_face_frame, current_front_relation


def verify_front_review(landmark, observation, frame, box, metrics, description, side_boxes=None):
    orientation = landmark.attributes.get('object_orientation')
    if not orientation or orientation['coordinate_epoch'] != observation['coordinate_epoch']:
        return None
    if not isinstance(description, str) or not 0 < len(description.strip()) <= 1000:
        raise ValueError('Fresh front verification requires a concrete visual description')
    if box is None or not metrics or metrics[0].get('status') != 'ok':
        return None
    outer = observation['bbox']
    if not (outer[0] <= box[0] < box[2] <= outer[2] and outer[1] <= box[1] < box[3] <= outer[3]):
        raise ValueError('Fresh front face must be inside the actually reported same-target box')
    if orientation.get('basis') == FRONT_FACE_BASIS:
        if (frame['capture_id'] == orientation['source_capture_id']
                or observation['stamp_sim_s'] <= orientation['part_measurements'][0]['image_stamp_sim_s']):
            return None
        if side_boxes is None or any(b is None for b in side_boxes):
            return None
        fresh = estimate_front_face_frame(frame, observation, [box, *side_boxes], metrics, description)['orientation']
        if fresh is None:
            return None
        relation = current_front_relation(fresh, observation, frame['snapshot']['body_pose_enu'])
        difference = abs(math.atan2(math.sin(fresh['yaw_enu_rad']-orientation['yaw_enu_rad']),
                                    math.cos(fresh['yaw_enu_rad']-orientation['yaw_enu_rad'])))
        if relation is None or difference > min(math.pi/2, 2*math.sqrt(
                fresh['yaw_variance_rad2']+orientation['yaw_variance_rad2'])):
            return None
        bound = 2*math.sqrt(sum(orientation['front_covariance_enu_m2'][i][i]+
                                fresh['front_covariance_enu_m2'][i][i] for i in range(3)))
        if math.dist(orientation['front_surface_enu_m'], fresh['front_surface_enu_m']) > max(.5, bound):
            return None
        return {'schema_version': 1, 'status': 'verified', 'capture_id': frame['capture_id'],
            'image_sha256': copy.deepcopy(observation['image_sha256']),
            'coordinate_epoch': copy.deepcopy(observation['coordinate_epoch']),
            'front_box': list(box), 'front_measurement': copy.deepcopy(metrics[0]),
            'front_face_evidence': fresh, 'minimum_front_distance_m': relation['minimum_front_distance_m'],
            'visual_description': description, 'basis': 'fresh_same_frame_front_face_stereo_relation',
            'semantic_front_independently_verified': False}
    current = measured_part(frame, observation, box, metrics[0])
    prior = orientation['front_surface_enu_m']
    covariance = [[orientation['front_covariance_enu_m2'][i][j]+current['covariance_enu_m2'][i][j]
                   for j in range(3)] for i in range(3)]
    bound = 2*math.sqrt(max(sum(abs(v) for v in row) for row in covariance))
    if math.dist(prior, current['position_enu_m']) > max(.5, bound):
        return None
    position = frame['snapshot']['body_pose_enu']['position_m']
    yaw = orientation['yaw_enu_rad']
    delta = [position[i]-prior[i] for i in (0, 1)]
    along = delta[0]*math.cos(yaw)+delta[1]*math.sin(yaw)
    across = abs(-delta[0]*math.sin(yaw)+delta[1]*math.cos(yaw))
    yaw_margin = 2*math.sqrt(orientation['yaw_variance_rad2'])
    anchor_margin = 2*math.sqrt(max(sum(abs(v) for v in row) for row in orientation['front_covariance_enu_m2']))
    minimum_front = along*math.cos(yaw_margin)-across*math.sin(yaw_margin)-anchor_margin
    if minimum_front < 2.:
        return None
    return {'schema_version': 1, 'status': 'verified', 'capture_id': frame['capture_id'],
            'image_sha256': copy.deepcopy(observation['image_sha256']),
            'coordinate_epoch': copy.deepcopy(observation['coordinate_epoch']),
            'front_box': list(box), 'front_measurement': copy.deepcopy(metrics[0]),
            'minimum_front_distance_m': minimum_front, 'visual_description': description,
            'basis': 'fresh_front_visual_roi_metric_agreement_and_conservative_object_front_relation',
            'semantic_front_independently_verified': False}


class LandingSkill:
    def __init__(self, search):
        self.search, self.client = search, search.client

    def prepare(self, owner, stage, plan_revision, goal, reference):
        child = self.search.sessions[owner]
        semantic = self.search.semantic_maps.maps[owner]
        self.search.validate_state(owner, self.client.state(), allow_skill_handoff=True)
        review = goal.get('relation_review') or {}
        if (stage.kind != 'relative_land' or stage.relation != 'object_front'
                or review.get('status') != 'verified' or review.get('capture_id') != reference['capture_id']
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
        goal_context = {'schema_version': 1, 'stage_id': stage.stage_id,
            'stage_revision': child['authority_stage_revision'], 'plan_revision': plan_revision,
            'target_id': reference['target_id'], 'target_revision': version, 'relation': stage.relation,
            'terminal_action': stage.kind}
        _, observation = observation_for(semantic, reference['target_id'], reference['capture_id'])
        self.client.request('/navigation/goal-poses', {'operation': 'context', 'pair_id': child['last_stereo_pair_id'],
            'origin_xy_enu_m': child['boundary_origin'][:2], 'expected_identity': map_identity(semantic),
            'search_context': context, 'goal_context': goal_context, 'candidates': [],
            'probe_points_enu_m': [observation['position_enu_m']]})
        result = self.client.request('/navigation/authorize-goal-land', {'owner': list(owner),
            'expected_mission_id': execution['mission_id'], 'search_context': context, 'goal_context': goal_context,
            'verification_pair_id': child['last_stereo_pair_id']})
        return {**result, 'context': context, 'goal_context': goal_context,
                'expected_mission_id': execution['mission_id'], 'execution': None}

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
            state = self.client.state()
            external = state.get('external_vision') or {}
            ekf = external.get('ekf') or {}
            reference_continuous = (external.get('healthy') is not False and
                (before_ev.get('required') is not True or external.get('healthy') is True)
                and ekf.get('primary_instance') == before_ekf.get('primary_instance')
                and ekf.get('local_velocity_reset_counters') == before_ekf.get('local_velocity_reset_counters'))
            pose = state.get('body_pose_enu') or {}
            centre = landing['proof']['centre_enu_m']
            position = state.get('position_enu_m')
            verified = (reference_continuous and coordinate_epoch(state) == semantic.coordinate_epoch and position is not None
                and state.get('ready') is True and 0 <= pose.get('age_seconds', 100.) <= 1.
                and math.dist(position[:2], centre[:2]) <= .35)
            if not reference_continuous or coordinate_epoch(state) != semantic.coordinate_epoch:
                semantic.invalidate('coordinate reset during target landing; final relative location unverified')
            record.update(status='grounded_verified' if verified else 'grounded_relation_unverified')
            return {'actual_ground_disarmed': True, 'relation_verified': bool(verified),
                    'mission_id': record['mission_id'], 'position_enu_m': position,
                    'coordinate_epoch': coordinate_epoch(state), 'landing_proof': landing['proof'],
                    'navigation_reference_continuous': bool(reference_continuous),
                    'actual_mission': copy.deepcopy(result['mission']), 'actual_state': copy.deepcopy(result)}
        except Exception:
            record['status'] = 'failed_or_unknown_effect'
            raise
