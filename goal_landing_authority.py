"""Executor-owned one-use landing permits for verified static object-front hover.

Called under Controller.submit_lock + Controller.lock. No model/network work.
Operator/safety land APIs remain independent and do not need these permits.
"""
import copy
import math
import re
import time
import uuid

from flat_landing import flat_support_proof, LandingUnavailable
from goal_pose_geometry import validate_goal_context
from mission import MissionError
from search_tools import coordinate_epoch
from viewpoint_authority import owner_key
from viewpoint_geometry import registered_identity


class GoalLandingAuthority:
    def __init__(self, clock=time.monotonic):
        self.clock, self.tickets = clock, {}

    def _guard(self, c, owner, expected_mission_id, search_context, goal_context):
        c.ready()
        reason = c.ev_guard.reason(self.clock())
        if reason:
            raise MissionError(reason)
        if (not c.static_navigation_enabled() or not c.navigation_active or c.navigation_map is None
                or c.navigation_moving or c.navigation_gate_error or c.state_msg.armed is not True or c.landed != 2):
            raise MissionError('goal landing requires healthy stable airborne static navigation')
        mission = c.mission or {}
        scope = c.search_authority().require_batch(search_context)
        validate_goal_context(goal_context, scope)
        identity = registered_identity(c.navigation_map, coordinate_epoch(c.ev_guard.snapshot(self.clock())))
        prior = mission.get('goal_context') or {}
        if (owner_key(owner) != tuple(scope['owner']) or mission.get('id') != expected_mission_id
                or mission.get('status') != 'succeeded' or identity != scope['identity']
                or goal_context.get('terminal_action') != 'relative_land'
                or prior.get('terminal_action') != 'relative_land'
                or goal_context.get('relation') != 'object_front'
                or any(prior.get(k) != goal_context[k] for k in ('stage_id', 'plan_revision', 'target_id', 'relation'))
                or goal_context['target_revision'] < prior.get('target_revision', 0)):
            raise MissionError('landing owner, completed goal hover, target or coordinate identity changed')
        point = c.position()
        expected = mission.get('goal_pose_enu_m')
        desired_yaw = mission.get('goal_yaw_enu_rad')
        if expected is None or desired_yaw is None or math.dist(point, expected) > .35:
            raise MissionError('landing lacks measured arrival at its executor-cached goal')
        if c.pose is None or not 0 <= self.clock()-c.pose_time <= 1.:
            raise MissionError('landing has no fresh base_link pose')
        q = c.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if abs(math.atan2(math.sin(yaw-desired_yaw), math.cos(yaw-desired_yaw))) > math.radians(5.):
            raise MissionError('landing hover no longer faces the verified goal')
        if math.dist(point[:2], scope['origin_xy_enu_m']) > scope['search_radius_m']-.35:
            raise MissionError('landing tracking corridor would exceed the immutable search radius')
        return identity, point

    def authorize(self, c, owner, expected_mission_id, search_context, goal_context, verification_pair_id):
        identity, point = self._guard(c, owner, expected_mission_id, search_context, goal_context)
        metadata = c.stereo_reports.observation_metadata(verification_pair_id, max_age_wall_s=30.)
        rotations = [e['rotation']['stamp_s'] for e in c.mission.get('events', [])
                     if e.get('action') == 'rotate' and isinstance(e.get('rotation'), dict)]
        if (not rotations or metadata['stamp_sim_s'] <= max(rotations)
                or metadata['coordinate_epoch'] != identity['coordinate_epoch']
                or metadata['navigation_registration_id'] != identity['sortie_id']
                or math.dist(metadata['body_pose_enu']['position_m'], point) > .35):
            raise MissionError('landing requires a real same-epoch stereo pair after actual goal rotation')
        try:
            proof = flat_support_proof(c.world_path, c.scene, c.navigation_map, point)
        except LandingUnavailable as exc:
            return {'status': 'not_executed', 'reason_code': 'FLAT_SUPPORT_UNAVAILABLE', 'reason': str(exc),
                    'mission_submitted': False, 'expected_mission_id': expected_mission_id}
        self.tickets = {k: v for k, v in self.tickets.items() if self.clock()-v['created'] <= 30.}
        if len(self.tickets) >= 16:
            raise MissionError('landing permit capacity reached')
        ticket = uuid.uuid4().hex
        self.tickets[ticket] = {'owner': list(owner), 'identity': identity, 'created': self.clock(),
            'expected_mission_id': expected_mission_id, 'search_context': copy.deepcopy(search_context),
            'goal_context': copy.deepcopy(goal_context), 'proof': proof, 'verification_pair_id': verification_pair_id,
            'verification_image_sha256': metadata['image_sha256'], 'verification_stamp_sim_s': metadata['stamp_sim_s']}
        return {'status': 'authorized', 'ticket': ticket, 'proof': copy.deepcopy(proof), 'expires_in_wall_s': 30.,
                'verification_stamp_sim_s': metadata['stamp_sim_s'], 'mission_submitted': False}

    def execute(self, c, owner, ticket, goal_context, search_context, expected_mission_id):
        if not isinstance(ticket, str) or not re.fullmatch(r'[0-9a-f]{32}', ticket):
            raise MissionError('invalid goal-landing ticket')
        value = self.tickets.get(ticket)
        if (value is None or self.clock()-value['created'] > 30. or owner_key(owner) != tuple(value['owner'])
                or goal_context != value['goal_context'] or search_context != value['search_context']
                or expected_mission_id != value['expected_mission_id']):
            raise MissionError('goal landing permit expired, consumed or changed stage/target/owner')
        identity, point = self._guard(c, owner, expected_mission_id, search_context, goal_context)
        if identity != value['identity'] or math.dist(point, value['proof']['hover_enu_m']) > .35:
            raise MissionError('landing coordinate identity or verified hover position changed')
        # Independently recheck the support column and current synchronized scene
        # supervision immediately before normal land submission.
        flat_support_proof(c.world_path, c.scene, c.navigation_map, point)
        c.validate_scene_steps([{'action': 'land'}])
        self.tickets.pop(ticket)
        return c.submit([{'action': 'land'}], 'Program: verified object-front flat landing',
            parent_mission_id=expected_mission_id, landing_authorization=value)
