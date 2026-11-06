"""Bounded body-relative turns and image-grounded facing; no generated world waypoint.

Target references bind stage ownership/history. A body turn does not use an old
target coordinate as a navigation anchor or claim it remains visible afterward.
"""
import copy
import math
from dataclasses import asdict

from photo_navigation import relative_photo_yaw
from stereo_localization import iris_left_optical_to_body, matvec
from target_confirmation import observation_for
from viewpoint_planner import map_revision
from .contracts import GoalSpec, GoalPoseCandidate, ObservationNeed
from .memory import SharedTargetMemory


def heading_goal(semantic_map, reference, pose, origin_xy, radius_m, kind, direction=None, angle_deg=None):
    target = SharedTargetMemory(semantic_map).view(reference['target_id'], reference['confirmation_level'])
    if (pose.get('frame_id') not in ('map', 'local_enu') or pose.get('body_reference') != 'base_link'
            or not 0 <= pose.get('age_seconds', 100.) <= 1.):
        raise ValueError('Relative heading goal requires a fresh measured base_link ENU pose')
    point = list(pose['position_m'])
    if math.dist(point[:2], origin_xy) > radius_m or not 1. <= point[2] <= 20.:
        return {'status': 'no_safe_goal', 'candidates': []}
    if kind == 'relative_turn':
        yaw, _ = relative_photo_yaw(pose['orientation_xyzw'], direction, angle_deg)
        relation = 'body_'+direction
        basis = 'actual_body_heading_plus_bounded_user_relative_turn'
    elif kind == 'face_target':
        _, observed = observation_for(semantic_map, reference['target_id'], reference['capture_id'])
        recent = (reference['capture_id'] == semantic_map.observations[-1]['capture_id']
                  and math.dist(point, observed['body_pose_enu']['position_m']) <= .5)
        recalled = (observed.get('position_enu_m') is not None
                    and semantic_map.observations[-1]['stamp_sim_s']-observed['stamp_sim_s'] <= 120.)
        if not recent and not recalled:
            need = ObservationNeed('RECOVER_OCCLUDED', reference['target_id'], reference['capture_id'],
                                   ['fresh_target_bearing_at_current_pose'], ['actual_new_owned_image'])
            return {'status': 'needs_observation', 'need': asdict(need), 'candidates': []}
        ray = (observed['bearing_direction_enu'] if recent else
               [a-b for a, b in zip(observed['position_enu_m'], point)])
        length = math.sqrt(sum(v*v for v in ray))
        if length < .2:
            return {'status': 'no_safe_goal', 'candidates': []}
        ray = [v/length for v in ray]
        if math.hypot(*ray[:2]) < .2:
            return {'status': 'no_safe_goal', 'candidates': []}
        forward = matvec(iris_left_optical_to_body()['rotation'], [0., 0., 1.])
        yaw = math.atan2(ray[1], ray[0])-math.atan2(forward[1], forward[0])
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        relation, basis = 'facing_target', ('calibrated_actual_image_bearing_without_invented_target_range' if recent else
                                          'remembered_metric_surface_for_heading_recovery_not_navigation')
    else:
        raise ValueError('Unknown heading-only route skill')
    revision = map_revision(semantic_map)
    spec = GoalSpec(reference['target_id'], relation, kind, facing='body_relative_heading' if kind == 'relative_turn' else 'target',
                    reference_frame='body' if kind == 'relative_turn' else 'actual_camera_bearing')
    source = semantic_map.observations[-1]
    candidate = GoalPoseCandidate('goal_001', point, yaw, asdict(spec),
        [{'capture_id': source['capture_id'], 'image_sha256': copy.deepcopy(source['image_sha256']), 'basis': basis}],
        target['target_revision'], revision, copy.deepcopy(semantic_map.coordinate_epoch),
        {'identity': True, 'permission_volume': True, 'static_rotation_clearance': True, 'path': 'unknown'},
        {'relative_heading': 1., 'target_visibility': None, 'world_position_margin_m': None})
    return {'status': 'candidates', 'candidates': [asdict(candidate)], 'target_revision': target['target_revision'],
            'map_revision': revision, 'target_anchor_enu_m': None, 'requires_target_visibility': False,
            'target_metric_position_required': False, 'basis': basis}
