"""Finite task-relative terminal poses, separate from information-gathering NBV.

All scores/thresholds are project engineering choices. A soft score never
overrides identity, relation, visibility, permission or path constraints.
"""
import copy
import math
from dataclasses import asdict

from mission import MAX_ALTITUDE
from stereo_localization import covariance_matrix, iris_left_optical_to_body, matvec
from target_observation import relative_range_reference
from target_confirmation import observation_for
from viewpoint_planner import map_revision
from .contracts import GoalPoseCandidate, ObservationNeed
from .object_frame import validate_navigation_measurement, FRONT_RELATION_BASES, current_front_relation


def wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def camera_facing_yaw(position, target):
    optical_forward = matvec(iris_left_optical_to_body()['rotation'], [0., 0., 1.])
    mount_heading = math.atan2(optical_forward[1], optical_forward[0])
    return wrapped(math.atan2(target[1]-position[1], target[0]-position[0])-mount_heading)


class GoalPosePlanner:
    def framing_candidate(self, pose, operation, arguments, origin_xy, radius_m):
        """Photography profile: calibrated local motion with unknown whole-subject gain.

        The existing photo-action executor remains the path authority. This
        profile shares terminal pose construction/arrival validation; it never
        treats rising or retaining an old box's rays as full subject coverage.
        """
        from photo_navigation import relative_photo_goal, relative_photo_yaw
        from stereo_localization import quaternion_rotation
        if (pose.get('frame_id') not in ('map', 'local_enu') or pose.get('body_reference') != 'base_link'
                or not 0 <= pose.get('age_seconds', 100.) <= 1.):
            raise ValueError('Framing goal requires fresh measured base_link ENU pose')
        point = list(pose['position_m'])
        rotation = quaternion_rotation(pose['orientation_xyzw'])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
        if operation == 'adjust':
            point, _ = relative_photo_goal(point, pose['orientation_xyzw'], arguments['direction'],
                                           arguments['distance_m'], arguments['altitude'])
        elif operation == 'turn':
            yaw, _ = relative_photo_yaw(pose['orientation_xyzw'], arguments['direction'], arguments['angle_deg'])
        else:
            raise ValueError('Unknown framing goal profile')
        if math.dist(point[:2], origin_xy) > radius_m:
            raise ValueError('Framing candidate exceeds the original parent search boundary')
        return {'position_enu_m': point, 'yaw_enu_rad': yaw, 'profile': 'local_framing',
                'predicted_whole_subject_visibility': None, 'predicted_new_coverage_m2': None,
                'basis': 'actual_calibrated_camera_relative_motion_not_extent_prediction',
                'hard_constraints': {'fresh_pose': True, 'permission_volume': True, 'path': 'unknown'}}

    def generate(self, spec, semantic_map, reference, pose, origin_xy, radius_m):
        from .memory import SharedTargetMemory
        state = SharedTargetMemory(semantic_map).view(spec.target_id, reference['confirmation_level'])
        _, observation = observation_for(semantic_map, spec.target_id, reference['capture_id'])
        def need(kind, missing):
            return {'status': 'needs_observation', 'need': asdict(ObservationNeed(kind, spec.target_id,
                reference['capture_id'], missing, ['fresh_owned_visual_and_metric_evidence'])), 'candidates': []}
        if state['visibility'] != 'recently_visible' or observation['capture_id'] != semantic_map.observations[-1]['capture_id']:
            return need('RECOVER_OCCLUDED', ['fresh_same_instance_observation'])
        if reference['confirmation_level'] != 'multiview':
            return need('RESOLVE_INSTANCE', ['strong_multiview_identity'])
        try:
            validate_navigation_measurement(observation)
        except (KeyError, TypeError, ValueError):
            return need('OBTAIN_DEPTH', ['valid_metric_surface_and_covariance'])
        orientation = state['orientation']
        if orientation and orientation.get('basis') in FRONT_RELATION_BASES:
            if spec.relation != 'object_front':
                return need('REVEAL_FRONT', ['rear/extent evidence; front face does not localize the rear'])
            if orientation['coordinate_epoch'] != semantic_map.coordinate_epoch:
                raise ValueError('Front-face reference belongs to another coordinate epoch')
            relation = current_front_relation(orientation, observation, pose)
            if relation is not None:
                point = list(pose['position_m'])
                if math.dist(point[:2], origin_xy) > radius_m or not 1. <= point[2] <= MAX_ALTITUDE:
                    return {'status': 'no_safe_goal', 'candidates': []}
                anchor = orientation['front_surface_enu_m'];revision = map_revision(semantic_map)
                matrix = covariance_matrix(orientation['front_covariance_enu_m2'], 'actual front world covariance')
                candidate = GoalPoseCandidate('goal_001', point, camera_facing_yaw(point, anchor), asdict(spec),
                    [{'capture_id': observation['capture_id'], 'image_sha256': observation['image_sha256'],
                      'front_relation': relation}], state['target_revision'], revision,
                    copy.deepcopy(semantic_map.coordinate_epoch),
                    {'identity': True, 'object_relation': True, 'permission_volume': True,
                     'path': 'unknown', 'target_visibility': 'unknown'},
                    {'relation': 1., 'world_position_margin_m': 2*math.sqrt(max(sum(abs(v) for v in row) for row in matrix)),
                     'object_yaw_margin_rad': 2*math.sqrt(orientation['yaw_variance_rad2']),
                     'minimum_front_distance_m': relation['minimum_front_distance_m']})
                return {'status': 'candidates', 'candidates': [asdict(candidate)],
                    'target_anchor_enu_m': list(anchor), 'target_revision': state['target_revision'],
                    'target_anchor_covariance_enu_m2': copy.deepcopy(matrix),
                    'map_revision': revision, 'scoring': 'observed_current_front_region_v1',
                    'requires_target_visibility': True}
            if not orientation['global_axis_usable']:
                return need('IMPROVE_FRONT_RELATION', ['front_angular_uncertainty', 'front_relation_standoff_or_uncertainty'])
        if spec.relation in ('object_front', 'object_behind'):
            if not orientation or orientation['coordinate_epoch'] != semantic_map.coordinate_epoch:
                return need('REVEAL_FRONT', [
                    'measured_visible_front_face_normal_or_front_rear_axis' if spec.relation == 'object_front'
                    else 'visually_grounded_front_and_rear_metric_points'])
            anchor = orientation['front_surface_enu_m'] if spec.relation == 'object_front' else orientation['rear_surface_enu_m']
            direction = orientation['yaw_enu_rad']+(0. if spec.relation == 'object_front' else math.pi)
            angle_margin = 2*math.sqrt(orientation['yaw_variance_rad2'])
            matrix = covariance_matrix(orientation['front_covariance_enu_m2'] if spec.relation == 'object_front'
                                       else orientation['rear_covariance_enu_m2'], 'object anchor world covariance')
        else:
            anchor = observation['position_enu_m']
            direction = (math.atan2(anchor[1]-pose['position_m'][1], anchor[0]-pose['position_m'][0])
                         if spec.relation == 'route_forward' else
                         math.atan2(pose['position_m'][1]-anchor[1], pose['position_m'][0]-anchor[0]))
            angle_margin = 0.
            matrix = covariance_matrix(observation['covariance_enu_m2'], 'target world covariance')
        if any(matrix[i][i] <= 0 for i in range(3)):
            return need('OBTAIN_DEPTH', ['positive_actual_position_covariance'])
        margin = 2*math.sqrt(max(sum(abs(v) for v in row) for row in matrix))
        revision = map_revision(semantic_map)
        candidates = []
        # Position uncertainty is kept, and expands standoff rather than being
        # silently reduced to obtain a visually appealing close waypoint.
        for distance in (max(4., margin+3.), max(6., margin+5.), max(8., margin+7.)):
            for side in (0., -1., 1.):
                x = anchor[0]+distance*math.cos(direction)-side*math.sin(direction)
                y = anchor[1]+distance*math.sin(direction)+side*math.cos(direction)
                z = max(2., min(5., pose['position_m'][2]))
                point = [x, y, z]
                relation_valid = distance*math.cos(angle_margin)-abs(side)*math.sin(angle_margin)-margin >= 2.
                volume = math.dist(point[:2], origin_xy) <= radius_m and 1. <= z <= MAX_ALTITUDE
                if not relation_valid or not volume:
                    continue
                route = spec.terminal_action == 'pass_landmark'
                candidate = GoalPoseCandidate('goal_%03d' % (len(candidates)+1), point,
                    wrapped(direction) if route else camera_facing_yaw(point, anchor), asdict(spec),
                    [{'capture_id': reference['capture_id'], 'image_sha256': observation['image_sha256'],
                      'orientation_capture_id': orientation['source_capture_id'] if orientation else None}],
                    state['target_revision'], revision, copy.deepcopy(semantic_map.coordinate_epoch),
                    {'identity': True, 'object_relation': True, 'permission_volume': True,
                     'path': 'unknown', **({} if route else {'target_visibility': 'unknown'})},
                    {'relation': 1., 'world_position_margin_m': margin, 'object_yaw_margin_rad': angle_margin})
                candidates.append(asdict(candidate))
        return {'status': 'candidates' if candidates else 'no_safe_goal', 'candidates': candidates,
                'target_anchor_enu_m': list(anchor), 'target_revision': state['target_revision'],
                **({'target_anchor_covariance_enu_m2': copy.deepcopy(matrix)}
                   if orientation and orientation.get('basis') in FRONT_RELATION_BASES else {}),
                'map_revision': revision, 'scoring': 'project_fixed_scale_goal_v1',
                'requires_target_visibility': spec.terminal_action != 'pass_landmark',
                **({'route_axis_xy_enu': [math.cos(direction), math.sin(direction)],
                    'route_reference': 'observed_surface_plane_not_object_center_or_full_extent'}
                   if spec.terminal_action == 'pass_landmark' else {})}

    def evaluate(self, generated, evaluator):
        """evaluator performs the existing registered geometry and ticket checks."""
        if generated['status'] != 'candidates':
            return generated
        safe = []
        for candidate in generated['candidates']:
            result = evaluator(candidate, generated['target_anchor_enu_m'])
            view_required = generated.get('requires_target_visibility', True)
            if result.get('reachable') is not True or view_required and result.get('target_visible') is not True:
                continue
            value = copy.deepcopy(candidate)
            value['hard_constraints'].update(path=True)
            if view_required:
                value['hard_constraints']['target_visibility'] = True
            value['path'] = result['path']
            value.update({k: result[k] for k in ('execution_ticket', 'batch_id') if k in result})
            length = result['path']['path_length_m']
            margin = value['score_terms'].get('world_position_margin_m')
            value['score_terms'].update(view_alignment=1., target_visibility=1. if view_required else None, path_cost=-.25*length/10.,
                                        uncertainty_cost=-.2*margin/4. if type(margin) in (int, float) else None)
            value['score'] = sum(value['score_terms'][k] for k in
                ('relation', 'relative_heading', 'view_alignment', 'target_visibility', 'path_cost', 'uncertainty_cost')
                if type(value['score_terms'].get(k)) in (int, float))
            safe.append(value)
        safe.sort(key=lambda v: (-v['score'], v['id']))
        return {**generated, 'status': 'ready' if safe else 'no_safe_goal', 'candidates': safe,
                'selected_id': safe[0]['id'] if safe else None}

    @staticmethod
    def verify_arrival(candidate, position, yaw, epoch):
        return (epoch == candidate['coordinate_epoch']
                and math.dist(position, candidate['position_enu_m']) <= .35
                and abs(wrapped(yaw-candidate['yaw_enu_rad'])) <= math.radians(5.))
