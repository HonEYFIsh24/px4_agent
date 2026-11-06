"""Object front/rear from visual part ROIs with actual synchronized stereo.

The VLM supplies semantic regions, never world coordinates. The axis is computed
from measured front/rear surfaces; this does not estimate a vehicle centre or
complete volume. Shared pose translation cancels from the axis difference, but
world angular uncertainty remains explicitly in its yaw variance.
"""
import copy
import math

from semantic_map import bbox
from stereo_localization import (covariance_matrix, iris_left_optical_to_body,
                                matmul, matvec, optical_to_enu, quaternion_rotation, transpose)
from target_observation import relative_range_reference


FRONT_FACE_BASIS = 'visible_front_face_lateral_stereo_v1'


def estimate_front_face_frame(frame, observation, boxes, metrics, description):
    """Infer the outward front normal from two *lateral front-face* regions.

    A frontal view need not contain the hidden rear of a vehicle. This profile
    requires three measured ROIs: front face and image-left/right portions of
    that same face. It never relabels those portions as front/rear endpoints.
    Global pose uncertainty is retained; same-frame relation uses local stereo
    covariance, with shared coordinate translation/yaw cancelling explicitly.
    """
    def need(reason):
        return {'status': 'needs_observation', 'reason_code': reason, 'orientation': None}
    if frame['capture_id'] != observation['capture_id'] or frame.get('delivered') is not True:
        raise ValueError('Front-face frame requires the actually delivered same-target image')
    if not isinstance(description, str) or not 1 <= len(description.strip()) <= 1000:
        raise ValueError('Front-face regions require actual visible structure descriptions')
    if len(boxes) != 3 or any(b is None for b in boxes):
        return need('MISSING_FRONT_FACE_LATERAL_REGIONS')
    boxes = [bbox(b) for b in boxes]
    outer, face, left, right = observation['bbox'], *boxes
    for b in boxes:
        if not (outer[0] <= b[0] < b[2] <= outer[2] and outer[1] <= b[1] < b[3] <= outer[3]):
            raise ValueError('Front-face ROI is outside the current same-target box')
    if any(min(b[0], b[1], 1-b[2], 1-b[3]) <= .005 for b in boxes):
        return need('FRONT_FACE_REGIONS_CLIPPED')
    if any(not (face[0] <= b[0] < b[2] <= face[2] and face[1] <= b[1] < b[3] <= face[3])
           for b in (left, right)):
        raise ValueError('Lateral ROIs must belong to the visible front face')
    overlap_y = min(left[3], right[3])-max(left[1], right[1])
    if left[2] > right[0] or overlap_y < .5*min(left[3]-left[1], right[3]-right[1]):
        return need('FRONT_FACE_LATERAL_REGIONS_NOT_SEPARATED_AT_SAME_HEIGHT')
    if len(metrics) != 3:
        raise ValueError('Front-face stereo response has the wrong region count')
    if any(m.get('status') != 'ok' for m in metrics):
        return need('FRONT_FACE_LATERAL_DEPTH_UNAVAILABLE')
    front, a, b = [measured_part(frame, observation, box, m) for box, m in zip(boxes, metrics)]
    pose = frame['snapshot']['body_pose_enu']
    angular = covariance_matrix(pose['orientation_covariance_rad2'], 'actual pose angular covariance')
    if any(angular[i][i] <= 0 for i in range(3)):
        return need('FRONT_FACE_POSE_UNCERTAINTY_UNAVAILABLE')
    delta = [a['position_enu_m'][i]-b['position_enu_m'][i] for i in range(3)]
    width = math.hypot(*delta[:2])
    if not 1. <= width <= 6. or abs(delta[2]) > min(.5, width*.25):
        return need('FRONT_FACE_METRIC_LATERAL_BASELINE_INVALID')
    # Both measured side points must also agree in depth with the face ROI;
    # a background edge cannot silently become a vehicle front normal.
    depths = [m['position_camera_optical_m'][2] for m in metrics]
    for i in (1, 2):
        bound = 2*(math.sqrt(metrics[0]['covariance_camera_optical_m2'][2][2])+
                   math.sqrt(metrics[i]['covariance_camera_optical_m2'][2][2]))
        if abs(depths[i]-depths[0]) > .5+bound:
            return need('FRONT_FACE_REGION_DEPTHS_DISAGREE')
    normal = [-delta[1]/width, delta[0]/width]
    observer = [pose['position_m'][i]-front['position_enu_m'][i] for i in (0, 1)]
    if sum(normal[i]*observer[i] for i in (0, 1)) < 0:
        normal = [-v for v in normal]
    along = sum(normal[i]*observer[i] for i in (0, 1))
    across = abs(-normal[1]*observer[0]+normal[0]*observer[1])
    if along <= 0 or across > along:
        return need('FRONT_FACE_NOT_VIEWED_FROM_ITS_FRONTAL_REGION')
    rotation = matmul(quaternion_rotation(pose['orientation_xyzw']), iris_left_optical_to_body()['rotation'])
    # Unknown patch correlation is bounded with Var(A-B) <= 2*(Var(A)+Var(B)).
    # No sqrt(number_of_pixels) reduction or artificially small covariance.
    cov_optical = [[2*sum(m['covariance_camera_optical_m2'][i][j] for m in metrics[1:])
                    for j in range(3)] for i in range(3)]
    cov_width = matmul(matmul(rotation, cov_optical), transpose(rotation))
    perpendicular = [-delta[1]/width, delta[0]/width, 0.]
    local_variance = sum(perpendicular[i]*cov_width[i][j]*perpendicular[j]
                         for i in range(3) for j in range(3))/(width*width)
    if local_variance <= 0:
        return need('FRONT_FACE_NORMAL_UNCERTAINTY_UNAVAILABLE')
    # Common world yaw/translation cancel only for this same-frame relation.
    # Tilt uncertainty remains. World yaw variance stays in the global axis.
    angle_margin = 2*math.sqrt(local_variance+angular[0][0]+angular[1][1])
    position_margin = 2*math.sqrt(sum(metrics[0]['covariance_camera_optical_m2'][i][i] for i in range(3)))
    minimum = (along*math.cos(angle_margin)-across*math.sin(angle_margin)-position_margin-.70
               if angle_margin < math.pi/3 else -1.)
    orientation = {'schema_version': 1, 'basis': FRONT_FACE_BASIS,
        'source_capture_id': frame['capture_id'], 'image_sha256': copy.deepcopy(observation['image_sha256']),
        'coordinate_epoch': copy.deepcopy(observation['coordinate_epoch']),
        'front_surface_enu_m': front['position_enu_m'], 'front_covariance_enu_m2': front['covariance_enu_m2'],
        'rear_surface_enu_m': None, 'rear_covariance_enu_m2': None,
        'yaw_enu_rad': math.atan2(normal[1], normal[0]),
        'yaw_variance_rad2': local_variance+angular[2][2],
        'global_axis_usable': local_variance+angular[2][2] <= math.radians(20.)**2,
        'relative_axis_variance_rad2': local_variance, 'axis_length_m': width,
        'part_boxes': boxes, 'part_measurements': copy.deepcopy(metrics), 'visual_descriptions': [description],
        'body_position_at_capture_enu_m': list(pose['position_m']),
        'current_front_relation': {'minimum_front_distance_m': minimum, 'minimum_required_m': 2.,
            'normal_uncertainty_margin_rad': angle_margin, 'position_uncertainty_margin_m': position_margin,
            'pose_and_descent_tracking_reserve_m': .70,
            'basis': 'same_frame_front_normal_and_stereo_surface_with_common_reference_cancelled'},
        'complete_extent_known': False, 'semantic_front_independently_verified': False,
        'uncertainty_statistics_calibrated': False}
    return {'status': 'available', 'reason_code': 'VISIBLE_FRONT_FACE_STEREO_NORMAL', 'orientation': orientation}


def current_front_relation(orientation, observation, pose):
    """Recompute recorded measurements; only a nearby current pose may use it."""
    if orientation.get('basis') != FRONT_FACE_BASIS or orientation['source_capture_id'] != observation['capture_id']:
        return None
    frame = {'capture_id': observation['capture_id'], 'delivered': True,
             'snapshot': {'body_pose_enu': observation['body_pose_enu'],
                          'stereo_pair_id': observation['stereo_pair_id']}}
    computed = estimate_front_face_frame(frame, observation, orientation['part_boxes'],
                                        orientation['part_measurements'], orientation['visual_descriptions'][0])
    if computed['orientation'] != orientation:
        raise ValueError('Stored front-face normal differs from its actual stereo evidence')
    distance = math.dist(pose['position_m'], orientation['body_position_at_capture_enu_m'])
    value = orientation['current_front_relation']
    if (distance > .35 or not 0 <= pose.get('age_seconds', 100.) <= 1.
            or value['minimum_front_distance_m']-distance < value['minimum_required_m']):
        return None
    return {**copy.deepcopy(value), 'capture_to_pose_distance_m': distance,
            'minimum_front_distance_m': value['minimum_front_distance_m']-distance}


def validate_navigation_measurement(observation):
    relative_range_reference(observation)
    metric = observation['relative_stereo']['measurement']
    _, expected, _ = optical_to_enu(metric['position_camera_optical_m'], metric['covariance_camera_optical_m2'],
                                   observation['body_pose_enu'])
    actual = covariance_matrix(metric['covariance_enu_m2'], 'world surface covariance')
    if any(not math.isclose(actual[i][j], expected[i][j], rel_tol=1e-7, abs_tol=1e-9)
           for i in range(3) for j in range(3)):
        raise ValueError('Navigation world covariance differs from the acquired stereo/pose model')
    return actual


def measured_part(frame, target_observation, box, measurement):
    o = copy.deepcopy(target_observation)
    o.update(bbox=bbox(box), position_enu_m=measurement.get('position_enu_m'),
             covariance_enu_m2=measurement.get('covariance_enu_m2'),
             relative_stereo={'capture_id': frame['capture_id'], 'pair_id': frame['snapshot']['stereo_pair_id'],
                              'bbox': list(box), 'measurement': copy.deepcopy(measurement)})
    validate_navigation_measurement(o)
    return o


def estimate_object_frame(frame, observation, front_box, rear_box, metrics, descriptions):
    if frame['capture_id'] != observation['capture_id'] or frame.get('delivered') is not True:
        raise ValueError('Object frame requires the actually delivered target image')
    if not all(isinstance(d, str) and 1 <= len(d.strip()) <= 600 for d in descriptions):
        raise ValueError('Front/rear require actual visual descriptions')
    outer = observation['bbox']
    boxes = [bbox(front_box), bbox(rear_box)]
    for b in boxes:
        if not (outer[0] <= b[0] < b[2] <= outer[2] and outer[1] <= b[1] < b[3] <= outer[3]):
            raise ValueError('Object part ROI must belong to the current target box')
        if min(b[0], b[1], 1-b[2], 1-b[3]) <= .005:
            raise ValueError('Clipped parts cannot establish an object front frame')
    overlap = max(0., min(boxes[0][2], boxes[1][2])-max(boxes[0][0], boxes[1][0]))*max(
        0., min(boxes[0][3], boxes[1][3])-max(boxes[0][1], boxes[1][1]))
    if overlap > .1*min((b[2]-b[0])*(b[3]-b[1]) for b in boxes):
        raise ValueError('Front/rear ROIs must be distinct observed parts')
    if len(metrics) != 2 or any(m.get('status') != 'ok' for m in metrics):
        return None  # Missing depth requests observation; never invent an axis.
    front, rear = [measured_part(frame, observation, b, m) for b, m in zip(boxes, metrics)]
    delta = [a-b for a, b in zip(front['position_enu_m'], rear['position_enu_m'])]
    baseline = math.hypot(*delta[:2])
    if not 1. <= baseline <= 10. or abs(delta[2]) > max(1., baseline*.5):
        return None
    pose = frame['snapshot']['body_pose_enu']
    if 'orientation_covariance_rad2' not in pose:
        return None
    angular = covariance_matrix(pose['orientation_covariance_rad2'], 'actual pose angular covariance')
    if angular[2][2] <= 0:
        return None
    rotation = matmul(quaternion_rotation(pose['orientation_xyzw']), iris_left_optical_to_body()['rotation'])
    cov_optical = [[sum(m['covariance_camera_optical_m2'][i][j] for m in metrics) for j in range(3)] for i in range(3)]
    cov_axis = matmul(matmul(rotation, cov_optical), transpose(rotation))
    perpendicular = [-delta[1]/baseline, delta[0]/baseline, 0.]
    variance = sum(perpendicular[i]*cov_axis[i][j]*perpendicular[j] for i in range(3) for j in range(3))
    yaw_variance = variance/(baseline*baseline)+angular[2][2]
    if not 0 < yaw_variance <= math.radians(20.)**2:
        return None
    return {'schema_version': 1, 'source_capture_id': frame['capture_id'],
        'image_sha256': copy.deepcopy(observation['image_sha256']), 'coordinate_epoch': copy.deepcopy(observation['coordinate_epoch']),
        'front_surface_enu_m': front['position_enu_m'], 'rear_surface_enu_m': rear['position_enu_m'],
        'front_covariance_enu_m2': front['covariance_enu_m2'], 'yaw_enu_rad': math.atan2(delta[1], delta[0]),
        'rear_covariance_enu_m2': rear['covariance_enu_m2'],
        'yaw_variance_rad2': yaw_variance, 'axis_length_m': baseline, 'part_boxes': boxes,
        'part_measurements': copy.deepcopy(metrics), 'visual_descriptions': list(descriptions),
        'semantic_front_independently_verified': False, 'uncertainty_statistics_calibrated': False,
        'basis': 'visual_front_rear_regions_with_synchronized_stereo_surface_axis',
        'complete_extent_known': False}
