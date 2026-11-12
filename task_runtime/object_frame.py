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
FRONT_SURFACE_BASIS = 'opaque_front_surface_consensus_v1'
FRONT_RELATION_BASES = frozenset((FRONT_FACE_BASIS, FRONT_SURFACE_BASIS))


def front_surface_regions(face_box):
    """At most 64 small regions inside a visually delimited opaque front face.

    Two patch heights sample texture at different rows. This is extraction from
    one actual stereo pair, never 64 observations or independent measurements.
    """
    x1, y1, x2, y2 = bbox(face_box)
    width, height = (x2-x1)*640., (y2-y1)*480.
    if min(x1, y1, 1-x2, 1-y2) <= .005 or width < 16.:
        return []
    regions = []
    for ph in (24., 32.):
        if ph > height:
            continue
        for row in range(8):
            cy = y1*480+ph/2+(height-ph)*row/7
            for col in range(4):
                cx = x1*640+8.+(width-16.)*col/3
                region = [(cx-8.)/640, (cy-ph/2)/480, (cx+8.)/640, (cy+ph/2)/480]
                if region not in regions:
                    regions.append(region)
    return regions


def estimate_front_surface_frame(frame, observation, face_box, regions, metrics, description):
    """Robust horizontal front normal under the existing near-vertical-face assumption.

    Every sample passes the unchanged stereo gates. Consensus rejects depth
    outliers; neither consensus size nor pixel count reduces covariance. The
    normal retains the old conservative two-extreme covariance bound, plus
    observed fit disagreement. Glass/background must be excluded by the actual
    visual opaque-face region; geometric consensus cannot identify a vehicle.
    """
    import numpy as np
    from itertools import combinations

    def need(reason):
        return {'status': 'needs_observation', 'reason_code': reason, 'orientation': None}
    if frame['capture_id'] != observation['capture_id'] or frame.get('delivered') is not True:
        raise ValueError('Front surface needs the actually delivered same-target image')
    if not isinstance(description, str) or not 0 < len(description.strip()) <= 1000:
        raise ValueError('Opaque front surface requires a visible structure description')
    face = bbox(face_box)
    outer = observation['bbox']
    if not (outer[0] <= face[0] < face[2] <= outer[2] and outer[1] <= face[1] < face[3] <= outer[3]):
        raise ValueError('Opaque front face is outside the current same-target box')
    expected = front_surface_regions(face)
    if not expected:
        return need('FRONT_SURFACE_CLIPPED_OR_TOO_SMALL')
    if regions != expected or len(metrics) != len(regions):
        raise ValueError('Front surface samples differ from the bounded acquired regions')
    samples, seen = [], set()
    for index, (box, metric) in enumerate(zip(regions, metrics)):
        if metric.get('status') != 'ok':
            continue
        measured_part(frame, observation, box, metric)
        pixel = metric.get('quality', {}).get('representative_pixel_uv')
        if (not isinstance(pixel, list) or len(pixel) != 2
                or not box[0]*640 <= pixel[0] <= box[2]*640
                or not box[1]*480 <= pixel[1] <= box[3]*480):
            raise ValueError('Front sample pixel is outside its actual region')
        key = tuple(pixel)
        if key not in seen:
            seen.add(key)
            samples.append((index, metric))
    if len(samples) < 5:
        return need('FRONT_SURFACE_DEPTH_SUPPORT_MISSING')
    points = np.asarray([m['position_enu_m'] for _, m in samples], dtype=float)
    pixels = np.asarray([m['quality']['representative_pixel_uv'] for _, m in samples], dtype=float)
    hypotheses = []
    for a, b in combinations(range(len(samples)), 2):
        delta = points[b, :2]-points[a, :2]
        length = float(np.linalg.norm(delta))
        if not 1. <= length <= 6. or abs(points[a, 2]-points[b, 2]) > .5:
            continue
        n = np.array([-delta[1], delta[0]])/length
        residuals = np.abs((points[:, :2]-points[a, :2])@n)
        inside = residuals <= .12
        if int(inside.sum()) >= 5:
            hypotheses.append((int(inside.sum()), float(np.median(residuals[inside])), inside))
    if not hypotheses:
        return need('FRONT_SURFACE_DIRECTION_UNRESOLVED')
    _, _, inside = max(hypotheses, key=lambda v: (v[0], -v[1]))
    indices = np.flatnonzero(inside)
    if len(indices) < max(5, math.ceil(.6*len(samples))):
        return need('FRONT_SURFACE_NON_COPLANAR_SUPPORT')
    span = np.ptp(pixels[inside], axis=0)
    if span[0] < (face[2]-face[0])*640*.55:
        return need('FRONT_SURFACE_LATERAL_SUPPORT_TOO_NARROW')
    centered = points[inside, :2]-points[inside, :2].mean(axis=0)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    normal = axes[-1]
    # A whole unsupported depth layer or a grazing line is not a front plane.
    residuals = np.abs(centered@normal)
    if float(residuals.max()) > .12:
        return need('FRONT_SURFACE_REFIT_DISAGREES')
    tangent = np.array([-normal[1], normal[0]])
    along_face = points[inside, :2]@tangent
    extremes = [int(indices[j]) for j in (np.argmin(along_face), np.argmax(along_face))]
    width = float(np.ptp(along_face))
    if not 1. <= width <= 6.:
        return need('FRONT_SURFACE_METRIC_BASELINE_INVALID')
    pose = frame['snapshot']['body_pose_enu']
    angular = covariance_matrix(pose['orientation_covariance_rad2'], 'actual pose angular covariance')
    if any(angular[i][i] <= 0 for i in range(3)):
        return need('FRONT_FACE_POSE_UNCERTAINTY_UNAVAILABLE')
    rotation = np.asarray(matmul(quaternion_rotation(pose['orientation_xyzw']), iris_left_optical_to_body()['rotation']))
    cov_optical = 2*sum((np.asarray(samples[i][1]['covariance_camera_optical_m2']) for i in extremes))
    cov_width = rotation@cov_optical@rotation.T
    direction = np.array([normal[0], normal[1], 0.])
    extreme_bound = float(direction@cov_width@direction)
    largest_sample = max(float(direction@rotation@np.asarray(samples[i][1]['covariance_camera_optical_m2'])
        @rotation.T@direction) for i in indices)
    # Interior samples can be noisier than the two extremes. Their unknown
    # correlation must not vanish through the fit or a favourable pair choice.
    variance = max(extreme_bound, 4*largest_sample)/(width*width)
    # Leave out each lateral support group. Fit instability is a systematic
    # uncertainty contribution, never divided by the number of patches.
    disagreement = 0.
    for group in range(4):
        retained = [i for i in indices if min(3, int(4*(pixels[i, 0]/640-face[0])/(face[2]-face[0]))) != group]
        if len(retained) < 3 or np.ptp(points[retained, :2]@tangent) < width*.5:
            continue
        _, _, other_axes = np.linalg.svd(points[retained, :2]-points[retained, :2].mean(axis=0), full_matrices=False)
        disagreement = max(disagreement, math.acos(min(1., abs(float(other_axes[-1]@normal)))))
    variance += disagreement*disagreement+(float(residuals.max())/width)**2
    if not variance > 0:
        return need('FRONT_FACE_NORMAL_UNCERTAINTY_UNAVAILABLE')
    medoid = min(indices, key=lambda j: float(np.linalg.norm(pixels[j]-np.median(pixels[inside], axis=0))))
    front = samples[medoid][1]
    observer = [pose['position_m'][i]-front['position_enu_m'][i] for i in (0, 1)]
    if float(normal@observer) < 0:
        normal = -normal
    along = float(normal@observer)
    across = abs(float(np.array([-normal[1], normal[0]])@observer))
    if along <= 0 or across > along:
        return need('FRONT_FACE_NOT_VIEWED_FROM_ITS_FRONTAL_REGION')
    angle = 2*math.sqrt(variance+angular[0][0]+angular[1][1])
    margin = 2*math.sqrt(sum(front['covariance_camera_optical_m2'][i][i] for i in range(3)))
    minimum = along*math.cos(angle)-across*math.sin(angle)-margin-.70 if angle < math.pi/3 else -1.
    orientation = {'schema_version': 1, 'basis': FRONT_SURFACE_BASIS,
        'source_capture_id': frame['capture_id'], 'image_sha256': copy.deepcopy(observation['image_sha256']),
        'coordinate_epoch': copy.deepcopy(observation['coordinate_epoch']),
        'front_surface_enu_m': list(front['position_enu_m']), 'front_covariance_enu_m2': copy.deepcopy(front['covariance_enu_m2']),
        'rear_surface_enu_m': None, 'rear_covariance_enu_m2': None,
        'yaw_enu_rad': math.atan2(normal[1], normal[0]), 'yaw_variance_rad2': variance+angular[2][2],
        'global_axis_usable': variance+angular[2][2] <= math.radians(20.)**2,
        'relative_axis_variance_rad2': variance, 'axis_length_m': width,
        'face_box': face, 'sample_regions': copy.deepcopy(regions), 'sample_measurements': copy.deepcopy(metrics),
        'inlier_sample_indices': [samples[i][0] for i in indices], 'front_sample_index': samples[medoid][0],
        'extreme_sample_indices': [samples[i][0] for i in extremes], 'visual_descriptions': [description],
        'fitting': {'valid_samples': len(samples), 'inliers': len(indices), 'lateral_span_px': float(span[0]),
                    'maximum_residual_m': float(residuals.max()), 'leave_group_out_angle_rad': disagreement,
                    'uncertainty_basis': 'two_extremes_and_all_inlier_unknown_correlation_plus_fit_systematics_no_count_reduction'},
        'body_position_at_capture_enu_m': list(pose['position_m']),
        'current_front_relation': {'minimum_front_distance_m': minimum, 'minimum_required_m': 2.,
            'normal_uncertainty_margin_rad': angle, 'position_uncertainty_margin_m': margin,
            'pose_and_descent_tracking_reserve_m': .70,
            'basis': 'same_frame_front_normal_and_stereo_surface_with_common_reference_cancelled'},
        'complete_extent_known': False, 'semantic_front_independently_verified': False,
        'uncertainty_statistics_calibrated': False}
    return {'status': 'available', 'reason_code': 'OPAQUE_FRONT_SURFACE_NORMAL_MEASURED', 'orientation': orientation}


def front_relation_quality(evidence, capture_id, face_box=None):
    """A measurement is distinct from usable local relation or global direction."""
    orientation = evidence.get('orientation')
    quality = {'source_capture_id': capture_id, 'face_box': copy.deepcopy(face_box),
               'measurement_acquired': orientation is not None, 'relation_usable': False,
               'goal_pose_evidence_usable': False,
               'global_axis_usable': False, 'reason_code': evidence['reason_code'], 'missing_information': []}
    if orientation is None:
        reason = evidence['reason_code']
        quality['missing_information'] = ['front_clipped' if 'CLIPPED' in reason else
            'front_depth_missing' if 'DEPTH' in reason else 'opaque_front_lateral_structure']
        return quality
    relation = orientation['current_front_relation']
    local = relation['minimum_front_distance_m'] >= relation['minimum_required_m']
    quality.update(global_axis_usable=orientation['global_axis_usable'], same_frame_relation_usable=local,
        relation_usable=local, goal_pose_evidence_usable=local or orientation['global_axis_usable'],
        angular_margin_rad=relation['normal_uncertainty_margin_rad'],
        relative_axis_variance_rad2=orientation['relative_axis_variance_rad2'],
        lateral_baseline_m=orientation['axis_length_m'], minimum_front_distance_m=relation['minimum_front_distance_m'])
    metrics = orientation.get('part_measurements') or orientation['sample_measurements']
    metric = metrics[orientation.get('front_sample_index', 0)]
    quality.update(optical_depth_m=metric['position_camera_optical_m'][2],
                   depth_sigma_m=math.sqrt(metric['covariance_camera_optical_m2'][2][2]))
    # An uncertain measured normal may guide another observation. It is not a
    # usable object axis or a landing proof; preserve its source and covariance.
    quality['viewpoint_reference'] = {
        'basis': 'measured_front_surface_for_viewpoint_prediction_only',
        'prediction_only': True, 'execution_authorized': False,
        'source_capture_id': orientation['source_capture_id'],
        'image_sha256': copy.deepcopy(orientation['image_sha256']),
        'coordinate_epoch': copy.deepcopy(orientation['coordinate_epoch']),
        'point_enu_m': list(orientation['front_surface_enu_m']),
        'covariance_enu_m2': copy.deepcopy(orientation['front_covariance_enu_m2']),
        'normal_enu_xy': [math.cos(orientation['yaw_enu_rad']), math.sin(orientation['yaw_enu_rad'])],
        'yaw_variance_rad2': orientation['yaw_variance_rad2'],
        'position_uncertainty_margin_m': relation['position_uncertainty_margin_m'],
        'pose_and_descent_tracking_reserve_m': relation['pose_and_descent_tracking_reserve_m'],
        'minimum_required_m': relation['minimum_required_m']}
    if not quality['relation_usable']:
        quality['missing_information'] = ['front_angular_uncertainty' if relation['normal_uncertainty_margin_rad'] >= math.pi/3
            else 'front_relation_standoff_or_uncertainty']
    return quality


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
    if orientation.get('basis') not in FRONT_RELATION_BASES or orientation['source_capture_id'] != observation['capture_id']:
        return None
    frame = {'capture_id': observation['capture_id'], 'delivered': True,
             'snapshot': {'body_pose_enu': observation['body_pose_enu'],
                          'stereo_pair_id': observation['stereo_pair_id']}}
    computed = (estimate_front_surface_frame(frame, observation, orientation['face_box'], orientation['sample_regions'],
                orientation['sample_measurements'], orientation['visual_descriptions'][0])
                if orientation['basis'] == FRONT_SURFACE_BASIS else
                estimate_front_face_frame(frame, observation, orientation['part_boxes'],
                                         orientation['part_measurements'], orientation['visual_descriptions'][0]))
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


def verify_front_review(landmark, observation, frame, box, metrics, description, side_boxes=None,
                        surface_regions=None, surface_metrics=None):
    orientation = landmark.attributes.get('object_orientation')
    if not orientation or orientation['coordinate_epoch'] != observation['coordinate_epoch']:
        return None
    if not isinstance(description, str) or not 0 < len(description.strip()) <= 1000:
        raise ValueError('Fresh front verification requires a concrete visual description')
    surface_profile = orientation.get('basis') == FRONT_SURFACE_BASIS
    if box is None or (not surface_profile and (not metrics or metrics[0].get('status') != 'ok')):
        return None
    outer = observation['bbox']
    if not (outer[0] <= box[0] < box[2] <= outer[2] and outer[1] <= box[1] < box[3] <= outer[3]):
        raise ValueError('Fresh front face must be inside the actually reported same-target box')
    if orientation.get('basis') in FRONT_RELATION_BASES:
        source_metric = (orientation['sample_measurements'][orientation['front_sample_index']]
                         if surface_profile else orientation['part_measurements'][0])
        if (frame['capture_id'] == orientation['source_capture_id']
                or observation['stamp_sim_s'] <= source_metric['image_stamp_sim_s']):
            return None
        if surface_profile:
            if surface_regions is None or surface_metrics is None:
                return None
            fresh = estimate_front_surface_frame(frame, observation, box, surface_regions, surface_metrics, description)['orientation']
        else:
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
        front_metric = (surface_metrics[fresh['front_sample_index']] if surface_profile else metrics[0])
        return {'schema_version': 1, 'status': 'verified', 'capture_id': frame['capture_id'],
            'image_sha256': copy.deepcopy(observation['image_sha256']),
            'coordinate_epoch': copy.deepcopy(observation['coordinate_epoch']),
            'front_box': list(box), 'front_measurement': copy.deepcopy(front_metric),
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
