"""Target-centred observation goals from owned visual evidence, never scene truth.

The reference is a measured visible surface, not the centre/front of an object
or a landing site. Range limits are observation policy, not flight clearance.
"""
import copy
import math
import re

from stereo_localization import (covariance_matrix, vector, finite, optical_to_enu,
                                 triangulate_pixel, iris_left_optical_to_body, DEFAULT_LIMITS)


MIN_RANGE_M = 2.5
DESIRED_RANGE_M = 4.
MAX_RANGE_M = 5.5
RANGE_POLICY = 'same_frame_optical_radial_v1'
LEGACY_RANGE_POLICY = 'legacy_enu_bound_v1'


def surface_reference(landmark, observation):
    point = observation.get('position_enu_m')
    covariance = observation.get('covariance_enu_m2', landmark.covariance_enu_m2)
    if point is None:
        return None
    try:
        point = vector(point, 3, 'observed surface')
        matrix = covariance_matrix(covariance, 'surface covariance')
    except (TypeError, ValueError):
        return None
    if any(matrix[i][i] <= 0 for i in range(3)):
        return None
    # A conservative bound, including off-diagonal terms. No calibrated
    # probability is claimed for this propagated/fused stereo covariance.
    sigma = math.sqrt(max(sum(abs(v) for v in row) for row in matrix))
    if sigma > .75:
        return None
    return {'point_enu_m': point, 'uncertainty_margin_m': 2*sigma,
            'source_capture_id': observation['capture_id'], 'target_id': landmark.id,
            'basis': 'validated_stereo_visible_surface_not_object_center',
            'min_range_m': MIN_RANGE_M, 'desired_range_m': DESIRED_RANGE_M,
            'max_range_m': MAX_RANGE_M}


def relative_range_reference(observation):
    """Validate owned stereo provenance and propagate range in the optical frame.

    The covariance is an uncalibrated first-order measurement model. Translation
    and attitude uncertainty of the world frame do not change this same-frame
    relative range. No fallback to fused/world positions is permitted here.
    """
    evidence = observation.get('relative_stereo')
    if not isinstance(evidence, dict):
        raise ValueError('No saved same-frame optical stereo measurement')
    metric = evidence['measurement']
    epoch = observation['coordinate_epoch']
    hashes = observation['image_sha256']
    if (not isinstance(observation['capture_id'], str) or not observation['capture_id']
            or not isinstance(observation['stereo_pair_id'], str) or not observation['stereo_pair_id']
            or not isinstance(epoch, dict) or type(epoch.get('vio_epoch')) is not int
            or epoch['vio_epoch'] < 0 or not isinstance(epoch.get('ekf_counters'), dict)
            or set(epoch['ekf_counters']) != {'xy', 'z', 'heading'}
            or any(type(v) is not int or not 0 <= v <= 255 for v in epoch['ekf_counters'].values())
            or not isinstance(hashes, dict) or set(hashes) != {'left', 'right'}
            or any(not isinstance(v, str) or not re.fullmatch('[0-9a-f]{64}', v) for v in hashes.values())):
        raise ValueError('Relative stereo has incomplete capture/coordinate provenance')
    if (evidence['capture_id'] != observation['capture_id']
            or evidence['pair_id'] != observation['stereo_pair_id']
            or evidence['bbox'] != observation['bbox']
            or metric['status'] != 'ok' or metric['method'] != 'stereo_sgbm_robust_surface'
            or metric['frame'] != 'local_enu'
            or metric['coordinate_epoch'] != observation['coordinate_epoch']
            or metric['image_sha256'] != observation['image_sha256']):
        raise ValueError('Relative stereo belongs to another capture, box, pair or coordinate epoch')
    stamp = finite(metric['image_stamp_sim_s'], 'stereo image stamp')
    pose_stamp = finite(metric['body_pose_stamp_sim_s'], 'stereo pose stamp')
    if (stamp < 0 or pose_stamp < 0
            or abs(stamp - finite(observation['stamp_sim_s'], 'observation stamp')) > 1e-6
            or abs(pose_stamp - finite(observation['body_pose_enu']['stamp_sim_s'], 'pose stamp')) > 1e-6
            or abs(pose_stamp - stamp) > DEFAULT_LIMITS['max_pose_offset_s']):
        raise ValueError('Relative stereo time is inconsistent with the actual observation')
    point = vector(metric['position_camera_optical_m'], 3, 'optical surface')
    matrix = covariance_matrix(metric['covariance_camera_optical_m2'], 'optical covariance')
    if (any(matrix[i][i] <= 0 for i in range(3))
            or not DEFAULT_LIMITS['min_depth_m'] <= point[2] <= DEFAULT_LIMITS['max_depth_m']):
        raise ValueError('Relative stereo has no usable optical depth/covariance')
    calibration, quality = metric['calibration'], metric['quality']
    k = vector(observation['camera_info']['K'], 9, 'observation intrinsics')
    expected_calibration = dict(fx=k[0], fy=k[4], cx=k[2], cy=k[5],
                                baseline_m=iris_left_optical_to_body()['baseline_m'])
    if any(abs(finite(calibration[name], name) - value) > 1e-6
           for name, value in expected_calibration.items()):
        raise ValueError('Relative stereo calibration differs from the captured camera')
    u, v = vector(quality['representative_pixel_uv'], 2, 'surface pixel')
    sigma_d = finite(quality['sigma_disparity_px'], 'disparity sigma')
    box = vector(observation['bbox'], 4, 'target box')
    if (sigma_d < .5 or not box[0]*640 <= u <= box[2]*640
            or not box[1]*480 <= v <= box[3]*480):
        raise ValueError('Relative stereo pixel or uncertainty is inconsistent with its target box')
    if (not DEFAULT_LIMITS['min_valid_ratio'] <= finite(quality['valid_ratio'], 'stereo valid ratio') <= 1.
            or not DEFAULT_LIMITS['min_lr_consistent_ratio'] <= finite(quality['lr_consistent_ratio'], 'stereo LR ratio') <= 1.
            or finite(quality['robust_inlier_pixels'], 'stereo inliers') < DEFAULT_LIMITS['min_valid_pixels']):
        raise ValueError('Relative stereo lacks accepted surface support')
    # Reconstruct the localizer's analytic measurement model. A missing/zero or
    # artificially reduced covariance cannot become high-confidence range data.
    expected_point, expected_covariance = triangulate_pixel(
        u, v, quality['median_disparity_px'], calibration, sigma_disparity_px=sigma_d)
    if (any(not math.isclose(a, b, rel_tol=1e-7, abs_tol=1e-9)
            for a, b in zip(point, expected_point))
            or any(not math.isclose(matrix[i][j], expected_covariance[i][j], rel_tol=1e-7, abs_tol=1e-9)
                   for i in range(3) for j in range(3))):
        raise ValueError('Relative stereo point/covariance differs from its recorded measurement model')
    world_point, _, _ = optical_to_enu(point, matrix, observation['body_pose_enu'])
    if math.dist(world_point, vector(observation['position_enu_m'], 3, 'observed ENU surface')) > 1e-6:
        raise ValueError('Optical surface does not correspond to this actual ENU observation')
    distance = math.sqrt(sum(p*p for p in point))
    direction = [p/distance for p in point]
    variance = sum(direction[i]*matrix[i][j]*direction[j] for i in range(3) for j in range(3))
    if variance <= 0:
        raise ValueError('Relative range variance must be positive')
    sigma = math.sqrt(variance)
    return {'source_capture_id': observation['capture_id'], 'pair_id': evidence['pair_id'],
            'image_sha256': dict(observation['image_sha256']), 'stamp_sim_s': stamp,
            'coordinate_epoch': copy.deepcopy(observation['coordinate_epoch']), 'frame': 'left_camera_optical',
            'range_m': distance, 'radial_sigma_m': sigma, 'uncertainty_margin_m': 2*sigma,
            'range_interval_m': [distance-2*sigma, distance+2*sigma],
            'uncertainty_statistics_calibrated': False,
            'basis': 'same_frame_stereo_visible_surface_radial_not_world_position'}


def observation_quality(landmark, observation, *, policy=RANGE_POLICY):
    """Photo quality; callers must separately verify visual identity and freshness.

    Legacy evaluation is exclusively for read-only audits of archived outcomes.
    Production calls always use the explicit same-frame relative-range policy.
    """
    if policy == LEGACY_RANGE_POLICY:
        return _legacy_observation_quality(landmark, observation)
    if policy != RANGE_POLICY:
        raise ValueError('Unknown close observation range policy')
    box = vector(observation['bbox'], 4, 'target box')
    framed = (min(box[0], box[1], 1-box[2], 1-box[3]) >= .06-1e-9
              and box[2]-box[0] >= .12-1e-9 and box[3]-box[1] >= .12-1e-9)
    try:
        relative = relative_range_reference(observation)
        reason = None
    except (ValueError, KeyError, TypeError) as exc:
        relative, reason = None, str(exc)
    distance = relative['range_m'] if relative else None
    in_range = bool(relative and relative['range_interval_m'][0] >= MIN_RANGE_M
                    and relative['range_interval_m'][1] <= MAX_RANGE_M)
    return {'completed': framed and in_range, 'capture_id': observation['capture_id'],
            'target_id': landmark.id, 'range_m': distance, 'framing_usable': framed,
            'range_usable': in_range, 'range_policy': RANGE_POLICY,
            'range_reference': relative, 'range_unavailable_reason': reason,
            'surface_reference': surface_reference(landmark, observation),
            'basis': 'actual_saved_observation_depth_and_framing_after_visual_review'}


def _legacy_observation_quality(landmark, observation):
    """Exact historical calculation; never infer missing local measurements."""
    reference = surface_reference(landmark, observation)
    box = vector(observation['bbox'], 4, 'target box')
    framed = (min(box[0], box[1], 1-box[2], 1-box[3]) >= .06-1e-9
              and box[2]-box[0] >= .12-1e-9 and box[3]-box[1] >= .12-1e-9)
    distance = (math.dist(observation['bearing_origin_enu_m'], reference['point_enu_m'])
                if reference else None)
    in_range = bool(reference and distance-reference['uncertainty_margin_m'] >= MIN_RANGE_M
                    and distance+reference['uncertainty_margin_m'] <= MAX_RANGE_M)
    return {'completed': framed and in_range, 'capture_id': observation['capture_id'],
            'target_id': landmark.id, 'range_m': distance, 'framing_usable': framed,
            'range_usable': in_range, 'surface_reference': reference,
            'basis': 'actual_saved_observation_depth_and_framing_after_visual_review'}
