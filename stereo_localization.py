"""Read-only, calibrated stereo surface localization; no ROS, Agent or flight IO.

The initial profile is the current rectified Iris pair, not an arbitrary stereo
rig. Optical x/right y/down z/forward is converted using the actual SDF mounting
relative to base_link. Gazebo object poses are neither read nor accepted.
"""
import base64
import hashlib
import math


class StereoLocalizationError(ValueError):
    pass


DEFAULT_LIMITS = {
    "max_pair_offset_s": 1e-6, "max_pose_offset_s": .05,
    "max_image_age_wall_s": 2., "max_pose_age_wall_s": 1.,
    "max_measurement_age_sim_s": .5, "min_depth_m": .4, "max_depth_m": 12.,
    "min_valid_ratio": .35, "min_lr_consistent_ratio": .65,
    "max_lr_error_px": 1., "min_texture_variance": 20.,
    "min_valid_pixels": 40, "max_relative_disparity_spread": .20,
    "max_relative_depth_sigma": .20, "max_depth_sigma_m": 1.5,
    "roi_inner_fraction": .60,
}


def configured_limits(overrides=None):
    if overrides is not None and not isinstance(overrides, dict):
        raise StereoLocalizationError("Stereo quality limits must be a mapping")
    if overrides and set(overrides) - set(DEFAULT_LIMITS):
        raise StereoLocalizationError("Unknown stereo quality limit")
    values = {**DEFAULT_LIMITS, **(overrides or {})}
    for key, value in values.items():
        if finite(value, key) <= 0:
            raise StereoLocalizationError("Stereo quality limits must be positive")
    for key in ("min_valid_ratio", "min_lr_consistent_ratio", "max_relative_disparity_spread",
                "max_relative_depth_sigma", "roi_inner_fraction"):
        if values[key] > 1:
            raise StereoLocalizationError("Stereo quality fractions must not exceed one")
    if values["max_depth_m"] <= values["min_depth_m"]:
        raise StereoLocalizationError("Stereo depth bounds are reversed")
    if (values["max_pair_offset_s"] > 1e-4 or values["max_pose_offset_s"] > .05
            or values["max_image_age_wall_s"] > 2 or values["max_pose_age_wall_s"] > 1
            or values["max_measurement_age_sim_s"] > .5):
        raise StereoLocalizationError("Stereo synchronization and freshness limits cannot be relaxed")
    return values


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise StereoLocalizationError(name + " must be finite")
    return float(value)


def vector(values, length, name):
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise StereoLocalizationError(name + " has invalid dimensions")
    return [finite(value, name) for value in values]


def matmul(a, b):
    return [[sum(x * y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def transpose(a):
    return [list(row) for row in zip(*a)]


def matvec(a, b):
    return [sum(x * y for x, y in zip(row, b)) for row in a]


def covariance_matrix(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise StereoLocalizationError(name + " must be 3x3")
    matrix = [vector(row, 3, name) for row in value]
    if any(matrix[i][i] < 0 for i in range(3)) or any(abs(matrix[i][j] - matrix[j][i]) > 1e-9
                                                   for i in range(3) for j in range(3)):
        raise StereoLocalizationError(name + " must be symmetric positive semidefinite")
    # All principal minors are nonnegative for a symmetric PSD 3x3 matrix.
    for i in range(3):
        for j in range(i + 1, 3):
            if matrix[i][i] * matrix[j][j] - matrix[i][j] ** 2 < -1e-10:
                raise StereoLocalizationError(name + " is not positive semidefinite")
    determinant = (matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] ** 2)
                   - matrix[0][1] * (matrix[0][1] * matrix[2][2] - matrix[1][2] * matrix[0][2])
                   + matrix[0][2] * (matrix[0][1] * matrix[1][2] - matrix[1][1] * matrix[0][2]))
    if determinant < -1e-10:
        raise StereoLocalizationError(name + " is not positive semidefinite")
    return matrix


def quaternion_rotation(quaternion):
    x, y, z, w = vector(quaternion, 4, "body orientation")
    norm = x * x + y * y + z * z + w * w
    if not .99 <= norm <= 1.01:
        raise StereoLocalizationError("body quaternion is not unit normalized")
    scale = math.sqrt(norm)
    x, y, z, w = [component / scale for component in (x, y, z, w)]
    return [[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]]


def iris_left_optical_to_body():
    """Static SDF calibration, relative to base_link; IMU-relative z=.16 is different."""
    angle = .35
    return {"profile": "iris_vio_sdf_base_link_v1", "body_reference": "base_link",
            "translation_m": [.12, .06, .18], "baseline_m": .12,
            "rotation": [[0., -math.sin(angle), math.cos(angle)], [-1., 0., 0.],
                         [0., -math.cos(angle), -math.sin(angle)]]}


def calibration_from_camera_info(left, right, baseline_m=None):
    result = {}
    for side, info in (("left", left), ("right", right)):
        if (info.get("width"), info.get("height")) != (640, 480):
            raise StereoLocalizationError("Only the calibrated 640x480 Iris profile is supported")
        k = vector(info["K"], 9, side + " CameraInfo K")
        if k[0] <= 0 or k[4] <= 0 or abs(k[1]) > 1e-9 or abs(k[3]) > 1e-9 or k[6:] != [0., 0., 1.]:
            raise StereoLocalizationError("CameraInfo is not a rectified pinhole calibration")
        distortion = info.get("distortion_coefficients", info.get("D", []))
        if not isinstance(distortion, (list, tuple)) or any(abs(finite(v, "distortion")) > 1e-9 for v in distortion):
            raise StereoLocalizationError("Distorted images require rectification before localization")
        if "R" in info and any(abs(a-b) > 1e-8 for a, b in zip(vector(info["R"], 9, "CameraInfo R"),
                                                               [1., 0., 0., 0., 1., 0., 0., 0., 1.])):
            raise StereoLocalizationError("Nonidentity rectification is outside the current profile")
        result[side] = {"fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5]}
    if any(abs(result["left"][key] - result["right"][key]) > 1e-6 for key in ("fx", "fy", "cx", "cy")):
        raise StereoLocalizationError("Stereo intrinsics differ; rectify the pair first")
    if "P" in left and "P" in right:
        lp, rp = vector(left["P"], 12, "left P"), vector(right["P"], 12, "right P")
        for side, projection in (("left", lp), ("right", rp)):
            intrinsics = result[side]
            expected = [intrinsics["fx"], 0., intrinsics["cx"], projection[3],
                        0., intrinsics["fy"], intrinsics["cy"], 0., 0., 0., 1., 0.]
            if any(abs(a-b) > 1e-6 for a, b in zip(projection, expected)):
                raise StereoLocalizationError("Projection matrix differs from the rectified intrinsics")
        if abs(lp[3]) > 1e-9:
            raise StereoLocalizationError("Left camera projection must define the stereo reference origin")
        measured_baseline = -rp[3] / rp[0]
        if measured_baseline <= 0:
            raise StereoLocalizationError("Stereo baseline sign/order is invalid")
        if baseline_m is not None and abs(measured_baseline - baseline_m) > .0001:
            raise StereoLocalizationError("CameraInfo baseline differs from the mounting calibration")
        baseline_m, source = measured_baseline, "CameraInfo_P_right"
    else:
        if baseline_m is None:
            raise StereoLocalizationError("Stereo baseline requires CameraInfo P or an explicitly known calibration")
        source = "known_static_SDF_calibration"
    baseline_m = finite(baseline_m, "baseline")
    if abs(baseline_m - .12) > .0001:
        raise StereoLocalizationError("Baseline does not match the current Iris stereo profile")
    return {**result["left"], "width": 640, "height": 480,
            "baseline_m": baseline_m, "baseline_source": source}


def validate_pair(pair, now_sim_s, limits=None):
    limits = configured_limits(limits)
    now = finite(now_sim_s, "simulation now")
    stamps = []
    for side in ("left", "right"):
        frame = pair[side]
        stamp = finite(frame["stamp_sim_s"], side + " image stamp")
        if not 0 <= finite(frame["age_seconds"], side + " receipt age") <= limits["max_image_age_wall_s"]:
            raise StereoLocalizationError("Stereo image receipt is stale or future-dated")
        if not 0 <= now - stamp <= limits["max_measurement_age_sim_s"]:
            raise StereoLocalizationError("Stereo image measurement is stale or future-dated")
        info_stamp = finite(frame["camera_intrinsics"]["stamp_sim_s"], "CameraInfo stamp")
        if abs(info_stamp - stamp) > 1e-6:
            raise StereoLocalizationError("CameraInfo does not belong to the image frame")
        stamps.append(stamp)
    if abs(stamps[0] - stamps[1]) > limits["max_pair_offset_s"]:
        raise StereoLocalizationError("Stereo images are not synchronized")
    pose = pair["body_pose_enu"]
    if pose.get("frame_id") not in ("map", "local_enu") or pose.get("body_reference") != "base_link":
        raise StereoLocalizationError("Pose must be local ENU at base_link, not VIO/world/IMU origin")
    pose_stamp = finite(pose["stamp_sim_s"], "body pose stamp")
    if abs(pose_stamp - stamps[0]) > limits["max_pose_offset_s"] or pose_stamp > now:
        raise StereoLocalizationError("Body pose is not aligned with the image timestamp")
    if not 0 <= finite(pose["age_seconds"], "body pose receipt age") <= limits["max_pose_age_wall_s"]:
        raise StereoLocalizationError("Body pose is stale")
    vector(pose["position_m"], 3, "body position")
    quaternion_rotation(pose["orientation_xyzw"])
    return stamps[0], limits


def triangulate_pixel(u, v, disparity_px, calibration, sigma_disparity_px=.5,
                      sigma_pixel_px=2., sigma_baseline_m=.001, sigma_focal_px=.5):
    """Analytic first-order covariance; stereo samples are correlated, so no sqrt(N) claim."""
    u, v, d = (finite(value, name) for value, name in
               ((u, "u"), (v, "v"), (disparity_px, "disparity")))
    fx, fy, cx, cy, baseline = [finite(calibration[key], key)
                               for key in ("fx", "fy", "cx", "cy", "baseline_m")]
    if d <= 0 or fx <= 0 or fy <= 0 or baseline <= 0:
        raise StereoLocalizationError("Positive disparity, focal lengths and baseline are required")
    sigmas = [finite(value, "measurement sigma") for value in
              (sigma_disparity_px, sigma_pixel_px, sigma_baseline_m, sigma_focal_px)]
    if any(value < 0 for value in sigmas):
        raise StereoLocalizationError("Measurement sigma must be nonnegative")
    sd, sp, sb, sf = sigmas
    z = fx * baseline / d
    x, y = (u - cx) * z / fx, (v - cy) * z / fy
    jacobian = [[z/fx, 0., -x/d], [0., z/fy, -y/d], [0., 0., -z/d]]
    measurement = [[sp*sp, 0., 0.], [0., sp*sp, 0.], [0., 0., sd*sd]]
    covariance = matmul(matmul(jacobian, measurement), transpose(jacobian))
    for gradient, variance in (([x/baseline, y/baseline, z/baseline], sb*sb),
                               ([0., y/fx, z/fx], sf*sf), ([0., -y/fy, 0.], sf*sf)):
        for i in range(3):
            for j in range(3):
                covariance[i][j] += gradient[i] * gradient[j] * variance
    return [x, y, z], covariance


def optical_to_enu(position, covariance, body_pose):
    optical = vector(position, 3, "optical point")
    cov = covariance_matrix(covariance, "optical covariance")
    if body_pose.get("body_reference") != "base_link" or body_pose.get("frame_id") not in ("map", "local_enu"):
        raise StereoLocalizationError("ENU transform requires a base_link pose")
    mounting = iris_left_optical_to_body()
    body_rotation = quaternion_rotation(body_pose["orientation_xyzw"])
    body_position = vector(body_pose["position_m"], 3, "body position")
    rotated_optical = matvec(mounting["rotation"], optical)
    body_point = [a+b for a, b in zip(rotated_optical, mounting["translation_m"])]
    enu_point = [a+b for a, b in zip(matvec(body_rotation, body_point), body_position)]
    rotation = matmul(body_rotation, mounting["rotation"])
    enu_cov = matmul(matmul(rotation, cov), transpose(rotation))
    assumed = (not ("position_covariance_m2" in body_pose and "orientation_covariance_rad2" in body_pose)
               or str(body_pose.get("pose_covariance_source", "")).startswith("configured"))
    position_cov = covariance_matrix(body_pose.get("position_covariance_m2", [[.25**2, 0., 0.],
                                [0., .25**2, 0.], [0., 0., .25**2]]), "body position covariance")
    angular_cov = covariance_matrix(body_pose.get("orientation_covariance_rad2", [[math.radians(3)**2, 0., 0.],
                        [0., math.radians(3)**2, 0.], [0., 0., math.radians(3)**2]]), "body angular covariance")
    # Angular covariance is explicitly in ENU small-angle coordinates.
    px, py, pz = matvec(body_rotation, body_point)
    cross = [[0., -pz, py], [pz, 0., -px], [-py, px, 0.]]
    angle_contribution = matmul(matmul(cross, angular_cov), transpose(cross))
    for i in range(3):
        for j in range(3):
            enu_cov[i][j] += position_cov[i][j] + angle_contribution[i][j]
    return enu_point, enu_cov, assumed


def _cv_dependencies():
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise StereoLocalizationError("Stereo matching requires cv2 and numpy in ROS Python") from exc
    return cv2, np


def decode_frame(frame):
    cv2, np = _cv_dependencies()
    raw = frame.get("jpeg_bytes")
    if raw is None:
        image = frame.get("image_base64")
        if not isinstance(image, str) or len(image) > 3000000:
            raise StereoLocalizationError("Stereo image encoding is invalid")
        try:
            raw = base64.b64decode(image, validate=True)
        except (ValueError, TypeError) as exc:
            raise StereoLocalizationError("Stereo image base64 is invalid") from exc
    if not isinstance(raw, bytes) or not 0 < len(raw) <= 2 * 1024 * 1024:
        raise StereoLocalizationError("Stereo JPEG bytes are unavailable or oversized")
    gray = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if gray is None or gray.shape != (480, 640):
        raise StereoLocalizationError("Stereo image is not the calibrated 640x480 frame")
    return gray, hashlib.sha256(raw).hexdigest()


def image_quality_from_gray(gray):
    """Measured image-quality metadata for coverage bookkeeping in ROS Python."""
    cv2, np = _cv_dependencies()
    if gray.shape != (480, 640) or gray.dtype != np.uint8:
        raise StereoLocalizationError("Image quality requires the calibrated uint8 gray image")
    return {"laplacian_variance": float(cv2.Laplacian(gray, cv2.CV_32F).var()),
            "mean_luma": float(gray.mean()),
            "saturated_fraction": float(((gray <= 5) | (gray >= 250)).mean()),
            "source": "actual_decoded_camera_JPEG", "not_semantic_visibility_proof": True}


def compute_disparities(left_gray, right_gray):
    cv2, np = _cv_dependencies()
    if left_gray.shape != (480, 640) or right_gray.shape != left_gray.shape:
        raise StereoLocalizationError("SGBM requires the calibrated equal-sized stereo images")
    block, disparity_count = 5, 128
    common = {"numDisparities": disparity_count, "blockSize": block,
              "P1": 8*block*block, "P2": 32*block*block, "disp12MaxDiff": 1,
              "preFilterCap": 31, "uniquenessRatio": 12, "speckleWindowSize": 100,
              "speckleRange": 2, "mode": cv2.STEREO_SGBM_MODE_SGBM_3WAY}
    left_matcher = cv2.StereoSGBM_create(minDisparity=0, **common)
    right_matcher = cv2.StereoSGBM_create(minDisparity=-disparity_count, **common)
    return (left_matcher.compute(left_gray, right_gray).astype(np.float32) / 16.,
            right_matcher.compute(right_gray, left_gray).astype(np.float32) / 16.)


def estimate_from_disparities(left_gray, disparity_left, disparity_right, bbox, calibration,
                              body_pose, image_stamp_sim_s, limits=None):
    cv2, np = _cv_dependencies()
    limits = configured_limits(limits)
    bounds = vector(bbox, 4, "normalized bbox")
    x1, y1, x2, y2 = bounds
    if not 0 <= x1 < x2 <= 1 or not 0 <= y1 < y2 <= 1:
        raise StereoLocalizationError("bbox must lie within the image")
    if left_gray.shape != (480, 640) or disparity_left.shape != left_gray.shape or disparity_right.shape != left_gray.shape:
        raise StereoLocalizationError("Disparity arrays do not match the calibrated image")
    fraction = limits["roi_inner_fraction"]
    mx, my = (x1+x2)*320., (y1+y2)*240.
    half_x, half_y = (x2-x1)*320.*fraction, (y2-y1)*240.*fraction
    xmin, xmax = math.ceil(mx-half_x), math.floor(mx+half_x)
    ymin, ymax = math.ceil(my-half_y), math.floor(my+half_y)
    if xmax-xmin < 8 or ymax-ymin < 8:
        raise StereoLocalizationError("Target bbox is too small for robust stereo localization")
    roi_gray = left_gray[ymin:ymax, xmin:xmax]
    texture = float(cv2.Laplacian(roi_gray, cv2.CV_32F).var())
    if not math.isfinite(texture) or texture < limits["min_texture_variance"]:
        raise StereoLocalizationError("Target surface has insufficient texture for stereo matching")
    yy, xx = np.mgrid[ymin:ymax, xmin:xmax]
    disparity = disparity_left[ymin:ymax, xmin:xmax]
    fxb = calibration["fx"] * calibration["baseline_m"]
    raw_valid = (np.isfinite(disparity) & (disparity > 0)
                 & (disparity >= fxb/limits["max_depth_m"])
                 & (disparity <= fxb/limits["min_depth_m"]))
    raw_count, total = int(raw_valid.sum()), int(disparity.size)
    xr = np.rint(xx - np.where(raw_valid, disparity, 0.)).astype(np.int32)
    mapped = (xr >= 0) & (xr < 640)
    right_values = disparity_right[yy, np.clip(xr, 0, 639)]
    lr_valid = (raw_valid & mapped & np.isfinite(right_values) & (right_values < 0)
                & (np.abs(disparity + right_values) <= limits["max_lr_error_px"]))
    lr_count = int(lr_valid.sum())
    ratio = lr_count / max(raw_count, 1)
    if ratio < limits["min_lr_consistent_ratio"]:
        raise StereoLocalizationError("Target stereo left-right consistency is insufficient")
    valid_ratio = lr_count / total
    if lr_count < limits["min_valid_pixels"] or valid_ratio < limits["min_valid_ratio"]:
        raise StereoLocalizationError("Target stereo valid pixel fraction is insufficient")
    median = float(np.median(disparity[lr_valid]))
    mad = float(np.median(np.abs(disparity[lr_valid] - median)))
    spread = 1.4826 * mad
    if spread / median > limits["max_relative_disparity_spread"]:
        raise StereoLocalizationError("Target ROI contains ambiguous depths or excessive disparity spread")
    inlier = lr_valid & (np.abs(disparity - median) <= max(1., 3.*spread))
    inlier_count = int(inlier.sum())
    if inlier_count < limits["min_valid_pixels"] or inlier_count / total < limits["min_valid_ratio"]:
        raise StereoLocalizationError("Target robust stereo inlier fraction is insufficient")
    u, v = float(np.median(xx[inlier])), float(np.median(yy[inlier]))
    median = float(np.median(disparity[inlier]))
    sigma_d = max(.5, spread)
    point, covariance = triangulate_pixel(u, v, median, calibration, sigma_disparity_px=sigma_d)
    sigma_z = math.sqrt(covariance[2][2])
    if sigma_z / point[2] > limits["max_relative_depth_sigma"] or sigma_z > limits["max_depth_sigma_m"]:
        raise StereoLocalizationError("Stereo depth uncertainty exceeds its accepted range")
    enu, enu_cov, assumed = optical_to_enu(point, covariance, body_pose)
    return {"status": "ok", "method": "stereo_sgbm_robust_surface", "frame": "local_enu",
            "image_stamp_sim_s": image_stamp_sim_s, "body_pose_stamp_sim_s": body_pose["stamp_sim_s"],
            "position_camera_optical_m": point, "covariance_camera_optical_m2": covariance,
            "position_enu_m": enu, "covariance_enu_m2": enu_cov,
            "sigma_enu_m": [math.sqrt(max(0., enu_cov[i][i])) for i in range(3)],
            "pose_uncertainty_assumed": assumed, "calibration": calibration,
            "pose_covariance_source": body_pose.get("pose_covariance_source", "provided" if not assumed else "default_assumed_floor"),
            "uncertainty_statistics_calibrated": False,
            "quality": {"roi_pixels": total, "raw_valid_pixels": raw_count, "lr_valid_pixels": lr_count,
                        "robust_inlier_pixels": inlier_count, "valid_ratio": valid_ratio,
                        "lr_consistent_ratio": ratio, "texture_laplacian_variance": texture,
                        "median_disparity_px": median, "disparity_mad_px": mad,
                        "sigma_disparity_px": sigma_d, "depth_sigma_m": sigma_z,
                        "representative_pixel_uv": [u, v]},
            "limitations": ["这是目标框内可见表面的估计点，不代表物体中心或无遮挡的全物体位置",
                            "目标类别和框来自视觉模型；没有实例分割，背景占据框时可能测得背景表面",
                            "协方差为一阶近似，未统计标定；无位姿协方差时采用0.25m/3deg假设",
                            "只读定位结果，不直接生成飞行航点"]}


def localize_stereo_target(pair, bbox, now_sim_s, limits=None):
    stamp, configured_limits = validate_pair(pair, now_sim_s, limits)
    calibration = calibration_from_camera_info(pair["left"]["camera_intrinsics"],
                                               pair["right"]["camera_intrinsics"],
                                               iris_left_optical_to_body()["baseline_m"])
    left, left_hash = decode_frame(pair["left"])
    right, right_hash = decode_frame(pair["right"])
    disparity_left, disparity_right = compute_disparities(left, right)
    result = estimate_from_disparities(left, disparity_left, disparity_right, bbox, calibration,
                                      pair["body_pose_enu"], stamp, configured_limits)
    result["image_sha256"] = {"left": left_hash, "right": right_hash}
    result["stereo_pair_offset_s"] = abs(pair["left"]["stamp_sim_s"] - pair["right"]["stamp_sim_s"])
    return result
