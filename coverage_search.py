"""Deterministic view sampling and measured-target memory, with zero flight IO.

Visiting a sample does not prove every ground patch is visible. Only successful
navigation followed by a fresh, clear real image marks a sample observed. Actual
poses determine potential geometric visibility; occlusion-free coverage is never
claimed. Measured target coordinates are report data, never generated waypoints.
"""
import copy
import math
import re
import time

from stereo_localization import (StereoLocalizationError, covariance_matrix, finite,
                                  iris_left_optical_to_body, matvec, quaternion_rotation,
                                  transpose, vector)


class CoverageSearchError(ValueError):
    pass


def bounds_vector(value):
    try:
        if isinstance(value, dict) and set(value) == {"x", "y"}:
            value = [value["x"][0], value["y"][0], value["x"][1], value["y"][1]]
        xmin, ymin, xmax, ymax = vector(value, 4, "ENU region bounds")
    except (ValueError, KeyError, IndexError, TypeError) as exc:
        raise CoverageSearchError("Bounds must be [xmin,ymin,xmax,ymax] or x/y ranges") from exc
    if not xmin < xmax or not ymin < ymax:
        raise CoverageSearchError("Region bounds must have positive width and height")
    return [xmin, ymin, xmax, ymax]


def _bounded_integer(value, minimum, maximum, name):
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise CoverageSearchError(name + " is outside its bounded integer range")
    return value


def navigation_goal_permission(map_bounds_enu_m, obstacle_inflation_m, visible_clearance_m):
    """Use the actual navigation goal rule once, after map's scene inset."""
    physical = bounds_vector(map_bounds_enu_m)
    inflation = finite(obstacle_inflation_m, "actual map obstacle inflation")
    clearance = finite(visible_clearance_m, "actual map visible clearance")
    if inflation < 1 or clearance < 1.2:
        raise CoverageSearchError("Actual navigation inflation/visible clearance evidence is insufficient")
    margin = max(inflation, clearance)
    usable = bounds_vector([physical[0]+margin, physical[1]+margin, physical[2]-margin, physical[3]-margin])
    return {"navigation_map_bounds_enu_m": physical, "obstacle_inflation_m": inflation,
            "visible_clearance_m": clearance, "navigation_goal_boundary_margin_m": margin,
            "usable_goal_bounds_enu_m": usable, "scene_flight_margin_applied_again": False,
            "basis": "NavigationMap.plan goal clearance; map bounds already contain the scene flight margin"}


def verified_nominal_footprint(camera_info, camera_height_m, max_range_m):
    """Inscribed ground rectangle for the actual pitch, K and clipped optical Z.

This geometric footprint assumes flat ground and unobstructed nominal attitude.
Its dimensions bound the sampling spacing; real-image visibility is handled
separately and does not inherit the planning assumption.
"""
    width, height = camera_info.get("width"), camera_info.get("height")
    if (width, height) != (640, 480):
        raise CoverageSearchError("Coverage planning requires the calibrated 640x480 camera")
    k = vector(camera_info["K"], 9, "CameraInfo K")
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    if fx <= 0 or fy <= 0 or not 0 < cx < width or not 0 < cy < height:
        raise CoverageSearchError("Camera intrinsics are invalid")
    camera_height_m = finite(camera_height_m, "camera height")
    max_range_m = finite(max_range_m, "camera optical range")
    if camera_height_m <= 0 or not .4 < max_range_m <= 12:
        raise CoverageSearchError("Ground geometry requires positive height and optical range <=12m")
    sine, cosine = math.sin(.35), math.cos(.35)
    top, bottom = -cy/fy, (height-1-cy)/fy
    far_t = max(top, (camera_height_m/max_range_m - sine)/cosine)
    if far_t >= bottom:
        raise CoverageSearchError("No ground footprint is visible inside the configured range")
    near_scale = camera_height_m / (sine + cosine*bottom)
    far_scale = camera_height_m / (sine + cosine*far_t)
    near_forward = (cosine-sine*bottom)*near_scale
    far_forward = (cosine-sine*far_t)*far_scale
    horizontal_half = min(cx/fx, (width-1-cx)/fx)
    inscribed_width = 2*horizontal_half*near_scale
    inscribed_length = far_forward-near_forward
    if min(inscribed_width, inscribed_length) <= 0:
        raise CoverageSearchError("Camera cannot support a positive ground footprint")
    return {"width_m": inscribed_width, "length_m": inscribed_length,
            "near_forward_m": near_forward, "far_forward_m": far_forward,
            "maximum_spacing_m": .8 * min(inscribed_width, inscribed_length),
            "basis": "nominal calibrated pitch; 20% overlap in inscribed flat-ground rectangle",
            "occlusion_free_guarantee": False}


def plan_search(bounds_enu_m, spacing_m, altitude_m, camera_info, max_range_m=10.,
                max_observations=12, allowed_bounds_enu_m=None, ground_enu_z=0.,
                required_altitude_m=3.5, flight_radius_m=30., max_duration_wall_s=900., planning_yaw_rad=0.,
                navigation_permission=None):
    bounds = bounds_vector(bounds_enu_m)
    if allowed_bounds_enu_m is None:
        raise CoverageSearchError("Explicit allowed ENU bounds are required for search planning")
    allowed = bounds_vector(allowed_bounds_enu_m)
    if navigation_permission is not None:
        canonical = navigation_goal_permission(navigation_permission["navigation_map_bounds_enu_m"],
                                               navigation_permission["obstacle_inflation_m"],
                                               navigation_permission["visible_clearance_m"])
        if canonical != navigation_permission or canonical["usable_goal_bounds_enu_m"] != allowed:
            raise CoverageSearchError("Vehicle goal bounds do not match actual navigation boundary clearance")
        navigation_permission = canonical
    altitude = finite(altitude_m, "search altitude")
    required = finite(required_altitude_m, "required navigation altitude")
    radius = finite(flight_radius_m, "flight radius")
    if not 1 <= altitude <= 10 or abs(altitude-required) > 1e-6 or not 0 < radius <= 30:
        raise CoverageSearchError("Search must use the permitted fixed altitude and flight radius")
    spacing = finite(spacing_m, "observation spacing")
    ground = finite(ground_enu_z, "ground ENU z")
    duration = finite(max_duration_wall_s, "search duration")
    budget = _bounded_integer(max_observations, 1, 24, "observation budget")
    if not .25 <= spacing <= 10 or not 1 <= duration <= 3600:
        raise CoverageSearchError("Spacing or search duration exceeds its supported range")
    footprint = verified_nominal_footprint(camera_info, altitude-ground+.18, max_range_m)
    if spacing > footprint["maximum_spacing_m"]:
        raise CoverageSearchError("Observation spacing exceeds the verified FOV/range footprint")
    nx = max(1, int(math.ceil((bounds[2]-bounds[0])/spacing)))
    ny = max(1, int(math.ceil((bounds[3]-bounds[1])/spacing)))
    if nx*ny > budget:
        raise CoverageSearchError("The whole-region sampling plan exceeds the observation budget")
    dx, dy = (bounds[2]-bounds[0])/nx, (bounds[3]-bounds[1])/ny
    yaw = finite(planning_yaw_rad, "actual planning heading")
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    mount = iris_left_optical_to_body()
    half_forward = (abs(cosine)*dx+abs(sine)*dy)/2
    half_lateral = (abs(sine)*dx+abs(cosine)*dy)/2
    # Place the target near the near side, with room for the whole cell. This
    # selects a camera lookahead, never clips a vehicle goal to its permission.
    forward = footprint["near_forward_m"] + max(.2*footprint["length_m"], half_forward+.05*footprint["length_m"])
    if (forward+half_forward > footprint["far_forward_m"]-.05*footprint["length_m"]
            or half_lateral > .475*footprint["width_m"]):
        raise CoverageSearchError("Rotated target cells do not fit the conservative nominal camera footprint")
    offset = matvec([[cosine, -sine], [sine, cosine]], [mount["translation_m"][0]+forward, mount["translation_m"][1]])
    orientation = [0., 0., math.sin(yaw/2), math.cos(yaw/2)]
    views = []
    for row in range(ny):
        columns = range(nx) if row % 2 == 0 else range(nx-1, -1, -1)
        for column in columns:
            center = [bounds[0]+(column+.5)*dx, bounds[1]+(row+.5)*dy, ground]
            vehicle = [center[0]-offset[0], center[1]-offset[1], altitude]
            if (not allowed[0] <= vehicle[0] <= allowed[2] or not allowed[1] <= vehicle[1] <= allowed[3]
                    or math.hypot(vehicle[0], vehicle[1]) > radius):
                raise CoverageSearchError("Required camera observation vehicle point exceeds permitted ENU bounds/radius; cannot clip it")
            cell = [bounds[0]+column*dx, bounds[1]+row*dy, bounds[0]+(column+1)*dx, bounds[1]+(row+1)*dy]
            nominal_pose = {"frame_id": "map", "body_reference": "base_link", "position_m": vehicle,
                            "orientation_xyzw": orientation}
            projections = [project_ground_point(camera_info, nominal_pose, point, max_range_m)
                           for point in ground_cell_corners(cell, ground)]
            if not all(point["inside_fov_and_range"] for point in projections):
                raise CoverageSearchError("Target cell corners are outside the planned calibrated image/range")
            mount_x, mount_y = mount["translation_m"][:2]
            camera_xy = [vehicle[0]+cosine*mount_x-sine*mount_y, vehicle[1]+sine*mount_x+cosine*mount_y]
            polygon = [[camera_xy[0]+cosine*x-sine*y, camera_xy[1]+sine*x+cosine*y, ground]
                       for x, y in ((footprint["near_forward_m"], -footprint["width_m"]/2),
                                    (footprint["far_forward_m"], -footprint["width_m"]/2),
                                    (footprint["far_forward_m"], footprint["width_m"]/2),
                                    (footprint["near_forward_m"], footprint["width_m"]/2))]
            views.append({"id": "view_{:02d}".format(len(views)+1), "position_enu_m": vehicle,
                          "target_center_enu_m": center, "cell_bounds_enu_m": cell,
                          "target_cell_bounds_enu_m": list(cell), "sample_cell_area_m2": dx*dy,
                          "planned_pose_enu": nominal_pose, "planned_ground_footprint_enu_m": polygon,
                          "target_cell_corner_projections": projections, "target_cell_nominal_fov_verified": True})
    return {"schema_version": 2, "bounds_enu_m": bounds, "allowed_bounds_enu_m": allowed,
            "bounds_basis": "requested target search region", "allowed_bounds_basis": "vehicle goal permission",
            "usable_goal_bounds_enu_m": list(allowed), "navigation_permission": copy.deepcopy(navigation_permission),
            "area_m2": (bounds[2]-bounds[0])*(bounds[3]-bounds[1]), "spacing_m": spacing,
            "actual_grid_spacing_m": [dx, dy], "altitude_m": altitude,
            "ground_enu_z": ground, "max_range_m": max_range_m,
            "max_observations": budget, "max_duration_wall_s": duration,
            "camera_info": copy.deepcopy(camera_info), "nominal_footprint": footprint, "views": views,
            "nominal_heading_yaw_rad": yaw, "camera_mount": mount, "camera_lookahead_forward_m": forward,
            "flight_radius_m": radius,
            "scope": "deterministic view sampling, not full occlusion-free coverage",
            "generated_target_flight_goals": False}


def ground_cell_corners(cell, ground):
    return [[x, y, ground] for x, y in ((cell[0], cell[1]), (cell[2], cell[1]), (cell[2], cell[3]), (cell[0], cell[3]))]


def project_ground_point(camera_info, pose, ground_point, max_range_m):
    """Calibrated static optical/body transform and actual ENU attitude; no GT."""
    if pose.get("frame_id") not in ("map", "local_enu") or pose.get("body_reference") != "base_link":
        raise CoverageSearchError("Visibility projection requires an ENU base_link pose")
    inverse = transpose(quaternion_rotation(pose["orientation_xyzw"]))
    position = vector(pose["position_m"], 3, "camera observer position")
    point = vector(ground_point, 3, "target ground point")
    mount = iris_left_optical_to_body()
    body = matvec(inverse, [a-b for a, b in zip(point, position)])
    optical = matvec(transpose(mount["rotation"]), [a-b for a, b in zip(body, mount["translation_m"])])
    k = vector(camera_info["K"], 9, "CameraInfo K")
    pixel = [k[0]*optical[0]/optical[2]+k[2], k[4]*optical[1]/optical[2]+k[5]] if optical[2] > 0 else None
    return {"point_enu_m": point, "camera_optical_m": optical, "pixel_uv": pixel,
            "inside_fov_and_range": bool(pixel and 0 < optical[2] <= max_range_m and 0 <= pixel[0] < 640 and 0 <= pixel[1] < 480)}


def target_region_membership(target, bounds):
    """Configured 3-sigma envelope containment, not a calibrated probability."""
    point = vector(target["position_enu_m"], 3, "measured target region position")
    covariance = covariance_matrix(target["covariance_enu_m2"], "measured target region uncertainty")
    radius = [3*math.sqrt(covariance[i][i]) for i in range(2)]
    envelope = [point[0]-radius[0], point[1]-radius[1], point[0]+radius[0], point[1]+radius[1]]
    mean_inside = bounds[0] <= point[0] <= bounds[2] and bounds[1] <= point[1] <= bounds[3]
    confirmed = mean_inside and bounds[0] <= envelope[0] <= envelope[2] <= bounds[2] and bounds[1] <= envelope[1] <= envelope[3] <= bounds[3]
    return {"confirmed_inside": confirmed, "mean_inside": mean_inside,
            "status": "inside_with_configured_uncertainty_margin" if confirmed else "uncertain_at_region_boundary" if mean_inside else "outside_region",
            "xy_uncertainty_envelope_enu_m": envelope, "sigma_multiplier": 3,
            "probability_calibrated": False}


def _label(value):
    if not isinstance(value, str) or not 0 < len(value.strip()) <= 80:
        raise CoverageSearchError("Target label must be a short nonempty string")
    return value.strip()


def _hash(value):
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value):
        raise CoverageSearchError("Real image SHA256 provenance is required")
    return value


def _validated_coordinate_epoch(value):
    """Validate the canonical evidence identity; legacy scalar epochs are unsafe."""
    if value is None:
        return None
    if not isinstance(value, dict) or set(value) != {"vio_epoch", "ekf_counters"}:
        raise CoverageSearchError("Coordinate epoch must use canonical VIO and EKF reset evidence")
    epoch, counters = value["vio_epoch"], value["ekf_counters"]
    if epoch is not None and (type(epoch) is not int or epoch < 0):
        raise CoverageSearchError("VIO epoch must be an observed nonnegative integer")
    if counters is not None and (not isinstance(counters, dict) or set(counters) != {"xy", "z", "heading"}
            or any(type(item) is not int or not 0 <= item <= 255 for item in counters.values())):
        raise CoverageSearchError("EKF coordinate reset evidence must contain valid xy/z/heading counters")
    if epoch is None and counters is None:
        raise CoverageSearchError("Absent coordinate epoch evidence must be explicit None")
    return copy.deepcopy(value)


class TargetMemory:
    """Bounded multi-view mixture estimate without pretending shared errors average away."""
    def __init__(self, max_targets=24, max_observations_per_target=24, distinct_view_distance_m=.5):
        self.max_targets = _bounded_integer(max_targets, 1, 64, "target memory size")
        self.max_observations = _bounded_integer(max_observations_per_target, 2, 64, "target evidence size")
        self.distinct_distance = finite(distinct_view_distance_m, "distinct viewpoint distance")
        if not .25 <= self.distinct_distance <= 5:
            raise CoverageSearchError("Distinct viewpoint distance is outside its supported range")
        self.targets = []
        self.coordinate_epoch, self._epoch_set = None, False

    def observe(self, label, measurement, capture_id, observer_pose_enu):
        label = _label(label)
        if (measurement.get("status") != "ok" or measurement.get("method") != "stereo_sgbm_robust_surface"
                or measurement.get("frame") != "local_enu"):
            raise CoverageSearchError("Target memory accepts actual validated stereo surface measurements only")
        point = vector(measurement["position_enu_m"], 3, "measured ENU target")
        covariance = covariance_matrix(measurement["covariance_enu_m2"], "target covariance")
        stamp = finite(measurement["image_stamp_sim_s"], "target image stamp")
        hashes = {side: _hash(measurement["image_sha256"][side]) for side in ("left", "right")}
        epoch = _validated_coordinate_epoch(measurement.get("coordinate_epoch"))
        if self._epoch_set and epoch != self.coordinate_epoch:
            raise CoverageSearchError("Target memory cannot combine measurements from different coordinate epochs")
        if not isinstance(capture_id, str) or not 0 < len(capture_id) <= 80:
            raise CoverageSearchError("Actual capture ownership ID is required")
        if (observer_pose_enu.get("frame_id") not in ("map", "local_enu")
                or observer_pose_enu.get("body_reference") != "base_link"):
            raise CoverageSearchError("Target observation pose must be ENU base_link")
        pose_stamp = finite(observer_pose_enu["stamp_sim_s"], "observer pose stamp")
        if abs(pose_stamp-stamp) > .05:
            raise CoverageSearchError("Target observation pose does not belong to its image")
        observer = vector(observer_pose_enu["position_m"], 3, "observer ENU position")
        if math.dist(point, observer) > 20:
            raise CoverageSearchError("Target lies outside the stereo observation distance bound")
        candidates = []
        for target in self.targets:
            if target["label"].casefold() != label.casefold():
                continue
            same_frame = [item for item in target["observations"]
                          if item["capture_id"] == capture_id or item["image_sha256"] == hashes]
            if same_frame:
                if any(math.dist(item["position_enu_m"], point) < .1 for item in same_frame):
                    return {"accepted": False, "reason": "duplicate_capture", "target_id": target["id"]}
                # Two different objects with one label can appear in the same image.
                # A new same-frame surface point must not be merged as another view.
                continue
            variance = sum(target["covariance_enu_m2"][i][i] + covariance[i][i] for i in range(3))
            distance = math.dist(target["position_enu_m"], point)
            if distance <= min(2.5, max(.6, 3*math.sqrt(variance))):
                candidates.append((distance, target))
        if candidates:
            target = min(candidates, key=lambda item: (item[0], item[1]["id"]))[1]
        else:
            if len(self.targets) >= self.max_targets:
                raise CoverageSearchError("Target memory is full; no target evidence was silently discarded")
            target = {"id": "target_{:02d}".format(len(self.targets)+1), "label": label, "observations": [],
                      "coordinate_epoch": copy.deepcopy(epoch)}
            self.targets.append(target)
        if len(target["observations"]) >= self.max_observations:
            return {"accepted": False, "reason": "target_evidence_budget_exhausted", "target_id": target["id"]}
        target["observations"].append({"capture_id": capture_id, "image_sha256": hashes,
                                      "image_stamp_sim_s": stamp, "position_enu_m": point,
                                      "covariance_enu_m2": covariance, "observer_position_enu_m": observer,
                                      "coordinate_epoch": copy.deepcopy(epoch)})
        self.coordinate_epoch, self._epoch_set = epoch, True
        evidence = target["observations"]
        weights = [1/max(1e-6, sum(item["covariance_enu_m2"][i][i] for i in range(3))) for item in evidence]
        total = sum(weights)
        mean = [sum(weight*item["position_enu_m"][i] for weight, item in zip(weights, evidence))/total for i in range(3)]
        fused_cov = [[sum(weight*(item["covariance_enu_m2"][i][j]
                                  + (item["position_enu_m"][i]-mean[i])*(item["position_enu_m"][j]-mean[j]))
                         for weight, item in zip(weights, evidence))/total for j in range(3)] for i in range(3)]
        distinct = []
        for item in evidence:
            if all(math.dist(item["observer_position_enu_m"], prior) >= self.distinct_distance for prior in distinct):
                distinct.append(item["observer_position_enu_m"])
        target.update(position_enu_m=mean, covariance_enu_m2=fused_cov,
                      sigma_enu_m=[math.sqrt(fused_cov[i][i]) for i in range(3)],
                      distinct_viewpoint_count=len(distinct), measurement_count=len(evidence),
                      estimation_method="uncertainty-weighted mixture including between-view scatter",
                      error_statistics_calibrated=False, generated_flight_goal=False)
        return {"accepted": True, "target_id": target["id"], "distinct_viewpoint_count": len(distinct)}

    def reports(self):
        return copy.deepcopy(self.targets)


def potential_visible_samples(plan, pose):
    """Whole target cells within actual image/range projection, without occlusion."""
    result = []
    for view in plan["views"]:
        corners = ground_cell_corners(view["target_cell_bounds_enu_m"], plan["ground_enu_z"])
        if all(project_ground_point(plan["camera_info"], pose, point, plan["max_range_m"])["inside_fov_and_range"]
               for point in corners):
            result.append(view["id"])
    return result


class CoverageSearch:
    def __init__(self, plan, target_memory=None, target_label=None, required_distinct_views=2, clock=time.monotonic):
        self.plan = copy.deepcopy(plan)
        self.memory = target_memory if target_memory is not None else TargetMemory()
        self.target_label = _label(target_label) if target_label is not None else None
        self.required_views = _bounded_integer(required_distinct_views, 1, 8, "required distinct views")
        self.clock, self.started = clock, clock()
        self.status, self.reason = "running", None
        self.states = {view["id"]: "pending" for view in plan["views"]}
        self.attempts, self.observations, self.potential_cells = 0, [], set()
        self.coordinate_epoch, self._epoch_set = None, False

    def _check_budget(self):
        if self.status == "running" and self.clock()-self.started >= self.plan["max_duration_wall_s"]:
            self.status, self.reason = "budget_exhausted", "wall_time_budget"

    def next_view(self):
        self._check_budget()
        if self.status != "running":
            return None
        return next((copy.deepcopy(view) for view in self.plan["views"]
                     if self.states[view["id"]] in ("pending", "arrived")), None)

    def _require_current(self, view_id):
        current = self.next_view()
        if current is None or current["id"] != view_id:
            raise CoverageSearchError("Only the current unfinished observation point may be updated")
        return current

    def record_navigation_result(self, view_id, mission_status, body_pose_enu):
        current = self._require_current(view_id)
        if mission_status != "succeeded":
            self.states[view_id] = "blocked"
            self.status, self.reason = "blocked", "navigation_did_not_succeed"
            return False
        position = vector(body_pose_enu["position_m"], 3, "arrived ENU pose")
        if (body_pose_enu.get("frame_id") not in ("map", "local_enu")
                or body_pose_enu.get("body_reference") != "base_link"
                or math.dist(position[:2], current["position_enu_m"][:2]) > .65
                or abs(position[2]-self.plan["altitude_m"]) > .3):
            self.status, self.reason = "blocked", "navigation_arrival_not_observed"
            self.states[view_id] = "blocked"
            return False
        self.states[view_id] = "arrived"
        return True

    def record_observation(self, view_id, capture, stereo_targets=None):
        current = self._require_current(view_id)
        if self.states[view_id] != "arrived":
            raise CoverageSearchError("A successful actual arrival is required before counting an observation")
        self.attempts += 1
        try:
            if self.attempts > self.plan["max_observations"]:
                raise CoverageSearchError("observation budget exhausted")
            identifier = capture["capture_id"]
            if not isinstance(identifier, str) or not 0 < len(identifier) <= 80:
                raise CoverageSearchError("Real capture ID is required")
            hashes = {side: _hash(capture["image_sha256"][side]) for side in ("left", "right")}
            epoch = _validated_coordinate_epoch(capture.get("coordinate_epoch"))
            if self._epoch_set and epoch != self.coordinate_epoch:
                raise CoverageSearchError("Search observations cannot cross a coordinate epoch change")
            if any(item["capture_id"] == identifier or item["image_sha256"] == hashes for item in self.observations):
                raise CoverageSearchError("The current view requires a new actual stereo capture")
            stamp = finite(capture["stamp_sim_s"], "capture stamp")
            if (not 0 <= finite(capture["age_seconds"], "capture age") <= 2
                    or not 0 <= finite(capture["measurement_age_seconds"], "capture source age") <= .5):
                raise CoverageSearchError("Image capture is stale or future-dated")
            pose = capture["body_pose_enu"]
            if abs(finite(pose["stamp_sim_s"], "capture pose stamp")-stamp) > .05:
                raise CoverageSearchError("Capture pose does not belong to the image")
            if not 0 <= finite(pose["age_seconds"], "capture pose receipt age") <= 1:
                raise CoverageSearchError("Capture pose receipt is stale")
            position = vector(pose["position_m"], 3, "capture ENU position")
            if (math.dist(position[:2], current["position_enu_m"][:2]) > .65
                    or abs(position[2]-self.plan["altitude_m"]) > .3):
                raise CoverageSearchError("Clear image was not captured at the observation point")
            allowed = self.plan["allowed_bounds_enu_m"]
            if (not allowed[0] <= position[0] <= allowed[2] or not allowed[1] <= position[1] <= allowed[3]
                    or math.hypot(position[0], position[1]) > self.plan["flight_radius_m"]):
                raise CoverageSearchError("Actual camera observer lies outside its vehicle flight permission")
            quality = capture["image_quality"]
            if (finite(quality["laplacian_variance"], "image sharpness") < 20
                    or not 10 <= finite(quality["mean_luma"], "mean luma") <= 245
                    or not 0 <= finite(quality["saturated_fraction"], "saturated fraction") <= .8):
                raise CoverageSearchError("Image quality does not support a clear observation")
            visible = potential_visible_samples(self.plan, pose)
            if view_id not in visible:
                raise CoverageSearchError("Actual image attitude/range does not show the current requested target cell")
            target_records = []
            # A report is one observation: reject the entire target batch if a
            # later measurement fails rather than retaining partial evidence.
            pending_memory = copy.deepcopy(self.memory)
            for target in stereo_targets or []:
                measurement = target["measurement"]
                if (measurement["image_sha256"] != hashes
                        or abs(measurement["image_stamp_sim_s"]-stamp) > 1e-6
                        or _validated_coordinate_epoch(measurement.get("coordinate_epoch")) != epoch):
                    raise CoverageSearchError("Target stereo measurement belongs to another capture")
                target_records.append(pending_memory.observe(target["label"], measurement, identifier, pose))
        except (KeyError, TypeError, ValueError) as exc:
            self.states[view_id] = "observation_failed"
            self.status, self.reason = "observation_failed", str(exc)
            return {"accepted": False, "reason": self.reason}
        self.states[view_id] = "observed"
        self.coordinate_epoch, self._epoch_set = epoch, True
        self.memory.targets = pending_memory.targets
        self.memory.coordinate_epoch, self.memory._epoch_set = pending_memory.coordinate_epoch, pending_memory._epoch_set
        self.potential_cells.update(visible)
        self.observations.append({"view_id": view_id, "capture_id": identifier, "image_sha256": hashes,
                                  "image_stamp_sim_s": stamp, "observer_position_enu_m": position,
                                  "observer_body_pose_enu": copy.deepcopy(pose),
                                  "coordinate_epoch": copy.deepcopy(epoch),
                                  "potential_visible_sample_ids": visible, "targets": target_records})
        if self.target_label and any(target["label"].casefold() == self.target_label.casefold()
                                     and target["distinct_viewpoint_count"] >= self.required_views
                                     and target_region_membership(target, self.plan["bounds_enu_m"])["confirmed_inside"]
                                     for target in self.memory.targets):
            self.status, self.reason = "target_found", "measured_target_from_required_distinct_viewpoints"
        elif all(status == "observed" for status in self.states.values()):
            self.status, self.reason = "sampling_complete", "all_planned_view_samples_observed"
        return {"accepted": True, "status": self.status, "targets": target_records}

    def cancel(self):
        if self.status == "running":
            self.status, self.reason = "cancelled", "operator_cancelled"

    def summary(self):
        self._check_budget()
        observed = [view for view in self.plan["views"] if self.states[view["id"]] == "observed"]
        potential = [view for view in self.plan["views"] if view["id"] in self.potential_cells]
        return {"status": self.status, "reason": self.reason, "planned_views": len(self.states),
                "observed_views": len(observed), "view_sampling_fraction": len(observed)/len(self.states),
                "sampled_region_area_m2": sum(view["sample_cell_area_m2"] for view in observed),
                "potential_geometric_view_area_m2": sum(view["sample_cell_area_m2"] for view in potential),
                "occlusion_free_coverage_proven": False,
                "coverage_basis": "fresh clear images geometrically showing requested target-cell corners; occlusion is untested",
                "target_identity_ground_truth_verified": False,
                "target_association_basis": "same-label spatial stereo consistency; object identity is not proven",
                "states": copy.deepcopy(self.states), "observation_attempts": self.attempts,
                "observations": copy.deepcopy(self.observations), "targets": [
                    {**target, "requested_region_membership": target_region_membership(target, self.plan["bounds_enu_m"])}
                    for target in self.memory.reports()],
                "generated_target_flight_goals": False}
