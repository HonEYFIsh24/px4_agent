#!/usr/bin/env python3
"""One bounded real single-VLM stereo search; default is a request-free plan.

Only --execute submits the flight/search/return/landing task. A new exclusive
evidence directory preserves the first outcome, with no task or model retries.
The current web API uses /api/commands (there is no /tasks endpoint).
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import time
from urllib.parse import urlsplit
import uuid

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from search_tools import coordinate_epoch
from coverage_search import bounds_vector, navigation_goal_permission, potential_visible_samples, target_region_membership
from stereo_localization import covariance_matrix, finite, vector
from scripts.test_realistic_scene import (check_start, collect_perceptions, distance,
                                         request, verify_return_home)

PROMPT = "起飞3.5米，在ENU x[3,5] y[0,2]区域按1米网格寻找蓝色桶，最多4观察点，逐点采图报告，最后返航降落。可使用目标区域外但仍在已验证ENU飞行许可内的观察点，使前视相机看见目标区域。未确认目标时完成观察预算，明确报告未确认。"
BOUNDS, ALTITUDE, MAX_VIEWS = [3., 0., 5., 2.], 3.5, 4


class SimulationOwnershipError(RuntimeError):
    pass


class LocalSimulationOwner:
    """Pin the existing manager and its one PX4 descendant; never start either."""
    def __init__(self, base, px4_directory=None):
        from scripts.run_reliability_suite import SystemOperations, owned_px4_process, process_identity
        from scripts.record_ev_fusion import stable_px4_identity
        endpoint = urlsplit(base)
        if (endpoint.scheme != "http" or endpoint.hostname not in ("localhost", "127.0.0.1", "::1")
                or endpoint.username is not None or endpoint.password is not None
                or endpoint.path not in ("", "/") or endpoint.query or endpoint.fragment):
            raise SimulationOwnershipError("Search acceptance requires the owned local simulation web endpoint")
        self.operations = SystemOperations()
        self.operations.px4_directory = Path(px4_directory or os.environ.get("PX4_DIR") or
                                             "/home/dut/PX4-Autopilot").resolve()
        self.manager = self.operations.current_manager()
        if self.manager is None:
            raise SimulationOwnershipError("No live owned project manager; this runner never starts one")
        self.binary = self.operations.px4_directory / "build/px4_sitl_default/bin/px4-listener"
        self._owned_px4, self._process_identity, self._stable = owned_px4_process, process_identity, stable_px4_identity
        self.px4 = self._stable(self._owned_px4(self.manager, self.binary))
        self.check_count = 0
        self.check()

    def check(self):
        self.check_count += 1
        try:
            if (self.operations.current_manager() != self.manager
                    or self._process_identity(self.manager["run_directory"]) != self.manager
                    or self._stable(self._owned_px4(self.manager, self.binary)) != self.px4):
                raise SimulationOwnershipError("Pinned simulation manager or PX4 identity changed")
        except SimulationOwnershipError:
            raise
        except Exception as exc:
            raise SimulationOwnershipError("Pinned simulation ownership can no longer be verified") from exc

    def evidence(self):
        return {"manager": copy.deepcopy(self.manager), "px4_process": copy.deepcopy(self.px4),
                "identity_check_count": self.check_count, "source": "actual_owned_local_process_identity"}


def task_missions(command):
    """Accepted flight audit is attribution; sampled telemetry alone is not."""
    result = {}
    if not isinstance(command, dict):
        return result
    candidates = []
    for field in ("missions", "flight_missions"):
        value = command.get(field)
        if isinstance(value, list):
            candidates.extend(value)
    candidates.extend([command.get("mission"), (command.get("safety_landing") or {}).get("mission")])
    for candidate in candidates:
        if isinstance(candidate, dict) and isinstance(candidate.get("mission"), dict):
            candidate = candidate["mission"]
        if (isinstance(candidate, dict) and isinstance(candidate.get("id"), str) and candidate["id"]
                and isinstance(candidate.get("steps"), list) and candidate["steps"]):
            result[candidate["id"]] = copy.deepcopy(candidate)
    return result


def recovery_ownership(state, command, accepted_id, submitted_prompt):
    """Prove an accepted flight, or an already running child land to wait for."""
    if (not isinstance(command, dict) or command.get("id") != accepted_id
            or command.get("prompt") != submitted_prompt or command.get("mode") != "task"):
        return False, "Current task audit does not prove the submitted command"
    if state.get("active_command_id") not in (None, accepted_id):
        return False, "Another operator command is active"
    mission = state.get("mission") or {}
    accepted_missions = task_missions(command)
    accepted = accepted_missions.get(mission.get("id"))
    if accepted is None or accepted.get("steps") != mission.get("steps"):
        parent = mission.get("parent_mission_id")
        if (isinstance(parent, str) and parent != mission.get("id") and parent in accepted_missions
                and mission.get("status") == "running" and mission.get("steps") == [{"action": "land"}]):
            # A supervisor's actual parent-child record permits waiting only.
            # This child ID is not used for POST /api/land: the web boundary
            # requires the command's own accepted flight_missions membership.
            return True, "Wait for running child landing with an accepted task-owned parent; no new land request"
        return False, "Current Bridge mission is absent from this command's accepted flight audit"
    return True, "Current Bridge mission matches this submitted command's accepted flight audit"


RECOVERY_TIMEOUT_WALL_S = 180.
MAXIMUM_FCU_RECEIPT_AGE_WALL_S = 3.


def landing_recovery_confirmed(state, landing_id):
    """Confirm the attributed land using current actual FCU receipts, never XY.

    The caller must first attribute landing_id to its accepted mission audit or
    guarded child-land acknowledgment. A historically succeeded mission alone
    cannot prove that the currently sampled heartbeat/ground state is fresh.
    """
    if not isinstance(state, dict):
        return False
    mission = state.get("mission") or {}
    actual = state.get("landing_readiness") or {}
    if (not isinstance(mission, dict) or not isinstance(actual, dict)
            or not isinstance(landing_id, str) or not landing_id
            or mission.get("id") != landing_id or mission.get("status") != "succeeded"
            or mission.get("steps") != [{"action": "land"}]
            or state.get("armed") is not False or type(state.get("landed_state")) is not int
            or state.get("landed_state") != 1
            or actual.get("ready") is not True
            or actual.get("source") != "actual_MAVROS_State_and_ExtendedState_receipts"
            or actual.get("connected") is not True or actual.get("armed") is not False
            or type(actual.get("landed_state")) is not int or actual.get("landed_state") != 1
            or not isinstance(actual.get("mode"), str) or not 1 <= len(actual["mode"]) <= 128):
        return False
    return all(type(actual.get(field)) in (int, float) and math.isfinite(actual[field])
               and 0 <= actual[field] <= MAXIMUM_FCU_RECEIPT_AGE_WALL_S
               for field in ("heartbeat_age_wall_s", "extended_state_age_wall_s"))


def write_once(path, value):
    with path.open("x") as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def near_point(candidate, reference, tolerance):
    try:
        return distance(vector(candidate, 3, "actual point"), vector(reference, 3, "reference point")) <= tolerance
    except (TypeError, ValueError):
        return False


def check_fusion_start(state, model):
    check_start(state, model, True)
    require(state.get("simulation_only") is True, "This acceptance requires verified simulation")
    ev = state.get("external_vision") or {}
    adapter, ekf = ev.get("adapter") or {}, ev.get("ekf") or {}
    epoch = coordinate_epoch(state)
    require(ev.get("required") is True and ev.get("healthy") is True
            and ev.get("used_for_flight_control") is True,
            "Actual healthy EV fusion must pass before this search is submitted")
    require(adapter.get("mode") == "fuse" and adapter.get("forwarding") is True
            and adapter.get("ready") is True and adapter.get("sitl_verified") is True,
            "A forwarding, verified SITL EV adapter is required")
    require(ekf.get("ready") is True and ekf.get("fusion_active") is True,
            "Selected actual PX4 EKF fusion evidence is required")
    require(epoch is not None and epoch["vio_epoch"] is not None and epoch["ekf_counters"] is not None,
            "Complete canonical VIO and actual EKF reset counters are required")


def check_live_plan(actual_plan):
    """Stop an out-of-scope plan as soon as its public audit is received."""
    require(actual_plan.get("status") == "ok" and isinstance(actual_plan.get("search_id"), str),
            "Public successful search_plan evidence is absent; update the web process")
    plan = actual_plan.get("plan") or {}
    require(vector(plan.get("bounds_enu_m"), 4, "actual plan bounds") == BOUNDS,
            "Actual plan does not use the user-requested ENU region")
    require(finite(plan.get("altitude_m"), "actual plan altitude") == ALTITUDE,
            "Actual search altitude differs from 3.5m")
    views, budget = plan.get("views") or [], plan.get("max_observations")
    require(type(budget) is int and 1 <= len(views) <= budget <= MAX_VIEWS,
            "Actual search plan exceeds the four-view budget")
    allowed = bounds_vector(plan.get("allowed_bounds_enu_m"))
    permission = plan.get("navigation_permission") or {}
    canonical = navigation_goal_permission(permission.get("navigation_map_bounds_enu_m"),
                                            permission.get("obstacle_inflation_m"), permission.get("visible_clearance_m"))
    require(canonical == permission and allowed == canonical["usable_goal_bounds_enu_m"]
            and plan.get("usable_goal_bounds_enu_m") == allowed,
            "Actual vehicle goal permission omitted or duplicated navigation boundary clearance")
    radius = finite(plan.get("flight_radius_m"), "actual vehicle radius")
    require(0 < radius <= 30, "Actual vehicle radius is outside the supported permission")
    view_ids = set()
    for view in views:
        require(isinstance(view, dict) and isinstance(view.get("id"), str) and view["id"] not in view_ids,
                "Actual plan contains an invalid or duplicate view ID")
        view_ids.add(view["id"])
        point = vector(view.get("position_enu_m"), 3, "actual planned point")
        require(allowed[0] <= point[0] <= allowed[2] and allowed[1] <= point[1] <= allowed[3]
                and math.hypot(point[0], point[1]) <= radius and point[2] == ALTITUDE,
                "Actual vehicle observation point is outside flight permission or altitude")
        cell = bounds_vector(view.get("target_cell_bounds_enu_m"))
        require(BOUNDS[0] <= cell[0] < cell[2] <= BOUNDS[2] and BOUNDS[1] <= cell[1] < cell[3] <= BOUNDS[3],
                "Target sample cell is outside the requested search region")
        center = vector(view.get("target_center_enu_m"), 3, "actual target cell center")
        require(near_point(center, [(cell[0]+cell[2])/2, (cell[1]+cell[3])/2, plan["ground_enu_z"]], 1e-7),
                "Target sample center does not represent its requested cell")
        nominal_pose = view.get("planned_pose_enu") or {}
        require(near_point(nominal_pose.get("position_m"), point, 1e-7)
                and view["id"] in potential_visible_samples(plan, nominal_pose),
                "Calibrated nominal camera does not actually show its target cell")
        require(len(view.get("planned_ground_footprint_enu_m") or []) == 4
                and all(len(vector(p, 3, "nominal footprint corner")) == 3
                        for p in view["planned_ground_footprint_enu_m"]), "Nominal footprint evidence is absent")
    require(plan.get("generated_target_flight_goals") is False,
            "Target estimates must never become flight goals")
    return plan


def audit_search(command, initial, final, samples, missions, model):
    """Validate actual search/bookkeeping evidence without presuming detection."""
    require(command.get("status") == "succeeded", command.get("error") or "Task did not succeed")
    require(final.get("armed") is False and final.get("landed_state") == 1,
            "Task did not finish landed and disarmed")
    require("image_base64" not in json.dumps(command), "Public task leaked image bytes")
    calls = command.get("model_calls") or []
    require(calls and all(c.get("model") == model and c.get("status") == "ok" for c in calls),
            "A real same-VLM call is missing or failed; no retries are permitted")
    sequence = [t.get("name") for c in calls for t in c.get("called_tools", [])]
    require(all(name in sequence for name in ("get_drone_state", "plan_region_search", "capture_scene", "report_scene")),
            "Required actual state/search/capture/report tool calls were not recorded")
    actual_plan = command.get("search_plan") or {}
    plan = check_live_plan(actual_plan)
    views = plan.get("views") or []
    progress = [event["result"] for event in command.get("search_events", [])
                if event.get("tool") == "search_progress" and isinstance(event.get("result"), dict)
                and event["result"].get("search_id") == actual_plan["search_id"]]
    require(progress and all(p.get("status") == "ok" for p in progress),
            "Successful actual saved-report search progress is missing")
    summary = progress[-1].get("search") or {}
    require(summary.get("status") in ("sampling_complete", "target_found"),
            "Search did not reach an evidence-supported terminal state")
    observations = summary.get("observations") or []
    require(1 <= len(observations) <= MAX_VIEWS and summary.get("observed_views") == len(observations)
            and summary.get("planned_views") == len(views), "Actual observation count is invalid")
    require([o.get("view_id") for o in observations] == [v["id"] for v in views[:len(observations)]],
            "Search skipped or reordered an unfinished observation point")
    require(all((p.get("search") or {}).get("occlusion_free_coverage_proven") is False for p in progress),
            "The search incorrectly claims complete occlusion-free coverage")
    require(summary.get("generated_target_flight_goals") is False,
            "Measured targets were used as flight goals")
    if summary["status"] == "target_found":
        require(any(t.get("distinct_viewpoint_count", 0) >= 2 and t.get("label", "").casefold()
                    == actual_plan.get("query", "").casefold()
                    and (t.get("requested_region_membership") or {}).get("confirmed_inside") is True
                    and target_region_membership(t, BOUNDS)["confirmed_inside"]
                    for t in summary.get("targets", [])),
                "Early target-found termination lacks two views and target-region uncertainty containment")
    else:
        require(len(observations) == len(views), "Sampling completed before all planned views were observed")
    reports = {}
    collect_perceptions(command, reports)
    require(set(reports) == {o["capture_id"] for o in observations},
            "Saved reports do not match the actual counted observations")
    require(sequence.count("capture_scene") == len(observations)
            and sequence.count("report_scene") == len(observations),
            "Capture/report tool counts differ from observed views")
    new_missions = [m for m in missions.values() if m.get("id") != (initial.get("mission") or {}).get("id")]
    require(new_missions and all(m.get("status") == "succeeded" for m in new_missions), "A real flight stage failed")
    events = [e for m in new_missions for e in m.get("events", [])]
    prior_epoch, geometry = None, []
    for index, observation in enumerate(observations):
        report = reports[observation["capture_id"]]
        require(report.get("status") == "ok" and report.get("architecture") == "single_vlm"
                and report.get("model") == model, "Saved report failed or used a different model")
        camera = report.get("camera") or {}
        require((camera.get("width"), camera.get("height")) == (640, 480), "Unexpected actual camera size")
        hashes = observation.get("image_sha256") or {}
        require(all(re.fullmatch(r"[0-9a-f]{64}", hashes.get(side, "")) for side in ("left", "right"))
                and camera.get("image_sha256") == hashes["left"], "Same-frame stereo JPEG hashes are missing or mismatched")
        require(any(report["capture_id"] in c.get("capture_ids", []) and "image" in c.get("input_modalities", [])
                    and any(t.get("name") == "report_scene" for t in c.get("called_tools", [])) for c in calls),
                "This saved report was not informed by its real model-input image")
        stamp = finite(report.get("captured_at_sim_s"), "saved image stamp")
        require(abs(stamp-finite(observation.get("image_stamp_sim_s"), "observed image stamp")) <= 1e-6,
                "Counted observation and saved report have different source stamps")
        epoch = observation.get("coordinate_epoch")
        require(isinstance(epoch, dict) and set(epoch) == {"vio_epoch", "ekf_counters"}
                and type(epoch.get("vio_epoch")) is int and epoch["vio_epoch"] >= 0
                and isinstance(epoch.get("ekf_counters"), dict)
                and set(epoch["ekf_counters"]) == {"xy", "z", "heading"}
                and all(type(v) is int and 0 <= v <= 255 for v in epoch["ekf_counters"].values()),
                "Counted observation lacks complete canonical epoch evidence")
        require(index == 0 or epoch == prior_epoch, "Search mixed local coordinate epochs")
        prior_epoch = copy.deepcopy(epoch)
        position = vector(observation.get("observer_position_enu_m"), 3, "actual observer position")
        require(distance(position, views[index]["position_enu_m"]) <= .75,
                "Image was acquired away from its actual planned point")
        observer_pose = observation.get("observer_body_pose_enu") or {}
        allowed = plan["allowed_bounds_enu_m"]
        require(near_point(observer_pose.get("position_m"), position, 1e-7)
                and abs(finite(observer_pose.get("stamp_sim_s"), "observer pose stamp")-stamp) <= .05
                and allowed[0] <= position[0] <= allowed[2] and allowed[1] <= position[1] <= allowed[3]
                and math.hypot(position[0], position[1]) <= plan["flight_radius_m"]
                and observation["view_id"] in potential_visible_samples(plan, observer_pose),
                "Actual captured attitude or vehicle permission does not show the requested target cell")
        require(any(event.get("action") == "goto" and event.get("finished_at", math.inf) <= report["created_at"]+.1
                    and near_point(event.get("position_enu_m"), views[index]["position_enu_m"], .65)
                    for event in events), "Image precedes an actual successful goto to its planned point")
        if index:
            previous = reports[observations[index-1]["capture_id"]]
            require(any(m.get("started_at", 0) >= previous["finished_at"]-.2 and any(
                        step.get("action") == "goto" and near_point([step.get("x"), step.get("y"), step.get("altitude")],
                                                                     views[index]["position_enu_m"], .02)
                        for step in m.get("steps", [])) for m in new_missions),
                    "A later observation flight started before the prior report was saved")
        require(samples, "Airborne trajectory evidence is absent")
        nearest = min(samples, key=lambda s: abs(s["wall_time"]-report["created_at"]))
        require(abs(nearest["wall_time"]-report["created_at"]) <= 1.5
                and nearest["state"].get("armed") is True
                and distance(nearest["state"]["position_enu_m"], position) <= .85,
                "No aligned actual airborne telemetry supports this observation")
        for target in report.get("targets", []):
            metric = target.get("stereo_location") or {}
            require(metric.get("status") in ("ok", "unavailable"), "Target has no actual stereo result")
            require(isinstance(metric.get("capture_pair_id"), str)
                    and abs(finite(metric.get("image_stamp_sim_s"), "metric image stamp")-stamp) <= 1e-6,
                    "Metric provenance does not belong to this historical image pair")
            if metric["status"] == "unavailable":
                require(bool(metric.get("reason")), "Unavailable stereo estimate lacks its quality reason")
                require("position_enu_m" not in metric, "Unavailable geometry must not contain fabricated coordinates")
                geometry.append({"capture_id": report["capture_id"], "label": target.get("label"),
                                 "status": "unavailable", "reason": metric["reason"]})
                continue
            require(metric.get("historical_observation") is True
                    and metric.get("usable_as_flight_waypoint") is False
                    and metric.get("uncertainty_statistics_calibrated") is False
                    and metric.get("method") == "stereo_sgbm_robust_surface"
                    and metric.get("frame") == "local_enu" and metric.get("coordinate_epoch") == epoch
                    and metric.get("image_sha256") == hashes,
                    "Metric is not a validated same-frame historical stereo surface estimate")
            covariance = covariance_matrix(metric.get("covariance_enu_m2"), "actual ENU uncertainty")
            sigma = vector(metric.get("sigma_enu_m"), 3, "actual ENU sigma")
            require(all(abs(sigma[i]**2-covariance[i][i]) <= 1e-7 for i in range(3)), "Metric uncertainty is inconsistent")
            geometry.append({"capture_id": report["capture_id"], "label": target.get("label"), "status": "ok",
                             "position_enu_m": vector(metric.get("position_enu_m"), 3, "actual measured point"),
                             "covariance_enu_m2": covariance, "sigma_enu_m": sigma})
    require(sum(e.get("action") == "takeoff" for e in events) == 1
            and sum(e.get("action") == "land" for e in events) == 1,
            "Actual takeoff/landing completion evidence is missing")
    home = vector(initial.get("position_enu_m"), 3, "initial recorded home")
    returned = verify_return_home(new_missions, list(reports.values()), samples, home)
    home_steps = {(r["mission_id"], r["step"]) for r in returned if r["method"] == "goto_recorded_home"}
    for mission in new_missions:
        for index, step in enumerate(mission.get("steps", []), 1):
            if step.get("action") == "takeoff":
                require(finite(step.get("altitude"), "takeoff altitude") == ALTITUDE, "Takeoff differs from 3.5m")
            if step.get("action") == "goto" and (mission["id"], index) not in home_steps:
                point = [step.get("x"), step.get("y"), step.get("altitude")]
                require(any(near_point(point, v["position_enu_m"], .02) for v in views),
                        "Flight used an unapproved search or measured-target waypoint")
    require(math.hypot(final["position_enu_m"][0]-home[0], final["position_enu_m"][1]-home[1]) < .6,
            "Landing was not near the recorded home")
    return {"passed": True, "search_id": actual_plan["search_id"], "search_status": summary["status"],
            "planned_views": len(views), "actual_observed_views": len(observations),
            "counted_capture_ids": list(reports), "same_frame_stereo_acquisition_checked": True,
            "metric_results": geometry, "metric_available_count": sum(g["status"] == "ok" for g in geometry),
            "target_detection_ground_truth_verified": False, "target_required_to_be_visible": False,
            "occlusion_free_coverage_proven": False, "returned_via": returned,
            "tool_sequence": sequence, "search_summary": summary,
            "limits": ["Target absence and unavailable metric localization are valid outcomes.",
                       "View sampling is not full coverage or a proof of target absence.",
                       "Approximate surface uncertainty is not statistically calibrated; target points are report-only."]}


def execute(base, output, model, timeout, interval, transport=request, clock=time.monotonic, sleep=time.sleep,
            ownership_factory=None, px4_directory=None, task_prompt=None, task_validator=None,
            live_validator=None, max_reports=MAX_VIEWS):
    # Other owned acceptance runners may reuse identity/evidence/recovery. The
    # default search contract remains fixed; no CLI accepts arbitrary code.
    prompt = PROMPT if task_prompt is None else task_prompt
    require(isinstance(prompt, str) and 0 < len(prompt.strip()) <= 1900, "Invalid bounded acceptance prompt")
    require(type(max_reports) is int and 1 <= max_reports <= MAX_VIEWS, "Invalid bounded report budget")
    require(task_validator is None or callable(task_validator), "Invalid task audit callback")
    require(live_validator is None or callable(live_validator), "Invalid live audit callback")
    validator = audit_search if task_validator is None else task_validator
    output.mkdir(parents=True, exist_ok=False)
    report = {"prompt": prompt, "started_at": time.time(), "actual_task_submitted": False,
              "actual_task_submission_attempted": False, "actual_model_call": False,
              "actual_flight_authorized": True, "task_retries": 0}
    samples, perceptions, missions, command = [], {}, {}, None
    accepted_id, last_hash, guard = None, None, None
    # An exact unique prompt allows attribution even when the HTTP submission
    # acknowledgment is lost. It adds no flight or perception instructions.
    submitted_prompt = prompt + "\n本次验收标识：" + uuid.uuid4().hex
    report["submitted_prompt"] = submitted_prompt
    with (output / "state_samples.jsonl").open("x") as state_log, \
            (output / "task_updates.jsonl").open("x") as task_log, \
            (output / "trajectory.jsonl").open("x") as trajectory_log:
        def guarded_request(path, payload=None):
            guard.check()
            result = transport(base, path, payload)
            guard.check()
            return result

        def sample_state():
            state = guarded_request("/api/state")
            sample = {"wall_time": time.time(), "monotonic_s": clock(), "state": state}
            samples.append(sample)
            state_log.write(json.dumps(sample, ensure_ascii=False, allow_nan=False)+"\n"); state_log.flush()
            trajectory_log.write(json.dumps({"wall_time": sample["wall_time"], "monotonic_s": sample["monotonic_s"],
                "position_enu_m": state.get("position_enu_m"), "armed": state.get("armed"),
                "landed_state": state.get("landed_state"), "coordinate_epoch": coordinate_epoch(state),
                "mission_id": (state.get("mission") or {}).get("id"),
                "mission_status": (state.get("mission") or {}).get("status")}, ensure_ascii=False)+"\n"); trajectory_log.flush()
            mission = state.get("mission") or {}
            if mission.get("id"):
                missions[mission["id"]] = copy.deepcopy(mission)
            return state

        def sample_command(identifier):
            nonlocal last_hash
            current = guarded_request("/api/commands/"+identifier)
            require(isinstance(current, dict) and current.get("id") == identifier
                    and current.get("prompt") == submitted_prompt and current.get("mode") == "task",
                    "Public task audit differs from the exact submitted command")
            encoded = json.dumps(current, ensure_ascii=False, sort_keys=True, allow_nan=False)
            current_hash = hashlib.sha256(encoded.encode()).hexdigest()
            if current_hash != last_hash:
                task_log.write(json.dumps({"wall_time": time.time(), "task": current}, ensure_ascii=False)+"\n")
                task_log.flush(); last_hash = current_hash
            collect_perceptions(current, perceptions)
            return current

        try:
            guard = (ownership_factory or LocalSimulationOwner)(base, px4_directory)
            report["simulation_owner"] = guard.evidence()
            initial = sample_state()
            write_once(output / "initial_state.json", initial)
            check_fusion_start(initial, model)
            payload = {"prompt": submitted_prompt, "mode": "task", "min_observations": 1}
            write_once(output / "submitted_task.json", payload)
            report["submission_started_at_wall_s"] = time.time()
            guard.check()
            report["actual_task_submission_attempted"] = True
            accepted = transport(base, "/api/commands", payload)
            write_once(output / "accepted_task.json", accepted)
            identifier = accepted.get("id") if isinstance(accepted, dict) else None
            require(isinstance(identifier, str) and re.fullmatch(r"[a-zA-Z0-9_-]+", identifier), "Invalid accepted task ID")
            accepted_id = identifier
            report.update(command_id=accepted_id, actual_task_submitted=True)
            guard.check()
            deadline = clock()+timeout
            while clock() < deadline:
                sample_state()
                command = sample_command(accepted_id)
                if command.get("search_plan"):
                    check_live_plan(command["search_plan"])
                if live_validator is not None:
                    live_validator(command)
                require(len(perceptions) <= max_reports, "Live task exceeded its distinct report budget")
                if command.get("status") != "running":
                    break
                sleep(interval)
            else:
                raise TimeoutError("Search task exceeded {} wall seconds; no automatic retry".format(timeout))
            final = sample_state()
            report["validation"] = validator(command, initial, final, samples, missions, model)
            report["validation_passed"] = True
        except Exception as exc:
            report.update(validation_passed=False, error=str(exc), exception_type=type(exc).__name__)
            if report["actual_task_submission_attempted"] and not accepted_id:
                report.update(submission_result_unknown=True, operator_check_required=True)
            # The submitted user task explicitly includes landing. Recover only
            # this owned sortie, never an unrelated command or failed preflight.
            if report["actual_task_submission_attempted"] and guard is not None:
                try:
                    state = sample_state()
                    if not accepted_id:
                        candidate = state.get("active_command_id")
                        if isinstance(candidate, str) and re.fullmatch(r"[a-zA-Z0-9_-]+", candidate):
                            recovered = sample_command(candidate)
                            created = finite(recovered.get("created_at"), "recovered task creation time")
                            require(created >= report["submission_started_at_wall_s"]-.1
                                    and created <= time.time()+.1,
                                    "Lost acknowledgment does not have exclusive submission-time evidence")
                            accepted_id, command = candidate, recovered
                            report.update(command_id=accepted_id, submission_owner_reconciled=True,
                                          actual_task_submitted=True, operator_check_required=False)
                    if accepted_id:
                        command = sample_command(accepted_id)
                    owned, reason = recovery_ownership(state, command, accepted_id, submitted_prompt)
                    report["recovery_ownership"] = {"proved": owned, "reason": reason,
                        "command_id": accepted_id, "mission_id": (state.get("mission") or {}).get("id")}
                    current_mission = state.get("mission") or {}
                    # A still-running takeoff worker can arm after a disarmed
                    # sample. Cancel that owned worker through the same guarded
                    # land boundary, including one that already appears landed.
                    needs_recovery = state.get("armed") is True or current_mission.get("status") == "running"
                    if owned and needs_recovery:
                        current_mission = state["mission"]
                        already_landing = (current_mission.get("status") == "running"
                                           and current_mission.get("steps") == [{"action": "land"}])
                        if already_landing:
                            report["recovery_waited_for_owned_existing_landing"] = True
                            landing_id = current_mission["id"]
                        else:
                            guard.check()
                            recovery = transport(base, "/api/land", {"expected_mission_id": current_mission["id"],
                                                                    "expected_command_id": accepted_id})
                            write_once(output / "authorized_recovery_landing.json", recovery)
                            guard.check()
                            landing = recovery.get("mission") if isinstance(recovery, dict) else None
                            require(isinstance(landing, dict) and isinstance(landing.get("id"), str)
                                    and landing["id"] and landing.get("steps") == [{"action": "land"}]
                                    and landing.get("parent_mission_id") == current_mission["id"],
                                    "Recovery landing acknowledgment does not prove its accepted mission")
                            landing_id = landing["id"]
                        report["recovery_timeout_wall_s"] = RECOVERY_TIMEOUT_WALL_S
                        deadline = clock()+RECOVERY_TIMEOUT_WALL_S
                        while clock() < deadline:
                            state = sample_state()
                            if state.get("active_command_id") not in (None, accepted_id):
                                report["recovery_command_replaced"] = True
                                break
                            if (state.get("mission") or {}).get("id") != landing_id:
                                report["recovery_landing_replaced"] = True
                                break
                            if landing_recovery_confirmed(state, landing_id):
                                break
                            sleep(min(.5, max(0., deadline-clock())))
                        else:
                            report["recovery_landing_timed_out"] = True
                        report["recovery_landing_confirmed"] = (not report.get("recovery_command_replaced", False)
                            and landing_recovery_confirmed(state, landing_id))
                        command = sample_command(accepted_id)
                    elif needs_recovery:
                        report["recovery_skipped_unproven_or_replaced_flight"] = True
                except SimulationOwnershipError as recovery_error:
                    report.update(recovery_skipped_identity_change=True, recovery_landing_error=str(recovery_error))
                except Exception as recovery_error:
                    report["recovery_landing_error"] = str(recovery_error)
        finally:
            if guard is not None:
                report["simulation_owner"] = guard.evidence()
            report.update(finished_at=time.time(), state_sample_count=len(samples),
                          actual_model_call=(bool(command.get("model_calls")) if command is not None else
                                             None if report["actual_task_submission_attempted"] else False),
                          actual_flight_observed=any(s["state"].get("armed") is True for s in samples))
            write_once(output / "final_task.json", command)
            write_once(output / "final_state.json", samples[-1]["state"] if samples else None)
            write_once(output / "missions.json", list(missions.values()))
            write_once(output / "perceptions.json", list(perceptions.values()))
            write_once(output / "validation.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="Submit the explicitly authorized real flight/search task")
    parser.add_argument("--output", type=Path, help="Required new evidence directory for --execute; never reuse")
    parser.add_argument("--base-url", default=os.getenv("UAV_WEB_URL", "http://127.0.0.1:8080"))
    parser.add_argument("--model", default="qwen3-vl-plus")
    parser.add_argument("--px4-directory", type=Path, default=Path(os.getenv("PX4_DIR") or "/home/dut/PX4-Autopilot"))
    parser.add_argument("--timeout", type=float, default=900)
    parser.add_argument("--sample-interval", type=float, default=.5)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 1 <= args.timeout <= 1800 or not math.isfinite(args.sample_interval) or not .1 <= args.sample_interval <= 1:
        parser.error("timeout must be 1..1800s; sample interval must be .1..1s")
    if not args.execute:
        print(json.dumps({"prompt": PROMPT, "bounds_enu_m": BOUNDS, "altitude_m": ALTITUDE,
            "max_observations": MAX_VIEWS, "actual_requests": False, "actual_model_call": False,
            "actual_flight": False, "requires_healthy_actual_ev_fusion": True, "task_retries": 0,
            "post_endpoint": "/api/commands", "valid_outcomes": ["target_found", "sampling_complete_without_confirmation"],
            "target_required_to_be_visible": False, "metric_required_to_be_available": False,
            "occlusion_free_coverage_proven": False}, ensure_ascii=False, indent=2))
        return 0
    if args.output is None:
        parser.error("--execute requires a new --output directory")
    result = execute(args.base_url.rstrip("/"), args.output, args.model, args.timeout, args.sample_interval,
                     px4_directory=args.px4_directory)
    print(json.dumps({"output": str(args.output), "passed": result["validation_passed"],
                      "command_id": result.get("command_id"), "error": result.get("error"),
                      "observed_views": result.get("validation", {}).get("actual_observed_views")}, ensure_ascii=False))
    return 0 if result["validation_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
