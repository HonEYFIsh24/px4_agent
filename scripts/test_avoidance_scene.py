"""Real Agent goal-only sorties with independent collision-ground-truth auditing.

--dry-run and importing this module issue no HTTP/model/flight requests.
Only prompts and minimum observation counts go to the Agent. obstacle_layout
is read exclusively by this evaluator, never supplied to runtime navigation.
"""
import argparse
import copy
import hashlib
import json
import math
import os
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_realistic_scene import (ROOT, check_start, collect_perceptions, distance,
                                  request, safety_land, semantic_result, valid_point,
                                  write_json)


def segment_rectangle_distance(start, end, box):
    """Exact Euclidean minimum from an entire XY segment to an axis-aligned box.

    Squared distance is quadratic between x/y boundary crossings; check the
    stationary point and endpoints on every interval, including intersections.
    """
    delta = [end[index]-start[index] for index in (0, 1)]
    breaks = [0., 1.]
    for axis, key in enumerate(("x", "y")):
        if abs(delta[axis]) > 1e-12:
            for boundary in box[key]:
                point = (boundary-start[axis])/delta[axis]
                if 0 < point < 1:
                    breaks.append(point)
    breaks = sorted(set(breaks))
    best = (float("inf"), 0.)
    for low, high in zip(breaks, breaks[1:]):
        middle = (low+high)/2
        coefficients = []
        for axis, key in enumerate(("x", "y")):
            value = start[axis]+delta[axis]*middle
            if value < box[key][0]:
                coefficients.append((delta[axis], start[axis]-box[key][0]))
            elif value > box[key][1]:
                coefficients.append((delta[axis], start[axis]-box[key][1]))
            else:
                coefficients.append((0., 0.))
        candidates = [low, high]
        denominator = sum(first*first for first, _ in coefficients)
        if denominator:
            stationary = -sum(first*second for first, second in coefficients)/denominator
            candidates.append(max(low, min(high, stationary)))
        for fraction in candidates:
            squared = sum((first*fraction+second)**2 for first, second in coefficients)
            if squared < best[0]:
                best = (squared, fraction)
    return {"distance_m": math.sqrt(max(0., best[0])), "fraction": best[1],
            "position_xy_m": [start[index]+delta[index]*best[1] for index in (0, 1)]}


def load_config(path):
    config = json.loads(path.read_text())
    tasks = config.get("tasks")
    if not isinstance(tasks, list) or not tasks:
        raise ValueError("Configuration requires tasks")
    identifiers = set()
    for task in tasks:
        identifier = task.get("id", "")
        if not re.fullmatch(r"[a-zA-Z0-9_-]+", identifier) or identifier in identifiers:
            raise ValueError("Task IDs must be unique safe file names")
        identifiers.add(identifier)
        if not valid_point(task.get("goal_enu_m")):
            raise ValueError("Task goal must be finite ENU [x,y,z] inside original flight limits")
        if abs(task["goal_enu_m"][2]-3.5) > .01:
            raise ValueError("This test requires fixed ENU cruise altitude 3.5 m")
        if not isinstance(task.get("prompt"), str) or not 1 <= len(task["prompt"]) <= 2000:
            raise ValueError("Task needs a natural-language prompt")
        minimum = task.get("min_observations", 0)
        if type(minimum) is not int or not 0 <= minimum <= 8:
            raise ValueError("min_observations must be 0..8")
        if minimum and not task.get("expected_target", {}).get("category_aliases"):
            raise ValueError("Visual task requires expected_target.category_aliases")
    return config


def load_ground_truth(path):
    layout = json.loads(path.read_text())
    if layout.get("ground_truth_only") is not True:
        raise ValueError("Expected explicit evaluation-only layout")
    objects = []
    for item in layout.get("objects", []):
        box = item.get("collision_aabb_world_m")
        if box is None:
            raise ValueError("Object {} lacks pure collision AABB".format(item.get("id")))
        for key in ("x", "y", "z"):
            bounds = box.get(key)
            if (not isinstance(bounds, list) or len(bounds) != 2
                    or not all(isinstance(value, (int, float)) and math.isfinite(value) for value in bounds)
                    or bounds[0] > bounds[1]):
                raise ValueError("Invalid collision AABB")
        objects.append({"id": item["id"], "collision_aabb_world_m": box})
    if not objects:
        raise ValueError("No collision objects available for independent evaluation")
    return layout, objects


def trajectory_evaluation(samples, objects, home_xy, goal_xy, config):
    points = []
    for index, sample in enumerate(samples):
        gazebo = sample["state"].get("gazebo") or {}
        position = gazebo.get("position_m")
        if (not gazebo.get("available") or not isinstance(position, list) or len(position) != 3
                or not all(isinstance(value, (int, float)) and math.isfinite(value) for value in position)):
            raise AssertionError("Missing/freshness-invalid world body pose at sample {}".format(index))
        points.append(position)
    assert len(points) >= 2, "Insufficient trajectory samples"
    minima = []
    for item in objects:
        box = item["collision_aabb_world_m"]
        best_point = min((segment_rectangle_distance(point, point, box)["distance_m"], index)
                         for index, point in enumerate(points))
        best_segment = None
        for index, (first, second) in enumerate(zip(points, points[1:])):
            geometry = segment_rectangle_distance(first, second, box)
            if best_segment is None or geometry["distance_m"] < best_segment["distance_m"]:
                best_segment = {**geometry, "start_sample": index, "end_sample": index+1,
                                "start_position_world_m": first, "end_position_world_m": second}
        direct = segment_rectangle_distance(home_xy, goal_xy, box) if goal_xy is not None else None
        minima.append({"object_id": item["id"], "collision_aabb_world_m": box,
                       "minimum_point_clearance_m": best_point[0], "minimum_point_sample": best_point[1],
                       "minimum_segment_clearance": best_segment,
                       "direct_home_goal_clearance_m": direct["distance_m"] if direct else None})
    closest = min(minima, key=lambda value: value["minimum_segment_clearance"]["distance_m"])
    minimum = closest["minimum_segment_clearance"]["distance_m"]
    physical = config.get("body_radius_m", .45)+config.get("tracking_margin_m", .25)
    direct_hit = [value["object_id"] for value in minima if value["direct_home_goal_clearance_m"] is not None
                  and value["direct_home_goal_clearance_m"] <= 1e-9]
    footprint_blocked = [value["object_id"] for value in minima if value["direct_home_goal_clearance_m"] is not None
                         and value["direct_home_goal_clearance_m"] <= physical]
    wall1 = next(item for item in objects if item["id"] == "obstacle_wall_1")["collision_aabb_world_m"]
    wall2 = next(item for item in objects if item["id"] == "obstacle_wall_2")["collision_aabb_world_m"]
    upper = [(index, point) for index, point in enumerate(points)
             if wall1["x"][0]-.5 <= point[0] <= wall1["x"][1]+.5 and point[1] > 3]
    lower = [(index, point) for index, point in enumerate(points)
             if wall2["x"][0]-.5 <= point[0] <= wall2["x"][1]+.5 and point[1] < -1]
    wall_zmax = min(wall1["z"][1], wall2["z"][1])
    high_samples = [index for index, point in upper+lower if point[2]+config.get("body_radius_m", .45) >= wall_zmax]
    wall_gaps = [second["wall_time"]-first["wall_time"] for first, second in zip(samples, samples[1:])]
    sim_gaps = [second["state"]["gazebo"]["stamp_sim_s"]-first["state"]["gazebo"]["stamp_sim_s"]
                for first, second in zip(samples, samples[1:])]
    expected = config.get("expected_clearance_m", 1.)
    return {"source": "independent_collision_aabb_world_m_evaluation_only",
            "body_center_coordinate_frame": "gazebo_world", "collision_objects_checked": len(objects),
            "minimum_point_clearance_m": min(item["minimum_point_clearance_m"] for item in minima),
            "minimum_interpolated_segment_clearance_m": minimum, "closest_obstacle": closest,
            "per_object_clearance": minima, "body_radius_m": config.get("body_radius_m", .45),
            "tracking_margin_m": config.get("tracking_margin_m", .25), "physical_clearance_requirement_m": physical,
            "physical_clearance_passed": minimum >= physical, "expected_clearance_m": expected,
            "expected_inflation_clearance_passed": minimum >= expected,
            "clearance_shortfall_m": max(0., expected-minimum),
            "direct_segment_intersection_object_ids": direct_hit,
            "direct_segment_footprint_blocked_object_ids": footprint_blocked,
            "direct_centerline_intersects_both_walls": all(name in direct_hit for name in ("obstacle_wall_1", "obstacle_wall_2")),
            "direct_path_blocked_by_both_walls": all(name in footprint_blocked for name in ("obstacle_wall_1", "obstacle_wall_2")),
            "direct_segment_baseline": {"frame": "gazebo_world", "home_xy_m": list(home_xy),
                                        "goal_xy_m": list(goal_xy) if goal_xy is not None else None,
                                        "required_body_and_tracking_clearance_m": physical,
                                        "criterion": "direct segment collision-AABB clearance <= body radius + tracking margin; centerline intersection recorded separately"},
            "passed_wall1_upper_y_gt_3": bool(upper), "passed_wall2_lower_y_lt_minus_1": bool(lower),
            "wall1_upper_sample_indices": [index for index, _ in upper],
            "wall2_lower_sample_indices": [index for index, _ in lower],
            "overflight_samples_at_wall_sides": high_samples,
            "max_wall_sample_gap_s": max(wall_gaps), "max_sim_sample_gap_s": max(sim_gaps),
            "limits": ["Exact XY clearance for all recorded body-center points and every connecting line segment; not a continuous-time contact sensor.",
                       "Sample-to-sample interpolation can differ from the real curved trajectory; sample spacing is reported.",
                       "Pure collision AABBs conservatively cover objects; all listed collision objects are included regardless of height.",
                       "Ground-truth layout is used only by this evaluator, not passed to the Agent or navigation map."]}


def readiness_statistics(samples):
    counts = {name: 0 for name in ("flight", "gazebo", "vio", "left", "right", "lidar")}
    bad, moving, nav_bad = [], 0, []
    for index, sample in enumerate(samples):
        state = sample["state"]
        nav = state.get("navigation") or {}
        flags = {"flight": bool(state.get("ready")), "gazebo": bool(state.get("gazebo", {}).get("available")),
                 "vio": bool(state.get("vio", {}).get("ready")),
                 "left": bool(state.get("camera", {}).get("left", {}).get("available")),
                 "right": bool(state.get("camera", {}).get("right", {}).get("available")),
                 "lidar": bool(nav.get("lidar_available"))}
        for name, value in flags.items():
            counts[name] += value
        if not all(flags.values()):
            bad.append({"sample_index": index, "wall_time": sample["wall_time"], "flags": flags,
                        "flight_reason": state.get("reason"), "vio_reason": state.get("vio", {}).get("reason")})
        if state.get("armed") and nav.get("active") and (state.get("mission") or {}).get("action") in ("goto", "return_home"):
            moving += 1
            if not nav.get("fresh") or nav.get("scan_count", 0) <= 0 or not nav.get("lidar_available"):
                nav_bad.append({"sample_index": index, "wall_time": sample["wall_time"], "navigation": nav})
    return {"sample_count": len(samples), "available_counts": counts,
            "available_fractions": {name: value/float(len(samples)) for name, value in counts.items()},
            "bad_sensor_samples": bad, "navigation_moving_samples": moving,
            "bad_navigation_moving_samples": nav_bad,
            "all_sensor_samples_ready": not bad, "navigation_motion_fresh": not nav_bad and moving > 0,
            "note": "Map fresh=false before fixed-altitude mapping starts or after landing is expected; moving gate evaluated separately."}


def actual_goal_world_baseline(samples, goal_event, commanded_goal):
    nearest = min(samples, key=lambda sample: abs(sample["wall_time"]-goal_event["finished_at"]))
    assert abs(nearest["wall_time"]-goal_event["finished_at"]) <= 1.5, "No fresh state sample near completed goal arrival"
    state = nearest["state"]
    assert state.get("gazebo", {}).get("available"), "Goal world pose was unavailable"
    assert distance(state["position_enu_m"], commanded_goal) <= .65, "Selected world baseline is not at the completed goal"
    return {"frame": "gazebo_world", "goal_body_position_world_m": state["gazebo"]["position_m"],
            "commanded_goal_enu_m": commanded_goal, "completed_goal_event_enu": goal_event,
            "sample_nearest_completed_goal": nearest,
            "source": "actual goal arrival event plus nearest fresh Gazebo body pose; no ENU-as-world substitution"}


def failure_trajectory_evaluation(samples, objects, initial, missions, task, config, started_at):
    """Evaluate only this instruction's goal event, retaining previous-sortie history."""
    first_sample = min(sample["wall_time"] for sample in samples)
    last_sample = max(sample["wall_time"] for sample in samples)
    initial_mission_id = (initial.get("mission") or {}).get("id")
    current_goal_events = []
    for mission in missions.values():
        mission_started = mission.get("started_at", 0)
        if (mission.get("id") == initial_mission_id
                or not started_at <= mission_started <= last_sample):
            continue
        for event in mission.get("events", []):
            finished = event.get("finished_at", 0)
            if (max(started_at, first_sample) <= finished <= last_sample
                    and event.get("action") == "goto"
                    and distance(event["position_enu_m"], task["goal_enu_m"]) <= .5):
                current_goal_events.append((mission["id"], event))
    baseline = None
    if current_goal_events:
        mission_id, event = min(current_goal_events, key=lambda item: item[1]["finished_at"])
        baseline = actual_goal_world_baseline(samples, event, task["goal_enu_m"])
        baseline["source_mission_id"] = mission_id
    geometry = trajectory_evaluation(samples, objects, initial["gazebo"]["position_m"][:2],
                                     baseline["goal_body_position_world_m"][:2] if baseline else None, config)
    if baseline:
        geometry["direct_segment_baseline"]["goal_conversion_evidence"] = baseline
    else:
        geometry["direct_segment_baseline"]["unavailable_reason"] = "No completed goal arrival in current task sample/time range; an ENU goal is not substituted for actual Gazebo world pose"
    return geometry


def audit(task, command, samples, initial, final, missions, perceptions, config, layout, objects):
    home = initial["position_enu_m"]
    goal = task["goal_enu_m"]
    calls = command.get("model_calls") or []
    assert command.get("status") == "succeeded", command.get("error") or command.get("reply")
    assert final.get("ready") and not final.get("armed") and final.get("landed_state") == 1
    assert calls and all(item.get("status") == "ok" and item.get("model") == config.get("model", "qwen3-vl-plus") for item in calls)
    tools = [tool.get("name") for item in calls for tool in item.get("called_tools", [])]
    assert "get_drone_state" in tools, "Agent must read actual flight/navigation state"
    assert not command.get("tool_errors"), "Agent tool failures are not successful navigation"
    assert "image_base64" not in json.dumps(command)
    new_missions = sorted((item for item in missions.values() if item.get("id") != (initial.get("mission") or {}).get("id")), key=lambda item: item.get("started_at", 0))
    assert new_missions and all(item.get("status") == "succeeded" for item in new_missions)
    events = [event for mission in new_missions for event in mission.get("events", [])]
    assert sum(item.get("action") == "takeoff" for item in events) == 1
    assert sum(item.get("action") == "land" for item in events) == 1
    goal_events = [item for item in events if item.get("action") == "goto" and distance(item["position_enu_m"], goal) <= .5]
    assert goal_events, "No completed goal arrival"
    goal_event = min(goal_events, key=lambda item: item["finished_at"])
    goal_finished = goal_event["finished_at"]
    if task.get("expected_hold_seconds"):
        held = []
        for mission in new_missions:
            for event in mission.get("events", []):
                if event.get("action") != "hold" or event.get("finished_at", 0) <= goal_finished:
                    continue
                step = mission["steps"][event["step"]-1]
                if abs(step.get("duration", -1)-task["expected_hold_seconds"]) <= .01 and distance(event["position_enu_m"], goal) <= .55:
                    held.append(event)
        assert held, "No actual requested hold at the target"
    reports = sorted(perceptions.values(), key=lambda item: item.get("created_at", 0))
    assert len(reports) == task.get("min_observations", 0), "Unexpected observation count"
    last_report = max((item.get("finished_at") or item.get("created_at", 0)+item.get("analysis_duration_seconds", 0) for item in reports), default=goal_finished)
    return_events = [item for item in events if item.get("action") in ("goto", "return_home")
                     and item.get("finished_at", 0) > max(last_report, goal_finished)
                     and math.hypot(item["position_enu_m"][0]-home[0], item["position_enu_m"][1]-home[1]) <= .55
                     and item["position_enu_m"][2] >= 1]
    assert return_events, "No actual airborne return event after reaching the goal and reporting"
    land_finished = min(item["finished_at"] for item in events if item.get("action") == "land")
    assert all(item["finished_at"] < land_finished for item in return_events)
    assert math.hypot(final["position_enu_m"][0]-home[0], final["position_enu_m"][1]-home[1]) < .6
    for mission in new_missions:
        for step in mission.get("steps", []):
            if step.get("action") == "goto":
                point = [step["x"], step["y"], step["altitude"]]
                is_home = math.hypot(point[0]-home[0], point[1]-home[1]) <= .25 and abs(point[2]-3.5) <= .01
                assert distance(point, goal) <= .02 or is_home, "Agent supplied a prohibited intermediate waypoint"
    plans = [item for mission in new_missions for item in mission.get("navigation_plans", [])]
    assert plans, "No autonomous executor navigation plans"
    outbound = [item for item in plans if distance(item["goal_enu_m"], goal) <= .02]
    inbound = [item for item in plans if math.hypot(item["goal_enu_m"][0]-home[0], item["goal_enu_m"][1]-home[1]) <= .25]
    assert outbound and inbound, "Need real outbound and inbound autonomous plans"
    assert len(plans) == len(outbound)+len(inbound), "Executor plans contained a different unauthorized target"
    for item in plans:
        plan = item["plan"]
        assert plan.get("source") == "horizontal_lidar_aligned_enu" and plan.get("scan_count", 0) > 0
        assert 0 <= plan.get("source_age_s", 10) <= 1
        assert plan.get("obstacle_inflation_m", 0) >= config.get("expected_clearance_m", 1.)
        assert plan.get("waypoints"), "Navigation plan made no sensor-confirmed progress"
    assert any(item["plan"].get("route_complete") for item in outbound) and any(item["plan"].get("route_complete") for item in inbound)
    baseline = actual_goal_world_baseline(samples, goal_event, goal)
    geometry = trajectory_evaluation(samples, objects, initial["gazebo"]["position_m"][:2],
                                     baseline["goal_body_position_world_m"][:2], config)
    geometry["direct_segment_baseline"]["goal_conversion_evidence"] = baseline
    sensors = readiness_statistics(samples)
    observed = []
    for report in reports:
        assert report.get("status") == "ok" and report.get("architecture") == "single_vlm"
        assert report.get("model") == config.get("model", "qwen3-vl-plus")
        camera = report.get("camera") or {}
        assert camera.get("width") == 640 and camera.get("height") == 480
        assert re.fullmatch(r"[0-9a-f]{64}", camera.get("image_sha256", ""))
        assert 0 <= camera.get("age_seconds", -1) <= 2 and -.05 <= camera.get("measurement_age_seconds", -1) <= .5
        assert report.get("created_at", 0) >= goal_finished-.1, "Observation preceded actual goal arrival"
        assert any("image" in item.get("input_modalities", []) and report["capture_id"] in item.get("capture_ids", []) for item in calls)
        nearest = min(samples, key=lambda sample: abs(sample["state"]["gazebo"]["stamp_sim_s"]-report["captured_at_sim_s"]))
        assert nearest["state"].get("armed") and distance(nearest["state"]["position_enu_m"], goal) <= .65
        observed.append({"report": report, "sample_nearest_capture": nearest,
                         "semantic_result": semantic_result(report, task["expected_target"])})
    passed = (geometry["physical_clearance_passed"] and geometry["expected_inflation_clearance_passed"]
              and geometry["direct_path_blocked_by_both_walls"] and geometry["passed_wall1_upper_y_gt_3"]
              and geometry["passed_wall2_lower_y_lt_minus_1"] and not geometry["overflight_samples_at_wall_sides"]
              and sensors["all_sensor_samples_ready"] and sensors["navigation_motion_fresh"])
    return {"validation_passed": passed, "flight_and_model_tools_passed": True,
            "trajectory": geometry, "sensor_statistics": sensors,
            "missions": new_missions, "navigation_plans": plans,
            "model_called_tools": tools,
            "outbound_plan_count": len(outbound), "inbound_plan_count": len(inbound),
            "partial_frontier_plan_count": sum(not item["plan"].get("route_complete") for item in plans),
            "observations": observed, "all_requested_targets_identified": all(item["semantic_result"]["status"] == "identified" for item in observed),
            "ground_truth_used_only_for_evaluation": True, "layout_world_sha256": layout.get("world_sha256")}


def run_task(base, output, task, config, layout, objects, timeout, interval):
    output.mkdir(parents=True, exist_ok=True)
    report = {"task": task, "started_at": time.time(), "actual_model_call": True, "actual_flight": True,
              "model_call_retries_by_test": 0, "ground_truth_used_only_for_evaluation": True}
    samples, missions, perceptions = [], {}, {}
    previous_hash = None
    try:
        initial = request(base, "/api/state")
        report["initial_state"] = initial
        check_start(initial, config.get("model", "qwen3-vl-plus"), True)
        assert initial.get("scene", {}).get("id") == "obstacle_yard" and initial["scene"].get("verified")
        assert initial["scene"].get("world_sha256") == layout.get("world_sha256"), "Evaluation layout/world mismatch"
        assert initial.get("navigation", {}).get("enabled") and initial["navigation"].get("lidar_available")
        assert initial["navigation"].get("lidar_sample_count") == 360
        accepted = request(base, "/api/commands", {"prompt": task["prompt"], "mode": "task",
                                                   "min_observations": task.get("min_observations", 0)})
        report["command_id"] = accepted["id"]
        print("Submitted real avoidance {}: {}".format(task["id"], accepted["id"]), flush=True)
        deadline = time.monotonic()+timeout
        with (output/"state_samples.jsonl").open("w") as state_log, (output/"command_updates.jsonl").open("w") as command_log:
            while time.monotonic() < deadline:
                sample = {"wall_time": time.time(), "state": request(base, "/api/state")}
                samples.append(sample)
                state_log.write(json.dumps(sample, ensure_ascii=False, allow_nan=False)+"\n"); state_log.flush()
                mission = sample["state"].get("mission") or {}
                if mission.get("id"):
                    missions[mission["id"]] = copy.deepcopy(mission)
                command = request(base, "/api/commands/"+accepted["id"])
                encoded = json.dumps(command, sort_keys=True, ensure_ascii=False)
                digest = hashlib.sha256(encoded.encode()).hexdigest()
                if digest != previous_hash:
                    command_log.write(json.dumps({"wall_time": time.time(), "command": command}, ensure_ascii=False, allow_nan=False)+"\n"); command_log.flush()
                    previous_hash = digest
                report["command"] = command
                collect_perceptions(command, perceptions)
                if command["status"] != "running":
                    break
                time.sleep(interval)
            else:
                raise TimeoutError("Avoidance task exceeded {} wall seconds".format(timeout))
        final = request(base, "/api/state")
        report["final_state"] = final
        if (final.get("mission") or {}).get("id"):
            missions[final["mission"]["id"]] = copy.deepcopy(final["mission"])
        report["validation"] = audit(task, command, samples, initial, final, missions, perceptions, config, layout, objects)
        report["validation_passed"] = report["validation"]["validation_passed"]
        write_json(output/"trajectory_evaluation.json", report["validation"]["trajectory"])
        write_json(output/"navigation_plans.json", report["validation"]["navigation_plans"])
        if not report["validation_passed"]:
            raise AssertionError("Recorded clearance/sensor/S-route evidence did not meet all requirements; inspect detailed metrics")
        print("PASS {}: min actual segment clearance {:.3f}m, plans={}".format(task["id"], report["validation"]["trajectory"]["minimum_interpolated_segment_clearance_m"], len(report["validation"]["navigation_plans"])), flush=True)
    except Exception as exc:
        report["validation_passed"] = False
        report["error"] = str(exc)
        if samples:
            report["sensor_statistics_on_failure"] = readiness_statistics(samples)
            try:
                report["trajectory_on_failure"] = failure_trajectory_evaluation(
                    samples, objects, report["initial_state"], missions, task, config, report["started_at"])
                write_json(output/"trajectory_evaluation.json", report["trajectory_on_failure"])
            except Exception as geometry_error:
                report["trajectory_evaluation_error"] = str(geometry_error)
        print("FAIL {}: {}".format(task["id"], exc), flush=True)
        safety_land(base, output, report)
    finally:
        report["finished_at"] = time.time(); report["duration_wall_s"] = round(report["finished_at"]-report["started_at"], 3)
        report["state_sample_count"] = len(samples); report["perceptions"] = list(perceptions.values())
        report["missions"] = list(missions.values())
        write_json(output/"navigation_plans.json", [plan for mission in report["missions"]
                   if mission.get("id") != (report.get("initial_state", {}).get("mission") or {}).get("id")
                   for plan in mission.get("navigation_plans", [])])
        write_json(output/"task_validation.json", report); write_json(output/"perceptions.json", report["perceptions"])
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"scene/avoidance_tasks.json")
    parser.add_argument("--layout", type=Path, default=ROOT/"scene/obstacle_layout.json")
    parser.add_argument("--base-url", default=os.environ.get("UAV_WEB_URL", "http://127.0.0.1:8080"))
    parser.add_argument("--output", type=Path, default=ROOT/"logs/avoidance")
    parser.add_argument("--timeout", type=float, default=800)
    parser.add_argument("--sample-interval", type=float, default=.5)
    parser.add_argument("--task", action="append")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    config = load_config(args.config); layout, objects = load_ground_truth(args.layout)
    tasks = [item for item in config["tasks"] if not args.task or item["id"] in args.task]
    if not tasks or args.task and set(args.task)-{item["id"] for item in config["tasks"]}:
        parser.error("Unknown task selection")
    if args.timeout <= 0 or not 0 < args.sample_interval <= 2:
        parser.error("Positive timeout and sampling interval within (0,2] required")
    if args.dry_run:
        dry_goal = tasks[0]["goal_enu_m"]
        direct = trajectory_evaluation([{"wall_time":0,"state":{"gazebo":{"available":True,"position_m":[0,0,3.5],"stamp_sim_s":0}}},
                                        {"wall_time":1,"state":{"gazebo":{"available":True,"position_m":dry_goal,"stamp_sim_s":1}}}], objects, [0,0], dry_goal[:2], config)
        print(json.dumps({"tasks":tasks,"ground_truth_collision_objects":len(objects),"direct_path_intersects":direct["direct_segment_intersection_object_ids"],"actual_requests":False},ensure_ascii=False,indent=2))
        return 0
    args.output.mkdir(parents=True, exist_ok=True)
    write_json(args.output/"executed_plan.json", config)
    aggregate = {"started_at":time.time(),"actual_model_call":True,"actual_flight":True,"tasks":[],"evaluation_layout":str(args.layout.resolve()),"world_sha256":layout.get("world_sha256")}
    for task in tasks:
        value = run_task(args.base_url.rstrip("/"),args.output/task["id"],task,config,layout,objects,args.timeout,args.sample_interval)
        aggregate["tasks"].append({"id":task["id"],"command_id":value.get("command_id"),"validation_passed":value["validation_passed"],"error":value.get("error"),"duration_wall_s":value["duration_wall_s"],"report":str(args.output/task["id"]/"task_validation.json")})
        write_json(args.output/"validation.json",aggregate)
        if not value["validation_passed"]:
            break
    aggregate["finished_at"]=time.time();aggregate["validation_passed"]=len(aggregate["tasks"])==len(tasks) and all(item["validation_passed"] for item in aggregate["tasks"])
    write_json(args.output/"validation.json",aggregate)
    return 0 if aggregate["validation_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
