"""Run configured natural-language sorties through the real single-VLM web Agent.

No request is issued when this module is imported or when --dry-run is used.
The scenario configuration contains explicit approved observation waypoints;
ground-truth object aliases are used only by this test, never added to prompts.
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
from urllib.error import HTTPError
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def request(base, path, payload=None):
    data = json.dumps(payload, ensure_ascii=False).encode() if payload is not None else None
    try:
        with urlopen(Request(base + path, data=data,
                             headers={"Content-Type": "application/json"}), timeout=8) as response:
            return json.load(response)
    except HTTPError as exc:
        try:
            detail = json.loads(exc.read()).get("error")
        except (ValueError, AttributeError):
            detail = None
        raise RuntimeError("HTTP {} {}: {}".format(exc.code, path, detail or exc.reason)) from None


def distance(first, second):
    return math.sqrt(sum((float(a)-float(b)) ** 2 for a, b in zip(first, second)))


def valid_point(point):
    return (isinstance(point, list) and len(point) == 3
            and all(isinstance(value, (int, float)) and not isinstance(value, bool)
                    and math.isfinite(value) for value in point)
            and math.hypot(point[0], point[1]) <= 30 and 1 <= point[2] <= 10)


def load_plan(path):
    config = json.loads(path.read_text())
    if not isinstance(config, dict) or not isinstance(config.get("tasks"), list) or not config["tasks"]:
        raise ValueError("Scenario must contain a nonempty tasks list")
    identifiers = set()
    for task in config["tasks"]:
        identifier = task.get("id", "")
        if not re.fullmatch(r"[a-zA-Z0-9_-]+", identifier) or identifier in identifiers:
            raise ValueError("Task IDs must be unique safe file names")
        identifiers.add(identifier)
        if not isinstance(task.get("prompt"), str) or not 1 <= len(task["prompt"]) <= 2000:
            raise ValueError("Each task needs an explicit natural-language prompt, up to 2000 characters")
        observations = task.get("observations")
        if not isinstance(observations, list) or not observations:
            raise ValueError("Each task needs expected observations")
        for observation in observations:
            if not valid_point(observation.get("waypoint")):
                raise ValueError("Observation waypoint must be finite ENU [x,y,z] within flight limits")
            aliases = observation.get("category_aliases")
            if not isinstance(aliases, list) or not aliases or not all(isinstance(item, str) and item for item in aliases):
                raise ValueError("Each expected target needs nonempty category_aliases")
            colors = observation.get("color_aliases", [])
            if not isinstance(colors, list) or not all(isinstance(item, str) and item for item in colors):
                raise ValueError("color_aliases must be a list of strings")
        approved = task.get("approved_waypoints", [item["waypoint"] for item in observations])
        if not isinstance(approved, list) or not all(valid_point(point) for point in approved):
            raise ValueError("approved_waypoints must contain valid ENU positions")
        task["approved_waypoints"] = approved
        minimum = task.get("min_observations", len(observations))
        if isinstance(minimum, bool) or not isinstance(minimum, int) or not 1 <= minimum <= 8:
            raise ValueError("min_observations must be an integer within 1..8")
        task["min_observations"] = minimum
        conditional = task.get("conditional_second_observation")
        if conditional:
            for name in ("if_confirmed", "if_unconfirmed"):
                branch = conditional.get(name) or {}
                if not valid_point(branch.get("waypoint")) or not branch.get("category_aliases"):
                    raise ValueError("Conditional branches require waypoint and category_aliases")
    return config


def semantic_result(report, expected):
    """Conservative category/color match; this is separate from tool success.

    A match is an Agent identification, not an independent vision ground truth
    judgment. The actual captured image must be reviewed alongside the report.
    """
    categories = [item.casefold() for item in expected["category_aliases"]]
    colors = [item.casefold() for item in expected.get("color_aliases", [])]
    matches = []
    for target in report.get("targets") or []:
        label = str(target.get("label") or "").casefold()
        if any(alias in label for alias in categories) and (not colors or any(alias in label for alias in colors)):
            matches.append(copy.deepcopy(target))
    return {"status": "identified" if matches else "not_found",
            "category_aliases": expected["category_aliases"],
            "color_aliases": expected.get("color_aliases", []), "matched_targets": matches,
            "evidence_basis": "structured target labels; independent visual review still required",
            "summary": report.get("summary"), "limitations": report.get("limitations", [])}


def collect_perceptions(command, retained):
    values = list(command.get("perceptions") or [])
    if command.get("perception"):
        values.append(command["perception"])
    for item in values:
        if isinstance(item, dict):
            key = item.get("capture_id") or "error-{}".format(item.get("error", "unknown"))
            retained[key] = copy.deepcopy(item)


def resolve_observations(task, reports):
    expected = copy.deepcopy(task["observations"])
    conditional = task.get("conditional_second_observation")
    if not conditional:
        return expected, None
    first = reports[0]
    first_expected = expected[0]
    semantics = semantic_result(first, first_expected)
    threshold = conditional.get("minimum_confidence", .6)
    confirmed = [target for target in semantics["matched_targets"]
                 if isinstance(target.get("confidence"), (int, float))
                 and target["confidence"] >= threshold]
    summary = str(first.get("summary") or "")
    summary_category = any(alias.casefold() in summary.casefold() for alias in first_expected["category_aliases"])
    summary_color = any(alias.casefold() in summary.casefold() for alias in first_expected.get("color_aliases", []))
    negation = bool(re.search(r"未(?:发现|确认|找到|识别)|没有(?:发现|看到)|不确定|无法(?:确认|识别)|not (?:found|visible|confirmed)|no (?:blue|barrel)", summary, re.I))
    # Contradictory or unstructured affirmative prose cannot establish a reliable branch.
    if confirmed and negation or not semantics["matched_targets"] and summary_category and summary_color and not negation:
        raise AssertionError("branch_ambiguous: first report does not establish a consistent blue-barrel confirmation")
    branch_name = "if_confirmed" if confirmed else "if_unconfirmed"
    expected[1] = copy.deepcopy(conditional[branch_name])
    return expected, {"status": "resolved", "selected_branch": branch_name,
                      "minimum_confidence": threshold,
                      "first_observation_semantics": semantics,
                      "note": "Branch follows the Agent's first-frame evidence; it is not ground-truth detection accuracy."}


def check_start(state, model, require_vio):
    assert state.get("ready"), state.get("reason") or "Flight executor is not ready"
    assert not state.get("armed") and state.get("landed_state") == 1, "Every sortie must start landed/disarmed"
    assert not state.get("active_command_id"), "Another Agent command is already running"
    assert state.get("agent_architecture") == "single_vlm", "Expected the active single-VLM Agent"
    assert state.get("model", {}).get("name") == model, "Unexpected planning model"
    assert state.get("model", {}).get("available"), "The VLM is not configured"
    assert state.get("vision_model", {}).get("name") == model, "Planning and vision must use the same model"
    assert all(state.get("camera", {}).get(side, {}).get("available") for side in ("left", "right")), "Stereo camera is not fresh"
    if require_vio:
        assert state.get("vio", {}).get("ready"), state.get("vio", {}).get("reason") or "Independent VIO is not ready"


def verify_return_home(missions, reports, samples, home):
    """Accept an actual post-report return action or an equivalent home goto.

    Final landed coordinates alone are insufficient: the completed flight event,
    its submitted waypoint, temporal order, altitude and armed telemetry must
    jointly establish a return before landing.
    """
    report_finished = max(item.get("finished_at") or item.get("created_at", 0)
                          + item.get("analysis_duration_seconds", 0) for item in reports)
    land_events = [event for mission in missions for event in mission.get("events", [])
                   if event.get("action") == "land"]
    assert land_events, "No completed landing after the requested return"
    landing_finished = min(event.get("finished_at", 0) for event in land_events)
    verified = []
    for mission in missions:
        if mission.get("started_at", 0) < report_finished-.2:
            continue
        for event in mission.get("events", []):
            action = event.get("action")
            if action not in ("return_home", "goto"):
                continue
            finished = event.get("finished_at", 0)
            if not report_finished-.2 <= finished < landing_finished:
                continue
            position = event.get("position_enu_m") or []
            if len(position) != 3 or not 1 <= position[2] <= 10:
                continue
            if math.hypot(position[0]-home[0], position[1]-home[1]) >= .55:
                continue
            index = event.get("step", 0)-1
            steps = mission.get("steps", [])
            if not 0 <= index < len(steps) or steps[index].get("action") != action:
                continue
            step = steps[index]
            if action == "goto" and (not 1 <= step.get("altitude", 0) <= 10
                                      or math.hypot(step["x"]-home[0], step["y"]-home[1]) > .25):
                continue
            nearest = min(samples, key=lambda sample: abs(sample["wall_time"]-finished))
            state = nearest["state"]
            sampled_position = state.get("position_enu_m") or []
            if (abs(nearest["wall_time"]-finished) > 1.5 or not state.get("armed")
                    or len(sampled_position) != 3
                    or math.hypot(sampled_position[0]-home[0], sampled_position[1]-home[1]) >= .65
                    or sampled_position[2] < .8):
                continue
            verified.append({"method": "return_home" if action == "return_home" else "goto_recorded_home",
                             "mission_id": mission["id"], "step": event["step"],
                             "submitted_step": step, "completed_event": event,
                             "last_report_finished_at": report_finished,
                             "sample_nearest_return": nearest})
    assert verified, "No verified airborne post-report return to the recorded home before landing"
    return verified


def audit_task(task, command, samples, initial, final, perceptions, missions, config):
    assert command.get("status") == "succeeded", command.get("error") or command.get("reply")
    assert final.get("ready") and not final.get("armed") and final.get("landed_state") == 1, "Sortie did not finish landed/disarmed"
    calls = command.get("model_calls") or []
    assert calls and all(item.get("model") == config.get("model", "qwen3-vl-plus") for item in calls), "Every audited call must use the same VLM"
    assert all(item.get("status") == "ok" for item in calls), "A model call failed; no automatic test retry is permitted"
    assert "image_base64" not in json.dumps(command), "Public command leaked image bytes"
    tool_sequence = [tool.get("name") for call in calls for tool in call.get("called_tools", [])]
    assert "get_drone_state" in tool_sequence, "Agent never read the actual vehicle state"
    reports = sorted(perceptions.values(), key=lambda item: item.get("created_at", 0))
    assert len(reports) == len(task["observations"]), "Expected {} distinct view reports, received {}".format(len(task["observations"]), len(reports))
    assert tool_sequence.count("capture_scene") == len(reports), "Capture count differs from expected views"
    assert tool_sequence.count("report_scene") == len(reports), "Report count differs from expected views"

    home = initial["position_enu_m"]
    new_missions = [item for item in missions.values() if item.get("id") != (initial.get("mission") or {}).get("id")]
    assert new_missions and all(item.get("status") == "succeeded" for item in new_missions), "A flight stage was not completed successfully"
    completed = [event for item in new_missions for event in item.get("events", [])]
    assert sum(event.get("action") == "takeoff" for event in completed) == 1, "Expected one completed takeoff"
    assert sum(event.get("action") == "land" for event in completed) == 1, "Expected one completed landing"
    returned_via = verify_return_home(new_missions, reports, samples, home) if task.get("require_return_home", True) else []
    home_goto_steps = {(item["mission_id"], item["step"]) for item in returned_via if item["method"] == "goto_recorded_home"}
    assert math.hypot(final["position_enu_m"][0]-home[0], final["position_enu_m"][1]-home[1]) < .6, "Landing did not occur near home"
    for mission in new_missions:
        for index, step in enumerate(mission.get("steps", []), 1):
            if step.get("action") == "goto":
                point = [step.get("x"), step.get("y"), step.get("altitude")]
                assert (mission["id"], index) in home_goto_steps or any(distance(point, approved) < .02 for approved in task["approved_waypoints"]), "Agent used an unapproved waypoint: {}".format(point)

    expected_observations, branch = resolve_observations(task, reports)
    observations = []
    previous_report_finished = None
    for expected, report in zip(expected_observations, reports):
        assert report.get("status") == "ok" and report.get("architecture") == "single_vlm", report
        assert report.get("model") == config.get("model", "qwen3-vl-plus"), "Report names a different vision model"
        camera = report.get("camera") or {}
        assert camera.get("width") == 640 and camera.get("height") == 480, "Unexpected camera resolution"
        assert re.fullmatch(r"[0-9a-f]{64}", camera.get("image_sha256", "")), "No genuine image hash"
        assert 0 <= camera.get("age_seconds", -1) <= 2 and -.05 <= camera.get("measurement_age_seconds", -1) <= .5, "Camera evidence was stale"
        capture = report["capture_id"]
        image_calls = [item for item in calls if capture in item.get("capture_ids", []) and "image" in item.get("input_modalities", [])]
        assert image_calls, "The VLM was not shown this report's actual image"
        assert any(any(tool.get("name") == "report_scene" for tool in item.get("called_tools", [])) for item in image_calls), "No image-informed report_scene call"
        stamp = report["captured_at_sim_s"]
        nearest = min(samples, key=lambda sample: abs((sample["state"].get("gazebo") or {}).get("stamp_sim_s", -1000)-stamp))
        actual = nearest["state"]
        assert abs((actual.get("gazebo") or {}).get("stamp_sim_s", -1000)-stamp) <= .5, "No sufficiently close state sample for the capture"
        assert actual.get("armed"), "Observation was not made while airborne"
        assert distance(actual["position_enu_m"], expected["waypoint"]) <= task.get("waypoint_tolerance_m", .65), "Capture occurred away from its prescribed observation point"
        capture_time = report.get("created_at", 0)
        before = [event for event in completed if event.get("finished_at", 0) <= capture_time + .1]
        assert any(event.get("action") == "takeoff" for event in before), "Image preceded completed takeoff"
        assert any(event.get("action") == "goto" and distance(event["position_enu_m"], expected["waypoint"]) < .65 for event in before), "Image preceded reaching the prescribed waypoint"
        if previous_report_finished:
            staged = [mission for mission in new_missions
                      if mission.get("started_at", 0) >= previous_report_finished-.2
                      and any(step.get("action") == "goto" and distance([step["x"], step["y"], step["altitude"]], expected["waypoint"]) < .02
                              for step in mission.get("steps", []))]
            assert staged, "Next observation flight stage started before the previous report was saved"
        previous_report_finished = report.get("finished_at") or capture_time+report.get("analysis_duration_seconds", 0)
        observations.append({"expected": expected, "capture_id": capture,
                             "sample_nearest_capture": nearest,
                             "semantic_result": semantic_result(report, expected),
                             "report": report})
    assert len({item["capture_id"] for item in observations}) == len(observations), "Different view stages reused a capture"
    return {"flight_and_tool_validation_passed": True,
            "all_requested_targets_identified": all(item["semantic_result"]["status"] == "identified" for item in observations),
            "conditional_branch": branch,
            "returned_via": returned_via,
            "tool_sequence": tool_sequence, "missions": new_missions, "observations": observations,
            "limits": ["Category matching is independent of tool success but still uses model labels; review saved capture evidence.",
                       "Only the specified views were inspected; unobserved or occluded regions were not searched.",
                       "Routes are explicit approved waypoints; this test does not claim autonomous obstacle avoidance."]}


def safety_land(base, output, report):
    try:
        report["safety_landing"] = request(base, "/api/land", {})
        deadline = time.monotonic() + 100
        while time.monotonic() < deadline:
            state = request(base, "/api/state")
            report["state_after_safety_landing"] = state
            if not state.get("armed") and state.get("landed_state") == 1:
                report["safety_landing_confirmed"] = True
                break
            time.sleep(.5)
        else:
            report["safety_landing_confirmed"] = False
    except Exception as exc:
        report["safety_landing_error"] = str(exc)
    write_json(output / "task_validation.json", report)


def run_task(base, output, task, config, timeout, interval):
    output.mkdir(parents=True, exist_ok=True)
    report = {"task": copy.deepcopy(task), "started_at": time.time(),
              "actual_model_call": True, "actual_flight": True,
              "model_call_retries_by_test": 0}
    samples, perceptions, missions = [], {}, {}
    last_command_hash = None
    try:
        initial = request(base, "/api/state")
        report["initial_state"] = initial
        check_start(initial, config.get("model", "qwen3-vl-plus"), config.get("require_vio_ready", True))
        accepted = request(base, "/api/commands", {"prompt": task["prompt"], "mode": "task",
                                                   "min_observations": task["min_observations"]})
        report["command_id"] = accepted["id"]
        print("Submitted {}: {}".format(task["id"], accepted["id"]), flush=True)
        deadline = time.monotonic() + timeout
        with (output / "state_samples.jsonl").open("w") as state_log, (output / "command_updates.jsonl").open("w") as command_log:
            while time.monotonic() < deadline:
                sample = {"wall_time": time.time(), "state": request(base, "/api/state")}
                samples.append(sample)
                state_log.write(json.dumps(sample, ensure_ascii=False, allow_nan=False) + "\n")
                state_log.flush()
                mission = sample["state"].get("mission") or {}
                if mission.get("id"):
                    missions[mission["id"]] = copy.deepcopy(mission)
                command = request(base, "/api/commands/" + accepted["id"])
                encoded = json.dumps(command, ensure_ascii=False, sort_keys=True)
                command_hash = hashlib.sha256(encoded.encode()).hexdigest()
                if command_hash != last_command_hash:
                    command_log.write(json.dumps({"wall_time": time.time(), "command": command}, ensure_ascii=False, allow_nan=False) + "\n")
                    command_log.flush()
                    last_command_hash = command_hash
                collect_perceptions(command, perceptions)
                report["command"] = command
                if command["status"] != "running":
                    break
                time.sleep(interval)
            else:
                raise TimeoutError("{} did not finish within {} wall seconds".format(task["id"], timeout))
        final = request(base, "/api/state")
        report["final_state"] = final
        if (final.get("mission") or {}).get("id"):
            missions[final["mission"]["id"]] = copy.deepcopy(final["mission"])
        report["validation"] = audit_task(task, command, samples, initial, final, perceptions, missions, config)
        report["validation_passed"] = True
        print("PASS {}: flight/report stages verified; requested targets identified={}".format(
            task["id"], report["validation"]["all_requested_targets_identified"]), flush=True)
    except Exception as exc:
        report["validation_passed"] = False
        report["error"] = str(exc)
        print("FAIL {}: {}".format(task["id"], exc), flush=True)
        safety_land(base, output, report)
    finally:
        report["finished_at"] = time.time()
        report["duration_wall_s"] = round(report["finished_at"]-report["started_at"], 3)
        report["state_sample_count"] = len(samples)
        report["perceptions"] = list(perceptions.values())
        report["sample_file"] = str(output / "state_samples.jsonl")
        report["command_updates_file"] = str(output / "command_updates.jsonl")
        write_json(output / "task_validation.json", report)
        write_json(output / "perceptions.json", report["perceptions"])
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True, help="Scenario JSON with explicit prompts and approved ENU observation points")
    parser.add_argument("--base-url", default=os.environ.get("UAV_WEB_URL", "http://127.0.0.1:8080"))
    parser.add_argument("--output", type=Path, default=ROOT / "logs/realistic_scene")
    parser.add_argument("--timeout", type=float, default=350, help="Wall-clock seconds allowed per task")
    parser.add_argument("--sample-interval", type=float, default=.5)
    parser.add_argument("--task", action="append", help="Run selected task ID; repeat to select several")
    parser.add_argument("--dry-run", action="store_true", help="Validate and print the plan without HTTP requests or flight")
    args = parser.parse_args(argv)
    config = load_plan(args.config)
    tasks = [task for task in config["tasks"] if not args.task or task["id"] in args.task]
    if not tasks or args.task and set(args.task) - {task["id"] for task in config["tasks"]}:
        parser.error("Unknown or empty task selection")
    if not 0 < args.sample_interval <= 2 or args.timeout <= 0:
        parser.error("Timeout must be positive and sampling interval within (0,2]")
    if args.dry_run:
        print(json.dumps({"scenario": config.get("name"), "model": config.get("model", "qwen3-vl-plus"),
                          "tasks": tasks, "actual_requests": False}, ensure_ascii=False, indent=2))
        return 0
    args.output.mkdir(parents=True, exist_ok=True)
    write_json(args.output / "executed_plan.json", config)
    aggregate = {"scenario": config.get("name"), "started_at": time.time(),
                 "configuration_file": str(args.config.resolve()),
                 "actual_model_call": True, "actual_flight": True, "tasks": []}
    for task in tasks:
        task_report = run_task(args.base_url.rstrip("/"), args.output / task["id"], task, config,
                               args.timeout, args.sample_interval)
        aggregate["tasks"].append({"id": task["id"], "command_id": task_report.get("command_id"),
                                   "validation_passed": task_report["validation_passed"],
                                   "all_requested_targets_identified": task_report.get("validation", {}).get("all_requested_targets_identified", False),
                                   "error": task_report.get("error"), "duration_wall_s": task_report["duration_wall_s"],
                                   "report": str(args.output / task["id"] / "task_validation.json")})
        write_json(args.output / "validation.json", aggregate)
        if not task_report["validation_passed"]:
            break  # No automatic retry or further sorties after a failure.
    aggregate["finished_at"] = time.time()
    aggregate["validation_passed"] = len(aggregate["tasks"]) == len(tasks) and all(item["validation_passed"] for item in aggregate["tasks"])
    aggregate["all_requested_targets_identified"] = aggregate["validation_passed"] and all(item["all_requested_targets_identified"] for item in aggregate["tasks"])
    write_json(args.output / "validation.json", aggregate)
    return 0 if aggregate["validation_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
