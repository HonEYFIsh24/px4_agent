#!/usr/bin/env python3
"""Actual local SITL reliability tests; imports and --dry-run never issue I/O.

These tests call the deterministic executor directly. The separate baseline
runner exercises real Qwen. Synthetic faults modify measured sensor input;
collision truth is read only by this independent evaluator.
"""
import argparse
import copy
import hashlib
import json
import math
import socket
import subprocess
import sys
import time
import uuid
import xmlrpc.client
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from reliability_faults import FAULT_PARAMETER, fault_request
from test_realistic_scene import request, write_json
from test_avoidance_scene import load_ground_truth, readiness_statistics, trajectory_evaluation

CASES = {
    "success": (None, 0., "complete_round_trip"),
    "blocked_route": (None, 0., "safe_failure"),
    "lidar_drop": ("lidar_drop", 4., "safe_failure"),
    "lidar_delay": ("lidar_delay", 3., "safe_failure"),
    "lidar_invalid": ("lidar_invalid", 3., "safe_failure"),
    # At 0.35 simulation speed the hold-position settling phase can take ~8
    # wall seconds. Keep this pulse present into the clearance-wait branch;
    # expiry still leaves its unchanged 8-second budget for real free rays.
    "temporary_obstacle": ("lidar_near_obstacle", 10., "wait_then_complete"),
    "persistent_obstacle": ("lidar_near_obstacle", 20., "wait_then_safe_failure"),
    "vio_tracking_loss": ("vio_tracking_loss", 3., "monitor_fault_and_complete"),
    "vio_reset": ("vio_reset", 3., "monitor_fault_and_complete"),
}


class TimedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 5
        return connection


def set_fault(value):
    master = xmlrpc.client.ServerProxy("http://127.0.0.1:11311", transport=TimedTransport())
    code, message, _ = master.setParam("/navigation_reliability_suite", FAULT_PARAMETER, value)
    if code != 1:
        raise RuntimeError("fault parameter refused: " + message)


def inject(kind, duration, identifier):
    value = fault_request(kind, duration, identifier)
    if kind != "vio_reset":
        set_fault(value)
        return {"request": value, "estimator_restart_published": False}
    # The script publishes the actual upstream /vins_restart message. This
    # operation does not publish aircraft commands or fabricated odometry.
    command = ["bash", "-c", 'source "$1/scripts/setup_ros.bash"; shift; exec "$@"',
               "navigation_reliability", str(ROOT), "/usr/bin/python3",
               str(ROOT / "scripts/inject_reliability_fault.py"), "--kind", kind,
               "--duration", str(duration), "--id", identifier]
    result = subprocess.run(command, capture_output=True, text=True, timeout=10)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "VINS restart injection failed")
    return json.loads(result.stdout.strip().splitlines()[-1])


def read_events(offset):
    with (ROOT / "logs/flight.jsonl").open("rb") as source:
        source.seek(offset)
        lines = source.read().splitlines()
    # Called only after mission completion; every line must be complete JSON.
    return [json.loads(line) for line in lines if line.strip()]


def stopping_metrics(samples, pause_time):
    """Measured physical XY displacement until continuously low speed for 1s."""
    after = [row for row in samples if row["wall_time"] >= pause_time]
    if not after:
        return {"observed": False}
    before = [row for row in samples if row["wall_time"] <= pause_time]
    first_offset = after[0]["wall_time"]-pause_time
    if first_offset > .5:
        return {"observed": False, "reason": "first sample is too late to observe stopping"}
    origin_sample = before[-1] if before and pause_time-before[-1]["wall_time"] <= .5 else after[0]
    origin = origin_sample["state"]["gazebo"]["position_m"][:2]
    stable = None
    selected = []
    previous = pause_time
    for row in after:
        if row["wall_time"]-previous > .5:
            return {"observed": False, "reason": "sampling gap cannot establish continuous low speed"}
        previous = row["wall_time"]
        body = row["state"]["gazebo"]
        selected.append(row)
        if math.hypot(*body["linear_velocity_m_s"][:2]) <= .15:
            stable = row["wall_time"] if stable is None else stable
            if row["wall_time"] - stable >= 1.:
                return {"observed": True, "pause_wall_time": pause_time,
                        "first_sample_offset_wall_s": after[0]["wall_time"]-pause_time,
                        "origin_sample_offset_wall_s": origin_sample["wall_time"]-pause_time,
                        "stationary_confirmed_after_wall_s": row["wall_time"]-pause_time,
                        "max_recorded_xy_displacement_m": max(math.dist(origin, x["state"]["gazebo"]["position_m"][:2]) for x in selected),
                        "end_sample_wall_time": row["wall_time"],
                        "criterion": "world XY speed <= 0.15 m/s continuously for >=1 wall s; recorded samples only"}
        else:
            stable = None
    return {"observed": False, "pause_wall_time": pause_time}


def evaluate(case, samples, missions, original_id, events, injection, objects, config):
    kind, _, expectation = CASES[case]
    original = missions[original_id]
    final = samples[-1]["state"]
    started = samples[0]["state"]
    checks = []

    def check(name, passed, detail=None):
        checks.append({"name": name, "passed": bool(passed), "detail": detail})

    check("final_landed_disarmed", not final["armed"] and final["landed_state"] == 1)
    geometry = trajectory_evaluation(samples, objects, started["gazebo"]["position_m"][:2], None, config)
    check("recorded_clearance_at_least_1m", geometry["minimum_interpolated_segment_clearance_m"] >= 1.,
          geometry["minimum_interpolated_segment_clearance_m"])
    check("fresh_world_evaluation_pose", all(x["state"]["gazebo"]["available"] for x in samples))
    check("cruise_did_not_fly_over_walls", max(x["state"]["gazebo"]["position_m"][2] for x in samples) < 5.)
    submitted_goals = [s for s in original["steps"] if s["action"] == "goto"]
    expected_goal = [2. if kind else 13., 0., 3.5]
    check("no_unrequested_intermediate_goals", len(submitted_goals) == 1 and
          [submitted_goals[0]["x"], submitted_goals[0]["y"], submitted_goals[0]["altitude"]]
          == expected_goal)
    plans = original.get("navigation_plans", [])
    check("actual_sensor_plans", bool(plans) and all(p["plan"].get("source") == "horizontal_lidar_aligned_enu"
          and p["plan"].get("scan_count", 0) > 0 and p["plan"].get("obstacle_inflation_m", 0) >= 1.
          for p in plans))
    actions = [x["action"] for x in original.get("events", [])]
    goal_step = next(s for s in original["steps"] if s["action"] == "goto")
    goal = [goal_step["x"], goal_step["y"], goal_step["altitude"]]
    if expectation in ("complete_round_trip", "wait_then_complete", "monitor_fault_and_complete"):
        check("requested_round_trip_completed", original["status"] == "succeeded" and
              all(action in actions for action in ("takeoff", "goto", "return_home", "land")), original.get("error"))
        check("landed_near_actual_start", math.dist(final["gazebo"]["position_m"][:2],
              started["gazebo"]["position_m"][:2]) <= .6)
        arrivals = [e for e in original.get("events", []) if e.get("action") == "goto"]
        check("actual_goal_reached", any(abs(row["wall_time"]-e.get("finished_at", -1e20)) <= 1.5
              and row["state"].get("armed") and row["state"]["gazebo"].get("available")
              and math.dist(row["state"]["position_enu_m"], goal) <= .65
              for e in arrivals for row in samples))
    else:
        check("original_mission_remains_failed", original["status"] == "failed", original.get("error"))
        check("failed_mission_has_no_completed_goal_or_return", not any(x in actions for x in ("goto", "return_home")))
        check("independent_landing_completed", any(m["id"] != original_id and m["status"] == "succeeded"
              and any(x["action"] == "land" for x in m.get("events", [])) for m in missions.values()))
    relevant = [e for e in events if e.get("mission_id") == original_id]
    fault_id = (injection or {}).get("request", {}).get("id")
    injected_events = [e for e in events if e.get("fault_id") == fault_id and e.get("kind") == kind] if fault_id else []
    applied = [e for e in injected_events if e.get("event") == "reliability_sample"]
    if kind == "vio_reset":
        applied_ok = (injection or {}).get("estimator_restart_published") is True
    elif kind == "lidar_drop":
        applied_ok = any(e.get("dropped") is True for e in applied)
    elif kind and kind.startswith("lidar_"):
        applied_ok = any(e.get("accepted_by_adapter") is True for e in applied)
    else:
        applied_ok = bool(applied)
    check("fault_was_actually_applied", not kind or applied_ok)
    installed = (injection or {}).get("installed_wall_time", 0)
    waits = [e for e in relevant if e.get("event") == "navigation_clearance_wait" and e.get("time", 0) >= installed]
    wait_ends = [e for e in relevant if e.get("event") == "navigation_clearance_wait_ended" and e.get("time", 0) >= installed]
    if "wait_then" in expectation:
        check("actual_blocked_start_wait_branch", bool(waits))
        if expectation == "wait_then_complete" and waits:
            recovered = [e for e in wait_ends if e.get("outcome") == "observed_clearance"
                         and waits[0]["time"] <= e.get("time", 0) and e.get("elapsed_wall_s", 9) <= 8.1]
            check("wait_observed_clearance_before_timeout", bool(recovered))
            check("new_plan_only_after_wait", any(e.get("event") == "navigation_planned"
                  and e.get("created_at", 0) >= end["time"] for e in relevant for end in recovered))
        if expectation == "wait_then_safe_failure":
            check("wait_failed_at_8_second_limit", any(e.get("outcome") == "failed"
                  and 7.9 <= e.get("elapsed_wall_s", 0) <= 8.5 and "timed out after 8" in e.get("error", "")
                  for e in wait_ends))
    stop = None
    if kind and kind.startswith("lidar_"):
        pauses = [e for e in relevant if e.get("event") == "navigation_paused" and e.get("created_at", 0) >= installed-.1]
        check("navigation_gate_stopped_motion", bool(pauses))
        if pauses:
            gate = pauses[0]["created_at"]
            stop = stopping_metrics(samples, gate)
            check("physical_stop_observed", stop["observed"], stop)
            check("recorded_stop_displacement_within_0_5m", stop.get("max_recorded_xy_displacement_m", float("inf")) <= .5)
            if kind != "lidar_near_obstacle":
                check("sensor_failure_not_reclassified_as_map_change", bool(pauses[0].get("sensor_fault")))
            else:
                check("valid_obstacle_pause_is_map_change", pauses[0].get("sensor_fault") is False)
            check("gate_response_within_2_wall_seconds", gate-installed <= 2., gate-installed)
    if kind and kind.startswith("vio_"):
        check("vio_fault_was_observed", any(not x["state"]["vio"].get("ready") for x in samples))
        check("vio_recovered", final["vio"].get("ready"))
        check("vio_still_independent_of_flight_control", all(x["state"]["vio"].get("used_for_flight_control") is False for x in samples))
        if kind == "vio_reset":
            check("actual_vins_restart_published", (injection or {}).get("estimator_restart_published") is True)
    return {"validation_passed": all(c["passed"] for c in checks), "checks": checks,
            "trajectory": geometry, "sensor_statistics": readiness_statistics(samples),
            "original_mission_status": original["status"], "original_mission_error": original.get("error"),
            "expectation": expectation, "actual_wait_branch_count": len(waits), "stopping_metrics": stop,
            "navigation_plan_count": len(plans), "navigation_pause_count": len(original.get("navigation_pauses", []))}


def run(case, output, layout_path, base="http://127.0.0.1:8765", timeout=800., interval=.25):
    if output.exists():
        raise ValueError("refusing to overwrite an existing reliability case directory")
    output.mkdir(parents=True)
    layout, objects = load_ground_truth(layout_path)
    report = {"schema_version": 1, "case": case, "scope": "actual_local_PX4_Gazebo_executor",
              "actual_model_call": False, "actual_flight": False, "synthetic_fault": CASES[case][0] is not None,
              "started_at": time.time(), "automatic_flight_retries": 0,
              "runner_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "case_configuration": {"kind": CASES[case][0], "duration_wall_s": CASES[case][1],
                                     "expectation": CASES[case][2]},
              "ground_truth_used_only_for_evaluation": True, "evaluation_layout": str(layout_path.resolve())}
    samples, missions, injection = [], {}, None
    offset = (ROOT / "logs/flight.jsonl").stat().st_size
    original_id = None
    emergency_requested = False
    config = {"body_radius_m": .45, "tracking_margin_m": .25, "expected_clearance_m": 1.}
    try:
        # Allow actual PX4 ground-height recovery; no estimator reset/offset.
        deadline = time.monotonic()+90.
        while True:
            initial = request(base, "/state")
            if not initial.get("ready") or initial.get("armed") or initial.get("landed_state") != 1:
                raise RuntimeError("every case must begin ready, landed and disarmed")
            if abs(initial["position_enu_m"][2]) <= .2:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("PX4 local ground height did not recover within 90 wall seconds")
            time.sleep(.5)
        assert initial.get("simulation_only") is True
        assert initial.get("scene", {}).get("verified") and initial["scene"]["world_sha256"] == layout["world_sha256"]
        assert initial["navigation"].get("enabled") and initial["navigation"].get("lidar_available")
        assert initial["vio"].get("ready") and all(initial["camera"][s].get("available") for s in ("left", "right"))
        if CASES[case][0]:
            assert (initial.get("reliability_test") or {}).get("enabled") is True, "test hook must be explicitly enabled"
            set_fault({})
        report["initial_state"] = initial
        goal_x = 2. if CASES[case][0] else 13.
        steps = [{"action": "takeoff", "altitude": 3.5},
                 {"action": "goto", "x": goal_x, "y": 0., "altitude": 3.5},
                 {"action": "return_home"}, {"action": "land"}]
        accepted = request(base, "/mission", {"steps": steps, "description": "Reliability: "+case})
        # /mission acknowledges an already-started worker in a mission wrapper.
        # Record the accepted submission before interpreting further fields, so
        # a response-contract failure still invokes owned-flight recovery.
        report["actual_flight"] = True
        original_id = accepted["mission"]["id"]
        report.update(original_mission_id=original_id, actual_flight=True)
        print("Started actual SITL {}: {}".format(case, original_id), flush=True)
        deadline = time.monotonic()+timeout
        with (output/"state_samples.jsonl").open("w") as stream:
            while time.monotonic() < deadline:
                state = request(base, "/state")
                row = {"wall_time": time.time(), "state": state}
                samples.append(row); stream.write(json.dumps(row, allow_nan=False)+"\n"); stream.flush()
                mission = state.get("mission") or {}
                if mission.get("id") == original_id or emergency_requested:
                    missions[mission["id"]] = copy.deepcopy(mission)
                kind, duration, _ = CASES[case]
                displacement = math.dist(state["gazebo"]["position_m"][:2], initial["gazebo"]["position_m"][:2])
                if (kind and injection is None and mission.get("id") == original_id
                        and mission.get("status") == "running" and mission.get("action") == "goto"
                        and state["navigation"].get("active") and state["navigation"].get("fresh")
                        and displacement >= .55):
                    identifier = case+"_"+uuid.uuid4().hex[:12]
                    installed = time.time()
                    injection = inject(kind, duration, identifier)
                    injection["installed_wall_time"] = installed
                    injection["trigger_observation"] = {
                        "sample_wall_time": row["wall_time"], "mission_id": original_id,
                        "mission_action": mission.get("action"),
                        "position_enu_m": state["position_enu_m"],
                        "physical_position_m": state["gazebo"]["position_m"],
                        "physical_velocity_m_s": state["gazebo"]["linear_velocity_m_s"],
                        "target_enu_m": state.get("target_enu_m"),
                        "navigation_plan_count": len(mission.get("navigation_plans", [])),
                        "home_displacement_xy_m": displacement,
                    }
                    write_json(output/"injection.json", injection)
                    print("Injected Synthetic {}: {}".format(kind, identifier), flush=True)
                finished = mission.get("status") in ("succeeded", "failed", "cancelled")
                if mission.get("id") == original_id and finished and state["armed"] and not emergency_requested:
                    if mission["status"] == "succeeded":
                        raise AssertionError("completed mission still armed")
                    request(base, "/land", {}); emergency_requested = True
                if original_id in missions and missions[original_id].get("status") in ("succeeded", "failed", "cancelled"):
                    if not state["armed"] and state["landed_state"] == 1:
                        if kind and kind.startswith("vio_") and not state["vio"].get("ready"):
                            time.sleep(interval); continue
                        break
                time.sleep(interval)
            else:
                raise TimeoutError("reliability case exceeded its wall-time limit")
        events = read_events(offset)
        write_json(output/"flight_events.json", events)
        report["injection"] = injection
        report["validation"] = evaluate(case, samples, missions, original_id, events, injection, objects, config)
        report["validation_passed"] = report["validation"]["validation_passed"]
        write_json(output/"trajectory_evaluation.json", report["validation"]["trajectory"])
        print("{} {}: {} plans, {} waits, min clearance {:.3f}m".format(
            "PASS" if report["validation_passed"] else "FAIL", case,
            report["validation"]["navigation_plan_count"], report["validation"]["actual_wait_branch_count"],
            report["validation"]["trajectory"]["minimum_interpolated_segment_clearance_m"]), flush=True)
    except Exception as exc:
        report.update(validation_passed=False, error=type(exc).__name__+": "+str(exc))
        print("FAIL {}: {}".format(case, report["error"]), flush=True)
        try:
            current = request(base, "/state")
            # An accepted worker can still be preparing takeoff while disarmed.
            # Cancel that owned worker before it has a chance to arm; /land
            # cancels/joins it and starts a separate, observable landing task.
            if report["actual_flight"] and (current.get("armed") or
                    (current.get("mission") or {}).get("status") == "running"):
                request(base, "/land", {})
                deadline = time.monotonic()+120
                while time.monotonic() < deadline:
                    current = request(base, "/state")
                    if (not current["armed"] and current["landed_state"] == 1 and
                            (current.get("mission") or {}).get("status") != "running"):
                        break
                    time.sleep(.5)
                report["fallback_landing_confirmed"] = (not current["armed"] and current["landed_state"] == 1
                        and (current.get("mission") or {}).get("status") != "running")
        except Exception as land_error:
            report["fallback_landing_error"] = str(land_error)
    finally:
        if CASES[case][0]:
            try:
                set_fault({})
            except Exception as clear_error:
                report["fault_clear_error"] = str(clear_error)
        report.update(finished_at=time.time(), state_sample_count=len(samples), missions=list(missions.values()))
        report["duration_wall_s"] = report["finished_at"]-report["started_at"]
        write_json(output/"task_validation.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, required=True)
    parser.add_argument("--layout", type=Path, default=ROOT/"scene/obstacle_layout.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=800.)
    parser.add_argument("--sample-interval", type=float, default=.25)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    if not 0 < args.sample_interval <= 1 or not 1 <= args.timeout <= 1200:
        parser.error("sampling interval must be (0,1], timeout 1..1200")
    if args.dry_run:
        layout, objects = load_ground_truth(args.layout)
        print(json.dumps({"case": args.case, "expectation": CASES[args.case][2],
                          "layout_hash": layout["world_sha256"], "collision_objects": len(objects),
                          "actual_requests": False}, ensure_ascii=False))
        return 0
    value = run(args.case, args.output, args.layout, timeout=args.timeout, interval=args.sample_interval)
    return 0 if value["validation_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
