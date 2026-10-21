#!/usr/bin/env python3
"""Read-only aggregation of recorded reliability evidence; default is dry-run.

No simulator, HTTP, ROS, model, or process operations occur. Only a new --output
file is written, exclusively; original reports are never changed. Counts report
observations rather than statistical confidence or continuous-time guarantees.
"""
import argparse
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOGS = ROOT / "logs/reliability"
TERMINAL_SUITE = {"passed", "completed_with_failures", "aborted"}


def load(path):
    return json.loads(Path(path).read_text())


def file_evidence(path):
    path = Path(path)
    return {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def finite(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


def landed(state):
    return isinstance(state, dict) and state.get("armed") is False and state.get("landed_state") == 1


def mission_summary(report):
    initial_id = (report.get("initial_state", {}).get("mission") or {}).get("id")
    missions = [m for m in report.get("missions", []) if isinstance(m, dict) and m.get("id") != initial_id]
    original_id = report.get("original_mission_id")
    original = next((m for m in missions if m.get("id") == original_id), None)
    if original_id is None:
        candidates = [m for m in missions if any(s.get("action") == "takeoff" for s in m.get("steps", [])
                                                if isinstance(s, dict))]
        original = candidates[0] if len(candidates) == 1 else None
    validation = report.get("validation") or {}
    original_status = validation.get("original_mission_status") or (original or {}).get("status")
    completed = original_status == "succeeded" and original is not None
    if completed:
        steps, events = original.get("steps", []), original.get("events", [])
        completed = (original.get("step") == len(steps) and bool(steps)
                     and [e.get("action") for e in events] == [s.get("action") for s in steps]
                     and steps[-1].get("action") == "land")
    physical = any(any(e.get("action") == "takeoff" for e in m.get("events", [])) for m in missions)
    stats = validation.get("sensor_statistics") or report.get("sensor_statistics_on_failure") or {}
    if stats.get("navigation_moving_samples", 0) > 0:
        physical = True
    # actual_flight in a runner can mean accepted submission, before arming.
    # With no takeoff/moving evidence, leave physical flight unknown unless an
    # independent recovery addendum explicitly establishes no physical flight.
    return {"original_mission_id": original_id or (original or {}).get("id"),
            "original_mission_status": original_status, "requested_mission_completed": bool(completed),
            "actual_worker_submission": bool(original_id or original),
            "physical_flight_observed": True if physical else None,
            "missions": [{"id": m.get("id"), "status": m.get("status"), "error": m.get("error"),
                          "completed_actions": [e.get("action") for e in m.get("events", [])]} for m in missions]}


def event_metrics(path, fault_id=None):
    if not Path(path).is_file():
        return {"available": False, "evidence_path": str(Path(path).resolve())}
    events = load(path)
    if not isinstance(events, list):
        raise ValueError("Expected a recorded flight-events list")
    samples = [e for e in events if e.get("event") == "reliability_sample"
               and e.get("synthetic") is True and (fault_id is None or e.get("fault_id") == fault_id)]
    ends = [e for e in events if e.get("event") == "reliability_ended"
            and (fault_id is None or e.get("fault_id") == fault_id)]
    waits = [e for e in events if e.get("event") == "navigation_clearance_wait_ended"]
    return {"available": True, "evidence_path": str(Path(path).resolve()),
            "synthetic_fault_sample_count": len(samples),
            "synthetic_accepted_sample_count": sum(e.get("accepted_by_adapter") is True for e in samples),
            "synthetic_dropped_sample_count": sum(e.get("dropped") is True for e in samples),
            "wait_outcomes": [{"mission_id": e.get("mission_id"), "outcome": e.get("outcome"),
                               "elapsed_wall_s": finite(e.get("elapsed_wall_s")), "error": e.get("error")} for e in waits],
            "fault_ends": [{"fault_id": e.get("fault_id"), "reason": e.get("reason"),
                            "elapsed_wall_s": finite(e.get("elapsed_wall_s"))} for e in ends]}


def report_metrics(report, path):
    validation = report.get("validation") or {}
    geometry = validation.get("trajectory") or report.get("trajectory_on_failure") or {}
    checks = validation.get("checks") or []
    injection = report.get("injection") or {}
    request = injection.get("request") or {}
    ttl = None
    if finite(request.get("issued_at_wall_s")) is not None and finite(request.get("expires_at_wall_s")) is not None:
        ttl = request["expires_at_wall_s"] - request["issued_at_wall_s"]
    result = {"error": report.get("error") or validation.get("original_mission_error"),
              "state_sample_count": report.get("state_sample_count"), "duration_wall_s": report.get("duration_wall_s"),
              "failed_checks": [c for c in checks if c.get("passed") is False],
              "recorded_clearance": {"minimum_interpolated_segment_m": finite(geometry.get("minimum_interpolated_segment_clearance_m")),
                                     "requirement_m": finite(geometry.get("expected_clearance_m")),
                                     "requirement_passed": geometry.get("expected_inflation_clearance_passed"),
                                     "max_wall_sample_gap_s": finite(geometry.get("max_wall_sample_gap_s")),
                                     "max_sim_sample_gap_s": finite(geometry.get("max_sim_sample_gap_s")),
                                     "source": geometry.get("source"), "continuous_time_guarantee": False},
              "navigation": {"plan_count": validation.get("navigation_plan_count"),
                             "pause_count": validation.get("navigation_pause_count"),
                             "wait_branch_count": validation.get("actual_wait_branch_count"),
                             "wall_1_upper_observed": geometry.get("passed_wall1_upper_y_gt_3"),
                             "wall_2_lower_observed": geometry.get("passed_wall2_lower_y_lt_minus_1"),
                             "overflight_samples": len(geometry.get("overflight_samples_at_wall_sides", []))},
              "stopping_metrics": validation.get("stopping_metrics"),
              "synthetic_injection": {"kind": request.get("kind"), "duration_wall_s": finite(request.get("duration_s")),
                                      "request_ttl_wall_s": finite(ttl), "range_m": finite(request.get("range_m")),
                                      "bearing_rad": finite(request.get("bearing_rad")), "delay_sim_s": finite(request.get("delay_s")),
                                      "estimator_restart_published": injection.get("estimator_restart_published"),
                                      "source": "labelled local test adapter; not a real provider/sensor outage" if request else None},
              "events": event_metrics(Path(path).parent / "flight_events.json", request.get("id")),
              "automatic_flight_retries": report.get("automatic_flight_retries", report.get("model_call_retries_by_test", 0))}
    if result["navigation"]["plan_count"] is None:
        result["navigation"]["plan_count"] = sum(len(m.get("navigation_plans") or []) for m in report.get("missions", []))
    if result["navigation"]["pause_count"] is None:
        result["navigation"]["pause_count"] = sum(len(m.get("navigation_pauses") or []) for m in report.get("missions", []))
    return result


def summarize_baseline(path):
    report = load(path)
    if report.get("actual_model_call") is not True:
        raise ValueError("A baseline must explicitly record actual model calls")
    command = report.get("command") or {}
    finished = report.get("finished_at") is not None and command.get("status") in {"succeeded", "failed"}
    traces = command.get("model_calls") or []
    model_calls = [{"model": t.get("model"), "status": t.get("status"), "error": t.get("error"),
                    "error_type": t.get("error_type"), "input_modalities": t.get("input_modalities"),
                    "duration_seconds": t.get("duration_seconds")} for t in traces]
    result = {"id": Path(path).parents[1].name, "source": "recorded_actual_qwen_agent_attempt",
              "eligible_for_totals": finished, "pending": not finished, "command_status": command.get("status"),
              "agent_end_to_end_pass": finished and command.get("status") == "succeeded" and report.get("validation_passed") is True,
              "original_validation_passed": report.get("validation_passed"), "model_calls": model_calls,
              "safety_landing_confirmed": report.get("safety_landing_confirmed") is True
                                          or landed(report.get("state_after_safety_landing") or report.get("final_state")),
              "evidence": [file_evidence(path)]}
    result.update(mission_summary(report))
    result.update(report_metrics(report, path))
    return result


def summarize_suite(path):
    summary = load(path)
    if summary.get("source") != "actual_local_sitl_reliability_suite":
        raise ValueError("Unknown suite summary source")
    directory = Path(path).parent
    completed = summary.get("status") in TERMINAL_SUITE and bool(summary.get("finished_at_utc"))
    addendum_path = directory / "recovery_addendum.json"
    addendum = load(addendum_path) if addendum_path.is_file() else None
    result = {"id": directory.name, "status": summary.get("status"), "eligible_for_totals": completed,
              "pending": not completed, "planned_case_count": summary.get("planned_case_count"),
              "automatic_failure_retries": summary.get("automatic_failure_retries"),
              "restore": summary.get("restore"), "error": summary.get("error"),
              "evidence": [file_evidence(path)], "cases": []}
    if addendum is not None:
        result["evidence"].append(file_evidence(addendum_path))
        result["post_abort_observation"] = {"preserves_original_report": addendum.get("preserves_original_report"),
            "actual_worker_submission": addendum.get("actual_worker_submission"),
            "physical_flight_observed": addendum.get("physical_flight_observed"),
            "accepted_mission_id": addendum.get("accepted_mission", {}).get("id"),
            "accepted_mission_status": addendum.get("accepted_mission", {}).get("status"),
            "accepted_mission_error": addendum.get("accepted_mission", {}).get("error"),
            "landed_disarmed": landed(addendum), "infrastructure_failure": addendum.get("infrastructure_failure")}
    known = set()
    for recorded in summary.get("cases", []):
        identifier = recorded.get("id")
        known.add(identifier)
        report_path = Path(recorded.get("report_path") or directory / identifier / "runner/task_validation.json")
        # Resolve copied fixture paths relative to this suite when no absolute
        # path was recorded; no report path is ever handed to runtime navigation.
        if not report_path.is_absolute():
            report_path = directory / report_path
        case = {"id": directory.name + "/" + str(identifier), "case": recorded.get("case"),
                "expected_contract": recorded.get("expected_contract"), "runner_exit_code": recorded.get("runner_exit_code"),
                "runner_timed_out": recorded.get("runner_timed_out"), "world_sha256": recorded.get("world_sha256"),
                "report_path": str(report_path.resolve()), "eligible_for_totals": False,
                "error": recorded.get("error"), "report_missing": not report_path.is_file(),
                "original_requested_mission_completed": recorded.get("requested_mission_completed"),
                "original_expected_safety_contract_pass": recorded.get("expected_safety_contract_pass"),
                "original_safety_confirmed": recorded.get("safety_confirmed"),
                "requested_mission_completed": False, "expected_safety_contract_pass": False,
                "safety_confirmed": recorded.get("safety_confirmed") is True, "pending": True, "evidence": []}
        candidates = {"task_validation": report_path,
                      "suite_case_summary": directory / identifier / "suite_case_summary.json",
                      "flight_events": report_path.parent / "flight_events.json",
                      "trajectory_evaluation": report_path.parent / "trajectory_evaluation.json",
                      "runner_stdout": Path(recorded.get("stdout_path") or directory / identifier / "runner.log"),
                      "manager_stdout": directory / identifier / "manager.log"}
        preflight_path = (recorded.get("initial_ground_state") or {}).get("px4_preflight", {}).get("evidence_path")
        if preflight_path:
            candidates["px4_preflight_checks"] = Path(preflight_path)
        case["evidence_paths"] = {name: str(path.resolve()) for name, path in candidates.items() if path.is_file()}
        if report_path.is_file():
            report = load(report_path)
            report_completed = report.get("finished_at") is not None
            case.update(mission_summary(report))
            case.update(report_metrics(report, report_path))
            case["eligible_for_totals"] = completed and report_completed
            case["pending"] = not case["eligible_for_totals"]
            case["expected_safety_contract_pass"] = (case["eligible_for_totals"]
                and recorded.get("expected_safety_contract_pass") is True
                and report.get("validation_passed") is True and recorded.get("runner_exit_code") == 0
                and recorded.get("runner_timed_out") is not True and case["safety_confirmed"])
            case["evidence"].append(file_evidence(report_path))
        if addendum is not None and len(summary.get("cases", [])) == 1:
            case["recovery_addendum"] = result["post_abort_observation"]
            case["actual_worker_submission"] = addendum.get("actual_worker_submission") is True
            case["physical_flight_observed"] = addendum.get("physical_flight_observed")
            case["post_abort_landing_confirmed"] = landed(addendum)
            # An independently observed worker rejection does not repair the
            # original runner's missing samples or failed safety evaluation.
            case["requested_mission_completed"] = False
            case["expected_safety_contract_pass"] = False
        result["cases"].append(case)
    plan_path = directory / "plan.json"
    if plan_path.is_file():
        plan = load(plan_path)
        result["unexecuted_case_ids"] = [c.get("id") for c in plan.get("cases", []) if c.get("id") not in known]
        result["unexecuted_status"] = "pending" if not completed else "not_executed_after_suite_finished"
    return result


def aggregate(logs=DEFAULT_LOGS):
    logs = Path(logs).resolve()
    baselines, suites, warnings = [], [], []
    for path in sorted(logs.glob("baseline*/autonomous_goal_round_trip/task_validation.json")):
        try:
            baselines.append(summarize_baseline(path))
        except (OSError, ValueError, TypeError, KeyError) as exc:
            warnings.append({"path": str(path), "error": str(exc)})
    for path in sorted(logs.glob("suite*/summary.json")):
        try:
            suites.append(summarize_suite(path))
        except (OSError, ValueError, TypeError, KeyError) as exc:
            warnings.append({"path": str(path), "error": str(exc)})
    offline_path = logs / "agent_offline_v3.json"
    offline = None
    if offline_path.is_file():
        value = load(offline_path)
        if value.get("source") != "offline_fault_injection_real_langchain_graph":
            warnings.append({"path": str(offline_path), "error": "Offline injection source label mismatch"})
        else:
            offline = {"source": value["source"], "case_count": value.get("case_count"),
                       "passed_count": value.get("passed_count"), "passed": value.get("passed"),
                       "real_provider_call_count": 0, "actual_flight_count": 0,
                       "cases": [{"id": c.get("id"), "passed": c.get("passed"), "injection": c.get("injection"),
                                  "expected_behavior": c.get("expected_behavior")} for c in value.get("cases", [])],
                       "evidence": file_evidence(offline_path)}
    complete_baselines = [b for b in baselines if b["eligible_for_totals"]]
    complete_cases = [c for s in suites for c in s["cases"] if c["eligible_for_totals"]]
    traces = [t for b in complete_baselines for t in b["model_calls"]]
    preceding = {}
    for s in suites:
        for c in s["cases"]:
            previous = preceding.get(c["case"])
            if previous:
                c["preceding_attempt"] = previous["id"]
                current_params = c.get("synthetic_injection")
                previous_params = previous.get("synthetic_injection")
                c["injection_parameters_changed"] = current_params != previous_params
                if current_params != previous_params:
                    c["injection_parameter_comparison"] = {"preceding": previous_params, "current": current_params}
            preceding[c["case"]] = c
    counts = {"real_qwen_api_calls": {"total": len(traces), "ok": sum(t["status"] == "ok" for t in traces),
                                      "error": sum(t["status"] == "error" for t in traces)},
              "agent_end_to_end": {"completed_attempts": len(complete_baselines),
                                   "passed": sum(b["agent_end_to_end_pass"] for b in complete_baselines)},
              "agent_flight_execution": {"completed_original_sorties": sum(b["requested_mission_completed"] for b in complete_baselines),
                                         "attempts": len(complete_baselines)},
              "suite_safety_contract": {"evaluated_cases": len(complete_cases),
                                        "passed": sum(c["expected_safety_contract_pass"] for c in complete_cases)},
              "suite_original_missions": {"completed": sum(c["requested_mission_completed"] for c in complete_cases),
                                          "evaluated_cases": len(complete_cases)},
              "suite_worker_submissions": sum(c.get("actual_worker_submission") is True for c in complete_cases),
              "suite_physical_flights_observed": sum(c.get("physical_flight_observed") is True for c in complete_cases),
              "pending_suites": sum(s["pending"] for s in suites),
              "pending_or_unevaluated_cases": sum(c["pending"] for s in suites for c in s["cases"]),
              "pending_planned_case_count": sum(len(s["cases"]) + len(s.get("unexecuted_case_ids", [])) for s in suites if s["pending"]),
              "not_executed_after_finished_suite": sum(len(s.get("unexecuted_case_ids", [])) for s in suites if not s["pending"])}
    return {"schema_version": 1, "source": "offline_read_only_reliability_evidence_aggregation",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(), "logs_root": str(logs),
            "counts": counts, "baselines": baselines, "suites": suites, "offline_injection": offline,
            "warnings": warnings,
            "limits": ["Counts are recorded attempts, not confidence intervals or claims of a 95% success rate.",
                       "Minimum clearance applies to sampled body poses and connecting segments; continuous-time safety is not certified.",
                       "A failed requested mission can pass its expected stopping/landing contract; these are separate counts.",
                       "Running suites and missing/incomplete reports never contribute passes or completed totals.",
                       "Offline injected model exceptions are not real Qwen outages and are excluded from provider-call totals.",
                       "All failed attempts remain present; changed injection settings in a later suite are a distinct explicit attempt."]}


def compact(value):
    return {"source": value["source"], "counts": value["counts"],
            "baseline_statuses": [{"id": b["id"], "command_status": b["command_status"],
                                   "flight_completed": b["requested_mission_completed"], "error": b["error"]} for b in value["baselines"]],
            "suite_statuses": [{"id": s["id"], "status": s["status"], "pending": s["pending"],
                                "recorded_cases": len(s["cases"])} for s in value["suites"]],
            "offline_contracts": {"cases": value["offline_injection"]["case_count"],
                                  "passed": value["offline_injection"]["passed_count"], "real_provider_calls": 0}
                                 if value["offline_injection"] else None,
            "warnings": value["warnings"]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--logs-root", type=Path, default=DEFAULT_LOGS)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--dry-run", action="store_true", help="Default: print a compact summary only")
    action.add_argument("--output", type=Path, help="Write full aggregation to a NEW file exclusively")
    args = parser.parse_args(argv)
    if args.output is not None and args.output.exists():
        parser.error("Refusing to overwrite existing evidence: " + str(args.output))
    result = aggregate(args.logs_root)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    print(json.dumps(compact(result), ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if not result["warnings"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
