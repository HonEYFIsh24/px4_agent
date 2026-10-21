"""Pure evidence aggregation contracts; no network, process or simulator calls."""
import contextlib
import copy
import importlib.util
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/summarize_reliability.py"
SPEC = importlib.util.spec_from_file_location("reliability_evidence_summary", SCRIPT)
summary = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(summary)


def mission(status="succeeded"):
    steps = [{"action": "takeoff", "altitude": 3.5}, {"action": "goto", "x": 2., "y": 0., "altitude": 3.5},
             {"action": "return_home"}, {"action": "land"}]
    return {"id": "original", "status": status, "step": len(steps), "steps": steps,
            "events": [{"action": s["action"]} for s in steps] if status == "succeeded" else [{"action": "takeoff"}]}


class SummarizeReliabilityTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def write(self, relative, value):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value))
        return path

    def make_suite(self, name="suite_navigation_02", status="completed_with_failures", mission_status="failed", passed=True,
                   duration=20., finished=True):
        report = {"finished_at": 20. if finished else None, "original_mission_id": "original",
                  "automatic_flight_retries": 0, "actual_model_call": False, "actual_flight": True,
                  "missions": [mission(mission_status)], "validation_passed": passed,
                  "validation": {"original_mission_status": mission_status, "navigation_plan_count": 1,
                                 "actual_wait_branch_count": 1,
                                 "checks": [{"name": "actual_wait_branch", "passed": passed}],
                                 "trajectory": {"minimum_interpolated_segment_clearance_m": 1.7,
                                                "expected_clearance_m": 1., "expected_inflation_clearance_passed": True}},
                  "injection": {"request": {"id": "fault", "kind": "lidar_near_obstacle", "duration_s": duration,
                                             "issued_at_wall_s": 1., "expires_at_wall_s": 1. + duration + 5.,
                                             "range_m": .7, "bearing_rad": 0., "delay_s": 2.}}}
        report_path = self.write(name + "/persistent_obstacle_01/runner/task_validation.json", report)
        value = {"source": "actual_local_sitl_reliability_suite", "status": status,
                 "finished_at_utc": "done" if status != "running" else None,
                 "automatic_failure_retries": 0, "planned_case_count": 1,
                 "cases": [{"id": "persistent_obstacle_01", "case": "persistent_obstacle",
                            "expected_contract": "wait_then_safe_failure", "report_path": str(report_path),
                            "runner_exit_code": 0 if passed else 1, "runner_timed_out": False,
                            "requested_mission_completed": mission_status == "succeeded",
                            "expected_safety_contract_pass": passed, "safety_confirmed": True}]}
        self.write(name + "/summary.json", value)
        return report_path

    def test_failed_original_mission_can_pass_safety_without_becoming_completed(self):
        self.make_suite()
        value = summary.aggregate(self.root)
        self.assertEqual(value["counts"]["suite_safety_contract"], {"evaluated_cases": 1, "passed": 1})
        self.assertEqual(value["counts"]["suite_original_missions"]["completed"], 0)
        case = value["suites"][0]["cases"][0]
        self.assertEqual(case["original_mission_status"], "failed")
        self.assertFalse(case["requested_mission_completed"])

    def test_running_suite_and_missing_or_incomplete_report_never_contribute_passes(self):
        self.make_suite(status="running", passed=True)
        value = summary.aggregate(self.root)
        self.assertEqual(value["counts"]["suite_safety_contract"], {"evaluated_cases": 0, "passed": 0})
        self.assertEqual(value["counts"]["pending_suites"], 1)
        self.assertTrue(value["suites"][0]["cases"][0]["pending"])
        self.make_suite(name="suite_navigation_03", finished=False)
        value = summary.aggregate(self.root)
        self.assertEqual(value["counts"]["suite_safety_contract"]["passed"], 0)

    def test_api_error_after_completed_flight_remains_failed_agent_attempt(self):
        self.write("baseline2/autonomous_goal_round_trip/task_validation.json", {
            "actual_model_call": True, "finished_at": 10., "validation_passed": False,
            "command": {"status": "failed", "model_calls": [{"model": "qwen3-vl-plus", "status": "ok"},
                         {"model": "qwen3-vl-plus", "status": "error", "error_type": "APIConnectionError"}]},
            "missions": [mission()], "final_state": {"armed": False, "landed_state": 1}})
        value = summary.aggregate(self.root)
        self.assertEqual(value["counts"]["real_qwen_api_calls"], {"total": 2, "ok": 1, "error": 1})
        self.assertEqual(value["counts"]["agent_end_to_end"]["passed"], 0)
        self.assertEqual(value["counts"]["agent_flight_execution"]["completed_original_sorties"], 1)

    def test_offline_faults_are_not_counted_as_real_provider_calls_or_flights(self):
        self.write("agent_offline_v3.json", {"source": "offline_fault_injection_real_langchain_graph",
            "case_count": 15, "passed_count": 15, "passed": True,
            "cases": [{"id": "injected_connection_error", "passed": True, "injection": "offline APIConnectionError"}]})
        value = summary.aggregate(self.root)
        self.assertEqual(value["counts"]["real_qwen_api_calls"]["total"], 0)
        self.assertEqual(value["offline_injection"]["real_provider_call_count"], 0)
        self.assertEqual(value["offline_injection"]["actual_flight_count"], 0)

    def test_wrapper_recovery_records_accepted_worker_and_zero_physical_flight_without_repairing_failure(self):
        report_path = self.make_suite(name="suite_navigation_01", status="aborted", passed=False)
        report = summary.load(report_path)
        report.update(missions=[], original_mission_id=None, actual_flight=False)
        report_path.write_text(json.dumps(report))
        before = report_path.read_bytes()
        self.write("suite_navigation_01/recovery_addendum.json", {
            "actual_worker_submission": True, "physical_flight_observed": False,
            "accepted_mission": {"id": "accepted-worker", "status": "failed", "error": "PX4 rejected arming"},
            "preserves_original_report": True, "armed": False, "landed_state": 1,
            "infrastructure_failure": "wrapper misread accepted mission"})
        value = summary.aggregate(self.root)
        case = value["suites"][0]["cases"][0]
        self.assertTrue(case["actual_worker_submission"])
        self.assertIs(case["physical_flight_observed"], False)
        self.assertFalse(case["expected_safety_contract_pass"])
        self.assertFalse(case["requested_mission_completed"])
        self.assertEqual(value["counts"]["suite_worker_submissions"], 1)
        self.assertEqual(value["counts"]["suite_physical_flights_observed"], 0)
        self.assertEqual(report_path.read_bytes(), before)

    def test_changed_stimulus_in_later_suite_keeps_both_attempts_distinct(self):
        self.make_suite(name="suite_navigation_02", duration=2., passed=False)
        self.make_suite(name="suite_navigation_03", duration=6., passed=True)
        value = summary.aggregate(self.root)
        self.assertEqual(value["counts"]["suite_safety_contract"], {"evaluated_cases": 2, "passed": 1})
        later = value["suites"][1]["cases"][0]
        self.assertEqual(later["preceding_attempt"], "suite_navigation_02/persistent_obstacle_01")
        self.assertTrue(later["injection_parameters_changed"])
        self.assertEqual(later["injection_parameter_comparison"]["preceding"]["duration_wall_s"], 2.)
        self.assertEqual(later["injection_parameter_comparison"]["current"]["duration_wall_s"], 6.)

    def test_event_metrics_preserve_wait_duration_fault_expiry_and_sample_labels(self):
        report_path = self.make_suite()
        self.write(str(report_path.parent.relative_to(self.root)) + "/flight_events.json", [
            {"event": "reliability_sample", "fault_id": "fault", "synthetic": True, "accepted_by_adapter": True},
            {"event": "reliability_sample", "fault_id": "fault", "synthetic": True, "dropped": True},
            {"event": "reliability_sample", "fault_id": "other", "synthetic": True},
            {"event": "navigation_clearance_wait_ended", "outcome": "failed", "elapsed_wall_s": 8.02},
            {"event": "reliability_ended", "fault_id": "fault", "reason": "automatic_monotonic_expiry", "elapsed_wall_s": 20.03}])
        case = summary.aggregate(self.root)["suites"][0]["cases"][0]
        self.assertEqual(case["events"]["synthetic_fault_sample_count"], 2)
        self.assertEqual(case["events"]["wait_outcomes"][0]["elapsed_wall_s"], 8.02)
        self.assertEqual(case["synthetic_injection"]["request_ttl_wall_s"], 25.)
        self.assertFalse(case["recorded_clearance"]["continuous_time_guarantee"])

    def test_default_dry_run_does_not_write_and_output_is_exclusive(self):
        self.make_suite()
        before = sorted(str(p.relative_to(self.root)) for p in self.root.rglob("*"))
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(summary.main(["--logs-root", str(self.root)]), 0)
        self.assertEqual(before, sorted(str(p.relative_to(self.root)) for p in self.root.rglob("*")))
        output = self.root / "validation.json"
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(summary.main(["--logs-root", str(self.root), "--output", str(output)]), 0)
        original = output.read_bytes()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            summary.main(["--logs-root", str(self.root), "--output", str(output)])
        self.assertEqual(output.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
