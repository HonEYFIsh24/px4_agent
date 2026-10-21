"""Offline evidence tests for the reliability runner; never connect to ROS/API."""
import contextlib
import copy
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT/"scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location("navigation_reliability_runner",
                                             SCRIPTS/"test_navigation_reliability.py")
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)

MID = "offline-original"
FID = "offline-fault"
CONFIG = {"body_radius_m": .45, "tracking_margin_m": .25, "expected_clearance_m": 1.}
OBJECTS = [{"id": "obstacle_wall_1", "collision_aabb_world_m":
            {"x": [4., 5.], "y": [20., 25.], "z": [.003, 5.503]}},
           {"id": "obstacle_wall_2", "collision_aabb_world_m":
            {"x": [9., 10.], "y": [20., 25.], "z": [.003, 5.503]}}]


def sample(wall_time, x=0., speed=0., armed=True, landed=2):
    return {"wall_time": wall_time, "state": {
        "ready": True, "armed": armed, "landed_state": landed,
        "position_enu_m": [x, 0., 3.5 if armed else .05],
        "gazebo": {"available": True, "position_m": [x, 0., 3.5 if armed else .05],
                   "linear_velocity_m_s": [speed, 0., 0.], "stamp_sim_s": wall_time*.35},
        "vio": {"ready": True, "used_for_flight_control": False},
        "camera": {"left": {"available": True}, "right": {"available": True}},
        "navigation": {"enabled": True, "active": armed, "fresh": True,
                       "scan_count": 12, "lidar_available": True},
        "mission": {"id": MID, "action": "goto"}}}


def mission(status="failed", full=False, goal=13.):
    actions = ("takeoff", "goto", "return_home", "land") if full else ("takeoff",)
    times = {"takeoff": 11., "goto": 16., "return_home": 20., "land": 21.}
    return {"id": MID, "status": status, "error": None if status == "succeeded" else "closed navigation gate",
        "steps": [{"action": "takeoff", "altitude": 3.5},
                  {"action": "goto", "x": goal, "y": 0., "altitude": 3.5},
                  {"action": "return_home"}, {"action": "land"}],
        "events": [{"action": action, "finished_at": times[action],
                    "position_enu_m": [goal if action == "goto" else 0., 0., 3.5]}
                   for action in actions],
        "navigation_plans": [{"created_at": 11., "plan": {
            "source": "horizontal_lidar_aligned_enu", "scan_count": 12, "obstacle_inflation_m": 1.}}],
        "navigation_pauses": []}


def evidence(case, full=False):
    kind = runner.CASES[case][0]
    goal = 2. if kind else 13.
    samples = [sample(11., 0., .4), sample(11.5, .05, .4), sample(12., .1, .35),
        sample(12.25, .14, .15), sample(12.5, .17, .1), sample(12.75, .18, .05),
        sample(13., .18, 0.), sample(13.25, .18, 0.), sample(13.5, .18, 0.)]
    if full:
        samples += [sample(16., goal, 0.), sample(20., 0., 0.), sample(21., 0., 0., False, 1)]
    elif case == "persistent_obstacle":
        samples.extend(sample(13.75+index*.25, .18, 0.) for index in range(29))
        samples.append(sample(21., .18, 0., False, 1))
    else:
        samples.append(sample(14., .18, 0., False, 1))
    original = mission("succeeded" if full else "failed", full=full, goal=goal)
    missions = {MID: original}
    if not full:
        missions["offline-land"] = {"id": "offline-land", "status": "succeeded",
                                    "events": [{"action": "land"}]}
    injection = {"request": {"id": FID, "kind": kind}, "installed_wall_time": 11.5} if kind else None
    events = []
    if kind:
        events += [{"event": "reliability_sample", "fault_id": FID, "kind": kind, "time": 11.6,
                    "accepted_by_adapter": kind != "lidar_drop", "dropped": kind == "lidar_drop"},
                   {"event": "navigation_paused", "mission_id": MID, "created_at": 12., "time": 12.,
                    "sensor_fault": kind != "lidar_near_obstacle"}]
    if case in ("temporary_obstacle", "persistent_obstacle"):
        outcome = "observed_clearance" if full else "failed"
        events += [{"event": "navigation_clearance_wait", "mission_id": MID, "time": 12.3},
                   {"event": "navigation_clearance_wait_ended", "mission_id": MID,
                    "time": 13.5 if full else 20.3, "outcome": outcome,
                    "elapsed_wall_s": 1.2 if full else 8.,
                    "error": None if full else "flight operation timed out after 8 seconds"}]
        if full:
            events.append({"event": "navigation_planned", "mission_id": MID,
                           "time": 14., "created_at": 14.})
    return samples, missions, events, injection


def evaluate(case, values):
    samples, missions, events, injection = values
    return runner.evaluate(case, samples, missions, MID, events, injection, copy.deepcopy(OBJECTS), CONFIG)


def checks(result):
    return {entry["name"]: entry["passed"] for entry in result["checks"]}


class StopEvidenceTests(unittest.TestCase):
    def test_low_speed_requires_full_second_and_reports_maximum_physical_displacement(self):
        rows = [sample(10., 0., .3), sample(10.25, .08, .15), sample(10.5, .1, .12),
                sample(10.75, .13, .1), sample(11., .12, .05), sample(11.25, .11, 0.)]
        self.assertFalse(runner.stopping_metrics(rows[:-1], 10.)["observed"])
        result = runner.stopping_metrics(rows, 10.)
        self.assertTrue(result["observed"])
        self.assertAlmostEqual(result["stationary_confirmed_after_wall_s"], 1.25)
        self.assertAlmostEqual(result["max_recorded_xy_displacement_m"], .13)
        self.assertEqual(result["end_sample_wall_time"], 11.25)

    def test_velocity_interruption_restarts_entire_stationary_window(self):
        rows = [sample(10., 0., .1), sample(10.25, .02, .1), sample(10.5, .03, .2),
                sample(10.75, .05, .1), sample(11., .06, .1), sample(11.25, .07, .1),
                sample(11.5, .07, .1), sample(11.75, .07, .1)]
        self.assertFalse(runner.stopping_metrics(rows[:-1], 10.)["observed"])
        self.assertEqual(runner.stopping_metrics(rows, 10.)["end_sample_wall_time"], 11.75)

    def test_missing_post_pause_samples_never_assert_physical_stop(self):
        self.assertFalse(runner.stopping_metrics([], 10.)["observed"])
        self.assertFalse(runner.stopping_metrics([sample(9.)], 10.)["observed"])

    def test_long_sampling_gap_cannot_certify_continuous_low_speed(self):
        self.assertFalse(runner.stopping_metrics([sample(10.), sample(30.)], 10.)["observed"])

    def test_late_first_sample_cannot_hide_unmeasured_stop_displacement(self):
        rows = [sample(20., 4.), sample(20.25, 4.), sample(20.5, 4.), sample(20.75, 4.), sample(21., 4.)]
        self.assertFalse(runner.stopping_metrics(rows, 10.)["observed"])

    def test_recent_pre_pause_sample_is_kept_as_displacement_origin(self):
        rows = [sample(9.75, 0., .3), sample(10.1, .3, .1), sample(10.35, .35, .1),
                sample(10.6, .4, .1), sample(10.85, .38, .1), sample(11.1, .37, .1)]
        result = runner.stopping_metrics(rows, 10.)
        self.assertTrue(result["observed"])
        self.assertAlmostEqual(result["origin_sample_offset_wall_s"], -.25)
        self.assertAlmostEqual(result["max_recorded_xy_displacement_m"], .4)


class ReliabilityAuditTests(unittest.TestCase):
    def test_expected_failure_passes_only_with_failed_original_and_independent_landing(self):
        values = evidence("lidar_drop")
        result = evaluate("lidar_drop", values)
        self.assertTrue(result["validation_passed"], result["checks"])
        self.assertEqual(result["expectation"], "safe_failure")
        self.assertEqual(result["original_mission_status"], "failed")
        values[1][MID]["status"] = "succeeded"
        self.assertFalse(evaluate("lidar_drop", values)["validation_passed"])

    def test_complete_trip_cannot_pass_when_original_failed_or_return_action_missing(self):
        values = evidence("success", full=True)
        self.assertTrue(evaluate("success", values)["validation_passed"])
        values[1][MID]["status"] = "failed"
        self.assertFalse(evaluate("success", values)["validation_passed"])
        values = evidence("success", full=True)
        values[1][MID]["events"] = [event for event in values[1][MID]["events"] if event["action"] != "return_home"]
        self.assertFalse(evaluate("success", values)["validation_passed"])

    def test_completed_goal_event_requires_actual_nearby_position_sample(self):
        values = evidence("success", full=True)
        for row in values[0]:
            if row["wall_time"] == 16.:
                row["state"]["position_enu_m"] = [0., 0., 3.5]
                row["state"]["gazebo"]["position_m"] = [0., 0., 3.5]
        self.assertFalse(evaluate("success", values)["validation_passed"])

    def test_short_fault_goal_cannot_be_counted_as_full_world_success(self):
        values = evidence("success", full=True)
        values[1][MID]["steps"][1]["x"] = 2.
        next(event for event in values[1][MID]["events"] if event["action"] == "goto")["position_enu_m"][0] = 2.
        next(row for row in values[0] if row["wall_time"] == 16.)["state"].update(
            position_enu_m=[2., 0., 3.5])
        self.assertFalse(checks(evaluate("success", values))["no_unrequested_intermediate_goals"])

    def test_fault_case_cannot_silently_expand_into_full_world_goal(self):
        values = evidence("lidar_drop")
        values[1][MID]["steps"][1]["x"] = 13.
        self.assertFalse(checks(evaluate("lidar_drop", values))["no_unrequested_intermediate_goals"])

    def test_blocked_route_keeps_full_world_goal_beyond_barrier(self):
        values = evidence("blocked_route")
        self.assertTrue(evaluate("blocked_route", values)["validation_passed"])
        values[1][MID]["steps"][1]["x"] = 2.
        self.assertFalse(checks(evaluate("blocked_route", values))["no_unrequested_intermediate_goals"])

    def test_synthetic_fault_requires_actual_sample_evidence(self):
        values = evidence("lidar_invalid")
        values[2][:] = [event for event in values[2] if event["event"] != "reliability_sample"]
        self.assertFalse(checks(evaluate("lidar_invalid", values))["fault_was_actually_applied"])

    def test_adapter_rejection_is_not_an_applied_near_obstacle(self):
        values = evidence("temporary_obstacle", full=True)
        event = next(event for event in values[2] if event["event"] == "reliability_sample")
        event.update(accepted_by_adapter=False, reason="invalid/time-misaligned pose; original scan remains unmodified")
        self.assertFalse(checks(evaluate("temporary_obstacle", values))["fault_was_actually_applied"])

    def test_stale_and_invalid_scan_gates_remain_sensor_failures(self):
        for case in ("lidar_drop", "lidar_delay", "lidar_invalid"):
            values = evidence(case)
            self.assertTrue(evaluate(case, values)["validation_passed"])
            next(event for event in values[2] if event["event"] == "navigation_paused")["sensor_fault"] = False
            self.assertFalse(evaluate(case, values)["validation_passed"])

    def test_near_obstacle_gate_must_remain_map_change(self):
        values = evidence("temporary_obstacle", full=True)
        self.assertTrue(evaluate("temporary_obstacle", values)["validation_passed"])
        next(event for event in values[2] if event["event"] == "navigation_paused")["sensor_fault"] = True
        self.assertFalse(evaluate("temporary_obstacle", values)["validation_passed"])

    def test_temporary_obstacle_requires_actual_wait_in_this_mission(self):
        values = evidence("temporary_obstacle", full=True)
        self.assertTrue(evaluate("temporary_obstacle", values)["validation_passed"])
        for event in values[2]:
            if event["event"].startswith("navigation_clearance_wait"):
                event["mission_id"] = "unrelated-prior-mission"
        self.assertFalse(evaluate("temporary_obstacle", values)["validation_passed"])

    def test_temporary_obstacle_requires_replan_after_observed_clearance_end(self):
        values = evidence("temporary_obstacle", full=True)
        planned = next(event for event in values[2] if event["event"] == "navigation_planned")
        planned.update(time=12.5, created_at=12.5)
        self.assertFalse(evaluate("temporary_obstacle", values)["validation_passed"])

    def test_failed_wait_cannot_be_laundered_into_success_by_later_plan_event(self):
        values = evidence("temporary_obstacle", full=True)
        ended = next(event for event in values[2] if event["event"] == "navigation_clearance_wait_ended")
        ended["outcome"] = "failed"
        self.assertFalse(evaluate("temporary_obstacle", values)["validation_passed"])

    def test_persistent_obstacle_requires_failed_mission_and_actual_wait(self):
        values = evidence("persistent_obstacle")
        result = evaluate("persistent_obstacle", values)
        self.assertTrue(result["validation_passed"], result["checks"])
        values[2][:] = [event for event in values[2] if event["event"] != "navigation_clearance_wait"]
        self.assertFalse(evaluate("persistent_obstacle", values)["validation_passed"])

    def test_persistent_wait_requires_real_eight_second_timeout(self):
        for changes in ({"elapsed_wall_s": .2}, {"error": "sensor suddenly became stale"},
                        {"outcome": "observed_clearance"}):
            values = evidence("persistent_obstacle")
            next(event for event in values[2] if event["event"] == "navigation_clearance_wait_ended").update(changes)
            self.assertFalse(evaluate("persistent_obstacle", values)["validation_passed"])

    def test_mismatched_fault_kind_is_not_actual_application_of_requested_fault(self):
        values = evidence("lidar_delay")
        next(event for event in values[2] if event["event"] == "reliability_sample")["kind"] = "lidar_near_obstacle"
        self.assertFalse(evaluate("lidar_delay", values)["validation_passed"])


class LocalRunnerTransportTests(unittest.TestCase):
    def test_local_executor_report_truthfully_marks_no_model_call_and_issues_no_agent_requests(self):
        layout = json.loads((ROOT/"scene/obstacle_layout.json").read_text())
        initial = sample(10., armed=False, landed=1)["state"]
        initial.update(simulation_only=True, scene={"verified": True, "world_sha256": layout["world_sha256"]})
        initial["mission"] = None
        running, completed = copy.deepcopy(initial), copy.deepcopy(initial)
        running.update(armed=True, landed_state=2)
        running["mission"] = mission("running", full=False)
        completed["mission"] = mission("succeeded", full=True)
        state_iterator = iter((initial, running, completed))
        requests = []

        def local_request(base, path, payload=None):
            requests.append((base, path, copy.deepcopy(payload)))
            if path == "/state":
                return copy.deepcopy(next(state_iterator))
            if path == "/mission":
                return {"mission": {"id": MID, "status": "running"}}
            raise AssertionError("Unexpected request: " + path)

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/"case"
            with mock.patch.object(runner, "request", side_effect=local_request), \
                    mock.patch.object(runner, "read_events", return_value=[]), \
                    mock.patch.object(runner.time, "sleep"), contextlib.redirect_stdout(io.StringIO()):
                report = runner.run("success", output, ROOT/"scene/obstacle_layout.json", timeout=10.)
            stored = json.loads((output/"task_validation.json").read_text())
        self.assertIs(report["actual_model_call"], False)
        self.assertIs(stored["actual_model_call"], False)
        self.assertIs(report["actual_flight"], True)
        self.assertEqual(report["original_mission_id"], MID)
        self.assertEqual(report["state_sample_count"], 2)
        self.assertNotIn("error", report)
        self.assertEqual(report["automatic_flight_retries"], 0)
        self.assertEqual([path for _, path, _ in requests], ["/state", "/mission", "/state", "/state"])
        self.assertTrue(all(base == "http://127.0.0.1:8765" for base, _, _ in requests))
        self.assertEqual(requests[1][2]["steps"][1], {"action": "goto", "x": 13., "y": 0., "altitude": 3.5})

    def test_malformed_acknowledgement_still_recovers_owned_airborne_submission(self):
        report, requests = self.malformed_submission(armed_after_acceptance=True)
        self.assertTrue(report["actual_flight"])
        self.assertFalse(report["validation_passed"])
        self.assertEqual(report["state_sample_count"], 0)
        self.assertIn("KeyError", report["error"])
        self.assertTrue(report["fallback_landing_confirmed"])
        self.assertEqual(requests, ["/state", "/mission", "/state", "/land", "/state"])

    def test_malformed_acknowledgement_cancels_owned_worker_before_it_can_arm(self):
        report, requests = self.malformed_submission(armed_after_acceptance=False)
        self.assertTrue(report["actual_flight"])
        self.assertTrue(any(path in ("/stop", "/land") for path in requests), requests)
        self.assertTrue(report["fallback_landing_confirmed"])

    def test_owned_recovery_waits_for_worker_completion_even_when_already_landed(self):
        report, requests = self.malformed_submission(armed_after_acceptance=False, pending_recovery=True)
        self.assertTrue(report["fallback_landing_confirmed"])
        self.assertEqual(requests, ["/state", "/mission", "/state", "/land", "/state", "/state"])

    def malformed_submission(self, armed_after_acceptance, pending_recovery=False):
        layout = json.loads((ROOT/"scene/obstacle_layout.json").read_text())
        initial = sample(10., armed=False, landed=1)["state"]
        initial.update(simulation_only=True, scene={"verified": True, "world_sha256": layout["world_sha256"]})
        initial["mission"] = None
        pending = copy.deepcopy(initial)
        pending.update(armed=armed_after_acceptance, landed_state=2 if armed_after_acceptance else 1)
        pending["mission"] = mission("running", full=False)
        landed = copy.deepcopy(initial)
        landed["mission"] = {"id": "owned-recovery", "status": "succeeded", "events": [{"action": "land"}]}
        recovering = copy.deepcopy(landed)
        recovering["mission"]["status"] = "running"
        state_iterator = iter((initial, pending, recovering, landed) if pending_recovery else (initial, pending, landed))
        requests = []

        def local_request(base, path, payload=None):
            requests.append(path)
            if path == "/state":
                return copy.deepcopy(next(state_iterator))
            if path == "/mission":
                # Actual HTTP 200 started a worker but failed the response
                # contract. The caller already owns that accepted submission.
                return {"mission": {"status": "running"}}
            if path in ("/land", "/stop"):
                return {"mission": copy.deepcopy(landed["mission"])}
            raise AssertionError("Unexpected request: " + path)

        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(runner, "request", side_effect=local_request), \
                    mock.patch.object(runner, "read_events", return_value=[]), \
                    mock.patch.object(runner.time, "sleep"), contextlib.redirect_stdout(io.StringIO()):
                report = runner.run("success", Path(directory)/"case", ROOT/"scene/obstacle_layout.json", timeout=10.)
        return report, requests


if __name__ == "__main__":
    unittest.main()
