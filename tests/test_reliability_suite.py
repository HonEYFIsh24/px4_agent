"""Orchestration contracts using process/transport fixtures; no actual simulator."""
import contextlib
import copy
import importlib.util
import io
import json
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/run_reliability_suite.py"
SPEC = importlib.util.spec_from_file_location("reliability_suite_runner", SCRIPT)
suite = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(suite)


def ground(digest=None, testing=True):
    return {"simulation_only": True, "armed": False, "landed_state": 1, "ready": True,
            "position_enu_m": [0., 0., 0.],
            "active_command_id": None, "mission": {"status": "succeeded"},
            "gazebo": {"available": True}, "vio": {"ready": True},
            "camera": {"left": {"available": True}, "right": {"available": True}},
            "scene": {"verified": True, "world_sha256": digest or suite.sha256(suite.CANONICAL_WORLD)},
            "navigation": {"enabled": True, "lidar_available": True, "lidar_sample_count": 360},
            "reliability_test": {"enabled": testing, "active_fault": None, "pending_fault": None}}


class LocalOperations:
    """Stateful transport/process fixture; production run_suite makes decisions."""
    def __init__(self, failures=(), expected_failures=(), unsafe_after=None, busy=False, readiness_failure=False):
        self.calls = []
        self.identity = {"pid": 100, "start_time": "123", "run_directory": "/local/initial"}
        self.started = None
        self.state = ground()
        if busy:
            self.state["active_command_id"] = "operator-owned-command"
        self.failures = set(failures)
        self.expected_failures = set(expected_failures)
        self.unsafe_after = unsafe_after
        self.readiness_failure = readiness_failure

    def states(self):
        return copy.deepcopy(self.state), copy.deepcopy(self.state)

    def initial_manager(self):
        suite.safe_ground(*self.states())
        return copy.deepcopy(self.identity)

    def stop_manager(self, identity, stdout_path):
        suite.safe_ground(*self.states())
        if identity != self.identity:
            raise suite.SafetyError("Fixture manager changed")
        self.calls.append(("stop", copy.deepcopy(identity)))
        self.identity = None

    def start_manager(self, world, testing, stdout_path):
        self.calls.append(("start", str(world), testing))
        pid = 100 + len(self.calls)
        self.identity = {"pid": pid, "start_time": str(pid), "run_directory": "/local/" + str(pid)}
        self.started = types.SimpleNamespace(pid=pid)
        self.state = ground(suite.sha256(world), testing)

    def wait_ready(self, digest, testing, timeout):
        self.calls.append(("ready", digest, testing, timeout))
        if self.readiness_failure:
            self.readiness_failure = False
            raise TimeoutError("offline startup readiness timeout")
        if not suite.ready_state(*self.states(), digest, testing):
            raise AssertionError("Fixture unexpectedly not ready")
        return copy.deepcopy(self.identity), suite.safe_ground(*self.states())

    def run_runner(self, args, stdout_path, timeout):
        self.calls.append(("runner", list(args), timeout))
        output = Path(args[args.index("--output") + 1])
        key = output.parent.name
        passed = key not in self.failures
        status = "failed" if key in self.failures or key in self.expected_failures else "succeeded"
        if "--task" in args:
            report_path = output / "autonomous_goal_round_trip/task_validation.json"
            report = {"command": {"status": status}, "validation_passed": passed, "actual_flight": True}
        else:
            report_path = output / "task_validation.json"
            report = {"validation": {"original_mission_status": status}, "validation_passed": passed, "actual_flight": True}
        report_path.parent.mkdir(parents=True)
        report_path.write_text(json.dumps(report))
        if key == self.unsafe_after:
            self.state.update(armed=True, landed_state=2)
        return {"exit_code": 0 if passed else 1, "timed_out": False}


class ReliabilitySuiteTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)

    def plan(self, chosen, **kwargs):
        return suite.build_plan(self.directory / "new-suite", chosen, **kwargs)

    def run_local(self, plan, operations):
        with contextlib.redirect_stdout(io.StringIO()):
            return suite.run_suite(plan, operations)

    def test_default_plan_runs_qwen_three_seed_routes_and_all_required_faults(self):
        plan = suite.build_plan(self.directory / "planned")
        self.assertEqual(len(plan["cases"]), 11)
        self.assertEqual(plan["cases"][0]["case"], "qwen_baseline")
        self.assertEqual([item["case"] for item in plan["cases"]][1:4], ["seed_17", "seed_73", "seed_211"])
        self.assertTrue(any(item["case"] == "vio_reset" for item in plan["cases"]))
        self.assertFalse(any(item["case"] == "vio_tracking_loss" for item in plan["cases"]))
        self.assertEqual(sum(item["actual_model_call"] for item in plan["cases"]), 1)
        self.assertFalse((self.directory / "planned").exists())
        for item in plan["cases"]:
            self.assertEqual(item["world_sha256"], suite.sha256(item["world"]))
            self.assertIsInstance(item["args"], list)
            self.assertNotIn("shell", item)

    def test_dry_run_issues_no_requests_processes_lock_or_output_writes(self):
        output = self.directory / "dry-only"
        with mock.patch.object(suite, "urlopen", side_effect=AssertionError("HTTP forbidden")), \
             mock.patch.object(suite.subprocess, "Popen", side_effect=AssertionError("Process forbidden")), \
             mock.patch.object(suite.subprocess, "run", side_effect=AssertionError("Process forbidden")), \
             mock.patch.object(suite, "suite_lock", side_effect=AssertionError("Lock write forbidden")), \
             contextlib.redirect_stdout(io.StringIO()) as stream:
            self.assertEqual(suite.main(["--dry-run", "--case", "lidar_drop", "--output", str(output)]), 0)
        plan = json.loads(stream.getvalue())
        self.assertFalse(plan["actual_requests"])
        self.assertFalse(plan["actual_processes"])
        self.assertFalse(output.exists())

    def test_existing_evidence_and_duplicate_selection_are_refused(self):
        output = self.directory / "prior-evidence"
        output.mkdir()
        with self.assertRaises(ValueError):
            suite.build_plan(output, ["lidar_drop"])
        with self.assertRaises(ValueError):
            self.plan(["lidar_drop", "lidar_drop"])

    def test_explicit_repeats_have_distinct_outputs_and_do_not_add_hidden_retries(self):
        plan = self.plan(["lidar_invalid"], repeat=2)
        self.assertEqual([item["id"] for item in plan["cases"]], ["lidar_invalid_01", "lidar_invalid_02"])
        self.assertEqual(plan["automatic_failure_retries"], 0)
        operations = LocalOperations(failures=["lidar_invalid_01"])
        result = self.run_local(plan, operations)
        self.assertEqual(result["status"], "completed_with_failures")
        self.assertEqual(len([c for c in operations.calls if c[0] == "runner"]), 2)
        self.assertEqual([c["runner_exit_code"] for c in result["cases"]], [1, 0])

    def test_only_navigation_removes_qwen_without_dropping_selected_cases(self):
        plan = self.plan(["qwen_baseline", "seed_73", "vio_tracking_loss"], only_navigation=True)
        self.assertEqual(plan["requested_case_ids"], ["seed_73", "vio_tracking_loss"])
        self.assertTrue(all(not item["actual_model_call"] for item in plan["cases"]))

    def test_safe_failure_contract_pass_does_not_claim_requested_mission_completed(self):
        plan = self.plan(["lidar_drop", "seed_17"])
        operations = LocalOperations(expected_failures=["lidar_drop_01"])
        result = self.run_local(plan, operations)
        self.assertEqual(result["status"], "passed")
        self.assertTrue(result["cases"][0]["expected_safety_contract_pass"])
        self.assertFalse(result["cases"][0]["requested_mission_completed"])
        self.assertEqual(result["requested_mission_completed_count"], 1)
        self.assertEqual(result["expected_safety_contract_pass_count"], 2)
        self.assertEqual(result["restore"]["status"], "ready")
        starts = [call for call in operations.calls if call[0] == "start"]
        self.assertEqual([call[2] for call in starts], [True, True, False])
        self.assertEqual(starts[-1][1], str(suite.CANONICAL_WORLD))

    def test_failed_landed_case_keeps_failed_report_and_continues_distinct_case(self):
        plan = self.plan(["qwen_baseline", "lidar_delay"])
        operations = LocalOperations(failures=["qwen_baseline_01"])
        result = self.run_local(plan, operations)
        self.assertEqual(result["status"], "completed_with_failures")
        self.assertEqual(result["executed_case_count"], 2)
        self.assertFalse(result["cases"][0]["expected_safety_contract_pass"])
        report = suite.read_json(plan["cases"][0]["report_path"])
        self.assertFalse(report["validation_passed"])
        self.assertEqual(report["command"]["status"], "failed")
        self.assertTrue(Path(plan["cases"][0]["case_directory"], "suite_case_summary.json").exists())

    def test_armed_case_stops_suite_and_preserves_running_manager_for_recovery(self):
        plan = self.plan(["lidar_drop", "seed_17"])
        operations = LocalOperations(unsafe_after="lidar_drop_01")
        result = self.run_local(plan, operations)
        self.assertEqual(result["status"], "aborted")
        self.assertEqual(result["executed_case_count"], 1)
        self.assertEqual(result["restore"]["status"], "blocked")
        self.assertFalse(result["cases"][0]["safety_confirmed"])
        self.assertEqual(len([c for c in operations.calls if c[0] == "stop"]), 1)
        self.assertEqual(len([c for c in operations.calls if c[0] == "start"]), 1)

    def test_active_operator_command_prevents_any_manager_stop_or_start(self):
        plan = self.plan(["seed_17"])
        operations = LocalOperations(busy=True)
        result = self.run_local(plan, operations)
        self.assertEqual(result["status"], "aborted")
        self.assertEqual(result["executed_case_count"], 0)
        self.assertEqual(operations.calls, [])
        self.assertTrue(result["restore"]["initial_manager_preserved"])

    def test_failed_startup_can_restore_its_owned_landed_manager(self):
        plan = self.plan(["seed_17"])
        operations = LocalOperations(readiness_failure=True)
        result = self.run_local(plan, operations)
        self.assertEqual(result["status"], "aborted")
        self.assertEqual(result["restore"]["status"], "ready")
        self.assertEqual(result["executed_case_count"], 0)
        self.assertEqual(len([c for c in operations.calls if c[0] == "runner"]), 0)

    def test_readiness_waits_for_unknown_landed_state_and_sensor_health(self):
        state = ground()
        state["landed_state"] = 0
        self.assertFalse(suite.ready_state(state, state, state["scene"]["world_sha256"], True))
        state = ground()
        state["vio"]["ready"] = False
        self.assertFalse(suite.ready_state(state, state, state["scene"]["world_sha256"], True))
        state = ground()
        state["reliability_test"]["pending_fault"] = "lidar_drop"
        self.assertFalse(suite.ready_state(state, state, state["scene"]["world_sha256"], True))
        state = ground()
        self.assertFalse(suite.ready_state(state, state, "wrong-world", True))
        state["position_enu_m"][2] = -1.4
        self.assertFalse(suite.ready_state(state, state, state["scene"]["world_sha256"], True))
        state["armed"] = True
        with self.assertRaises(suite.SafetyError):
            suite.ready_state(state, state, state["scene"]["world_sha256"], True)

    def test_readiness_accepts_normal_ground_and_rejects_nonfinite_or_outside_height(self):
        state = ground()
        self.assertTrue(suite.ready_state(state, state, state["scene"]["world_sha256"], True))
        for height in (float("nan"), float("inf"), -.201, .201):
            with self.subTest(height=height):
                state["position_enu_m"][2] = height
                self.assertFalse(suite.ready_state(state, state, state["scene"]["world_sha256"], True))
        for height in (-.2, .2):
            with self.subTest(height=height):
                state["position_enu_m"][2] = height
                self.assertTrue(suite.ready_state(state, state, state["scene"]["world_sha256"], True))

    def test_case_report_true_cannot_override_runner_failure_exitcode(self):
        case = self.plan(["lidar_drop"])["cases"][0]
        path = Path(case["report_path"])
        path.parent.mkdir(parents=True)
        path.write_text(json.dumps({"validation_passed": True,
                                    "validation": {"original_mission_status": "failed"}}))
        result = suite.case_outcome(case, {"exit_code": 1, "timed_out": False})
        self.assertFalse(result["expected_safety_contract_pass"])
        self.assertFalse(result["requested_mission_completed"])

    def test_process_start_uses_list_arguments_new_session_and_explicit_restore_environment(self):
        operations = suite.SystemOperations()
        world = self.directory / "world with spaces $(literal).world"
        world.write_text("test only")
        with mock.patch.object(suite.subprocess, "Popen", return_value=types.SimpleNamespace(pid=10)) as popen:
            operations.start_manager(world, False, self.directory / "restore.log")
        args, kwargs = popen.call_args.args, popen.call_args.kwargs
        self.assertEqual(args[0], ["bash", str(suite.START_SCRIPT), "--world-file", str(world)])
        self.assertTrue(kwargs["start_new_session"])
        self.assertEqual(kwargs["env"]["GUI"], "true")
        self.assertEqual(kwargs["env"]["PX4_SIM_SPEED_FACTOR"], "0.35")
        self.assertEqual(kwargs["env"]["UAV_RELIABILITY_TESTING"], "false")
        self.assertNotIn("shell", kwargs)

    def test_stop_refuses_changed_manager_identity_before_any_signal(self):
        operations = suite.SystemOperations()
        expected = {"pid": 12, "start_time": "100", "run_directory": "/local/owned"}
        changed = {"pid": 13, "start_time": "101", "run_directory": "/local/operator"}
        with mock.patch.object(operations, "states", return_value=(ground(), ground())), \
             mock.patch.object(operations, "current_manager", return_value=changed), \
             mock.patch.object(suite.subprocess, "run") as run:
            with self.assertRaises(suite.SafetyError):
                operations.stop_manager(expected, self.directory / "stop.log")
        run.assert_not_called()
        self.assertFalse((self.directory / "stop.log").exists())

    def test_stop_pins_validated_run_directory_and_never_uses_latest_as_signal_target(self):
        operations = suite.SystemOperations()
        expected = {"pid": 12, "start_time": "100", "run_directory": "/local/owned with spaces"}
        with mock.patch.object(operations, "states", return_value=(ground(), ground())), \
             mock.patch.object(operations, "current_manager", return_value=expected), \
             mock.patch.object(suite, "process_identity", return_value=expected), \
             mock.patch.object(suite.subprocess, "run", return_value=types.SimpleNamespace(returncode=0)) as run:
            operations.stop_manager(expected, self.directory / "stop.log")
        self.assertEqual(run.call_args.args[0], ["bash", str(suite.STOP_SCRIPT), expected["run_directory"]])
        self.assertNotIn("shell", run.call_args.kwargs)

    def test_unowned_live_endpoint_prevents_startup_when_other_endpoint_is_offline(self):
        operations = suite.SystemOperations()
        with mock.patch.object(operations, "current_manager", return_value=None), \
             mock.patch.object(operations, "states", side_effect=OSError("web unavailable")), \
             mock.patch.object(operations, "read_state", return_value=ground()):
            with self.assertRaises(suite.SafetyError):
                operations.initial_manager()

    def configured_preflight_operations(self):
        operations = suite.SystemOperations()
        operations.px4_directory = self.directory / "PX4 with spaces"
        operations.preflight_evidence_path = self.directory / "preflight_checks.jsonl"
        identity = {"pid": 100, "start_time": "123", "run_directory": "/local/owned"}
        operations.started = types.SimpleNamespace(pid=100, returncode=None, poll=lambda: None)
        return operations, identity

    def preflight_mocks(self, operations, identity):
        stack = contextlib.ExitStack()
        stack.enter_context(mock.patch.object(operations, "current_manager", return_value=identity))
        stack.enter_context(mock.patch.object(suite, "process_identity", return_value=identity))
        stack.enter_context(mock.patch.object(suite, "owned_px4_process", return_value={"pid": 101, "ancestor_pids": [101, 100]}))
        return stack

    def test_preflight_requires_explicit_ok_in_stdout_not_only_zero_exit_status(self):
        self.assertTrue(suite.preflight_status("INFO [commander] Preflight check: OK\n", "", 0)["passed"])
        self.assertTrue(suite.preflight_status("\x1b[32mINFO Preflight check: OK\x1b[0m\n", "", 0)["passed"])
        for stdout, stderr, code in (("INFO Preflight check: FAILED", "", 0), ("commander check accepted", "", 0),
                                     ("Preflight check: OK", "", 1), ("", "Preflight check: OK", 0),
                                     ("Preflight check: OKAY", "", 0),
                                     ("Preflight check: OK", "Preflight check: FAILED", 0)):
            with self.subTest(stdout=stdout, code=code):
                self.assertFalse(suite.preflight_status(stdout, stderr, code)["passed"])

    def test_preflight_ok_records_stdout_timestamps_owned_process_and_exact_check_args(self):
        operations, identity = self.configured_preflight_operations()
        result = types.SimpleNamespace(returncode=0, stdout="INFO [commander] Preflight check: OK\n", stderr="")
        with self.preflight_mocks(operations, identity), \
             mock.patch.object(suite.subprocess, "run", return_value=result) as run:
            evidence = operations.preflight_check(identity, 20.)
        self.assertTrue(evidence["passed"])
        self.assertEqual(run.call_args.args[0], [str(operations.px4_directory / "build/px4_sitl_default/bin/px4-commander"), "check"])
        self.assertNotIn("shell", run.call_args.kwargs)
        self.assertEqual(run.call_args.kwargs["timeout"], 5.)
        rows = [json.loads(line) for line in operations.preflight_evidence_path.read_text().splitlines()]
        self.assertEqual(rows[0]["stdout"], result.stdout)
        self.assertTrue(rows[0]["passed"])
        self.assertEqual(rows[0]["manager"], identity)
        self.assertGreaterEqual(rows[0]["completed_at_wall_s"], rows[0]["started_at_wall_s"])

    def test_preflight_failed_with_exit_zero_cannot_pass_readiness_or_run_task(self):
        operations, identity = self.configured_preflight_operations()
        clock = [0.]
        def sleep(seconds):
            clock[0] += seconds
        failed = types.SimpleNamespace(returncode=0, stdout="Preflight check: FAILED\n", stderr="GPS check failed")
        with self.preflight_mocks(operations, identity), \
             mock.patch.object(operations, "states", return_value=(ground(), ground())), \
             mock.patch.object(suite.time, "monotonic", side_effect=lambda: clock[0]), \
             mock.patch.object(suite.time, "sleep", side_effect=sleep), \
             mock.patch.object(suite.subprocess, "run", return_value=failed), \
             mock.patch.object(operations, "run_runner") as run_runner:
            with self.assertRaises(TimeoutError):
                operations.wait_ready(ground()["scene"]["world_sha256"], True, 3.)
        run_runner.assert_not_called()
        rows = [json.loads(line) for line in operations.preflight_evidence_path.read_text().splitlines()]
        self.assertEqual(len(rows), 3)
        self.assertTrue(all(row["return_code"] == 0 and row["passed"] is False for row in rows))

    def test_preflight_timeout_records_error_and_stays_inside_readiness_deadline(self):
        operations, identity = self.configured_preflight_operations()
        clock = [0.]
        def timeout(args, **kwargs):
            clock[0] += kwargs["timeout"]
            raise suite.subprocess.TimeoutExpired(args, kwargs["timeout"], output=b"check waiting", stderr=b"timeout")
        with self.preflight_mocks(operations, identity), \
             mock.patch.object(operations, "states", return_value=(ground(), ground())), \
             mock.patch.object(suite.time, "monotonic", side_effect=lambda: clock[0]), \
             mock.patch.object(suite.time, "sleep", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)), \
             mock.patch.object(suite.subprocess, "run", side_effect=timeout), \
             mock.patch.object(operations, "run_runner") as run_runner:
            with self.assertRaises(TimeoutError):
                operations.wait_ready(ground()["scene"]["world_sha256"], True, 5.)
        run_runner.assert_not_called()
        self.assertEqual(clock[0], 5.)
        evidence = json.loads(operations.preflight_evidence_path.read_text())
        self.assertFalse(evidence["passed"])
        self.assertEqual(evidence["error_type"], "TimeoutExpired")
        self.assertEqual(evidence["stdout"], "check waiting")
        self.assertEqual(evidence["stderr"], "timeout")

    def test_readiness_polls_failed_preflight_until_explicit_ok_then_returns_evidence(self):
        operations, identity = self.configured_preflight_operations()
        clock = [0.]
        results = [types.SimpleNamespace(returncode=0, stdout="Preflight check: FAILED\n", stderr=""),
                   types.SimpleNamespace(returncode=0, stdout="Preflight check: OK\n", stderr="")]
        with self.preflight_mocks(operations, identity), \
             mock.patch.object(operations, "states", return_value=(ground(), ground())), \
             mock.patch.object(suite.time, "monotonic", side_effect=lambda: clock[0]), \
             mock.patch.object(suite.time, "sleep", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)), \
             mock.patch.object(suite.subprocess, "run", side_effect=results):
            manager, state = operations.wait_ready(ground()["scene"]["world_sha256"], True, 10.)
        self.assertEqual(manager, identity)
        self.assertTrue(state["px4_preflight"]["passed"])
        self.assertEqual(state["px4_preflight"]["attempt"], 2)
        rows = [json.loads(line) for line in operations.preflight_evidence_path.read_text().splitlines()]
        self.assertEqual([row["passed"] for row in rows], [False, True])


if __name__ == "__main__":
    unittest.main()
