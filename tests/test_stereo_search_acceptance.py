"""Read-only acceptance contracts; these tests never run a model or ROS flight."""
import contextlib
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from scripts import test_stereo_search as search
from coverage_search import navigation_goal_permission, plan_search


EPOCH = {"vio_epoch": 0, "ekf_counters": {"xy": 1, "z": 2, "heading": 3}}
HASHES = {"left": 'a'*64, "right": 'b'*64}
MODEL = "qwen3-vl-plus"


class FakeOwner:
    """Only test ownership decisions: no /proc, HTTP, or simulator inspection."""
    def __init__(self, *args):
        self.changed = False
        self.checks = 0

    def check(self):
        self.checks += 1
        if self.changed:
            raise search.SimulationOwnershipError("test manager/PX4 replaced")

    def evidence(self):
        return {"source": "offline_test_fixture", "identity_check_count": self.checks}


class FakeClock:
    """Advance bounded waits without sleeping or touching any actual service."""
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class RecoveryTransport:
    """Fake web/flight audit, preserving exact task attribution contracts."""
    def __init__(self, owner, malformed_ack=False, wrong_mission=False, other_command=False,
                 change_identity=False, wrong_prompt=False, already_landing=False, inactive=False):
        self.owner, self.paths, self.state = owner, [], fusion_state()
        self.command = None
        self.malformed_ack, self.wrong_mission, self.other_command = malformed_ack, wrong_mission, other_command
        self.change_identity, self.wrong_prompt, self.already_landing = change_identity, wrong_prompt, already_landing
        self.state_reads = 0
        self.inactive = inactive

    def __call__(self, base, path, payload=None):
        self.paths.append((path, copy.deepcopy(payload)))
        if path == "/api/state":
            self.state_reads += 1
            if self.already_landing and self.command is not None and self.state_reads >= 5:
                self.state.update(armed=False, landed_state=1)
                self.state["mission"]["status"] = "succeeded"
            self.state["landing_readiness"].update(armed=self.state["armed"], landed_state=self.state["landed_state"])
            return copy.deepcopy(self.state)
        if path == "/api/commands":
            steps = [{"action": "land"}] if self.already_landing else [{"action": "takeoff", "altitude": 3.5}]
            owned = {"id": "owned-mission", "status": "running" if self.already_landing else "failed", "steps": steps}
            self.command = {"id": "task-a", "prompt": "another prompt" if self.wrong_prompt else payload["prompt"],
                "mode": "task", "status": "failed", "error": "original test failure", "created_at": search.time.time(),
                "flight_missions": [{"path": "/mission", "wall_s": search.time.time(), "mission": owned}], "model_calls": []}
            self.state.update(armed=True, landed_state=2,
                              active_command_id="task-other" if self.other_command else None if self.inactive else "task-a",
                              mission={**owned, "id": "different-mission" if self.wrong_mission else owned["id"]})
            if self.change_identity:self.owner.changed = True
            return {"missing_id": True} if self.malformed_ack else copy.deepcopy(self.command)
        if path == "/api/commands/task-a":
            return copy.deepcopy(self.command)
        if path == "/api/land":
            if payload != {"expected_mission_id": "owned-mission", "expected_command_id": "task-a"}:
                raise AssertionError("Landing must guard both the exact current flight and submitted command")
            landing = {"id": "owned-landing", "status": "succeeded", "steps": [{"action": "land"}],
                       "parent_mission_id": "owned-mission"}
            self.state.update(armed=False, landed_state=1, mission=landing)
            return {"mission": copy.deepcopy(landing)}
        raise AssertionError("Unexpected local fixture request: " + path)


def fusion_state():
    return {"ready": True, "simulation_only": True, "armed": False, "landed_state": 1,
            "position_enu_m": [0., 0., 0.], "active_command_id": None, "agent_architecture": "single_vlm",
            "model": {"name": MODEL, "available": True}, "vision_model": {"name": MODEL},
            "camera": {side: {"available": True} for side in ("left", "right")}, "vio": {"ready": True},
            "landing_readiness": {"ready": True, "source": "actual_MAVROS_State_and_ExtendedState_receipts",
                "connected": True, "armed": False, "mode": "AUTO.LAND", "landed_state": 1,
                "heartbeat_age_wall_s": .1, "extended_state_age_wall_s": .1,
                "maximum_receipt_age_wall_s": 3., "xy_or_vio_readiness_required": False},
            "external_vision": {"required": True, "healthy": True, "used_for_flight_control": True,
                "adapter": {"mode": "fuse", "forwarding": True, "ready": True, "sitl_verified": True, "epoch": 0},
                "ekf": {"ready": True, "fusion_active": True, "local_position_reset_counters": EPOCH["ekf_counters"]}}}


def evidence():
    permission = navigation_goal_permission([-2.4, -7.4, 15.4, 7.4], 1., 1.2)
    plan = plan_search(search.BOUNDS, 1., 3.5,
                       {"width": 640, "height": 480, "K": [320., 0., 320., 0., 320., 240., 0., 0., 1.]},
                       max_observations=4, allowed_bounds_enu_m=permission["usable_goal_bounds_enu_m"],
                       navigation_permission=permission)
    observations, reports, samples, calls, flown = [], [], [], [], []
    def mission(identifier, action, started, finished, position, **parameters):
        return {"id": identifier, "status": "succeeded", "started_at": started,
                "steps": [{"action": action, **parameters}], "events": [{"action": action, "step": 1,
                    "finished_at": finished, "position_enu_m": position}]}
    flown.append(mission("takeoff", "takeoff", 0., 2., [0., 0., 3.5], altitude=3.5))
    for index, view in enumerate(plan["views"]):
        stamp, created = 10.+index, 10.+index*5
        identifier = "capture-a" if index == 0 else "capture-"+str(index)
        hashes = HASHES if index == 0 else {"left": format(index, 'x')*64, "right": format(index+4, 'x')*64}
        observer_pose = {**view["planned_pose_enu"], "stamp_sim_s": stamp, "age_seconds": .1}
        reports.append({"status": "ok", "architecture": "single_vlm", "model": MODEL, "capture_id": identifier,
            "created_at": created, "finished_at": created+1, "captured_at_sim_s": stamp, "targets": [],
            "camera": {"width": 640, "height": 480, "image_sha256": hashes["left"]}})
        observations.append({"view_id": view["id"], "capture_id": identifier, "image_sha256": hashes,
            "image_stamp_sim_s": stamp, "coordinate_epoch": EPOCH,
            "observer_position_enu_m": view["position_enu_m"], "observer_body_pose_enu": observer_pose})
        calls.extend([{"status": "ok", "model": MODEL, "input_modalities": ["text"],
                       "called_tools": [{"name": n} for n in
                           (("get_drone_state", "plan_region_search", "capture_scene") if index == 0 else ("capture_scene",))]},
                      {"status": "ok", "model": MODEL, "input_modalities": ["text", "image"],
                       "capture_ids": [identifier], "called_tools": [{"name": "report_scene"}]}])
        samples.append({"wall_time": created, "state": {"armed": True, "position_enu_m": view["position_enu_m"]}})
        flown.append(mission("goto-"+str(index), "goto", 3. if index == 0 else created-3,
                             created-2, view["position_enu_m"],
                             x=view["position_enu_m"][0], y=view["position_enu_m"][1], altitude=3.5))
    summary = {"status": "sampling_complete", "planned_views": 4, "observed_views": 4,
               "occlusion_free_coverage_proven": False, "generated_target_flight_goals": False, "targets": [],
               "observations": observations}
    command = {"status": "succeeded", "perceptions": reports, "search_plan": {
        "status": "ok", "search_id": "search-a", "query": "蓝色桶", "plan": plan}, "search_events": [
        {"tool": "search_progress", "result": {"status": "ok", "search_id": "search-a", "search": summary}}],
        "model_calls": calls}
    initial = fusion_state(); final = fusion_state()
    samples.append({"wall_time": 28., "state": {"armed": True, "position_enu_m": [0., 0., 3.5]}})
    flown.extend([mission("return", "return_home", 27., 28., [0., 0., 3.5]),
                  mission("land", "land", 29., 30., [0., 0., 0.])])
    missions = {m["id"]: m for m in flown}
    return copy.deepcopy((command, initial, final, samples, missions))


class StereoSearchAcceptanceTests(unittest.TestCase):
    def test_cleanup_confirmation_requires_current_actual_fcu_without_xy_or_vio(self):
        state = fusion_state()
        state.update(ready=False, position_enu_m=None, vio={"ready": False},
                     mission={"id": "land-owned", "steps": [{"action": "land"}], "status": "succeeded"})
        self.assertTrue(search.landing_recovery_confirmed(state, "land-owned"))
        for field, value in (("ready", False), ("source", "fixture_or_old_state"), ("connected", False),
                             ("armed", True), ("landed_state", True), ("mode", ""),
                             ("heartbeat_age_wall_s", None), ("heartbeat_age_wall_s", True),
                             ("heartbeat_age_wall_s", float("nan")), ("heartbeat_age_wall_s", 3.001),
                             ("extended_state_age_wall_s", -1.), ("extended_state_age_wall_s", float("inf"))):
            invalid = copy.deepcopy(state); invalid["landing_readiness"][field] = value
            with self.subTest(field=field, value=value):
                self.assertFalse(search.landing_recovery_confirmed(invalid, "land-owned"))
        for change in ("missing_metadata", "other_mission", "worker_running", "armed", "unknown_ground"):
            invalid = copy.deepcopy(state)
            if change == "missing_metadata": invalid.pop("landing_readiness")
            elif change == "other_mission": invalid["mission"]["id"] = "foreign"
            elif change == "worker_running": invalid["mission"]["status"] = "running"
            elif change == "armed": invalid["armed"] = True
            else: invalid["landed_state"] = 0
            with self.subTest(change=change):
                self.assertFalse(search.landing_recovery_confirmed(invalid, "land-owned"))

    def test_owned_disarmed_running_worker_is_guarded_cancelled_even_when_already_landed(self):
        for action in ("takeoff", "return_home"):
            owner = FakeOwner(); original = RecoveryTransport(owner); clock = FakeClock()
            def transport(base, path, payload=None):
                result = original(base, path, payload)
                if path == "/api/commands":
                    # A sampled ground state cannot prove a running worker will
                    # never arm or resume. Actual accepted flight audit is needed.
                    steps = [{"action": action, **({"altitude": 3.5} if action == "takeoff" else {})}]
                    original.state.update(armed=False, landed_state=1, ready=False, position_enu_m=None, vio={"ready": False})
                    original.state["mission"].update(status="running", steps=steps)
                    original.command["flight_missions"][0]["mission"].update(status="running", steps=steps)
                return result
            with self.subTest(action=action), tempfile.TemporaryDirectory() as temporary:
                result = search.execute("unused", Path(temporary)/"exclusive", MODEL, 1., .5,
                                        transport=transport, clock=clock, sleep=clock.sleep,
                                        ownership_factory=lambda *args: owner)
                self.assertFalse(result["validation_passed"])
                self.assertTrue(result["recovery_ownership"]["proved"])
                self.assertTrue(result["recovery_landing_confirmed"])
                self.assertEqual(sum(path == "/api/land" for path, _ in original.paths), 1)
                self.assertEqual(result["recovery_timeout_wall_s"], 180.)

    def test_disarmed_foreign_or_replaced_running_worker_receives_no_cancel_or_land(self):
        for options in ({"wrong_mission": True}, {"other_command": True}):
            owner = FakeOwner(); original = RecoveryTransport(owner, **options); clock = FakeClock()
            def transport(base, path, payload=None):
                result = original(base, path, payload)
                if path == "/api/commands":
                    original.state.update(armed=False, landed_state=1)
                    original.state["mission"]["status"] = "running"
                    original.command["flight_missions"][0]["mission"]["status"] = "running"
                return result
            with self.subTest(options=options), tempfile.TemporaryDirectory() as temporary:
                result = search.execute("unused", Path(temporary)/"exclusive", MODEL, 1., .5,
                                        transport=transport, clock=clock, sleep=clock.sleep,
                                        ownership_factory=lambda *args: owner)
                self.assertFalse(result["validation_passed"])
                self.assertFalse(any(path == "/api/land" for path, _ in original.paths))
                self.assertTrue(result["recovery_skipped_unproven_or_replaced_flight"])

    def test_slow_owned_land_can_complete_after_100_seconds_within_fixed_budget(self):
        owner = FakeOwner(); original = RecoveryTransport(owner); clock = FakeClock()
        def transport(base, path, payload=None):
            result = original(base, path, payload)
            if path == "/api/land":
                original.state.update(armed=True, landed_state=2)
                original.state["mission"]["status"] = "running"
            if path == "/api/state" and original.state.get("mission", {}).get("id") == "owned-landing":
                original.state.update(armed=clock.now < 130., landed_state=2 if clock.now < 130. else 1)
                original.state["mission"]["status"] = "running" if clock.now < 130. else "succeeded"
                original.state["landing_readiness"].update(armed=original.state["armed"],landed_state=original.state["landed_state"])
                result = copy.deepcopy(original.state)
            return result
        with tempfile.TemporaryDirectory() as temporary:
            result = search.execute("unused", Path(temporary)/"exclusive", MODEL, 1., .5,
                                    transport=transport, clock=clock, sleep=clock.sleep,
                                    ownership_factory=lambda *args: owner)
        self.assertTrue(result["recovery_landing_confirmed"])
        self.assertEqual(clock.now, 130.)
        self.assertFalse(result.get("recovery_landing_timed_out", False))
        self.assertEqual(sum(path == "/api/land" for path, _ in original.paths), 1)

    def test_historical_succeeded_land_with_current_stale_fcu_is_not_confirmed(self):
        owner = FakeOwner(); original = RecoveryTransport(owner); clock = FakeClock()
        def transport(base, path, payload=None):
            result = original(base, path, payload)
            if path == "/api/state" and original.state.get("mission", {}).get("id") == "owned-landing":
                result["landing_readiness"].update(ready=False, heartbeat_age_wall_s=10., extended_state_age_wall_s=10.)
            return result
        with tempfile.TemporaryDirectory() as temporary:
            result = search.execute("unused", Path(temporary)/"exclusive", MODEL, 1., .5,
                                    transport=transport, clock=clock, sleep=clock.sleep,
                                    ownership_factory=lambda *args: owner)
        self.assertFalse(result["recovery_landing_confirmed"])
        self.assertTrue(result["recovery_landing_timed_out"])
        self.assertEqual(clock.now, 180.)
        self.assertEqual(sum(path == "/api/land" for path, _ in original.paths), 1)
        self.assertIn("original test failure", result["error"])

    def test_guarded_child_ack_requires_actual_owned_parent_and_other_operator_stops_wait(self):
        for kind in ("wrong_parent", "new_operator"):
            owner = FakeOwner(); original = RecoveryTransport(owner); clock = FakeClock()
            def transport(base, path, payload=None):
                result = original(base, path, payload)
                if path == "/api/land" and kind == "wrong_parent": result["mission"]["parent_mission_id"] = "foreign"
                if path == "/api/state" and original.state.get("mission", {}).get("id") == "owned-landing" and kind == "new_operator":
                    result["active_command_id"] = "operator-other"
                return result
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temporary:
                result = search.execute("unused", Path(temporary)/"exclusive", MODEL, 1., .5,
                                        transport=transport, clock=clock, sleep=clock.sleep,
                                        ownership_factory=lambda *args: owner)
                self.assertFalse(result.get("recovery_landing_confirmed", False))
                self.assertEqual(sum(path == "/api/land" for path, _ in original.paths), 1)
                if kind == "wrong_parent": self.assertIn("accepted mission", result["recovery_landing_error"])
                else: self.assertTrue(result["recovery_command_replaced"])

    def test_local_identity_helper_pins_real_reused_interfaces_without_path_mutation(self):
        import sys
        from scripts import run_reliability_suite as reliability
        manager = {"pid": 10, "start_time": "a", "run_directory": "/tmp/fake-owned-run", "script": "fake"}
        px4 = {"pid": 11, "start_time": "b", "state": "R", "binary": "/tmp/fake-px4"}
        operations = mock.Mock(); operations.current_manager.return_value = manager
        original_path = list(sys.path)
        with mock.patch.object(reliability, "SystemOperations", return_value=operations), \
                mock.patch.object(reliability, "owned_px4_process", return_value=px4) as owned, \
                mock.patch.object(reliability, "process_identity", return_value=manager):
            owner = search.LocalSimulationOwner("http://127.0.0.1:8080", "/tmp/fake-px4-root")
            px4["state"] = "S"  # Scheduler state is not a process replacement.
            owner.check()
            self.assertEqual(owner.evidence()["px4_process"]["pid"], 11)
            self.assertNotIn("state", owner.evidence()["px4_process"])
            operations.current_manager.return_value = {**manager, "pid": 20}
            with self.assertRaises(search.SimulationOwnershipError):owner.check()
            self.assertTrue(owned.called)
        self.assertEqual(sys.path, original_path)

    def test_existing_owned_landing_is_waited_without_duplicate_land_request(self):
        owner = FakeOwner(); transport = RecoveryTransport(owner, already_landing=True)
        with tempfile.TemporaryDirectory() as temporary:
            result = search.execute("unused", Path(temporary)/"exclusive", MODEL, 1., .5, transport=transport,
                                    ownership_factory=lambda *args: owner)
            self.assertFalse(result["validation_passed"])
            self.assertTrue(result["recovery_waited_for_owned_existing_landing"])
            self.assertTrue(result["recovery_landing_confirmed"])
            self.assertFalse(any(path == "/api/land" for path, _ in transport.paths))

    def test_owned_land_worker_still_running_on_disarmed_ground_is_waited_not_reposted(self):
        owner = FakeOwner(); original = RecoveryTransport(owner, already_landing=True); clock = FakeClock()
        def transport(base, path, payload=None):
            result = original(base, path, payload)
            if path == "/api/commands":
                original.state.update(armed=False, landed_state=1, ready=False, position_enu_m=None)
            if path == "/api/state" and original.command is not None:
                result["mission"]["status"] = "running" if clock.now < 1. else "succeeded"
            return result
        with tempfile.TemporaryDirectory() as temporary:
            result = search.execute("unused", Path(temporary)/"exclusive", MODEL, 1., .5,
                                    transport=transport, clock=clock, sleep=clock.sleep,
                                    ownership_factory=lambda *args: owner)
        self.assertTrue(result["recovery_waited_for_owned_existing_landing"])
        self.assertTrue(result["recovery_landing_confirmed"])
        self.assertEqual(clock.now, 1.)
        self.assertFalse(any(path == "/api/land" for path, _ in original.paths))

    def test_existing_supervisor_child_land_only_waits_with_actual_owned_parent(self):
        owner = FakeOwner(); original = RecoveryTransport(owner, already_landing=True)
        def transport(base, path, payload=None):
            result = original(base, path, payload)
            if path == "/api/state" and original.command is not None:
                result["mission"].update(id="supervisor-land", parent_mission_id="owned-mission")
            return result
        with tempfile.TemporaryDirectory() as temporary:
            result = search.execute("unused", Path(temporary)/"exclusive", MODEL, 1., .5,
                                    transport=transport, ownership_factory=lambda *args: owner)
            self.assertFalse(result["validation_passed"])
            self.assertTrue(result["recovery_waited_for_owned_existing_landing"])
            self.assertTrue(result["recovery_landing_confirmed"])
            self.assertFalse(any(path == "/api/land" for path, _ in original.paths))
        for parent, status in (("foreign", "running"), ("owned-mission", "failed")):
            state = {"active_command_id": "task-a", "mission": {"id": "child", "steps": [{"action": "land"}],
                       "parent_mission_id": parent, "status": status}}
            self.assertFalse(search.recovery_ownership(state, original.command, "task-a",
                                                       original.command["prompt"])[0])

    def test_owned_failed_task_has_real_guarded_cleanup_and_retains_failure(self):
        owner = FakeOwner(); transport = RecoveryTransport(owner)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)/"exclusive"
            result = search.execute("unused", output, MODEL, 1., .5, transport=transport,
                                    ownership_factory=lambda *args: owner)
            self.assertFalse(result["validation_passed"])
            self.assertIn("original test failure", result["error"])
            self.assertTrue(result["recovery_ownership"]["proved"])
            self.assertTrue(result["recovery_landing_confirmed"])
            self.assertEqual(sum(path == "/api/land" for path, _ in transport.paths), 1)
            self.assertEqual(sum(path == "/api/commands" for path, _ in transport.paths), 1)
            self.assertEqual(json.loads((output/"authorized_recovery_landing.json").read_text())["mission"]["id"], "owned-landing")
            self.assertGreater(owner.checks, len(transport.paths))

    def test_changed_manager_identity_stops_all_further_requests_and_never_lands(self):
        owner = FakeOwner(); transport = RecoveryTransport(owner, change_identity=True)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)/"exclusive"
            result = search.execute("unused", output, MODEL, 1., .5, transport=transport,
                                    ownership_factory=lambda *args: owner)
            self.assertFalse(result["validation_passed"])
            self.assertTrue(result["recovery_skipped_identity_change"])
            self.assertEqual([path for path, _ in transport.paths], ["/api/state", "/api/commands"])
            self.assertEqual(json.loads((output/"accepted_task.json").read_text())["id"], "task-a")

    def test_other_bridge_mission_or_operator_command_cannot_receive_recovery_landing(self):
        for options in ({"wrong_mission": True}, {"wrong_mission": True, "inactive": True},
                        {"other_command": True}, {"wrong_prompt": True}):
            with self.subTest(options=options), tempfile.TemporaryDirectory() as temporary:
                owner = FakeOwner(); transport = RecoveryTransport(owner, **options)
                result = search.execute("unused", Path(temporary)/"exclusive", MODEL, 1., .5, transport=transport,
                                        ownership_factory=lambda *args: owner)
                self.assertFalse(result["validation_passed"])
                self.assertFalse(any(path == "/api/land" for path, _ in transport.paths))

    def test_completed_failed_command_can_land_only_its_own_current_bridge_mission(self):
        owner = FakeOwner(); transport = RecoveryTransport(owner, inactive=True)
        with tempfile.TemporaryDirectory() as temporary:
            result = search.execute("unused", Path(temporary)/"exclusive", MODEL, 1., .5, transport=transport,
                                    ownership_factory=lambda *args: owner)
            self.assertTrue(result["recovery_ownership"]["proved"])
            self.assertTrue(result["recovery_landing_confirmed"])
            self.assertEqual(sum(path == "/api/land" for path, _ in transport.paths), 1)

    def test_malformed_ack_can_reconcile_only_exact_unique_task_and_current_flight_audit(self):
        for wrong_prompt in (False, True):
            with self.subTest(wrong_prompt=wrong_prompt), tempfile.TemporaryDirectory() as temporary:
                owner = FakeOwner(); transport = RecoveryTransport(owner, malformed_ack=True, wrong_prompt=wrong_prompt)
                output = Path(temporary)/"exclusive"
                result = search.execute("unused", output, MODEL, 1., .5, transport=transport,
                                        ownership_factory=lambda *args: owner)
                self.assertFalse(result["validation_passed"])
                self.assertTrue(result["submission_result_unknown"])
                self.assertEqual(json.loads((output/"accepted_task.json").read_text()), {"missing_id": True})
                self.assertEqual(sum(path == "/api/land" for path, _ in transport.paths), 0 if wrong_prompt else 1)
                if not wrong_prompt:
                    self.assertTrue(result["submission_owner_reconciled"])
                    self.assertTrue(result["recovery_landing_confirmed"])

    def test_vehicle_outside_target_roi_is_valid_but_inside_permission_and_actual_fov_are_required(self):
        values = evidence()
        actual = values[0]["search_plan"]
        self.assertLess(actual["plan"]["views"][0]["position_enu_m"][0], 0.)
        self.assertTrue(search.check_live_plan(actual))
        invalid = copy.deepcopy(actual)
        invalid["plan"]["views"][0]["position_enu_m"][0] = -1.3
        with self.assertRaisesRegex(ValueError, "flight permission"):
            search.check_live_plan(invalid)
        invalid = copy.deepcopy(actual)
        view = invalid["plan"]["views"][0]
        view["position_enu_m"] = [3.5, .5, 3.5]
        view["planned_pose_enu"]["position_m"] = list(view["position_enu_m"])
        with self.assertRaisesRegex(ValueError, "actually show"):
            search.check_live_plan(invalid)
        values[0]["search_events"][0]["result"]["search"]["observations"][0]["observer_body_pose_enu"]["orientation_xyzw"] = [0., 0., 1., 0.]
        with self.assertRaisesRegex(ValueError, "captured attitude"):
            search.audit_search(*values, MODEL)

    def test_outside_or_uncertain_target_cannot_be_used_for_early_found(self):
        for position, covariance in (([6., 1., 0.], .01), ([4., 1., 0.], 1.)):
            values = evidence()
            summary = values[0]["search_events"][0]["result"]["search"]
            summary.update(status="target_found", targets=[{"label": "蓝色桶", "distinct_viewpoint_count": 2,
                "position_enu_m": position, "covariance_enu_m2": [[covariance, 0., 0.], [0., covariance, 0.], [0., 0., covariance]],
                "requested_region_membership": {"confirmed_inside": True}}])
            with self.subTest(position=position), self.assertRaisesRegex(ValueError, "uncertainty containment"):
                search.audit_search(*values, MODEL)

    def test_actual_sampling_without_visible_target_is_a_valid_outcome(self):
        result = search.audit_search(*evidence(), MODEL)
        self.assertTrue(result["passed"])
        self.assertEqual(result["actual_observed_views"], 4)
        self.assertEqual(result["metric_available_count"], 0)
        self.assertFalse(result["target_required_to_be_visible"])
        self.assertFalse(result["occlusion_free_coverage_proven"])

    def test_quality_rejection_is_retained_without_inventing_target_coordinates(self):
        values = evidence()
        metric = {"status": "unavailable", "reason": "insufficient texture", "capture_pair_id": "pair-a", "image_stamp_sim_s": 10.}
        values[0]["perceptions"][0]["targets"] = [{"label": "possibly blue barrel", "stereo_location": metric}]
        result = search.audit_search(*values, MODEL)
        self.assertEqual(result["metric_results"][0]["status"], "unavailable")
        metric["position_enu_m"] = [3., 1., 0.]
        with self.assertRaisesRegex(ValueError, "fabricated"):
            search.audit_search(*values, MODEL)

    def test_valid_historical_geometry_requires_uncertainty_and_exact_same_frame_provenance(self):
        values = evidence()
        metric = {"status": "ok", "capture_pair_id": "pair-a", "image_stamp_sim_s": 10.,
                  "method": "stereo_sgbm_robust_surface", "frame": "local_enu", "historical_observation": True,
                  "usable_as_flight_waypoint": False, "coordinate_epoch": EPOCH, "image_sha256": HASHES,
                  "uncertainty_statistics_calibrated": False,
                  "position_enu_m": [4., 1., 0.], "covariance_enu_m2": [[.04, 0., 0.], [0., .04, 0.], [0., 0., .04]],
                  "sigma_enu_m": [.2, .2, .2]}
        values[0]["perceptions"][0]["targets"] = [{"label": "蓝色桶", "stereo_location": metric}]
        self.assertEqual(search.audit_search(*values, MODEL)["metric_available_count"], 1)
        metric["sigma_enu_m"][0] = 0.
        with self.assertRaisesRegex(ValueError, "uncertainty"):
            search.audit_search(*values, MODEL)

    def test_missing_actual_plan_budget_violation_or_wrong_capture_hash_cannot_pass(self):
        for mutate in (lambda c: c.pop("search_plan"),
                       lambda c: c["search_plan"]["plan"].update(max_observations=5),
                       lambda c: c["perceptions"][0]["camera"].update(image_sha256='c'*64),
                       lambda c: c["search_events"][0]["result"]["search"].update(occlusion_free_coverage_proven=True)):
            values = evidence(); mutate(values[0])
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                search.audit_search(*values, MODEL)

    def test_default_plan_never_requests_or_writes_and_execute_directory_is_exclusive(self):
        with mock.patch.object(search, "request", side_effect=AssertionError("HTTP forbidden")), \
                mock.patch.object(Path, "mkdir", side_effect=AssertionError("write forbidden")), \
                contextlib.redirect_stdout(io.StringIO()) as capture:
            self.assertEqual(search.main([]), 0)
        self.assertFalse(json.loads(capture.getvalue())["actual_requests"])
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(FileExistsError):
                search.execute("unused", Path(temporary), MODEL, 1., .5,
                               transport=lambda *args: self.fail("HTTP before exclusive directory creation"))

    def test_shadow_preflight_failure_is_preserved_without_submit_or_landing(self):
        state = fusion_state(); state["external_vision"].update(required=False, used_for_flight_control=False)
        state["external_vision"]["adapter"]["mode"] = "shadow"
        paths = []
        def transport(base, path, payload=None):
            paths.append(path)
            self.assertIsNone(payload)
            return copy.deepcopy(state)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "exclusive"
            result = search.execute("unused", output, MODEL, 1., .5, transport=transport,
                                    ownership_factory=FakeOwner)
            self.assertFalse(result["validation_passed"])
            self.assertFalse(result["actual_task_submitted"])
            self.assertEqual(paths, ["/api/state"])
            saved = json.loads((output / "validation.json").read_text())
            self.assertIn("Actual healthy EV fusion", saved["error"])


if __name__ == "__main__":
    unittest.main()
