"""Synthetic input faults must preserve the real navigation safety contract."""
import copy
import json
import math
import os
import socket
import threading
import time
import types
import unittest
from unittest import mock

from reliability_faults import ReliabilityFaults, fault_request, validate_request, read_ros_fault_parameter
from obstacle_navigation import NavigationMap
import test_bridge_navigation as navigation_contract


class Clock:
    def __init__(self):
        self.monotonic = 1000.
        self.wall = 1700000000.

    def advance(self, seconds):
        self.monotonic += seconds
        self.wall += seconds


def scan(stamp=10.):
    return {"stamp": stamp, "ranges": [float("inf")] * 360,
            "angle_min": -math.pi, "angle_increment": 2*math.pi/359,
            "range_min": .2, "range_max": 20., "frame_id": "uav_lidar"}


def sensor_pose(stamp=10., x=0., y=0., yaw=0.):
    return {"stamp": stamp, "position_m": [x, y, 3.85],
            "orientation_xyzw": [0., 0., math.sin(yaw/2), math.cos(yaw/2)]}


class ReliabilityFaultTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.parameter = {}
        self.events = []
        self.getter = mock.Mock(side_effect=lambda: self.parameter)
        self.faults = ReliabilityFaults(
            True, True, self.getter,
            lambda event, **fields: self.events.append({"event": event, **fields}),
            lambda: self.clock.monotonic, lambda: self.clock.wall)

    def request(self, kind, duration=2., identifier="case_01"):
        self.parameter = fault_request(kind, duration, identifier, self.clock.wall)

    def test_default_disabled_never_reads_ros_parameter_or_changes_scan(self):
        disabled = ReliabilityFaults(False, False, self.getter)
        source = scan()
        self.assertIs(disabled.transform_scan(source, sensor_pose(), True), source)
        self.assertIsNone(disabled.snapshot(True))
        self.getter.assert_not_called()

    def test_explicit_environment_flag_and_verified_simulation_are_required(self):
        for flag in ("", "1", "yes", "false"):
            with self.subTest(flag=flag), mock.patch.dict(os.environ, {"UAV_RELIABILITY_TESTING": flag}):
                self.assertFalse(ReliabilityFaults.from_environment(False, self.getter, None).enabled)
        with mock.patch.dict(os.environ, {"UAV_RELIABILITY_TESTING": "true"}):
            with self.assertRaisesRegex(RuntimeError, "verified local simulation"):
                ReliabilityFaults.from_environment(False, self.getter, None)
            enabled = ReliabilityFaults.from_environment(True, self.getter, None)
            self.addCleanup(enabled.close)
            self.assertTrue(enabled.enabled)

    def test_unbounded_expired_future_and_unlabelled_requests_are_rejected(self):
        self.request("lidar_drop")
        cases = [{"duration_s": 31.}, {"duration_s": float("nan")},
                 {"expires_at_wall_s": self.clock.wall-1}, {"issued_at_wall_s": self.clock.wall+10},
                 {"expires_at_wall_s": self.clock.wall+3600}, {"synthetic": False},
                 {"simulation_only": False}, {"id": "../bad"}, {"range_m": .1},
                 {"unexpected": True}]
        for change in cases:
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_request({**self.parameter, **change}, self.clock.wall)

    def test_ground_request_is_pending_and_does_not_modify_source(self):
        self.request("lidar_drop")
        source = scan()
        self.assertIs(self.faults.transform_scan(source, sensor_pose(), False), source)
        self.assertEqual(self.faults.snapshot(False)["pending_fault"], "lidar_drop")
        self.assertIsNone(self.faults.snapshot(False)["active_fault"])
        self.clock.advance(8.)
        self.assertIs(self.faults.transform_scan(source, sensor_pose(), True), source)
        self.assertFalse(any(event["event"] == "reliability_started" for event in self.events))

    def test_monotonic_expiry_recovers_with_dead_injector_and_cannot_replay(self):
        self.request("lidar_drop")
        source = scan()
        self.assertIsNone(self.faults.transform_scan(source, sensor_pose(), True))
        self.clock.advance(2.1)
        self.clock.wall -= 100.  # Epoch clock changes cannot extend the active TTL.
        self.assertIs(self.faults.transform_scan(source, sensor_pose(), True), source)
        self.clock.advance(100.)
        self.assertIs(self.faults.transform_scan(source, sensor_pose(), True), source)
        self.assertEqual(sum(event["event"] == "reliability_started" for event in self.events), 1)
        self.assertEqual(self.events[-1]["reason"], "automatic_monotonic_expiry")
        self.assertIsNone(self.faults.snapshot(True)["configuration_error"])

    def test_explicit_clear_restores_source_before_ttl(self):
        self.request("lidar_drop", 20.)
        self.faults.transform_scan(scan(), sensor_pose(), True)
        self.parameter = {}
        self.clock.advance(.11)
        source = scan()
        self.assertIs(self.faults.transform_scan(source, sensor_pose(), True), source)
        self.assertEqual(self.events[-1]["reason"], "explicit_clear")

    def test_delay_and_invalid_change_only_copied_measurements(self):
        for kind in ("lidar_delay", "lidar_invalid"):
            with self.subTest(kind=kind):
                self.request(kind, identifier=kind)
                self.clock.advance(.11)
                source, original = scan(), scan()
                result = self.faults.transform_scan(source, sensor_pose(), True)
                self.assertEqual(source, original)
                if kind == "lidar_delay":
                    self.assertEqual(result["stamp"], 8.)
                    self.assertEqual(result["ranges"], source["ranges"])
                else:
                    self.assertEqual(result["ranges"][0], -float("inf"))
                    self.assertEqual(result["stamp"], source["stamp"])
        json.dumps(self.events, allow_nan=False)

    def test_near_endpoint_is_fixed_and_has_original_ray_provenance(self):
        self.request("lidar_near_obstacle", 20.)
        source = scan()
        first = self.faults.transform_scan(source, sensor_pose(), True)
        anchor = self.faults.snapshot(True)["anchor_xy_enu_m"]
        self.assertEqual(anchor, [.7, 0.])
        self.assertEqual(source["ranges"], [float("inf")] * 360)
        self.assertEqual(sum(value < 1 for value in first["ranges"]), 3)
        self.clock.advance(.11)
        second = self.faults.transform_scan(scan(10.1), sensor_pose(10.1, x=.1, y=.05, yaw=.2), True)
        self.assertEqual(self.faults.snapshot(True)["anchor_xy_enu_m"], anchor)
        event = self.events[-1]
        self.assertEqual(event["source_stamp_sim_s"], 10.1)
        self.assertEqual(event["original_ranges"], ["+inf"] * 3)
        self.assertEqual(event["anchor_xy_enu_m"], anchor)
        for index in event["changed_ray_indices"]:
            angle = .2 + second["angle_min"] + index*second["angle_increment"]
            endpoint = [.1+second["ranges"][index]*math.cos(angle),
                        .05+second["ranges"][index]*math.sin(angle)]
            self.assertLess(math.dist(endpoint, anchor), .025)
        json.dumps(self.events, allow_nan=False)

    def test_near_injection_cannot_clear_a_nearer_real_obstacle(self):
        self.request("lidar_near_obstacle")
        source = scan()
        source["ranges"] = [.3] * 360
        result = self.faults.transform_scan(source, sensor_pose(), True)
        self.assertEqual(result["ranges"], [.3] * 360)

    def test_bad_or_misaligned_pose_leaves_scan_for_existing_supervisor_to_reject(self):
        self.request("lidar_near_obstacle")
        source = scan()
        for pose in (None, sensor_pose(9.), {**sensor_pose(), "position_m": [float("nan"), 0., 3.85]},
                     {**sensor_pose(), "orientation_xyzw": [0., 0., 0., 2.]}):
            with self.subTest(pose=pose):
                self.assertIs(self.faults.transform_scan(source, pose, True), source)
        self.assertIsNone(self.faults.snapshot(True)["anchor_xy_enu_m"])

    def test_actual_ray_mapping_blocks_and_recovers_without_score_edits(self):
        mapper = NavigationMap([-6., -6., 6., 6.])
        mapper.update_scan(scan(), sensor_pose(), self.clock.monotonic)
        self.assertTrue(mapper.segment_safe([0., 0.], [0., 0.], self.clock.monotonic))
        self.request("lidar_near_obstacle")
        for index in range(3):
            self.clock.advance(.1)
            source = scan(10.+(index+1)*.1)
            pose = sensor_pose(source["stamp"])
            transformed = self.faults.transform_scan(source, pose, True)
            mapper.update_scan(transformed, pose, self.clock.monotonic)
        self.assertFalse(mapper.segment_safe([0., 0.], [0., 0.], self.clock.monotonic))
        grid = mapper.diagnostics(self.clock.monotonic, [0., 0.], include_map=True)["grid"]
        self.assertTrue(grid["occupied_hit_evidence"])
        self.assertTrue(all(item["score"] == 8 for item in grid["occupied_hit_evidence"]))
        self.clock.advance(2.)
        for index in range(9):
            self.clock.advance(.1)
            source = scan(11.+index*.1)
            pose = sensor_pose(source["stamp"])
            self.assertIs(self.faults.transform_scan(source, pose, True), source)
            mapper.update_scan(source, pose, self.clock.monotonic)
        self.assertTrue(mapper.segment_safe([0., 0.], [0., 0.], self.clock.monotonic))
        self.assertEqual(mapper.snapshot(self.clock.monotonic)["scan_count"], 13)

    def test_vio_faults_are_explicitly_independent_of_flight_control(self):
        self.request("vio_tracking_loss")
        self.assertEqual(self.faults.vio_decision("/vins_estimator/tracking_ok", 10.), "tracking_false")
        self.assertIsNone(self.faults.transform_scan(scan(), sensor_pose(), False).get("synthetic"))
        self.assertFalse(self.events[-1]["used_for_flight_control"])
        self.request("vio_drop", identifier="vio_drop")
        self.clock.advance(.11)
        self.assertEqual(self.faults.vio_decision("/vins_estimator/odometry", 10.), "drop")
        self.request("vio_reset", identifier="vio_reset")
        self.clock.advance(.11)
        self.assertIsNone(self.faults.vio_decision("/vins_estimator/odometry", 10.))
        self.assertEqual(self.faults.snapshot()["active_fault"], "vio_reset")

    def test_disabled_mode_does_not_start_even_an_explicitly_requested_poll_worker(self):
        adapter = ReliabilityFaults(False, False, self.getter, background_poll=True)
        self.assertIsNone(adapter.poll_thread)
        self.getter.assert_not_called()

    def test_blocked_parameter_poll_holds_no_locks_and_cache_callbacks_remain_fast(self):
        entered, release = threading.Event(), threading.Event()

        def blocking_getter():
            entered.set()
            release.wait(2.)
            return {}

        adapter = ReliabilityFaults(True, True, blocking_getter, background_poll=True)
        self.addCleanup(adapter.close)
        self.addCleanup(release.set)
        self.assertTrue(entered.wait(1.))
        started = time.monotonic()
        source = scan()
        self.assertIs(adapter.transform_scan(source, sensor_pose(), True), source)
        self.assertIsNone(adapter.snapshot(True)["active_fault"])
        self.assertLess(time.monotonic()-started, .05)
        for lock in (adapter.lock, adapter.cache_lock):
            self.assertTrue(lock.acquire(timeout=.01))
            lock.release()
        started = time.monotonic()
        self.assertFalse(adapter.close(.01))
        self.assertLess(time.monotonic()-started, .05)
        release.set()
        self.assertTrue(adapter.close())
        self.assertFalse(adapter.poll_thread.is_alive())

    def test_local_xmlrpc_socket_timeout_is_bounded_for_a_stalled_master(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        release = threading.Event()

        def stalled_master():
            connection, _ = listener.accept()
            with connection:
                connection.recv(4096)
                release.wait(1.)

        worker = threading.Thread(target=stalled_master, daemon=True)
        worker.start()
        try:
            started = time.monotonic()
            with self.assertRaises((TimeoutError, OSError)):
                read_ros_fault_parameter("http://127.0.0.1:"+str(port), timeout_s=.05)
            self.assertLess(time.monotonic()-started, .2)
        finally:
            release.set()
            listener.close()
            worker.join(.5)

    def test_fault_parameter_poller_refuses_remote_master(self):
        with self.assertRaisesRegex(RuntimeError, "local ROS master"):
            read_ros_fault_parameter("http://example.com:11311")


class ReliabilityBridgeTests(unittest.TestCase):
    def setUp(self):
        fixture = navigation_contract.BridgeNavigationTests(
            "test_map_rays_clip_to_five_metres_without_mutating_sensor_message")
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.fixture, self.controller = fixture, fixture.controller
        self.parameter = {}
        self.controller.navigation_map = NavigationMap([-2.4, -7.4, 15.4, 7.4])
        self.controller.reliability_faults = ReliabilityFaults(
            True, True, lambda: self.parameter, self.controller.log,
            lambda: fixture.fixture.wall_time, lambda: fixture.fixture.wall_time+1700000000.)
        fixture.navigation_scan()

    def request(self, kind, duration=2.):
        self.parameter = fault_request(kind, duration, kind, self.fixture.fixture.wall_time+1700000000.)

    def raw_scan(self, elapsed=.1):
        fixture = self.fixture.fixture
        fixture.wall_time += elapsed
        stamp = fixture.sim_time+elapsed
        fixture.sim_time = stamp+.02
        self.fixture.set_pose(self.controller.position(), stamp=stamp, received=fixture.wall_time)
        message = navigation_contract.scan_message(stamp=stamp)
        self.controller.on_lidar(message)
        return message

    def test_bad_scans_latch_sensor_fault_and_good_scans_cannot_unlock_flight(self):
        for kind in ("lidar_invalid", "lidar_delay"):
            with self.subTest(kind=kind):
                self.controller.navigation_gate_fault = None
                self.controller.navigation_moving = True
                self.controller.target = self.controller.commanded = [1., 0., 3.5]
                before = self.controller.navigation_map.snapshot(self.fixture.fixture.wall_time)["scan_count"]
                self.request(kind)
                source = self.raw_scan()
                self.assertTrue(self.controller.lidar_error)
                self.assertEqual(self.controller.navigation_gate_fault, "sensor")
                self.assertEqual(self.controller.target, [0., 0., 3.5])
                self.assertEqual(source.ranges, [float("inf")] * 360)
                self.assertEqual(self.controller.navigation_map.snapshot(self.fixture.fixture.wall_time)["scan_count"], before)
                self.parameter = {}
                self.raw_scan(.11)
                self.assertIsNone(self.controller.lidar_error)
                self.assertEqual(self.controller.navigation_gate_fault, "sensor")

    def test_dropped_callbacks_do_not_refresh_map_or_source_age(self):
        self.request("lidar_drop")
        mapper = self.controller.navigation_map
        count = mapper.snapshot(self.fixture.fixture.wall_time)["scan_count"]
        received = self.controller.lidar_state["received"]
        for _ in range(5):
            self.raw_scan(.25)
        self.assertEqual(self.controller.lidar_state["received"], received)
        self.assertEqual(mapper.snapshot(self.fixture.fixture.wall_time)["scan_count"], count)
        self.assertFalse(mapper.snapshot(self.fixture.fixture.wall_time)["fresh"])
        self.assertEqual(self.controller.reliability_faults.snapshot(True)["dropped_count"], 5)

    def prepare_blocked_start(self, duration):
        controller = self.controller
        self.request("lidar_near_obstacle", duration)
        for _ in range(3):
            self.raw_scan()
        self.assertIsNone(controller.lidar_error)
        self.assertFalse(controller.navigation_map.segment_safe([0., 0.], [0., 0.], self.fixture.fixture.wall_time))
        controller.navigation_pausing = True
        controller.mission = {"id": "synthetic_wait_test"}
        controller.navigation_hold()
        controller.checkpoint = mock.Mock()
        controller.set_target = mock.Mock()
        controller.navigation_map.reset = mock.Mock(wraps=controller.navigation_map.reset)
        controller.navigation_map.plan = mock.Mock(wraps=controller.navigation_map.plan)
        controller.cancel = types.SimpleNamespace(wait=mock.Mock(side_effect=lambda duration: self.raw_scan(.25)))

    def test_transient_sensor_returns_exercise_bounded_wait_and_observed_recovery(self):
        self.prepare_blocked_start(2.)
        before = self.fixture.fixture.wall_time
        self.controller.wait_for_navigation_clearance()
        self.assertLess(self.fixture.fixture.wall_time-before, 8.)
        self.assertGreater(self.fixture.fixture.wall_time-before, 2.)
        self.assertTrue(self.controller.navigation_map.segment_safe([0., 0.], [0., 0.], self.fixture.fixture.wall_time))
        self.controller.set_target.assert_not_called()
        self.controller.navigation_map.reset.assert_not_called()
        self.controller.navigation_map.plan.assert_not_called()
        self.assertEqual(self.controller.target, [0., 0., 3.5])
        events = [call.args[0] for call in self.controller.log.call_args_list]
        self.assertIn("navigation_clearance_wait", events)
        self.assertIn("reliability_ended", events)

    def test_persistent_valid_returns_time_out_without_an_escape_or_map_reset(self):
        self.prepare_blocked_start(20.)
        before = self.fixture.fixture.wall_time
        with self.assertRaisesRegex(navigation_contract.bridge.MissionError, "timed out after 8 seconds"):
            self.controller.wait_for_navigation_clearance()
        self.assertEqual(self.fixture.fixture.wall_time-before, 8.)
        self.assertTrue(self.controller.navigation_map.snapshot(self.fixture.fixture.wall_time)["fresh"])
        self.assertFalse(self.controller.navigation_map.segment_safe([0., 0.], [0., 0.], self.fixture.fixture.wall_time))
        self.controller.set_target.assert_not_called()
        self.controller.navigation_map.reset.assert_not_called()
        self.controller.navigation_map.plan.assert_not_called()
        self.assertEqual(self.controller.target, [0., 0., 3.5])

    def test_vio_drop_and_tracking_loss_do_not_mutate_mavros_control_or_map(self):
        controller = self.controller
        controller.navigation_gate_fault = None
        controller.navigation_moving = True
        controller.target = controller.commanded = [1., 0., 3.5]
        controller.vio_message = object()
        controller.vio_time = self.fixture.fixture.wall_time
        controller.vio_tracking_ok = True
        controller.vio_tracking_time = self.fixture.fixture.wall_time
        previous = controller.vio_message
        self.request("vio_drop")
        self.fixture.fixture.wall_time += .11
        message = types.SimpleNamespace(header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(to_sec=lambda: 11.)))
        controller.on_vio(message)
        controller.on_vio_tracking(types.SimpleNamespace(data=False))
        self.assertIs(controller.vio_message, previous)
        self.assertTrue(controller.vio_tracking_ok)
        self.assertIsNone(controller.navigation_gate_fault)
        self.assertEqual(controller.target, [1., 0., 3.5])
        self.parameter = {}
        self.fixture.fixture.wall_time += .11
        controller.on_vio(message)
        controller.on_vio_tracking(types.SimpleNamespace(data=True))
        self.assertIs(controller.vio_message, message)
        self.assertTrue(controller.vio_tracking_ok)
        self.request("vio_tracking_loss")
        self.fixture.fixture.wall_time += .11
        source = types.SimpleNamespace(data=True)
        controller.on_vio_tracking(source)
        self.assertFalse(controller.vio_tracking_ok)
        self.assertTrue(source.data)
        self.assertIsNone(controller.navigation_gate_fault)
        self.assertEqual(controller.target, [1., 0., 3.5])

    def test_reliability_state_field_exists_only_in_explicit_test_mode(self):
        controller = self.controller
        controller.vio_snapshot = mock.Mock(return_value={"ready": True})
        controller.gazebo_snapshot = mock.Mock(return_value={"available": True})
        controller.camera_snapshot = mock.Mock(return_value={})
        self.assertTrue(controller.snapshot()["reliability_test"]["enabled"])
        controller.reliability_faults = ReliabilityFaults(False, False)
        self.assertNotIn("reliability_test", controller.snapshot())

    def test_unverified_registered_world_is_rejected_before_any_ros_flight_setup(self):
        bridge = navigation_contract.bridge
        registered = str(bridge.Path(bridge.__file__).parent / "scene/reliability/seed_17.world")
        with mock.patch.object(bridge, "load_scene", return_value={"verified": False}):
            with self.assertRaisesRegex(bridge.MissionError, "refusing unverified flight geometry"):
                bridge.Controller(world_path=registered)

    def test_rejection_diagnostic_distinguishes_offset_from_receipt_age(self):
        controller = self.controller
        controller.navigation_moving = True
        current = self.fixture.fixture.wall_time
        self.fixture.set_pose([0., 0., 3.5], stamp=10., received=current-.2)
        controller.on_lidar(navigation_contract.scan_message(stamp=10.08))
        events = [call for call in controller.log.call_args_list if call.args[0] == "lidar_scan_rejected"]
        diagnostic = events[-1].kwargs
        self.assertEqual(diagnostic["source_scan_stamp_sim_s"], 10.08)
        self.assertEqual(diagnostic["effective_scan_stamp_sim_s"], 10.08)
        local = diagnostic["local_pose_alignment"]
        self.assertAlmostEqual(local["signed_offset_sim_s"], -.08)
        self.assertAlmostEqual(local["absolute_offset_sim_s"], .08)
        self.assertAlmostEqual(local["receipt_age_wall_s_at_callback"], .2)
        self.assertAlmostEqual(local["receipt_age_wall_s_at_decision"], .2)
        self.assertEqual(local["oldest_stamp_sim_s"], 10.)
        self.assertEqual(local["latest_stamp_sim_s"], 10.)
        self.assertIn("poll_duration_wall_s", diagnostic["reliability_poll"])
        self.assertEqual(controller.navigation_gate_fault, "sensor")

    def test_scan_alignment_refreshes_pose_history_after_adapter_processing(self):
        controller = self.controller
        stamp = self.fixture.fixture.sim_time+.1
        self.fixture.fixture.sim_time = stamp+.02
        self.fixture.set_pose([0., 0., 3.5], stamp=stamp-.1, received=self.fixture.fixture.wall_time)
        actual_adapter = controller.reliability_faults.transform_scan

        def processing_updates_pose(source, sensor_pose, navigation_active):
            self.fixture.set_pose([0., 0., 3.5], stamp=stamp, received=self.fixture.fixture.wall_time)
            return actual_adapter(source, sensor_pose, navigation_active)

        controller.reliability_faults.transform_scan = mock.Mock(side_effect=processing_updates_pose)
        source = navigation_contract.scan_message(stamp=stamp)
        count = controller.navigation_map.snapshot(self.fixture.fixture.wall_time)["scan_count"]
        controller.on_lidar(source)
        self.assertIsNone(controller.lidar_error)
        self.assertEqual(controller.navigation_map.snapshot(self.fixture.fixture.wall_time)["scan_count"], count+1)
        self.assertEqual(controller.navigation_map.snapshot(self.fixture.fixture.wall_time)["scan_stamp_sim_s"], stamp)

    def test_controller_snapshot_and_scan_do_not_wait_for_blocked_test_parameter_poll(self):
        controller = self.controller
        entered, release = threading.Event(), threading.Event()

        def blocked():
            entered.set()
            release.wait(2.)
            return {}

        adapter = ReliabilityFaults(True, True, blocked, background_poll=True)
        self.addCleanup(adapter.close)
        self.addCleanup(release.set)
        controller.reliability_faults = adapter
        controller.vio_snapshot = mock.Mock(return_value={"ready": True})
        controller.gazebo_snapshot = mock.Mock(return_value={"available": True})
        controller.camera_snapshot = mock.Mock(return_value={})
        self.assertTrue(entered.wait(1.))
        started = time.perf_counter()
        controller.snapshot()
        self.raw_scan()
        self.assertLess(time.perf_counter()-started, .1)
        self.assertIsNone(controller.lidar_error)
        self.assertTrue(controller.lock.acquire(timeout=.01))
        controller.lock.release()


if __name__ == "__main__":
    unittest.main()
