"""LiDAR flight supervisor contracts, exercised without ROS or a simulator."""
import math
import threading
import types
import unittest
from collections import deque
from unittest import mock

import test_bridge_camera as camera_contract


bridge = camera_contract.bridge
IDENTITY = [0., 0., 0., 1.]


def navigation_scene():
    # Public permission geometry only: no obstacle positions or occupancy truth.
    return {"id": "test-autonomous-yard", "verified": True,
            "ground_plane": {"known": True, "z_m": .003,
                             "bounds_xy_m": [-20., -20., 20., 20.]},
            "flight_region": {"frame": "gazebo_world", "bounds_min_m": [-3., -8., -.2],
                              "bounds_max_m": [16., 8., 6.], "margin_m": .6},
            "navigation": {"enabled": True, "method": "lidar_grid",
                           "bounds_xy_m": [-3., -8., 16., 8.],
                           "flight_altitude_m": 3.5, "minimum_clearance_m": 1.},
            "takeoff_pad": {"center_world_m": [0., 0., .003],
                            "radius_m": 1., "vertical_clearance_m": 6.}}


def pose_message(position, orientation=IDENTITY, stamp=10.):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: stamp)),
        pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=position[0], y=position[1], z=position[2]),
            orientation=types.SimpleNamespace(x=orientation[0], y=orientation[1],
                                              z=orientation[2], w=orientation[3])))


def scan_message(ranges=None, stamp=10., frame="uav_lidar"):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: stamp), frame_id=frame),
        ranges=[float("inf")] * 360 if ranges is None else ranges,
        angle_min=-math.pi, angle_increment=2 * math.pi / 359,
        range_min=.2, range_max=20.)


def rpy_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]


class BridgeNavigationTests(unittest.TestCase):
    def setUp(self):
        fixture = camera_contract.CameraCaptureTests(
            "test_real_frame_and_calibration_remain_one_immutable_snapshot")
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.fixture, self.controller = fixture, fixture.controller
        controller = self.controller
        controller.scene = navigation_scene()
        controller.navigation_map = mock.Mock()
        controller.navigation_active = True
        controller.navigation_moving = False
        controller.navigation_pausing = False
        controller.navigation_gate_error = None
        controller.navigation_gate_fault = None
        controller.navigation_trace = []
        controller.lidar_state = controller.lidar_error = None
        controller.submit_lock = threading.RLock()
        controller.command_lock = threading.Lock()
        controller.cancel = threading.Event()
        controller.closed = threading.Event()
        controller.ready = mock.Mock()
        controller.state_msg = types.SimpleNamespace(connected=True, armed=False, mode="POSCTL")
        controller.state_time = controller.pose_time = fixture.wall_time
        controller.landed = 1
        bridge.ExtendedState.LANDED_STATE_ON_GROUND = 1
        controller.on_extended(types.SimpleNamespace(landed_state=1))
        controller.deadline = fixture.wall_time+900.
        controller.home = controller.target = controller.commanded = controller.mission = None
        controller.worker = None
        controller.yaw = 0.
        controller.log = mock.Mock()
        controller.navigation_map.snapshot.return_value = {"fresh": True, "scan_count": 1}
        controller.navigation_map.diagnostics.return_value = {
            "fresh": True, "scan_count": 1, "grid": {"free_cells": [[1, 1]], "occupied_cells": []}}
        self.set_pose([0., 0., 3.5])
        # These stubs are local to the isolated bridge module, never a ROS node.
        shutdown_patch = mock.patch.object(bridge.rospy, "is_shutdown", return_value=False, create=True)
        shutdown_patch.start()
        self.addCleanup(shutdown_patch.stop)

    def set_pose(self, position, orientation=IDENTITY, stamp=10., received=999.95):
        self.controller.pose = pose_message(position, orientation, stamp)
        self.controller.local_pose_history = deque([{
            "stamp_sim_s": stamp, "received": received, "position_m": list(position),
            "orientation_xyzw": list(orientation)}])
        self.controller.gazebo_history = deque([{
            "stamp_sim_s": stamp, "received": received,
            "position_m": [position[0], position[1], position[2] + .19],
            "orientation_xyzw": list(orientation), "linear_velocity_m_s": [0., 0., 0.]}])
        self.controller.gazebo_time = received

    def navigation_scan(self, ranges=None, elapsed=.1):
        self.fixture.wall_time += elapsed
        stamp = self.fixture.sim_time + elapsed
        self.fixture.sim_time = stamp + .02
        self.set_pose(self.controller.position(), stamp=stamp, received=self.fixture.wall_time)
        self.controller.on_lidar(scan_message(ranges, stamp=stamp))
        self.assertIsNone(self.controller.lidar_error)

    def test_pose_callback_preserves_full_orientation_for_scan_alignment(self):
        q = rpy_quaternion(.08, -.06, .4)
        message = pose_message([1., 2., 3.5], q, 10.1)
        self.controller.on_pose(message)
        recorded = self.controller.local_pose_history[-1]
        self.assertEqual(recorded["position_m"], [1., 2., 3.5])
        self.assertEqual(recorded["orientation_xyzw"], q)
        self.assertEqual(recorded["stamp_sim_s"], 10.1)
        self.assertEqual(recorded["received"], self.fixture.wall_time)

    def test_map_rays_clip_to_five_metres_without_mutating_sensor_message(self):
        values = [2., 5., 7., 20., float("inf"), float("nan"), float("-inf"), .1] + [20.] * 352
        message = scan_message(values)
        self.controller.on_lidar(message)
        mapped, pose, received = self.controller.navigation_map.update_scan.call_args.args
        self.assertEqual(mapped["ranges"][:5], [2., 5., 5., 5., 5.])
        self.assertTrue(math.isnan(mapped["ranges"][5]))
        self.assertEqual(mapped["ranges"][6:8], [float("-inf"), .1])
        self.assertEqual(mapped["range_max"], 5.)
        self.assertEqual(mapped["range_min"], .2)
        self.assertEqual(mapped["angle_increment"], message.angle_increment)
        self.assertEqual(values[:5], [2., 5., 7., 20., float("inf")])
        self.assertEqual(pose["position_m"], [0., 0., 3.85])
        self.assertEqual(received, self.fixture.wall_time)
        self.assertIsNone(self.controller.lidar_error)

    def test_sensor_origin_uses_nearest_timestamp_and_full_quaternion_extrinsic(self):
        q = rpy_quaternion(.12, 0., math.pi / 2)
        self.set_pose([1., 2., 3.5], q, stamp=10.01)
        self.controller.local_pose_history.append({
            "stamp_sim_s": 10.08, "received": 999.95,
            "position_m": [50., 50., 3.5], "orientation_xyzw": list(IDENTITY)})
        self.controller.on_lidar(scan_message(stamp=10.))
        _, pose, _ = self.controller.navigation_map.update_scan.call_args.args
        # A roll followed by 90-degree yaw rotates the vertical lever arm into +X.
        expected = [1. + .35 * math.sin(.12), 2., 3.5 + .35 * math.cos(.12)]
        for actual, wanted in zip(pose["position_m"], expected):
            self.assertAlmostEqual(actual, wanted)
        self.assertEqual(pose["orientation_xyzw"], q)
        self.assertEqual(pose["stamp"], 10.01)

    def test_mapping_starts_only_after_navigation_is_active(self):
        self.controller.navigation_active = False
        self.set_pose([0., 0., 0.])
        self.controller.on_lidar(scan_message())
        self.controller.navigation_map.update_scan.assert_not_called()
        self.assertEqual(self.controller.lidar_state["sample_count"], 360)
        self.assertIsNone(self.controller.lidar_error)

    def test_initial_navigation_requires_five_accepted_and_fresh_scans(self):
        controller = self.controller
        mapper = bridge.NavigationMap([-2.4, -7.4, 15.4, 7.4])
        controller.navigation_active = False
        controller.navigation_map = None
        controller.navigation_gate_fault = "sensor"
        controller.navigation_gate_error = "previous mission scan fault"
        controller.lidar_error = "previous mission stale scan"

        def wait_for_map(ready, timeout, require_offboard=False):
            self.assertEqual(timeout, 5)
            self.assertTrue(require_offboard)
            self.assertIsNone(controller.navigation_gate_fault)
            self.assertIsNone(controller.navigation_gate_error)
            self.assertIsNone(controller.lidar_error)
            self.assertFalse(ready())
            for count in range(1, 6):
                stamp = 10. + count * .1
                self.fixture.sim_time = stamp + .02
                self.fixture.wall_time += .05
                self.set_pose([0., 0., 3.5], stamp=stamp, received=self.fixture.wall_time)
                controller.on_lidar(scan_message(stamp=stamp))
                self.assertEqual(mapper.snapshot(self.fixture.wall_time)["scan_count"], count)
                self.assertEqual(ready(), count >= 5)
            self.fixture.wall_time += 1.1
            self.assertFalse(ready(), "five old scans cannot authorize navigation")

        controller.wait_until = mock.Mock(side_effect=wait_for_map)
        with mock.patch.object(bridge, "NavigationMap", return_value=mapper) as constructor:
            controller.start_navigation()
            constructor.assert_called_once()
        self.assertTrue(controller.navigation_active)
        self.assertEqual(mapper.snapshot(self.fixture.wall_time)["scan_count"], 5)

    def test_wrong_frame_never_enters_map(self):
        self.controller.on_lidar(scan_message(frame="base_link"))
        self.controller.navigation_map.update_scan.assert_not_called()
        self.assertIn("frame", self.controller.lidar_error)

    def test_stale_unaligned_and_future_measurements_never_enter_map(self):
        cases = [(10.06, 999.95, 10.), (10., 998.9, 10.),
                 (9.5, 999.95, 9.5), (10.2, 999.95, 10.2)]
        for pose_stamp, receipt, scan_stamp in cases:
            with self.subTest(pose_stamp=pose_stamp, receipt=receipt, scan_stamp=scan_stamp):
                self.set_pose([0., 0., 3.5], stamp=pose_stamp, received=receipt)
                self.controller.navigation_map.reset_mock()
                self.controller.on_lidar(scan_message(stamp=scan_stamp))
                self.controller.navigation_map.update_scan.assert_not_called()
                self.assertIsNotNone(self.controller.lidar_error)

    def test_height_gate_uses_absolute_body_height_before_lidar_offset(self):
        for height in (3.25, 3.5, 3.75):
            with self.subTest(accepted_height=height):
                self.set_pose([0., 0., height])
                self.controller.navigation_map.reset_mock()
                self.controller.on_lidar(scan_message())
                self.controller.navigation_map.update_scan.assert_called_once()
                self.assertIsNone(self.controller.lidar_error)
        for height in (3.249, 3.751):
            with self.subTest(rejected_height=height):
                self.set_pose([0., 0., height])
                self.controller.navigation_map.reset_mock()
                self.controller.on_lidar(scan_message())
                self.controller.navigation_map.update_scan.assert_not_called()
                self.assertIn("cruise height", self.controller.lidar_error)

    def test_safe_enu_height_cannot_mask_a_too_high_world_lidar_origin(self):
        self.set_pose([0., 0., 3.5])
        self.controller.gazebo_history[0]["position_m"][2] = 4.4
        self.controller.on_lidar(scan_message())
        self.controller.navigation_map.update_scan.assert_not_called()
        self.assertIsNotNone(self.controller.lidar_error)

    def test_world_lidar_origin_below_proven_scan_band_is_rejected(self):
        self.set_pose([0., 0., 3.5])
        self.controller.gazebo_history[0]["position_m"][2] = 2.1
        self.controller.on_lidar(scan_message())
        self.controller.navigation_map.update_scan.assert_not_called()
        self.assertIsNotNone(self.controller.lidar_error)

    def test_missing_stale_or_unaligned_own_world_pose_never_enters_map(self):
        for invalid in ("missing", "stale", "unaligned"):
            with self.subTest(invalid=invalid):
                self.set_pose([0., 0., 3.5])
                if invalid == "missing":
                    self.controller.gazebo_history.clear()
                elif invalid == "stale":
                    self.controller.gazebo_history[0]["received"] = 998.9
                else:
                    self.controller.gazebo_history[0]["stamp_sim_s"] = 10.06
                self.controller.navigation_map.reset_mock()
                self.controller.on_lidar(scan_message())
                self.controller.navigation_map.update_scan.assert_not_called()
                self.assertIsNotNone(self.controller.lidar_error)

    def test_world_height_gate_uses_nearest_pose_and_normalized_full_orientation(self):
        q = rpy_quaternion(0., .19, .6)
        # At the upper boundary, adding an unrotated .35 m offset or rotating
        # with the slightly scaled quaternion would wrongly reject this pose.
        self.set_pose([1., 2., 3.5], q, stamp=10.01)
        world = self.controller.gazebo_history[0]
        world["position_m"][2] = 3.954
        world["orientation_xyzw"] = [value * 1.01 for value in q]
        self.controller.gazebo_history.append({
            "stamp_sim_s": 10.08, "received": 999.95,
            "position_m": [1., 2., 9.], "orientation_xyzw": list(IDENTITY)})
        self.assertLess(3.954 + .35 * math.cos(.19), 4.3)
        self.controller.on_lidar(scan_message())
        self.controller.navigation_map.update_scan.assert_called_once()
        self.assertIsNone(self.controller.lidar_error)
        _, aligned, _ = self.controller.navigation_map.update_scan.call_args.args
        # Occupancy still uses local ENU coordinates after the world-Z safety
        # check; the world height must not be substituted into the local map.
        self.assertAlmostEqual(aligned["position_m"][2], 3.5 + .35 * math.cos(.19))
        self.assertEqual(aligned["orientation_xyzw"], q)

    def test_invalid_rays_or_combined_tilt_leave_real_map_unmodified(self):
        cases = [(IDENTITY, [float("nan")] * 360),
                 (rpy_quaternion(.16, .16, 0.), [float("inf")] * 360),
                 ([0., 0., 0., 2.], [float("inf")] * 360)]
        for q, ranges in cases:
            with self.subTest(orientation=q, invalid_ranges=math.isnan(ranges[0])):
                mapper = bridge.NavigationMap([-3., -8., 16., 8.])
                self.controller.navigation_map = mapper
                self.set_pose([0., 0., 3.5], q)
                self.controller.on_lidar(scan_message(ranges))
                status = mapper.snapshot(self.fixture.wall_time)
                self.assertEqual(status["scan_count"], 0)
                self.assertEqual(status["known_cells"], 0)
                self.assertFalse(status["fresh"])
                self.assertIsNotNone(self.controller.lidar_error)

    def test_rejected_scan_freezes_motion_and_latch_survives_a_valid_followup(self):
        controller = self.controller
        controller.navigation_moving = True
        controller.target, controller.commanded = [5., 0., 3.5], [.5, 0., 3.5]
        controller.on_lidar(scan_message(frame="wrong"))
        first_error = controller.navigation_gate_error
        self.assertIn("rejected", first_error)
        self.assertEqual(controller.target, controller.position())
        self.assertEqual(controller.commanded, controller.position())
        controller.navigation_map.update_scan.assert_not_called()
        controller.on_lidar(scan_message(stamp=10.01))
        controller.navigation_map.update_scan.assert_called_once()
        self.assertIsNone(controller.lidar_error)
        self.assertEqual(controller.navigation_gate_error, first_error)
        self.assertEqual(controller.navigation_gate_fault, "sensor")
        self.assertEqual(controller.target, controller.position())

    def test_invalid_cruise_altitude_is_rejected_before_a_worker_exists(self):
        for action in ("takeoff", "goto"):
            with self.subTest(action=action):
                self.controller.mission = None
                if action == "takeoff":
                    self.set_pose([0., 0., 0.])
                    self.controller.state_msg.armed, self.controller.landed = False, 1
                    steps = [{"action": "takeoff", "altitude": 2.}]
                else:
                    self.set_pose([0., 0., 3.5])
                    self.controller.state_msg.armed, self.controller.landed = True, 2
                    steps = [{"action": "goto", "x": 1., "y": 0., "altitude": 2.}]
                with mock.patch.object(bridge.threading, "Thread") as worker:
                    with self.assertRaisesRegex(bridge.MissionError, "fixed cruise altitude"):
                        self.controller.submit(steps)
                    worker.assert_not_called()
                self.assertIsNone(self.controller.mission)

    def test_valid_sortie_can_be_submitted_before_the_after_takeoff_scan(self):
        self.set_pose([0., 0., 0.])
        self.controller.navigation_active = False
        self.controller.navigation_map = None
        with mock.patch.object(bridge.threading, "Thread") as worker:
            result = self.controller.submit([
                {"action": "takeoff", "altitude": 3.5},
                {"action": "goto", "x": 13., "y": 0., "altitude": 3.5},
                {"action": "return_home"}, {"action": "land"}])
            self.assertEqual(result["mission"]["status"], "running")
            worker.return_value.start.assert_called_once()
        self.assertIsNone(self.controller.lidar_state)

    def test_legacy_scene_keeps_arbitrary_valid_altitude_and_direct_executor(self):
        self.controller.scene.pop("navigation")
        self.set_pose([0., 0., 0.])
        with mock.patch.object(bridge.threading, "Thread") as worker:
            result = self.controller.submit([
                {"action": "takeoff", "altitude": 2.},
                {"action": "goto", "x": 2., "y": 0., "altitude": 2.}, {"action": "land"}])
            worker.return_value.start.assert_called_once()
        self.assertEqual(result["mission"]["steps"][0]["altitude"], 2.)
        self.controller.move_direct = mock.Mock()
        self.controller.move([2., 0., 2.])
        self.controller.move_direct.assert_called_once_with([2., 0., 2.])
        self.controller.navigation_map.plan.assert_not_called()

    def test_worker_stops_on_a_latched_gate_even_when_next_map_check_would_pass(self):
        controller = self.controller
        controller.home = [0., 0., 0.]
        controller.mission = {"id": "test-mission"}
        controller.checkpoint = mock.Mock()
        controller.navigation_map.plan.return_value = {
            "waypoints": [[2., 0.]], "route_complete": True}
        controller.navigation_map.segment_safe.return_value = True

        def set_target_and_latch(target):
            controller.target = list(target)
            controller.navigation_gate_error = "publisher detected stale scan"
            controller.navigation_gate_fault = "sensor"

        controller.set_target = mock.Mock(side_effect=set_target_and_latch)
        with self.assertRaisesRegex(bridge.MissionError, "gate closed"):
            controller.move([2., 0., 3.5])
        self.assertEqual(controller.target, controller.position())
        self.assertEqual(controller.commanded, controller.position())
        self.assertFalse(controller.navigation_moving)
        self.assertEqual(controller.navigation_map.plan.call_count, 1)
        self.assertIn("stale scan", controller.navigation_gate_error)

    def test_valid_updated_map_holds_and_settles_before_replanning_same_goal(self):
        controller = self.controller
        controller.home = [0., 0., 0.]
        controller.mission = {"id": "test-mission"}
        controller.checkpoint = mock.Mock()
        controller.submit = mock.Mock(side_effect=AssertionError("must not retry a mission"))
        events, settled = [], [False]

        def plan(start, goal, now):
            events.append("plan")
            self.assertEqual(goal, [4., 0.])
            if events.count("plan") == 1:
                return {"waypoints": [[2., 0.]], "route_complete": False}
            self.assertTrue(settled[0], "a replacement route must wait for actual settling")
            return {"waypoints": [[4., 0.]], "route_complete": True}

        controller.navigation_map.plan.side_effect = plan
        controller.navigation_map.segment_safe.side_effect = [False] + [True] * 5
        original_target = controller.set_target

        def set_target(target):
            events.append("target")
            original_target(target)
            if target[0] == 4.:
                self.set_pose(target, received=self.fixture.wall_time)

        controller.set_target = mock.Mock(side_effect=set_target)
        controller.navigation_hold = mock.Mock(wraps=controller.navigation_hold)

        def advance(seconds):
            self.fixture.wall_time += seconds
            controller.gazebo_time = self.fixture.wall_time
            controller.gazebo_history[-1]["received"] = self.fixture.wall_time

        def wait_until(stopped, timeout, require_offboard=False):
            events.append("settle")
            self.assertEqual(timeout, 15)
            self.assertTrue(require_offboard)
            self.assertFalse(controller.navigation_moving)
            self.assertTrue(controller.navigation_pausing)
            self.assertEqual(controller.target, controller.position())
            controller.gazebo_history[-1]["linear_velocity_m_s"] = [.6, 0., 0.]
            self.assertFalse(stopped(), "setting a hover target does not remove inertia")
            advance(.2)
            self.set_pose([.2, 0., 3.5], received=self.fixture.wall_time)
            self.assertFalse(stopped(), "low speed alone cannot satisfy the hold position")
            advance(.2)
            self.set_pose([0., 0., 3.5], received=self.fixture.wall_time)
            self.assertFalse(stopped(), "the stationary hold must be sustained")
            advance(.6)
            self.assertFalse(stopped())
            advance(.5)
            self.assertTrue(stopped())
            settled[0] = True
            events.append("settled")

        controller.wait_until = mock.Mock(side_effect=wait_until)
        controller.cancel = types.SimpleNamespace(wait=lambda duration: advance(.6))
        controller.move([4., 0., 3.5])
        self.assertEqual(events, ["plan", "target", "settle", "settled", "plan", "target"])
        self.assertEqual(controller.navigation_map.plan.call_count, 2)
        controller.navigation_hold.assert_called_once()
        controller.submit.assert_not_called()
        self.assertEqual(len(controller.mission["navigation_pauses"]), 1)
        self.assertFalse(controller.mission["navigation_pauses"][0]["sensor_fault"])
        self.assertNotIn("grid", controller.mission["navigation_pauses"][0]["map"])
        local_pauses = [call.kwargs for call in controller.log.call_args_list
                        if call.args[0] == "navigation_paused"]
        self.assertEqual(len(local_pauses), 1)
        self.assertIn("grid", local_pauses[0]["map"])
        self.assertFalse(controller.navigation_moving)
        self.assertFalse(controller.navigation_pausing)

    def test_bad_scan_during_pause_stays_fatal_after_a_valid_followup(self):
        controller = self.controller
        controller.navigation_moving = False
        controller.navigation_pausing = True
        controller.navigation_hold()
        controller.on_lidar(scan_message(frame="wrong"))
        controller.on_lidar(scan_message(stamp=10.01))
        self.assertIsNone(controller.lidar_error)
        self.assertEqual(controller.navigation_gate_fault, "sensor")

        def check_stopped(stopped, timeout, require_offboard=False):
            with self.assertRaisesRegex(bridge.MissionError, "invalid while stopped"):
                stopped()
            # The real wait_until propagates an error from its predicate.
            stopped()

        controller.wait_until = mock.Mock(side_effect=check_stopped)
        with self.assertRaisesRegex(bridge.MissionError, "invalid while stopped"):
            controller.settle_navigation_hold()
        controller.navigation_map.plan.assert_not_called()

    def test_observed_clearance_requires_new_rays_and_continuous_safe_time(self):
        controller = self.controller
        mapper = bridge.NavigationMap([-2.4, -7.4, 15.4, 7.4])
        controller.navigation_map = mapper
        controller.navigation_pausing = True
        controller.mission = {"id": "test-mission"}
        controller.navigation_gate_fault = "map"
        controller.navigation_gate_error = "observed footprint blocked"
        controller.navigation_hold()
        controller.set_target = mock.Mock()
        mapper.plan = mock.Mock(wraps=mapper.plan)
        mapper.segment_safe = mock.Mock(wraps=mapper.segment_safe)
        self.navigation_scan([.8] * 360)
        self.assertGreater(mapper.snapshot(self.fixture.wall_time)["occupied_cells"], 0)

        def observe_clearance(cleared, timeout, require_offboard=False):
            self.assertEqual(timeout, 8)
            self.assertTrue(require_offboard)
            self.assertFalse(cleared())
            for index in range(4):
                self.navigation_scan()
                self.assertEqual(mapper.segment_safe([0., 0.], [0., 0.], self.fixture.wall_time),
                                 index == 3, "occupied and unknown cells need repeated free rays")
                self.assertFalse(cleared(), "first safe measurement starts the stable interval")
            self.navigation_scan(elapsed=.6)
            self.assertFalse(cleared())
            # A new observed obstruction breaks the stable interval. The
            # earlier safe interval must not be accumulated across this gap.
            self.navigation_scan([.8] * 360)
            self.assertFalse(cleared())
            for _ in range(4):
                self.navigation_scan()
                self.assertFalse(cleared())
            self.navigation_scan(elapsed=.6)
            self.assertFalse(cleared(), "clearance has only been continuous for .6 seconds")
            self.navigation_scan(elapsed=.5)
            self.assertTrue(cleared())
            controller.set_target.assert_not_called()
            self.assertEqual(controller.target, [0., 0., 3.5])

        controller.wait_until = mock.Mock(side_effect=observe_clearance)
        controller.wait_for_navigation_clearance()
        mapper.plan.assert_not_called()
        for call in mapper.segment_safe.call_args_list:
            self.assertEqual(call.args[:2], ([0., 0.], [0., 0.]))
        self.assertEqual(controller.navigation_gate_fault, "map")
        self.assertEqual(controller.navigation_gate_error, "observed footprint blocked")

    def test_persistent_unknown_or_occupied_footprint_times_out_without_moving(self):
        controller = self.controller
        real_wait_until = controller.wait_until
        for kind, ranges in (("unknown", [float("nan")] * 50 + [float("inf")] * 310),
                             ("occupied", [.8] * 360)):
            with self.subTest(kind=kind):
                mapper = bridge.NavigationMap([-2.4, -7.4, 15.4, 7.4])
                controller.navigation_map = mapper
                controller.navigation_pausing = True
                controller.mission = {"id": "test-mission"}
                controller.navigation_hold()
                controller.set_target = mock.Mock()
                controller.checkpoint = mock.Mock()
                mapper.plan = mock.Mock(wraps=mapper.plan)
                mapper.reset = mock.Mock(wraps=mapper.reset)
                self.navigation_scan(ranges)
                controller.cancel = types.SimpleNamespace(
                    wait=mock.Mock(side_effect=lambda duration: self.navigation_scan(ranges, elapsed=.25)))
                controller.wait_until = mock.Mock(wraps=real_wait_until)
                start = self.fixture.wall_time
                with self.assertRaisesRegex(bridge.MissionError, "timed out after 8 seconds"):
                    controller.wait_for_navigation_clearance()
                self.assertEqual(self.fixture.wall_time - start, 8.)
                self.assertTrue(controller.wait_until.call_args.kwargs["require_offboard"])
                status = mapper.snapshot(self.fixture.wall_time)
                self.assertTrue(status["fresh"], "fresh unsafe scans must not authorize an escape")
                self.assertGreater(status["scan_count"], 1)
                self.assertEqual(status["occupied_cells"] > 0, kind == "occupied")
                self.assertFalse(mapper.segment_safe([0., 0.], [0., 0.], self.fixture.wall_time))
                controller.set_target.assert_not_called()
                mapper.plan.assert_not_called()
                mapper.reset.assert_not_called()
                self.assertEqual(controller.target, [0., 0., 3.5])

    def test_invalid_sources_abort_clearance_before_any_safe_footprint_can_pass(self):
        controller = self.controller
        real_wait_until = controller.wait_until
        for failure in ("latest_scan_error", "stale", "bad_then_good"):
            with self.subTest(failure=failure):
                controller.navigation_map = mock.Mock()
                controller.navigation_map.snapshot.return_value = {"fresh": True, "scan_count": 5}
                controller.navigation_map.segment_safe.return_value = True
                controller.navigation_gate_error = controller.navigation_gate_fault = None
                controller.lidar_error = None
                controller.navigation_pausing = True
                controller.mission = {"id": "test-mission"}
                controller.checkpoint = mock.Mock()
                controller.cancel = types.SimpleNamespace(wait=mock.Mock())
                controller.navigation_hold()
                controller.set_target = mock.Mock()
                if failure == "latest_scan_error":
                    controller.lidar_error = "latest scan has invalid rays"
                elif failure == "stale":
                    controller.navigation_map.snapshot.return_value["fresh"] = False
                else:
                    controller.on_lidar(scan_message(frame="wrong"))
                    self.navigation_scan()
                    self.assertIsNone(controller.lidar_error)
                    self.assertEqual(controller.navigation_gate_fault, "sensor")
                controller.wait_until = mock.Mock(wraps=real_wait_until)
                start = self.fixture.wall_time
                with self.assertRaisesRegex(bridge.MissionError, "invalid while waiting for observed clearance"):
                    controller.wait_for_navigation_clearance()
                self.assertEqual(self.fixture.wall_time, start)
                controller.cancel.wait.assert_not_called()
                controller.navigation_map.segment_safe.assert_not_called()
                controller.navigation_map.plan.assert_not_called()
                controller.set_target.assert_not_called()
                self.assertEqual(controller.target, [0., 0., 3.5])
                if failure == "bad_then_good":
                    self.assertEqual(controller.navigation_gate_fault, "sensor")

    def test_blocked_pause_waits_for_observed_clearance_before_replanning(self):
        controller = self.controller
        controller.home = [0., 0., 0.]
        controller.mission = {"id": "test-mission"}
        controller.checkpoint = mock.Mock()
        mapper = bridge.NavigationMap([-2.4, -7.4, 15.4, 7.4])
        controller.navigation_map = mapper
        self.navigation_scan()
        events = []

        def plan(start, goal, now):
            events.append("plan")
            self.assertEqual(goal, [4., 0.])
            if events.count("plan") == 1:
                return {"waypoints": [[2., 0.]], "route_complete": False}
            self.assertIn("cleared", events)
            self.assertTrue(mapper.segment_safe(start, start, now))
            return {"waypoints": [[4., 0.]], "route_complete": True}

        mapper.plan = mock.Mock(side_effect=plan)
        original_target = controller.set_target

        def set_target(target):
            events.append("target")
            original_target(target)
            if target[0] == 2.:
                self.navigation_scan([.8] * 360)
            else:
                self.set_pose(target, received=self.fixture.wall_time)
                self.navigation_scan()

        controller.set_target = mock.Mock(side_effect=set_target)

        def wait_until(predicate, timeout, require_offboard=False):
            self.assertTrue(require_offboard)
            self.assertFalse(controller.navigation_moving)
            self.assertTrue(controller.navigation_pausing)
            self.assertEqual(controller.target, [0., 0., 3.5])
            self.assertEqual(mapper.plan.call_count, 1)
            controller.set_target.assert_called_once_with([2., 0., 3.5])
            self.assertFalse(predicate())
            if timeout == 15:
                events.append("settle")
                self.navigation_scan([.8] * 360, elapsed=.6)
                self.assertFalse(predicate())
                self.navigation_scan([.8] * 360, elapsed=.5)
                self.assertTrue(predicate())
                events.append("settled")
            else:
                self.assertEqual(timeout, 8)
                self.assertEqual(events[-1], "settled")
                events.append("clearance")
                for _ in range(9):
                    self.navigation_scan()
                    self.assertFalse(predicate())
                    controller.set_target.assert_called_once_with([2., 0., 3.5])
                    self.assertEqual(mapper.plan.call_count, 1)
                self.navigation_scan(elapsed=.6)
                self.assertFalse(predicate())
                self.navigation_scan(elapsed=.5)
                self.assertTrue(predicate())
                events.append("cleared")

        controller.wait_until = mock.Mock(side_effect=wait_until)
        controller.cancel = types.SimpleNamespace(wait=lambda duration: self.navigation_scan(elapsed=.6))
        controller.move([4., 0., 3.5])
        self.assertEqual(events, ["plan", "target", "settle", "settled", "clearance", "cleared",
                                  "plan", "target"])
        self.assertEqual(controller.set_target.call_args_list,
                         [mock.call([2., 0., 3.5]), mock.call([4., 0., 3.5])])
        self.assertEqual(len(controller.mission["navigation_pauses"]), 1)
        self.assertFalse(controller.mission["navigation_pauses"][0]["sensor_fault"])
        self.assertFalse(controller.navigation_moving)
        self.assertFalse(controller.navigation_pausing)

    def test_sensor_latch_after_settling_blocks_the_next_segment_despite_good_scan(self):
        controller = self.controller
        controller.home = [0., 0., 0.]
        controller.mission = {"id": "test-mission"}
        controller.checkpoint = mock.Mock()
        controller.navigation_map.plan.side_effect = [
            {"waypoints": [[2., 0.]], "route_complete": False},
            {"waypoints": [[4., 0.]], "route_complete": True}]
        controller.navigation_map.segment_safe.return_value = False
        controller.set_target = mock.Mock(wraps=controller.set_target)

        def wait_until_then_sensor_fault(stopped, timeout, require_offboard=False):
            self.assertTrue(controller.navigation_pausing)
            self.assertFalse(stopped())
            self.fixture.wall_time += 1.1
            self.set_pose([0., 0., 3.5], received=self.fixture.wall_time)
            self.assertTrue(stopped(), "the hold has completed before the fault arrives")
            # Reproduce the callback window after the final settling predicate
            # succeeds but before move() arms its replacement segment.
            controller.on_lidar(scan_message(frame="wrong"))
            controller.on_lidar(scan_message(stamp=10.01))
            self.assertIsNone(controller.lidar_error)
            self.assertEqual(controller.navigation_gate_fault, "sensor")

        controller.wait_until = mock.Mock(side_effect=wait_until_then_sensor_fault)
        with self.assertRaisesRegex(bridge.MissionError, "LiDAR"):
            controller.move([4., 0., 3.5])
        self.assertEqual(controller.navigation_map.plan.call_count, 1)
        controller.set_target.assert_called_once_with([2., 0., 3.5])
        controller.navigation_map.update_scan.assert_called_once()
        self.assertEqual(controller.navigation_gate_fault, "sensor")
        self.assertFalse(controller.navigation_moving)
        self.assertEqual(controller.target, controller.position())

    def test_navigation_hold_restores_cruise_height_with_continuous_commands(self):
        controller = self.controller
        self.set_pose([1., 2., 3.7])
        controller.navigation_pausing = True
        controller.target, controller.commanded = [5., 0., 3.5], [.5, 0., 3.5]
        controller.navigation_hold()
        self.assertEqual(controller.target, [1., 2., 3.5])
        self.assertEqual(controller.commanded, [1., 2., 3.7])
        messages = self.run_publisher_iterations()
        self.assertEqual(len(messages), 2)
        self.assertEqual([message.pose.position.x for message in messages], [1., 1.])
        self.assertEqual([message.pose.position.y for message in messages], [2., 2.])
        self.assertGreater(messages[0].pose.position.z, 3.5)
        self.assertAlmostEqual(messages[0].pose.position.z, 3.65)
        self.assertAlmostEqual(messages[1].pose.position.z, 3.60)
        self.assertEqual(controller.target[2], 3.5)

    def test_stale_scan_during_motion_is_fatal_without_settle_or_replan(self):
        controller = self.controller
        controller.home = [0., 0., 0.]
        controller.mission = {"id": "test-mission"}
        controller.checkpoint = mock.Mock()
        controller.navigation_map.plan.return_value = {
            "waypoints": [[2., 0.]], "route_complete": True}
        controller.navigation_map.segment_safe.return_value = False
        controller.navigation_map.diagnostics.return_value = {"fresh": False, "scan_count": 1}
        controller.settle_navigation_hold = mock.Mock()
        with self.assertRaisesRegex(bridge.MissionError, "sensor/freshness gate closed"):
            controller.move([2., 0., 3.5])
        controller.settle_navigation_hold.assert_not_called()
        controller.navigation_map.plan.assert_called_once()
        self.assertEqual(controller.target, controller.position())
        self.assertTrue(controller.mission["navigation_pauses"][0]["sensor_fault"])

    def test_bad_scan_latch_is_fatal_even_after_a_valid_followup_refreshes_map(self):
        controller = self.controller
        controller.home = [0., 0., 0.]
        controller.mission = {"id": "test-mission"}
        controller.checkpoint = mock.Mock()
        controller.navigation_map.plan.return_value = {
            "waypoints": [[2., 0.]], "route_complete": True}
        controller.settle_navigation_hold = mock.Mock()
        controller.cancel = types.SimpleNamespace(wait=lambda duration: None)

        def newer_measurements(start, end, now):
            self.assertTrue(controller.navigation_moving)
            controller.on_lidar(scan_message(frame="wrong"))
            controller.on_lidar(scan_message(stamp=10.01))
            self.assertIsNone(controller.lidar_error)
            self.assertEqual(controller.navigation_gate_fault, "sensor")
            return True

        controller.navigation_map.segment_safe.side_effect = newer_measurements
        with self.assertRaisesRegex(bridge.MissionError, "sensor/freshness gate closed"):
            controller.move([2., 0., 3.5])
        controller.settle_navigation_hold.assert_not_called()
        controller.navigation_map.plan.assert_called_once()
        controller.navigation_map.update_scan.assert_called_once()
        self.assertEqual(controller.target, controller.position())
        self.assertTrue(controller.mission["navigation_pauses"][0]["sensor_fault"])

    def test_latest_invalid_scan_blocks_motion_despite_a_fresh_valid_old_plan(self):
        controller = self.controller
        controller.home = [0., 0., 0.]
        controller.mission = {"id": "test-mission"}
        controller.checkpoint = mock.Mock()
        controller.navigation_map.snapshot.return_value = {"fresh": True, "scan_count": 1}
        controller.navigation_map.plan.return_value = {
            "waypoints": [[2., 0.]], "route_complete": True}
        controller.navigation_map.segment_safe.return_value = True
        controller.target = controller.commanded = controller.position()
        controller.navigation_gate_error = "previous segment froze"
        controller.lidar_error = "latest scan contains an unsafe close return"
        controller.set_target = mock.Mock()
        self.assertTrue(controller.navigation_map.snapshot(self.fixture.wall_time)["fresh"])
        with self.assertRaisesRegex(bridge.MissionError, "latest LiDAR scan is invalid"):
            controller.move([2., 0., 3.5])
        controller.navigation_map.plan.assert_called_once()
        controller.set_target.assert_not_called()
        self.assertFalse(controller.navigation_moving)
        self.assertEqual(controller.target, controller.position())
        self.assertEqual(controller.commanded, controller.position())
        self.assertEqual(controller.navigation_gate_error, "previous segment froze")

    def run_publisher_iterations(self, count=2):
        fixture = self.fixture

        class IterationEvent:
            def __init__(self):
                self.calls = 0

            def wait(self, timeout):
                self.calls += 1
                fixture.sim_time += timeout
                return self.calls > count

        self.controller.closed = IterationEvent()
        self.controller.publisher = mock.Mock()
        with mock.patch.object(bridge, "PoseStamped", side_effect=lambda: pose_message([0., 0., 0.])):
            self.controller.publish_loop()
        return [call.args[0] for call in self.controller.publisher.publish.call_args_list]

    def test_closed_motion_gate_keeps_publishing_hover_and_latches_failure(self):
        controller = self.controller
        controller.navigation_moving = True
        controller.target, controller.commanded = [5., 0., 3.5], [.5, 0., 3.5]
        controller.navigation_map.segment_safe.side_effect = [False, True]
        messages = self.run_publisher_iterations()
        self.assertEqual(len(messages), 2)
        for message in messages:
            self.assertEqual([message.pose.position.x, message.pose.position.y, message.pose.position.z],
                             controller.position())
            self.assertEqual(message.header.frame_id, "map")
        self.assertEqual(controller.target, controller.position())
        self.assertEqual(controller.commanded, controller.position())
        self.assertIn("gate closed", controller.navigation_gate_error)
        self.assertEqual(controller.navigation_gate_fault, "map")

    def test_stale_publisher_gate_latches_sensor_fault_and_keeps_hover_stream(self):
        controller = self.controller
        controller.navigation_moving = True
        controller.target, controller.commanded = [5., 0., 3.5], [.5, 0., 3.5]
        controller.navigation_map.segment_safe.side_effect = [False, True]
        controller.navigation_map.snapshot.return_value = {"fresh": False, "scan_count": 1}
        messages = self.run_publisher_iterations()
        self.assertEqual(len(messages), 2)
        self.assertEqual(controller.navigation_gate_fault, "sensor")
        for message in messages:
            self.assertEqual([message.pose.position.x, message.pose.position.y, message.pose.position.z],
                             controller.position())

    def test_autonomous_publisher_uses_quarter_metre_per_second_motion_limit(self):
        controller = self.controller
        controller.navigation_moving = True
        controller.target, controller.commanded = [2., 0., 3.5], [0., 0., 3.5]
        controller.navigation_map.segment_safe.return_value = True
        messages = self.run_publisher_iterations()
        self.assertEqual(len(messages), 2)
        self.assertAlmostEqual(messages[0].pose.position.x, .0125)
        self.assertAlmostEqual(messages[1].pose.position.x, .025)
        self.assertIsNone(controller.navigation_gate_error)

    def test_quarter_speed_diagonal_motion_uses_euclidean_cap(self):
        controller = self.controller
        controller.navigation_moving = True
        controller.target, controller.commanded = [2., 2., 3.5], [0., 0., 3.5]
        controller.navigation_map.segment_safe.return_value = True
        messages = self.run_publisher_iterations()
        previous = [0., 0., 3.5]
        for message in messages:
            position = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
            self.assertAlmostEqual(math.dist(previous, position), .0125)
            self.assertAlmostEqual(position[0], position[1])
            self.assertEqual(position[2], 3.5)
            previous = position
        self.assertEqual(controller.target, [2., 2., 3.5])
        self.assertIsNone(controller.navigation_gate_error)

    def test_valid_near_hit_still_freezes_quarter_speed_publisher(self):
        controller = self.controller
        controller.navigation_map = bridge.NavigationMap([-2.4, -7.4, 15.4, 7.4])
        values = [float("inf")] * 360
        values[307] = .31  # Valid short return above range_min, never filtered away.
        self.navigation_scan(values)
        self.navigation_scan(values)
        self.assertIsNone(controller.lidar_error)
        self.assertTrue(controller.navigation_map.snapshot(self.fixture.wall_time)["fresh"])
        self.assertFalse(controller.navigation_map.segment_safe([0., 0.], [2., 0.], self.fixture.wall_time))
        controller.navigation_moving = True
        controller.target, controller.commanded = [2., 0., 3.5], [.2, 0., 3.5]
        messages = self.run_publisher_iterations()
        self.assertEqual(len(messages), 2)
        for message in messages:
            self.assertEqual([message.pose.position.x, message.pose.position.y, message.pose.position.z],
                             controller.position())
        self.assertEqual(values[307], .31)
        self.assertEqual(controller.target, controller.position())
        self.assertEqual(controller.commanded, controller.position())
        self.assertEqual(controller.navigation_gate_fault, "map")

    def test_legacy_publisher_advances_setpoints_without_consulting_lidar(self):
        controller = self.controller
        controller.scene.pop("navigation")
        controller.navigation_moving = False
        controller.target, controller.commanded = [2., 0., 3.5], [0., 0., 3.5]
        messages = self.run_publisher_iterations()
        self.assertEqual(len(messages), 2)
        self.assertGreater(messages[-1].pose.position.x, messages[0].pose.position.x)
        self.assertGreater(messages[0].pose.position.x, 0.)
        controller.navigation_map.segment_safe.assert_not_called()
        self.assertIsNone(controller.navigation_gate_error)

    def test_auto_land_recovers_without_any_lidar_scan(self):
        controller = self.controller
        controller.state_msg.armed, controller.state_msg.mode = True, "OFFBOARD"
        controller.on_extended(types.SimpleNamespace(landed_state=2))
        controller.navigation_moving = True
        controller.navigation_gate_error = "scan unavailable"
        controller.navigation_map = None
        controller.target, controller.commanded = [1., 0., 3.5], [0., 0., 3.5]
        def accept_auto_land(mode, landing_only=False):
            self.assertEqual(mode, "AUTO.LAND")
            self.assertTrue(landing_only)
            controller.on_state(types.SimpleNamespace(connected=True, armed=True, mode="AUTO.LAND"))
        controller.switch_mode = mock.Mock(side_effect=accept_auto_land)

        def observe_landing(predicate, timeout, landing_only=False):
            self.assertFalse(predicate())
            self.assertEqual(timeout, 120)
            self.assertTrue(landing_only)
            controller.on_extended(types.SimpleNamespace(landed_state=1))
            controller.on_state(types.SimpleNamespace(connected=True, armed=False, mode="AUTO.LAND"))
            self.assertTrue(predicate())

        controller.wait_until = mock.Mock(side_effect=observe_landing)
        controller.land()
        controller.switch_mode.assert_called_once_with("AUTO.LAND", landing_only=True)
        self.assertFalse(controller.navigation_active)
        self.assertFalse(controller.navigation_moving)
        self.assertIsNone(controller.target)
        self.assertIsNone(controller.commanded)
        self.assertIsNone(controller.lidar_state)

    def test_emergency_land_submits_landing_without_requiring_a_scan(self):
        controller = self.controller
        controller.stop = mock.Mock()
        controller.state_msg.armed, controller.state_msg.mode = True, "OFFBOARD"
        controller.on_extended(types.SimpleNamespace(landed_state=2))
        controller.navigation_map = None
        with mock.patch.object(bridge.threading, "Thread") as worker:
            result = controller.emergency_land()
            worker.return_value.start.assert_called_once()
        controller.stop.assert_called_once_with()
        self.assertEqual(result["mission"]["status"], "running")
        self.assertEqual(result["mission"]["steps"], [{"action": "land"}])
        self.assertIsNone(controller.lidar_state)


if __name__ == "__main__":
    unittest.main()
