"""Production Bridge heading alignment contracts with isolated ROS stubs.

No Controller constructor, ROS service, HTTP, provider, or flight is started.
Import fixture modules, not their TestCase classes, to avoid duplicate discovery.
"""
import copy
import json
import math
import types
import unittest
from unittest import mock

import test_bridge_navigation as navigation_fixture
from ev_health import ExternalVisionFlightGuard
from search_tools import coordinate_epoch


bridge = navigation_fixture.bridge


def ekf_sample(source=10., heading=0, aligned=False, in_air=False,
               heading_good=True, delta=0.):
    return {"source": "actual_owned_px4_uorb", "ready": True, "fusion_active": True,
            "position_fusion": True, "velocity_fusion": True,
            "local_position_valid": True, "local_velocity_valid": True,
            "primary_instance": 0, "local_position_source_stamp_sim_s": source,
            "local_position_publish_stamp_sim_s": source+.01, "observed_sim_s": source+.03,
            "local_position_reset_counters": {"xy": 0, "z": 0, "heading": heading},
            "local_velocity_reset_counters": {"vxy": 0, "vz": 0},
            "heading_good_for_control": heading_good, "heading_delta_ned_rad": delta,
            "mag_alignment": {"source_stamp_sim_s": source, "publish_stamp_sim_s": source+.01,
                              "aligned_in_flight": aligned, "yaw_aligned": True, "in_air": in_air}}


class TakeoffHeadingAlignmentTests(unittest.TestCase):
    def setUp(self):
        fixture = navigation_fixture.BridgeNavigationTests(
            "test_pose_callback_preserves_full_orientation_for_scan_alignment")
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.fixture, self.controller = fixture, fixture.controller
        self.clock = fixture.fixture
        self.controller.deadline = self.clock.wall_time+300.
        self.controller.ev_guard = ExternalVisionFlightGuard()
        self.controller.ev_guard.observe_adapter(
            {"mode": "fuse", "epoch": 3, "ready": True, "forwarding": True}, self.clock.wall_time)
        self.controller.mission = {"id": "takeoff-owned", "status": "running", "action": "takeoff"}
        self.controller.navigation_map = None
        self.controller.navigation_active = self.controller.navigation_moving = False
        self.emit(ekf_sample())

    def emit(self, value):
        self.controller.on_ekf_status(types.SimpleNamespace(data=json.dumps(value)))

    def reset_then_align(self, delta=.02):
        self.controller.state_msg.armed = True
        self.emit(ekf_sample(10.2, 1, in_air=True, heading_good=False, delta=delta))
        self.assertTrue(self.controller.ev_guard.alignment_pending)
        self.assertIsNone(self.controller.ev_guard.heading_alignment_event)
        self.emit(ekf_sample(10.3, 1, aligned=True, in_air=True, heading_good=False, delta=delta))
        self.assertIsNone(self.controller.ev_guard.latched)
        self.assertIsNotNone(self.controller.ev_guard.heading_alignment_event)

    def test_only_running_vertical_takeoff_context_passes_the_alignment_permission(self):
        baseline = {"mission": copy.deepcopy(self.controller.mission), "navigation_map": None,
                    "navigation_active": False, "navigation_moving": False}
        cases = [(True, {}), (False, {"mission": {"status": "running", "action": "goto"}}),
                 (False, {"mission": {"status": "succeeded", "action": "takeoff"}}),
                 (False, {"mission": None}), (False, {"navigation_active": True}),
                 (False, {"navigation_moving": True}), (False, {"navigation_map": object()})]
        for allowed, changed in cases:
            with self.subTest(changed=changed):
                for key, value in {**baseline, **changed}.items():
                    setattr(self.controller, key, value)
                guard = mock.Mock(); guard.observe_ekf.return_value = None
                self.controller.ev_guard = guard
                self.controller.state_msg.armed = True
                sample = ekf_sample()
                self.emit(sample)
                guard.observe_ekf.assert_called_once_with(sample, self.clock.wall_time, True,
                    allow_takeoff_mag_alignment=allowed)

    def test_horizontal_stage_heading_reset_remains_fatal_and_changes_no_commanded_yaw(self):
        self.controller.state_msg.armed = True
        self.controller.mission["action"] = "goto"
        self.controller.yaw = .3
        old_history = list(self.controller.local_pose_history)
        self.emit(ekf_sample(10.2, 1, aligned=True, in_air=True, delta=.02))
        self.assertIn("reset", self.controller.ev_guard.latched)
        self.assertEqual(self.controller.yaw, .3)
        self.assertEqual(list(self.controller.local_pose_history), old_history)
        self.controller.log.assert_not_called()

    def test_accepted_ned_heading_delta_subtracts_from_enu_yaw_with_wrap_once(self):
        self.controller.yaw = -math.pi+.005
        self.reset_then_align(.02)
        expected = math.pi-.015
        self.assertAlmostEqual(self.controller.yaw, expected)
        self.assertEqual(len(self.controller.ev_guard.heading_alignment_history), 1)
        self.controller.log.assert_called_once()
        event, = self.controller.log.call_args.args
        self.assertEqual(event, "takeoff_mag_heading_alignment")
        self.assertEqual(self.controller.log.call_args.kwargs["mission_id"], "takeoff-owned")
        self.assertAlmostEqual(self.controller.log.call_args.kwargs["commanded_yaw_enu_rad"], expected)
        # PX4 delta_heading is sticky until another heading reset. Clearing it
        # while this accepted reset is pending is malformed source evidence.
        self.emit(ekf_sample(10.4, 1, aligned=True, in_air=True, delta=.02))
        self.assertIsNone(self.controller.ev_guard.latched)
        self.assertFalse(self.controller.ev_guard.alignment_pending)
        self.assertAlmostEqual(self.controller.yaw, expected)
        self.controller.log.assert_called_once()

    def test_alignment_clears_history_and_old_pose_cannot_enter_new_camera_epoch(self):
        old_epoch = coordinate_epoch(self.controller.ev_guard.snapshot(self.clock.wall_time))
        self.reset_then_align()
        barrier = self.controller.heading_alignment_pose_barrier_sim_s
        self.assertEqual(barrier, 10.2)
        self.assertEqual(list(self.controller.local_pose_history), [])
        self.controller.on_pose(navigation_fixture.pose_message([0., 0., 3.5], stamp=10.19))
        self.assertEqual(list(self.controller.local_pose_history), [])
        self.controller.on_pose(navigation_fixture.pose_message([0., 0., 3.5], stamp=barrier))
        self.assertEqual(len(self.controller.local_pose_history), 1)
        recorded = self.controller.local_pose_history[-1]
        new_epoch = coordinate_epoch(self.controller.ev_guard.snapshot(self.clock.wall_time))
        self.assertEqual(recorded["coordinate_epoch"], new_epoch)
        self.assertNotEqual(old_epoch, new_epoch)
        self.assertEqual(new_epoch["ekf_counters"], {"xy": 0, "z": 0, "heading": 1})

    def test_heading_ready_requires_pending_clear_good_control_and_post_reset_pose(self):
        self.reset_then_align()
        controller = self.controller
        controller.on_pose(navigation_fixture.pose_message([0., 0., 3.5], stamp=10.4))
        self.assertFalse(controller.takeoff_heading_ready())
        self.emit(ekf_sample(10.4, 1, aligned=True, in_air=True, delta=.02))
        self.assertIsNone(controller.ev_guard.latched)
        self.assertFalse(controller.ev_guard.alignment_pending)
        self.assertTrue(controller.takeoff_heading_ready())
        for invalid_heading in (False, None, 1):
            controller.ev_guard.ekf["heading_good_for_control"] = invalid_heading
            self.assertFalse(controller.takeoff_heading_ready())
        controller.ev_guard.ekf["heading_good_for_control"] = True
        controller.pose = None
        self.assertFalse(controller.takeoff_heading_ready())
        controller.on_pose(navigation_fixture.pose_message([0., 0., 3.5], stamp=10.19))
        self.assertFalse(controller.takeoff_heading_ready())
        controller.on_pose(navigation_fixture.pose_message([0., 0., 3.5], stamp=10.2))
        self.assertTrue(controller.takeoff_heading_ready())

    def test_takeoff_rebuilds_prior_map_only_after_alignment_pose_and_height_gates(self):
        controller = self.controller
        self.fixture.set_pose([0., 0., .05])
        previous_map = mock.Mock()
        controller.navigation_map, controller.navigation_active = previous_map, True
        controller.heading_alignment_pose_barrier_sim_s = 9.
        order = []
        def pre_stream():
            self.assertIsNone(controller.navigation_map)
            self.assertFalse(controller.navigation_active)
            self.assertFalse(controller.navigation_moving)
            self.assertIsNone(controller.heading_alignment_pose_barrier_sim_s)
            order.append("stream")
        controller.pre_stream = mock.Mock(side_effect=pre_stream)
        controller.set_target = mock.Mock()
        controller.switch_mode = mock.Mock()
        controller.arm_client = mock.Mock(return_value=types.SimpleNamespace(success=True))
        def move(point):
            self.assertEqual(point, [0., 0., 3.5])
            self.assertIsNone(controller.navigation_map)
            order.append("vertical")
            controller.on_pose(navigation_fixture.pose_message(point, stamp=10.1))
            self.reset_then_align()
        controller.move_direct = mock.Mock(side_effect=move)
        def wait(predicate, timeout, require_offboard=False):
            if timeout == 5:
                controller.state_msg.armed = True
                self.assertTrue(predicate())
            elif predicate.__name__ == "takeoff_heading_ready":
                self.assertEqual(timeout, 15)
                self.assertTrue(require_offboard)
                self.assertFalse(predicate())
                self.emit(ekf_sample(10.4, 1, aligned=True, in_air=True, delta=.02))
                self.assertIsNone(controller.ev_guard.latched)
                self.assertFalse(predicate())  # existing pose predates reset
                controller.on_pose(navigation_fixture.pose_message([0., 0., 3.5], stamp=10.4))
                self.assertTrue(predicate())
                order.append("heading")
            else:
                self.assertEqual(timeout, 15)
                self.assertTrue(require_offboard)
                self.assertEqual(order[-1], "heading")
                self.assertFalse(predicate())
                self.clock.wall_time += 1.01
                self.assertTrue(predicate())
                order.append("level")
        controller.wait_until = mock.Mock(side_effect=wait)
        def start_map():
            self.assertEqual(order, ["stream", "vertical", "heading", "level"])
            self.assertIsNone(controller.navigation_map)
            self.assertTrue(controller.takeoff_heading_ready())
            order.append("map")
        controller.start_navigation = mock.Mock(side_effect=start_map)
        controller.takeoff(3.5)
        self.assertEqual(order[-1], "map")
        previous_map.snapshot.assert_not_called()
        controller.start_navigation.assert_called_once_with()

    def test_new_map_still_requires_five_fresh_lidar_scans(self):
        controller = self.controller
        mapper = mock.Mock()
        current = {"fresh": True, "scan_count": 0}
        mapper.snapshot.side_effect = lambda _now: dict(current)
        def wait(predicate, timeout, require_offboard=False):
            self.assertEqual(timeout, 5)
            self.assertTrue(require_offboard)
            for count in range(5):
                current.update(fresh=True, scan_count=count)
                self.assertFalse(predicate())
            current.update(fresh=False, scan_count=5)
            self.assertFalse(predicate())
            current["fresh"] = True
            self.assertTrue(predicate())
        controller.wait_until = mock.Mock(side_effect=wait)
        with mock.patch.object(bridge, "NavigationMap", return_value=mapper) as constructor:
            controller.start_navigation()
            constructor.assert_called_once()
        self.assertIs(controller.navigation_map, mapper)
        self.assertTrue(controller.navigation_active)


if __name__ == "__main__":
    unittest.main()
