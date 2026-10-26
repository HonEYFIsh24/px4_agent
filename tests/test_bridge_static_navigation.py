import unittest
from unittest import mock
from scene_config import load_scene, OBSTACLE_WORLD
from mission import MissionError
import test_bridge_navigation as nav_fixture


class StaticBridgeTests(unittest.TestCase):
    def setUp(self):
        self.fixture=nav_fixture.BridgeNavigationTests('test_auto_land_recovers_without_any_lidar_scan')
        # Reuse setup only, without running an inherited LiDAR test suite.
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.c=self.fixture.controller
        self.c.scene=load_scene(OBSTACLE_WORLD,navigation_mode='static_3d')
        self.c.world_path=OBSTACLE_WORLD

    def test_hold_preserves_current_height_in_variable_height_mode(self):
        self.fixture.set_pose([0.,0.,8.])
        self.c.navigation_hold()
        self.assertEqual(self.c.target,[0.,0.,8.])

    def test_horizontal_lidar_does_not_reimpose_old_height_gate(self):
        self.fixture.set_pose([0.,0.,8.])
        self.fixture.navigation_scan()
        self.c.navigation_map.update_scan.assert_not_called()
        self.assertIsNone(self.c.lidar_error)

    def test_occupied_goal_is_rejected_before_a_worker_is_created(self):
        self.c.state_msg.armed=True
        self.c.landed=2
        with self.assertRaises(MissionError):
            self.c.submit([{'action':'goto','x':4.5,'y':0.,'altitude':3.5}],'occupied goal')
        self.assertIsNone(self.c.worker)

    def test_static_telemetry_does_not_claim_lidar_mapping(self):
        self.c.navigation_map=None
        s=self.c.navigation_snapshot()
        self.assertEqual(s['method'],'static_3d')
        self.assertEqual(s['altitude_mode'],'variable')
        self.assertEqual(s['source'],'verified_static_collision_map_3d')
        self.assertIn('telemetry_only',s['lidar_role'])

    def test_preview_rejection_never_creates_flight_or_closes_navigation_gate(self):
        self.c.navigation_active = True
        self.c.navigation_map = mock.Mock()
        self.c.navigation_map.plan.side_effect = MissionError('occupied goal')
        result = self.c.preview_navigation([2., 0., 5.])
        self.assertFalse(result['reachable'])
        self.assertIsNone(self.c.worker)
        self.assertIsNone(self.c.navigation_gate_error)

    def test_preview_success_has_no_flight_side_effect_and_validates_coordinates(self):
        self.c.navigation_active = True
        self.c.navigation_map = mock.Mock()
        self.c.navigation_map.plan.return_value = {'path_length_m': 3.}
        self.assertTrue(self.c.preview_navigation([2., 0., 5.])['reachable'])
        self.assertIsNone(self.c.worker)
        for point in ([True, 0., 5.], [2., 0., float('nan')], [2., 0., 11.]):
            with self.assertRaises((MissionError, ValueError)):
                self.c.preview_navigation(point)

    def test_active_sortie_registration_is_reused_for_preview_and_submission(self):
        from static_navigation import StaticNavigator
        navigator = StaticNavigator.from_scene(self.c.world_path, self.c.scene)
        pose = self.c.pose.pose
        q = pose.orientation
        body = self.c.gazebo_history[-1]
        navigator.bind(self.c.position(), [q.x, q.y, q.z, q.w], body)
        self.c.navigation_map = navigator
        self.c.navigation_active = True
        with mock.patch.object(StaticNavigator, 'from_scene', side_effect=AssertionError('must not re-register mid-sortie')):
            self.c.validate_scene_steps([{'action': 'goto', 'x': 0., 'y': 0., 'altitude': 5.}])
            self.assertTrue(self.c.preview_navigation([0., 0., 5.])['reachable'])

    def test_static_cruise_speeds_up_but_approach_and_tracking_lag_slow_down(self):
        self.fixture.set_pose([0., 0., 3.])
        self.c.target, self.c.commanded = [3., 0., 3.], [0., 0., 3.]
        self.assertAlmostEqual(self.c.navigation_speed_limit(), .65)
        self.c.commanded = [.6, 0., 3.]
        self.assertAlmostEqual(self.c.navigation_speed_limit(), .25)
        self.c.target, self.c.commanded = [.5, 0., 3.], [0., 0., 3.]
        self.assertAlmostEqual(self.c.navigation_speed_limit(), .25)

    def test_faster_publisher_keeps_euclidean_cap_and_freezes_on_closed_gate(self):
        import math
        self.fixture.set_pose([0., 0., 3.])
        self.c.navigation_moving = True
        self.c.target, self.c.commanded = [2., 2., 3.], [0., 0., 3.]
        self.c.navigation_map.segment_safe.return_value = True
        messages = self.fixture.run_publisher_iterations()
        previous = [0., 0., 3.]
        for message in messages:
            position = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
            self.assertAlmostEqual(math.dist(previous, position), .65*.05)
            previous = position
        self.c.navigation_map.segment_safe.return_value = False
        self.c.target, self.c.commanded = [2., 2., 3.], [0., 0., 3.]
        messages = self.fixture.run_publisher_iterations()
        self.assertEqual(self.c.target, [0., 0., 3.])
