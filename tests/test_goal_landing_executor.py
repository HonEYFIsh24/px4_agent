"""Actual Controller landing-authority methods; explicit local ROS/state doubles."""
import copy
import math
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from mission import MissionError
from scene_config import load_scene
from static_navigation import StaticNavigator
from viewpoint_authority import ViewpointAuthority
from viewpoint_geometry import registered_identity
from test_semantic_map import EPOCH
import test_bridge_navigation as fixtures


class GoalLandingExecutorTests(unittest.TestCase):
    def setUp(self):
        f = fixtures.BridgeNavigationTests('test_auto_land_recovers_without_any_lidar_scan')
        f.setUp();self.addCleanup(f.doCleanups);self.c = f.controller
        c = self.c
        c.world_path = Path(__file__).resolve().parents[1]/'scene/test_environments/courtyard_spacious.world'
        c.scene = load_scene(c.world_path, navigation_mode='static_3d')
        c.navigation_map = StaticNavigator.from_scene(c.world_path, c.scene)
        c.navigation_map.bind([0., 0., 0.], [0., 0., 0., 1.],
                             {'position_m': [0., 0., 0.], 'orientation_xyzw': [0., 0., 0., 1.]})
        c.ready = mock.Mock();c.validate_scene_steps = mock.Mock()
        from goal_landing_authority import GoalLandingAuthority
        # Camera/ROS fixtures use a controlled monotonic clock. Inject the same
        # clock explicitly rather than depending on module import order.
        c._goal_landing_authority = GoalLandingAuthority(clock=time.monotonic)
        c.navigation_active, c.navigation_moving, c.navigation_gate_error = True, False, None
        c.position = mock.Mock(return_value=[-5., 0., 3.])
        c.state_msg.armed, c.landed = True, 2
        c.pose_time = time.monotonic()
        q = c.pose.pose.orientation;q.x = q.y = 0.;q.z = math.sin(.5);q.w = math.cos(.5)
        self.epoch = copy.deepcopy(EPOCH)
        def external():return {'adapter': {'epoch': self.epoch['vio_epoch']},
                               'ekf': {'local_position_reset_counters': copy.deepcopy(self.epoch['ekf_counters'])}}
        c.ev_guard = SimpleNamespace(reason=lambda now: None, snapshot=lambda now: external())
        self.owner = ['actual-parent', 'actual-command']
        self.identity = registered_identity(c.navigation_map, self.epoch)
        self.context = {'owner': self.owner, 'batch_id': 'b'*32, 'map_revision': 'a'*64}
        a = c.search_authority()
        a.bind(self.context, 'initial-pair', {'stamp_sim_s': 1., 'body_pose_enu': {'position_m': [0., 0., 3.]}},
               self.identity, [0., 0.])
        a.advance_stage(self.owner, 'front-land', 1, self.identity)
        self.context = {**self.context, 'batch_id': 'd'*32, 'map_revision': 'c'*64}
        a.bind(self.context, 'verified-pair', {'stamp_sim_s': 3., 'body_pose_enu': {'position_m': [-5., 0., 3.]}},
               self.identity, [0., 0.])
        self.goal = {'schema_version': 1, 'stage_id': 'front-land', 'stage_revision': 1, 'plan_revision': 1,
                     'target_id': 'target_01', 'target_revision': 3, 'relation': 'object_front', 'terminal_action': 'relative_land'}
        c.mission = {'id': 'actual-hover', 'status': 'succeeded', 'goal_context': {**self.goal, 'target_revision': 2},
            'goal_pose_enu_m': [-5., 0., 3.], 'goal_yaw_enu_rad': 1.,
            'events': [{'action': 'rotate', 'rotation': {'stamp_s': 2.5}}]}
        self.metadata = {'stamp_sim_s': 3., 'coordinate_epoch': copy.deepcopy(self.epoch),
            'navigation_registration_id': c.navigation_map.registration_id, 'image_sha256': {'left': 'e'*64, 'right': 'f'*64},
            'body_pose_enu': {'position_m': [-5., 0., 3.]}}
        c.stereo_reports = SimpleNamespace(observation_metadata=mock.Mock(side_effect=lambda *a, **k: copy.deepcopy(self.metadata)))
        c.submit = mock.Mock(return_value={'mission': {'id': 'actual-land-submission', 'status': 'running'}})

    def authorize(self):
        return self.c.authorize_goal_land(owner=self.owner, expected_mission_id='actual-hover',
            search_context=self.context, goal_context=self.goal, verification_pair_id='verified-pair')

    def execute(self, ticket):
        return self.c.execute_goal_land(owner=self.owner, expected_mission_id='actual-hover',
            search_context=self.context, goal_context=self.goal, ticket=ticket)

    def test_actual_static_support_authorization_and_one_use_land_submission(self):
        value = self.authorize()
        self.assertEqual(value['status'], 'authorized')
        self.assertFalse(value['mission_submitted'])
        self.c.submit.assert_not_called()
        self.c.stereo_reports.observation_metadata.assert_called_with('verified-pair', max_age_wall_s=30.)
        result = self.execute(value['ticket'])
        self.assertEqual(result['mission']['id'], 'actual-land-submission')
        self.assertEqual(self.c.submit.call_args[0][0], [{'action': 'land'}])
        proof = self.c.submit.call_args[1]['landing_authorization']['proof']
        self.assertTrue(proof['continuous_descent_verified'])
        self.assertTrue(proof['horizontal_support_verified'])
        with self.assertRaises(MissionError):self.execute(value['ticket'])
        self.assertEqual(self.c.submit.call_count, 1)

    def test_old_pre_rotation_pair_and_coordinate_reset_cannot_authorize_or_land(self):
        self.metadata['stamp_sim_s'] = 2.4
        with self.assertRaises(MissionError):self.authorize()
        self.metadata['stamp_sim_s'] = 3.
        permit = self.authorize()
        self.epoch['vio_epoch'] += 1
        with self.assertRaises(MissionError):self.execute(permit['ticket'])
        self.c.submit.assert_not_called()

    def test_operator_stop_new_mission_or_target_revision_invalidates_permit(self):
        permit = self.authorize()
        self.c.search_authority().revoke()
        with self.assertRaises(MissionError):self.execute(permit['ticket'])
        self.c.submit.assert_not_called()

    def test_obstructed_descent_is_a_structured_unexecuted_candidate_rejection(self):
        self.c.navigation_map.boxes.append(([-6., -1., 0.], [-4., 1., 1.]))
        value = self.authorize()
        self.assertEqual(value['status'], 'not_executed')
        self.assertEqual(value['reason_code'], 'FLAT_SUPPORT_UNAVAILABLE')
        self.assertFalse(value['mission_submitted'])
        self.c.submit.assert_not_called()

    def test_stale_hover_pose_and_foreign_owner_are_not_candidate_rejections(self):
        self.c.pose_time -= 2.
        with self.assertRaises(MissionError):self.authorize()
        self.c.pose_time = time.monotonic()
        with self.assertRaises(MissionError):self.c.authorize_goal_land(owner=['other', 'task'],
            expected_mission_id='actual-hover', search_context=self.context, goal_context=self.goal,
            verification_pair_id='verified-pair')
        self.c.submit.assert_not_called()


if __name__ == '__main__':unittest.main()
