"""Actual executor goal method with ROS transport doubles; also runs on Python 3.8."""
import copy
import unittest
from types import SimpleNamespace
from unittest import mock

from mission import MissionError
from viewpoint_geometry import registered_identity
import test_task_runtime_bridge as fixtures


class GoalExecutorTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.ControllerStageTests()
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        self.c = self.fixture.c
        self.c.navigation_active = True
        self.c.navigation_moving = False
        self.c.yaw = 0.
        self.c.position = mock.Mock(return_value=[0., 0., 3.])
        self.c.ev_guard = SimpleNamespace(reason=lambda now: None,
            snapshot=lambda now: copy.deepcopy(self.fixture.state['external_vision']))
        self.c.submit = mock.Mock(return_value={'mission': {'id': 'actual-submission', 'status': 'running'}})
        self.c.transition_search_stage(self.fixture.owner, 'front-hover', 1)
        self.context = {**self.fixture.context, 'batch_id': 'c'*32}
        self.authority = self.fixture.authority
        self.authority.bind(self.context, 'pair-1', {'stamp_sim_s': 1., 'body_pose_enu': {'position_m': [0., 0., 3.]}},
                            self.fixture.identity, [0., 0.])
        self.goal = {'schema_version': 1, 'stage_id': 'front-hover', 'stage_revision': 1,
                     'plan_revision': 1, 'target_id': 'target_01', 'target_revision': 3, 'relation': 'object_front', 'terminal_action': 'relative_hover'}
        self.response = {'identity': self.fixture.identity, 'origin_xy_enu_m': [0., 0.], 'search_radius_m': 12.,
            'start_enu_m': [0., 0., 3.], 'fixed_yaw_enu_rad': 0., 'results': [{
                'id': 'goal_001', 'position_enu_m': [5., -6., 3.], 'target_yaw_enu_rad': 1.25, 'reachable': True,
                'path': {'search_origin_xy_enu_m': [0., 0.], 'search_radius_m': 12., 'waypoints': [[5., -6., 3.]]}}]}
        self.authority.issue(self.context, self.response, goal_context=self.goal)
        self.args = (self.fixture.owner, self.context['batch_id'], self.context['map_revision'], 'goal_001',
                     self.response['results'][0]['execution_ticket'])

    def test_actual_controller_submits_cached_position_and_continuous_yaw_once(self):
        result = self.c.execute_viewpoint(*self.args, goal_context=self.goal)
        self.assertEqual(result['mission']['id'], 'actual-submission')
        steps = self.c.submit.call_args[0][0]
        self.assertEqual(steps, [{'action': 'goto', 'x': 5., 'y': -6., 'altitude': 3.},
                                 {'action': 'rotate', 'yaw_rad': 1.25}])
        self.assertEqual(self.c.submit.call_args[1]['search_authorization']['goal_context'], self.goal)
        with self.assertRaises(MissionError):self.c.execute_viewpoint(*self.args, goal_context=self.goal)
        self.assertEqual(self.c.submit.call_count, 1)

    def test_old_route_wrong_target_revision_and_operator_cancel_do_not_submit(self):
        with self.assertRaises(MissionError):self.c.execute_viewpoint(*self.args)
        with self.assertRaises(MissionError):self.c.execute_viewpoint(*self.args, goal_context={**self.goal, 'target_revision': 4})
        self.authority.revoke()
        with self.assertRaises(MissionError):self.c.execute_viewpoint(*self.args, goal_context=self.goal)
        self.c.submit.assert_not_called()

    def test_epoch_reset_cannot_consume_cached_goal(self):
        self.fixture.state['external_vision']['adapter']['epoch'] += 1
        with self.assertRaises(MissionError):self.c.execute_viewpoint(*self.args, goal_context=self.goal)
        self.c.submit.assert_not_called()


if __name__ == '__main__':unittest.main()
