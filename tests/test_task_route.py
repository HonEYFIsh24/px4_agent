"""Route skills use actual geometry/authority code with synthetic camera/flight IO."""
import copy
import json
import math
import os
import unittest
from unittest import mock

from coverage_search import project_ground_point
from task_runtime.contracts import TaskLimits
from task_runtime.planning import compile_plan
from task_runtime.route import heading_goal
from test_goal_pose import relative_fixture
from test_single_vlm import tool_call
from test_task_goal_graph import GoalRuntimeBridge, response_for
import test_task_runtime as runtime_fixtures
from test_task_runtime import review


def route_plan():
    return {'targets': [{'ref': 'van', 'label': '白色货车'}], 'stages': [
        {'kind': 'search', 'target_ref': 'van'},
        {'kind': 'pass_landmark', 'target_ref': 'van'},
        {'kind': 'relative_turn', 'target_ref': 'van', 'direction': 'left', 'angle_deg': 45.},
        {'kind': 'face_target', 'target_ref': 'van'},
        {'kind': 'report', 'target_ref': 'van'}, {'kind': 'return_home_land'}]}


class RouteGeometryTests(unittest.TestCase):
    def test_turn_is_body_relative_wraps_pi_and_never_guesses_target_distance(self):
        _, f, m, ref, *_ = relative_fixture()
        pose = copy.deepcopy(f['snapshot']['body_pose_enu'])
        pose['age_seconds'] = .1
        yaw = math.radians(170.)
        pose['orientation_xyzw'] = [0., 0., math.sin(yaw/2), math.cos(yaw/2)]
        result = heading_goal(m, ref, pose, [0., 0.], 30., 'relative_turn', 'left', 45.)
        candidate = result['candidates'][0]
        self.assertAlmostEqual(candidate['yaw_enu_rad'], math.radians(-145.))
        self.assertEqual(candidate['position_enu_m'], pose['position_m'])
        self.assertIsNone(result['target_anchor_enu_m'])
        self.assertFalse(result['target_metric_position_required'])
        self.assertIsNone(candidate['score_terms']['target_visibility'])

    def test_route_keeps_declared_target_order_and_rejects_free_model_coordinates(self):
        limits = TaskLimits(12, 900., 60, 1, 30., 20.)
        text = '找到白色货车，经过它，左转45度，重新面对货车，报告然后返航降落'
        value = route_plan()
        plan = compile_plan(value, ('thread', 'command'), text, limits)
        self.assertEqual(plan.stages[1].relation, 'route_forward')
        self.assertEqual(plan.stages[2].relation, 'body_left')
        self.assertEqual(plan.stages[3].relation, 'facing_target')
        bad = copy.deepcopy(value);bad['stages'][2]['position_enu_m'] = [10., 0., 3.]
        with self.assertRaises(ValueError):compile_plan(bad, ('t', 'c'), text, limits)
        bad = copy.deepcopy(value);bad['stages'][1]['target_ref'] = 'unknown'
        with self.assertRaises(ValueError):compile_plan(bad, ('t', 'c'), text, limits)
        bad = copy.deepcopy(value);bad['stages'] = [s for s in bad['stages'] if s['kind'] != 'pass_landmark']
        with self.assertRaises(ValueError):compile_plan(bad, ('t', 'c'), text, limits)


class RouteGraphTests(unittest.TestCase):
    def test_approach_without_photo_is_a_verified_near_goal_not_a_photo_substitution(self):
        bridge = GoalRuntimeBridge()
        runner = runtime_fixtures.RuntimeGraphTests()
        value = {'targets': [{'ref': 'van', 'label': '白色货车'}], 'stages': [
            {'kind': 'search', 'target_ref': 'van'}, {'kind': 'approach', 'target_ref': 'van'},
            {'kind': 'report', 'target_ref': 'van'}, {'kind': 'return_home_land'}]}
        with mock.patch.dict(os.environ, {'UAV_NBV_HEADING_MODE': 'discrete'}):
            result, _, _ = runner.run_graph([tool_call('define_task_plan', {'plan': value})]+[response_for(bridge)]*20,
                '找到白色货车，接近它并报告，完成后返航降落', bridge)
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(bridge.goal_executions, 1)
        self.assertEqual(result['task_runtime']['plan']['stages'][1]['kind'], 'approach')
        self.assertEqual(result['task_runtime']['plan']['stages'][1]['conditions']['fresh_target_verification'], 'true')
        self.assertNotIn('photo', [s['kind'] for s in result['task_runtime']['plan']['stages']])

    def test_pass_turn_and_face_are_serial_verified_actions_not_new_searches(self):
        bridge = GoalRuntimeBridge()
        bridge.front = list(bridge.surface)  # This fixture localizes the reported surface ROI only.
        runner = runtime_fixtures.RuntimeGraphTests()
        def response(messages):
            state = next(json.loads(m.content.split('父任务实际状态：', 1)[1]) for m in reversed(messages)
                if isinstance(getattr(m, 'content', None), str) and m.content.startswith('父任务实际状态：'))
            if state['next_tool'] == 'review_target_candidate':return review(messages)
            if state['next_tool'] == 'revise_task_plan':
                return tool_call('revise_task_plan', {'remaining_stage_ids': [s['stage_id'] for s in
                    state['plan']['stages'] if s['status'] != 'completed'], 'strategy': 'reacquire_target',
                    'reason': 'controlled retained-instance route recovery'})
            self.assertEqual(state['next_tool'], 'report_scene')
            call = next(m for m in reversed(messages) if getattr(m, 'name', None) in
                        ('observe_target_search_view', 'execute_task_goal'))
            capture = json.loads(call.content)['capture_id']
            pair = next(reversed(bridge.pairs.values()))
            projection = project_ground_point(pair['left']['camera_intrinsics'], pair['body_pose_enu'], bridge.surface, 30.)
            targets = []
            if projection['inside_fov_and_range']:
                u, v = projection['pixel_uv'];u /= 640.;v /= 480.
                targets = [{'label': '白色货车', 'bbox': [max(.01, u-.09), max(.01, v-.09),
                            min(.99, u+.09), min(.99, v+.09)], 'position_basis': 'image_only', 'surroundings': []}]
            return tool_call('report_scene', {'capture_id': capture, 'summary': 'controlled route image',
                'targets': targets, 'limitations': ['synthetic camera/flight IO, not real recognition']})
        script = [tool_call('define_task_plan', {'plan': route_plan()})]+[response]*30
        with mock.patch.dict(os.environ, {'UAV_NBV_HEADING_MODE': 'discrete'}):
            result, app, _ = runner.run_graph(script,
                '找到白色货车，经过它，左转45度，重新面对货车，报告然后返航降落', bridge)
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertTrue(result['task_outcome']['user_goal_completed'])
        self.assertEqual(bridge.goal_executions, 3)
        stages = result['task_runtime']['plan']['stages']
        self.assertEqual(stages[1]['conditions']['passed_observed_landmark_plane'], 'true')
        self.assertEqual(stages[2]['conditions']['actual_body_relative_heading'], 'true')
        self.assertEqual(stages[3]['conditions']['fresh_target_horizontal_alignment'], 'true')
        runtime = app.audited_client.task_runtime;owner = next(iter(runtime.sessions))
        child = runtime.skills.search.sessions[owner]
        self.assertEqual(child['boundary_origin'][:2], [0., 0.])
        self.assertEqual(len(runtime.skills.search.semantic_maps.maps), 1)
        self.assertEqual(sum(r['steps'][0]['action'] == 'takeoff' for r in bridge.flight_requests if r['path'] == '/mission'), 1)
        self.assertFalse(bridge.telemetry['armed'])
        self.assertEqual(bridge.telemetry['landed_state'], 1)


if __name__ == '__main__':unittest.main()
