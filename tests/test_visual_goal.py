"""Actual parent/Agent/executor wiring with explicit camera and model doubles."""
import json
import tempfile
import unittest
from dataclasses import replace
from unittest.mock import patch

from coverage_search import project_ground_point
from task_runtime.visual_goal import image_ground_point
from test_task_goal_graph import plan, response_for
from test_task_landing_graph import FlatGoalBridge
import test_task_runtime as runtime_fixtures
from test_single_vlm import tool_call


class ImageGroundTests(unittest.TestCase):
    def test_ground_pixel_round_trip_uses_saved_calibration_and_pose(self):
        from test_semantic_map import navigator, observation
        frame, _, _ = observation(navigator())
        expected = [3., -1., 0.]
        snapshot = frame['snapshot']
        uv = project_ground_point(snapshot['camera_intrinsics'], snapshot['body_pose_enu'], expected, 20.)['pixel_uv']
        point = image_ground_point(snapshot, [uv[0]/640, uv[1]/480], 0.)
        for actual, wanted in zip(point, expected):self.assertAlmostEqual(actual, wanted)
        for bad in ([.5, 0.], [1.1, .7], [float('nan'), .7], [.5]):
            with self.subTest(pixel=bad), self.assertRaises(ValueError):image_ground_point(snapshot, bad, 0.)


class VisualGoalGraphTests(unittest.TestCase):
    def run_visual(self, *, decision='here', reviews=None, foreign=False, reset=False,
                   relation='object_front', label='白色货车', invalid_first=False, review_at_limit=False,
                   no_ground=False, reset_before_review=False, observe_first=False,
                   drop_after_ground=False, blocked_move_first=False):
        directory = tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        class Bridge(FlatGoalBridge):
            # Frontal controlled fixture; image/recognition are still doubles.
            front, rear, surface = [5., 0., 1.], [8., 0., 1.], [5., 0., 1.]
            def wait_for(self, accepted, on_progress=None):
                result = super().wait_for(accepted, on_progress)
                if no_ground and self.goal_land_executions:
                    result.update(armed=True, landed_state=2)
                if drop_after_ground and self.goal_land_executions:
                    self.telemetry['external_vision'] = {}
                return result
        bridge = Bridge(directory.name, reset)
        if blocked_move_first:
            bridge.nav.raw_boxes.append(([2.8, -1.2, 0.], [3.2, -.8, 5.]))
            bridge.nav.boxes.append(([1.8, -2.2, -1.], [4.2, .2, 6.]))
        runner = runtime_fixtures.RuntimeGraphTests();proposal = plan(landing=True)
        proposal['targets'][0]['label'] = label;proposal['stages'][1]['relation'] = relation
        ordinary = response_for(bridge);seen = [];review_count = [0];choices = [0]
        from task_runtime.orchestrator import TaskOrchestrator
        ensure = TaskOrchestrator.ensure
        original_review = TaskOrchestrator.review_task_goal
        def bounded_review(runtime, owner, *args, **kwargs):
            if foreign:args = (runtime.sessions[owner]['goal_state']['source_capture_id'],) + args[1:]
            if review_at_limit:runtime.sessions[owner]['model_calls'] = runtime.sessions[owner]['limits'].max_model_calls
            return original_review(runtime, owner, *args, **kwargs)
        def bounded(runtime, *args, **kwargs):
            state = ensure(runtime, *args, **kwargs)
            if reviews and all(x[0] != 'satisfied' or not x[1] for x in reviews):
                state['limits'] = replace(state['limits'], max_observations=3)
            return state
        def respond(messages):
            state = json.loads(next(m.content.split('父任务实际状态：', 1)[1] for m in reversed(messages)
                if isinstance(getattr(m, 'content', None), str) and m.content.startswith('父任务实际状态：')))
            name = state['next_tool'];seen.append(name)
            if name == 'choose_task_goal':
                choices[0] += 1
                if observe_first and choices[0] == 1:
                    return tool_call(name, {'decision': 'observe',
                        'description': 'controlled initial relation uncertain', 'ground_point_uv': []})
                if reviews and review_count[0] and all(x[0] != 'satisfied' or not x[1] for x in reviews):
                    return tool_call(name, {'decision': 'observe',
                        'description': 'controlled relation still uncertain', 'ground_point_uv': []})
                point = []
                selected_decision = 'here' if blocked_move_first and choices[0] > 1 else decision
                if selected_decision == 'move':
                    pair = next(reversed(bridge.pairs.values()))
                    uv = project_ground_point(pair['left']['camera_intrinsics'], pair['body_pose_enu'], [3., -1., 0.], 20.)['pixel_uv']
                    point = [uv[0]/640, uv[1]/480]
                if invalid_first and choices[0] == 1:point = [.5, 0.]
                return tool_call(name, {'decision': selected_decision,
                    'description': 'controlled model-selected visual relation', 'ground_point_uv': point})
            if name == 'review_task_goal':
                index = review_count[0];review_count[0] += 1
                status, same = reviews[min(index, len(reviews)-1)] if reviews else ('satisfied', True)
                if reset_before_review:bridge.telemetry['external_vision']['adapter']['epoch'] += 1
                return tool_call(name, {'status': status, 'same_target': same,
                    'description': 'controlled fresh visual conclusion'})
            answer = ordinary(messages)
            if name == 'report_scene':answer.tool_calls[0]['args']['targets'][0]['label'] = label
            return answer
        with patch.object(TaskOrchestrator, 'ensure', bounded), patch.object(TaskOrchestrator, 'review_task_goal', bounded_review):
            result, app, model = runner.run_graph([tool_call('define_task_plan', {'plan': proposal})]+[respond]*80,
                ('找到'+label+'后降落在它旁边' if relation == 'object_beside' else '找到'+label+'后降落在它前方'),
                bridge, timeout=30)
        return bridge, result, app, model, seen

    def test_already_satisfied_location_completes_normally_without_an_object_axis(self):
        bridge, result, _, model, seen = self.run_visual()
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertTrue(result['task_outcome']['user_goal_completed'])
        self.assertTrue(result['task_outcome']['target_found'])
        self.assertTrue(result['task_outcome']['target_confirmed'])
        self.assertEqual(result['task_outcome']['confirmation_level'], 'visual_review')
        self.assertTrue(result['search_outcome']['target_confirmed'])
        self.assertEqual(bridge.goal_land_executions, 1)
        self.assertEqual(bridge.telemetry['position_enu_m'][:2], [0., 0.])
        self.assertEqual(len(result['perceptions']), 2)
        self.assertNotIn('report_target_frame', seen)
        self.assertNotIn('review_target_candidate', seen)
        self.assertIsNone(result['task_runtime']['target_memory']['van']['orientation'])
        self.assertIsNone(result['safety_landing'])
        review = next(c for c in result['model_calls'] if c['image_purpose'] == 'goal_relation_review')
        self.assertEqual(len(set(review['capture_ids'])), 2)

    def test_model_image_point_moves_then_fresh_review_completes(self):
        bridge, result, *_ = self.run_visual(decision='move')
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertAlmostEqual(bridge.telemetry['position_enu_m'][0], 3.)
        self.assertAlmostEqual(bridge.telemetry['position_enu_m'][1], -1.)
        self.assertEqual(bridge.goal_executions, 1)
        self.assertEqual(bridge.goal_land_executions, 1)

    def test_same_flow_handles_nonvehicle_beside_relation(self):
        bridge, result, *_ = self.run_visual(label='红色集装箱', relation='object_beside')
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(bridge.goal_land_executions, 1)

    def test_uncertainty_gets_another_fresh_review_before_landing(self):
        bridge, result, *_ = self.run_visual(reviews=[('uncertain', True), ('satisfied', True)])
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(bridge.goal_executions, 2)
        self.assertEqual(len(result['perceptions']), 3)
        self.assertEqual(bridge.goal_land_executions, 1)

    def test_negative_or_different_target_review_never_submits_target_landing(self):
        for verdict in (('unsatisfied', True), ('uncertain', True), ('satisfied', False)):
            with self.subTest(verdict=verdict):
                bridge, result, *_ = self.run_visual(reviews=[verdict])
                self.assertFalse(result['task_outcome']['user_goal_completed'])
                self.assertEqual(bridge.goal_land_executions, 0)

    def test_invalid_ground_pixel_can_be_replaced_without_executing_it(self):
        bridge, result, *_ = self.run_visual(invalid_first=True)
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(bridge.goal_executions, 1)
        self.assertEqual(len(result['perceptions']), 2)

    def test_old_image_and_midair_epoch_change_cannot_authorize_landing(self):
        for values in ({'foreign': True}, {'reset_before_review': True}):
            with self.subTest(fault=values):
                bridge, result, *_ = self.run_visual(**values)
                self.assertFalse(result['task_outcome']['user_goal_completed'])
                self.assertEqual(bridge.goal_land_executions, 0)

    def test_touchdown_velocity_diagnostic_does_not_deny_completed_landing(self):
        _, result, *_ = self.run_visual(reset=True)
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        facts = result['task_runtime_result']['evidence_summary']['relative_landing']
        self.assertTrue(facts['actual_ground_disarmed'])
        self.assertTrue(facts['relation_verified'])
        self.assertFalse(facts['navigation_reference_continuous'])

    def test_model_confirmation_without_actual_ground_is_not_completion(self):
        _, result, *_ = self.run_visual(no_ground=True)
        self.assertFalse(result['task_outcome']['user_goal_completed'])

    def test_model_can_request_a_new_view_without_reproving_the_category(self):
        bridge, result, _, _, seen = self.run_visual(observe_first=True)
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(len(result['perceptions']), 3)
        self.assertNotIn('review_target_candidate', seen)
        self.assertEqual(bridge.goal_land_executions, 1)

    def test_final_visual_confirmation_at_model_limit_still_completes_the_owned_descent(self):
        bridge, result, *_ = self.run_visual(review_at_limit=True)
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(result['task_runtime']['budget']['model_calls_used'], 60)
        self.assertEqual(bridge.goal_land_executions, 1)

    def test_unreachable_model_position_is_rejected_before_flight_and_can_be_replaced(self):
        bridge, result, *_ = self.run_visual(decision='move', blocked_move_first=True)
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(bridge.goal_executions, 1)
        self.assertEqual(bridge.telemetry['position_enu_m'][:2], [0., 0.])

    def test_completed_landing_survives_loss_of_post_touchdown_navigation_diagnostics(self):
        _, result, *_ = self.run_visual(drop_after_ground=True)
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        facts = result['task_runtime_result']['evidence_summary']['relative_landing']
        self.assertTrue(facts['actual_ground_disarmed'])
        self.assertFalse(facts['post_landing_navigation_available'])


if __name__ == '__main__':unittest.main()
