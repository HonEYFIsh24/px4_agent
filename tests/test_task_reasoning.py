import copy
import json
import unittest

from task_runtime.contracts import TaskStage
from task_runtime.reasoning import AdaptiveReasoning
from semantic_map import SemanticMap
from test_semantic_map import OWNER, navigator, observation, target
import test_task_runtime as runtime_fixtures
from test_task_runtime import RuntimeBridge, proposal, current_report
from test_single_vlm import tool_call


class ReasoningTests(unittest.TestCase):
    def test_new_evidence_no_progress_escalates_once_then_cools_down_without_resetting_observations(self):
        reasoning = AdaptiveReasoning();state = reasoning.initial()
        stage = TaskStage('s1', 'search', 'van', None, 'visual')
        n = navigator();semantic = None
        child = {'query': '白色货车', 'search_phase': 'exploring', 'status': 'searching'}
        for i in range(3):
            frame, report, geometry = observation(n, 'view-'+str(i), [0., float(i), 3.], i+1.)
            if semantic is None:semantic = SemanticMap(OWNER, geometry)
            semantic.update_observation(frame, report, geometry)
            reasoning.update(state, stage, semantic, child)
        self.assertEqual(state['mode'], 'GLOBAL_REASONING')
        used = len(semantic.observations)
        reasoning.apply(state, 'continue_search', semantic)
        reasoning.update(state, stage, semantic, child)
        self.assertEqual(state['global_replans'], 1)
        self.assertEqual(len(semantic.observations), used)
        self.assertEqual(state['mode'], 'LOCAL_OBSERVATION')
        for _ in range(10):reasoning.update(state, stage, semantic, child)
        self.assertEqual(state['stagnant_views'], 0)  # repeated context isn't another failed view

    def test_lost_target_keeps_source_and_requests_reacquisition_not_new_xyz(self):
        reasoning = AdaptiveReasoning();state = reasoning.initial();n = navigator()
        f, r, g = observation(n);r['targets'] = [target(f, [6., 0., 1.], '白色货车')]
        m = SemanticMap(OWNER, g);m.update_observation(f, r, g)
        stage = TaskStage('s', 'relative_hover', 'van', 'object_front', 'relation')
        child = {'query': '白色货车', 'search_phase': 'confirming', 'status': 'searching',
                 'confirmation': {'target_id': 'target_01'}}
        reasoning.update(state, stage, m, child)
        self.assertEqual(state['need']['kind'], 'VISUAL_GOAL_REVIEW')
        f, r, g = observation(n, 'occlusion', [0., 1., 3.], 2.)
        m.update_observation(f, r, g);reasoning.update(state, stage, m, child)
        self.assertEqual(state['need']['kind'], 'RECOVER_OCCLUDED')
        self.assertEqual(state['need']['target_id'], 'target_01')
        self.assertEqual(state['need']['source_capture_id'], 'capture-1')
        m.invalidate('real epoch reset')
        with self.assertRaises(ValueError):reasoning.update(state, stage, m, child)


class ReasoningGraphTests(unittest.TestCase):
    def run_graph(self, script, prompt):
        fixture = runtime_fixtures.RuntimeGraphTests()
        return fixture.run_graph(script, prompt)

    def run_unavailable_replan(self, repeat=False):
        def respond(messages):
            state = next(json.loads(m.content.split('父任务实际状态：', 1)[1]) for m in reversed(messages)
                if isinstance(getattr(m, 'content', None), str) and m.content.startswith('父任务实际状态：'))
            if state['reasoning_mode'] == 'GLOBAL_REASONING':
                self.assertEqual(state['available_replan_strategies'], ['continue_search'])
                bad = repeat or state['budget']['recoveries_used'] == 0
                return tool_call('revise_task_plan', {'remaining_stage_ids': [s['stage_id'] for s in
                    state['plan']['stages'] if s['status'] != 'completed'],
                    'strategy': 'alternate_view' if bad else 'continue_search',
                    'reason': 'controlled unsubmitted strategy proposal'})
            answer = current_report(messages)
            if state['plan']['plan_revision'] == 1:answer.tool_calls[0]['args']['targets'] = []
            return answer
        return self.run_graph([tool_call('define_task_plan', {'plan': proposal()})]+[respond]*12,
                              '找到蓝色油桶并报告，然后返航降落')

    def test_unavailable_recovery_without_target_is_corrected_before_any_replan_or_flight(self):
        result, app, _ = self.run_unavailable_replan()
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        corrected = next(e for e in result['task_runtime_events'] if e['event'] == 'unexecuted_plan_correction')
        self.assertEqual(corrected['budget']['observations_used'], 3)
        self.assertEqual(corrected['budget']['global_replans_used'], 0)
        self.assertEqual(corrected['plan']['plan_revision'], 1)
        self.assertEqual(corrected['next_tool'], 'revise_task_plan')
        self.assertEqual(result['task_runtime']['budget']['recoveries_used'], 1)
        self.assertEqual(result['task_runtime']['budget']['global_replans_used'], 1)
        self.assertEqual(result['task_runtime']['budget']['observations_used'], 4)
        self.assertIsNone(result['safety_landing'])

    def test_repeated_unavailable_strategy_uses_same_single_correction_then_owned_safety_landing(self):
        result, app, _ = self.run_unavailable_replan(repeat=True)
        self.assertEqual(result['status'], 'failed')
        self.assertIn('correction budget exhausted', result['error'])
        self.assertEqual(result['task_runtime']['budget']['recoveries_used'], 1)
        self.assertEqual(result['task_runtime']['budget']['global_replans_used'], 0)
        self.assertEqual(result['task_runtime_result']['termination_source'], 'protocol_error')
        self.assertFalse(app.client.telemetry['armed'])
        self.assertEqual(app.client.telemetry['landed_state'], 1)

    def test_no_progress_global_replan_then_fresh_observation_resumes_original_task(self):
        def respond(messages):
            snapshot = next(json.loads(m.content.split('父任务实际状态：', 1)[1])
                            for m in reversed(messages) if isinstance(getattr(m, 'content', None), str)
                            and m.content.startswith('父任务实际状态：'))
            if snapshot['reasoning_mode'] == 'GLOBAL_REASONING':
                ids = [s['stage_id'] for s in snapshot['plan']['stages'] if s['status'] != 'completed']
                return tool_call('revise_task_plan', {'remaining_stage_ids': ids, 'strategy': 'continue_search',
                                                      'reason': '实际不同视点均未见目标，继续探索未观察区域'})
            if snapshot['plan']['plan_revision'] > 1:
                return current_report(messages)
            message = current_report(messages);message.tool_calls[0]['args']['targets'] = []
            return message
        script = [tool_call('define_task_plan', {'plan': proposal()})]+[respond]*10
        result, app, model = self.run_graph(script, '找到蓝色油桶并报告，然后返航降落')
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        state = result['task_runtime']
        self.assertEqual(state['plan']['plan_revision'], 2)
        self.assertEqual(state['budget']['observations_used'], 4)
        self.assertEqual(state['budget']['model_calls_used'], 6)  # parse, 3 reports, replan, report
        runtime = app.audited_client.task_runtime;owner = next(iter(runtime.sessions))
        child = runtime.skills.search.sessions[owner]
        self.assertEqual(len(runtime.skills.search.semantic_maps.maps), 1)
        self.assertEqual(child['boundary_origin'][:2], [0., 0.])
        self.assertEqual(runtime.sessions[owner]['reasoning']['global_replans'], 1)
        self.assertFalse(result['mission']['armed'] if 'armed' in result.get('mission', {}) else app.client.telemetry['armed'])

    def test_multiple_targets_share_owner_map_binding_observations_and_permission(self):
        plan = proposal()
        plan['targets'].append({'ref': 'tent', 'label': '橙色帐篷'})
        plan['stages'][1:1] = [{'kind': 'search', 'target_ref': 'tent'}]
        def tent(messages):
            answer = current_report(messages)
            answer.tool_calls[0]['args']['targets'][0]['label'] = '橙色帐篷'
            return answer
        result, app, _ = self.run_graph([tool_call('define_task_plan', {'plan': plan}), current_report, tent],
                                       '先找到蓝色油桶，再找到橙色帐篷，报告然后返航降落')
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        runtime = app.audited_client.task_runtime;owner = next(iter(runtime.sessions))
        child = runtime.skills.search.sessions[owner];m = runtime.skills.search.semantic_maps.maps[owner]
        self.assertEqual(len(child['observations']), 2)
        self.assertEqual(len(m.observations), 2)
        self.assertEqual(m.query, '橙色帐篷')
        self.assertEqual(set(runtime.sessions[owner]['targets']), {'target', 'tent'})
        self.assertEqual(len(runtime.skills.search.sessions), 1)
        self.assertTrue(result['task_outcome']['user_goal_completed'])


if __name__ == '__main__':unittest.main()
