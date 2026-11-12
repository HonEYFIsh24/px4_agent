"""Current-task model projections preserve real graph state and evidence owners."""
import copy
import json
import unittest

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage

from task_memory import TaskContextMemory


def tool_result(name, result, identifier='tool-result', status='success'):
    return ToolMessage(json.dumps(result, ensure_ascii=False), name=name, id=identifier,
                       tool_call_id=identifier+'-call', status=status)


class CurrentTaskProjectionTests(unittest.TestCase):
    def project(self, result, name='report_scene'):
        message = tool_result(name, result)
        raw = [HumanMessage('找到白色货车后降落在货车前方。', id='active'),
               AIMessage('', id='assistant', tool_calls=[dict(name=name, args={},
                    id=message.tool_call_id, type='tool_call')]), message]
        original = copy.deepcopy(raw)
        projected, metrics = TaskContextMemory().project(raw, ('thread', 'active'))
        self.assertEqual(raw, original)
        self.assertIs(projected[0], raw[0])
        self.assertIs(projected[1], raw[1])
        for field in ('id', 'name', 'tool_call_id', 'status'):
            self.assertEqual(getattr(projected[-1], field), getattr(message, field))
        self.assertEqual(len(projected), len(raw))
        return json.loads(projected[-1].content), metrics, raw, projected

    def test_current_report_keeps_visual_metric_result_and_source_ids_without_cumulative_calls(self):
        current = {'status': 'ok', 'capture_id': 'current', 'summary': '实际可见白色货车前脸',
                   'camera': {'image_sha256': 'a'*64},
                   'targets': [{'label': '白色货车', 'bbox': [.1, .2, .8, .9],
                       'stereo_location': {'status': 'ok', 'coordinate_epoch': {'vio_epoch': 2},
                                           'covariance_enu_m2': [[1., 0., 0.]]*3}}],
                   'vlm_calls': [{'capture_ids': ['source', 'current'], 'audit': 'x'*10000}]*20,
                   'search_progress': {'search_status': 'searching', 'search_phase': 'awaiting_review',
                       'confirmation': {'target_id': 'van-1', 'source_capture_id': 'source',
                                        'current_capture_id': 'current'},
                       'coordinate_epoch': {'vio_epoch': 2}, 'next_tool': 'review_target_candidate',
                       'observations': [{'capture_id': str(i), 'summary': 'saved report '+str(i)} for i in range(11)]}}
        result, metrics, raw, projected = self.project(current)
        for key in ('status', 'capture_id', 'summary', 'camera', 'targets'):
            self.assertEqual(result[key], current[key])
        progress = result['search_progress']
        self.assertEqual(progress['confirmation'], current['search_progress']['confirmation'])
        self.assertEqual(progress['coordinate_epoch'], current['search_progress']['coordinate_epoch'])
        self.assertEqual(progress['next_tool'], 'review_target_candidate')
        self.assertEqual(progress['observation_count_total'], 11)
        self.assertEqual([o['capture_id'] for o in progress['observations']], ['9', '10'])
        self.assertNotIn('vlm_calls', result)
        self.assertEqual(len(json.loads(raw[-1].content)['vlm_calls']), 20)
        self.assertEqual(metrics['projected_current_tool_results'], 1)
        self.assertLess(metrics['output_characters'], metrics['input_characters']/10)
        again, _ = TaskContextMemory().project(projected, ('thread', 'active'))
        self.assertEqual(again, projected)

    def test_parent_projection_keeps_required_stages_conditions_budget_and_measured_relation_quality(self):
        stages = [{'stage_id': 'search', 'kind': 'search', 'status': 'completed',
                   'evidence_refs': [{'capture_id': 'source', 'audit': 'e'*20000}]},
                  {'stage_id': 'land', 'kind': 'relative_land', 'relation': 'object_front',
                   'status': 'running', 'conditions': {'object_orientation': 'unknown'}, 'evidence_refs': []}]
        orientation = {'source_capture_id': 'source', 'image_sha256': {'left': 'a'*64, 'right': 'b'*64},
                       'yaw_variance_rad2': 1.9, 'current_front_relation': {'minimum_front_distance_m': -1.},
                       'part_measurements': [{'raw_samples': 'm'*10000}]*3}
        quality = {'measurement_acquired': True, 'relation_usable': False,
                   'normal_uncertainty_margin_rad': 1.9, 'depth_missing': False, 'front_face_clipped': True,
                   'information_gap': ['ANGULAR_UNCERTAINTY', 'FRONT_FACE_CLIPPED']}
        original = {'schema_version': 1, 'owner': ['thread', 'active'], 'stage_revision': 5,
                    'plan': {'original_instruction': '原句', 'stages': stages},
                    'current_stage': stages[-1], 'next_tool': 'report_target_frame',
                    'budget': {'max_observations': 12, 'max_duration_seconds': 900., 'model_calls_used': 28},
                    'target_memory': {'van': {'target_id': 'van-1', 'observation_refs': ['source', 'current'],
                        'target_revision': 5, 'orientation': orientation, 'front_relation_quality': quality,
                        'association_hypotheses': {'audit': 'h'*15000}}},
                    'reasoning_events': [{'reason': str(i)} for i in range(8)],
                    'goal_state': {'phase': 'ready', 'need_kind': 'IMPROVE_FRONT_RELATION',
                        'object_frame_attempts': [{'capture_id': str(i), 'target_revision': i,
                            'front_relation_quality': quality} for i in range(6)],
                        'prepared': {'status': 'ready', 'selected_id': 'goal_1',
                        'target_revision': 3, 'candidates': [{'id': 'goal_1', 'position_enu_m': [1., 2., 3.],
                            'yaw_enu_rad': 1., 'score': 1., 'hard_constraints': {'path': True},
                            'execution_ticket': 'private-ticket', 'path': {'audit': 'p'*10000}}]}},
                    'result': None}
        result, _, _, projected = self.project(original, 'report_target_frame')
        for key in ('owner', 'stage_revision', 'next_tool', 'budget', 'result'):
            self.assertEqual(result[key], original[key])
        self.assertEqual(result['current_stage']['conditions'], stages[-1]['conditions'])
        self.assertEqual([s['kind'] for s in result['plan']['stages']], ['search', 'relative_land'])
        target = result['target_memory']['van']
        self.assertEqual(target['target_id'], 'van-1')
        self.assertEqual(target['target_revision'], 5)
        self.assertEqual(target['front_relation_quality'], quality)
        self.assertEqual(target['observation_refs'], ['source', 'current'])
        for key in ('source_capture_id', 'image_sha256', 'yaw_variance_rad2', 'current_front_relation'):
            self.assertEqual(target['orientation'][key], orientation[key])
        self.assertEqual(result['goal_state']['prepared']['selected_id'], 'goal_1')
        self.assertEqual(result['goal_state']['need_kind'], 'IMPROVE_FRONT_RELATION')
        self.assertEqual(result['goal_state']['object_frame_attempt_count_total'], 6)
        self.assertEqual(result['goal_state']['object_frame_attempts'], original['goal_state']['object_frame_attempts'][-2:])
        self.assertEqual(result['goal_state']['candidate_summary'][0]['position_enu_m'], [1., 2., 3.])
        self.assertNotIn('private-ticket', projected[-1].content)
        again, _ = TaskContextMemory().project(projected, ('thread', 'active'))
        self.assertEqual(again, projected)

    def test_surface_consensus_samples_are_projected_but_fitting_uncertainty_and_identity_remain(self):
        quality = {'measurement_acquired': True, 'relation_usable': False,
                   'normal_uncertainty_margin_rad': 1.2,
                   'information_gap': ['ANGULAR_UNCERTAINTY']}
        orientation = {'basis': 'opaque_front_surface_consensus_v1', 'source_capture_id': 'actual-source',
            'image_sha256': {'left': 'a'*64, 'right': 'b'*64}, 'face_box': [.2, .3, .8, .7],
            'coordinate_epoch': {'vio_epoch': 2, 'ekf_counters': {'xy': 2, 'z': 2, 'heading': 2}},
            'front_covariance_enu_m2': [[.2, 0., 0.], [0., .3, 0.], [0., 0., .4]],
            'yaw_variance_rad2': .36, 'current_front_relation': {'minimum_front_distance_m': -1.},
            'fitting': {'valid_samples': 64, 'inliers': 40, 'lateral_span_px': 200.,
                        'uncertainty_basis': 'correlated_samples_no_count_reduction'},
            'sample_regions': [[.2+i*.001, .3, .21+i*.001, .31] for i in range(64)],
            'sample_measurements': [{'status': 'ok', 'image_sha256': {'left': 'a'*64},
                                     'raw_stereo_audit': 'x'*2000} for _ in range(64)]}
        payload = {'stage_revision': 4, 'owner': ['thread', 'active'],
            'plan': {'stages': [{'kind': 'relative_land', 'relation': 'object_front'}]},
            'next_tool': 'report_target_frame', 'budget': {'max_observations': 12},
            'target_memory': {'van': {'target_id': 'van-1', 'target_revision': 9,
                'orientation': orientation, 'front_relation_quality': quality}}}
        result, _, raw, projected = self.project(payload, 'report_target_frame')
        target = result['target_memory']['van']
        expected = {key: value for key, value in orientation.items()
                    if key not in ('sample_measurements', 'sample_regions')}
        self.assertEqual(target['orientation'], expected)
        self.assertEqual(target['front_relation_quality'], quality)
        self.assertEqual(target['target_revision'], 9)
        self.assertEqual(result['budget'], payload['budget'])
        omitted = result['_context_projection']['omitted_collection_entries']
        self.assertEqual(omitted['sample_measurements'], 64)
        self.assertEqual(omitted['sample_regions'], 64)
        original = json.loads(raw[-1].content)['target_memory']['van']['orientation']
        self.assertEqual(len(original['sample_measurements']), 64)
        self.assertEqual(len(original['sample_regions']), 64)
        self.assertLess(len(projected[-1].content), len(raw[-1].content)/10)

    def test_errors_unknown_tools_and_non_json_are_not_projected(self):
        for name, content, status in [
            ('report_scene', {'status': 'error', 'error': 'actual failure', 'vlm_calls': ['x'*2000]}, 'success'),
            ('report_scene', {'status': 'ok', 'vlm_calls': ['x'*2000]}, 'error'),
            ('custom_tool', {'status': 'ok', 'vlm_calls': ['x'*2000]}, 'success'),
        ]:
            with self.subTest(name=name, status=status):
                message = tool_result(name, content, status=status)
                raw = [HumanMessage('当前指令', id='active'), message]
                projected, _ = TaskContextMemory().project(raw, ('thread', 'active'))
                self.assertIs(projected[-1], message)
        message = ToolMessage('not json', name='report_scene', tool_call_id='invalid')
        raw = [HumanMessage('当前指令', id='active'), message]
        self.assertIs(TaskContextMemory().project(raw, ('thread', 'active'))[0][-1], message)

    def test_full_mode_and_current_message_pairs_remain_exact_in_both_message_representations(self):
        raw = [{'role': 'user', 'id': 'active', 'content': '完整原句'},
               {'role': 'assistant', 'id': 'assistant', 'content': '',
                'tool_calls': [{'id': 'call', 'name': 'report_scene', 'args': {}}]},
               {'role': 'tool', 'id': 'result', 'tool_call_id': 'call', 'name': 'report_scene',
                'content': json.dumps({'status': 'ok', 'capture_id': 'actual', 'vlm_calls': ['x'*5000]})}]
        original = copy.deepcopy(raw)
        projected, _ = TaskContextMemory().project(raw, ('thread', 'active'))
        self.assertEqual(raw, original)
        self.assertEqual(projected[-1]['tool_call_id'], 'call')
        self.assertEqual(projected[-1]['id'], 'result')
        self.assertEqual(json.loads(projected[-1]['content'])['capture_id'], 'actual')
        self.assertEqual(TaskContextMemory(mode='full').project(raw, ('thread', 'active'))[0], raw)


class CurrentTaskGraphTests(unittest.TestCase):
    def test_real_agent_graph_projects_model_copy_but_keeps_checkpoint_report_and_frame_ownership(self):
        from test_single_vlm import GraphTests, capture_request, report_request, image_urls, tool_payload
        from single_vlm import turn_owner
        helper = GraphTests()
        def final(messages):
            report = tool_payload(messages, 'report_scene')
            self.assertEqual(report['status'], 'ok')
            self.assertNotIn('vlm_calls', report)
            self.assertIn('_context_projection', report)
            self.assertEqual(image_urls(messages), [])
            self.assertEqual(turn_owner(messages, {'configurable': {'thread_id': 'thread-a'}}), ('thread-a', 'turn-a'))
            return AIMessage('报告已保存')
        graph, model, store, client = helper.build([capture_request, report_request, final])
        result, config = helper.invoke(graph)
        report = tool_payload(result['messages'], 'report_scene')
        self.assertGreater(len(report['vlm_calls']), 0)
        self.assertNotIn('_context_projection', report)
        checkpoint = tool_payload(graph.get_state(config).values['messages'], 'report_scene')
        self.assertEqual(report, checkpoint)
        capture_id = report['capture_id']
        self.assertTrue(store.get(('thread-a', 'turn-a'), capture_id, require_delivered=True))
        with self.assertRaises(RuntimeError):
            store.get(('thread-a', 'other-turn'), capture_id)
        self.assertEqual([len(image_urls(messages)) for messages in model.calls], [0, 1, 0])
        self.assertEqual(client.camera_reads, ['left'])
        self.assertEqual(client.calls, [])


if __name__ == '__main__':
    unittest.main()
