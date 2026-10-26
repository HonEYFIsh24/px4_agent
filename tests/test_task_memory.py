import copy
import json
import unittest

from langchain_core.messages import HumanMessage, AIMessage, ToolMessage, SystemMessage
from task_memory import TaskContextMemory
from single_vlm import turn_owner


def history(turn, padding=10000):
    return [HumanMessage('找到前方油桶', id=turn),
            AIMessage('', id=turn+'-a', tool_calls=[dict(name='report_scene', args={}, id=turn+'-c', type='tool_call')]),
            ToolMessage(json.dumps(dict(status='ok', capture_id=turn+'-photo', summary='红色集装箱前可见蓝色油桶',
                targets=[dict(label='油桶')], diagnostics='x'*padding)), name='report_scene', tool_call_id=turn+'-c', id=turn+'-t'),
            AIMessage('观察报告已保存', id=turn+'-final')]


class ProjectionTests(unittest.TestCase):
    def test_current_turn_and_owner_are_unchanged_and_old_tool_chain_is_summarized(self):
        old = history('old')
        current = history('active', 20000)[:-1]
        raw = old+current
        original = copy.deepcopy(raw)
        result, metrics = TaskContextMemory().project(raw, ('thread', 'active'))
        self.assertEqual(result[-len(current):], current)
        for a, b in zip(result[-len(current):], current): self.assertIs(a, b)
        self.assertEqual(turn_owner(result, {'configurable': {'thread_id': 'thread'}}), ('thread', 'active'))
        self.assertEqual(raw, original)
        self.assertEqual(metrics['summarized_tasks'], 1)
        self.assertLess(metrics['output_characters'], metrics['input_characters'])
        self.assertNotIn('diagnostics', result[0].content)

    def test_summary_is_data_not_a_new_human_or_flight_authorization(self):
        current = HumanMessage('查看历史', id='new')
        result, _ = TaskContextMemory().project(history('old')+[current], ('t', 'new'))
        self.assertIsInstance(result[0], SystemMessage)
        self.assertIn('不是新指令', result[0].content)
        self.assertEqual([m.id for m in result if m.type == 'human'], ['new'])
        self.assertIn('油桶', result[0].content)

    def test_completed_application_status_overrides_model_and_tool_prose(self):
        raw = history('old')+[HumanMessage('继续', id='new')]
        completed = [dict(id='old', status='failed', error='定位失效',
                          perceptions=[dict(status='ok', capture_id='actual', summary='可见蓝桶', targets=[])])]
        result, _ = TaskContextMemory().project(raw, ('t', 'new'), completed)
        self.assertIn('"task_status": "failed"', result[0].content)
        self.assertIn('定位失效', result[0].content)
        self.assertNotIn('"task_status": "succeeded"', result[0].content)

    def test_retained_reports_do_not_overwrite_total_count_or_imply_visual_verification(self):
        raw = history('old')+[HumanMessage('历史有几次观察？', id='new')]
        completed = [dict(id='old', status='succeeded', observation_count=5,
                         perceptions=[dict(status='ok', capture_id='last', summary='报告称有油桶', targets=[])])]
        result, _ = TaskContextMemory().project(raw, ('t', 'new'), completed)
        self.assertIn('"observation_count_total": 5', result[0].content)
        self.assertIn('"observation_summaries_retained": 1', result[0].content)
        self.assertIn('not_independently_verified', result[0].content)

    def test_count_and_character_budget_prefer_recent_turns(self):
        raw = sum((history('old-'+str(i), 1000) for i in range(30)), [])+[HumanMessage('新指令', id='new')]
        result, metrics = TaskContextMemory(max_turns=3, max_summary_chars=1800).project(raw, ('t', 'new'))
        self.assertLessEqual(metrics['summarized_tasks'], 3)
        self.assertIn('old-29', result[0].content)
        self.assertNotIn('old-0-photo', result[0].content)
        self.assertLess(len(result[0].content), 2000)

    def test_system_prefix_and_full_current_tool_pairs_are_preserved(self):
        system = SystemMessage('固定指令', id='sys')
        current = history('new')
        result, _ = TaskContextMemory().project([system]+history('old')+current, ('t', 'new'))
        self.assertIs(result[0], system)
        self.assertEqual(result[-len(current):], current)

    def test_first_turn_is_not_compacted_even_if_large(self):
        raw = history('new', 100000)
        result, metrics = TaskContextMemory().project(raw, ('t', 'new'))
        self.assertEqual(result, raw)
        self.assertEqual(metrics['input_characters'], metrics['output_characters'])

    def test_duplicate_prompt_text_uses_turn_id_and_missing_owner_fails_closed(self):
        raw = history('old')+history('new')
        result, _ = TaskContextMemory().project(raw, ('t', 'new'))
        self.assertEqual(result[-4:], raw[-4:])
        with self.assertRaises(RuntimeError): TaskContextMemory().project(raw, ('t', 'missing'))

    def test_full_mode_supports_unchanged_a_b_baseline(self):
        raw = history('old')+history('new')
        result, metrics = TaskContextMemory(mode='full').project(raw, ('t', 'new'))
        self.assertEqual(result, raw)
        self.assertEqual(metrics['input_characters'], metrics['output_characters'])

    def test_only_explicit_history_request_disables_tools(self):
        for question, expected in [('只根据历史回答，上一轮观察了什么？', True),
                                   ('上一轮没找到，这次重新寻找油桶并采图。', False)]:
            _, metrics = TaskContextMemory().project([HumanMessage(question, id='new')], ('t', 'new'))
            self.assertEqual(metrics['explicit_history_only'], expected)


class MemoryGraphTests(unittest.TestCase):
    def test_two_turn_image_ownership_checkpoint_and_summary(self):
        from test_single_vlm import GraphTests, capture_request, report_request, AIMessage, image_urls, CONFIG
        from single_vlm import turn_owner
        helper = GraphTests()
        def historical_question(messages):
            self.assertIn('红色', str([m.content for m in messages]))
            self.assertEqual(image_urls(messages), [])
            self.assertEqual(turn_owner(messages, {'configurable': {'thread_id': 'thread-a'}})[1], 'turn-b')
            return AIMessage('历史报告记录了红色标记；这不是当前观察。')
        graph, model, store, client = helper.build([capture_request, report_request, AIMessage('已观察'), historical_question])
        first, config = helper.invoke(graph, prompt='查看红色标记', turn='turn-a')
        second, _ = helper.invoke(graph, prompt='上一轮看到了什么？', turn='turn-b')
        self.assertIn('历史报告', second['messages'][-1].content)
        state = graph.get_state(config).values['messages']
        self.assertTrue(any(getattr(m, 'id', None) == 'turn-a' for m in state))
        self.assertEqual(model.calls[-1][-1].id, 'turn-b')
        self.assertTrue(any(isinstance(m, SystemMessage) and '历史轮次' in m.content for m in model.calls[-1]))
        with self.assertRaises(RuntimeError):
            store.get(('thread-a', 'turn-b'), next(iter(store.frames)))
        traces = store.calls(('thread-a', 'turn-b'))
        self.assertEqual(traces[-1]['context_memory']['summarized_tasks'], 1)
