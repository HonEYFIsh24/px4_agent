"""Task admission and completion tests: mocked bridge/model only, no flight."""
import copy
import unittest

from langchain_core.messages import AIMessage
from task_contract import assess_task, search_result
from test_web_server import FakeBridge, application, completed
from test_single_vlm import tool_call
import test_nbv_search_integration as fixtures


class TaskContractTests(unittest.TestCase):
    def test_known_relative_endings_are_rejected_without_model_or_transport(self):
        for text in ('找到白色货车后降落在货车前方。', '找到货车，在其前方着陆',
                     '找到帐篷后降落在它旁边', '在货车车顶降落',
                     'Find the white truck and land in front of it.',
                     '找到白色货车后在车头前方悬停', '找到红色水塔后降落在水塔旁边',
                     '寻找货车，没找到就保持悬停'):
            with self.subTest(text=text):
                bridge = FakeBridge()
                def forbidden(*args): self.fail('unsupported task invoked the model')
                app = application(bridge, forbidden)
                result = app.submit(text)
                self.assertEqual(result['status'], 'failed')
                self.assertEqual(result['task_contract']['original_request'], text)
                self.assertFalse(result['task_outcome']['user_goal_completed'])
                self.assertEqual(bridge.calls, [])
                self.assertIsNone(app.active_id)
                self.assertIsNone(app.current_command)

    def test_explicit_success_hold_and_default_unmet_return_are_independent(self):
        contract = assess_task('找到白色货车后保持悬停')
        self.assertEqual(contract['admission'], 'accepted')
        self.assertEqual(contract['search_success_end'], 'hold')
        self.assertEqual(contract['search_unmet_end'], 'return_home_land_if_healthy_else_owned_safety_cleanup')
        self.assertEqual(assess_task('找到货车后不要悬停，返航降落')['search_success_end'], 'return_home_land')

    def test_supported_search_and_real_operator_landing_not_blocked(self):
        for text in ('找到前方的白色货车，完成后返航降落。', '寻找前方的油桶，拍照后返航降落',
                     '找到前方的白色货车后降落',
                     '立即降落', '在起飞点降落', '找到白色货车，不要在货车前方降落，返航降落。',
                     'Find the truck in front of us, then return home and land.'):
            with self.subTest(text=text):
                self.assertEqual(assess_task(text)['admission'], 'accepted')
        bridge = FakeBridge(); app = application(bridge, lambda *a: None)
        app.submit('找到货车后降落在货车前方')
        app.interrupt('/land')
        self.assertIn('/land', bridge.calls)

    def test_goal_outcome_cannot_be_upgraded_by_landing_or_prose(self):
        command = {'prompt': '找到白色货车后返航降落', 'reply': '全部完成',
                   'search_events': [{'result': {'search_status': 'budget_exhausted',
                       'target_confirmation_mode': 'multiview', 'termination': {
                           'source': 'normal_completion', 'purpose': 'unmet_goal_recovery',
                           'flight_status': 'grounded_disarmed'}}}],
                   'target_confirmation_events': [{'outcome': 'unconfirmed'}]}
        result = search_result(command)
        self.assertTrue(result['search_process_ended'])
        self.assertFalse(result['user_goal_completed'])
        self.assertEqual(result['flight_purpose'], 'unmet_goal_recovery')
        command['search_events'][0]['result']['search_status'] = 'target_found'
        self.assertFalse(search_result(command)['user_goal_completed'])
        command['target_confirmation_events'] = [{'outcome': 'confirmed'}]
        self.assertFalse(search_result(command)['user_goal_completed'])
        command['search_events'][0]['result']['termination']['purpose'] = 'goal_completion'
        self.assertTrue(search_result(command)['user_goal_completed'])
        command['search_events'][0]['result']['termination']['source'] = 'safety_fault'
        self.assertFalse(search_result(command)['user_goal_completed'])

    def test_model_prose_without_search_evidence_is_not_success(self):
        def script(client, payload, config):
            return {'messages': [payload['messages'][0], {'role': 'assistant', 'content': '找到了，全部完成'}]}
        bridge = FakeBridge(); app = application(bridge, script)
        result = completed(app, app.submit('找到白色货车后返航降落')['id'])
        self.assertEqual(result['status'], 'failed')
        self.assertFalse(result['task_outcome']['user_goal_completed'])
        self.assertEqual(result['task_outcome']['search_status'], 'not_started')
        self.assertEqual(bridge.calls, [])

    def test_model_hold_does_not_override_original_request(self):
        h = fixtures.NBVGraphTests('test_exploration_return_correction_preserves_ticket_and_continues_observation')
        prefix = h.prefix()
        prefix[1] = tool_call('begin_target_search', {'query': '蓝色油桶', 'end_behavior': 'hold'}, 'begin')
        result, app, _ = h.run_task(fixtures.ScopedFixtureBridge(), prefix+h.suffix(), auto_prepare=True)
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        session = next(iter(app.audited_client.search_manager.adaptive.sessions.values()))
        self.assertEqual(session['task_contract']['original_request'], result['prompt'])
        self.assertEqual(session['end_behavior'], 'land')
        self.assertTrue(result['task_outcome']['user_goal_completed'])

    def test_budget_exhaustion_is_failed_goal_with_program_return_not_safety_fault(self):
        h = fixtures.NBVGraphTests('test_exploration_return_correction_preserves_ticket_and_continues_observation')
        bridge = fixtures.ScopedFixtureBridge()
        def exhaust(messages):
            session = next(iter(h.last_app.audited_client.search_manager.adaptive.sessions.values()))
            session.update(status='budget_exhausted', pending=None, end_behavior='hold')
            return tool_call('get_drone_state', {'view': 'all'}, 'exhausted')
        result, app, _ = h.run_task(bridge, h.prefix()+[exhaust], auto_prepare=True, confirmation='multiview')
        self.assertEqual(result['status'], 'failed', result)
        self.assertEqual(result['search_outcome']['search_status'], 'budget_exhausted')
        self.assertFalse(result['task_outcome']['user_goal_completed'])
        self.assertTrue(result['task_outcome']['search_process_ended'])
        self.assertEqual(result['task_outcome']['flight_purpose'], 'unmet_goal_recovery')
        self.assertEqual(result['task_outcome']['flight_status'], 'grounded_disarmed')
        self.assertFalse(result.get('safety_landing'))
        self.assertFalse(bridge.telemetry['armed'])
        self.assertEqual(len([r for r in bridge.flight_requests if r['steps'][0]['action']=='return_home']), 1)
        self.assertIn('用户目标未完成', result['reply'])

    def test_contract_owner_and_direct_begin_cannot_bypass_admission(self):
        from autonomous_search import AdaptiveSearch
        bridge = FakeBridge()
        app = application(bridge, lambda *a: None)
        with self.assertRaisesRegex(ValueError, '当前活动指令'):
            app.audited_client.search_task_contract(('other', 'turn'))
        search = AdaptiveSearch(bridge)
        with self.assertRaisesRegex(ValueError, '不支持'):
            search.begin(('t', 'u'), '白色货车', original_request='找到白色货车后降落在货车前方')
        self.assertEqual(bridge.calls, [])
        self.assertEqual(search.sessions, {})

    def test_direct_agent_graph_also_rejects_before_model_call(self):
        from single_vlm import build_single_vlm_agent
        from test_single_vlm import LocalScriptModel
        from agent import flight_tools, SYSTEM_PROMPT
        from test_vision import CONFIG
        bridge = fixtures.ScopedFixtureBridge()
        model = LocalScriptModel(script=[AIMessage('must not be called')])
        agent = build_single_vlm_agent(bridge, CONFIG, flight_tools(bridge), SYSTEM_PROMPT, model=model)
        result = agent.invoke({'messages': [{'role': 'user', 'id': 'unsupported',
                                'content': '找到白色货车后降落在货车前方'}]},
                              {'configurable': {'thread_id': 'no-flight'}})
        self.assertIn('不支持', result['messages'][-1].content)
        self.assertEqual(model.calls, [])
        self.assertEqual(bridge.flight_requests, [])

    def test_exhaustion_with_changed_epoch_never_returns_on_old_coordinates(self):
        h = fixtures.NBVGraphTests('test_exploration_return_correction_preserves_ticket_and_continues_observation')
        bridge = fixtures.ScopedFixtureBridge()
        def exhaust(messages):
            session = next(iter(h.last_app.audited_client.search_manager.adaptive.sessions.values()))
            session.update(status='budget_exhausted', pending=None)
            bridge.telemetry['external_vision']['adapter']['epoch'] += 1
            return tool_call('get_drone_state', {'view': 'all'}, 'reset')
        result, _, _ = h.run_task(bridge, h.prefix()+[exhaust], auto_prepare=True)
        self.assertEqual(result['status'], 'failed')
        self.assertFalse(any(r['steps'][0]['action']=='return_home' for r in bridge.flight_requests))
        self.assertEqual(result['safety_landing']['status'], 'ok')
        self.assertFalse(result['task_outcome']['user_goal_completed'])

    def test_exhaustion_does_not_take_over_another_flight(self):
        h = fixtures.NBVGraphTests('test_exploration_return_correction_preserves_ticket_and_continues_observation')
        bridge = fixtures.ScopedFixtureBridge()
        def exhaust(messages):
            session = next(iter(h.last_app.audited_client.search_manager.adaptive.sessions.values()))
            session.update(status='budget_exhausted', pending=None)
            bridge.telemetry['mission']['id'] = 'unowned-flight'
            return tool_call('get_drone_state', {'view': 'all'}, 'unowned')
        result, _, _ = h.run_task(bridge, h.prefix()+[exhaust], auto_prepare=True)
        self.assertEqual(result['status'], 'failed')
        self.assertFalse(any(r['steps'][0]['action'] in ('return_home','land') for r in bridge.flight_requests))
        self.assertFalse(result['task_outcome']['user_goal_completed'])


if __name__ == '__main__': unittest.main()
