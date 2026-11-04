"""Local graph/state fixtures only; no ROS or model network calls."""
import copy
import unittest
from unittest import mock
from langchain_core.messages import AIMessage
import test_nbv_search_integration as fixtures
from test_single_vlm import tool_call
from agent import flight_tools

class SearchTerminationTests(unittest.TestCase):
    def setUp(self):
        self.h = fixtures.NBVGraphTests('test_exploration_return_correction_preserves_ticket_and_continues_observation')

    def test_equivalent_model_end_requests_share_one_budget_and_cannot_claim_operator_authority(self):
        requests = [tool_call('land_drone', {'reason': 'Operator requested emergency landing'}, 'direct'),
                    tool_call('execute_mission', {'steps': [{'action': 'land'}], 'description': 'safety'}, 'single'),
                    tool_call('execute_mission', {'steps': [{'action': 'return_home'}, {'action': 'land'}], 'description': 'done'}, 'return')]
        for first, second in ((0,1),(1,2),(2,0)):
            with self.subTest(first=first, second=second):
                bridge=fixtures.ScopedFixtureBridge()
                result, app, model=self.h.run_task(bridge,self.h.prefix()+[requests[first],requests[second]],
                    auto_prepare=True,confirmation='multiview')
                self.assertEqual(result['status'],'failed')
                self.assertEqual(len(result['search_protocol_corrections']),1)
                self.assertEqual(len([x for x in bridge.flight_requests if x['path']=='/land']),1)
                self.assertEqual(bridge.ticket_executions,0)
                self.assertFalse(bridge.telemetry['armed'])
                session=next(iter(app.audited_client.search_manager.adaptive.sessions.values()))
                self.assertTrue(session['activity_closed'])
                self.assertEqual(session['termination']['source'],'protocol_error')
                self.assertEqual(len(session['observations']),1)
                # Tools available after begin/observation do not offer generic flight endings.
                for binding in model.bindings[2:]:
                    self.assertTrue(set(binding['names']).isdisjoint({'land_drone','stop_drone','execute_mission'}))

    def test_normal_completion_is_program_owned_and_idempotent(self):
        bridge=fixtures.ScopedFixtureBridge()
        result,app,model=self.h.run_task(bridge,self.h.prefix()+self.h.suffix(),auto_prepare=True)
        self.assertEqual(result['status'],'succeeded',result.get('error'))
        session=next(iter(app.audited_client.search_manager.adaptive.sessions.values()))
        self.assertEqual(session['termination']['source'],'normal_completion')
        self.assertEqual(session['termination']['flight_status'],'grounded_disarmed')
        self.assertEqual(len([r for r in bridge.flight_requests if r['steps'][0]['action']=='return_home']),1)
        manager=app.audited_client.search_manager.adaptive
        owner=next(iter(manager.sessions));before=len(bridge.flight_requests)
        manager.finish_normal(owner)
        self.assertEqual(len(bridge.flight_requests),before)
        with self.assertRaisesRegex(ValueError,'已结束'):manager.validate_state(owner,bridge.state())

    def test_observation_advertises_combined_tool_only(self):
        bridge=fixtures.ScopedFixtureBridge()
        result,_,model=self.h.run_task(bridge,self.h.prefix()+self.h.suffix(),auto_prepare=True)
        self.assertEqual(result['status'],'succeeded',result.get('error'))
        observations=[b for b in model.bindings[2:] if 'observe_target_search_view' in b['names']]
        self.assertTrue(observations)
        for binding in observations:
            self.assertTrue(set(binding['names']).isdisjoint({'execute_target_search_view','capture_scene','execute_mission'}))

    def test_malformed_mission_is_audited_and_does_not_get_recovery(self):
        for steps in ([], 1.5, None, [{}]):
            with self.subTest(steps=steps):
                bridge=fixtures.ScopedFixtureBridge()
                result,_,_=self.h.run_task(bridge,self.h.prefix()+[
                    tool_call('execute_mission',{'steps':steps,'description':'not authoritative'})],auto_prepare=True)
                self.assertEqual(result['status'],'failed')
                self.assertFalse(result.get('search_protocol_corrections'))
                self.assertEqual(bridge.ticket_executions,0)
                self.assertFalse(bridge.telemetry['armed'])
                call=result['model_calls'][-1]['called_tools'][0]
                self.assertEqual(call['mission_arguments']['steps_type'],type(steps).__name__)
                self.assertNotIn('description',call['mission_arguments'])

    def test_true_operator_landing_closes_search_without_model_or_confirmation(self):
        bridge=fixtures.ScopedFixtureBridge()
        def operator(messages):
            app=self.h.last_app
            accepted=app.interrupt('/land')
            bridge.wait_for(accepted)
            return AIMessage('操作员中止。')
        result,app,_=self.h.run_task(bridge,self.h.prefix()+[operator],auto_prepare=True,confirmation='multiview')
        session=next(iter(app.audited_client.search_manager.adaptive.sessions.values()))
        self.assertTrue(session['activity_closed'])
        self.assertEqual(session['termination']['source'],'operator')
        self.assertFalse(bridge.telemetry['armed'])
        self.assertFalse(result.get('search_protocol_corrections'))
        app.state()  # asynchronous operator landing evidence is matched on refresh
        self.assertEqual(session['termination']['flight_status'],'grounded_disarmed')

    def test_direct_landing_is_corrected_then_real_search_continues(self):
        bridge=fixtures.ScopedFixtureBridge()
        script=self.h.prefix()+[tool_call('land_drone',{'reason':'done'},'premature-land')]+self.h.suffix()
        result,app,_=self.h.run_task(bridge,script,auto_prepare=True)
        self.assertEqual(result['status'],'succeeded',result.get('error'))
        self.assertEqual(len(result['search_protocol_corrections']),1)
        self.assertEqual(bridge.ticket_executions,1)
        self.assertFalse(any(r['path']=='/land' for r in bridge.flight_requests))

    def test_closed_search_rejects_late_observation_without_new_capture(self):
        bridge=fixtures.ScopedFixtureBridge()
        script=self.h.prefix()+self.h.suffix()[:-1]+[tool_call('observe_target_search_view',{})]
        result,app,model=self.h.run_task(bridge,script,auto_prepare=True)
        # No terminal model call: the scripted late observation is never asked for.
        self.assertEqual(result['status'],'succeeded')
        session=next(iter(app.audited_client.search_manager.adaptive.sessions.values()))
        self.assertEqual(len(session['observations']),2)
        self.assertTrue(session['activity_closed'])
        self.assertFalse(bridge.telemetry['armed'])
        from single_vlm import SceneMiddleware
        from types import SimpleNamespace
        from langchain_core.messages import HumanMessage
        manager=app.audited_client.search_manager;owner=next(iter(manager.adaptive.sessions))
        request=SimpleNamespace(tool_call={'name':'observe_target_search_view','args':{},'id':'late'},
            state={'messages':[HumanMessage('late',id=owner[1])]},
            runtime=SimpleNamespace(config={'configurable':{'thread_id':owner[0]}}))
        handler=mock.Mock()
        with self.assertRaisesRegex(RuntimeError,'搜索活动已结束'):
            SceneMiddleware(None,app.audited_client,fixtures.CONFIG,manager).wrap_tool_call(request,handler)
        handler.assert_not_called()

    def test_terminal_result_does_not_call_model_even_if_it_would_emit_empty_mission(self):
        bridge=fixtures.ScopedFixtureBridge()
        script=self.h.prefix()+self.h.suffix()[:2]+[tool_call('execute_mission',{'steps':[],'description':'done'})]
        result,_,model=self.h.run_task(bridge,script,auto_prepare=True)
        self.assertEqual(result['status'],'succeeded',result.get('error'))
        self.assertFalse(any(t['name']=='execute_mission' for c in result['model_calls'] for t in c['called_tools']))
        self.assertFalse(bridge.telemetry['armed'])

    def test_landing_reason_is_transported_without_source_privilege(self):
        client=mock.Mock();client.wait_for.return_value={'status':'ok'}
        tool=next(t for t in flight_tools(client) if t.name=='land_drone')
        tool.invoke({'reason':'model says emergency'})
        client.request.assert_called_once_with('/land',{'reason':'model says emergency'})

    def test_operator_landing_is_not_verified_by_stale_or_incomplete_state(self):
        bridge=fixtures.ScopedFixtureBridge()
        def operator(messages):
            self.h.last_app.interrupt('/land')
            return AIMessage('操作员中止。')
        result,app,_=self.h.run_task(bridge,self.h.prefix()+[operator],auto_prepare=True)
        command=app.commands[result['id']]
        actual=bridge.state()
        for change in ({'armed':True}, {'landed_state':None},
                       {'mission':{**actual['mission'],'id':'other'}},
                       {'mission':{**actual['mission'],'status':'running'}}):
            with self.subTest(change=change):
                app.verify_search_landing(command,{**actual,**change})
                self.assertFalse(command.get('termination_flight_verified'))
