"""Adaptive decisions, real graph wiring, bounded completion; no network/ROS."""
import copy
import math
import unittest

from autonomous_search import AdaptiveSearch
from search_tools import coordinate_epoch
from test_search_tools import MetadataBridge, OWNER


class AdaptiveBridge(MetadataBridge):
    def __init__(self):
        super().__init__()
        self.telemetry['position_enu_m'] = [0., 0., 3.]
        self.telemetry['navigation'].update(method='static_3d', source='verified_static_collision_map_3d', map_registered=True)
        self.telemetry['navigation']['registration_id'] = 'adaptive-registration'
        self.reachable = True

    def state(self):
        result = super().state()
        result['stamp_sim_s'] = 10.02
        return result

    def capture_stereo(self):
        pair = super().capture_stereo()
        pair['stamp_sim_s'] = 10.
        pair['body_pose_enu'].update(position_m=list(self.telemetry['position_enu_m']), stamp_sim_s=10., age_seconds=.1)
        pair['navigation_registration_id'] = self.telemetry['navigation'].get('registration_id')
        if 'external_vision' in self.telemetry:
            pair['external_vision'] = copy.deepcopy(self.telemetry['external_vision'])
        return pair

    def request(self, path, payload=None):
        assert path == '/navigation/preview'
        self.reads.append(copy.deepcopy(payload))
        return dict(reachable=self.reachable, reason='occupied goal' if not self.reachable else None)


class AdaptiveTests(unittest.TestCase):
    def setUp(self):
        self.client = AdaptiveBridge()
        self.now = 0.
        self.search = AdaptiveSearch(self.client, clock=lambda: self.now)
        self.search.begin(OWNER, '油桶')
        self.count = 0

    def report(self, targets=None, **updates):
        self.count += 1
        identifier = 'capture-' + str(self.count)
        frame = dict(owner=OWNER, delivered=True, capture_id=identifier, snapshot=dict(
            coordinate_epoch=coordinate_epoch(self.client.telemetry),
            stamp_sim_s=10. + self.count, navigation_registration_id='adaptive-registration',
            body_pose_enu=dict(position_m=list(self.client.telemetry['position_enu_m']))))
        frame.update(updates)
        return self.search.consume(OWNER, frame, dict(status='ok', capture_id=identifier,
            summary='实际图像观察', targets=targets or []))

    def arrive(self, step):
        self.search.check_mission(OWNER, [step])
        self.client.telemetry['position_enu_m'] = [step['x'], step['y'], step['altitude']]

    def test_absence_continues_then_rises_then_finds_without_metric_depth(self):
        self.assertEqual(self.report()['search_status'], 'searching')
        view = self.search.next_view(OWNER, 'rise', '高墙遮挡')
        self.assertEqual(view['goto']['altitude'], 5.)
        self.arrive(view['goto'])
        result = self.report([{'label': '油桶', 'bbox': [.2, .2, .4, .5], 'stereo_location': {'status': 'unavailable'}}])
        self.assertEqual(result['search_status'], 'target_found')
        self.assertFalse(self.search.guidance(OWNER)[1])
        with self.assertRaises(ValueError):
            self.search.check_mission(OWNER, [{'action': 'return_home'}, {'action': 'land'}])
        self.client.telemetry.update(armed=False, landed_state=1)
        self.assertTrue(self.search.guidance(OWNER)[1])

    def test_left_and_forward_follow_body_heading_not_world_truth(self):
        s = self.search.sessions[OWNER]
        s['forward'] = [0., 1.]
        self.report()
        step = self.search.next_view(OWNER, 'left', '换位')['goto']
        self.assertAlmostEqual(step['x'], -2.)
        self.assertAlmostEqual(step['y'], 0.)

    def test_rejected_candidates_end_normally_without_flying_or_forging_observation(self):
        self.report()
        self.client.reachable = False
        r = self.search.next_view(OWNER, 'rise', '遮挡')
        self.assertEqual(r['search_status'], 'no_reachable_view')
        self.assertNotIn('goto', r)
        self.assertEqual(len(r['observations']), 1)

    def test_cannot_move_to_unchecked_or_measured_target_coordinates(self):
        self.report()
        self.search.next_view(OWNER, 'rise', '扩大视野')
        with self.assertRaises(ValueError):
            self.search.check_mission(OWNER, [dict(action='goto', x=8., y=0., altitude=3.)])

    def test_float_roundtrip_tolerance_is_small_and_does_not_change_waypoint(self):
        self.report()
        step = self.search.next_view(OWNER, 'rise', '扩大视野')['goto']
        rounded = dict(step, x=step['x']+1e-8)
        self.search.check_mission(OWNER, [rounded])
        with self.assertRaises(ValueError):
            self.search.check_mission(OWNER, [dict(step, x=step['x']+.01)])

    def test_no_new_point_before_first_observation_and_no_repeated_current_report(self):
        self.assertEqual(self.search.next_view(OWNER, 'auto', '搜索')['next_tool'], 'observe_target_search_view')
        self.report()
        with self.assertRaises(ValueError): self.report()

    def test_cross_turn_and_undelivered_report_rejected(self):
        with self.assertRaises(ValueError): self.report(owner=('other', 'turn'))
        with self.assertRaises(ValueError): self.report(delivered=False)

    def test_new_capture_must_be_at_planned_view(self):
        self.report()
        self.search.next_view(OWNER, 'rise', '遮挡')
        with self.assertRaises(ValueError): self.report()

    def test_time_budget_ends_search_and_blocks_new_flight(self):
        self.report()
        self.now = 901.
        self.assertEqual(self.search.next_view(OWNER, 'auto', '继续')['search_status'], 'budget_exhausted')
        with self.assertRaises(ValueError): self.search.check_mission(OWNER, [dict(action='goto', x=0., y=0., altitude=5.)])

    def test_observation_budget_and_radius_are_enforced(self):
        self.report()
        budget = self.search.summary(OWNER)['max_observations']
        self.assertEqual(budget, 16)
        for index in range(budget-1):
            r = self.search.next_view(OWNER, 'auto', '未发现')
            if 'goto' not in r: break
            self.arrive(r['goto'])
            self.assertLessEqual(math.dist(self.client.telemetry['position_enu_m'][:2], [0., 0.]), 30.)
            self.report()
            if index == 6:
                self.assertEqual(self.search.summary(OWNER)['search_status'], 'searching')
        self.assertEqual(self.search.summary(OWNER)['search_status'], 'budget_exhausted')
        self.assertEqual(len(self.search.summary(OWNER)['observations']), budget)

    def test_operator_or_tool_failure_forbids_resuming_search(self):
        self.report()
        step = self.search.next_view(OWNER, 'rise', '继续')['goto']
        self.search.fail(OWNER, 'interrupted')
        with self.assertRaises(ValueError): self.search.check_mission(OWNER, [step])
        with self.assertRaises(ValueError): self.search.check_mission(OWNER, [{'action': 'land'}])

    def test_vio_reset_invalidates_old_points(self):
        self.report()
        self.client.telemetry['external_vision'] = {'adapter': {'epoch': 1}}
        with self.assertRaises(ValueError): self.search.next_view(OWNER, 'auto', '继续')

    def test_first_observation_must_follow_binding_and_keep_registered_identity(self):
        frame = {'owner': OWNER, 'delivered': True, 'capture_id': 'first', 'snapshot': {
            'coordinate_epoch': coordinate_epoch(self.client.telemetry),
            'stamp_sim_s': self.search.sessions[OWNER]['binding']['stamp_sim_s'],
            'navigation_registration_id': 'adaptive-registration',
            'body_pose_enu': {'position_m': list(self.client.telemetry['position_enu_m'])}}}
        report = {'status': 'ok', 'capture_id': 'first', 'summary': 'frame', 'targets': []}
        with self.assertRaises(ValueError): self.search.consume(OWNER, frame, report)
        frame['snapshot']['stamp_sim_s'] += 1.
        frame['snapshot']['navigation_registration_id'] = 'other-registration'
        with self.assertRaises(ValueError): self.search.consume(OWNER, frame, report)
        frame['snapshot']['navigation_registration_id'] = 'adaptive-registration'
        self.client.telemetry['external_vision'] = {'adapter': {'epoch': 1}}
        with self.assertRaises(ValueError): self.search.consume(OWNER, frame, report)
        self.assertEqual(self.search.sessions[OWNER]['observations'], [])

    def test_coordinate_reset_before_first_observation_rejects_old_search(self):
        for changed in ('vio', 'heading'):
            with self.subTest(changed=changed):
                client = AdaptiveBridge()
                client.telemetry['external_vision'] = {'required': True, 'adapter': {'mode': 'fuse', 'epoch': 1},
                    'healthy': True, 'heading_alignment_pending': False,
                    'ekf': {'local_position_reset_counters': {'xy': 2, 'z': 2, 'heading': 1},
                            'heading_good_for_control': True,
                            'mag_alignment': {'aligned_in_flight': True, 'yaw_aligned': True, 'in_air': True}}}
                search = AdaptiveSearch(client)
                search.begin(OWNER, '油桶')
                if changed == 'vio':
                    client.telemetry['external_vision']['adapter']['epoch'] += 1
                else:
                    client.telemetry['external_vision']['ekf']['local_position_reset_counters']['heading'] += 1
                with self.assertRaisesRegex(ValueError, '坐标系已重置'):
                    search.validate_state(OWNER, client.state())
                self.assertEqual(search.sessions[OWNER]['observations'], [])
                with self.assertRaises(ValueError): search.check_mission(OWNER, [{'action': 'land'}])

    def test_reset_before_takeoff_rejects_search_takeoff(self):
        client = AdaptiveBridge()
        client.telemetry.update(armed=False, landed_state=1)
        client.telemetry['external_vision'] = {'required': True, 'adapter': {'mode': 'fuse', 'epoch': 1},
            'ekf': {'local_position_reset_counters': {'xy': 2, 'z': 2, 'heading': 1}}}
        search = AdaptiveSearch(client)
        search.begin(OWNER, '油桶')
        client.telemetry['external_vision']['ekf']['local_position_reset_counters']['heading'] += 1
        with self.assertRaisesRegex(ValueError, '坐标系已重置'):
            search.check_mission(OWNER, [{'action': 'takeoff', 'altitude': 3.}])
        with self.assertRaises(ValueError): search.check_mission(OWNER, [{'action': 'land'}])

    def test_search_origin_capture_requires_same_coordinate_identity(self):
        for failure in ('missing', 'pair_reset', 'state_reset'):
            with self.subTest(failure=failure):
                client = AdaptiveBridge()
                client.telemetry['external_vision'] = {'required': True, 'adapter': {'mode': 'fuse', 'epoch': 1},
                    'ekf': {'local_position_reset_counters': {'xy': 2, 'z': 2, 'heading': 1}}}
                original_capture = client.capture_stereo
                def capture():
                    pair = original_capture()
                    if failure == 'missing':
                        del pair['external_vision']
                    elif failure == 'pair_reset':
                        pair['external_vision']['ekf']['local_position_reset_counters']['heading'] += 1
                    else:
                        client.telemetry['external_vision']['adapter']['epoch'] += 1
                    return pair
                client.capture_stereo = capture
                search = AdaptiveSearch(client)
                with self.assertRaisesRegex(ValueError, '坐标系已改变'):
                    search.begin(OWNER, '油桶')
                self.assertNotIn(OWNER, search.sessions)

    def test_explicit_hold_allows_final_report_without_forced_landing(self):
        self.search.sessions[OWNER]['end_behavior'] = 'hold'
        self.report([{'label': '油桶'}])
        self.assertTrue(self.search.guidance(OWNER)[1])

    def test_json_flight_failure_closes_search_and_landing_finishes_once(self):
        from types import SimpleNamespace
        from langchain_core.messages import HumanMessage, ToolMessage
        from single_vlm import SceneMiddleware
        from search_tools import SearchManager
        manager = SearchManager(self.client)
        manager.adaptive = self.search
        request = SimpleNamespace(tool_call={'name': 'execute_target_search_view', 'args': {}, 'id': 'failed-flight'},
            state={'messages': [HumanMessage('找油桶', id=OWNER[1])]},
            runtime=SimpleNamespace(config={'configurable': {'thread_id': OWNER[0]}}))
        middleware = SceneMiddleware(None, self.client, {'model': 'local-test'}, manager)
        with self.assertRaisesRegex(RuntimeError, 'mission endpoint lacks clearance'):
            middleware.wrap_tool_call(request, lambda _: ToolMessage(name='execute_target_search_view',
                content='{"status":"error","error":"mission endpoint lacks clearance"}', tool_call_id='failed-flight'))
        self.assertEqual(self.search.summary(OWNER)['search_status'], 'failed')
        self.assertTrue(self.search.sessions[OWNER]['activity_closed'])
        self.assertFalse(self.search.guidance(OWNER)[1])
        self.client.telemetry.update(armed=False, landed_state=1)
        self.assertTrue(self.search.guidance(OWNER)[1])

    def test_model_telemetry_is_compact_without_losing_flight_result(self):
        from single_vlm import compact_flight_evidence
        raw = self.client.state()
        raw['external_vision'] = {'healthy': True, 'ekf': {'local_position_reset_counters': {'xy': 1, 'z': 2, 'heading': 3},
                                                          'raw_uorb_dump': 'x'*50000}}
        compact = compact_flight_evidence(raw)
        self.assertEqual(compact['mission'], raw['mission'])
        self.assertEqual(compact['external_vision']['ekf']['local_position_reset_counters'], {'xy': 1, 'z': 2, 'heading': 3})
        self.assertNotIn('raw_uorb_dump', compact['external_vision']['ekf'])
        self.assertIn('raw_uorb_dump', raw['external_vision']['ekf'])


class AutonomousGraphTests(unittest.TestCase):
    def test_combined_tool_cannot_fly_in_observe_only_mode(self):
        import time
        from langchain_core.messages import AIMessage
        from test_single_vlm import LocalScriptModel, tool_call, CONFIG
        from single_vlm import build_single_vlm_agent
        from agent import flight_tools, SYSTEM_PROMPT
        from web_server import WebApplication
        bridge = AdaptiveBridge()
        bridge.telemetry.update(armed=False, landed_state=1, position_enu_m=[0., 0., 0.])
        model = LocalScriptModel(script=[tool_call('begin_target_search', {'query': '油桶'}),
            tool_call('observe_target_search_view', {}), AIMessage('仅观察模式没有执行起飞。')])
        app = WebApplication(bridge, CONFIG, lambda client, config:
            build_single_vlm_agent(client, config, flight_tools(client), SYSTEM_PROMPT, model=model))
        command = app.submit('只描述当前画面', mode='observe')
        deadline = time.monotonic()+3
        while time.monotonic()<deadline:
            result = app.command(command['id'])
            if result['status'] != 'running': break
            time.sleep(.01)
        self.assertEqual(result['status'], 'failed')
        self.assertFalse(bridge.telemetry['armed'])
        self.assertEqual(result['flight_missions'], [])
        self.assertIn('仅允许视觉观察', result['error'])

    def test_graph_corrects_premature_question_then_continues_and_lands(self):
        self.run_graph(False)

    def test_combined_flight_capture_preserves_real_image_report_and_landing(self):
        self.run_graph(True)

    def run_graph(self, combined):
        from langchain_core.messages import AIMessage, HumanMessage
        from test_single_vlm import LocalScriptModel, tool_call, tool_payload, CONFIG
        from test_search_graph_integration import LocalStereoBridge
        from agent import flight_tools, SYSTEM_PROMPT
        from single_vlm import build_single_vlm_agent

        class Bridge(LocalStereoBridge):
            def __init__(self):
                super().__init__()
                self.telemetry['external_vision']['ekf'].update(primary_instance=0,
                    local_velocity_reset_counters={'vxy': 2, 'vz': 2})

            def execute(self, steps, description='', on_progress=None):
                return self.wait_for(self.request('/mission', dict(steps=steps, description=description)), on_progress)

            def state(self):
                s = super().state()
                s['stamp_sim_s'] = 42. + self.counter + .02
                s['navigation'].pop('map', None)
                s['navigation'].update(method='static_3d', source='verified_static_collision_map_3d',
                    map_registered=s['armed'], bounds_xy_enu_m=[-10., -10., 16., 16.],
                    obstacle_inflation_m=1., visible_clearance_m=1.2,
                    registration_id='fixture-registration' if s['armed'] else None)
                return s

            def wait_for(self, accepted, on_progress=None):
                result = super().wait_for(accepted, on_progress)
                if accepted['mission']['steps'][0]['action'] == 'takeoff':
                    ev = self.telemetry['external_vision']
                    ev['heading_alignment_pending'] = False
                    ev['ekf'].update(heading_good_for_control=True,
                        mag_alignment={'aligned_in_flight': True, 'yaw_aligned': True, 'in_air': True})
                    return self.state()
                return result

            def capture_stereo(self):
                pair = super().capture_stereo()
                pair['navigation_registration_id'] = 'fixture-registration' if self.telemetry['armed'] else None
                return pair

            def request(self, path, payload=None):
                if path == '/navigation/preview': return {'reachable': True}
                return super().request(path, payload)

        def report(found):
            def response(messages):
                capture = tool_payload(messages, 'observe_target_search_view' if combined else 'capture_scene')
                return tool_call('report_scene', dict(capture_id=capture['capture_id'], summary='发现油桶' if found else '高墙遮挡，未发现油桶',
                    targets=[dict(label='油桶', bbox=[.2, .2, .4, .5], confidence=None,
                                  surroundings=['旁边可见墙体'], position_basis='image_only')] if found else [], limitations=[]))
            return response

        def move(messages):
            return tool_call('execute_target_search_view', {})

        bridge = Bridge()
        script = [
            tool_call('get_drone_state', {}), tool_call('begin_target_search', {'query': '油桶'}),
            tool_call('execute_target_search_view', {}),
            tool_call('capture_scene', dict(query='油桶')), report(False),
            AIMessage('未看到油桶，要不要升高？'),
            tool_call('next_target_search_view', dict(strategy='rise', reason='高墙遮挡')),
            move, tool_call('capture_scene', dict(query='油桶')), report(True),
            tool_call('execute_mission', dict(steps=[dict(action='return_home'), dict(action='land')], description='返航降落')),
            AIMessage('找到油桶，照片报告已保存，返航降落完成。')]
        if combined:
            script[2:4] = [tool_call('observe_target_search_view', {})]
            # Replace move+capture after next-view selection by one flight/capture call.
            index = script.index(move)
            script[index:index+2] = [tool_call('observe_target_search_view', {})]
        model = LocalScriptModel(script=script)
        graph = build_single_vlm_agent(bridge, CONFIG, flight_tools(bridge), SYSTEM_PROMPT, model=model)
        result = graph.invoke({'messages': [HumanMessage('找到前方油桶并报告周围情况', id='autonomous-turn')]},
            {'configurable': {'thread_id': 'autonomous-test'}, 'recursion_limit': 60, 'max_concurrency': 1})
        from langchain_core.messages import ToolMessage
        import json
        reports = [json.loads(m.content) for m in result['messages'] if isinstance(m, ToolMessage) and m.name == 'report_scene']
        errors = [(m.name, m.content) for m in result['messages'] if isinstance(m, ToolMessage) and json.loads(m.content).get('status') == 'error']
        self.assertEqual(errors, [])
        self.assertEqual(len(reports), 2)
        self.assertEqual(len(model.calls), 8 if combined else 10)
        self.assertEqual(len(bridge.flight_requests), 3)
        self.assertFalse(bridge.state()['armed'])
        self.assertNotIn('要不要', result['messages'][-1].content)
        self.assertNotIn('要不要', str([m.content for m in result['messages']]))
        self.assertEqual(tool_payload(result['messages'], 'report_scene')['search_progress']['search_status'], 'target_found')
