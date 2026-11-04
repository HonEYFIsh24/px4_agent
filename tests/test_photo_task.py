"""Offline framing contracts and real LangChain graph tests; no simulated flight claimed."""
import unittest
import copy
import json

from langchain_core.messages import AIMessage, HumanMessage
from agent import flight_tools
from photo_task import PhotoTask, is_overview_request
from single_vlm import FrameStore, build_single_vlm_agent
from test_single_vlm import (CameraClient, FlightCameraClient, LocalScriptModel, capture_request,
                             report_request, tool_call, tool_payload, image_urls)
from test_vision import CONFIG, JPEG_B64
from test_web_server import application, completed
from web_server import WebApplication

PROMPT = '请为我拍摄一张俯瞰整个果园的照片。'


class PhotoFlightClient(FlightCameraClient):
    def __init__(self, armed=False):
        super().__init__(armed=armed)
        self.position = [0., 0., 8.]
        self.epoch = {'vio_epoch': 0, 'ekf_counters': {'xy': 1, 'z': 1, 'heading': 1}}
        self.healthy = True
        self.photo_paths = []

    def state(self):
        s = super().state()
        s.update(simulation_only=True, connected=True, mode='OFFBOARD', position_enu_m=list(self.position), home_enu_m=[0., 0., 0.],
            external_vision={'healthy': self.healthy, 'used_for_flight_control': True,
                'adapter': {'epoch': self.epoch['vio_epoch']},
                'ekf': {'local_position_reset_counters': self.epoch['ekf_counters']}},
            navigation={'enabled': True, 'active': True, 'method': 'static_3d', 'map_registered': True,
                        'source': 'verified_static_collision_map_3d'})
        return s

    def capture_camera(self, side='left'):
        result = super().capture_camera(side)
        result['coordinate_epoch'] = copy.deepcopy(self.epoch)
        return result

    def request(self, path, payload=None):
        if path != '/navigation/photo-action':
            return super().request(path, payload)
        from photo_navigation import require_photo_navigation, relative_photo_goal
        require_photo_navigation(self.state(), payload['expected_mission_id'], payload['expected_epoch'])
        self.calls.append(path)
        self.photo_paths.append(copy.deepcopy(payload))
        if payload['operation'] == 'adjust':
            self.position, _ = relative_photo_goal(self.position, [0., 0., 0., 1.], payload['direction'],
                                                   payload['distance_m'], payload['altitude'])
            steps = [dict(action='goto', x=self.position[0], y=self.position[1], altitude=self.position[2])]
        elif payload['operation'] == 'turn':
            from photo_navigation import relative_photo_yaw
            yaw, _ = relative_photo_yaw([0., 0., 0., 1.], payload['direction'], payload['angle_deg'])
            steps = [dict(action='rotate', yaw_rad=yaw)]
        else:
            steps = [dict(action='return_home'), dict(action='land')]
            self.position = [0., 0., 0.]; self.armed = False
        self.mission = {'id': 'photo-'+str(len(self.calls)), 'status': 'succeeded', 'steps': steps, 'step': len(steps)}
        return {'mission': copy.deepcopy(self.mission)}

    def execute(self, steps, description, on_progress=None):
        return self.wait_for(self.request('/mission', {'steps': steps, 'description': description}))


def edge_evidence(status='clear'):
    return {edge: {'status': status, 'evidence': '离线夹具边界证据'} for edge in ('top','right','bottom','left')}


def review(subject='yes', view='yes', adjustment='', stop='', edges=None):
    def call(messages):
        current = next(m for m in reversed(messages) if isinstance(m, HumanMessage) and isinstance(m.content, list))
        metadata = json.loads(current.content[0]['text'])
        capture_id = metadata['capture_id']
        return tool_call('review_photo', dict(capture_id=capture_id, subject_complete=subject,
            view_suitable=view, reason='图像边缘与视角证据（离线夹具）',
            next_adjustment=adjustment, stop_reason=stop, edges=edges or edge_evidence()), 'review-'+capture_id)
    return call


class PhotoStateTests(unittest.TestCase):
    def test_cut_or_uncertain_edge_overrides_claimed_complete_without_failing_flight(self):
        for edge_status, effective in [('cut','no'), ('uncertain','uncertain')]:
            task = PhotoTask(PROMPT); self.observe(task, 'one')
            edges = edge_evidence(); edges['bottom']['status'] = edge_status
            result = task.review('one', 'yes', 'yes', '模型称已拍全', edges=edges)
            self.assertEqual(result['photo_status'], 'adjust')
            record = result['latest_observation']['review']
            self.assertEqual(record['subject_complete'], effective)
            self.assertEqual(record['claimed_subject_complete'], 'yes')
            self.assertTrue(record['evidence_conflict'])
            self.assertTrue(record['next_adjustment'])

    def test_clear_edges_do_not_promote_unknown_extent_and_bad_edges_do_not_mutate(self):
        task = PhotoTask(PROMPT); self.observe(task, 'one')
        bad = [None, {}, {**edge_evidence(), 'extra': {}},
               {**edge_evidence(), 'right': {'status':'clear','evidence':'  '}}]
        for value in bad:
            with self.assertRaises(ValueError):
                task.review('one', 'yes', 'yes', 'claims', edges=value)
            self.assertEqual(task.status, 'awaiting_review')
            self.assertIsNone(task.observations[-1]['review'])
        result = task.review('one', 'uncertain', 'yes', '范围未知', '换位', edges=edge_evidence())
        self.assertEqual(result['photo_status'], 'adjust')
        self.assertEqual(result['latest_observation']['review']['subject_complete'], 'uncertain')

    def test_conflict_at_final_view_ends_with_budget_without_false_met(self):
        task = PhotoTask(PROMPT)
        for i in range(4):
            self.observe(task, str(i))
            task.review(str(i), 'yes', 'yes', 'incorrect yes', edges=edge_evidence('cut'))
        self.assertEqual(task.status, 'budget_exhausted')
        self.assertEqual(len(task.observations), 4)

    def observe(self, task, capture_id):
        task.captured(capture_id)
        task.observe({'capture_id': capture_id, 'camera': {'image_sha256': capture_id}},
                     {'status': 'ok', 'capture_id': capture_id, 'summary': 'partial'})

    def test_narrow_scope_and_history(self):
        for query in (PROMPT, '拍摄厂区全貌', 'Take a photo of the entire orchard'):
            self.assertTrue(is_overview_request(query))
        for query in ('起飞到2米拍照然后降落', '描述当前场景', '上一张俯瞰果园照片为什么没拍全', '只根据历史照片说明整个区域'):
            self.assertFalse(is_overview_request(query))

    def test_unknown_boundary_and_wrong_angle_cannot_meet_goal(self):
        for subject, view in (('uncertain', 'yes'), ('yes', 'no'), ('no', 'no')):
            task = PhotoTask(PROMPT); self.observe(task, 'one')
            result = task.review('one', subject, view, '裁切/未知', '调整取景', edges=edge_evidence())
            self.assertEqual(result['photo_status'], 'adjust')
            self.assertIsNone(result['next_tool'])
            self.assertEqual(result['adjustment_tools'], ['adjust_photo_view', 'rotate_photo_view'])
            self.assertTrue(task.unfinished)

    def test_budget_dedup_and_old_capture(self):
        task = PhotoTask(PROMPT)
        for i in range(4):
            self.observe(task, str(i))
            with self.assertRaises(ValueError): self.observe(task, str(i))
            with self.assertRaises(ValueError): task.review('old', 'yes', 'yes', 'fake')
            task.review(str(i), 'uncertain', 'yes', '边界未知', '后退', edges=edge_evidence())
        self.assertEqual(task.status, 'budget_exhausted')
        with self.assertRaises(ValueError): task.guard_tool('capture_scene', {})
        with self.assertRaises(ValueError): task.guard_tool('execute_mission', {'steps': [{'action': 'goto'}]})
        for name, args in [('land_drone', {}), ('stop_drone', {}),
                           ('execute_mission', {'steps': [{'action': 'return_home'}, {'action': 'land'}]})]:
            task.guard_tool(name, args)

    def test_no_fake_met_or_new_search_bypass(self):
        task = PhotoTask(PROMPT); self.observe(task, 'one')
        for args in [('yes', 'yes', '', '', ''), ('yes', 'yes', 'visible', 'move', ''),
                     ('no', 'yes', 'cut', '', ''), (True, 'yes', 'visible', '', '')]:
            with self.assertRaises(ValueError): task.review('one', *args)
        with self.assertRaises(ValueError): task.guard_tool('begin_target_search', {})
        with self.assertRaises(ValueError): task.guard_tool('execute_mission', {'steps': [{'action': 'goto'}]})

    def test_cannot_skip_report_or_restart_after_successful_landing(self):
        task = PhotoTask(PROMPT)
        task.captured('one')
        with self.assertRaises(ValueError): task.captured('two')
        with self.assertRaises(ValueError): task.guard_tool('begin_target_search', {})
        with self.assertRaises(ValueError): task.review('one', 'yes', 'yes', 'no saved report')
        task.observe({'capture_id': 'one', 'camera': {}}, {'capture_id': 'one', 'status': 'ok', 'summary': 'saved'})
        task.review('one', 'yes', 'yes', 'supported', edges=edge_evidence())
        task.end('landed')
        self.assertEqual(task.status, 'met')
        with self.assertRaises(ValueError): task.captured('two')
        with self.assertRaises(ValueError): task.guard_tool('execute_mission', {'steps': [{'action': 'takeoff'}]})


class PhotoGraphTests(unittest.TestCase):
    def test_review_uses_only_original_request_current_image_and_no_budget_or_previous_claims(self):
        graph, model, store, client = self.build([capture_request, report_request, review(), AIMessage('完成')])
        self.invoke(graph)
        request = model.calls[2]
        self.assertEqual(len(request), 2)
        self.assertEqual(model.bindings[2]['names'], ['review_photo'])
        metadata = json.loads(request[-1].content[0]['text'])
        self.assertEqual(metadata['user_request'], PROMPT)
        self.assertEqual(metadata['capture_id'], store.photo_tasks[('photos','photo')].observations[0]['capture_id'])
        self.assertEqual(set(metadata), {'user_request', 'camera', 'capture_id'})
        self.assertEqual(image_urls(request), image_urls(model.calls[1]))
        self.assertFalse(any(isinstance(m, AIMessage) for m in request))
        trace = next(t for t in store.calls(('photos','photo')) if t.get('image_purpose') == 'photo_framing_review')
        self.assertFalse(trace['context_memory']['review_budget_included'])
        self.assertFalse(trace['context_memory']['review_history_included'])

    def build(self, script, client=None):
        client = client or PhotoFlightClient()
        model = LocalScriptModel(script=list(script))
        store = FrameStore(client, CONFIG)
        graph = build_single_vlm_agent(client, CONFIG, flight_tools(client), '', model=model, store=store)
        return graph, model, store, client

    def invoke(self, graph, prompt=PROMPT, turn='photo'):
        return graph.invoke({'messages': [HumanMessage(prompt, id=turn)]},
            {'configurable': {'thread_id': 'photos'}, 'recursion_limit': 60, 'max_concurrency': 1})

    def test_partial_review_adjustment_new_image_then_met(self):
        move = tool_call('adjust_photo_view', {'direction': 'backward', 'distance_m': 2., 'altitude': 8.,
                                             'reason': '后退取景'}, 'move')
        graph, model, store, client = self.build([capture_request, report_request,
            review('no', 'yes', '后退以纳入边界'), move, capture_request, report_request, review(), AIMessage('完成')],
            client=PhotoFlightClient(armed=True))
        result = self.invoke(graph)
        task = store.photo_tasks[('photos', 'photo')]
        self.assertEqual(task.status, 'met')
        self.assertEqual(len(task.observations), 2)
        self.assertEqual(client.calls, ['/navigation/photo-action'])
        self.assertEqual(client.position, [-2., 0., 8.])
        self.assertEqual(client.camera_reads, ['left', 'left'])
        self.assertNotEqual(*[o['capture_id'] for o in task.observations])
        self.assertEqual([len(image_urls(c)) for c in model.calls], [0, 1, 1, 1, 0, 1, 1, 0])
        self.assertEqual(image_urls(model.calls[3]), image_urls(model.calls[1]))
        metadata = json.loads(model.calls[3][-1].content[0]['text'].split('元数据：', 1)[1])
        self.assertEqual(metadata['capture_id'], task.observations[0]['capture_id'])
        self.assertIs(metadata['is_live_image'], False)
        self.assertIs(metadata['counts_as_new_observation'], False)
        self.assertEqual(metadata['bearing_geometry']['status'], 'unavailable')
        traces = [t for t in store.calls(('photos','photo')) if t.get('image_purpose') == 'photo_view_selection']
        self.assertEqual(len(traces), 1)
        self.assertEqual(traces[0]['capture_ids'], [task.observations[0]['capture_id']])
        self.assertEqual(len(task.capture_ids), 2)
        self.assertNotIn(JPEG_B64, str(result['messages']))
        for i in (2, 6):
            self.assertIn('review_photo', model.bindings[i]['names'])
            self.assertNotIn('execute_mission', model.bindings[i]['names'])
        self.assertEqual(result['messages'][-1].content, '完成')
        self.assertNotIn(JPEG_B64, str(store.photo_tasks[('photos', 'photo')].summary()))

    def test_photo_tool_reply_omits_call_audit_but_saved_report_keeps_it(self):
        graph,model,store,client=self.build([capture_request,report_request,review(),AIMessage('完成')])
        result=self.invoke(graph)
        model_report=tool_payload(result['messages'],'report_scene')
        saved=next(value for name,value in client.records if name=='inspect_scene' and value.get('status')=='ok')
        self.assertTrue(saved['vlm_calls'])
        self.assertNotIn('vlm_calls',model_report)
        for key in ('capture_id','camera','summary','targets','limitations','photo_bearing_feedback'):
            self.assertEqual(model_report[key],saved[key])
        self.assertEqual(store.photo_tasks[('photos','photo')].status,'met')

    def test_planning_image_corruption_epoch_expiry_and_other_owner_block_before_new_model_call(self):
        for mode in ('raw', 'hash', 'epoch', 'expired', 'owner'):
            with self.subTest(mode=mode):
                def corrupt(messages):
                    answer = review('no', 'yes', '换位')(messages)
                    frame = next(iter(store.frames.values()))
                    if mode == 'raw': frame['image_base64'] = 'YQ=='
                    if mode == 'hash': frame['camera']['image_sha256'] = 'bad'
                    if mode == 'epoch': frame['snapshot']['coordinate_epoch'] = {'vio_epoch': 99}
                    if mode == 'expired': frame['created_monotonic'] = -1e9
                    if mode == 'owner': frame['owner'] = ('different', 'command')
                    return answer
                graph, model, store, client = self.build([capture_request, report_request, corrupt],
                                                       client=PhotoFlightClient(armed=True))
                with self.assertRaises(RuntimeError): self.invoke(graph)
                self.assertEqual(len(model.calls), 3)
                self.assertEqual(client.photo_paths, [])
                self.assertEqual(len(client.camera_reads), 1)

    def test_bearing_feedback_is_from_saved_report_and_only_planning_metadata(self):
        from test_photo_framing import inputs
        client=PhotoFlightClient(armed=True)
        original=client.capture_camera
        def capture(side='left'):
            snapshot=original(side); frame,_=inputs()
            pose=frame['snapshot']['body_pose_enu']
            pose['stamp_sim_s']=snapshot['stamp_sim_s']
            snapshot.update(camera_intrinsics=frame['snapshot']['camera_intrinsics'],body_pose_enu=pose)
            return snapshot
        client.capture_camera=capture
        move=tool_call('rotate_photo_view',dict(direction='right',angle_deg=15.,reason='fixture'))
        graph,model,store,_=self.build([capture_request,report_request,review('no','yes','调整'),
            move,capture_request,report_request,review(),AIMessage('完成')],client=client)
        self.invoke(graph)
        metadata=json.loads(model.calls[3][-1].content[0]['text'].split('元数据：',1)[1])
        feedback=metadata['bearing_geometry']
        self.assertEqual(feedback['status'],'ok',feedback)
        # Review requests deliberately contain no prior report or geometric advice.
        self.assertFalse(any(getattr(m,'name',None)=='report_scene' for m in model.calls[2]))
        review_metadata=json.loads(model.calls[2][-1].content[0]['text'])
        self.assertNotIn('bearing_geometry',review_metadata)
        saved=next(v for name,v in client.records if name=='inspect_scene' and v.get('photo_bearing_feedback'))
        self.assertEqual(feedback,saved['photo_bearing_feedback'])
        self.assertEqual(feedback['capture_id'],metadata['capture_id'])
        self.assertFalse(feedback['usable_for_flight_authorization'])
        self.assertFalse(feedback['predicts_whole_subject_complete'])
        self.assertEqual(client.calls,['/navigation/photo-action'])
        self.assertEqual(len(store.photo_tasks[('photos','photo')].observations),2)

    def test_premature_finish_corrects_without_repeating_photo(self):
        graph, model, store, client = self.build([capture_request, report_request, AIMessage('拍全了'), review(), AIMessage('完成')])
        self.invoke(graph)
        self.assertEqual(store.photo_tasks[('photos', 'photo')].corrections, 1)
        self.assertEqual(len(client.camera_reads), 1)
        self.assertEqual(client.calls, [])
        self.assertTrue(any(name == 'photo_protocol_correction' for name, _ in client.records))

    def test_repeated_false_completion_is_bounded(self):
        graph, model, _, client = self.build([capture_request, report_request, *[AIMessage('完成') for _ in range(3)]])
        with self.assertRaisesRegex(RuntimeError, '提前结束'):
            self.invoke(graph)
        self.assertEqual(len(model.calls), 5)
        self.assertEqual(len(client.camera_reads), 1)

    def test_review_cannot_read_other_turn_or_unreported_photo(self):
        def fake_review(messages):
            return tool_call('review_photo', dict(capture_id='other-turn', subject_complete='yes',
                view_suitable='yes', reason='fake', edges=edge_evidence()))
        graph, _, _, client = self.build([capture_request, report_request, fake_review])
        with self.assertRaises(RuntimeError): self.invoke(graph)
        self.assertFalse(any(name == 'photo_progress' and value['photo_status'] == 'met'
                             for name, value in client.records))

    def test_limited_cannot_claim_success_and_no_mandatory_extra_views(self):
        graph, _, store, client = self.build([capture_request, report_request,
            review('uncertain', 'no', stop='当前固定前向相机无法垂直向下'), AIMessage('整个果园拍全了')])
        result = self.invoke(graph)
        self.assertIn('未完成', result['messages'][-1].content)
        self.assertNotIn('拍全了', result['messages'][-1].content)
        self.assertEqual(len(client.camera_reads), 1)
        self.assertEqual(store.photo_tasks[('photos', 'photo')].status, 'limited')

    def test_landing_before_framing_complete_ends_activity(self):
        land = tool_call('land_drone', {'reason': '结束'}, 'land')
        graph, _, store, client = self.build([capture_request, report_request, land, AIMessage('结束')])
        self.invoke(graph)
        self.assertEqual(client.calls, ['/land'])
        self.assertEqual(store.photo_tasks[('photos', 'photo')].status, 'limited')
        with self.assertRaises(ValueError): store.photo_tasks[('photos', 'photo')].guard_tool('capture_scene', {})

    def test_previous_turn_does_not_force_new_observation(self):
        graph, _, store, client = self.build([capture_request, report_request, review(), AIMessage('完成'), AIMessage('当时报告称')])
        self.invoke(graph)
        self.invoke(graph, '只根据上一张照片说明为什么没拍全', turn='history')
        self.assertNotIn(('photos', 'history'), store.photo_tasks)
        self.assertEqual(len(client.camera_reads), 1)


class PhotoApplicationTests(unittest.TestCase):
    def graph_app(self, script):
        bridge = PhotoFlightClient()
        model = LocalScriptModel(script=list(script))
        app = WebApplication(bridge, CONFIG,
            lambda client, config: build_single_vlm_agent(client, config, flight_tools(client), '', model=model),
            vision_config=CONFIG, agent_architecture='single_vlm')
        return app, bridge, model

    def test_unmet_after_owned_takeoff_preserves_photo_and_returns_home(self):
        takeoff = tool_call('execute_mission', {'steps': [{'action': 'takeoff', 'altitude': 8.}],
                                               'description': '取景起飞'})
        app, bridge, _ = self.graph_app([takeoff, capture_request, report_request,
            review('uncertain', 'no', stop='相机视角无法满足'), AIMessage('成功')])
        result = completed(app, app.submit(PROMPT)['id'])
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(result['photo_progress']['photo_status'], 'limited')
        self.assertEqual(bridge.calls, ['/mission', '/navigation/photo-action'])
        self.assertFalse(bridge.armed)
        self.assertEqual(result['photo_return']['status'], 'ok')
        self.assertIsNone(result['safety_landing'])
        self.assertEqual(bridge.mission['steps'], [{'action': 'return_home'}, {'action': 'land'}])
        self.assertEqual(result['perception']['status'], 'ok')

    def test_interruption_during_review_never_resumes(self):
        takeoff = tool_call('execute_mission', {'steps': [{'action': 'takeoff', 'altitude': 8.}],
                                               'description': '取景起飞'})
        def interrupt(messages):
            app.interrupt('/land')
            return review()(messages)
        app, bridge, _ = self.graph_app([takeoff, capture_request, report_request, interrupt])
        result = completed(app, app.submit(PROMPT)['id'])
        self.assertEqual(result['status'], 'failed')
        self.assertIn('中断', result['reply'])
        self.assertEqual(bridge.calls, ['/mission', '/land'])
        self.assertFalse(bridge.armed)
        self.assertNotEqual(result['photo_progress']['photo_status'], 'met')

    def test_four_views_end_without_fifth_capture_or_success_claim(self):
        script = []
        for _ in range(4):
            script.extend([capture_request, report_request, review('uncertain', 'yes', '取景仍不完整')])
        app, bridge, _ = self.graph_app(script + [AIMessage('拍全了')])
        result = completed(app, app.submit(PROMPT)['id'])
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(result['photo_progress']['photo_status'], 'budget_exhausted')
        self.assertEqual(len(bridge.camera_reads), 4)
        self.assertEqual(bridge.calls, [])  # unowned ground observation cannot trigger flight cleanup
        self.assertNotIn('拍全了', result['reply'])

    def test_partial_photo_cannot_be_green_from_final_prose(self):
        def script(client, payload, config):
            client.record('photo_progress', {'status': 'ok', 'photo_status': 'limited', 'views_used': 1,
                'latest_observation': {'review': {'stop_reason': '固定视角限制'}}})
            return {'messages': [payload['messages'][0], {'role': 'assistant', 'content': '全部完成'}]}
        app = application(CameraClient(), script)
        result = completed(app, app.submit(PROMPT)['id'])
        self.assertEqual(result['status'], 'failed')
        self.assertIn('构图目标未完成', result['reply'])
        self.assertIn('固定视角限制', result['reply'])
        self.assertNotIn('全部完成', result['reply'])


if __name__ == '__main__':
    unittest.main()
