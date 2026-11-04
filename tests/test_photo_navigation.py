"""Current-pose transforms, executor gates and trusted return-home completion."""
import copy
import math
import unittest
from unittest import mock

from photo_navigation import camera_directions, relative_photo_goal, relative_photo_yaw, require_photo_navigation
from mission import MissionError, PathUnavailable
from test_photo_task import PhotoFlightClient, PROMPT, review
from test_single_vlm import capture_request, report_request, tool_call
from test_web_server import completed
import test_bridge_navigation as nav_fixture


def yaw_quaternion(yaw):
    return [0., 0., math.sin(yaw/2), math.cos(yaw/2)]


class DirectionTests(unittest.TestCase):
    def test_turn_sign_wrap_and_camera_mount(self):
        for source, direction, degrees, expected in ((0, 'left', 90, 90),
                (0, 'right', 45, -45), (170, 'left', 30, -160), (-170, 'right', 30, 160)):
            yaw, detail = relative_photo_yaw(yaw_quaternion(math.radians(source)), direction, degrees)
            self.assertAlmostEqual(math.degrees(yaw), expected)
            self.assertLess(detail['optical_axis_enu'][2], 0)
        for value in (True, float('nan'), float('inf'), 0, 4.9, 90.1):
            with self.assertRaises(MissionError): relative_photo_yaw(yaw_quaternion(0), 'left', value)
        with self.assertRaises(MissionError): relative_photo_yaw(yaw_quaternion(0), 'up', 30)

    def test_boolean_distance_is_rejected_before_flight_tool_coercion(self):
        from test_photo_task import PhotoGraphTests
        client = PhotoFlightClient(armed=True)
        helper = PhotoGraphTests()
        graph, _, _, _ = helper.build([capture_request, report_request, review('no','yes','后退'),
            tool_call('adjust_photo_view', {'direction':'backward','distance_m':True,'altitude':10.,'reason':'invalid'})], client=client)
        with self.assertRaises(RuntimeError): helper.invoke(graph)
        self.assertEqual(client.calls, [])

    def test_backward_is_negative_x_at_zero_yaw_and_negative_y_at_quarter_turn(self):
        for yaw, expected in ((0., [-5., 0., 12.]), (math.pi/2, [0., -5., 12.]),
                              (math.pi, [5., 0., 12.]), (-math.pi, [5., 0., 12.])):
            goal, _ = relative_photo_goal([0., 0., 8.], yaw_quaternion(yaw), 'backward', 5., 12.)
            for actual, target in zip(goal, expected): self.assertAlmostEqual(actual, target)

    def test_horizontal_translation_uses_camera_frame_and_absolute_height(self):
        for direction, expected in [('left', [2., 7., 15.]), ('right', [2., -1., 15.]),
                                    ('forward', [6., 3., 15.]), ('vertical', [2., 3., 15.])]:
            goal, axes = relative_photo_goal([2., 3., 8.], yaw_quaternion(0), direction,
                                            0 if direction == 'vertical' else 4., 15.)
            self.assertEqual(goal, expected)
            self.assertLess(axes['optical_axis_enu'][2], 0)

    def test_invalid_geometry_and_numbers_never_generate_goal(self):
        for distance, height in [(True, 8), (float('nan'), 8), (11., 8), (2., float('inf')), (2., 20.1)]:
            with self.assertRaises(ValueError):
                relative_photo_goal([0,0,8], yaw_quaternion(0), 'backward', distance, height)
        with self.assertRaises(ValueError): camera_directions([0., 0., 0., 0.])
        with self.assertRaises(ValueError): relative_photo_goal([0,0,8], yaw_quaternion(0), 'vertical', 2, 10)

    def test_reference_and_health_are_required_even_for_return(self):
        client = PhotoFlightClient(armed=True); state = client.state()
        require_photo_navigation(state, state['mission']['id'], client.epoch)
        mutations = [dict(ready=False), dict(mode='AUTO.LAND'), dict(armed=False),
                     dict(mission={'id':'other','status':'succeeded'}), dict(mission={'id':state['mission']['id'],'status':'running'}),
                     dict(external_vision={}), dict(navigation={})]
        for changed in mutations:
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                require_photo_navigation({**state, **changed}, state['mission']['id'], client.epoch)
        with self.assertRaises(ValueError): require_photo_navigation(state, state['mission']['id'], None)
        changed = copy.deepcopy(client.epoch); changed['ekf_counters']['heading'] += 1
        with self.assertRaises(ValueError): require_photo_navigation(state, state['mission']['id'], changed)


class ExecutorTests(unittest.TestCase):
    def setUp(self):
        fixture = nav_fixture.BridgeNavigationTests('test_pose_callback_preserves_full_orientation_for_scan_alignment')
        fixture.setUp(); self.addCleanup(fixture.doCleanups)
        self.fixture = fixture; self.c = fixture.controller
        fixture.set_pose([2.,3.,8.], yaw_quaternion(math.pi/2))
        self.c.home = [1.,1.,0.]
        self.client = PhotoFlightClient(armed=True)
        self.state = self.client.state(); self.state['position_enu_m'] = [2.,3.,8.]
        self.c.snapshot = mock.Mock(return_value=self.state)
        self.c.search_authority = mock.Mock()
        self.c.search_authority.return_value.constraint.return_value = None
        self.c.preview_navigation = mock.Mock(return_value={'reachable':True})
        self.c.submit = mock.Mock(return_value={'mission':{'id':'new','status':'running'}})

    def action(self, operation='adjust', **changes):
        values = dict(operation=operation, expected_mission_id=self.state['mission']['id'],
                      expected_epoch=self.client.epoch, description='fixture')
        if operation == 'adjust': values.update(direction='backward', distance_m=5., altitude=12.)
        values.update(changes)
        return self.c.photo_action(**values)

    def test_current_attitude_converts_and_previews_before_real_submit(self):
        result = self.action()
        goal = result['photo_navigation']['goal_enu_m']
        self.assertAlmostEqual(goal[0], 2.); self.assertAlmostEqual(goal[1], -2.)
        self.assertEqual(goal[2], 12.)
        self.c.preview_navigation.assert_called_once_with(goal)
        self.assertEqual(self.c.submit.call_args.kwargs['parent_mission_id'], self.state['mission']['id'])

    def test_return_uses_true_home_and_existing_return_land_actions(self):
        self.action('return_home')
        self.c.preview_navigation.assert_called_once_with([1.,1.,8.])
        self.assertEqual(self.c.submit.call_args.args[0], [{'action':'return_home'}, {'action':'land'}])

    def test_turn_uses_fresh_current_heading_and_existing_rotation_executor(self):
        result = self.action('turn', direction='right', angle_deg=45.)
        self.c.preview_navigation.assert_called_once_with([2.,3.,8.])
        steps = self.c.submit.call_args.args[0]
        self.assertEqual(len(steps), 1)
        self.assertEqual(steps[0]['action'], 'rotate')
        self.assertAlmostEqual(steps[0]['yaw_rad'], math.pi/4)
        self.assertEqual(result['photo_navigation']['operation'], 'turn')
        self.assertEqual(self.c.submit.call_args.kwargs['parent_mission_id'], self.state['mission']['id'])

    def test_turn_cannot_mix_translation_or_recover_invalid_anchor(self):
        with self.assertRaises(MissionError): self.action('turn', direction='left', angle_deg=30., altitude=8.)
        self.c.preview_navigation.return_value = {'reachable':False, 'rejection_kind':'candidate_geometry', 'reason':'invalid anchor'}
        with self.assertRaises(MissionError): self.action('turn', direction='left', angle_deg=30.)
        self.c.submit.assert_not_called()

    def test_stale_future_reset_other_owner_and_active_search_cannot_submit(self):
        for change in ('wall_stale', 'future', 'reset', 'other_owner', 'search'):
            with self.subTest(change=change):
                self.c.submit.reset_mock(); self.c.pose_time = 1000.
                self.fixture.set_pose([2.,3.,8.])
                self.c.snapshot.return_value = copy.deepcopy(self.state)
                self.c.search_authority.return_value.constraint.return_value = None
                if change == 'wall_stale': self.c.pose_time = 999.
                if change == 'future': self.fixture.set_pose([2.,3.,8.], stamp=20.)
                if change == 'reset': self.c.snapshot.return_value['external_vision']['adapter']['epoch'] += 1
                if change == 'other_owner': self.c.snapshot.return_value['mission']['id'] = 'foreign'
                if change == 'search': self.c.search_authority.return_value.constraint.return_value = {'owner':['other','search']}
                with self.assertRaises(ValueError): self.action()
                with self.assertRaises(ValueError): self.action('turn', direction='left', angle_deg=30.)
                self.c.submit.assert_not_called()

    def test_path_refusal_is_not_submitted_and_does_not_weaken_navigation(self):
        self.c.preview_navigation.return_value = {'reachable':False, 'reason':'blocked'}
        for operation in ('adjust','return_home'):
            with self.assertRaisesRegex(ValueError, 'path unavailable'): self.action(operation)
        self.c.submit.assert_not_called()

    def test_only_typed_candidate_rejection_is_recoverable(self):
        self.c.preview_navigation.return_value = {'reachable': False, 'reason': 'outside region',
                                                  'rejection_kind': 'candidate_geometry'}
        result = self.action()
        self.assertEqual(result['status'], 'not_executed')
        self.assertIs(result['mission_submitted'], False)
        self.assertNotIn('mission', result)
        self.assertEqual(result['photo_navigation']['goal_enu_m'][2], 12.)
        with self.assertRaises(MissionError): self.action('return_home')
        self.c.submit.assert_not_called()

    def test_scene_invalid_start_is_not_a_candidate_rejection(self):
        from scene_config import validate_scene_trajectory
        scene = {'verified': True, 'flight_region': {'bounds_min_m': [-1,-1,0],
                 'bounds_max_m': [5,5,20], 'margin_m': .1}}
        for start in ([0,0,8], [-3,0,8]):
            try:
                validate_scene_trajectory(scene, [{'action':'goto','x':-4,'y':0,'altitude':8}], start,
                    [0,0,0], {'position_m':start,'orientation_xyzw':[0,0,0,1]}, [0,0,0,1])
            except MissionError as exc:
                self.assertEqual(isinstance(exc, PathUnavailable), start[0] == 0)
            else:
                self.fail('out of bounds not rejected')


class CandidateRecoveryTests(unittest.TestCase):
    def test_owned_turn_followed_by_new_photo_and_same_budget(self):
        from test_photo_task import PhotoApplicationTests
        from langchain_core.messages import AIMessage
        takeoff = tool_call('execute_mission', {'steps':[{'action':'takeoff','altitude':8.}], 'description':'fixture'})
        app, bridge, model = PhotoApplicationTests().graph_app([takeoff, capture_request, report_request,
            review('no', 'yes', '右转观察边界'),
            tool_call('rotate_photo_view', {'direction':'right','angle_deg':45.,'reason':'构图调整'}),
            capture_request, report_request, review(), AIMessage('完成')])
        result = completed(app, app.submit(PROMPT)['id'])
        self.assertEqual(result['status'], 'succeeded')
        self.assertEqual(result['photo_progress']['views_used'], 2)
        self.assertEqual(result['photo_progress']['max_views'], 4)
        self.assertEqual(bridge.photo_paths[0]['operation'], 'turn')
        self.assertEqual(bridge.mission['steps'][0]['action'], 'rotate')
        self.assertFalse(result.get('flight_failure'))
        self.assertIn('rotate_photo_view', model.bindings[4]['names'])
        self.assertNotIn('rotate_photo_view', model.bindings[5]['names'])

    def test_turn_requires_review_and_rejects_boolean_angle(self):
        from test_photo_task import PhotoGraphTests
        helper = PhotoGraphTests()
        for preceding, angle in (([], 45.), ([capture_request, report_request, review('no','yes','转向')], True)):
            client = PhotoFlightClient(armed=True)
            graph, _, _, _ = helper.build(preceding + [tool_call('rotate_photo_view',
                {'direction':'left','angle_deg':angle,'reason':'invalid'})], client=client)
            with self.assertRaises(RuntimeError): helper.invoke(graph)
            self.assertEqual(client.photo_paths, [])

    def run_case(self, rejection_count=1, corrupt=None, flight_failure=False):
        from test_photo_task import PhotoApplicationTests
        from langchain_core.messages import AIMessage
        takeoff = tool_call('execute_mission', {'steps':[{'action':'takeoff','altitude':8.}], 'description':'fixture'})
        script = [takeoff, capture_request, report_request, review('no','yes','调整')]
        script += [tool_call('adjust_photo_view', {'direction':'backward', 'distance_m':5.-i,
                    'altitude':12., 'reason':'frame'}, 'reject-'+str(i)) for i in range(rejection_count)]
        if rejection_count < 3:
            script += [tool_call('adjust_photo_view', {'direction':'vertical','distance_m':0.,'altitude':12.,'reason':'升高'}),
                       capture_request, report_request, review(), AIMessage('完成')]
        app, bridge, _ = PhotoApplicationTests().graph_app(script)
        original = bridge.request
        rejected = []
        def request(path, payload=None):
            if path == '/navigation/photo-action' and payload['operation'] == 'adjust' and len(rejected) < rejection_count:
                result = dict(status='not_executed', mission_submitted=False, reason_code='photo_path_unreachable',
                    operation='adjust', expected_mission_id=payload['expected_mission_id'],
                    expected_epoch=copy.deepcopy(payload['expected_epoch']), reason='outside permitted region')
                if corrupt: corrupt(result, bridge)
                rejected.append(result)
                return result
            result = original(path, payload)
            if flight_failure and path == '/navigation/photo-action' and payload['operation'] == 'adjust':
                bridge.mission.update(status='failed', error='actual execution failed')
                result['mission'] = copy.deepcopy(bridge.mission)
            return result
        bridge.request = request
        result = completed(app, app.submit(PROMPT)['id'])
        result['flight_failure'] = app.commands[result['id']]['flight_failure']
        return result, bridge

    def test_rejected_candidate_can_select_alternative_without_fake_flight(self):
        result, bridge = self.run_case()
        self.assertEqual(result['status'], 'succeeded')
        self.assertEqual(len(result['flight_missions']), 2)
        self.assertIsNone(result['flight_failure'])
        self.assertEqual(result['photo_progress']['plan_rejections_used'], 1)
        self.assertEqual(len(bridge.camera_reads), 2)

    def test_shared_rejection_budget_returns_home_without_claiming_goal_met(self):
        result, bridge = self.run_case(3)
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(result['photo_progress']['photo_status'], 'limited')
        self.assertEqual(result['photo_progress']['plan_rejections_used'], 3)
        self.assertEqual(len(result['flight_missions']), 2)  # takeoff + actual return
        self.assertIsNone(result['flight_failure'])
        self.assertEqual(result['photo_return']['status'], 'ok')
        self.assertFalse(bridge.armed)

    def test_forged_rejection_or_changed_state_is_fatal(self):
        for corrupt in (lambda r,b:r.update(mission={'id':'fake'}),
                        lambda r,b:r.update(expected_mission_id='other'),
                        lambda r,b:r.update(mission_submitted=True),
                        lambda r,b:setattr(b,'healthy',False),
                        lambda r,b:b.epoch.update(vio_epoch=99)):
            with self.subTest(corrupt=corrupt):
                result, bridge = self.run_case(corrupt=corrupt)
                self.assertEqual(result['status'], 'failed')
                self.assertTrue(result['flight_failure'])
                self.assertNotIn('photo_return', result)
                self.assertFalse(bridge.armed)

    def test_accepted_flight_failure_is_not_a_candidate_retry(self):
        result, bridge = self.run_case(flight_failure=True)
        self.assertEqual(result['status'], 'failed')
        self.assertTrue(result['flight_failure'])
        self.assertNotIn('photo_return', result)
        self.assertEqual(len(bridge.camera_reads), 1)
        self.assertFalse(bridge.armed)


class ReturnPolicyTests(unittest.TestCase):
    def run_case(self, action):
        from test_photo_task import PhotoApplicationTests
        helper = PhotoApplicationTests()
        takeoff = tool_call('execute_mission', {'steps':[{'action':'takeoff','altitude':8.}], 'description':'fixture'})
        def last_review(messages):
            action(app, bridge)
            return review('no','yes',stop='边界无法完整入镜')(messages)
        app, bridge, model = helper.graph_app([takeoff,capture_request,report_request,last_review])
        result = completed(app, app.submit(PROMPT)['id'])
        return result, bridge

    def test_unhealthy_pose_or_changed_epoch_falls_back_to_owned_land(self):
        for mutate in (lambda a,b:setattr(b,'healthy',False),
                       lambda a,b:b.epoch.update(vio_epoch=9)):
            result, bridge = self.run_case(mutate)
            self.assertEqual(result['status'],'failed')
            self.assertEqual(bridge.calls, ['/mission','/land'])
            self.assertEqual(result['safety_landing']['status'],'ok')
            self.assertNotEqual(result.get('photo_return',{}).get('status'),'ok')

    def test_blocked_return_falls_back_without_retrying_return(self):
        def block(app, bridge):
            original = bridge.request
            def request(path, payload=None):
                if path == '/navigation/photo-action':
                    bridge.calls.append(path)
                    raise RuntimeError('path blocked')
                return original(path,payload)
            bridge.request = request
        result, bridge = self.run_case(block)
        self.assertEqual(bridge.calls, ['/mission','/navigation/photo-action','/land'])
        self.assertEqual(result['safety_landing']['status'],'ok')

    def test_replaced_mission_is_neither_returned_nor_landed(self):
        result, bridge = self.run_case(lambda a,b:b.mission.update(id='foreign-flight'))
        self.assertEqual(bridge.calls, ['/mission'])
        self.assertNotEqual(result.get('photo_return',{}).get('status'),'ok')
        self.assertTrue(bridge.armed)

    def test_accepted_return_failure_falls_back_on_its_new_owned_mission(self):
        def fail_return(app, bridge):
            original = bridge.wait_for
            def wait(accepted, on_progress=None):
                if accepted['mission']['steps'][0]['action'] == 'return_home':
                    bridge.mission.update(status='failed', error='return execution fault')
                    bridge.armed = True
                    return bridge.state()
                return original(accepted, on_progress)
            bridge.wait_for = wait
        result, bridge = self.run_case(fail_return)
        self.assertEqual(bridge.calls, ['/mission','/navigation/photo-action','/land'])
        self.assertEqual(result['photo_return']['status'],'error')
        self.assertEqual(result['safety_landing']['status'],'ok')
        self.assertFalse(bridge.armed)

    def test_operator_interrupt_during_return_is_not_overridden(self):
        def interrupt_return(app, bridge):
            original = bridge.wait_for
            def wait(accepted, on_progress=None):
                if accepted['mission']['steps'][0]['action'] == 'return_home':
                    app.interrupt('/land')
                    raise RuntimeError('operator interrupted return')
                return original(accepted, on_progress)
            bridge.wait_for = wait
        result, bridge = self.run_case(interrupt_return)
        self.assertEqual(bridge.calls, ['/mission','/navigation/photo-action','/land'])
        self.assertIn('中断', result['reply'])
        self.assertFalse(bridge.armed)

    def test_flight_failure_is_not_eligible_for_framing_return(self):
        def fail(app, bridge):
            app.audited_client.record('execute_mission', {'status':'error','error':'real flight fault'})
        result, bridge = self.run_case(fail)
        self.assertEqual(result['status'],'failed')
        self.assertEqual(bridge.calls, ['/mission','/land'])
        self.assertNotIn('photo_return',result)


if __name__ == '__main__': unittest.main()
