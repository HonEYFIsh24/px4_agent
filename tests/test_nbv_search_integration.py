"""Actual Agent graph with synthetic images/transport; no model, ROS or real flight."""
import copy
import json
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from langchain_core.messages import AIMessage
from agent import flight_tools, SYSTEM_PROMPT
from single_vlm import build_single_vlm_agent
from search_tools import coordinate_epoch
from web_server import WebApplication
from viewpoint_authority import ViewpointAuthority
from viewpoint_geometry import StaticViewpointEvaluator, registered_identity
from test_search_graph_integration import LocalStereoBridge
from test_semantic_map import navigator
from test_single_vlm import LocalScriptModel, tool_call
from test_vision import CONFIG


class ScopedFixtureBridge(LocalStereoBridge):
    def __init__(self, search_radius=12., grid_resolution=.5):
        super().__init__()
        self.search_radius,self.grid_resolution = search_radius,grid_resolution
        self.nav = navigator()
        self.telemetry['scene']['world_sha256'] = self.nav.world_sha256
        self.telemetry['external_vision']['ekf'].update(primary_instance=0,
            local_velocity_reset_counters={'vxy': 2, 'vz': 2})
        self.authority = ViewpointAuthority()
        self.ticket_executions, self.generic_gotos = 0, 0
        self.reject_geometry = False

    def capture_stereo(self):
        pair = super().capture_stereo()
        pair['navigation_registration_id'] = self.nav.registration_id if self.telemetry['armed'] else None
        self.pairs[pair['pair_id']]['navigation_registration_id'] = pair['navigation_registration_id']
        return pair

    def wait_for(self, accepted, on_progress=None):
        result = super().wait_for(accepted, on_progress)
        if accepted['mission']['steps'][0]['action'] == 'takeoff':
            ev = self.telemetry['external_vision']
            ev['heading_alignment_pending'] = False
            ev['ekf'].update(heading_good_for_control=True,
                             mag_alignment={'aligned_in_flight': True, 'yaw_aligned': True, 'in_air': True})
            return self.state()
        return result

    def state(self):
        state = super().state()
        state['stamp_sim_s'] = 42. + self.counter + .02
        state['navigation'] = {**self.nav.diagnostics(), 'enabled': True, 'active': state['armed'],
            'viewpoint_execution_supported': True, 'viewpoint_prediction_supported': True,
            'observation_geometry_supported': True}
        return state

    def request(self, path, payload=None, timeout=None):
        if path == '/navigation/observation-geometry':
            if self.reject_geometry: raise ValueError('controlled geometry failure')
            pair = self.pairs[payload['pair_id']]
            info = pair['left']['camera_intrinsics']
            result = self.nav.observation_visibility(pair['body_pose_enu'], info, payload['origin_xy_enu_m'],
                radius=self.search_radius,resolution=self.grid_resolution)
            result.update(capture_pair_id=pair['pair_id'], coordinate_epoch=coordinate_epoch(pair['external_vision']),
                stamp_sim_s=pair['stamp_sim_s'], image_sha256=pair['image_sha256'],
                body_pose_enu=pair['body_pose_enu'], camera_intrinsics=info)
            return result
        if path == '/navigation/viewpoints':
            pair = self.pairs[payload['pair_id']]
            epoch = coordinate_epoch(self.telemetry)
            identity = registered_identity(self.nav, epoch)
            context = payload.get('search_context')
            if context:
                if payload['operation'] == 'context':
                    self.authority.bind(context, pair['pair_id'], pair, identity, payload['origin_xy_enu_m'],
                                        search_radius_m=self.search_radius)
                else: self.authority.require_batch(context)
            pose = {**pair['body_pose_enu'], 'position_m': list(self.telemetry['position_enu_m'])}
            result = StaticViewpointEvaluator(self.nav, pose, pair['left']['camera_intrinsics'], epoch,
                fixed_yaw=0., reference_kind='live_enu_pose', probe_points_enu_m=payload.get('probe_points_enu_m'),
                search_radius_m=self.search_radius,grid_resolution_m=self.grid_resolution).query(payload['operation'], payload['origin_xy_enu_m'],
                    payload['expected_identity'], payload['candidates'])
            if context and payload['operation'] == 'paths': self.authority.issue(context, result)
            return result
        if path == '/navigation/execute-viewpoint':
            accepted = self.authority.consume(payload['owner'], payload['batch_id'], payload['map_revision'],
                payload['candidate_id'], payload['ticket'], registered_identity(self.nav, coordinate_epoch(self.telemetry)),
                self.telemetry['position_enu_m'], 0.)
            point = accepted['goal_enu_m']
            plan = self.nav.plan(self.telemetry['position_enu_m'], point,
                search_origin_xy=accepted['origin_xy_enu_m'], search_radius_m=accepted['search_radius_m'])
            self.ticket_executions += 1
            result = super().request('/mission', {'steps': [{'action': 'goto', 'x': point[0], 'y': point[1], 'altitude': point[2]}],
                                                  'description': payload['description']})
            self.telemetry['mission']['search_constraint'] = self.authority.constraint()
            self.telemetry['mission']['viewpoint_candidate_id'] = accepted['candidate_id']
            self.telemetry['mission']['navigation_plans'] = [{'goal_enu_m': point, 'plan': plan}]
            self.telemetry['mission']['events'] = [{'action': 'goto', 'position_enu_m': list(point)}]
            return {'mission': copy.deepcopy(self.telemetry['mission'])}
        if path == '/navigation/search-release':
            self.authority.clear()
            return {'status': 'ok'}
        if path == '/mission' and any(s['action'] == 'goto' for s in payload.get('steps', [])):
            self.generic_gotos += 1
            raise AssertionError('NBV movement escaped the ticket-only transport')
        result = super().request(path, payload)
        if self.authority.active and path == '/mission':
            self.telemetry['mission']['search_constraint'] = self.authority.constraint()
            result = {'mission': copy.deepcopy(self.telemetry['mission'])}
        if path == '/land' or (path == '/mission' and payload['steps'][-1]['action'] == 'land'):
            self.authority.clear()
        return result


def saved_report(messages, found=False):
    message = next(m for m in reversed(messages) if getattr(m, 'name', None) == 'observe_target_search_view')
    capture = json.loads(message.content)
    targets = ([{'label': '蓝色油桶', 'bbox': [.2, .3, .4, .6], 'confidence': .8,
                 'surroundings': [], 'position_basis': 'image_only'}] if found else [])
    return tool_call('report_scene', {'capture_id': capture['capture_id'], 'summary': 'controlled synthetic fixture',
        'targets': targets, 'limitations': ['not independent visual recognition evidence']}, 'report-'+capture['capture_id'])


def synthetic_metric_target(bridge, pair_id, bboxes):
    result = LocalStereoBridge.localize_stereo(bridge, pair_id, bboxes)
    for item in result['targets']:
        item['position_enu_m'] = [5., 0., 1.]
    return result


class NBVGraphTests(unittest.TestCase):
    def run_task(self, bridge, script, mode='nbv_geometry', selector='deterministic', pool='ranked', auto_prepare=False,
                 confirmation='single_frame', architecture='single_vlm', prompt=None, timeout=15):
        model = LocalScriptModel(script=script)
        with mock.patch.dict(os.environ, {'UAV_SEARCH_MODE': mode, 'UAV_NBV_SELECTOR': selector,
                'UAV_NBV_CANDIDATE_POOL': pool, 'UAV_NBV_AUTO_PREPARE': '1' if auto_prepare else '0',
                'UAV_TARGET_CONFIRMATION': confirmation}):
            app = WebApplication(bridge, CONFIG, lambda client, config:
                build_single_vlm_agent(client, config, flight_tools(client), SYSTEM_PROMPT, model=model),
                agent_architecture=architecture)
            self.last_app = app
            identifier = app.submit(prompt or '找到蓝色油桶，完成后返航降落。')['id']
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                result = app.command(identifier)
                if result['status'] != 'running': return result, app, model
                time.sleep(.01)
        self.fail('local NBV graph fixture did not finish')

    def prefix(self):
        return [tool_call('get_drone_state', {'view': 'all'}, 'state'),
                tool_call('begin_target_search', {'query': '蓝色油桶'}, 'begin'),
                tool_call('observe_target_search_view', {}, 'first-view'), saved_report]

    def test_staged_binding_accepts_only_owned_alignment_and_keeps_ground_boundary(self):
        from autonomous_search import AdaptiveSearch
        bridge = ScopedFixtureBridge()
        ev = bridge.telemetry['external_vision']
        ev['ekf'].update(primary_instance=0, local_velocity_reset_counters={'vxy': 2, 'vz': 2})
        search = AdaptiveSearch(bridge, search_mode='nbv_geometry')
        owner = ('thread', 'turn')
        search.begin(owner, '蓝色油桶')
        session = search.sessions[owner]
        self.assertFalse(session['epoch_bound'])
        self.assertIsNone(session['origin'])
        accepted = bridge.request('/mission', {'steps': [{'action': 'takeoff', 'altitude': 3.}], 'description': 'fixture'})
        bridge.wait_for(accepted)
        ev = bridge.telemetry['external_vision']
        ev['ekf']['local_position_reset_counters']['heading'] = 4
        ev['heading_alignment_event'] = {'source': 'actual_owned_px4_uorb', 'old_heading_reset_counter': 3,
            'new_heading_reset_counter': 4, 'adapter_epoch': 0, 'xy_reset_counter': 1, 'z_reset_counter': 2,
            'primary_instance': 0, 'velocity_reset_counters': {'vxy': 2, 'vz': 2},
            'permitted_context': 'takeoff_before_horizontal_navigation', 'reset_source_stamp_sim_s': 43.5}
        bridge.telemetry['position_enu_m'] = [.1, 0., 3.]
        search.bind_after_takeoff(owner, bridge.state())
        self.assertTrue(session['epoch_bound'])
        self.assertEqual(session['origin'], [.1, 0., 3.])
        self.assertEqual(session['boundary_origin'], [0., 0., 0.])
        self.assertAlmostEqual(session['binding']['ground_offset_m'], .1)
        with self.assertRaisesRegex(ValueError, '已经绑定'):
            search.bind_after_takeoff(owner, bridge.state())

    def test_staged_binding_rejects_invalid_evidence_without_creating_identity(self):
        from autonomous_search import AdaptiveSearch
        for case in ('unaccepted_heading', 'vio_reset', 'xy_reset', 'velocity_reset', 'primary_change',
                     'not_aligned', 'pending_alignment', 'bad_heading', 'unregistered', 'old_pose',
                     'ground_drift', 'mission_running', 'world_change', 'existing_observation', 'issued_ticket'):
            with self.subTest(case=case):
                bridge = ScopedFixtureBridge()
                search = AdaptiveSearch(bridge, search_mode='nbv_geometry')
                owner = ('thread', 'turn')
                search.begin(owner, '蓝色油桶')
                accepted = bridge.request('/mission', {'steps': [{'action': 'takeoff', 'altitude': 3.}], 'description': 'fixture'})
                bridge.wait_for(accepted)
                ev, session = bridge.telemetry['external_vision'], search.sessions[owner]
                ekf = ev['ekf']
                if case == 'unaccepted_heading': ekf['local_position_reset_counters']['heading'] += 1
                elif case == 'vio_reset': ev['adapter']['epoch'] += 1
                elif case == 'xy_reset': ekf['local_position_reset_counters']['xy'] += 1
                elif case == 'velocity_reset': ekf['local_velocity_reset_counters']['vxy'] += 1
                elif case == 'primary_change': ekf['primary_instance'] = 1
                elif case == 'not_aligned': ekf['mag_alignment']['aligned_in_flight'] = False
                elif case == 'pending_alignment': ev['heading_alignment_pending'] = True
                elif case == 'bad_heading': ekf['heading_good_for_control'] = False
                elif case == 'unregistered': bridge.nav.registration_id = None
                elif case == 'ground_drift': bridge.telemetry['position_enu_m'][0] = .5
                elif case == 'mission_running': bridge.telemetry['mission']['status'] = 'running'
                elif case == 'world_change': bridge.telemetry['scene']['world_sha256'] = 'f'*64
                elif case == 'existing_observation': session['observations'].append({})
                elif case == 'issued_ticket': session['nbv_bound'] = True
                elif case == 'old_pose':
                    original = bridge.capture_stereo
                    def capture():
                        pair = original()
                        pair['body_pose_enu']['age_seconds'] = 2.
                        return pair
                    bridge.capture_stereo = capture
                with self.assertRaises(ValueError):
                    search.bind_after_takeoff(owner, bridge.state())
                self.assertFalse(session['epoch_bound'])
                self.assertIsNone(session['origin'])
                self.assertIsNone(session['binding'])

    def test_failed_search_initialization_prevents_fallback_takeoff(self):
        bridge = ScopedFixtureBridge()
        script = [tool_call('begin_target_search', {'query': '蓝色油桶'}, 'begin'),
                  tool_call('execute_mission', {'steps': [{'action': 'takeoff', 'altitude': 3}],
                      'description': 'fallback takeoff'}, 'fallback'), AIMessage('完成')]
        with mock.patch.dict(os.environ, {'UAV_NBV_HEADING_MODE': 'discrete'}):
            result, _, _ = self.run_task(bridge, script, confirmation='multiview', architecture='single_vlm')
        self.assertEqual(result['status'], 'failed')
        self.assertIn('航向NBV', result['search_initialization_failure'])
        self.assertFalse(any(x['path'] == '/mission' for x in bridge.flight_requests))
        self.assertFalse(bridge.telemetry['armed'])

    def test_reset_during_completed_takeoff_stops_before_capture_and_lands(self):
        class ResetOnTakeoff(ScopedFixtureBridge):
            def wait_for(self, accepted, on_progress=None):
                result = super().wait_for(accepted, on_progress)
                if accepted['mission']['steps'][0]['action'] == 'takeoff':
                    self.telemetry['external_vision']['ekf']['local_position_reset_counters']['heading'] += 1
                    return self.state()
                return result
        bridge = ResetOnTakeoff()
        result, _, _ = self.run_task(bridge, self.prefix()[:3] + [AIMessage('完成')],
                                     architecture='single_vlm')
        self.assertEqual(result['status'], 'failed')
        self.assertIn('坐标系已重置', result['error'])
        self.assertEqual(len(bridge.camera_reads), 1)  # ground origin only; no airborne capture
        self.assertEqual(len([x for x in bridge.flight_requests if x['path'] == '/mission']), 1)
        self.assertEqual(len([x for x in bridge.flight_requests if x['path'] == '/land']), 1)
        self.assertEqual(result['safety_landing']['status'], 'ok')
        self.assertFalse(bridge.telemetry['armed'])

    def test_reset_after_direct_takeoff_also_fails_and_lands(self):
        class ResetOnTakeoff(ScopedFixtureBridge):
            def wait_for(self, accepted, on_progress=None):
                result = super().wait_for(accepted, on_progress)
                if accepted['mission']['steps'][0]['action'] == 'takeoff':
                    self.telemetry['external_vision']['ekf']['local_position_reset_counters']['heading'] += 1
                    return self.state()
                return result
        bridge = ResetOnTakeoff()
        script = [tool_call('begin_target_search', {'query': '蓝色油桶'}, 'begin'),
                  tool_call('execute_mission', {'steps': [{'action': 'takeoff', 'altitude': 3.}],
                            'description': 'takeoff'}, 'takeoff'), AIMessage('完成')]
        result, _, _ = self.run_task(bridge, script, architecture='single_vlm')
        self.assertEqual(result['status'], 'failed')
        self.assertIn('坐标系已重置', result['error'])
        self.assertEqual(len(bridge.camera_reads), 1)
        self.assertEqual(len([x for x in bridge.flight_requests if x['path'] == '/land']), 1)
        self.assertEqual(result['safety_landing']['status'], 'ok')
        self.assertFalse(bridge.telemetry['armed'])

    def test_invalid_report_schema_stops_graph_and_lands_once(self):
        bridge = ScopedFixtureBridge()
        def invalid(messages):
            capture = json.loads(next(m for m in reversed(messages)
                if getattr(m, 'name', None) == 'observe_target_search_view').content)
            return tool_call('report_scene', {'capture_id': capture['capture_id'], 'summary': 'nothing visible',
                'targets': '[{"label":"白色货车","bbox":[0,0,0,0]}', 'limitations': []}, 'invalid-report')
        result, _, model = self.run_task(bridge, self.prefix()[:-1]+[invalid, invalid, invalid],
                                         auto_prepare=True, confirmation='multiview', architecture='single_vlm')
        self.assertEqual(result['status'], 'failed')
        self.assertIn('report_scene', result['error'])
        self.assertEqual(len([c for c in result['model_calls'] if any(
            t.get('name') == 'report_scene' for t in c.get('called_tools', []))]), 1)
        self.assertEqual(len([x for x in bridge.flight_requests if x['path'] == '/land']), 1)
        self.assertFalse(bridge.telemetry['armed'])
        self.assertLessEqual(len(model.calls), 5)

    def test_missing_confirmation_reuses_original_source_for_one_new_view(self):
        from coverage_search import project_ground_point
        bridge = ScopedFixtureBridge()
        bridge.localize_stereo = lambda pair_id, bboxes: synthetic_metric_target(bridge, pair_id, bboxes)
        def candidate(messages):
            capture = json.loads(next(m for m in reversed(messages)
                if getattr(m, 'name', None) == 'observe_target_search_view').content)
            pair = next(reversed(bridge.pairs.values()))
            u, v = project_ground_point(pair['left']['camera_intrinsics'], pair['body_pose_enu'], [5., 0., 1.], 20.)['pixel_uv']
            return tool_call('report_scene', {'capture_id': capture['capture_id'], 'summary': 'synthetic candidate',
                'targets': [{'label': '蓝色油桶', 'bbox': [u/640-.06, v/480-.07, u/640+.06, v/480+.07],
                             'confidence': None, 'surroundings': [], 'position_basis': 'image_only'}],
                'limitations': ['synthetic fixture']}, 'report-'+capture['capture_id'])
        def review(messages, found):
            value = json.loads(next(m for m in reversed(messages) if getattr(m, 'name', None) == 'report_scene').content)
            active = value['search_progress']['confirmation']
            return tool_call('review_target_candidate', {'source_capture_id': active['source_capture_id'],
                'current_capture_id': active['current_capture_id'],
                'current_target_id': active['current_target_ids'][0] if found else '',
                'source_supported': found, 'source_uncertain': not found,
                'current_supported': found, 'current_uncertain': not found, 'same_object': found,
                'source_description': 'controlled source', 'current_description': 'controlled current',
                'reason': 'controlled review'}, 'review-'+active['current_capture_id'])
        script = self.prefix()[:-1] + [candidate, tool_call('observe_target_search_view', {}, 'second'),
            saved_report, lambda m: review(m, False),
            tool_call('execute_mission', {'steps': [{'action': 'return_home'}, {'action': 'land'}],
                'description': 'premature plan before confirmation'}, 'premature-return'),
            tool_call('observe_target_search_view', {}, 'third'),
            candidate, lambda m: review(m, True)] + self.suffix()[-2:]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR': str(root/'captures')}):
                result, _, _ = self.run_task(bridge, script, auto_prepare=True, confirmation='multiview')
            from audit_target_confirmation import audit
            record = {'command': result, 'target_confirmation_mode': 'multiview',
                      'flight': bridge.state(), 'capture_directory': str(root/'captures')}
            path = root/'result.json'; path.write_text(json.dumps(record))
            reviewed = audit(path, root/'audit')
            self.assertTrue(reviewed['passed'], reviewed)
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        calls = [c for c in result['model_calls'] if c.get('image_purpose') == 'target_confirmation']
        self.assertEqual(len(calls), 2)
        self.assertTrue(all('review_arguments' in t for c in calls for t in c['called_tools']
                            if t['name'] == 'review_target_candidate'))
        self.assertEqual(calls[0]['capture_ids'][0], calls[1]['capture_ids'][0])
        self.assertNotEqual(calls[0]['capture_ids'][1], calls[1]['capture_ids'][1])
        self.assertEqual(bridge.ticket_executions, 2)
        self.assertEqual(len(result['search_protocol_corrections']), 1)
        self.assertFalse(result['search_protocol_corrections'][0]['mission_submitted'])
        self.assertEqual(len([x for x in bridge.flight_requests if x['path'] == '/mission'
                              and x['steps'][0]['action'] == 'return_home']), 1)
        self.assertFalse(bridge.telemetry['armed'])

        # Repeated premature plans exhaust the single correction; they must
        # not trigger an unbounded loop or submit a return mission.
        bridge = ScopedFixtureBridge()
        bridge.localize_stereo = lambda pair_id, bboxes: synthetic_metric_target(bridge, pair_id, bboxes)
        repeated = script[:8] + [tool_call('execute_mission', {'steps': [
            {'action': 'return_home'}, {'action': 'land'}], 'description': 'repeated premature plan'}, 'again'),
            tool_call('land_drone', {'reason': 'safe stop'}, 'land'), AIMessage('已停止。')]
        result, _, _ = self.run_task(bridge, repeated, auto_prepare=True, confirmation='multiview', architecture='single_vlm')
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(len(result['search_protocol_corrections']), 1)
        self.assertEqual(result['flight_failure_stage'], 'search_mission_precheck')
        self.assertFalse(any(x['path'] == '/mission' and x['steps'][0]['action'] == 'return_home'
                             for x in bridge.flight_requests))
        self.assertFalse(bridge.telemetry['armed'])

        bridge = ScopedFixtureBridge()
        bridge.localize_stereo = lambda pair_id, bboxes: synthetic_metric_target(bridge, pair_id, bboxes)
        def reset_before_return(messages):
            bridge.telemetry['external_vision']['adapter']['epoch'] += 1
            return tool_call('execute_mission', {'steps': [{'action': 'return_home'}, {'action': 'land'}],
                             'description': 'return after reset'}, 'reset-return')
        result, _, _ = self.run_task(bridge, script[:7] + [reset_before_return,
            tool_call('land_drone', {'reason': 'safe stop'}, 'land'), AIMessage('已停止。')],
            auto_prepare=True, confirmation='multiview', architecture='single_vlm')
        self.assertEqual(result['status'], 'failed')
        self.assertFalse(result.get('search_protocol_corrections'))
        self.assertFalse(bridge.telemetry['armed'])

        # Even after a genuine unconfirmed review, a running executor task or
        # a different owner cannot be treated as a harmless rejected plan.
        for scenario in ('running', 'wrong_owner', 'world_changed', 'map_invalid', 'interrupted', 'vision_failure', 'flight_failure', 'initialization_failure'):
            with self.subTest(scenario=scenario):
                bridge = ScopedFixtureBridge()
                bridge.localize_stereo = lambda pair_id, bboxes: synthetic_metric_target(bridge, pair_id, bboxes)
                from autonomous_search import AdaptiveSearch
                original_validate = AdaptiveSearch.validate_state
                invalid_map = [False]
                def validate_with_fault(search, owner, state):
                    if invalid_map[0]:
                        search.sessions[owner]['spatial_map_identity']['status'] = 'invalid'
                    return original_validate(search, owner, state)
                def blocked_return(messages):
                    if scenario == 'running':
                        bridge.telemetry['mission']['status'] = 'running'
                    elif scenario == 'wrong_owner':
                        self.last_app.current_command['id'] = 'different-command'
                    elif scenario == 'world_changed':
                        bridge.telemetry['scene']['world_sha256'] = 'changed-world'
                    elif scenario == 'map_invalid':
                        invalid_map[0] = True
                    elif scenario == 'interrupted':
                        self.last_app.current_command['interrupted'] = True
                    elif scenario == 'vision_failure':
                        self.last_app.current_command['vision_failure'] = 'controlled failure'
                    elif scenario == 'flight_failure':
                        self.last_app.current_command['flight_failure'] = 'controlled failure'
                    else:
                        self.last_app.current_command['search_initialization_failure'] = 'controlled failure'
                    return tool_call('execute_mission', {'steps': [{'action': 'return_home'}, {'action': 'land'}],
                                     'description': 'blocked return'}, 'blocked-'+scenario)
                with mock.patch.object(AdaptiveSearch, 'validate_state', validate_with_fault):
                    result, _, _ = self.run_task(bridge, script[:7] + [blocked_return,
                        tool_call('land_drone', {'reason': 'safe stop'}, 'land'), AIMessage('已停止。')],
                        auto_prepare=True, confirmation='multiview', architecture='single_vlm')
                self.assertEqual(result['status'], 'failed')
                self.assertFalse(result.get('search_protocol_corrections'))
                self.assertFalse(any(x['path'] == '/mission' and x['steps'][0]['action'] == 'return_home'
                                     for x in bridge.flight_requests))

    def suffix(self):
        return [tool_call('observe_target_search_view', {}, 'second-view'), lambda m: saved_report(m, True),
                tool_call('execute_mission', {'steps': [{'action': 'return_home'}, {'action': 'land'}],
                    'description': 'user-authorized completion'}, 'finish'), AIMessage('完成并降落。')]

    def test_multiview_graph_requires_paired_delivery_and_distinct_confirmation_move(self):
        from coverage_search import project_ground_point
        bridge = ScopedFixtureBridge()
        # A fake metric surface exercises the positive confirmation path; it is not visual evidence.
        bridge.localize_stereo = lambda pair_id, bboxes: synthetic_metric_target(bridge, pair_id, bboxes)
        def candidate(messages):
            capture = json.loads(next(m for m in reversed(messages)
                if getattr(m, 'name', None) == 'observe_target_search_view').content)
            pair = next(reversed(bridge.pairs.values()))
            u, v = project_ground_point(pair['left']['camera_intrinsics'], pair['body_pose_enu'],
                                       [5., 0., 1.], 20.)['pixel_uv']
            bbox = [u/640-.06, v/480-.07, u/640+.06, v/480+.07]
            return tool_call('report_scene', {'capture_id': capture['capture_id'], 'summary': 'synthetic candidate',
                'targets': [{'label': '蓝色油桶', 'bbox': bbox, 'confidence': None,
                             'surroundings': [], 'position_basis': 'image_only'}],
                'limitations': ['controlled fixture, not recognition accuracy']}, 'report-'+capture['capture_id'])
        def review(messages):
            value = json.loads(next(m for m in reversed(messages) if getattr(m, 'name', None) == 'report_scene').content)
            progress = value['search_progress']
            self.assertEqual(progress['search_status'], 'searching')
            self.assertEqual(progress['search_phase'], 'awaiting_review')
            active = progress['confirmation']
            return tool_call('review_target_candidate', {'source_capture_id': active['source_capture_id'],
                'current_capture_id': active['current_capture_id'], 'current_target_id': active['current_target_ids'][0],
                'source_supported': True, 'current_supported': True, 'same_object': True,
                'source_description': 'controlled source appearance', 'current_description': 'controlled current appearance',
                'reason': 'controlled two-view review'}, 'review')
        script = self.prefix()[:-1]+[candidate, tool_call('observe_target_search_view', {}, 'second-view'), candidate,
            review]+self.suffix()[-2:]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR': str(root/'captures')}):
                result, _, _ = self.run_task(bridge, script, auto_prepare=True, confirmation='multiview')
            self.assertEqual(result['status'], 'succeeded', result.get('error'))
            import sys
            sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
            from audit_target_confirmation import audit
            record = {'command': result, 'target_confirmation_mode': 'multiview',
                      'flight': bridge.state(), 'capture_directory': str(root/'captures')}
            path = root/'result.json'
            path.write_text(json.dumps(record))
            reviewed = audit(path, root/'audit')
            self.assertTrue(reviewed['passed'], reviewed)
            self.assertTrue(audit(path, root/'positive-audit', require_confirmed=True)['passed'])
            tampered_review = copy.deepcopy(record)
            event = next(e for e in tampered_review['command']['target_confirmation_events']
                         if e['outcome'] == 'confirmed')
            event.update(source_supported=False, source_uncertain=True)
            uncertain_path = root/'uncertain-source-result.json'
            uncertain_path.write_text(json.dumps(tampered_review))
            uncertain_audit = audit(uncertain_path, root/'uncertain-source-audit')
            self.assertFalse(uncertain_audit['passed'])
            self.assertIn('Review outcome differs', str(uncertain_audit['problems']))
            tampered_basis = copy.deepcopy(record)
            event = next(e for e in tampered_basis['command']['target_confirmation_events']
                         if e['outcome'] == 'confirmed')
            event['geometry']['association_basis'] = 'bearing_intersection'
            basis_path = root/'incorrect-basis-result.json'
            basis_path.write_text(json.dumps(tampered_basis))
            basis_audit = audit(basis_path, root/'incorrect-basis-audit')
            self.assertFalse(basis_audit['passed'])
            self.assertIn('association basis differs', str(basis_audit['problems']))
            self.assertFalse(audit(path, root/'missing-rejection-audit', require_rejected=True)['passed'])
            tampered_record = copy.deepcopy(record)
            tampered_record['command']['search_outcome']['target_confirmed'] = False
            tampered_path = root/'inconsistent-result.json'
            tampered_path.write_text(json.dumps(tampered_record))
            inconsistent = audit(tampered_path, root/'inconsistent-audit')
            self.assertFalse(inconsistent['checks']['structured_final_outcome_consistent'])
            paired_call = next(c for c in result['model_calls'] if c.get('image_purpose') == 'target_confirmation')
            roi = root/'captures'/paired_call['derived_roi']['archive_filename']
            saved = roi.read_bytes()
            roi.write_bytes(b'corrupted archived ROI')
            rejected = audit(path, root/'tampered-audit')
            self.assertFalse(rejected['passed'])
            self.assertIn('Archived ROI', str(rejected['problems']))
            roi.write_bytes(saved)
            current_roi = root/'captures'/paired_call['current_rois'][0]['archive_filename']
            current_roi.write_bytes(b'corrupted current ROI')
            rejected = audit(path, root/'tampered-current-audit')
            self.assertFalse(rejected['passed'])
            self.assertIn('Archived current ROI', str(rejected['problems']))
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(bridge.ticket_executions, 1)
        self.assertEqual(result['nbv_events'][0]['planning_mode'], 'confirmation')
        self.assertEqual(len(result['perceptions']), 2)
        calls = [c for c in result['model_calls'] if c.get('image_purpose') == 'target_confirmation']
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0]['capture_ids'], [p['capture_id'] for p in result['perceptions']])
        self.assertTrue(calls[0]['derived_roi']['derived_not_new_capture'])
        self.assertEqual(result['target_confirmation_events'][-1]['outcome'], 'confirmed')
        self.assertTrue(result['search_outcome']['target_confirmed'])
        self.assertEqual(result['search_outcome']['review_count'], 1)
        self.assertIn('多视角确认流程已确认', result['reply'])
        self.assertFalse(bridge.telemetry['armed'])

    def test_real_graph_uses_same_owner_map_and_ticket_transport(self):
        bridge = ScopedFixtureBridge()
        result, app, _ = self.run_task(bridge, self.prefix()+[
            tool_call('next_target_search_view', {'strategy': 'auto', 'reason': 'actual report did not confirm target'}, 'next')]+self.suffix())
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(bridge.ticket_executions, 1)
        self.assertEqual(bridge.generic_gotos, 0)
        self.assertEqual(len(result['perceptions']), 2)
        self.assertEqual([e['status'] for e in result['semantic_map_events']], ['valid', 'valid'])
        self.assertEqual(len(result['nbv_events']), 1)
        self.assertEqual(result['nbv_events'][0]['owner'][1], result['id'])
        self.assertEqual(result['nbv_events'][0]['source_capture_id'], result['perceptions'][0]['capture_id'])
        self.assertEqual(len(result['flight_mission_results']), 3)
        self.assertNotIn('execution_ticket', json.dumps(result['nbv_events']))
        event = next(e for e in result['search_events'] if e['tool'] == 'next_target_search_view')['result']
        self.assertIn('candidate_id', event)
        self.assertNotIn('goto', event)
        self.assertEqual(result['flight_missions'][1]['mission']['search_constraint']['owner'][1], result['id'])
        self.assertFalse(bridge.telemetry['armed'])

    def test_vlm_can_choose_offered_alternative_using_same_image_and_private_ticket(self):
        bridge = ScopedFixtureBridge()
        chosen = []
        def choose(messages):
            response = next(m for m in reversed(messages) if getattr(m, 'name', None) == 'next_target_search_view')
            value = json.loads(response.content)
            self.assertEqual(value['next_tool'], 'select_target_search_view')
            self.assertIsNone(value['pending'])
            self.assertGreaterEqual(len(value['candidates']), 2)
            encoded = json.dumps(value['candidates'])
            self.assertNotIn('execution_ticket', encoded)
            self.assertNotIn('position_enu_m', encoded)
            self.assertEqual(bridge.ticket_executions, 0)
            self.assertNotIn('numeric_best_id', value)
            self.assertFalse(any('score' in c or 'score_terms' in c for c in value['candidates']))
            # Only the test harness reads internal audit facts. The actual model
            # receives neither the numerical winner nor weighted scores.
            numeric_best = self.last_app.current_command['nbv_candidate_events'][-1]['numeric_best_id']
            alternative = next(c['id'] for c in reversed(value['candidates']) if c['id'] != numeric_best)
            chosen.append((numeric_best, alternative))
            return tool_call('select_target_search_view', {'batch_id': value['batch_id'],
                'candidate_id': alternative, 'reason': 'fixture chooses a verified alternative'}, 'select')
        result, _, _ = self.run_task(bridge, self.prefix()+[
            tool_call('next_target_search_view', {'strategy': 'auto', 'reason': 'fixture'}, 'next'),
            choose]+self.suffix(), selector='vlm')
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertNotEqual(*chosen[0])
        self.assertEqual(bridge.ticket_executions, 1)
        self.assertEqual(bridge.generic_gotos, 0)
        self.assertEqual(len(result['nbv_candidate_events']), 1)
        prediction = result['nbv_events'][0]
        self.assertEqual(prediction['selector'], 'vlm')
        self.assertEqual(prediction['selected_id'], chosen[0][1])
        self.assertEqual(prediction['numeric_best_id'], chosen[0][0])
        calls = [c for c in result['model_calls'] if c.get('image_purpose') == 'nbv_candidate_selection']
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0]['capture_ids'], [result['perceptions'][0]['capture_id']])
        self.assertIn('image', calls[0]['input_modalities'])
        self.assertEqual(len(result['semantic_map_events']), 2)

    def test_vlm_unknown_id_or_foreign_batch_never_flies_and_safely_lands(self):
        for bad in ('id', 'batch'):
            with self.subTest(bad=bad):
                bridge = ScopedFixtureBridge()
                def choose(messages):
                    value = json.loads(next(m for m in reversed(messages)
                        if getattr(m, 'name', None) == 'next_target_search_view').content)
                    return tool_call('select_target_search_view', {
                        'batch_id': '0'*32 if bad == 'batch' else value['batch_id'],
                        'candidate_id': 'view_999' if bad == 'id' else value['candidates'][0]['id'],
                        'reason': 'controlled invalid selection'}, 'select')
                result, _, _ = self.run_task(bridge, self.prefix()+[
                    tool_call('next_target_search_view', {'strategy': 'auto', 'reason': 'fixture'}, 'next'),
                    choose, tool_call('land_drone', {'reason': 'failed selection'}, 'land'),
                    AIMessage('选择失败，已降落。')], selector='vlm', architecture='single_vlm')
                self.assertEqual(result['status'], 'failed')
                self.assertEqual(bridge.ticket_executions, 0)
                self.assertEqual(bridge.generic_gotos, 0)
                self.assertFalse(bridge.telemetry['armed'],
                    {'error':result.get('error'),'safety_landing':result.get('safety_landing'),
                     'flight_requests':bridge.flight_requests,'tool_errors':result.get('tool_errors')})

    def test_failed_selection_followed_by_model_failure_triggers_owned_safety_landing(self):
        bridge = ScopedFixtureBridge()
        def choose(messages):
            value = json.loads(next(m for m in reversed(messages)
                if getattr(m, 'name', None) == 'next_target_search_view').content)
            return tool_call('select_target_search_view', {'batch_id':value['batch_id'],
                'candidate_id':'view_999','reason':'invalid test ID'}, 'select')
        result, _, _ = self.run_task(bridge, self.prefix()+[
            tool_call('next_target_search_view', {'strategy':'auto','reason':'fixture'}, 'next'),
            choose, RuntimeError('controlled model outage before landing')],
            selector='vlm', architecture='single_vlm')
        self.assertEqual(result['status'],'failed')
        self.assertEqual(bridge.ticket_executions,0)
        self.assertFalse(bridge.telemetry['armed'])
        self.assertEqual(result['safety_landing']['status'],'ok')
        self.assertEqual(len([x for x in bridge.flight_requests if x['path']=='/land']),1)

    def test_premature_return_is_marked_as_rejected_before_submission_and_lands_safely(self):
        bridge = ScopedFixtureBridge()
        script=self.prefix()+[tool_call('execute_mission', {'steps':[{'action':'return_home'},{'action':'land'}],
            'description':'premature return in active search'}, 'premature'),
            tool_call('execute_mission', {'steps':[{'action':'return_home'},{'action':'land'}],
                'description':'repeat rejected plan'}, 'premature-again'),
            tool_call('land_drone', {'reason':'safe stop'}, 'land'), AIMessage('已停止。')]
        result, _, _ = self.run_task(bridge, script, auto_prepare=True, confirmation='multiview', architecture='single_vlm')
        self.assertEqual(result['status'],'failed')
        self.assertEqual(result['flight_failure_stage'],'search_mission_precheck')
        self.assertIn('未通过提交前的搜索检查，该计划未被提交执行',result['reply'])
        self.assertFalse(result['search_outcome']['target_confirmed'])
        self.assertFalse(any(step['action']=='return_home' for request in bridge.flight_requests for step in request['steps']))
        self.assertEqual(len([request for request in bridge.flight_requests if request['path']=='/land']),1)
        self.assertFalse(bridge.telemetry['armed'])

    def test_exploration_return_correction_preserves_ticket_and_continues_observation(self):
        bridge = ScopedFixtureBridge()
        def premature(messages):
            self.tickets_before_correction = copy.deepcopy(bridge.authority.tickets)
            return tool_call('execute_mission', {'steps': [{'action': 'return_home'}, {'action': 'land'}],
                'description': 'controlled premature completion'}, 'premature-explore')
        def continue_observation(messages):
            correction = json.loads(next(m for m in reversed(messages) if getattr(m, 'name', None) == 'execute_mission').content)
            self.assertEqual(correction['status'], 'not_executed')
            self.assertEqual(correction['next_tool'], 'observe_target_search_view')
            self.assertEqual(bridge.authority.tickets, self.tickets_before_correction)
            self.assertEqual(bridge.ticket_executions, 0)
            return tool_call(correction['next_tool'], {}, 'resume-exploring')
        script = self.prefix() + [premature, continue_observation, saved_report,
            tool_call('land_drone', {'reason': 'controlled test ends after second observation'}, 'land'),
            AIMessage('受控测试结束，目标未确认。')]
        result, _, _ = self.run_task(bridge, script, auto_prepare=True, confirmation='multiview')
        self.assertEqual(len(result['search_protocol_corrections']), 1)
        self.assertEqual(result['search_protocol_corrections'][0]['reason'], 'unfinished_exploration')
        self.assertEqual(bridge.ticket_executions, 1)
        self.assertEqual(len([p for p in result['perceptions'] if p.get('status') == 'ok']), 2)
        self.assertFalse(any(step['action'] == 'return_home' for r in bridge.flight_requests for step in r['steps']))
        self.assertFalse(bridge.telemetry['armed'])

    def test_even_matching_generic_flight_call_is_routed_through_private_ticket(self):
        bridge = ScopedFixtureBridge()
        def copied_coordinates(messages):
            # Deliberately adversarial test harness knows the hidden point; the
            # actual model's next-view result contains only an ID and metrics.
            item = next(iter(bridge.authority.tickets.values()))
            p = item['goal_enu_m']
            return tool_call('execute_mission', {'steps': [{'action': 'goto', 'x': p[0], 'y': p[1], 'altitude': p[2]}],
                'description': 'attempt to copy matching coordinates'}, 'generic-attempt')
        result, _, _ = self.run_task(bridge, self.prefix()+[
            tool_call('next_target_search_view', {'strategy': 'rise', 'reason': 'fixture'}, 'next'),
            copied_coordinates]+self.suffix())
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(bridge.ticket_executions, 1)
        self.assertEqual(bridge.generic_gotos, 0)

    def test_map_failure_cannot_silently_fall_back_to_fixed_movement(self):
        bridge = ScopedFixtureBridge()
        bridge.reject_geometry = True
        result, _, _ = self.run_task(bridge, self.prefix()+[
            tool_call('land_drone', {'reason': 'independent safety completion'}, 'land'), AIMessage('地图失败，已降落。')])
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(bridge.ticket_executions, 0)
        self.assertEqual(bridge.generic_gotos, 0)
        self.assertFalse(bridge.telemetry['armed'])
        self.assertEqual(result['semantic_map_events'][0]['status'], 'rejected')

    def test_archived_graph_output_passes_execution_and_camera_provenance_audits(self):
        from scripts.audit_nbv_evidence import audit_record
        from scripts.audit_semantic_map_evidence import audit as semantic_audit
        bridge = ScopedFixtureBridge()
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            captures = directory/'captures'
            with mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR': str(captures)}):
                result, _, _ = self.run_task(bridge, self.prefix()+[
                    tool_call('next_target_search_view', {'strategy': 'auto', 'reason': 'fixture'}, 'next')]+self.suffix())
            self.assertEqual(result['status'], 'succeeded', result.get('error'))
            record = {'command': result, 'capture_directory': str(captures),
                      'flight': {'armed': False, 'landed_state': 1}}
            audited = audit_record(record,
                lambda capture: json.loads((captures/(capture+'.semantic-map.json')).read_text()),
                lambda identity, a, b: bridge.nav.segment_safe(a, b))
            self.assertTrue(audited['passed'], audited)
            self.assertEqual(len(audited['moves']), 1)
            source = audited['moves'][0]['source_capture_id']
            archive = json.loads((captures/(source+'.nbv_geometry.json')).read_text())
            self.assertEqual(archive, result['nbv_events'][0])
            source_result = directory/'result.json'
            source_result.write_text(json.dumps(record))
            evidence = semantic_audit(source_result, directory/'map-audit')
            for name in ('online_replay_equal', 'map_events_cover_successful_reports',
                         'duplicate_rejected_without_mutation', 'replay_has_all_successful_reports'):
                self.assertTrue(evidence['checks'][name], evidence)

    def test_archived_vlm_choice_has_offer_model_image_choice_and_execution_lineage(self):
        from scripts.audit_nbv_evidence import audit_record
        bridge = ScopedFixtureBridge()
        def choose(messages):
            value = json.loads(next(m for m in reversed(messages)
                if getattr(m, 'name', None) == 'next_target_search_view').content)
            return tool_call('select_target_search_view', {'batch_id': value['batch_id'],
                'candidate_id': value['candidates'][-1]['id'], 'reason': 'verified alternate fixture'}, 'select')
        with tempfile.TemporaryDirectory() as temporary:
            captures = Path(temporary)/'captures'
            with mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR': str(captures)}):
                result, _, _ = self.run_task(bridge, self.prefix()+[
                    tool_call('next_target_search_view', {'strategy': 'auto', 'reason': 'fixture'}, 'next'),
                    choose]+self.suffix(), selector='vlm')
            self.assertEqual(result['status'], 'succeeded', result.get('error'))
            record = {'command': result, 'flight': {'armed': False, 'landed_state': 1}}
            def audit(value):
                return audit_record(value,
                    lambda capture: json.loads((captures/(capture+'.semantic-map.json')).read_text()),
                    lambda identity, a, b: bridge.nav.segment_safe(a, b), require_vlm_selection=True)
            checked = audit(record)
            self.assertTrue(checked['passed'], checked)
            self.assertEqual(checked['vlm_selected_move_count'], 1)
            for change in ('offer', 'reason', 'image'):
                with self.subTest(change=change):
                    broken = copy.deepcopy(record)
                    if change == 'offer': broken['command']['nbv_candidate_events'][0]['offered_candidate_ids'] = ['view_999']
                    elif change == 'reason': broken['command']['nbv_events'][0]['selection_reason'] = 'unrecorded reason'
                    else:
                        for call in broken['command']['model_calls']:
                            if call.get('image_purpose') == 'nbv_candidate_selection': call['capture_ids'] = ['other-capture']
                    self.assertFalse(audit(broken)['passed'])

    def test_report_auto_prepares_diverse_choices_without_next_view_model_round_trip(self):
        from scripts.audit_nbv_evidence import audit_record
        bridge = ScopedFixtureBridge()
        def choose(messages):
            value = json.loads(next(m for m in reversed(messages)
                if getattr(m, 'name', None) == 'report_scene').content)['search_progress']
            self.assertEqual(value['next_tool'], 'select_target_search_view')
            self.assertEqual(value['candidate_pool_mode'], 'diverse')
            rising = next(c for c in value['candidates'] if c['direction'].endswith('升高'))
            return tool_call('select_target_search_view', {'batch_id': value['batch_id'],
                'candidate_id': rising['id'], 'reason': 'synthetic fixture selects a verified rising alternative'}, 'select')
        with tempfile.TemporaryDirectory() as temporary:
            captures = Path(temporary)/'captures'
            with mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR': str(captures)}):
                result, _, _ = self.run_task(bridge, self.prefix()+[choose]+self.suffix(),
                    selector='vlm', pool='diverse', auto_prepare=True)
            self.assertEqual(result['status'], 'succeeded', result.get('error'))
            self.assertEqual(len(result['perceptions']), 2)
            self.assertEqual(len(result['nbv_events']), 1)
            self.assertEqual(result['nbv_events'][0]['prepared_by'], 'report_scene')
            self.assertLessEqual(result['nbv_events'][0]['path_evaluation_count'], 12)
            self.assertFalse(any(t['name'] == 'next_target_search_view' for c in result['model_calls'] for t in c['called_tools']))
            record = {'command': result, 'flight': {'armed': False, 'landed_state': 1}}
            audit = audit_record(record,
                lambda capture: json.loads((captures/(capture+'.semantic-map.json')).read_text()),
                lambda identity, a, b: bridge.nav.segment_safe(a, b), require_vlm_selection=True,
                require_diverse_pool=True, require_auto_prepare=True)
            self.assertTrue(audit['passed'], audit)

    def test_auto_preparation_map_failure_never_falls_back_to_movement(self):
        bridge = ScopedFixtureBridge()
        bridge.reject_geometry = True
        result, _, _ = self.run_task(bridge, self.prefix()+[
            tool_call('land_drone', {'reason': 'map failed'}, 'land'), AIMessage('已安全降落。')],
            selector='vlm', pool='diverse', auto_prepare=True)
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(bridge.ticket_executions, 0)
        self.assertFalse(bridge.telemetry['armed'])

    def test_numeric_and_vlm_selectors_share_the_same_diverse_pool_configuration(self):
        numeric, _, _ = self.run_task(ScopedFixtureBridge(), self.prefix()+self.suffix(),
            selector='deterministic', pool='diverse', auto_prepare=True)
        def choose(messages):
            value = json.loads(next(m for m in reversed(messages)
                if getattr(m, 'name', None) == 'report_scene').content)['search_progress']
            return tool_call('select_target_search_view', {'batch_id': value['batch_id'],
                'candidate_id': value['candidates'][0]['id'], 'reason': 'same-pool control'}, 'select')
        vlm, _, _ = self.run_task(ScopedFixtureBridge(), self.prefix()+[choose]+self.suffix(),
            selector='vlm', pool='diverse', auto_prepare=True)
        for result in (numeric, vlm):
            self.assertEqual(result['status'], 'succeeded', result.get('error'))
        a, b = numeric['nbv_events'][0], vlm['nbv_events'][0]
        self.assertEqual(a['config'], b['config'])
        self.assertEqual(a['candidate_pool_ids'], b['candidate_pool_ids'])
        self.assertEqual(a['path_evaluation_count'], b['path_evaluation_count'])
        self.assertEqual(a['ranked_candidates'], b['ranked_candidates'])
