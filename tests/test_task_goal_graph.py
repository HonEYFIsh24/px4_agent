"""Actual Agent/authority/geometry wiring with explicit synthetic camera/flight doubles."""
import copy
import json
import math
import os
import unittest
from unittest import mock

from task_runtime.goal_planner import camera_facing_yaw
from viewpoint_geometry import StaticViewpointEvaluator, registered_identity
from goal_pose_geometry import query_goal_poses, validate_goal_context
from search_tools import coordinate_epoch
import test_task_runtime as runtime_fixtures
from test_task_runtime import RuntimeBridge, review
from test_search_graph_integration import LocalStereoBridge
from test_target_observation import measured_stereo
from test_goal_pose import region
from test_single_vlm import tool_call


class GoalRuntimeBridge(RuntimeBridge):
    front, rear, surface = [5., -2., 1.], [5., 2., 1.], [5., 0., 1.]

    def __init__(self):
        super().__init__()
        self.actual_yaw, self.goal_executions = 0., 0

    def state(self):
        value = super().state()
        stamp = 42.+self.counter+.1
        value['navigation']['goal_pose_execution_supported'] = True
        value['navigation']['viewpoint_yaw_supported'] = True
        value['stamp_sim_s'] = stamp
        value['yaw_enu_rad'] = self.actual_yaw
        value['body_pose_enu'] = {'frame_id': 'map', 'body_reference': 'base_link',
            'position_m': value['position_enu_m'], 'orientation_xyzw': [0., 0., math.sin(self.actual_yaw/2), math.cos(self.actual_yaw/2)],
            'stamp_sim_s': stamp, 'age_seconds': .1}
        return value

    def capture_stereo(self):
        pair = super().capture_stereo()
        pair['body_pose_enu'].update(orientation_xyzw=[0., 0., math.sin(self.actual_yaw/2), math.cos(self.actual_yaw/2)],
            orientation_covariance_rad2=[[.003, 0., 0.], [0., .003, 0.], [0., 0., .003]])
        self.pairs[pair['pair_id']] = copy.deepcopy(pair)
        return pair

    def localize_stereo(self, pair_id, boxes):
        pair = self.pairs[pair_id]
        snapshot = {'body_pose_enu': pair['body_pose_enu'], 'stamp_sim_s': pair['stamp_sim_s'],
            'camera_intrinsics': pair['left']['camera_intrinsics'], 'stereo_image_sha256': pair['image_sha256'],
            'coordinate_epoch': coordinate_epoch(self.telemetry)}
        points = ([self.front, self.rear] if len(boxes) == 2 else
                  [self.front if box[2]-box[0] < .2 else self.surface for box in boxes])
        return {'status': 'ok', 'targets': [measured_stereo({'snapshot': snapshot}, p) for p in points]}

    def request(self, path, payload=None, **kwargs):
        if path == '/navigation/viewpoints':
            pair = self.pairs[payload['pair_id']]
            identity = registered_identity(self.nav, coordinate_epoch(self.telemetry))
            context = payload['search_context']
            if payload['operation'] == 'context':
                self.authority.bind(context, pair['pair_id'], pair, identity, payload['origin_xy_enu_m'])
            else:
                self.authority.require_batch(context)
            backend = StaticViewpointEvaluator(self.nav, self.state()['body_pose_enu'], pair['left']['camera_intrinsics'],
                coordinate_epoch(self.telemetry), fixed_yaw=self.actual_yaw, reference_kind='live_enu_pose',
                probe_points_enu_m=payload.get('probe_points_enu_m'), allow_yaw=True)
            result = backend.query(payload['operation'], payload['origin_xy_enu_m'], identity, payload['candidates'])
            if payload['operation'] == 'paths':self.authority.issue(context, result)
            return result
        if path == '/navigation/goal-poses':
            pair = self.pairs[payload['pair_id']]
            identity = registered_identity(self.nav, coordinate_epoch(self.telemetry))
            context = payload['search_context']
            if payload['operation'] == 'context':
                self.authority.bind(context, pair['pair_id'], pair, identity, payload['origin_xy_enu_m'])
            else:
                self.authority.require_batch(context)
            validate_goal_context(payload['goal_context'], self.authority.active)
            backend = StaticViewpointEvaluator(self.nav, self.state()['body_pose_enu'], pair['left']['camera_intrinsics'],
                coordinate_epoch(self.telemetry), fixed_yaw=self.actual_yaw, reference_kind='live_enu_pose',
                probe_points_enu_m=payload['probe_points_enu_m'], allow_yaw=True)
            result = query_goal_poses(backend, payload['operation'], payload['origin_xy_enu_m'], identity, payload['candidates'],
                                      goal_context=payload['goal_context'])
            if payload['operation'] == 'paths':self.authority.issue(context, result, goal_context=payload['goal_context'])
            return result
        if path in ('/navigation/execute-goal-pose', '/navigation/execute-viewpoint'):
            value = self.authority.consume(payload['owner'], payload['batch_id'], payload['map_revision'],
                payload['candidate_id'], payload['ticket'], registered_identity(self.nav, coordinate_epoch(self.telemetry)),
                self.telemetry['position_enu_m'], self.actual_yaw, goal_context=payload.get('goal_context'))
            p = value['goal_enu_m']
            accepted = LocalStereoBridge.request(self, '/mission', {'steps': [{'action': 'goto', 'x': p[0], 'y': p[1], 'altitude': p[2]}],
                                                                  'description': payload['description']})
            if 'target_yaw_enu_rad' in value:
                self.actual_yaw = value['target_yaw_enu_rad']
                self.telemetry['mission']['steps'].append({'action': 'rotate', 'yaw_rad': self.actual_yaw})
            self.telemetry['mission']['goal_context'] = payload.get('goal_context')
            self.telemetry['mission']['goal_pose_enu_m'] = list(p)
            self.telemetry['mission']['goal_yaw_enu_rad'] = self.actual_yaw
            self.telemetry['mission']['events'] = [{'action': 'rotate', 'rotation': {'stamp_s': self.state()['stamp_sim_s']}}]
            if path == '/navigation/execute-goal-pose':self.goal_executions += 1
            else:self.ticket_executions += 1
            return {'mission': copy.deepcopy(self.telemetry['mission'])}
        return super().request(path, payload, **kwargs)


def plan(landing=False):
    return {'targets': [{'ref': 'van', 'label': '白色货车'}], 'stages': [
        {'kind': 'search', 'target_ref': 'van'},
        {'kind': 'relative_land' if landing else 'relative_hover', 'target_ref': 'van', 'relation': 'object_front'},
        *([] if landing else [{'kind': 'hold'}])]}


def response_for(bridge, missing_front_once=False):
    unknown = [missing_front_once]
    def response(messages):
        state = next(json.loads(m.content.split('父任务实际状态：', 1)[1]) for m in reversed(messages)
            if isinstance(getattr(m, 'content', None), str) and m.content.startswith('父任务实际状态：'))
        next_tool = state['next_tool']
        if next_tool == 'review_target_candidate':
            return review(messages)
        if next_tool == 'report_target_frame':
            reference = state['target_memory']['van']
            capture = reference['last_capture_id']
            pair = next(reversed(bridge.pairs.values()))
            frame = {'snapshot': {'body_pose_enu': pair['body_pose_enu'], 'camera_intrinsics': pair['left']['camera_intrinsics']}}
            boxes = [region(frame, p) for p in (bridge.front, bridge.rear)]
            if unknown[0]:
                boxes = [None, None];unknown[0] = False
            return tool_call('report_target_frame', {'capture_id': capture,
                'front_description': 'controlled visible front', 'rear_description': 'controlled visible rear',
                'front_box': boxes[0], 'rear_box': boxes[1]})
        if next_tool == 'review_goal_region':
            reference = state['target_memory']['van']
            pair = next(reversed(bridge.pairs.values()))
            frame = {'snapshot': {'body_pose_enu': pair['body_pose_enu'], 'camera_intrinsics': pair['left']['camera_intrinsics']}}
            return tool_call('review_goal_region', {'capture_id': reference['last_capture_id'], 'front_visible': True,
                'front_box': region(frame, bridge.front), 'description': 'controlled fresh front face'})
        if next_tool == 'report_scene':
            message = next(m for m in reversed(messages) if getattr(m, 'name', None) in ('observe_target_search_view', 'execute_task_goal'))
            capture = json.loads(message.content)['capture_id']
            return tool_call('report_scene', {'capture_id': capture, 'summary': 'synthetic goal fixture',
                'targets': [{'label': '白色货车', 'bbox': [.1, .1, .9, .9], 'surroundings': [], 'position_basis': 'image_only'}],
                'limitations': ['controlled geometry, not actual recognition']})
        if next_tool == 'revise_task_plan':
            return tool_call('revise_task_plan', {'remaining_stage_ids': [s['stage_id'] for s in state['plan']['stages'] if s['status'] != 'completed'],
                'strategy': 'reacquire_target', 'reason': 'controlled retained-instance recovery'})
        raise AssertionError('Unexpected model phase '+str(next_tool))
    return response


class TaskGoalGraphTests(unittest.TestCase):
    def run_bearing_handoff(self, *, uncertain=False, reset=False):
        class BearingBridge(GoalRuntimeBridge):
            front, rear, surface = [8., -2., 1.], [8., 2., 1.], [8., 0., 1.]

            def __init__(self):
                super().__init__()
                self.no_metric_pairs = set()

            def capture_stereo(self):
                import base64
                import hashlib
                import io
                from test_task_association import textured
                pair = super().capture_stereo()
                # Independent synthetic textures deliberately provide no patch
                # identity shortcut. Stereo/visual IO are explicit doubles.
                buffer = io.BytesIO();textured(self.counter*101).save(buffer, 'JPEG')
                raw = buffer.getvalue()
                for side in ('left', 'right'):
                    pair[side]['image_base64'] = base64.b64encode(raw).decode()
                    pair['image_sha256'][side] = hashlib.sha256(raw).hexdigest()
                self.pairs[pair['pair_id']] = copy.deepcopy(pair)
                return pair

            def localize_stereo(self, pair_id, boxes):
                if pair_id in self.no_metric_pairs:
                    return {'status': 'ok', 'targets': [{'status': 'unavailable',
                        'reason': 'controlled first two views have no depth'} for _ in boxes]}
                return super().localize_stereo(pair_id, boxes)
        bridge = BearingBridge()
        runner = runtime_fixtures.RuntimeGraphTests()
        ordinary = response_for(bridge, missing_front_once=True)
        reviews = []
        stopped = [False]
        reports = [0]
        def response(messages):
            if stopped[0]:
                raise RuntimeError('controlled stop after unresolved original-reference review')
            state = next(json.loads(m.content.split('父任务实际状态：', 1)[1]) for m in reversed(messages)
                if isinstance(getattr(m, 'content', None), str) and m.content.startswith('父任务实际状态：'))
            if (state['next_tool'] == 'report_target_frame' and
                    any(r.get('purpose') == 'parent_reference_association' for r in reviews)):
                # This fixture ends at the repaired handoff. Existing goal and
                # landing graph positives separately verify the geometry flow.
                raise RuntimeError('controlled stop after verified original-reference handoff')
            if state['next_tool'] == 'report_scene':
                reports[0] += 1
                if reports[0] <= 2:
                    bridge.no_metric_pairs.add(next(reversed(bridge.pairs)))
            if state['next_tool'] != 'review_target_candidate':
                return ordinary(messages)
            for message in reversed(messages):
                content = getattr(message, 'content', None)
                if (isinstance(content, list) and content and content[0].get('type') == 'text'
                        and '调用review_target_candidate。元数据：' in content[0].get('text', '')):
                    metadata = json.loads(content[0]['text'].split('元数据：', 1)[1])
                    reviews.append(metadata)
                    original = metadata.get('purpose') == 'parent_reference_association'
                    if original and reset:
                        bridge.telemetry['external_vision']['adapter']['epoch'] += 1
                    stopped[0] = original and uncertain
                    return tool_call('review_target_candidate', {
                        'current_target_id': metadata['current_target_ids'][0],
                        'source_supported': not (original and uncertain), 'source_uncertain': original and uncertain,
                        'current_supported': True, 'same_object': True,
                        'source_description': 'controlled original candidate pixels',
                        'current_description': 'controlled current metric pixels', 'reason': 'controlled actual pair'})
            return ordinary(messages)
        script = [tool_call('define_task_plan', {'plan': plan()})]+[response]*24
        with mock.patch.dict(os.environ, {'UAV_NBV_HEADING_MODE': 'discrete'}):
            result, app, _ = runner.run_graph(script, '找到白色货车后，在它前方悬停并面对车头', bridge, timeout=40)
        return bridge, result, app, reviews

    def test_original_bearing_handoff_is_reviewed_against_metric_image_before_goal_execution(self):
        bridge, result, app, reviews = self.run_bearing_handoff()
        self.assertEqual(result['error'], 'controlled stop after verified original-reference handoff')
        extra = [r for r in reviews if r.get('purpose') == 'parent_reference_association']
        self.assertEqual(len(extra), 1)
        events = result['task_runtime_events']
        event = next(e for e in events if e['event'] == 'original_reference_review_required')
        runtime = app.audited_client.task_runtime
        reference = event['target_memory']['van']
        self.assertEqual(extra[0]['source_capture_id'], reference['observation_refs'][0])
        self.assertNotEqual(extra[0]['target_id'], extra[0]['current_target_ids'][0])
        self.assertEqual(event['goal_state']['phase'], 'observing')
        self.assertIsNone((event['goal_state'].get('prepared') or {}).get('execution'))
        self.assertEqual(event['next_tool'], 'review_target_candidate')
        self.assertTrue(any(e['outcome'] == 'instance_linked' and
            e.get('canonical_target_id') == extra[0]['target_id'] for e in result['target_confirmation_events']))
        self.assertEqual(result['task_runtime']['target_memory']['van']['target_id'], extra[0]['target_id'])
        self.assertEqual(len(runtime.skills.search.semantic_maps.maps), 1)
        self.assertEqual(result['task_runtime']['target_memory']['van']['confirmation_level'], 'multiview')
        self.assertIsNotNone(result['task_runtime']['target_memory']['van']['position_enu_m'])
        self.assertEqual(bridge.goal_executions, 0)
        self.assertLessEqual(result['task_runtime']['budget']['observations_used'], 12)
        self.assertFalse(bridge.telemetry['armed'])

    def test_uncertain_original_reference_cannot_authorize_another_candidate_goal(self):
        bridge, result, _, reviews = self.run_bearing_handoff(uncertain=True)
        self.assertEqual(result['status'], 'failed')
        extra = next(r for r in reviews if r.get('purpose') == 'parent_reference_association')
        self.assertEqual(bridge.goal_executions, 0)
        self.assertEqual(result['task_runtime']['target_memory']['van']['target_id'], extra['target_id'])
        review = next(e for e in result['target_confirmation_events'] if e['outcome'] == 'unconfirmed'
            and e['candidate'].get('purpose') == 'parent_reference_association')
        self.assertTrue(review['source_uncertain'])
        self.assertFalse(review['source_supported'])
        self.assertFalse(bridge.telemetry['armed'])

    def test_epoch_change_during_original_reference_review_still_fails_and_lands(self):
        bridge, result, _, reviews = self.run_bearing_handoff(reset=True)
        self.assertEqual(result['status'], 'failed')
        self.assertTrue(any(r.get('purpose') == 'parent_reference_association' for r in reviews))
        self.assertEqual(bridge.goal_executions, 0)
        self.assertFalse(bridge.telemetry['armed'])
        self.assertFalse(result['task_outcome']['user_goal_completed'])

    def run_goal(self, missing=False):
        bridge = GoalRuntimeBridge()
        runner = runtime_fixtures.RuntimeGraphTests()
        script = [tool_call('define_task_plan', {'plan': plan()})]+[response_for(bridge, missing)]*24
        with mock.patch.dict(os.environ, {'UAV_NBV_HEADING_MODE': 'discrete'}):
            result, app, model = runner.run_graph(script, '找到白色货车后，在它前方悬停并面对车头', bridge)
        return bridge, result, app, model

    def test_parent_reaches_object_front_faces_it_and_verifies_a_fresh_target_image(self):
        bridge, result, app, _ = self.run_goal()
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(bridge.goal_executions, 1)
        self.assertTrue(result['task_outcome']['user_goal_completed'])
        self.assertTrue(bridge.telemetry['armed'])
        self.assertLess(bridge.telemetry['position_enu_m'][1], -3.)
        expected_yaw = camera_facing_yaw(bridge.telemetry['position_enu_m'], bridge.front)
        self.assertAlmostEqual(bridge.actual_yaw, expected_yaw)
        stage = result['task_runtime']['plan']['stages'][1]
        self.assertEqual(stage['conditions']['fresh_target_verification'], 'true')
        self.assertEqual(stage['conditions']['facing_target'], 'true')
        self.assertGreaterEqual(result['task_runtime']['budget']['observations_used'], 3)

    def test_unknown_front_gets_an_actual_new_view_then_resumes_same_stage_and_target(self):
        bridge, result, app, _ = self.run_goal(missing=True)
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertTrue(any(e['event'] == 'object_frame_observation_required' for e in result['task_runtime_events']))
        self.assertEqual(bridge.goal_executions, 1)
        runtime = app.audited_client.task_runtime;owner = next(iter(runtime.sessions))
        self.assertEqual(len(runtime.skills.search.semantic_maps.maps), 1)
        self.assertEqual(set(result['task_runtime']['target_memory']), {'van'})
        self.assertEqual(sum(e['steps'][0]['action']=='takeoff' for e in bridge.flight_requests if e['path']=='/mission'), 1)


if __name__ == '__main__':unittest.main()
