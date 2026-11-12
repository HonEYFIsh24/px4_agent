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
                probe_points_enu_m=payload.get('probe_points_enu_m'), allow_yaw=True,
                probe_covariances_enu_m2=payload.get('probe_covariances_enu_m2'))
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
                probe_points_enu_m=payload['probe_points_enu_m'], allow_yaw=True,
                probe_covariances_enu_m2=payload.get('probe_covariances_enu_m2'))
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
        if next_tool == 'choose_task_goal':
            goal = state['goal_state']
            decision = 'observe' if unknown[0] else 'here'
            unknown[0] = False
            return tool_call('choose_task_goal', {'decision': decision,
                'description': 'controlled visual goal decision', 'ground_point_uv': []})
        if next_tool == 'review_task_goal':
            return tool_call('review_task_goal', {'status': 'satisfied', 'same_target': True, 'description': 'controlled fresh visual relation confirmed'})
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
    def test_rejected_pose_returns_to_model_choice_before_any_flight(self):
        from task_runtime.execution import GoalExecution
        original = GoalExecution.prepare;calls = [0]
        def first_rejected(executor, *args, **kwargs):
            calls[0] += 1
            if calls[0] == 1:return {'status': 'no_safe_goal', 'candidates': []}
            return original(executor, *args, **kwargs)
        with mock.patch.object(GoalExecution, 'prepare', first_rejected):
            bridge, result, _, _ = self.run_goal()
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(calls[0], 2)
        self.assertEqual(bridge.goal_executions, 1)
        self.assertEqual(len(result['perceptions']), 2)

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
        self.assertEqual(bridge.telemetry['position_enu_m'][:2], [0., 0.])
        expected_yaw = 0.
        self.assertAlmostEqual(bridge.actual_yaw, expected_yaw)
        stage = result['task_runtime']['plan']['stages'][1]
        self.assertEqual(stage['conditions']['fresh_target_verification'], 'true')
        self.assertEqual(stage['conditions']['facing_target'], 'true')
        self.assertEqual(result['task_runtime']['budget']['observations_used'], 2)

    def test_unknown_front_gets_an_actual_new_view_then_resumes_same_stage_and_target(self):
        bridge, result, app, _ = self.run_goal(missing=True)
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertTrue(any(e['event'] == 'visual_goal_observation_requested' for e in result['task_runtime_events']))
        self.assertEqual(bridge.goal_executions, 1)
        runtime = app.audited_client.task_runtime;owner = next(iter(runtime.sessions))
        self.assertEqual(len(runtime.skills.search.semantic_maps.maps), 1)
        self.assertEqual(set(result['task_runtime']['target_memory']), {'van'})
        self.assertEqual(sum(e['steps'][0]['action']=='takeoff' for e in bridge.flight_requests if e['path']=='/mission'), 1)


if __name__ == '__main__':unittest.main()
