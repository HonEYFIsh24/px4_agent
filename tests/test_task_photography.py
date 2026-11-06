"""Ordinary photography inside the parent lifecycle, explicit offline transports."""
import copy
import json
import unittest

from photo_navigation import relative_photo_goal, relative_photo_yaw, require_photo_navigation
from test_task_goal_graph import GoalRuntimeBridge
from test_search_graph_integration import LocalStereoBridge
from test_single_vlm import tool_call
from test_photo_task import review
import test_task_runtime as runtime_fixtures


class PhotoRuntimeBridge(GoalRuntimeBridge):
    def state(self):
        state = super().state()
        state['mode'] = 'OFFBOARD' if state['armed'] else 'AUTO.LAND'
        return state

    def request(self, path, payload=None, **kwargs):
        if path != '/navigation/photo-action':
            return super().request(path, payload, **kwargs)
        require_photo_navigation(self.state(), payload['expected_mission_id'], payload['expected_epoch'])
        if self.authority.active:
            raise AssertionError('ordinary photography cannot escape a bound NBV scope')
        pose = self.state()['body_pose_enu']
        if payload['operation'] == 'adjust':
            point, _ = relative_photo_goal(pose['position_m'], pose['orientation_xyzw'], payload['direction'],
                                           payload['distance_m'], payload['altitude'])
            self.nav.plan(pose['position_m'], point, record=False)
            steps = [{'action': 'goto', 'x': point[0], 'y': point[1], 'altitude': point[2]}]
            return LocalStereoBridge.request(self, '/mission', {'steps': steps, 'description': payload['description']})
        raise AssertionError('unscripted photography operation')


def plan(subject='白色货车'):
    return {'targets': [{'ref': 'subject', 'label': subject}], 'stages': [
        {'kind': 'photo', 'target_ref': 'subject', 'altitude_m': 3.},
        {'kind': 'report', 'target_ref': 'subject'}, {'kind': 'return_home_land'}]}


def report(messages):
    capture = json.loads(next(m for m in reversed(messages) if getattr(m, 'name', None) == 'observe_task_photo').content)
    return tool_call('report_scene', {'capture_id': capture['capture_id'], 'summary': 'controlled photography report',
        'targets': [], 'limitations': ['synthetic camera, not a physical image test']})


class TaskPhotographyTests(unittest.TestCase):
    def run_photo(self, script, prompt):
        bridge = PhotoRuntimeBridge()
        runner = runtime_fixtures.RuntimeGraphTests()
        result, app, _ = runner.run_graph(script, prompt, bridge)
        return bridge, result, app

    def test_photo_adjustment_new_capture_review_report_and_one_parent_finish(self):
        script = [tool_call('define_task_plan', {'plan': plan()}), report, review('no', 'yes', '向左侧换位'),
            tool_call('adjust_photo_view', {'direction': 'left', 'distance_m': 1., 'altitude': 3., 'reason': '改善构图'}),
            report, review()]
        bridge, result, app = self.run_photo(script, '拍摄白色货车的全貌照片，报告后返航降落')
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(result['photo_progress']['photo_status'], 'met')
        self.assertEqual(result['task_runtime']['budget']['observations_used'], 2)
        self.assertTrue(result['task_outcome']['user_goal_completed'])
        self.assertFalse(bridge.telemetry['armed'])
        runtime = app.audited_client.task_runtime;owner = next(iter(runtime.sessions))
        self.assertEqual(runtime.skills.search.sessions[owner]['status'], 'photo_completed')
        self.assertEqual(sum(e['steps'][0]['action']=='takeoff' for e in bridge.flight_requests if e['path']=='/mission'), 1)
        self.assertEqual(len(runtime.skills.search.semantic_maps.maps), 1)
        self.assertIsNone(result['safety_landing'])

    def test_unknown_area_extent_cannot_become_whole_orchard_success_even_if_model_says_complete(self):
        bridge, result, app = self.run_photo([tool_call('define_task_plan', {'plan': plan('果园')}), report, review()],
                                            '请为我拍摄一张俯瞰整个果园的照片，然后返航降落')
        self.assertEqual(result['status'], 'failed')
        self.assertFalse(result['task_outcome']['user_goal_completed'])
        self.assertEqual(result['task_runtime_result']['flight_status'], 'grounded_disarmed')
        self.assertEqual(result['task_runtime_result']['unmet_requirements'][0]['reason_code'], 'AREA_EXTENT_UNKNOWN')
        self.assertIsNone(result['safety_landing'])
        self.assertFalse(bridge.telemetry['armed'])


if __name__ == '__main__':unittest.main()
