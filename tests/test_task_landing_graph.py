"""End-to-end parent/geometry/landing-authority tests with synthetic transports only."""
import copy
import hashlib
import math
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from goal_landing_authority import GoalLandingAuthority
from search_tools import coordinate_epoch
from test_single_vlm import tool_call
from test_search_graph_integration import LocalStereoBridge
from test_task_goal_graph import GoalRuntimeBridge, plan, response_for
import test_task_runtime as runtime_fixtures


class FlatGoalBridge(GoalRuntimeBridge):
    def __init__(self, directory, reset_velocity=False):
        super().__init__()
        self.goal_land_executions = 0
        self.reset_velocity = reset_velocity
        self.world = Path(directory)/'flat-fixture.world'
        self.world.write_text('''<sdf version="1.6"><world name="controlled">
<model name="support"><static>true</static><link name="body"><collision name="c"><pose>0 0 -.05 0 0 0</pose>
<geometry><box><size>76 76 .1</size></box></geometry></collision></link></model>
<model name="obstacle"><static>true</static><pose>30.5 30.5 .5 0 0 0</pose><link name="body"><collision name="c">
<geometry><box><size>1 1 1</size></box></geometry></collision></link></model></world></sdf>''')
        digest = hashlib.sha256(self.world.read_bytes()).hexdigest()
        self.nav.world_sha256 = digest
        self.telemetry['scene'].update(world_sha256=digest, ground_plane={'known': True, 'z_m': 0., 'bounds_xy_m': [-38., -38., 38., 38.]},
                                       navigation={'method': 'static_3d'})
        self.landing_authority = GoalLandingAuthority()
        self.pair_times = {}

    def state(self):
        state = super().state()
        state['navigation']['target_landing_supported'] = True
        return state

    def capture_stereo(self):
        pair = super().capture_stereo()
        self.pair_times[pair['pair_id']] = time.monotonic()
        return pair

    def controller(self):
        state = self.state()
        def metadata(identifier, max_age_wall_s=None):
            if max_age_wall_s and time.monotonic()-self.pair_times[identifier] > max_age_wall_s:
                raise ValueError('expired fixture verification pair')
            pair = self.pairs[identifier]
            return {'stamp_sim_s': pair['stamp_sim_s'], 'body_pose_enu': copy.deepcopy(pair['body_pose_enu']),
                'coordinate_epoch': coordinate_epoch(self.telemetry), 'image_sha256': copy.deepcopy(pair['image_sha256']),
                'navigation_registration_id': self.nav.registration_id}
        q = SimpleNamespace(x=0., y=0., z=math.sin(self.actual_yaw/2), w=math.cos(self.actual_yaw/2))
        return SimpleNamespace(ready=lambda: None, ev_guard=SimpleNamespace(reason=lambda now: None,
            snapshot=lambda now: copy.deepcopy(self.telemetry['external_vision'])),
            static_navigation_enabled=lambda: True, navigation_active=True, navigation_map=self.nav,
            navigation_moving=False, navigation_gate_error=None, state_msg=SimpleNamespace(armed=state['armed']),
            landed=state['landed_state'], mission=self.telemetry['mission'], search_authority=lambda: self.authority,
            position=lambda: list(self.telemetry['position_enu_m']), pose=SimpleNamespace(pose=SimpleNamespace(orientation=q)),
            pose_time=time.monotonic(), stereo_reports=SimpleNamespace(observation_metadata=metadata),
            world_path=self.world, scene=self.telemetry['scene'], validate_scene_steps=lambda steps: None,
            submit=self.submit_landing)

    def submit_landing(self, steps, description, parent_mission_id=None, landing_authorization=None):
        point = list(self.telemetry['position_enu_m'])
        accepted = LocalStereoBridge.request(self, '/mission', {'steps': steps, 'description': description})
        self.telemetry['position_enu_m'] = [point[0], point[1], .05]
        self.telemetry['mission']['parent_mission_id'] = parent_mission_id
        self.telemetry['mission']['target_landing_evidence'] = copy.deepcopy(landing_authorization)
        self.goal_land_executions += 1
        return {'mission': copy.deepcopy(self.telemetry['mission'])}

    def request(self, path, payload=None, **kwargs):
        if path == '/navigation/authorize-goal-land':
            return self.landing_authority.authorize(self.controller(), **payload)
        if path == '/navigation/execute-goal-land':
            return self.landing_authority.execute(self.controller(), **payload)
        return super().request(path, payload, **kwargs)

    def wait_for(self, accepted, on_progress=None):
        result = super().wait_for(accepted, on_progress)
        if self.goal_land_executions and accepted['mission']['steps'] == [{'action': 'land'}]:
            self.authority.clear()
            if self.reset_velocity:
                self.telemetry['external_vision']['healthy'] = False
                counters = self.telemetry['external_vision']['ekf']['local_velocity_reset_counters']
                counters['vz'] += 1
                self.telemetry['external_vision']['reason'] = 'controlled actual velocity reset'
        return self.state()


class TaskLandingGraphTests(unittest.TestCase):
    def run_landing(self, reset=False):
        directory = tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        bridge = FlatGoalBridge(directory.name, reset)
        runner = runtime_fixtures.RuntimeGraphTests()
        script = [tool_call('define_task_plan', {'plan': plan(landing=True)})]+[response_for(bridge)]*24
        result, app, model = runner.run_graph(script, '找到白色货车后降落在货车前方', bridge)
        return bridge, result, app

    def test_positive_parent_goal_lands_at_verified_front_area_without_substituting_home(self):
        bridge, result, app = self.run_landing()
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertTrue(result['task_outcome']['user_goal_completed'])
        self.assertEqual(bridge.goal_executions, 1)
        self.assertEqual(bridge.goal_land_executions, 1)
        self.assertFalse(bridge.telemetry['armed'])
        self.assertEqual(bridge.telemetry['landed_state'], 1)
        self.assertLess(bridge.telemetry['position_enu_m'][1], -3.)
        self.assertEqual(result['task_runtime_result']['flight_status'], 'grounded_disarmed')
        self.assertEqual(result['task_runtime_result']['unmet_requirements'], [])
        self.assertTrue(result['task_runtime']['plan']['stages'][-1]['conditions']['landing_support_and_corridor'])
        self.assertIsNone(result['safety_landing'])
        self.assertFalse(any(s.get('action') == 'return_home' for e in bridge.flight_requests
                             for s in e.get('steps', [])))

    def test_landed_velocity_reset_preserves_actual_ground_but_cannot_complete_relative_goal(self):
        bridge, result, app = self.run_landing(reset=True)
        self.assertEqual(bridge.goal_land_executions, 1)
        self.assertFalse(bridge.telemetry['armed'])
        self.assertFalse(result['task_outcome']['user_goal_completed'])
        self.assertEqual(result['task_runtime_result']['goal_status'], 'partial')
        self.assertEqual(result['task_runtime_result']['flight_status'], 'grounded_disarmed')
        facts = result['task_runtime_result']['evidence_summary']['relative_landing']
        self.assertTrue(facts['actual_ground_disarmed'])
        self.assertFalse(facts['navigation_reference_continuous'])
        self.assertFalse(facts['relation_verified'])
        self.assertEqual(result['task_runtime']['target_memory']['van']['map_status'], 'invalid')


if __name__ == '__main__':unittest.main()
