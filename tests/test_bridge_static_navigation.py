import threading
import unittest
from unittest import mock
from scene_config import load_scene, OBSTACLE_WORLD
from mission import MissionError
import test_bridge_navigation as nav_fixture


class StaticBridgeTests(unittest.TestCase):
    def setUp(self):
        self.fixture=nav_fixture.BridgeNavigationTests('test_auto_land_recovers_without_any_lidar_scan')
        # Reuse setup only, without running an inherited LiDAR test suite.
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.c=self.fixture.controller
        self.c.scene=load_scene(OBSTACLE_WORLD,navigation_mode='static_3d')
        self.c.world_path=OBSTACLE_WORLD

    def test_observation_geometry_uses_saved_capture_and_rejects_re_registration(self):
        from test_semantic_map import navigator, observation, EPOCH
        nav = navigator()
        frame, _, geometry = observation(nav)
        snapshot = frame['snapshot']
        self.c.navigation_map = nav
        self.c.stereo_reports = mock.Mock()
        self.c.stereo_reports.observation_metadata.return_value = {
            'navigation_registration_id': nav.registration_id,
            'body_pose_enu': snapshot['body_pose_enu'], 'coordinate_epoch': EPOCH,
            'camera_info_for_visibility': snapshot['camera_intrinsics'],
            'stamp_sim_s': snapshot['stamp_sim_s'], 'image_sha256': snapshot['stereo_image_sha256']}
        self.c.ev_guard = mock.Mock()
        self.c.ev_guard.snapshot.return_value = {'adapter': {'epoch': 0},
            'ekf': {'local_position_reset_counters': EPOCH['ekf_counters']}}
        result = self.c.observation_geometry(snapshot['stereo_pair_id'], [0., 0.])
        current_geometry = nav.observation_visibility(snapshot['body_pose_enu'],snapshot['camera_intrinsics'],
            [0.,0.],radius=30.,resolution=1.)
        self.assertEqual(result['visible_cell_ids'], current_geometry['visible_cell_ids'])
        self.assertEqual(result['grid']['rows'],60)
        self.assertEqual(result['body_pose_enu'], snapshot['body_pose_enu'])
        self.assertIsNone(self.c.worker)
        nav.registration_id = 'another-registration'
        with self.assertRaisesRegex(MissionError, 'another sortie'):
            self.c.observation_geometry(snapshot['stereo_pair_id'], [0., 0.])
        self.c.ev_guard.snapshot.return_value['adapter']['epoch'] = 1
        with self.assertRaises(MissionError):
            self.c.observation_geometry(snapshot['stereo_pair_id'], [0., 0.])

    def test_hold_preserves_current_height_in_variable_height_mode(self):
        self.fixture.set_pose([0.,0.,8.])
        self.c.navigation_hold()
        self.assertEqual(self.c.target,[0.,0.,8.])

    def prepare_viewpoint_capture(self):
        from test_semantic_map import navigator, observation, EPOCH
        from viewpoint_planner import map_identity
        from semantic_map import SemanticMap
        nav = navigator()
        frame, _, geometry = observation(nav)
        self.fixture.set_pose([0., 0., 3.])
        self.c.navigation_map = nav
        self.c.navigation_active = True
        self.c.state_msg.armed, self.c.landed = True, 2
        self.c.ev_guard = mock.Mock()
        self.c.ev_guard.reason.return_value = None
        self.c.ev_guard.snapshot.return_value = {'adapter': {'epoch': 0},
            'ekf': {'local_position_reset_counters': EPOCH['ekf_counters']}}
        self.c.stereo_reports = mock.Mock()
        self.c.stereo_reports.observation_metadata.return_value = {
            'navigation_registration_id': nav.registration_id, 'coordinate_epoch': EPOCH,
            'stamp_sim_s': 1., 'body_pose_enu': frame['snapshot']['body_pose_enu'],
            'camera_info_for_visibility': frame['snapshot']['camera_intrinsics']}
        return nav, map_identity(SemanticMap(('test', 'turn'), geometry))

    def owned_viewpoint(self):
        nav, identity = self.prepare_viewpoint_capture()
        self.c.home = [0., 0., 0.]
        context = {'owner': ['thread', 'task'], 'map_revision': 'b'*64, 'batch_id': 'a'*32}
        self.c.evaluate_viewpoints('context', 'pair', [0., 0.], identity, [], context)
        result = self.c.evaluate_viewpoints('paths', 'pair', [0., 0.], identity,
            [{'id': 'view_001', 'position_enu_m': [1., 0., 4.]}], context)
        return nav, context, result['results'][0]['execution_ticket']

    def prepare_rotation(self):
        nav, _ = self.prepare_viewpoint_capture()
        self.c.state_msg.armed = True
        self.c.state_msg.mode = 'OFFBOARD'
        self.c.landed = 2
        self.c.ev_guard.required = True
        self.c.ev_guard.reason.return_value = None
        self.c.ev_guard.snapshot.return_value['ekf']['fusion_active'] = True
        return nav

    def test_internal_rotation_holds_actual_position_and_checks_real_pose(self):
        self.prepare_rotation()
        initial = self.c.position()
        def exercise(policy, **callbacks):
            callbacks['checkpoint']()
            actual = callbacks['read_pose']()
            self.assertAlmostEqual(actual['actual_rad'], 0.)
            self.assertEqual(actual['position'], initial)
            callbacks['set_heading'](.2)
            self.assertEqual(self.c.target, initial)
            self.assertEqual(self.c.commanded, initial)
            # No synthetic arrival: return only a fixture outcome.
            return {'status': 'fixture', 'stamp_s': 10.}
        with mock.patch('yaw_control.run_rotation', side_effect=exercise):
            self.assertEqual(self.c.rotate_heading(.3)['status'], 'fixture')
        self.assertAlmostEqual(self.c.yaw, .2)
        self.assertEqual(self.c.yaw_capture_barrier, 10.)

    def test_ticket_rotation_preserves_goal_instead_of_accumulating_height_error(self):
        self.prepare_rotation()
        self.fixture.set_pose([0., 0., 2.85])
        self.c.mission = dict(search_constraint={'fixture': True}, viewpoint_candidate_id='view_fixture', step=2,
            steps=[{'action': 'goto', 'x': 0., 'y': 0., 'altitude': 3.}, {'action': 'rotate', 'yaw_rad': .3}])
        self.c.check_search_constraint = mock.Mock()  # Ownership is tested separately with real tickets.
        def exercise(policy, **callbacks):
            self.assertEqual(self.c.target, [0., 0., 3.])
            self.assertEqual(self.c.commanded, [0., 0., 2.85])
            self.assertEqual(policy.position, (0., 0., 3.))
            return {'status': 'fixture', 'stamp_s': 10.}
        with mock.patch('yaw_control.run_rotation', side_effect=exercise):
            self.c.rotate_heading(.3)
        self.fixture.set_pose([0., 0., 2.5])
        with self.assertRaisesRegex(MissionError, 'arrival'):
            self.c.rotate_heading(.3)

    def test_rotation_failure_freezes_position_and_actual_heading(self):
        self.prepare_rotation()
        def fail(policy, **callbacks):
            callbacks['set_heading'](.5)
            raise ValueError('controlled stale sample')
        with mock.patch('yaw_control.run_rotation', side_effect=fail):
            with self.assertRaisesRegex(ValueError, 'stale'):
                self.c.rotate_heading(1.)
        self.assertAlmostEqual(self.c.yaw, 0.)
        self.assertEqual(self.c.target, self.c.position())

    def test_rotation_rejects_inactive_fusion_before_heading_change(self):
        self.prepare_rotation()
        self.c.ev_guard.snapshot.return_value['ekf']['fusion_active'] = False
        with self.assertRaisesRegex(MissionError, 'fusion'):
            self.c.rotate_heading(1.)
        self.assertEqual(self.c.yaw, 0.)

    def test_rotation_pose_detects_epoch_change_during_operation(self):
        self.prepare_rotation()
        def reset(policy, **callbacks):
            self.c.ev_guard.snapshot.return_value['adapter']['epoch'] = 1
            with self.assertRaisesRegex(ValueError, 'identity'):
                pose = callbacks['read_pose']()
                policy.observe(**pose, wall_s=self.fixture.fixture.wall_time)
            raise ValueError('coordinate reset')
        with mock.patch('yaw_control.run_rotation', side_effect=reset):
            with self.assertRaisesRegex(ValueError, 'coordinate reset'):
                self.c.rotate_heading(.5)
        self.assertEqual(self.c.target, self.c.position())

    def test_rotation_requires_offboard_before_heading_change(self):
        self.prepare_rotation()
        self.c.state_msg.mode = 'POSCTL'
        with self.assertRaisesRegex(MissionError, 'OFFBOARD'):
            self.c.rotate_heading(.5)
        self.assertEqual(self.c.yaw, 0.)

    def test_rotation_action_requires_opt_in_and_standalone_step(self):
        self.prepare_rotation()
        with mock.patch.dict('os.environ', {'UAV_ENABLE_YAW': '0'}):
            with self.assertRaisesRegex(MissionError, 'experimental'):
                self.c.submit([{'action': 'rotate', 'yaw_rad': .2}])
        with mock.patch.dict('os.environ', {'UAV_ENABLE_YAW': '1'}):
            with self.assertRaisesRegex(MissionError, 'standalone'):
                self.c.submit([{'action': 'rotate', 'yaw_rad': .2}, {'action': 'land'}])
        self.assertIsNone(self.c.worker)

    def test_active_nbv_search_cannot_use_unscoped_rotation(self):
        self.owned_viewpoint()
        with mock.patch.dict('os.environ', {'UAV_ENABLE_YAW': '1'}), \
             mock.patch.object(self.c, 'validate_scene_steps'):
            with self.assertRaisesRegex(MissionError, 'active NBV search'):
                self.c.submit([{'action': 'rotate', 'yaw_rad': .2}])
        self.assertIsNone(self.c.worker)

    def test_owned_heading_ticket_schedules_move_then_exact_rotation_and_blocks_capture(self):
        import math
        nav, identity = self.prepare_viewpoint_capture()
        self.c.home = [0., 0., 0.]
        context = {'owner': ['thread', 'task'], 'map_revision': 'b'*64, 'batch_id': 'a'*32}
        with mock.patch.dict('os.environ', {'UAV_ENABLE_YAW': '1', 'UAV_ENABLE_NBV_YAW': '1'}), \
             mock.patch.object(self.c, 'validate_scene_steps'), \
             mock.patch.object(nav_fixture.bridge.threading, 'Thread'):
            self.c.evaluate_viewpoints('context', 'pair', [0., 0.], identity, [], context)
            result = self.c.evaluate_viewpoints('paths', 'pair', [0., 0.], identity,
                [{'id': 'view_001', 'position_enu_m': [1., 0., 4.], 'heading_step': 1}], context)
            ticket = result['results'][0]['execution_ticket']
            mission = self.c.execute_viewpoint(context['owner'], context['batch_id'], context['map_revision'],
                                               'view_001', ticket)['mission']
            self.assertEqual([s['action'] for s in mission['steps']], ['goto', 'rotate'])
            self.assertAlmostEqual(mission['steps'][1]['yaw_rad'], math.pi/4)
            self.assertEqual(self.c.yaw_capture_barrier, float('inf'))
            with self.assertRaises(MissionError):
                self.c.execute_viewpoint(context['owner'], context['batch_id'], context['map_revision'], 'view_001', ticket)

    def test_heading_cannot_be_forged_through_regular_search_mission(self):
        self.owned_viewpoint()
        with mock.patch.dict('os.environ', {'UAV_ENABLE_YAW': '1', 'UAV_ENABLE_NBV_YAW': '1'}), \
             mock.patch.object(self.c, 'validate_scene_steps'):
            with self.assertRaises(MissionError):
                self.c.submit([{'action': 'goto', 'x': 1., 'y': 0., 'altitude': 4.},
                               {'action': 'rotate', 'yaw_rad': 2.}])

    def test_rotation_mission_clears_barrier_only_with_actual_arrival_evidence(self):
        self.prepare_rotation()
        def reached(target):
            self.c.yaw_capture_barrier = 12.5
            return {'status': 'reached', 'target_rad': target, 'actual_rad': .19,
                    'stamp_s': 12.5, 'elapsed_s': 1.2}
        with mock.patch.dict('os.environ', {'UAV_ENABLE_YAW': '1'}), \
             mock.patch.object(self.c, 'validate_scene_steps'), \
             mock.patch.object(nav_fixture.bridge.threading, 'Thread'), \
             mock.patch.object(self.c, 'rotate_heading', side_effect=reached):
            self.c.submit([{'action': 'rotate', 'yaw_rad': .2}])
            self.assertEqual(self.c.yaw_capture_barrier, float('inf'))
            self.c.run_mission()
        self.assertEqual(self.c.mission['status'], 'succeeded')
        self.assertEqual(self.c.yaw_capture_barrier, 12.5)
        self.assertEqual(self.c.mission['events'][0]['rotation']['actual_rad'], .19)

    def test_failed_rotation_keeps_image_barrier_closed(self):
        self.prepare_rotation()
        with mock.patch.dict('os.environ', {'UAV_ENABLE_YAW': '1'}), \
             mock.patch.object(self.c, 'validate_scene_steps'), \
             mock.patch.object(nav_fixture.bridge.threading, 'Thread'), \
             mock.patch.object(nav_fixture.bridge.rospy, 'logerr', create=True), \
             mock.patch.object(self.c, 'rotate_heading', side_effect=ValueError('pose stale')):
            self.c.submit([{'action': 'rotate', 'yaw_rad': .2}])
            self.c.run_mission()
        self.assertEqual(self.c.mission['status'], 'failed')
        self.assertEqual(self.c.yaw_capture_barrier, float('inf'))

    def test_scoped_stop_cannot_cancel_a_different_mission(self):
        self.prepare_rotation()
        self.c.mission = {'id': 'current', 'status': 'running'}
        self.c.target = self.c.position()
        with self.assertRaisesRegex(MissionError, 'another flight'):
            self.c.stop('earlier')
        self.assertFalse(self.c.cancel.is_set())
        with mock.patch.object(self.c, 'snapshot', return_value={'ready': True}):
            result = self.c.stop('current')
        self.assertTrue(self.c.cancel.is_set())
        self.assertTrue(result['holding'])

    def test_confirmation_probes_use_raw_occlusion_without_changing_navigation(self):
        nav, identity = self.prepare_viewpoint_capture()
        nav.raw_boxes = [([2.9, -1., 0.], [3.6, 1., 4.])]
        nav.last_plan = {'executing': 'keep'}
        result = self.c.evaluate_viewpoints('visibility', 'pair', [0., 0.], identity,
            [{'id': 'view_001', 'position_enu_m': [0., 0., 3.]}],
            probe_points_enu_m=[[5., 0., 1.], [5., 4., 1.]])
        probes = result['results'][0]['probe_visibility']
        self.assertFalse(probes[0]['unoccluded'])
        self.assertTrue(probes[1]['unoccluded'])
        self.assertEqual(nav.last_plan, {'executing': 'keep'})
        self.assertIsNone(self.c.worker)

    def test_owned_ticket_submits_cached_goal_and_actual_planning_retains_scope(self):
        nav, context, ticket = self.owned_viewpoint()
        with mock.patch.object(nav_fixture.bridge.threading, 'Thread'):
            result = self.c.execute_viewpoint(context['owner'], context['batch_id'], context['map_revision'],
                                             'view_001', ticket)
        self.assertEqual(result['mission']['steps'], [{'action': 'goto', 'x': 1., 'y': 0., 'altitude': 4.}])
        self.assertEqual(result['mission']['search_constraint']['origin_xy_enu_m'], [0., 0.])
        self.c.checkpoint = mock.Mock()
        self.c.wait_until = mock.Mock()
        with mock.patch.object(nav, 'plan', wraps=nav.plan) as planner:
            self.c.move_static([1., 0., 4.])
        self.assertEqual(planner.call_args.kwargs['search_origin_xy'], [0., 0.])
        self.assertEqual(planner.call_args.kwargs['search_radius_m'], 30.)
        self.assertTrue(self.c.mission['navigation_plans'][0]['plan']['full_route_within_search_volume'])

    def test_active_nbv_search_cannot_bypass_ticket_with_generic_goto(self):
        self.owned_viewpoint()
        with self.assertRaisesRegex(MissionError, 'candidate ticket'):
            self.c.submit([{'action': 'goto', 'x': 1., 'y': 0., 'altitude': 4.}])
        self.assertIsNone(self.c.worker)

    def test_revoked_search_still_allows_independent_landing(self):
        self.owned_viewpoint()
        self.c.search_authority().revoke()
        self.c.landing_ready = mock.Mock()
        self.c.landing_snapshot = mock.Mock(return_value={'fresh': True})
        with mock.patch.object(nav_fixture.bridge.threading, 'Thread'):
            result = self.c.submit([{'action': 'land'}])
        self.assertNotIn('search_constraint', result['mission'])

    def test_executor_checks_scope_epoch_again_before_any_movement(self):
        _, context, ticket = self.owned_viewpoint()
        with mock.patch.object(nav_fixture.bridge.threading, 'Thread'):
            self.c.execute_viewpoint(context['owner'], context['batch_id'], context['map_revision'], 'view_001', ticket)
        self.c.ev_guard.snapshot.return_value['adapter']['epoch'] += 1
        with self.assertRaisesRegex(MissionError, 'identity or task ownership changed'):
            self.c.checkpoint()

    def test_viewpoint_query_is_read_only_and_preserves_the_executing_plan(self):
        nav, identity = self.prepare_viewpoint_capture()
        nav.last_plan = {'executing': 'keep'}
        result = self.c.evaluate_viewpoints('paths', 'pair', [0., 0.], identity,
            [{'id': 'view_001', 'position_enu_m': [2., 0., 5.]}])
        self.assertTrue(result['prediction_only'])
        self.assertTrue(result['results'][0]['reachable'])
        self.assertEqual(result['fixed_yaw_enu_rad'], self.c.yaw)
        self.assertEqual(nav.last_plan, {'executing': 'keep'})
        self.assertIsNone(self.c.worker)
        self.assertIsNone(self.c.mission)

    def test_live_scene_boundary_rejects_candidate_before_ticket_but_keeps_alternative(self):
        nav, identity = self.prepare_viewpoint_capture()
        self.c.home = [0., 0., 0.]
        self.c.scene['flight_region'] = {'bounds_min_m': [-1., -5., -.2],
            'bounds_max_m': [5., 5., 20.6], 'margin_m': .6}
        # Registration says x=-.3 is inside; live world/ENU offset says -.5.
        # The current aircraft remains safely inside at world x=-.2.
        self.c.gazebo_history[-1]['position_m'][0] = -.2
        context = {'owner': ['thread','task'], 'map_revision': 'b'*64, 'batch_id': 'a'*32}
        self.c.evaluate_viewpoints('context', 'pair', [0.,0.], identity, [], context)
        nav.last_plan = {'preserve': True}
        result = self.c.evaluate_viewpoints('paths', 'pair', [0.,0.], identity,
            [{'id':'view_001','position_enu_m':[-.3,0.,3.]},
             {'id':'view_002','position_enu_m':[1.,0.,3.]}], context)
        refused, allowed = result['results']
        self.assertIs(refused['reachable'], False)
        self.assertEqual(refused['scene_precheck'], 'candidate_rejected')
        self.assertNotIn('execution_ticket', refused)
        self.assertIs(allowed['reachable'], True)
        self.assertEqual(allowed['scene_precheck'], 'passed')
        self.assertIn('execution_ticket', allowed)
        self.assertEqual(len(self.c.search_authority().tickets), 1)
        self.assertEqual(nav.last_plan, {'preserve': True})
        self.assertIsNone(self.c.worker)
        self.assertIsNone(self.c.mission)

    def test_scene_precheck_checks_intermediate_path_points_not_only_goal(self):
        from mission import PathUnavailable
        nav, identity = self.prepare_viewpoint_capture()
        plan = nav.preview_path(self.c.position(), [1.,0.,4.], [0.,0.])
        plan['waypoints'] = [[-.5,0.,4.], [1.,0.,4.]]
        def check(steps):
            self.assertEqual(steps[0]['x'], -.5)
            raise PathUnavailable('intermediate point outside live region')
        with mock.patch.object(nav, 'preview_path', return_value=plan), \
                mock.patch.object(self.c, 'validate_scene_steps', side_effect=check):
            result = self.c.evaluate_viewpoints('paths', 'pair', [0.,0.], identity,
                [{'id':'view_001','position_enu_m':[1.,0.,4.]}])
        self.assertIs(result['results'][0]['reachable'], False)

    def test_stale_world_pose_or_invalid_start_aborts_batch_without_tickets(self):
        for fault in ('stale', 'invalid_start'):
            with self.subTest(fault=fault):
                self.c._search_authority = None
                nav, identity = self.prepare_viewpoint_capture()
                context = {'owner': ['thread','task'], 'map_revision': 'b'*64, 'batch_id': 'a'*32}
                self.c.evaluate_viewpoints('context', 'pair', [0.,0.], identity, [], context)
                if fault == 'stale': self.c.gazebo_time = 998.
                else: self.c.gazebo_history[-1]['position_m'][0] = -100.
                with self.assertRaises(MissionError):
                    self.c.evaluate_viewpoints('paths', 'pair', [0.,0.], identity,
                        [{'id':'view_001','position_enu_m':[1.,0.,4.]}], context)
                self.assertEqual(self.c.search_authority().tickets, {})
                self.assertIsNone(self.c.worker)

    def test_path_precheck_acquires_next_actual_aligned_callback_without_flight(self):
        nav, identity = self.prepare_viewpoint_capture()
        self.c.home = [0., 0., 0.]
        context = {'owner': ['thread','task'], 'map_revision': 'b'*64, 'batch_id': 'a'*32}
        self.c.evaluate_viewpoints('context', 'pair', [0.,0.], identity, [], context)
        self.c.gazebo_history[-1]['stamp_sim_s'] = 9.92  # Actual failure: 80 ms offset.
        nav.last_plan = {'executing': 'preserve'}
        acquired = threading.Event()
        def next_callback():
            with self.c.lock:
                self.c.gazebo_history.append({**self.c.gazebo_history[-1], 'stamp_sim_s': 10.})
                self.c.gazebo_time = self.fixture.fixture.wall_time
                acquired.set()
        callback = threading.Thread(target=next_callback, daemon=True)
        # Nested ownership verifies Condition releases the whole RLock, so ROS
        # callbacks and operator actions can still run during acquisition.
        with self.c.lock:
            callback.start()
            result = self.c.evaluate_viewpoints('paths', 'pair', [0.,0.], identity,
                [{'id':'view_001','position_enu_m':[1.,0.,4.]}], context)
        callback.join(timeout=1)
        self.assertTrue(acquired.is_set())
        self.assertIn('execution_ticket', result['results'][0])
        self.assertEqual(nav.last_plan, {'executing': 'preserve'})
        self.assertIsNone(self.c.worker)
        self.assertIsNone(self.c.mission)

    def test_alignment_timeout_and_stale_source_issue_no_tickets(self):
        for fault in ('offset', 'stale', 'missing'):
            with self.subTest(fault=fault):
                self.c._search_authority = None
                _, identity = self.prepare_viewpoint_capture()
                context = {'owner': ['thread','task'], 'map_revision': 'b'*64, 'batch_id': 'a'*32}
                self.c.evaluate_viewpoints('context', 'pair', [0.,0.], identity, [], context)
                self.c.gazebo_history[-1]['stamp_sim_s'] = 9.92
                if fault == 'stale': self.c.gazebo_time = self.fixture.fixture.wall_time-2
                if fault == 'missing': self.c.gazebo_history.clear()
                began = self.fixture.fixture.wall_time
                def elapsed(seconds): self.fixture.fixture.wall_time += seconds
                with mock.patch.object(nav_fixture.bridge.threading, 'Condition') as condition:
                    condition.return_value.wait.side_effect = elapsed
                    with self.assertRaises(MissionError):
                        self.c.evaluate_viewpoints('paths', 'pair', [0.,0.], identity,
                            [{'id':'view_001','position_enu_m':[1.,0.,4.]}], context)
                    if fault == 'offset':
                        self.assertGreaterEqual(self.fixture.fixture.wall_time-began, .25)
                        self.assertLessEqual(self.fixture.fixture.wall_time-began, .251)
                    else:
                        condition.return_value.wait.assert_not_called()
                self.assertEqual(self.c.search_authority().tickets, {})
                self.assertIsNone(self.c.worker)
                self.assertIsNone(self.c.mission)

    def test_alignment_wait_rechecks_epoch_owner_mission_registration_and_drift(self):
        for fault in ('epoch', 'operator_stop', 'mission_running', 'registration', 'drift'):
            with self.subTest(fault=fault):
                self.c.mission = None
                self.c._search_authority = None
                nav, identity = self.prepare_viewpoint_capture()
                context = {'owner': ['thread','task'], 'map_revision': 'b'*64, 'batch_id': 'a'*32}
                self.c.evaluate_viewpoints('context', 'pair', [0.,0.], identity, [], context)
                self.c.gazebo_history[-1]['stamp_sim_s'] = 9.92
                def changed(_):
                    if fault == 'epoch': self.c.ev_guard.snapshot.return_value['adapter']['epoch'] = 1
                    elif fault == 'operator_stop': self.c.search_authority().revoke()
                    elif fault == 'mission_running': self.c.mission = {'status': 'running'}
                    elif fault == 'registration': nav.registration_id = 'changed'
                    else: self.fixture.set_pose([.5,0.,3.])
                with mock.patch.object(nav_fixture.bridge.threading, 'Condition') as condition:
                    condition.return_value.wait.side_effect = changed
                    with self.assertRaises(MissionError):
                        self.c.evaluate_viewpoints('paths', 'pair', [0.,0.], identity,
                            [{'id':'view_001','position_enu_m':[1.,0.,4.]}], context)
                    condition.return_value.wait.assert_called_once()
                self.assertEqual(self.c.search_authority().tickets, {})
                self.assertIsNone(self.c.worker)

    def test_final_execution_still_rejects_time_offset_after_valid_ticket(self):
        _, context, ticket = self.owned_viewpoint()
        self.c.gazebo_history[-1]['stamp_sim_s'] -= .08
        with self.assertRaisesRegex(MissionError, 'time aligned'):
            self.c.execute_viewpoint(context['owner'], context['batch_id'], context['map_revision'], 'view_001', ticket)
        self.assertIsNone(self.c.worker)
        self.assertIsNone(self.c.mission)

    def test_scene_can_still_reject_changed_conditions_at_final_submission(self):
        _, context, ticket = self.owned_viewpoint()
        self.c.gazebo_history[-1]['position_m'][0] = -100.
        with self.assertRaises(MissionError):
            self.c.execute_viewpoint(context['owner'], context['batch_id'], context['map_revision'], 'view_001', ticket)
        self.assertIsNone(self.c.worker)
        self.assertIsNone(self.c.mission)

    def test_viewpoint_geometry_does_not_hold_publisher_lock_and_rejects_mid_query_reset(self):
        nav, identity = self.prepare_viewpoint_capture()
        original = nav.observation_visibility
        def query(*args, **kwargs):
            self.assertFalse(self.c.lock._is_owned())
            result = original(*args, **kwargs)
            self.c.ev_guard.snapshot.return_value['adapter']['epoch'] = 1
            return result
        with mock.patch.object(nav, 'observation_visibility', side_effect=query):
            with self.assertRaisesRegex(MissionError, 'identity or capture epoch changed'):
                self.c.evaluate_viewpoints('visibility', 'pair', [0., 0.], identity,
                    [{'id': 'view_001', 'position_enu_m': [0., 2., 3.]}])
        self.assertIsNone(self.c.worker)

    def test_viewpoint_query_rejects_running_flight_or_guard_failure(self):
        _, identity = self.prepare_viewpoint_capture()
        self.c.mission = {'status': 'running'}
        with self.assertRaisesRegex(MissionError, 'stable airborne'):
            self.c.evaluate_viewpoints('context', 'pair', [0., 0.], identity, [])
        self.c.mission = None
        self.c.ev_guard.reason.return_value = 'coordinate reset'
        with self.assertRaisesRegex(MissionError, 'coordinate reset'):
            self.c.evaluate_viewpoints('context', 'pair', [0., 0.], identity, [])

    def test_horizontal_lidar_does_not_reimpose_old_height_gate(self):
        self.fixture.set_pose([0.,0.,8.])
        self.fixture.navigation_scan()
        self.c.navigation_map.update_scan.assert_not_called()
        self.assertIsNone(self.c.lidar_error)

    def test_occupied_goal_is_rejected_before_a_worker_is_created(self):
        self.c.state_msg.armed=True
        self.c.landed=2
        with self.assertRaises(MissionError):
            self.c.submit([{'action':'goto','x':4.5,'y':0.,'altitude':3.5}],'occupied goal')
        self.assertIsNone(self.c.worker)

    def test_static_telemetry_does_not_claim_lidar_mapping(self):
        self.c.navigation_map=None
        s=self.c.navigation_snapshot()
        self.assertEqual(s['method'],'static_3d')
        self.assertEqual(s['altitude_mode'],'variable')
        self.assertEqual(s['source'],'verified_static_collision_map_3d')
        self.assertIn('telemetry_only',s['lidar_role'])

    def test_preview_rejection_never_creates_flight_or_closes_navigation_gate(self):
        self.c.navigation_active = True
        self.c.navigation_map = mock.Mock()
        self.c.navigation_map.plan.side_effect = MissionError('occupied goal')
        result = self.c.preview_navigation([2., 0., 5.])
        self.assertFalse(result['reachable'])
        self.assertIsNone(self.c.worker)
        self.assertIsNone(self.c.navigation_gate_error)

    def test_preview_success_has_no_flight_side_effect_and_validates_coordinates(self):
        self.c.navigation_active = True
        self.c.navigation_map = mock.Mock()
        self.c.navigation_map.plan.return_value = {'path_length_m': 3.}
        self.assertTrue(self.c.preview_navigation([2., 0., 5.])['reachable'])
        self.assertIsNone(self.c.worker)
        for point in ([True, 0., 5.], [2., 0., float('nan')], [2., 0., 20.01]):
            with self.assertRaises((MissionError, ValueError)):
                self.c.preview_navigation(point)

    def test_active_sortie_registration_is_reused_for_preview_and_submission(self):
        from static_navigation import StaticNavigator
        navigator = StaticNavigator.from_scene(self.c.world_path, self.c.scene)
        pose = self.c.pose.pose
        q = pose.orientation
        body = self.c.gazebo_history[-1]
        navigator.bind(self.c.position(), [q.x, q.y, q.z, q.w], body)
        self.c.navigation_map = navigator
        self.c.navigation_active = True
        navigator.last_plan = {'executing': 'preserve during legacy preview'}
        with mock.patch.object(StaticNavigator, 'from_scene', side_effect=AssertionError('must not re-register mid-sortie')):
            self.c.validate_scene_steps([{'action': 'goto', 'x': 0., 'y': 0., 'altitude': 5.}])
            self.assertTrue(self.c.preview_navigation([0., 0., 5.])['reachable'])
        self.assertEqual(navigator.last_plan, {'executing': 'preserve during legacy preview'})

    def test_static_cruise_speeds_up_but_approach_and_tracking_lag_slow_down(self):
        self.fixture.set_pose([0., 0., 3.])
        self.c.target, self.c.commanded = [3., 0., 3.], [0., 0., 3.]
        self.assertAlmostEqual(self.c.navigation_speed_limit(), .65)
        self.c.commanded = [.6, 0., 3.]
        self.assertAlmostEqual(self.c.navigation_speed_limit(), .25)
        self.c.target, self.c.commanded = [.5, 0., 3.], [0., 0., 3.]
        self.assertAlmostEqual(self.c.navigation_speed_limit(), .25)

    def test_faster_publisher_keeps_euclidean_cap_and_freezes_on_closed_gate(self):
        import math
        self.fixture.set_pose([0., 0., 3.])
        self.c.navigation_moving = True
        self.c.target, self.c.commanded = [2., 2., 3.], [0., 0., 3.]
        self.c.navigation_map.segment_safe.return_value = True
        messages = self.fixture.run_publisher_iterations()
        previous = [0., 0., 3.]
        for message in messages:
            position = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
            self.assertAlmostEqual(math.dist(previous, position), .65*.05)
            previous = position
        self.c.navigation_map.segment_safe.return_value = False
        self.c.target, self.c.commanded = [2., 2., 3.], [0., 0., 3.]
        messages = self.fixture.run_publisher_iterations()
        self.assertEqual(self.c.target, [0., 0., 3.])
