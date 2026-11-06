"""Controlled target approach/retention; no ROS, real VLM or aircraft commands."""
import copy
import json
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from autonomous_search import AdaptiveSearch
from semantic_map import SemanticMap
from semantic_map_runtime import SemanticMapRuntime
from single_vlm import FrameStore
from target_confirmation import confirmation_goal
from target_observation import (observation_quality, surface_reference, relative_range_reference,
                                RANGE_POLICY, LEGACY_RANGE_POLICY)
from stereo_localization import (quaternion_rotation, transpose, matvec, iris_left_optical_to_body,
                                 triangulate_pixel, optical_to_enu)
from task_contract import assess_task, search_result
from viewpoint_geometry import StaticViewpointEvaluator
from viewpoint_planner import ViewpointConfig, ViewpointPlanner, ViewpointError
from test_nbv_search_integration import ScopedFixtureBridge
from test_semantic_map import navigator, observation, target, OWNER
import test_target_confirmation as confirmation_fixtures
from test_vision import CONFIG
import test_nbv_search_integration as graph_fixtures


def measured_stereo(frame, world_point):
    """Consistent synthetic stereo/rig geometry, not a real matching result."""
    snapshot = frame['snapshot']
    pose, mounting = snapshot['body_pose_enu'], iris_left_optical_to_body()
    body = matvec(transpose(quaternion_rotation(pose['orientation_xyzw'])),
                  [p-o for p, o in zip(world_point, pose['position_m'])])
    optical = matvec(transpose(mounting['rotation']),
                     [p-o for p, o in zip(body, mounting['translation_m'])])
    k = snapshot['camera_intrinsics']['K']
    calibration = dict(fx=k[0], fy=k[4], cx=k[2], cy=k[5], baseline_m=.12)
    u, v = k[0]*optical[0]/optical[2]+k[2], k[4]*optical[1]/optical[2]+k[5]
    disparity = k[0]*.12/optical[2]
    point, covariance = triangulate_pixel(u, v, disparity, calibration)
    enu, enu_covariance, assumed = optical_to_enu(point, covariance, pose)
    return {'status': 'ok', 'method': 'stereo_sgbm_robust_surface', 'frame': 'local_enu',
            'position_camera_optical_m': point, 'covariance_camera_optical_m2': covariance,
            'position_enu_m': enu, 'covariance_enu_m2': enu_covariance,
            'image_stamp_sim_s': snapshot['stamp_sim_s'], 'body_pose_stamp_sim_s': pose['stamp_sim_s'],
            'image_sha256': snapshot['stereo_image_sha256'], 'coordinate_epoch': snapshot['coordinate_epoch'],
            'calibration': calibration, 'pose_uncertainty_assumed': assumed,
            'uncertainty_statistics_calibrated': False,
            'quality': {'representative_pixel_uv': [u,v], 'sigma_disparity_px': .5,
                        'median_disparity_px': disparity, 'valid_ratio': .8,
                        'lr_consistent_ratio': .9, 'robust_inlier_pixels': 200}}


def relative_target(frame, point, *args, **kwargs):
    result = target(frame, point, *args, **kwargs)
    result['stereo_location'] = measured_stereo(frame, point)
    return result


class TargetObservationGeometryTests(unittest.TestCase):
    def setUp(self):
        self.nav = navigator()
        self.frame, report, geometry = observation(self.nav)
        report['targets'] = [target(self.frame, [10., 0., 1.], '蓝色油桶', [.4, .4, .6, .6])]
        self.map = SemanticMap(OWNER, geometry, query='蓝色油桶')
        self.map.update_observation(self.frame, report, geometry)
        self.landmark = next(iter(self.map.landmarks.values()))
        self.evidence = self.landmark.observations[0]
        self.goal = confirmation_goal(self.map, self.landmark.id, self.frame['capture_id'])
        snapshot = self.frame['snapshot']
        self.backend = StaticViewpointEvaluator(self.nav, snapshot['body_pose_enu'],
            snapshot['camera_intrinsics'], snapshot['coordinate_epoch'],
            probe_points_enu_m=self.goal['probe_points_enu_m'], allow_yaw=True)

    def test_range_reference_needs_real_metric_and_valid_uncertainty(self):
        self.assertIsNotNone(surface_reference(self.landmark, self.evidence))
        for covariance in (None, [[0.]*3]*3, [[-1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],
                           [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],
                           [[.04,.8,0.],[.8,.04,0.],[0.,0.,.04]]):
            with self.subTest(covariance=covariance):
                modified = dict(self.evidence, covariance_enu_m2=covariance)
                self.assertIsNone(surface_reference(self.landmark, modified))
        bearing = dict(self.evidence, position_enu_m=None)
        self.assertIsNone(surface_reference(self.landmark, bearing))

    def test_close_observation_uses_actual_range_box_and_surface_not_object_center(self):
        self.assertFalse(observation_quality(self.landmark, self.evidence)['completed'])
        frame, report, geometry = observation(self.nav, position=[6., 0., 3.])
        # The range is measured from this new camera, not manufactured by
        # subtracting a new observer from an old/fused world point.
        point = [10., 0., 1.]
        report['targets'] = [relative_target(frame, point, '蓝色油桶', [.35,.4,.65,.8])]
        semantic = SemanticMap(OWNER, geometry)
        semantic.update_observation(frame, report, geometry)
        landmark = next(iter(semantic.landmarks.values()))
        near = landmark.observations[0]
        result = observation_quality(self.landmark, near)
        self.assertTrue(result['completed'])
        self.assertAlmostEqual(result['range_m'], math.dist(near['bearing_origin_enu_m'], point))
        self.assertEqual(result['range_policy'], RANGE_POLICY)
        for box in ([.9,.4,1.,.6], [.4,.4,.45,.45]):
            self.assertFalse(observation_quality(self.landmark, dict(near, bbox=box))['completed'])
        # A changed bearing origin never changes the measured optical range.
        self.assertEqual(observation_quality(self.landmark, dict(near, bearing_origin_enu_m=[9.,0.,1.]))['range_m'], result['range_m'])

    def test_target_arc_sampling_stays_local_and_inside_original_search_permission(self):
        planner = ViewpointPlanner(ViewpointConfig(heading_mode='discrete'))
        planner.confirmation_goal = copy.deepcopy(self.goal)
        source = [5., 0., 3.]
        candidates = planner.generate_candidates({'position_m': source}, [0.,0.], 0.)
        arcs = [c for c in candidates if c.get('candidate_basis') == 'observed_surface_standoff_arc']
        self.assertTrue(arcs)
        self.assertTrue(all(.5 <= math.dist(c['position_enu_m'], source) <= 4. for c in arcs))
        self.assertTrue(all(math.hypot(*c['position_enu_m'][:2]) <= 12. for c in arcs))
        self.assertTrue(all(1.5 <= c['position_enu_m'][2] <= 19.5 for c in arcs))
        self.assertEqual(len(candidates), len({c['id'] for c in candidates}))
        bearing_goal = dict(self.goal, surface_reference=None)
        planner.confirmation_goal = bearing_goal
        self.assertFalse(any(c.get('candidate_basis') for c in planner.generate_candidates(
            {'position_m': source}, [0.,0.], 0.)))

    def test_approach_prefers_closer_visible_view_over_away_and_never_accepts_occlusion(self):
        class Candidates(ViewpointPlanner):
            def generate_candidates(self, *args, **kwargs):
                return [{'id': 'view_001', 'position_enu_m': [0., 2., 3.], 'direction': 'side', 'fixed_yaw_enu_rad': 0.},
                        {'id': 'view_002', 'position_enu_m': [4., 0., 3.], 'direction': 'toward target', 'fixed_yaw_enu_rad': 0.}]
        self.goal['purpose'] = 'approach'
        before = self.map.snapshot()
        result = Candidates().evaluate(self.map, self.backend, self.goal)
        self.assertEqual(result['selected_id'], 'view_002', result['ranked_candidates'])
        self.assertGreater(result['ranked_candidates'][0]['score_terms']['observation_range'], 0)
        self.assertEqual(self.map.snapshot(), before)
        original = self.backend.query
        def occlude(operation, *args):
            response = original(operation, *args)
            for c in response['results']:
                if c['id'] == 'view_002':
                    for probe in c.get('probe_visibility', []):
                        probe['unoccluded'] = False
            return response
        self.backend.query = occlude
        result = Candidates().evaluate(self.map, self.backend, self.goal)
        self.assertEqual(result['selected_id'], 'view_001')

    def test_range_probe_tampering_rejected_and_bearing_has_no_range_reward(self):
        bad = copy.deepcopy(self.goal)
        bad['surface_reference']['point_enu_m'] = [3., 0., 1.]
        with self.assertRaisesRegex(ViewpointError, 'metric evidence'):
            ViewpointPlanner().evaluate(self.map, self.backend, bad)
        goal = dict(self.goal, surface_reference=None, probe_basis='bearing_range_hypotheses_not_localization')
        result = ViewpointPlanner().evaluate(self.map, self.backend, goal)
        self.assertTrue(result['ranked_candidates'])
        self.assertTrue(all(c['score_terms']['observation_range'] == 0 for c in result['ranked_candidates']))


class RelativeRangeTests(unittest.TestCase):
    def evidence(self, depth=4.):
        nav = navigator()
        frame, report, geometry = observation(nav)
        pose = frame['snapshot']['body_pose_enu']
        pose.update(position_covariance_m2=[[.09,0.,0.],[0.,.09,0.],[0.,0.,.16]],
                    orientation_covariance_rad2=[[.008,0.,0.],[0.,.008,0.],[0.,0.,.069]],
                    pose_covariance_source='configured_conservative_floor')
        geometry['body_pose_enu'] = copy.deepcopy(pose)
        point = optical_to_enu([0.,0.,depth], [[.01,0.,0.],[0.,.01,0.],[0.,0.,.01]], pose)[0]
        report['targets'] = [relative_target(frame, point, '蓝色油桶', [.35,.25,.65,.75])]
        semantic = SemanticMap(OWNER, geometry)
        semantic.update_observation(frame, report, geometry)
        landmark = next(iter(semantic.landmarks.values()))
        return landmark, landmark.observations[0], semantic

    def test_world_pose_uncertainty_does_not_change_same_frame_range_or_authorize_world_arc(self):
        landmark, evidence, semantic = self.evidence()
        self.assertIsNone(surface_reference(landmark, evidence))
        result = observation_quality(landmark, evidence)
        self.assertTrue(result['completed'], result)
        self.assertAlmostEqual(result['range_m'], 4.)
        self.assertIsNone(result['surface_reference'])
        self.assertFalse(result['range_reference']['uncertainty_statistics_calibrated'])
        # Optical centre-line distance has derivative [0,0,1]. This is an
        # independent check of radial, rather than max-axis, propagation.
        matrix = evidence['relative_stereo']['measurement']['covariance_camera_optical_m2']
        self.assertAlmostEqual(result['range_reference']['radial_sigma_m'], math.sqrt(matrix[2][2]))
        goal = confirmation_goal(semantic, landmark.id, evidence['capture_id'])
        self.assertIsNone(goal['surface_reference'])
        planner = ViewpointPlanner()
        planner.confirmation_goal = goal
        self.assertFalse(any(c.get('candidate_basis') == 'observed_surface_standoff_arc'
            for c in planner.generate_candidates(evidence['body_pose_enu'], [0.,0.], 0.)))

    def test_rigid_world_transform_and_large_world_covariance_leave_relative_range_unchanged(self):
        landmark, evidence, _ = self.evidence()
        baseline = relative_range_reference(evidence)
        for yaw in (-math.pi+.01, .8, math.pi-.01):
            value = copy.deepcopy(evidence)
            pose = value['body_pose_enu']
            pose.update(position_m=[24.,-11.,4.], orientation_xyzw=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)],
                        orientation_covariance_rad2=[[1.,0.,0.],[0.,1.,0.],[0.,0.,2.]])
            metric = value['relative_stereo']['measurement']
            point, covariance, _ = optical_to_enu(metric['position_camera_optical_m'],
                metric['covariance_camera_optical_m2'], pose)
            value['position_enu_m'], value['covariance_enu_m2'] = point, covariance
            metric['position_enu_m'], metric['covariance_enu_m2'] = point, covariance
            current = relative_range_reference(value)
            self.assertAlmostEqual(current['range_m'], baseline['range_m'])
            self.assertAlmostEqual(current['radial_sigma_m'], baseline['radial_sigma_m'])
            self.assertIsNone(surface_reference(landmark, value))

    def test_invalid_or_different_measurement_cannot_satisfy_photo_quality(self):
        landmark, evidence, _ = self.evidence()
        mutations = {
            'capture': lambda e: e['relative_stereo'].update(capture_id='other'),
            'pair': lambda e: e['relative_stereo'].update(pair_id='other'),
            'box': lambda e: e['relative_stereo'].update(bbox=[.1,.1,.2,.2]),
            'epoch': lambda e: e['relative_stereo']['measurement'].update(coordinate_epoch={'vio_epoch': 99}),
            'hash': lambda e: e['relative_stereo']['measurement']['image_sha256'].update(right='f'*64),
            'old_image': lambda e: e['relative_stereo']['measurement'].update(image_stamp_sim_s=0.),
            'old_pose': lambda e: e['relative_stereo']['measurement'].update(body_pose_stamp_sim_s=0.),
            'method': lambda e: e['relative_stereo']['measurement'].update(method='model_guessed_range'),
            'missing_covariance': lambda e: e['relative_stereo']['measurement'].pop('covariance_camera_optical_m2'),
            'zero_covariance': lambda e: e['relative_stereo']['measurement'].update(covariance_camera_optical_m2=[[0.]*3]*3),
            'indefinite_covariance': lambda e: e['relative_stereo']['measurement'].update(covariance_camera_optical_m2=[[1.,2.,0.],[2.,1.,0.],[0.,0.,1.]]),
            'artificially_small_covariance': lambda e: e['relative_stereo']['measurement'].update(covariance_camera_optical_m2=[[1e-8,0.,0.],[0.,1e-8,0.],[0.,0.,1e-8]]),
            'small_disparity_sigma': lambda e: e['relative_stereo']['measurement']['quality'].update(sigma_disparity_px=.01),
            'weak_support': lambda e: e['relative_stereo']['measurement']['quality'].update(valid_ratio=.1),
            'calibration': lambda e: e['relative_stereo']['measurement']['calibration'].update(baseline_m=.2),
            'invalid_depth': lambda e: e['relative_stereo']['measurement'].update(position_camera_optical_m=[0.,0.,-4.]),
            'different_world_surface': lambda e: e.update(position_enu_m=[100.,100.,100.]),
            'missing_channel': lambda e: e.pop('relative_stereo'),
        }
        for name, mutate in mutations.items():
            with self.subTest(case=name):
                modified = copy.deepcopy(evidence)
                mutate(modified)
                quality = observation_quality(landmark, modified)
                self.assertFalse(quality['completed'], quality)
                self.assertFalse(quality['range_usable'])
                self.assertIsNotNone(quality['range_unavailable_reason'])

    def test_distance_interval_and_framing_still_reject_too_near_far_or_cropped(self):
        for depth in (1.5, 7.):
            landmark, evidence, _ = self.evidence(depth)
            self.assertFalse(observation_quality(landmark, evidence)['completed'])
        landmark, evidence, _ = self.evidence()
        for box in ([.35,.02,.65,.75], [.35,.25,.65,.95], [.49,.49,.51,.51]):
            value = copy.deepcopy(evidence)
            value['bbox'] = value['relative_stereo']['bbox'] = box
            quality = observation_quality(landmark, value)
            self.assertFalse(quality['completed'])
            self.assertFalse(quality['framing_usable'])

    def test_optical_measurement_is_copied_and_legacy_enu_is_only_for_explicit_archive_audit(self):
        landmark, evidence, semantic = self.evidence()
        before = semantic.snapshot()
        reference = relative_range_reference(evidence)
        reference['coordinate_epoch']['ekf_counters']['heading'] += 1
        reference['image_sha256']['left'] = 'f'*64
        self.assertEqual(before, semantic.snapshot())
        legacy = copy.deepcopy(evidence)
        legacy.pop('relative_stereo')
        legacy['covariance_enu_m2'] = [[.04,0.,0.],[0.,.04,0.],[0.,0.,.04]]
        self.assertFalse(observation_quality(landmark, legacy)['completed'])
        archived = observation_quality(landmark, legacy, policy=LEGACY_RANGE_POLICY)
        self.assertTrue(archived['completed'])
        self.assertNotIn('range_policy', archived)
        with self.assertRaisesRegex(ValueError, 'Unknown'):
            observation_quality(landmark, legacy, policy='silently_relax')

    def test_saved_relative_stereo_is_bound_to_original_report_without_aliasing(self):
        nav = navigator()
        frame, report, geometry = observation(nav)
        report['targets'] = [relative_target(frame, [5.,0.,1.], '蓝色油桶')]
        metric = report['targets'][0]['stereo_location']
        original = copy.deepcopy(metric)
        semantic = SemanticMap(OWNER, geometry)
        semantic.update_observation(frame, report, geometry)
        observation_record = next(iter(semantic.landmarks.values())).observations[0]
        self.assertEqual(observation_record['relative_stereo']['measurement'], original)
        self.assertEqual(observation_record['image_sha256'], frame['snapshot']['stereo_image_sha256'])
        metric['position_camera_optical_m'][2] = 999.
        metric['image_sha256']['right'] = 'f'*64
        self.assertEqual(observation_record['relative_stereo']['measurement'], original)


class TargetObservationFlowTests(unittest.TestCase):
    # Reuse the existing capture/report fixtures, not simulated visual claims.
    def observe(self, *args, **kwargs):
        with mock.patch.object(confirmation_fixtures, 'target', relative_target):
            return confirmation_fixtures.ConfirmationTests.observe(self, *args, **kwargs)
    next_confirmation = confirmation_fixtures.ConfirmationTests.next_confirmation
    review = confirmation_fixtures.ConfirmationTests.review

    def setUp(self):
        self.client = ScopedFixtureBridge()
        self.events = []
        self.client.record = lambda name, result: self.events.append((name, copy.deepcopy(result))) or result
        self.client.execute = lambda steps, description='': self.client.wait_for(
            self.client.request('/mission', {'steps': steps, 'description': description}))
        self.client.execute_viewpoint = lambda owner, batch_id, revision, candidate_id, ticket, description='': self.client.wait_for(
            self.client.request('/navigation/execute-viewpoint', {'owner': list(owner), 'batch_id': batch_id,
                'map_revision': revision, 'candidate_id': candidate_id, 'ticket': ticket, 'description': description}))
        self.runtime = SemanticMapRuntime(self.client)
        # A close photo deliberately upgrades this session to paired review,
        # even when the configured default is single-frame detection.
        with mock.patch.dict(os.environ, {'UAV_TARGET_CONFIRMATION': 'single_frame'}):
            self.search = AdaptiveSearch(self.client, search_mode='nbv_geometry', semantic_maps=self.runtime)
        self.store = FrameStore(self.client, CONFIG)
        self.request = '找到蓝色油桶，靠近拍照，然后返航降落'
        self.search.begin(confirmation_fixtures.OWNER, '蓝色油桶', original_request=self.request)
        self.search.execute_pending(confirmation_fixtures.OWNER)

    def test_confirmation_is_intermediate_then_actual_close_view_completes_request(self):
        point = [10., 0., 1.]
        self.assertEqual(self.search.summary(confirmation_fixtures.OWNER)['target_confirmation_mode'], 'multiview')
        first = self.observe(point=point)
        self.next_confirmation()
        self.observe(point=point)
        progress = self.review()
        self.assertEqual(progress['confirmation_outcome'], 'confirmed')
        self.assertTrue(progress['target_visual_confirmed'])
        self.assertFalse(progress['target_observation']['completed'])
        self.assertEqual(progress['search_status'], 'searching')
        self.assertEqual(progress['target_focus_state'], 'approaching')
        with self.assertRaisesRegex(ValueError, '收尾条件'):
            self.search.finish_normal(confirmation_fixtures.OWNER)
        for _ in range(5):
            self.next_confirmation()
            goal = self.search.sessions[confirmation_fixtures.OWNER]['nbv_public']['confirmation_goal']
            self.assertIn(goal['purpose'], ('approach', 'depth_acquisition'))
            self.observe(point=point)
            progress = self.review()
            if progress['search_status'] != 'searching': break
        self.assertEqual(progress['search_status'], 'target_found', progress)
        self.assertTrue(progress['target_observation']['completed'])
        self.assertNotEqual(progress['target_observation']['capture_id'], first['capture_id'])
        self.search.finish_normal(confirmation_fixtures.OWNER)
        self.assertFalse(self.client.telemetry['armed'])
        self.assertEqual(self.client.telemetry['landed_state'], 1)
        self.assertEqual(self.client.ticket_executions, len(self.search.sessions[confirmation_fixtures.OWNER]['observations'])-1)
        self.assertEqual(self.client.generic_gotos, 0)

    def test_repeated_missing_target_keeps_source_until_original_budget_exhausted(self):
        first = self.observe()
        started = self.search.sessions[confirmation_fixtures.OWNER]['started']
        for _ in range(2):
            self.next_confirmation()
            self.observe(present=False)
            result = self.review(current=False, same=False)
            self.assertEqual(result['search_phase'], 'confirming')
            self.assertEqual(result['confirmation']['source_capture_id'], first['capture_id'])
            self.assertEqual(result['target_focus_state'], 'reacquiring')
        self.assertEqual(self.search.sessions[confirmation_fixtures.OWNER]['started'], started)
        self.search.sessions[confirmation_fixtures.OWNER]['status'] = 'budget_exhausted'
        self.assertEqual(self.search.summary(confirmation_fixtures.OWNER)['next_tool'], 'return_home_then_land')

    def test_ambiguous_or_different_current_object_cannot_replace_unrefuted_source(self):
        first = self.observe(metric=False)
        self.next_confirmation()
        self.observe(metric=False)
        result = self.review(source=True, current=True, same=False)
        self.assertEqual(result['confirmation_outcome'], 'unconfirmed')
        self.assertEqual(result['confirmation']['source_capture_id'], first['capture_id'])
        self.assertEqual(result['target_focus_state'], 'association_unresolved')

    def test_no_safe_target_view_does_not_silently_resume_unrelated_exploration(self):
        first = self.observe()
        original = self.client.request
        def occlude(path, payload=None, **kwargs):
            response = original(path, payload, **kwargs)
            if path == '/navigation/viewpoints':
                for candidate in response['results']:
                    for probe in candidate.get('probe_visibility', []):
                        probe['unoccluded'] = False
            return response
        self.client.request = occlude
        result = self.search.next_view(confirmation_fixtures.OWNER, 'auto', 'controlled all views occluded')
        self.assertEqual(result['search_status'], 'no_reachable_view')
        self.assertEqual(result['confirmation']['source_capture_id'], first['capture_id'])
        self.assertEqual(result['target_focus_state'], 'no_safe_target_view')
        self.assertIsNone(result['pending'])
        self.search.finish_normal(confirmation_fixtures.OWNER)
        self.assertFalse(self.client.telemetry['armed'])

    def test_confirmed_target_without_close_photo_cannot_be_full_task_success(self):
        contract = assess_task(self.request)
        self.assertTrue(contract['target_observation_required'])
        progress = {'search_status': 'target_found', 'target_confirmation_mode': 'multiview',
                    'target_visual_confirmed': True,
                    'termination': {'source': 'normal_completion', 'purpose': 'goal_completion',
                                    'flight_status': 'grounded_disarmed'}}
        command = {'prompt': self.request, 'task_contract': contract,
                   'search_events': [{'result': progress}],
                   'target_confirmation_events': [{'outcome': 'confirmed'}]}
        self.assertFalse(search_result(command)['user_goal_completed'])
        progress['search_status'] = 'budget_exhausted'
        self.assertTrue(search_result(command)['target_confirmed'])
        self.assertFalse(search_result(command)['user_goal_completed'])
        from web_server import multiview_search_result
        progress['task_contract'] = contract
        outcome, reply = multiview_search_result(command)
        self.assertTrue(outcome['target_confirmed'])
        self.assertFalse(outcome['target_observation_completed'])
        self.assertIn('尚未取得', reply)

    def test_legacy_search_cannot_satisfy_close_photo_by_only_finding_target(self):
        command = {'prompt': self.request, 'search_events': [{'result': {'search': {'status': 'target_found'}}}]}
        result = search_result(command, flight_completion_verified=True)
        self.assertTrue(result['target_found'])
        self.assertFalse(result['user_goal_completed'])
        self.assertFalse(result['target_observation_completed'])

    def test_close_photo_intent_is_preserved_without_changing_plain_search_or_negation(self):
        for prompt in ('去白色货车附近拍摄一张照片', '找到油桶，靠近拍照', 'Find the van and take a close-up photo'):
            contract = assess_task(prompt)
            self.assertEqual(contract['admission'], 'accepted')
            self.assertTrue(contract['requires_target_search'])
            self.assertTrue(contract['target_observation_required'])
        for prompt in ('找到前方的白色货车，拍照后返航', '找到油桶，不要靠近拍照，直接报告'):
            self.assertFalse(assess_task(prompt)['target_observation_required'])

    def test_close_photo_selects_existing_nbv_capability_even_with_fixed_default(self):
        client = ScopedFixtureBridge()
        runtime = SemanticMapRuntime(client)
        search = AdaptiveSearch(client, search_mode='fixed', semantic_maps=runtime)
        result = search.begin(confirmation_fixtures.OWNER, '蓝色油桶', original_request=self.request)
        self.assertEqual(result['search_mode'], 'nbv_geometry')
        self.assertEqual(result['target_confirmation_mode'], 'multiview')
        self.assertEqual(search.search_mode, 'fixed')  # Only this session selects the required capability.
        self.assertFalse(client.telemetry['armed'])
        self.assertEqual(client.flight_requests, [])
        original = client.state
        def old_backend():
            state = original()
            state['navigation']['viewpoint_execution_supported'] = False
            return state
        client.state = old_backend
        search = AdaptiveSearch(client, search_mode='fixed', semantic_maps=runtime)
        with self.assertRaisesRegex(ValueError, '新版后端'):
            search.begin(confirmation_fixtures.OWNER, '蓝色油桶', original_request=self.request)
        self.assertEqual(search.sessions, {})
        self.assertEqual(client.flight_requests, [])

    def test_new_reference_does_not_authorize_old_epoch_or_flight_ticket(self):
        self.observe()
        self.search.next_view(confirmation_fixtures.OWNER, 'auto', 'prepare owned view')
        self.client.telemetry['external_vision']['adapter']['epoch'] += 1
        before = len(self.client.flight_requests)
        with self.assertRaisesRegex(ValueError, '坐标系已重置'):
            self.search.execute_pending(confirmation_fixtures.OWNER)
        self.assertEqual(len(self.client.flight_requests), before)


class TargetObservationGraphTests(unittest.TestCase):
    def test_full_agent_graph_retains_original_close_photo_goal_and_uses_owned_tickets(self):
        from coverage_search import project_ground_point
        from test_single_vlm import tool_call, image_urls
        harness = graph_fixtures.NBVGraphTests('test_multiview_graph_requires_paired_delivery_and_distinct_confirmation_move')
        bridge = ScopedFixtureBridge()
        point = [10., 0., 1.]
        original_localize = bridge.localize_stereo
        def metric(pair_id, boxes):
            result = original_localize(pair_id, boxes)
            pair = bridge.pairs[pair_id]
            for item in result['targets']:
                snapshot = dict(body_pose_enu=pair['body_pose_enu'], stamp_sim_s=pair['stamp_sim_s'],
                    stereo_image_sha256=pair['image_sha256'], coordinate_epoch=item['coordinate_epoch'],
                    camera_intrinsics=pair['left']['camera_intrinsics'])
                item.update(measured_stereo({'snapshot': snapshot}, point))
            return result
        bridge.localize_stereo = metric
        def report(messages):
            capture = json.loads(next(m for m in reversed(messages)
                if getattr(m, 'name', None) == 'observe_target_search_view').content)
            pair = next(reversed(bridge.pairs.values()))
            u, v = project_ground_point(pair['left']['camera_intrinsics'], pair['body_pose_enu'], point, 20.)['pixel_uv']
            return tool_call('report_scene', {'capture_id': capture['capture_id'],
                'summary': 'synthetic target observation', 'targets': [{'label': '蓝色油桶',
                    'bbox': [max(0.,u/640-.06), max(0.,v/480-.07), min(1.,u/640+.06), min(1.,v/480+.07)],
                    'surroundings': [], 'position_basis': 'image_only'}],
                'limitations': ['controlled fixture only']}, 'report-'+capture['capture_id'])
        def review(messages):
            self.assertGreaterEqual(len(image_urls(messages)), 2)
            value = json.loads(next(m for m in reversed(messages) if getattr(m, 'name', None) == 'report_scene').content)
            active = value['search_progress']['confirmation']
            return tool_call('review_target_candidate', {'current_target_id': active['current_target_ids'][0],
                'source_supported': True, 'current_supported': True, 'same_object': True,
                'source_description': 'synthetic source', 'current_description': 'synthetic current',
                'reason': 'controlled association'}, 'review-'+active['current_capture_id'])
        script = harness.prefix()[:-1]+[report]
        for i in range(5):
            script += [tool_call('next_target_search_view', {'strategy':'auto', 'reason':'继续围绕目标观察'}, 'prepare-'+str(i)),
                       tool_call('observe_target_search_view', {}, 'next-'+str(i)), report, review]
        prompt = '找到蓝色油桶，靠近拍照，然后返航降落'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR': str(root/'captures')}):
                result, app, _ = harness.run_task(bridge, script, mode='fixed', auto_prepare=False, prompt=prompt)
            from audit_target_confirmation import audit
            record = {'command': result, 'capture_directory': str(root/'captures'),
                      'target_confirmation_mode': 'multiview', 'flight': bridge.state()}
            path = root/'result.json'
            path.write_text(json.dumps(record))
            audited = audit(path, root/'audit', require_confirmed=True)
            self.assertTrue(audited['passed'], audited)
            # A valid map-local number also needs the unchanged original
            # report. Removing the policy cannot bypass the new evidence path.
            for name in ('report-measurement', 'policy-downgrade'):
                modified = copy.deepcopy(record)
                if name == 'report-measurement':
                    close = next(e['result']['target_observation'] for e in modified['command']['search_events']
                                 if (e.get('result',{}).get('target_observation') or {}).get('completed'))
                    report = next(p for p in modified['command']['perceptions'] if p['capture_id'] == close['capture_id'])
                    report['targets'][0]['stereo_location']['covariance_camera_optical_m2'][2][2] *= .01
                else:
                    for event in modified['command']['search_events']:
                        close = event.get('result',{}).get('target_observation') or {}
                        close.pop('range_policy', None)
                path.write_text(json.dumps(modified))
                self.assertFalse(audit(path, root/name)['passed'], name)
            broken = copy.deepcopy(record)
            for event in broken['command']['search_events']:
                close = (event.get('result') or {}).get('target_observation')
                if close and close.get('completed'):
                    close['range_m'] = .01  # It cannot be replaced by model prose/a bogus measurement.
            path.write_text(json.dumps(broken))
            self.assertFalse(audit(path, root/'tampered-audit')['passed'])
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(result['task_contract']['original_request'], prompt)
        self.assertTrue(result['task_outcome']['user_goal_completed'])
        self.assertTrue(result['task_outcome']['target_confirmed'])
        self.assertTrue(result['task_outcome']['target_observation_completed'])
        session = next(iter(app.audited_client.search_manager.adaptive.sessions.values()))
        self.assertTrue(any(e['outcome']=='observation_required' for e in result['target_confirmation_events']))
        self.assertEqual(bridge.ticket_executions, len(session['observations'])-1)
        self.assertEqual(bridge.generic_gotos, 0)
        self.assertFalse(bridge.telemetry['armed'])
        self.assertEqual(bridge.telemetry['landed_state'], 1)


if __name__ == '__main__':
    unittest.main()
