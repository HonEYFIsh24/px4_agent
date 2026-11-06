"""Synthetic observations test confirmation protocol, not recognition accuracy."""
import base64
import copy
import hashlib
import io
import math
import os
import unittest
from unittest import mock

from PIL import Image
from autonomous_search import AdaptiveSearch
from coverage_search import project_ground_point
from semantic_map_runtime import SemanticMapRuntime
from single_vlm import FrameStore
from target_confirmation import review_crop
from test_nbv_search_integration import ScopedFixtureBridge
from test_semantic_map import target
from test_vision import CONFIG

OWNER = ('confirmation-test', 'task')


class RegionGeometryTests(unittest.TestCase):
    def observation(self, position, box, stamp):
        from test_semantic_map import navigator, observation
        from stereo_localization import matvec, quaternion_rotation, iris_left_optical_to_body
        frame, _, geometry = observation(navigator(), position=position, stamp=stamp)
        snapshot = frame['snapshot']; k = snapshot['camera_intrinsics']['K']
        ray = [((box[0]+box[2])*320-k[2])/k[0], ((box[1]+box[3])*240-k[5])/k[4], 1.]
        ray = matvec(quaternion_rotation(snapshot['body_pose_enu']['orientation_xyzw']),
                     matvec(iris_left_optical_to_body()['rotation'], ray))
        norm = math.sqrt(sum(v*v for v in ray))
        return dict(capture_id=str(stamp), stamp_sim_s=stamp, body_pose_enu=snapshot['body_pose_enu'],
            camera_info=snapshot['camera_intrinsics'], bbox=list(box),
            bearing_origin_enu_m=geometry['camera_origin_enu_m'], bearing_direction_enu=[v/norm for v in ray],
            position_enu_m=None)

    def test_clipped_regions_can_be_compatible_without_inventing_object_center(self):
        from types import SimpleNamespace
        from target_confirmation import pair_geometry
        source = self.observation([0., 0., 3.], [.83, .3, 1., .7], 1.)
        current = self.observation([-1.4, -1.4, 5.], [.58, .5, .8, .8], 2.)
        before = copy.deepcopy((source, current))
        result = pair_geometry(SimpleNamespace(id='source'), source, SimpleNamespace(id='current'), current)
        self.assertTrue(result['compatible'], result)
        self.assertTrue(result['association_hypothesis_only'])
        self.assertTrue(result['support_spans_both_box_axes'])
        self.assertGreaterEqual(result['support_pair_count'], 4)
        self.assertFalse(result['target_xyz_estimated'])
        self.assertEqual((source, current), before)

    def test_incompatible_regions_and_repeated_view_are_rejected(self):
        from types import SimpleNamespace
        from target_confirmation import pair_geometry
        source = self.observation([0., 0., 3.], [.83, .3, 1., .7], 1.)
        for box in ([0., .1, .15, .25], [.8, 0., 1., .1]):
            current = self.observation([-1.4, -1.4, 5.], box, 2.)
            self.assertFalse(pair_geometry(SimpleNamespace(id='s'), source, SimpleNamespace(id='c'), current)['compatible'])
        current = copy.deepcopy(source); current['capture_id'] = 'another-id'; current['stamp_sim_s'] = 2.
        self.assertFalse(pair_geometry(SimpleNamespace(id='s'), source, SimpleNamespace(id='c'), current)['compatible'])

    def test_bearing_nearby_same_class_objects_and_wrong_box_are_not_confirmation_evidence(self):
        from types import SimpleNamespace
        from target_confirmation import pair_geometry, association_allows_confirmation, CONFIRMATION_GEOMETRY_POLICY
        def aimed(position, point, stamp):
            template = self.observation(position, [.4, .4, .6, .6], stamp)
            projection = project_ground_point(template['camera_info'], template['body_pose_enu'], point, 20.)
            self.assertTrue(projection['inside_fov_and_range'])
            u, v = projection['pixel_uv']
            return self.observation(position, [u/640-.04, v/480-.04, u/640+.04, v/480+.04], stamp)
        source = aimed([0., 0., 3.], [5., 0., 1.], 1.)
        current = aimed([0., 1., 3.], [5., .2, 1.], 2.)
        first, second = SimpleNamespace(id='object_a'), SimpleNamespace(id='object_b')
        result = pair_geometry(first, source, second, current, CONFIRMATION_GEOMETRY_POLICY)
        self.assertTrue(result['compatible'], result)  # Different nearby objects can still intersect approximately.
        self.assertEqual(result['association_basis'], 'bearing_intersection')
        self.assertFalse(association_allows_confirmation(result))
        wrong = copy.deepcopy(current)
        wrong['bbox'] = [.02, .02, .12, .12]  # A wrong reported box, without changing the measured ray.
        result = pair_geometry(first, source, second, wrong, CONFIRMATION_GEOMETRY_POLICY)
        self.assertFalse(association_allows_confirmation(result))

    def test_association_policy_rejects_missing_unknown_and_inconsistent_basis(self):
        from target_confirmation import association_allows_confirmation
        for geometry in ({'compatible': True}, {'compatible': True, 'association_basis': 'future'},
                         {'compatible': True, 'association_basis': 'stereo_unique',
                          'basis': 'bearing_intersection_not_localization'}):
            self.assertFalse(association_allows_confirmation(geometry))


class ConfirmationTests(unittest.TestCase):
    def test_final_report_cannot_claim_confirmation_from_model_prose(self):
        from web_server import multiview_search_result
        command = {'search_events': [{'result': {'target_confirmation_mode': 'multiview',
            'query': '白色货车', 'search_status': 'budget_exhausted'}}],
            'target_confirmation_events': [{'outcome': 'unconfirmed'}], 'perceptions': [{}, {}]}
        outcome, reply = multiview_search_result(command)
        self.assertFalse(outcome['target_confirmed'])
        self.assertIn('尚未完成', reply)
        self.assertIn('预算已用尽', reply)
        self.assertNotIn('已完成降落', reply)
        command['search_events'][0]['result']['search_status'] = 'target_found'
        # Even an inconsistent target_found flag does not override failed reviews.
        self.assertFalse(multiview_search_result(command)[0]['target_confirmed'])
        command['target_confirmation_events'].append({'outcome': 'confirmed'})
        self.assertTrue(multiview_search_result(command)[0]['target_confirmed'])

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
        with mock.patch.dict(os.environ, {'UAV_TARGET_CONFIRMATION': 'multiview'}):
            self.search = AdaptiveSearch(self.client, search_mode='nbv_geometry', nbv_selector='deterministic',
                                         semantic_maps=self.runtime)
        self.store = FrameStore(self.client, CONFIG)
        self.search.begin(OWNER, '蓝色油桶')
        self.search.execute_pending(OWNER)

    def observe(self, present=True, metric=True, point=None):
        point = [5., 0., 1.] if point is None else point
        identifier = self.store.capture(OWNER, '蓝色油桶')['capture_id']
        self.store.mark_delivered(OWNER, identifier, {})
        frame = self.store.get(OWNER, identifier)
        targets = []
        if present:
            projection = project_ground_point(frame['snapshot']['camera_intrinsics'], frame['snapshot']['body_pose_enu'], point, 20.)
            self.assertTrue(projection['inside_fov_and_range'])
            u, v = projection['pixel_uv']
            box = [max(0., u/640-.06), max(0., v/480-.07), min(1., u/640+.06), min(1., v/480+.07)]
            targets = [target(frame, point, label='蓝色油桶', box=box)]
            if not metric: targets[0]['stereo_location'] = {'status': 'unavailable', 'reason': 'controlled no metric fixture'}
        report = {'status': 'ok', 'capture_id': identifier, 'summary': 'synthetic observation', 'targets': targets}
        self.search.consume(OWNER, frame, report)
        self.runtime.observe(OWNER, frame, report, self.search.sessions[OWNER], after_update=lambda m:
            self.search.after_map_observation(OWNER, m, frame, report))
        return frame

    def next_confirmation(self):
        self.search.next_view(OWNER, 'auto', 'verify candidate')
        self.assertEqual(self.search.sessions[OWNER]['nbv_public']['planning_mode'], 'confirmation')
        self.search.execute_pending(OWNER)

    def review(self, source=True, current=True, same=True, current_id=None, source_uncertain=False, current_uncertain=False):
        active = self.search.sessions[OWNER]['confirmation']
        if current_id is None: current_id = next(iter(active.get('current_target_ids', [])), '')
        return self.search.review_confirmation(OWNER, active['source_capture_id'], active['current_capture_id'],
            current_id, source, current, same, 'controlled synthetic visual review', source_uncertain, current_uncertain)

    def test_review_validation_reports_exact_conflict_without_mutating_candidate(self):
        self.observe()
        self.next_confirmation()
        self.observe()
        before = copy.deepcopy(self.search.sessions[OWNER])
        for source_uncertain, current_uncertain, code in (
                (True, False, 'source_support_uncertainty_conflict'),
                (False, True, 'current_support_uncertainty_conflict')):
            with self.subTest(code=code), self.assertRaisesRegex(ValueError, code):
                self.review(source_uncertain=source_uncertain, current_uncertain=current_uncertain)
            self.assertEqual(self.search.sessions[OWNER], before)
        active = before['confirmation']
        with self.assertRaisesRegex(ValueError, 'source_capture_id_mismatch'):
            self.search.review_confirmation(OWNER, 'wrong-source', active['current_capture_id'], '',
                                            False, False, False, 'controlled review')
        self.assertEqual(self.search.sessions[OWNER], before)

    def test_first_label_and_valid_depth_are_only_candidate_then_distinct_view_can_confirm(self):
        self.observe()
        self.assertEqual(self.search.summary(OWNER)['search_status'], 'searching')
        self.assertEqual(self.search.summary(OWNER)['search_phase'], 'confirming')
        self.next_confirmation()
        self.observe()
        self.assertEqual(self.search.summary(OWNER)['next_tool'], 'review_target_candidate')
        result = self.review()
        self.assertEqual(result['search_status'], 'target_found')
        self.assertEqual(result['confirmation_outcome'], 'confirmed')
        self.assertEqual(result['geometry']['association_basis'], 'stereo_unique')

    def test_single_depth_projection_can_confirm_with_both_visual_views_supported(self):
        self.observe(metric=True)
        self.next_confirmation()
        self.observe(metric=False)
        result = self.review()
        self.assertEqual(result['geometry']['association_basis'], 'single_depth_projection')
        self.assertEqual(result['confirmation_outcome'], 'confirmed')
        self.assertEqual(result['search_status'], 'target_found')

    def test_roi_refutation_retains_original_measurement_and_resumes_search(self):
        first = self.observe()
        self.next_confirmation()
        self.observe(present=False)
        result = self.review(source=False, current=False, same=False)
        self.assertEqual(result['confirmation_outcome'], 'rejected')
        self.assertEqual(result['search_status'], 'searching')
        self.assertEqual(result['search_phase'], 'exploring')
        semantic_map = self.runtime.maps[OWNER]
        self.assertEqual(len(semantic_map.observations), 2)
        self.assertTrue(any(o['capture_id'] == first['capture_id'] for l in semantic_map.landmarks.values() for o in l.observations))

    def test_not_reobserved_does_not_prove_original_target_absent(self):
        source = self.observe()
        self.next_confirmation()
        self.observe(present=False)
        result = self.review(source=True, current=False, same=False)
        self.assertEqual(result['confirmation_outcome'], 'unconfirmed')
        self.assertEqual(result['search_status'], 'searching')
        self.assertEqual(result['search_phase'], 'confirming')
        self.assertEqual(result['confirmation']['source_capture_id'], source['capture_id'])
        self.assertEqual(result['confirmation']['review_attempts'], 1)
        self.next_confirmation()
        goal = self.search.sessions[OWNER]['nbv_public']['confirmation_goal']
        selected = self.search.sessions[OWNER]['nbv_selected']['position_enu_m']
        self.assertTrue(all(math.dist(selected, p) >= .75 for p in goal['avoid_observer_positions']))
        self.observe(present=False)
        result = self.review(source=True, current=False, same=False)
        self.assertEqual(result['confirmation_outcome'], 'unconfirmed')
        self.assertEqual(result['search_phase'], 'confirming')
        self.assertEqual(result['confirmation']['source_capture_id'], source['capture_id'])
        self.assertEqual(result['target_focus_state'], 'reacquiring')

    def test_uncertain_source_is_not_rejected_and_stays_bounded(self):
        source = self.observe(metric=False)
        self.next_confirmation()
        self.observe(present=False)
        result = self.review(source=False, current=False, same=False, source_uncertain=True, current_uncertain=True)
        self.assertEqual(result['confirmation_outcome'], 'unconfirmed')
        self.assertEqual(result['search_phase'], 'confirming')
        self.assertEqual(result['confirmation']['source_capture_id'], source['capture_id'])
        self.next_confirmation()
        self.observe(present=False)
        result = self.review(source=False, current=False, same=False, source_uncertain=True, current_uncertain=True)
        self.assertEqual(result['confirmation_outcome'], 'unconfirmed')
        self.assertEqual(result['search_phase'], 'confirming')
        self.assertEqual(result['confirmation']['source_capture_id'], source['capture_id'])

    def test_uncertainty_cannot_contradict_positive_support(self):
        self.observe()
        self.next_confirmation()
        self.observe()
        with self.assertRaises(ValueError): self.review(source=True, source_uncertain=True)
        with self.assertRaises(ValueError): self.review(current=True, current_uncertain=True)

    def test_uncertain_source_cannot_confirm_even_with_supported_current_and_strong_geometry(self):
        self.observe()
        self.next_confirmation()
        current = self.observe()
        result = self.review(source=False, source_uncertain=True, current=True, same=True)
        self.assertTrue(result['geometry']['compatible'])
        self.assertIn(result['geometry']['association_basis'], ('stereo_unique', 'single_depth_projection'))
        self.assertEqual(result['confirmation_outcome'], 'unconfirmed')
        self.assertEqual(result['search_status'], 'searching')
        self.assertEqual(result['search_phase'], 'confirming')
        self.assertEqual(result['confirmation']['source_capture_id'], current['capture_id'])
        self.next_confirmation()

    def test_uncertain_current_or_different_object_cannot_confirm(self):
        self.observe()
        self.next_confirmation()
        self.observe()
        active = copy.deepcopy(self.search.sessions[OWNER]['confirmation'])
        result = self.review(source=True, current=False, current_uncertain=True, same=True)
        self.assertEqual(result['confirmation_outcome'], 'unconfirmed')
        self.assertEqual(result['search_status'], 'searching')
        self.search.sessions[OWNER].update(confirmation=active, search_phase='awaiting_review')
        result = self.review(source=True, current=True, same=False)
        self.assertEqual(result['confirmation_outcome'], 'unconfirmed')
        self.assertEqual(result['search_status'], 'searching')

    def test_supported_current_view_is_retained_as_new_candidate_after_source_rejected(self):
        self.observe(metric=False)
        self.next_confirmation()
        current = self.observe(metric=False)
        result = self.review(source=False, current=True, same=False)
        self.assertEqual(result['confirmation_outcome'], 'rejected')
        self.assertEqual(result['search_status'], 'searching')
        self.assertEqual(result['search_phase'], 'confirming')
        self.assertEqual(result['confirmation']['source_capture_id'], current['capture_id'])
        self.assertEqual(len(self.runtime.maps[OWNER].observations), 2)
        self.next_confirmation()

    def test_candidate_refresh_cannot_extend_exhausted_search_budget(self):
        self.observe(metric=False)
        self.next_confirmation()
        self.observe(metric=False)
        self.search.sessions[OWNER]['status'] = 'budget_exhausted'
        result = self.review(source=False, current=True, same=False)
        self.assertEqual(result['search_status'], 'budget_exhausted')
        self.assertIsNone(result['confirmation'])

    def test_bearing_only_multiview_is_hypothesis_not_confirmation(self):
        from target_confirmation import observation_for, pair_geometry
        source = self.observe(metric=False)
        self.next_confirmation()
        current = self.observe(metric=False)
        active = self.search.sessions[OWNER]['confirmation']
        semantic_map = self.runtime.maps[OWNER]
        sl, so = observation_for(semantic_map, active['target_id'], source['capture_id'])
        cl, co = observation_for(semantic_map, active['current_target_ids'][0], current['capture_id'])
        old = pair_geometry(sl, so, cl, co, 'clipped_regions_v3')
        self.assertTrue(old['compatible'])
        self.assertFalse(old.get('association_hypothesis_only', False))
        result = self.review()
        self.assertEqual(result['confirmation_outcome'], 'unconfirmed', result)
        self.assertEqual(result['search_status'], 'searching')
        self.assertTrue(result['geometry']['compatible'])
        self.assertTrue(result['geometry']['association_hypothesis_only'])
        self.assertEqual(result['geometry']['association_basis'], 'bearing_intersection')
        self.assertFalse(result['geometry']['target_xyz_estimated'])
        self.assertTrue(all(l.position_enu_m is None for l in self.runtime.maps[OWNER].landmarks.values()))
        self.assertEqual(self.runtime.maps[OWNER].memory.targets, [])

    def test_region_overlap_alone_cannot_confirm_even_when_model_claims_same_object(self):
        self.observe(metric=False)
        self.next_confirmation()
        self.observe(metric=False)
        weak = {'compatible': True, 'basis': 'clipped_box_epipolar_region_not_localization',
                'association_basis': 'clipped_region_hypothesis',
                'association_hypothesis_only': True, 'target_xyz_estimated': False}
        with mock.patch('target_confirmation.pair_geometry', return_value=weak):
            result = self.review(source=True, current=True, same=True)
        self.assertEqual(result['confirmation_outcome'], 'unconfirmed')
        self.assertEqual(result['search_status'], 'searching')
        self.assertEqual(result['search_phase'], 'confirming')

    def test_ambiguous_same_label_detections_in_either_view_cannot_confirm(self):
        self.observe(metric=False)
        self.next_confirmation()
        self.observe(metric=False)
        semantic_map = self.runtime.maps[OWNER]
        active = self.search.sessions[OWNER]['confirmation']
        source_id, current_id = active['target_id'], active['current_target_ids'][0]
        for identifier in (source_id, current_id):
            duplicate = copy.deepcopy(semantic_map.landmarks[identifier])
            duplicate.id = 'ambiguous'
            semantic_map.landmarks[duplicate.id] = duplicate
            if identifier == current_id:
                active['current_target_ids'].append('ambiguous')
            result = self.review(current_id=current_id)
            self.assertEqual(result['confirmation_outcome'], 'unconfirmed')
            self.assertFalse(result['geometry']['compatible'])
            del semantic_map.landmarks['ambiguous']
            # Restore pending review to independently test the opposite view.
            self.search.sessions[OWNER]['confirmation'] = active
            self.search.sessions[OWNER]['search_phase'] = 'awaiting_review'

    def test_wrong_image_pair_cannot_confirm(self):
        self.observe()
        self.next_confirmation()
        self.observe()
        active = self.search.sessions[OWNER]['confirmation']
        with self.assertRaisesRegex(ValueError, 'source_capture_id_mismatch'):
            self.search.review_confirmation(OWNER, 'foreign', active['current_capture_id'],
                active['current_target_ids'][0], True, True, True, 'foreign source')
        self.assertEqual(self.search.summary(OWNER)['search_status'], 'searching')

    def test_coordinate_reset_while_waiting_for_review_cannot_confirm(self):
        self.observe()
        self.next_confirmation()
        self.observe()
        self.client.telemetry['external_vision']['adapter']['epoch'] += 1
        with self.assertRaisesRegex(ValueError, '坐标系已重置'):
            self.review()
        self.assertEqual(self.search.summary(OWNER)['search_status'], 'searching')

    def test_roi_has_explicit_original_provenance_and_does_not_change_capture_count(self):
        frame = self.observe()
        box = next(iter(self.runtime.maps[OWNER].landmarks.values())).observations[0]['bbox']
        before = len(self.runtime.maps[OWNER].observations)
        crop = review_crop(frame, box)
        raw = base64.b64decode(crop['image_base64'])
        self.assertTrue(crop['derived_not_new_capture'])
        self.assertEqual(crop['crop_sha256'], hashlib.sha256(raw).hexdigest())
        Image.open(io.BytesIO(raw)).verify()
        self.assertEqual(before, len(self.runtime.maps[OWNER].observations))
        broken = copy.deepcopy(frame); broken['camera']['image_sha256'] = '0'*64
        with self.assertRaisesRegex(ValueError, 'hash'):
            review_crop(broken, box)
