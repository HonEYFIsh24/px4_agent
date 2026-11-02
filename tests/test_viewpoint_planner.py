"""Controlled NBV fixtures: geometry, evidence immutability and selection contracts."""
import copy
import math
import unittest

from semantic_map import SemanticMap
from viewpoint_geometry import StaticViewpointEvaluator
from viewpoint_planner import ViewpointConfig, ViewpointError, ViewpointPlanner, map_identity, viewpoint_group
from test_semantic_map import navigator, observation, OWNER


class TwoCandidates(ViewpointPlanner):
    def generate_candidates(self, pose, origin_xy, fixed_yaw=None):
        return [{'id': 'view_001', 'position_enu_m': [2., 0., 3.], 'direction': '前', 'fixed_yaw_enu_rad': 0.},
                {'id': 'view_002', 'position_enu_m': [0., 2., 3.], 'direction': '左', 'fixed_yaw_enu_rad': 0.}]


class DiverseCandidates(ViewpointPlanner):
    def generate_candidates(self, pose, origin_xy, fixed_yaw=None):
        positions = [[-2., 0., 1.], [-3., 0., 1.], [-4., 0., 1.], [-2.5, -.1, 1.],
                     [0., 0., 5.], [2., 0., 3.], [0., 2., 5.], [0., -2., 3.],
                     [2., 0., 5.], [0., 2., 3.], [2., 0., 1.]]
        return [{'id': 'view_{:03d}'.format(i), 'position_enu_m': p,
                 'direction': 'controlled fixture', 'fixed_yaw_enu_rad': 0.}
                for i, p in enumerate(positions, 1)]


class ControlledVisibility:
    """Synthetic visible-cell sets; path checks still use the actual collision backend."""
    def __init__(self, backend, cells):
        self.backend, self.cells, self.hook = backend, cells, None
        self.operations = []

    def query(self, operation, origin, identity, candidates):
        self.operations.append(operation)
        if operation == 'visibility':
            result = self.backend.query('context', origin, identity, [])
            result['results'] = [{**c, 'reachable': True, 'visible_cell_ids': self.cells[c['id']]} for c in candidates]
        else:
            result = self.backend.query(operation, origin, identity, candidates)
        if self.hook:
            self.hook(operation, result)
        return result


class ViewpointTests(unittest.TestCase):
    def setUp(self):
        self.nav = navigator()
        self.frame, report, geometry = observation(self.nav)
        self.map = SemanticMap(OWNER, geometry)
        self.map.update_observation(self.frame, report, geometry)
        snapshot = self.frame['snapshot']
        self.backend = StaticViewpointEvaluator(self.nav, snapshot['body_pose_enu'],
            snapshot['camera_intrinsics'], snapshot['coordinate_epoch'])
        cells = self.map.insufficiently_observed_cells()
        self.cells = {'view_001': cells[:100], 'view_002': cells[100:200]}
        self.controlled = ControlledVisibility(self.backend, self.cells)

    def test_confirmation_can_choose_zero_new_coverage_and_requires_unoccluded_probes(self):
        from target_confirmation import confirmation_goal
        from test_semantic_map import target
        frame, report, geometry = observation(self.nav, targets=[target(self.frame, [5., 0., 1.], label='蓝色油桶')])
        semantic_map = SemanticMap(OWNER, geometry, query='蓝色油桶')
        semantic_map.update_observation(frame, report, geometry)
        for cell in semantic_map.cells:
            cell['coverage'] = 1.
        goal = confirmation_goal(semantic_map, next(iter(semantic_map.landmarks)), frame['capture_id'])
        pose = frame['snapshot']
        backend = StaticViewpointEvaluator(self.nav, pose['body_pose_enu'], pose['camera_intrinsics'],
            pose['coordinate_epoch'], probe_points_enu_m=goal['probe_points_enu_m'])
        result = ViewpointPlanner().evaluate(semantic_map, backend, goal)
        self.assertEqual(result['planning_mode'], 'confirmation')
        self.assertTrue(result['ranked_candidates'])
        self.assertTrue(all(c['coverage_gain_m2'] == 0 for c in result['ranked_candidates']))
        self.assertTrue(all(c['target_visibility_score'] > 0 for c in result['ranked_candidates']))
        original = backend.query
        def occluded(*args):
            response = original(*args)
            for candidate in response.get('results', []):
                for probe in candidate.get('probe_visibility', []):
                    probe['unoccluded'] = False
            return response
        backend.query = occluded
        blocked = ViewpointPlanner().evaluate(semantic_map, backend, goal)
        self.assertIsNone(blocked['selected_id'])
        self.assertEqual(blocked['ranked_candidates'], [])

    def test_candidates_sample_three_heights_and_eight_directions_without_duplicates(self):
        p = ViewpointPlanner()
        for height in (1., 3., 9., 10.):
            items = p.generate_candidates({'position_m': [0., 0., height]}, [0., 0.], .7)
            self.assertEqual(len(items), 33 if height in (1., 10.) else 50)
            self.assertEqual(len({tuple(round(v, 8) for v in c['position_enu_m']) for c in items}), len(items))
            for c in items:
                self.assertEqual(c['fixed_yaw_enu_rad'], .7)
                self.assertTrue(1 <= c['position_enu_m'][2] <= 10)
                self.assertGreaterEqual(math.dist(c['position_enu_m'], [0., 0., height]), .5)
        items = p.generate_candidates({'position_m': [11., 0., 3.]}, [0., 0.], 0.)
        self.assertLess(len(items), 50)
        self.assertTrue(all(math.hypot(*c['position_enu_m'][:2]) <= 12.+1e-9 for c in items))

    def test_discrete_heading_candidates_include_rotation_in_place_and_batch_visibility(self):
        planner = ViewpointPlanner(ViewpointConfig(heading_mode='discrete', max_path_evaluations=1))
        candidates = planner.generate_candidates({'position_m': [0., 0., 3.]}, [0., 0.], 0.)
        self.assertEqual(len(candidates), 152)
        self.assertEqual(len({c['id'] for c in candidates}), 152)
        in_place = [c for c in candidates if c['position_enu_m'] == [0., 0., 3.]]
        self.assertEqual({c['heading_step'] for c in in_place}, {-1, 1})
        snapshot = self.frame['snapshot']
        backend = StaticViewpointEvaluator(self.nav, snapshot['body_pose_enu'], snapshot['camera_intrinsics'],
                                            snapshot['coordinate_epoch'], allow_yaw=True)
        batches = []
        original = backend.query
        def checked(operation, origin, identity, batch):
            if operation == 'visibility': batches.append(len(batch))
            return original(operation, origin, identity, batch)
        backend.query = checked
        result = planner.evaluate(self.map, backend)
        self.assertEqual(batches, [64, 64, 24])
        self.assertLessEqual(result['path_evaluation_count'], 1)
        self.assertEqual(result['generated_count'], 152)

    def test_heading_response_tampering_cannot_be_selected(self):
        class OneYaw(ViewpointPlanner):
            def generate_candidates(self, pose, origin_xy, fixed_yaw=None):
                return [{'id': 'view_001', 'position_enu_m': [2., 0., 3.], 'direction': '转',
                         'fixed_yaw_enu_rad': fixed_yaw, 'heading_step': 1}]
        snapshot = self.frame['snapshot']
        backend = StaticViewpointEvaluator(self.nav, snapshot['body_pose_enu'], snapshot['camera_intrinsics'],
                                            snapshot['coordinate_epoch'], allow_yaw=True)
        original = backend.query
        def tamper(operation, origin, identity, candidates):
            result = original(operation, origin, identity, candidates)
            if operation == 'visibility': result['results'][0]['target_yaw_enu_rad'] += .1
            return result
        backend.query = tamper
        with self.assertRaisesRegex(ViewpointError, 'heading changed'):
            OneYaw(ViewpointConfig(heading_mode='discrete')).evaluate(self.map, backend)

    def test_real_geometry_predictions_do_not_change_observations_or_executing_plan(self):
        self.nav.last_plan = {'executing_plan': 'retain me'}
        before = self.map.snapshot()
        p = ViewpointPlanner()
        a = p.evaluate(self.map, self.backend)
        selected = p.select(self.map)
        self.assertEqual(a['status'], 'planned')
        self.assertGreater(selected['coverage_gain_m2'], 0)
        self.assertLess(a['path_evaluation_count'], a['generated_count'])
        self.assertEqual(self.map.snapshot(), before)
        self.assertEqual(self.nav.last_plan, {'executing_plan': 'retain me'})
        self.assertFalse(selected['execution_authorized'])
        points = [selected['path']['start_enu_m'], *selected['path']['waypoints']]
        self.assertTrue(all(self.nav.segment_safe(a, b) for a, b in zip(points, points[1:])))
        b = p.evaluate(self.map, self.backend)
        self.assertEqual(a, b)

    def test_fixed_scale_semantic_gain_changes_selection_against_geometry_baseline(self):
        for i in self.cells['view_002']:
            self.map.cells[i]['semantic_score'] = 1.
        geo = TwoCandidates(ViewpointConfig(mode='geometry'))
        sem = TwoCandidates(ViewpointConfig(mode='semantic'))
        a = geo.evaluate(self.map, self.controlled)
        b = sem.evaluate(self.map, self.controlled)
        self.assertEqual(a['selected_id'], 'view_001')
        self.assertEqual(b['selected_id'], 'view_002')
        selected = sem.select(self.map)
        self.assertEqual(selected['score_terms']['semantic'], 1.5)
        self.assertEqual(selected['coverage_gain_m2'], 25.)
        self.assertAlmostEqual(selected['motion_seconds'], 2./.65)

    def test_path_budget_is_reported_without_claiming_global_optimality(self):
        p = TwoCandidates(ViewpointConfig(max_path_evaluations=1, path_batch_size=1))
        result = p.evaluate(self.map, self.controlled)
        self.assertEqual(result['status'], 'path_budget_limited')
        self.assertEqual(result['path_evaluation_count'], 1)
        self.assertEqual(result['unvalidated_candidate_ids'], ['view_002'])
        with self.assertRaisesRegex(ViewpointError, 'not path-verified'):
            p.select(self.map, 'view_002')

    def test_selection_is_bound_to_current_map_evidence_and_private_verified_batch(self):
        p = TwoCandidates()
        result = p.evaluate(self.map, self.controlled)
        result['ranked_candidates'][0]['position_enu_m'][0] = 999
        self.assertEqual(p.select(self.map)['position_enu_m'], [2., 0., 3.])
        with self.assertRaises(ViewpointError):
            p.select(self.map, 'view_999')
        self.map.cells[self.cells['view_001'][0]]['coverage'] = 1.
        with self.assertRaisesRegex(ViewpointError, 'evaluate again'):
            p.select(self.map)

    def test_invalid_map_and_map_update_during_query_block_selection(self):
        p = TwoCandidates()
        def update(operation, result):
            if operation == 'visibility': self.map.cells[0]['semantic_score'] = .2
        self.controlled.hook = update
        with self.assertRaisesRegex(ViewpointError, 'map evidence changed'):
            p.evaluate(self.map, self.controlled)
        with self.assertRaises(ViewpointError): p.select(self.map)
        self.map.invalidate('test epoch reset')
        with self.assertRaisesRegex(ViewpointError, 'invalid semantic map'):
            p.evaluate(self.map, self.backend)

    def test_changed_epoch_pose_heading_calibration_or_candidate_id_is_rejected(self):
        for field in ('epoch', 'position', 'yaw', 'calibration', 'id'):
            with self.subTest(field=field):
                def change(operation, result):
                    if operation != 'paths': return
                    if field == 'epoch': result['identity']['coordinate_epoch']['vio_epoch'] += 1
                    if field == 'position': result['start_enu_m'][0] += 1.
                    if field == 'yaw': result['fixed_yaw_enu_rad'] += .1
                    if field == 'calibration': result['camera_calibration_sha256'] = 'b'*64
                    if field == 'id': result['results'][0]['id'] = 'view_999'
                self.controlled.hook = change
                with self.assertRaises(ViewpointError):
                    TwoCandidates().evaluate(self.map, self.controlled)

    def test_tampered_route_cannot_leave_radius_or_claim_shorter_length(self):
        for field in ('outside', 'length'):
            def change(operation, result):
                if operation != 'paths': return
                plan = result['results'][0]['path']
                if field == 'outside': plan['waypoints'].insert(0, [12.1, 0., 3.])
                else: plan['path_length_m'] = .01
            self.controlled.hook = change
            with self.assertRaises(ViewpointError):
                TwoCandidates().evaluate(self.map, self.controlled)

    def test_fully_observed_map_has_no_useful_view_and_no_path_work(self):
        for i in self.map.eligible: self.map.cells[i]['coverage'] = 1.
        p = TwoCandidates()
        result = p.evaluate(self.map, self.controlled)
        self.assertEqual(result['status'], 'no_useful_view')
        self.assertEqual(result['path_evaluation_count'], 0)
        self.assertIsNone(p.select(self.map))
        self.assertNotIn('paths', self.controlled.operations)

    def test_actual_route_length_changes_the_rank_instead_of_using_straight_distance(self):
        def detour(operation, result):
            if operation != 'paths': return
            plan = result['results'][0]['path']
            plan['waypoints'].insert(0, [2., 8., 3.])
            points = [plan['start_enu_m'], *plan['waypoints']]
            plan['path_length_m'] = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        self.controlled.hook = detour
        result = TwoCandidates().evaluate(self.map, self.controlled)
        self.assertEqual(result['selected_id'], 'view_002')
        self.assertGreater(result['ranked_candidates'][1]['motion_seconds'], 20.)

    def test_unreachable_candidates_do_not_become_executable_selections(self):
        def blocked(operation, result):
            if operation == 'paths':
                for candidate in result['results']:
                    candidate.update(reachable=False, reason='controlled fixture blocks this route')
        self.controlled.hook = blocked
        p = TwoCandidates()
        result = p.evaluate(self.map, self.controlled)
        self.assertEqual(result['status'], 'no_reachable_view')
        self.assertIsNone(p.select(self.map))

    def test_evaluator_rejects_identity_malformed_batches_and_unsafe_endpoint(self):
        identity = map_identity(self.map)
        for candidates in ([{'id': 'view_001', 'position_enu_m': [True, 0., 3.]}],
                           [{'id': 'free-form', 'position_enu_m': [0., 0., 3.]}],
                           [{'id': 'view_001', 'position_enu_m': [0., 0., 3.]}]*2):
            with self.assertRaises(ValueError):
                self.backend.query('visibility', [0., 0.], identity, candidates)
        identity['sortie_id'] = 'another sortie'
        with self.assertRaisesRegex(ValueError, 'epoch changed'):
            self.backend.query('context', [0., 0.], identity, [])
        result = self.backend.query('visibility', [0., 0.], map_identity(self.map),
            [{'id': 'view_001', 'position_enu_m': [13., 0., 3.]}])
        self.assertFalse(result['results'][0]['reachable'])

    def test_invalid_configuration_is_rejected(self):
        for config in ({'mode': 'oracle'}, {'semantic_weight': -1}, {'nominal_speed_m_s': 0},
                       {'max_path_evaluations': True}, {'path_batch_size': 9}, {'candidate_pool_mode': 'unbounded'}):
            with self.assertRaises(ValueError): ViewpointConfig(**config)

    def diverse_visibility(self):
        cells = self.map.insufficiently_observed_cells()
        return ControlledVisibility(self.backend, {'view_{:03d}'.format(i): cells[:300 if i <= 4 else 8]
                                                   for i in range(1, 12)})

    def test_diverse_pool_rechecks_low_score_groups_within_shared_budget(self):
        source = self.map.snapshot()
        ordinary = DiverseCandidates().evaluate(self.map, self.diverse_visibility())
        diverse = DiverseCandidates(ViewpointConfig(candidate_pool_mode='diverse'))
        result = diverse.evaluate(self.map, self.diverse_visibility())
        self.assertEqual(result['selected_id'], ordinary['selected_id'])
        self.assertEqual(ordinary['path_evaluation_count'], 4)
        self.assertLessEqual(result['path_evaluation_count'], 12)
        self.assertGreater(result['path_evaluation_count'], 4)
        self.assertEqual(len(result['candidate_pool_ids']), 8)
        self.assertIn('view_005', result['candidate_pool_ids'])  # vertical rise, pruned numerically before
        groups = set(result['candidate_pool_groups'].values())
        self.assertTrue({'rise:vertical', 'rise:front', 'level:front', 'level:left', 'level:right'} <= groups)
        self.assertFalse(set(result['candidate_pool_ids']) & set(result['pruned_by_upper_bound_ids']))
        for identifier in result['candidate_pool_ids']:
            self.assertTrue(diverse.select(self.map, identifier)['path_verified'])
        self.assertEqual(self.map.snapshot(), source)
        self.assertIsNone(self.nav.last_plan)

    def test_diversity_cannot_expand_a_small_or_already_consumed_budget(self):
        for budget in (4, 6):
            with self.subTest(budget=budget):
                result = DiverseCandidates(ViewpointConfig(candidate_pool_mode='diverse',
                    max_path_evaluations=budget)).evaluate(self.map, self.diverse_visibility())
                self.assertLessEqual(result['path_evaluation_count'], budget)
                if budget == 4:
                    self.assertNotIn('view_005', result['candidate_pool_ids'])
                    self.assertIn('rise:vertical', result['useful_groups_without_verified_offer'])

    def test_unreachable_diversity_representative_is_not_offered(self):
        backend = self.diverse_visibility()
        def reject(operation, response):
            if operation == 'paths':
                for item in response['results']:
                    if item['id'] == 'view_005':
                        item.update(reachable=False, reason='controlled unreachable rising route')
        backend.hook = reject
        result = DiverseCandidates(ViewpointConfig(candidate_pool_mode='diverse')).evaluate(self.map, backend)
        self.assertNotIn('view_005', result['candidate_pool_ids'])
        self.assertIn('rise:vertical', result['useful_groups_without_verified_offer'])
        self.assertLessEqual(result['path_evaluation_count'], 12)

    def test_additional_diversity_paths_use_the_same_full_route_verifier(self):
        backend = self.diverse_visibility()
        def corrupt(operation, response):
            if operation == 'paths':
                for item in response['results']:
                    if item['id'] == 'view_005':
                        item['path']['waypoints'].insert(0, [13., 0., 5.])
        backend.hook = corrupt
        with self.assertRaisesRegex(ViewpointError, 'exceeds'):
            DiverseCandidates(ViewpointConfig(candidate_pool_mode='diverse')).evaluate(self.map, backend)

    def test_groups_follow_commanded_body_heading(self):
        reference = {'start_enu_m': [0., 0., 3.], 'fixed_yaw_enu_rad': math.pi/2}
        self.assertEqual(viewpoint_group({'position_enu_m': [-2., 0., 5.]}, reference), 'rise:left')
        self.assertEqual(viewpoint_group({'position_enu_m': [0., 2., 3.]}, reference), 'level:front')
