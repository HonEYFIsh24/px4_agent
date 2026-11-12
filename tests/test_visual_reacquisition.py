"""Synthetic target memory and real geometric planning for lost-target recovery."""
import copy
import math
import unittest
from types import SimpleNamespace

from mission import MissionError
from semantic_map import SemanticMap
from task_runtime.memory import SharedTargetMemory
from target_confirmation import confirmation_goal
from viewpoint_geometry import StaticViewpointEvaluator, registered_identity
from viewpoint_planner import ViewpointConfig, ViewpointPlanner
from test_semantic_map import EPOCH, INFO, OWNER, navigator, observation, target


class RecoveryMemoryTests(unittest.TestCase):
    def setUp(self):
        def seen(capture, stamp):
            return {'capture_id': capture, 'stamp_sim_s': stamp,
                    'coordinate_epoch': copy.deepcopy(EPOCH)}
        self.first = seen('target-first', 1.)
        self.latest = seen('target-latest', 2.)
        self.landmark = SimpleNamespace(observations=[self.first, self.latest], attributes={})
        self.map = SimpleNamespace(status='valid', coordinate_epoch=copy.deepcopy(EPOCH),
            landmarks={'known': self.landmark,
                       'other': SimpleNamespace(observations=[seen('other-target', 9.)], attributes={})},
            observations=[self.first, self.latest, seen('empty-current-image', 10.)])
        self.memory = SharedTargetMemory(self.map)

    def test_latest_same_instance_observation_survives_newer_empty_or_other_target_frames(self):
        before = copy.deepcopy(self.landmark.observations)
        selected = self.memory.last_reliable_observation('known', '橙色帐篷')
        self.assertEqual(selected['capture_id'], 'target-latest')
        selected['coordinate_epoch']['vio_epoch'] += 1
        self.assertEqual(self.landmark.observations, before)

    def test_new_same_instance_sighting_updates_the_recovery_source(self):
        self.landmark.observations.append({**copy.deepcopy(self.latest),
            'capture_id': 'target-reacquired', 'stamp_sim_s': 11.})
        self.assertEqual(self.memory.last_reliable_observation('known', '橙色帐篷')['capture_id'],
                         'target-reacquired')

    def test_refuted_cross_epoch_and_ambiguous_observations_do_not_replace_reliable_source(self):
        for invalid in ('refuted', 'epoch', 'ambiguous'):
            with self.subTest(invalid=invalid):
                self.latest.clear()
                self.latest.update(capture_id='target-latest', stamp_sim_s=2.,
                                   coordinate_epoch=copy.deepcopy(EPOCH))
                self.landmark.attributes.clear()
                if invalid == 'refuted':
                    self.landmark.attributes['refuted_visual_observations'] = [
                        {'capture_id': 'target-latest', 'query': '橙色帐篷'}]
                elif invalid == 'epoch':
                    self.latest['coordinate_epoch']['vio_epoch'] += 1
                else:
                    self.latest['association'] = {'status': 'ambiguous', 'candidates': ['known', 'other']}
                self.assertEqual(self.memory.last_reliable_observation('known', '橙色帐篷')['capture_id'],
                                 'target-first')

    def test_invalid_map_or_all_refuted_observations_cannot_supply_a_recovery_source(self):
        self.map.status = 'invalid'
        with self.assertRaisesRegex(ValueError, 'invalid semantic map'):
            self.memory.last_reliable_observation('known', '橙色帐篷')
        self.map.status = 'valid'
        self.landmark.attributes['refuted_visual_observations'] = [
            {'capture_id': o['capture_id'], 'query': '橙色帐篷'} for o in self.landmark.observations]
        with self.assertRaisesRegex(ValueError, 'No retained reliable observation'):
            self.memory.last_reliable_observation('known', '橙色帐篷')


class RecoveryGeometryTests(unittest.TestCase):
    def make_map(self, *, radius=12., lost_pose=None):
        nav = navigator()
        nav.bounds = [[-35., -35., -.2], [35., 35., 20.4]]
        point = [7.268074087407504, -2.2961594346899754, .6808016283659941]

        def saved(capture, stamp, position=None, yaw=0., with_target=False):
            frame, report, _ = observation(nav, capture, position=position, stamp=stamp)
            snapshot = frame['snapshot']
            snapshot['body_pose_enu']['orientation_xyzw'] = [0., 0., math.sin(yaw/2), math.cos(yaw/2)]
            geometry = nav.observation_visibility(snapshot['body_pose_enu'], INFO, [0., 0.],
                                                  radius=radius, resolution=1. if radius == 30. else .5)
            geometry.update(capture_pair_id=snapshot['stereo_pair_id'],
                coordinate_epoch=copy.deepcopy(EPOCH), stamp_sim_s=stamp,
                image_sha256=copy.deepcopy(snapshot['stereo_image_sha256']),
                body_pose_enu=copy.deepcopy(snapshot['body_pose_enu']), camera_intrinsics=copy.deepcopy(INFO))
            if with_target:
                report['targets'] = [target(frame, point, label='橙色帐篷', box=[.54, .37, .77, .6], variance=4.)]
            return frame, report, geometry

        source, report, geometry = saved('source-visible', 1., with_target=True)
        semantic = SemanticMap(OWNER, geometry, query='橙色帐篷')
        semantic.update_observation(source, report, geometry)
        identifier = next(iter(semantic.landmarks))
        goal = confirmation_goal(semantic, identifier, source['capture_id'])
        goal['purpose'] = 'recovery'
        current = source
        if lost_pose:
            current, report, geometry = saved('current-empty', 2., position=lost_pose[0], yaw=lost_pose[1])
            semantic.update_observation(current, report, geometry)
        goal['avoid_observer_positions'] = [list(o['position_enu_m']) for o in semantic.observations]
        snapshot = current['snapshot']
        backend = StaticViewpointEvaluator(nav, snapshot['body_pose_enu'], snapshot['camera_intrinsics'],
            EPOCH, probe_points_enu_m=goal['probe_points_enu_m'], allow_yaw=True, search_radius_m=radius,
            grid_resolution_m=1. if radius == 30. else .5)
        return nav, semantic, goal, backend, current

    def test_passed_target_can_be_seen_again_by_a_memory_directed_turn(self):
        # Geometry reproduces the former first lost view. These are explicit
        # controlled measurements, not an online injection of scene truth.
        position = [14.599289894104004, 2.639202356338501, 3.244906425476074]
        yaw = math.radians(-9.831036036940198)
        _, semantic, goal, backend, _ = self.make_map(radius=30., lost_pose=(position, yaw))
        before = semantic.snapshot()
        planner = ViewpointPlanner(ViewpointConfig(heading_mode='discrete'))
        prediction = planner.evaluate(semantic, backend, goal)
        selected = planner.select(semantic)
        self.assertEqual(prediction['planning_mode'], 'confirmation')
        self.assertEqual(prediction['confirmation_goal']['source_capture_id'], 'source-visible')
        self.assertIsNotNone(selected)
        self.assertGreater(selected['target_visibility_score'], 0.)
        self.assertEqual(selected['candidate_basis'], 'retained_visual_target_bearing')
        self.assertGreater(abs(selected['heading_step']), 1)
        self.assertGreaterEqual(selected['motion_seconds'], abs(selected['heading_step'])*math.pi/4/.5)
        self.assertEqual(selected['score_terms']['parallax'], 0.)
        self.assertEqual(semantic.snapshot(), before)

    def test_returning_to_a_source_view_is_recovery_not_a_new_confirmation_baseline(self):
        class SamePosition(ViewpointPlanner):
            def generate_candidates(self, pose, origin_xy, fixed_yaw=None):
                return [{'id': 'view_001', 'position_enu_m': list(pose['position_m']),
                         'fixed_yaw_enu_rad': fixed_yaw, 'direction': 'revisit known view'}]

        _, semantic, goal, backend, _ = self.make_map()
        recovered = SamePosition().evaluate(semantic, backend, goal)
        self.assertEqual(recovered['selected_id'], 'view_001')
        self.assertGreater(recovered['ranked_candidates'][0]['target_visibility_score'], 0.)
        self.assertAlmostEqual(recovered['ranked_candidates'][0]['parallax_score'], 0.)
        ordinary = copy.deepcopy(goal)
        ordinary['purpose'] = 'confirmation'
        unconfirmed = SamePosition().evaluate(semantic, backend, ordinary)
        self.assertIsNone(unconfirmed['selected_id'])
        self.assertTrue(any(r['stage'] == 'confirmation' for r in unconfirmed['rejected']))

    def test_source_view_revisit_keeps_occlusion_predictions_honest(self):
        _, semantic, goal, backend, _ = self.make_map(lost_pose=([4., 0., 3.], math.pi/2))
        query = backend.query
        predicted_occlusions = []

        def all_probes_occluded(operation, *args):
            response = query(operation, *args)
            for candidate in response.get('results', []):
                for probe in candidate.get('probe_visibility', []):
                    probe['unoccluded'] = False
                    predicted_occlusions.append(probe)
            return response

        backend.query = all_probes_occluded
        before = semantic.snapshot()
        planner = ViewpointPlanner(ViewpointConfig(heading_mode='discrete'))
        prediction = planner.evaluate(semantic, backend, goal)
        selected = planner.select(semantic)
        self.assertIsNotNone(selected, prediction['status'])
        self.assertEqual(selected['candidate_basis'], 'last_reliable_observer_pose')
        self.assertEqual(selected['position_enu_m'], goal['source_observer_enu_m'])
        self.assertEqual(selected['target_visibility_score'], 0.)
        self.assertGreater(selected['source_view_revisit_score'], 0.)
        self.assertEqual(selected['score_terms']['target_visibility'], 0.)
        self.assertGreater(selected['score_terms']['source_view_revisit'], 0.)
        self.assertTrue(selected['path_verified'])
        self.assertTrue(predicted_occlusions)
        self.assertTrue(all(p['unoccluded'] is False for p in predicted_occlusions))
        self.assertEqual(semantic.snapshot(), before)
        ordinary = copy.deepcopy(goal)
        ordinary['purpose'] = 'confirmation'
        self.assertIsNone(ViewpointPlanner(ViewpointConfig(heading_mode='discrete')).evaluate(
            semantic, backend, ordinary)['selected_id'])

    def test_target_recovery_turns_still_require_yaw_authority_and_target_probes(self):
        nav = navigator()
        frame, _, _ = observation(nav)
        snapshot = frame['snapshot']
        identity = registered_identity(nav, EPOCH)

        def backend(*, yaw, probes):
            return StaticViewpointEvaluator(nav, snapshot['body_pose_enu'], INFO, EPOCH,
                allow_yaw=yaw, probe_points_enu_m=[[5., 0., 1.]] if probes else None)

        candidate = {'id': 'view_001', 'position_enu_m': [0., 0., 3.], 'heading_step': -3}
        result = backend(yaw=True, probes=True).query('visibility', [0., 0.], identity, [candidate])
        self.assertAlmostEqual(result['results'][0]['target_yaw_enu_rad'], -3*math.pi/4)
        for options in ({'yaw': False, 'probes': True}, {'yaw': True, 'probes': False}):
            with self.subTest(options=options), self.assertRaises(MissionError):
                backend(**options).query('visibility', [0., 0.], identity, [candidate])
        allowed = backend(yaw=True, probes=True)
        for step in (-4, 5, 0, True, 1.5, '3', None):
            with self.subTest(step=step), self.assertRaises(MissionError):
                allowed.query('visibility', [0., 0.], identity, [{**candidate, 'heading_step': step}])


if __name__ == '__main__':
    unittest.main()
