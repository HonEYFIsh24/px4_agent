"""Observed vehicle front-face geometry; no world labels or live model/flight."""
import copy
from contextlib import nullcontext
import math
import unittest
from unittest import mock

from semantic_map import SemanticMap
from task_runtime.object_frame import estimate_front_face_frame, current_front_relation, FRONT_FACE_BASIS
from task_runtime.goal_planner import GoalPosePlanner
from task_runtime.contracts import GoalSpec
from task_runtime.object_frame import verify_front_review
from test_semantic_map import navigator, observation, OWNER
from test_target_observation import relative_target, measured_stereo
from test_goal_pose import region


def fixture(identifier='front', stamp=1., yaw=0., translation=(0., 0., 0.), distance=4.):
    n = navigator()
    p = [translation[0], translation[1], translation[2]+3.]
    f, report, g = observation(n, identifier, p, stamp)
    pose = f['snapshot']['body_pose_enu']
    pose.update(orientation_xyzw=[0., 0., math.sin(yaw/2), math.cos(yaw/2)],
                orientation_covariance_rad2=[[.008, 0., 0.], [0., .008, 0.], [0., 0., .069]])
    g['body_pose_enu'] = copy.deepcopy(pose)
    def transform(v):
        return [translation[0]+v[0]*math.cos(yaw)-v[1]*math.sin(yaw),
                translation[1]+v[0]*math.sin(yaw)+v[1]*math.cos(yaw), translation[2]+v[2]]
    points = [transform([distance, 0., 1.]), transform([distance, 1.2, 1.]), transform([distance, -1.2, 1.])]
    report['targets'] = [relative_target(f, points[0], '白色货车', [.01, .01, .99, .99])]
    m = SemanticMap(OWNER, g);m.update_observation(f, report, g)
    landmark = m.landmarks['target_01'];o = landmark.observations[-1]
    sides = [region(f, point) for point in points[1:]]
    sides.sort(key=lambda b: b[0])
    face = [min(b[0] for b in sides), min(b[1] for b in sides), max(b[2] for b in sides), max(b[3] for b in sides)]
    boxes = [face, *sides]
    metrics = [measured_stereo(f, point) for point in points]
    value = estimate_front_face_frame(f, o, boxes, metrics, 'visible windshield and separated front face sides')
    return f, m, landmark, o, boxes, metrics, value


class FrontFaceTests(unittest.TestCase):
    def test_hidden_rear_uses_actual_lateral_front_points_without_inventing_a_rear(self):
        f, m, landmark, o, boxes, metrics, value = fixture()
        self.assertEqual(value['status'], 'available', value)
        orientation = value['orientation']
        self.assertEqual(orientation['basis'], FRONT_FACE_BASIS)
        self.assertIsNone(orientation['rear_surface_enu_m'])
        self.assertAlmostEqual(abs(orientation['yaw_enu_rad']), math.pi)
        self.assertGreater(orientation['yaw_variance_rad2'], .069)
        self.assertEqual(orientation['front_covariance_enu_m2'], metrics[0]['covariance_enu_m2'])
        self.assertIsNotNone(current_front_relation(orientation, o, f['snapshot']['body_pose_enu']))

    def test_relative_front_proof_is_invariant_to_common_world_translation_and_yaw(self):
        a = fixture()[-1]['orientation']; b = fixture(yaw=2.9, translation=(20., -9., 1.))[-1]['orientation']
        self.assertAlmostEqual(a['current_front_relation']['minimum_front_distance_m'],
                               b['current_front_relation']['minimum_front_distance_m'])
        self.assertAlmostEqual(a['relative_axis_variance_rad2'], b['relative_axis_variance_rad2'])
        self.assertGreater(b['yaw_variance_rad2'], b['relative_axis_variance_rad2'])

    def test_missing_depth_one_side_overlap_and_wrong_hash_cannot_create_a_frame(self):
        f, m, l, o, boxes, metrics, _ = fixture()
        weak = copy.deepcopy(metrics);weak[1] = {'status': 'unavailable'}
        self.assertIsNone(estimate_front_face_frame(f, o, boxes, weak, 'front')['orientation'])
        self.assertIsNone(estimate_front_face_frame(f, o, [boxes[0], boxes[1], None], [], 'front')['orientation'])
        self.assertIsNone(estimate_front_face_frame(f, o, [boxes[0], boxes[1], boxes[1]], metrics, 'front')['orientation'])
        clipped_o = copy.deepcopy(o);clipped_o['bbox'][0] = 0.
        clipped_boxes = copy.deepcopy(boxes);clipped_boxes[0][0] = 0.
        clipped = estimate_front_face_frame(f, clipped_o, clipped_boxes, [], 'part of front is clipped')
        self.assertEqual(clipped['reason_code'], 'FRONT_FACE_REGIONS_CLIPPED')
        self.assertIsNone(clipped['orientation'])
        bad = copy.deepcopy(metrics);bad[0]['image_sha256']['left'] = 'e'*64
        with self.assertRaises(ValueError):estimate_front_face_frame(f, o, boxes, bad, 'front')
        bad = copy.deepcopy(metrics);bad[0]['covariance_camera_optical_m2'] = [[1e-12, 0., 0.], [0., 1e-12, 0.], [0., 0., 1e-12]]
        with self.assertRaises(ValueError):estimate_front_face_frame(f, o, boxes, bad, 'front')

    def test_weak_or_changed_pose_cannot_reuse_a_current_front_proof(self):
        f, m, l, o, boxes, metrics, value = fixture(distance=7.)
        self.assertIsNone(current_front_relation(value['orientation'], o, f['snapshot']['body_pose_enu']))
        f, m, l, o, boxes, metrics, value = fixture()
        pose = copy.deepcopy(f['snapshot']['body_pose_enu']);pose['position_m'][0] += 1.
        self.assertIsNone(current_front_relation(value['orientation'], o, pose))
        pose = copy.deepcopy(f['snapshot']['body_pose_enu']);pose['age_seconds'] = 2.
        self.assertIsNone(current_front_relation(value['orientation'], o, pose))
        forged = copy.deepcopy(value['orientation']);forged['current_front_relation']['minimum_front_distance_m'] = 99.
        with self.assertRaises(ValueError):current_front_relation(forged, o, f['snapshot']['body_pose_enu'])

    def test_existing_front_position_becomes_a_goal_but_still_requires_executor_path_and_visibility(self):
        f, m, l, o, boxes, metrics, value = fixture()
        l.attributes.update(object_orientation=value['orientation'], object_frame_revision=1)
        ref = {'target_id': l.id, 'capture_id': f['capture_id'], 'confirmation_level': 'multiview'}
        planner = GoalPosePlanner();spec = GoalSpec(l.id, 'object_front', 'relative_land')
        generated = planner.generate(spec, m, ref, f['snapshot']['body_pose_enu'], [0., 0.], 30.)
        self.assertEqual(generated['scoring'], 'observed_current_front_region_v1')
        self.assertEqual(len(generated['candidates']), 1)
        self.assertEqual(generated['candidates'][0]['position_enu_m'], f['snapshot']['body_pose_enu']['position_m'])
        self.assertEqual(generated['target_anchor_covariance_enu_m2'],value['orientation']['front_covariance_enu_m2'])
        generated['target_anchor_covariance_enu_m2'][0][0]=0.
        self.assertGreater(value['orientation']['front_covariance_enu_m2'][0][0],0.)
        blocked = planner.evaluate(generated, lambda *args: {'reachable': False, 'target_visible': True})
        self.assertEqual(blocked['status'], 'no_safe_goal')
        behind = planner.generate(GoalSpec(l.id, 'object_behind', 'relative_hover'), m, ref,
                                  f['snapshot']['body_pose_enu'], [0., 0.], 30.)
        self.assertEqual(behind['status'], 'needs_observation')

    def test_fresh_front_review_needs_new_owned_pixels_and_measured_sides_again(self):
        f, m, l, o, boxes, metrics, value = fixture()
        l.attributes['object_orientation'] = value['orientation']
        self.assertIsNone(verify_front_review(l, o, f, boxes[0], metrics, 'front', side_boxes=boxes[1:]))
        f2, _, _, o2, boxes2, metrics2, _ = fixture('new-front', 2.)
        proof = verify_front_review(l, o2, f2, boxes2[0], metrics2, 'fresh front', side_boxes=boxes2[1:])
        self.assertEqual(proof['status'], 'verified')
        self.assertEqual(proof['basis'], 'fresh_same_frame_front_face_stereo_relation')
        self.assertIsNone(verify_front_review(l, o2, f2, boxes2[0], metrics2[:1], 'fresh front'))


# The following uses the real parent graph/goal and landing authorities, with
# explicit synthetic camera/model/PX4 transports only.
from test_task_landing_graph import FlatGoalBridge
from test_task_goal_graph import plan, response_for
import test_task_runtime as runtime_fixtures
from test_single_vlm import tool_call
from search_tools import coordinate_epoch
import json
import tempfile


class FrontFaceBridge(FlatGoalBridge):
    front, rear, surface = [4., 0., 1.], [8., 0., 1.], [4., 0., 1.]
    front_sides = [[4., 1.5, 1.], [4., -1.5, 1.]]

    def __init__(self, directory, weak_parts=False):
        super().__init__(directory)
        self.weak_parts = weak_parts

    def localize_stereo(self, identifier, boxes):
        pair = self.pairs[identifier]
        f = {'snapshot': {'body_pose_enu': pair['body_pose_enu'], 'stamp_sim_s': pair['stamp_sim_s'],
            'camera_intrinsics': pair['left']['camera_intrinsics'], 'stereo_image_sha256': pair['image_sha256'],
            'coordinate_epoch': coordinate_epoch(self.telemetry)}}
        if len(boxes) == 3:
            if self.weak_parts:
                return {'status': 'ok', 'targets': [{'status': 'unavailable', 'reason': 'controlled no front side depth'}]*3}
            sides = sorted(self.front_sides, key=lambda p: region(f, p)[0])
            points = [self.front, *sides]
        else:
            points = [self.front]*len(boxes)
        return {'status': 'ok', 'targets': [measured_stereo(f, p) for p in points]}


class RefutedRelationReferenceTests(unittest.TestCase):
    """Parent transition contracts; no fabricated recognition or real flight."""
    def transfer(self):
        from types import SimpleNamespace
        from unittest import mock
        from task_runtime.orchestrator import TaskOrchestrator
        runtime = TaskOrchestrator.__new__(TaskOrchestrator)
        epoch = {'vio_epoch': 0, 'ekf_counters': {'xy': 2, 'z': 2, 'heading': 2}}
        old = {'target_id': 'old-gray-box', 'capture_id': 'old-source', 'query': '白色货车',
            'confirmation_level': 'multiview', 'coordinate_epoch': epoch, 'stage_id': 'stage-02',
            'status': 'evidence_ready'}
        new = {**copy.deepcopy(old), 'target_id': 'new-white-van', 'capture_id': 'new-current'}
        marker = {'target_id': old['target_id'], 'capture_id': old['capture_id'], 'query': old['query']}
        child = {'query': old['query'], 'epoch': epoch, 'observations': [{'capture_id': new['capture_id']}],
                 'refuted_search_observations': [marker], 'unchanged_budget': 12}
        stage = SimpleNamespace(stage_id='stage-02', target_ref='van', evidence_refs=[],
                                conditions={'object_orientation': 'true'})
        session = {'goal_state': {'prepared': {'execution': None}, 'rejected_positions': [[1.,2.,3.]]},
                   'targets': {'van': old}, 'reasoning': {'target_id': old['target_id']}}
        runtime.sessions = {OWNER: session}
        runtime.skills = SimpleNamespace(search=SimpleNamespace(sessions={OWNER: child}))
        runtime.require_live = mock.Mock()
        runtime.emit = mock.Mock()
        return runtime, stage, old, new, session, child

    def test_explicitly_refuted_prior_confirmation_can_be_replaced_before_goal_flight(self):
        for prior in ('visual_candidate', 'multiview'):
            with self.subTest(prior=prior):
                runtime, stage, old, new, s, child = self.transfer()
                old['confirmation_level'] = prior
                previous = copy.deepcopy(old);child_before = copy.deepcopy(child)
                runtime._replace_refuted_relation_reference(OWNER, stage, old, new)
                self.assertEqual(s['targets']['van'], new)
                self.assertEqual(stage.evidence_refs[0]['previous'], previous)
                self.assertFalse(stage.evidence_refs[0]['same_object_claimed'])
                self.assertIsNone(s['goal_state']['prepared'])
                self.assertNotIn('rejected_positions', s['goal_state'])
                self.assertEqual(stage.conditions['object_orientation'], 'unknown')
                self.assertEqual(child, child_before)  # Same map/session/photo budget.
                runtime.require_live.assert_called_once_with(OWNER)

    def test_uncertain_foreign_stale_or_already_executed_replacement_is_forbidden(self):
        faults = ('not_refuted', 'other_source', 'other_query', 'candidate_only', 'not_ready',
                  'epoch', 'other_stage', 'other_capture', 'executed', 'live_fault')
        for fault in faults:
            with self.subTest(fault=fault):
                runtime, stage, old, new, s, child = self.transfer()
                if fault == 'not_refuted': child['refuted_search_observations'] = []
                elif fault == 'other_source': child['refuted_search_observations'][0]['capture_id'] = 'another-image'
                elif fault == 'other_query': new['query'] = '蓝色车辆'
                elif fault == 'candidate_only': new['confirmation_level'] = 'visual_candidate'
                elif fault == 'not_ready': new['status'] = 'candidate_acquired'
                elif fault == 'epoch': new['coordinate_epoch']['vio_epoch'] = 1
                elif fault == 'other_stage': new['stage_id'] = 'stage-99'
                elif fault == 'other_capture': new['capture_id'] = 'unreported-image'
                elif fault == 'executed': s['goal_state']['prepared']['execution'] = {'mission_id': 'already-flown'}
                else: runtime.require_live.side_effect = ValueError('controlled owner/coordinate/operator fault')
                before = copy.deepcopy(s)
                with self.assertRaises(ValueError):
                    runtime._replace_refuted_relation_reference(OWNER, stage, old, new)
                self.assertEqual(s, before)
                self.assertEqual(stage.evidence_refs, [])
                runtime.emit.assert_not_called()


if __name__ == '__main__': unittest.main()
