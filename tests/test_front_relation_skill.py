"""Front relation evidence and parent loop; all camera/model/flight IO are doubles."""
import copy
import json
import math
import tempfile
import unittest

from semantic_map import SemanticMap
from stereo_localization import iris_left_optical_to_body, matmul, matvec, quaternion_rotation
from task_runtime.object_frame import (front_surface_regions, estimate_front_surface_frame,
    front_relation_quality, current_front_relation, FRONT_SURFACE_BASIS)
from test_front_face import fixture
from test_target_observation import measured_stereo
from test_goal_pose import region


def pixel_surface(frame, u, v, plane_x=4.):
    pose = frame['snapshot']['body_pose_enu'];k = frame['snapshot']['camera_intrinsics']['K']
    mount = iris_left_optical_to_body();rotation = quaternion_rotation(pose['orientation_xyzw'])
    optical = matmul(rotation, mount['rotation'])
    origin = [a+b for a, b in zip(pose['position_m'], matvec(rotation, mount['translation_m']))]
    ray = matvec(optical, [(u-k[2])/k[0], (v-k[5])/k[4], 1.])
    distance = (plane_x-origin[0])/ray[0]
    return [a+distance*b for a, b in zip(origin, ray)]


def surface_case(distance=4., unavailable=0, outliers=False, face=None):
    frame, semantic, landmark, observed, _, _, _ = fixture(distance=distance)
    face = face or [.24, .25, .76, .60]
    regions = front_surface_regions(face)
    metrics = []
    for i, box in enumerate(regions):
        if i < unavailable:
            metrics.append({'status': 'unavailable', 'reason': 'controlled missing support'})
            continue
        u, v = (box[0]+box[2])*320, (box[1]+box[3])*240
        point = pixel_surface(frame, u, v, distance+(3. if outliers and i % 7 == 0 else 0.))
        metrics.append(measured_stereo(frame, point))
    value = estimate_front_surface_frame(frame, observed, face, regions, metrics, 'controlled opaque vertical fascia')
    return frame, semantic, landmark, observed, face, regions, metrics, value


class FrontSurfaceTests(unittest.TestCase):
    def test_uncertain_normal_can_guide_observation_without_becoming_authority(self):
        f, m, l, o, face, boxes, metrics, evidence = surface_case(distance=7., face=[.4, .25, .6, .60])
        original = copy.deepcopy(evidence)
        quality = front_relation_quality(evidence, f['capture_id'], face)
        reference = quality['viewpoint_reference']
        self.assertEqual(reference['source_capture_id'], f['capture_id'])
        self.assertEqual(reference['image_sha256'], o['image_sha256'])
        self.assertEqual(reference['coordinate_epoch'], o['coordinate_epoch'])
        self.assertEqual(reference['covariance_enu_m2'], evidence['orientation']['front_covariance_enu_m2'])
        self.assertEqual(reference['yaw_variance_rad2'], evidence['orientation']['yaw_variance_rad2'])
        self.assertTrue(reference['prediction_only'])
        self.assertFalse(reference['execution_authorized'])
        self.assertFalse(quality['global_axis_usable'])
        self.assertFalse(quality['relation_usable'])
        self.assertEqual(evidence, original)
        reference['covariance_enu_m2'][0][0] = 0.
        self.assertEqual(evidence, original)

    def test_actual_sample_set_has_bounded_support_and_replayable_relation(self):
        f, m, l, o, face, regions, metrics, evidence = surface_case(outliers=True)
        self.assertEqual(evidence['status'], 'available', evidence)
        orientation = evidence['orientation']
        self.assertEqual(orientation['basis'], FRONT_SURFACE_BASIS)
        self.assertLessEqual(len(regions), 64)
        self.assertLess(orientation['fitting']['inliers'], orientation['fitting']['valid_samples'])
        quality = front_relation_quality(evidence, f['capture_id'], face)
        self.assertTrue(quality['measurement_acquired'])
        self.assertTrue(quality['relation_usable'])
        self.assertIsNotNone(current_front_relation(orientation, o, f['snapshot']['body_pose_enu']))

    def test_fewer_or_forged_samples_never_create_a_relation(self):
        f, m, l, o, face, boxes, metrics, value = surface_case(unavailable=61)
        self.assertIsNone(value['orientation'])
        f, m, l, o, face, boxes, metrics, value = surface_case()
        for field in ('image_sha256', 'covariance_camera_optical_m2', 'coordinate_epoch'):
            bad = copy.deepcopy(metrics)
            if field == 'image_sha256':bad[0][field]['left'] = 'a'*64
            elif field == 'coordinate_epoch':bad[0][field]['vio_epoch'] += 1
            else:bad[0][field][2][2] *= .1
            with self.subTest(field=field), self.assertRaises(ValueError):
                estimate_front_surface_frame(f, o, face, boxes, bad, 'controlled opaque face')

    def test_measurement_is_separate_from_current_relation_and_covariance_is_not_count_reduced(self):
        f, m, l, o, face, boxes, metrics, evidence = surface_case(distance=2.8)
        q = front_relation_quality(evidence, f['capture_id'], face)
        self.assertTrue(q['measurement_acquired'])
        self.assertFalse(q['relation_usable'])
        self.assertTrue(q['global_axis_usable'])
        orientation = evidence['orientation']
        self.assertIn('no_count_reduction', orientation['fitting']['uncertainty_basis'])
        self.assertGreater(orientation['relative_axis_variance_rad2'], .0001)
        forged = copy.deepcopy(orientation);forged['relative_axis_variance_rad2'] /= len(boxes)
        with self.assertRaises(ValueError):current_front_relation(forged, o, f['snapshot']['body_pose_enu'])

    def test_clipping_and_non_coplanar_layers_require_new_information(self):
        f, m, l, o, face, boxes, metrics, _ = surface_case()
        self.assertEqual(front_surface_regions([0., .2, .7, .5]), [])
        for i, box in enumerate(boxes):
            u, v = (box[0]+box[2])*320, (box[1]+box[3])*240
            metrics[i] = measured_stereo(f, pixel_surface(f, u, v, 4. if i % 2 else 7.))
        value = estimate_front_surface_frame(f, o, face, boxes, metrics, 'controlled two unsupported depth layers')
        self.assertIsNone(value['orientation'])

    def test_noisy_interior_support_cannot_disappear_in_a_many_point_fit(self):
        from stereo_localization import triangulate_pixel, optical_to_enu
        f, m, l, o, face, boxes, metrics, baseline = surface_case()
        bad = copy.deepcopy(metrics);sample = bad[len(bad)//2]
        quality = sample['quality'];quality['sigma_disparity_px'] = 1.5
        point, covariance = triangulate_pixel(*quality['representative_pixel_uv'], quality['median_disparity_px'],
            sample['calibration'], sigma_disparity_px=1.5)
        position, world_covariance, _ = optical_to_enu(point, covariance, f['snapshot']['body_pose_enu'])
        sample.update(position_camera_optical_m=point, covariance_camera_optical_m2=covariance,
                      position_enu_m=position, covariance_enu_m2=world_covariance)
        quality['depth_sigma_m'] = math.sqrt(covariance[2][2])
        value = estimate_front_surface_frame(f, o, face, boxes, bad, 'controlled noisy interior support')
        self.assertEqual(value['status'], 'available')
        self.assertGreater(value['orientation']['relative_axis_variance_rad2'],
                           baseline['orientation']['relative_axis_variance_rad2']*2)


if __name__ == '__main__':unittest.main()
