"""Independent geometry and synthetic calibrated stereo tests, no ROS or object truth."""
import math
import unittest

import stereo_localization as stereo

try:
    import cv2
    import numpy as np
    HAS_CV = True
except ImportError:
    HAS_CV = False


def camera_info(right=False):
    return {"width": 640, "height": 480, "stamp_sim_s": 10.,
            "K": [320., 0., 320., 0., 320., 240., 0., 0., 1.],
            "R": [1., 0., 0., 0., 1., 0., 0., 0., 1.], "D": [0., 0., 0., 0., 0.],
            "P": [320., 0., 320., -38.4 if right else 0., 0., 320., 240., 0., 0., 0., 1., 0.]}


def body_pose():
    return {"position_m": [1., 2., 3.], "orientation_xyzw": [0., 0., 0., 1.],
            "stamp_sim_s": 10., "age_seconds": .1, "frame_id": "map", "body_reference": "base_link"}


def pair():
    return {"left": {"stamp_sim_s": 10., "age_seconds": .1, "camera_intrinsics": camera_info()},
            "right": {"stamp_sim_s": 10., "age_seconds": .1, "camera_intrinsics": camera_info(True)},
            "body_pose_enu": body_pose()}


class GeometryTests(unittest.TestCase):
    def test_invalid_or_relaxed_clock_limits_cannot_bypass_rejection(self):
        for limits in ({"max_pose_offset_s": float("nan")}, {"min_valid_ratio": -1.},
                       {"max_pair_offset_s": .1}, {"max_measurement_age_sim_s": 5.},
                       {"roi_inner_fraction": 1.1}, {"unknown": 1}):
            with self.subTest(limits=limits), self.assertRaises(stereo.StereoLocalizationError):
                stereo.validate_pair(pair(), 10.1, limits)

    def test_projection_baseline_and_static_fallback_are_explicit(self):
        calibration = stereo.calibration_from_camera_info(camera_info(), camera_info(True))
        self.assertAlmostEqual(calibration["baseline_m"], .12)
        self.assertEqual(calibration["baseline_source"], "CameraInfo_P_right")
        left, right = camera_info(), camera_info(True)
        del left["P"], right["P"]
        with self.assertRaisesRegex(stereo.StereoLocalizationError, "baseline requires"):
            stereo.calibration_from_camera_info(left, right)
        calibration = stereo.calibration_from_camera_info(left, right, baseline_m=.12)
        self.assertEqual(calibration["baseline_source"], "known_static_SDF_calibration")

    def test_rejects_reversed_baseline_distortion_and_bad_projection(self):
        left, right = camera_info(), camera_info(True)
        right["P"][3] = 38.4
        with self.assertRaisesRegex(stereo.StereoLocalizationError, "sign/order"):
            stereo.calibration_from_camera_info(left, right)
        right = camera_info(True)
        right["D"][0] = .1
        with self.assertRaisesRegex(stereo.StereoLocalizationError, "rectification"):
            stereo.calibration_from_camera_info(left, right)
        right = camera_info(True)
        right["P"][2] = 321.
        with self.assertRaisesRegex(stereo.StereoLocalizationError, "Projection matrix"):
            stereo.calibration_from_camera_info(left, right)

    def test_same_time_pair_and_pose_are_required_before_any_matching(self):
        self.assertEqual(stereo.validate_pair(pair(), 10.1)[0], 10.)
        for mutate, error in (
                (lambda p: p["right"].update(stamp_sim_s=10.001), "synchronized"),
                (lambda p: p["right"]["camera_intrinsics"].update(stamp_sim_s=9.99), "CameraInfo"),
                (lambda p: p["body_pose_enu"].update(stamp_sim_s=9.94), "aligned"),
                (lambda p: p["body_pose_enu"].update(age_seconds=1.01), "stale"),
                (lambda p: p["body_pose_enu"].update(body_reference="imu_link"), "base_link"),
                (lambda p: p["body_pose_enu"].update(frame_id="vins_world"), "local ENU")):
            with self.subTest(error=error):
                item = pair()
                mutate(item)
                # Keep changed image's CameraInfo time consistent to isolate sync rejection.
                if error == "synchronized":
                    item["right"]["camera_intrinsics"]["stamp_sim_s"] = item["right"]["stamp_sim_s"]
                with self.assertRaisesRegex(stereo.StereoLocalizationError, error):
                    stereo.localize_stereo_target(item, [.3, .3, .7, .7], 10.1)

    def test_measurement_future_stale_and_nonfinite_rejected(self):
        for now in (9.99, 10.51, float("nan")):
            with self.subTest(now=now), self.assertRaises(stereo.StereoLocalizationError):
                stereo.validate_pair(pair(), now)
        item = pair()
        item["left"]["age_seconds"] = 2.01
        with self.assertRaises(stereo.StereoLocalizationError):
            stereo.validate_pair(item, 10.1)

    def test_triangulation_axis_signs_depth_and_uncertainty(self):
        calibration = stereo.calibration_from_camera_info(camera_info(), camera_info(True))
        point, cov = stereo.triangulate_pixel(352., 256., 8., calibration)
        self.assertAlmostEqual(point[0], .48)
        self.assertAlmostEqual(point[1], .24)
        self.assertAlmostEqual(point[2], 4.8)
        self.assertGreater(cov[2][2], .3**2)
        self.assertAlmostEqual(cov[0][2], cov[2][0])
        _, far_cov = stereo.triangulate_pixel(352., 256., 4., calibration)
        self.assertGreater(far_cov[2][2], 10 * cov[2][2])
        for disparity in (0., -1., float("nan")):
            with self.assertRaises(stereo.StereoLocalizationError):
                stereo.triangulate_pixel(320., 240., disparity, calibration)

    def test_optical_mounting_pitch_and_lever_arm_use_base_link_not_imu(self):
        enu, covariance, assumed = stereo.optical_to_enu([0., 0., 5.], [[.01, 0., 0.],
                                                       [0., .01, 0.], [0., 0., .04]], body_pose())
        self.assertAlmostEqual(enu[0], 1.12 + 5 * math.cos(.35))
        self.assertAlmostEqual(enu[1], 2.06)
        self.assertAlmostEqual(enu[2], 3.18 - 5 * math.sin(.35))
        self.assertTrue(assumed)
        self.assertGreater(covariance[1][1], .25**2)
        pose = body_pose()
        pose["orientation_xyzw"] = [0., 0., math.sin(math.pi/4), math.cos(math.pi/4)]
        rotated, _, _ = stereo.optical_to_enu([0., 0., 5.], [[0., 0., 0.]] * 3, pose)
        self.assertAlmostEqual(rotated[0], .94)
        self.assertAlmostEqual(rotated[1], 2.12 + 5 * math.cos(.35))

    def test_covariance_negative_or_indefinite_rejected(self):
        for covariance in ([[1., 2., 0.], [2., 1., 0.], [0., 0., 1.]],
                           [[-1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]):
            with self.assertRaises(stereo.StereoLocalizationError):
                stereo.optical_to_enu([0., 0., 5.], covariance, body_pose())

    def test_explicit_configured_floor_is_still_reported_as_assumed_not_measured_covariance(self):
        pose = body_pose()
        pose.update(position_covariance_m2=[[.09, 0., 0.], [0., .09, 0.], [0., 0., .16]],
                    orientation_covariance_rad2=[[.008, 0., 0.], [0., .008, 0.], [0., 0., .069]],
                    pose_covariance_source="configured_conservative_floor")
        _, _, assumed = stereo.optical_to_enu([0., 0., 5.], [[.01, 0., 0.], [0., .01, 0.], [0., 0., .04]], pose)
        self.assertTrue(assumed)


@unittest.skipUnless(HAS_CV, "OpenCV/numpy stereo tests require ROS Python")
class StereoImageTests(unittest.TestCase):
    def setUp(self):
        self.left = np.random.RandomState(73).randint(0, 256, (480, 640)).astype(np.uint8)
        self.calibration = stereo.calibration_from_camera_info(camera_info(), camera_info(True))
        self.dleft = np.full((480, 640), 8., np.float32)
        self.dright = np.full((480, 640), -8., np.float32)

    def estimate(self, left=None, dleft=None, dright=None, bbox=None):
        return stereo.estimate_from_disparities(self.left if left is None else left,
                                               self.dleft if dleft is None else dleft,
                                               self.dright if dright is None else dright,
                                               [.3, .3, .7, .7] if bbox is None else bbox,
                                               self.calibration, body_pose(), 10.)

    def test_flat_surface_depth_and_covariance_are_measured(self):
        result = self.estimate()
        self.assertAlmostEqual(result["position_camera_optical_m"][2], 4.8)
        self.assertEqual(result["quality"]["lr_consistent_ratio"], 1.)
        self.assertEqual(result["quality"]["valid_ratio"], 1.)
        self.assertTrue(result["pose_uncertainty_assumed"])
        self.assertGreater(result["sigma_enu_m"][0], 0.)

    def test_image_quality_distinguishes_clear_textured_frame_from_blank_black_image(self):
        textured = stereo.image_quality_from_gray(self.left)
        blank = stereo.image_quality_from_gray(np.zeros((480, 640), np.uint8))
        self.assertGreater(textured["laplacian_variance"], 20.)
        self.assertEqual(blank["laplacian_variance"], 0.)
        self.assertEqual(blank["saturated_fraction"], 1.)
        self.assertEqual(blank["mean_luma"], 0.)

    def test_textureless_surface_and_lr_mismatch_are_rejected(self):
        with self.assertRaisesRegex(stereo.StereoLocalizationError, "insufficient texture"):
            self.estimate(left=np.zeros((480, 640), np.uint8))
        with self.assertRaisesRegex(stereo.StereoLocalizationError, "left-right consistency"):
            self.estimate(dright=np.full((480, 640), -3., np.float32))

    def test_invalid_disparities_sparse_support_and_out_of_range_rejected(self):
        for fill in (0., -1., float("nan"), .1, 120.):
            with self.subTest(fill=fill), self.assertRaises(stereo.StereoLocalizationError):
                self.estimate(dleft=np.full((480, 640), fill, np.float32))
        sparse = np.zeros((480, 640), np.float32)
        sparse[230:233, 320:330] = 8.
        with self.assertRaisesRegex(stereo.StereoLocalizationError, "valid pixel fraction"):
            self.estimate(dleft=sparse)

    def test_mixed_depths_and_high_depth_uncertainty_rejected(self):
        left = self.dleft.copy()
        left[:240] = 16.
        right = self.dright.copy()
        right[:240] = -16.
        with self.assertRaisesRegex(stereo.StereoLocalizationError, "ambiguous depths"):
            self.estimate(dleft=left, dright=right)
        with self.assertRaisesRegex(stereo.StereoLocalizationError, "uncertainty"):
            self.estimate(dleft=np.full((480, 640), 3.3, np.float32),
                          dright=np.full((480, 640), -3.3, np.float32))

    def test_sgbm_recovers_independently_shifted_textured_images(self):
        right = np.empty_like(self.left)
        right[:, :-8] = self.left[:, 8:]
        right[:, -8:] = self.left[:, :8]
        item = pair()
        for side, image in (("left", self.left), ("right", right)):
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            self.assertTrue(ok)
            item[side]["jpeg_bytes"] = encoded.tobytes()
        result = stereo.localize_stereo_target(item, [.3, .3, .7, .7], 10.1)
        self.assertAlmostEqual(result["position_camera_optical_m"][2], 4.8, delta=.08)
        self.assertGreater(result["quality"]["valid_ratio"], .8)
        self.assertEqual(set(result["image_sha256"]), {"left", "right"})

    def test_normalized_bbox_must_be_inside_image_and_large_enough(self):
        for bbox in ([-.1, .3, .7, .7], [.3, .3, .3, .7], [.3, .3, 1.01, .7], [.5, .5, .51, .51]):
            with self.subTest(bbox=bbox), self.assertRaises(stereo.StereoLocalizationError):
                self.estimate(bbox=bbox)


if __name__ == "__main__":
    unittest.main()
