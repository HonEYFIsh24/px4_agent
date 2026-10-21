"""Pure coverage contracts: actual evidence, bounded plans, no truth or flight IO."""
import copy
import math
import unittest

from coverage_search import (CoverageSearch, CoverageSearchError, TargetMemory,
                             plan_search, potential_visible_samples, project_ground_point,
                             navigation_goal_permission, target_region_membership, verified_nominal_footprint)


INFO = {"width": 640, "height": 480, "K": [320., 0., 320., 0., 320., 240., 0., 0., 1.]}
ALLOWED = [-8., -8., 16., 8.]


def plan(**kwargs):
    settings = {"bounds_enu_m": [0., -2., 6., 2.], "spacing_m": 3., "altitude_m": 3.5,
                "camera_info": INFO, "max_observations": 8, "allowed_bounds_enu_m": ALLOWED}
    settings.update(kwargs)
    return plan_search(**settings)


def pose(position, stamp=10.):
    return {"frame_id": "map", "body_reference": "base_link", "position_m": list(position),
            "orientation_xyzw": [0., 0., 0., 1.], "stamp_sim_s": stamp, "age_seconds": .1}


def capture(current, identifier="capture1", stamp=10.):
    return {"capture_id": identifier, "image_sha256": {"left": identifier.encode().hex().ljust(64, 'a'),
                                                       "right": identifier.encode().hex().ljust(64, 'b')},
            "stamp_sim_s": stamp, "age_seconds": .1, "measurement_age_seconds": .03,
            "body_pose_enu": pose(current["position_enu_m"], stamp),
            "image_quality": {"laplacian_variance": 100., "mean_luma": 120., "saturated_fraction": .1}}


def measurement(point, hashes, stamp=10., variance=.04):
    return {"status": "ok", "method": "stereo_sgbm_robust_surface", "frame": "local_enu",
            "position_enu_m": list(point), "image_stamp_sim_s": stamp, "image_sha256": hashes,
            "covariance_enu_m2": [[variance, 0., 0.], [0., variance, 0.], [0., 0., variance]]}


class PlanningTests(unittest.TestCase):
    def test_navigation_goal_permission_uses_actual_max_margin_once_and_refuses_collapsed_map(self):
        result = navigation_goal_permission([-2.4, -7.4, 15.4, 7.4], 2., 1.2)
        self.assertEqual(result["navigation_map_bounds_enu_m"], [-2.4, -7.4, 15.4, 7.4])
        self.assertEqual(result["navigation_goal_boundary_margin_m"], 2.)
        for actual, expected in zip(result["usable_goal_bounds_enu_m"], [-.4, -5.4, 13.4, 5.4]):
            self.assertAlmostEqual(actual, expected)
        self.assertFalse(result["scene_flight_margin_applied_again"])
        with self.assertRaises(CoverageSearchError):
            navigation_goal_permission([0., 0., 2., 2.], 1., 1.2)
        wrong = copy.deepcopy(result)
        wrong["usable_goal_bounds_enu_m"][0] -= .6
        with self.assertRaisesRegex(CoverageSearchError, "boundary clearance"):
            plan(bounds_enu_m=[3., 0., 5., 2.], spacing_m=1., max_observations=4,
                 allowed_bounds_enu_m=wrong["usable_goal_bounds_enu_m"], navigation_permission=wrong)

    def test_target_grid_is_deterministic_serpentine_and_vehicle_views_are_camera_offset(self):
        result = plan()
        self.assertEqual(result, plan())
        self.assertEqual([view["target_center_enu_m"][:2] for view in result["views"]],
                         [[1.5, -1.], [4.5, -1.], [4.5, 1.], [1.5, 1.]])
        self.assertTrue(all(v["position_enu_m"][0] < v["target_center_enu_m"][0] for v in result["views"]))
        self.assertEqual(result["area_m2"], 24.)
        self.assertEqual(sum(view["sample_cell_area_m2"] for view in result["views"]), 24.)
        self.assertFalse(result["generated_target_flight_goals"])

    def test_budget_rejects_large_whole_region_instead_of_truncating_it(self):
        with self.assertRaisesRegex(CoverageSearchError, "whole-region"):
            plan(max_observations=3)

    def test_bounds_radius_altitude_and_fov_spacing_cannot_be_overstated(self):
        for kwargs in ({"bounds_enu_m": [0., -15., 6., 2.]}, {"altitude_m": 2.},
                       {"spacing_m": 7.}, {"allowed_bounds_enu_m": None},
                       {"bounds_enu_m": [29., 29., 30., 30.], "allowed_bounds_enu_m": [0., 0., 30., 30.]}):
            with self.subTest(kwargs=kwargs), self.assertRaises(CoverageSearchError):
                plan(**kwargs)
        footprint = verified_nominal_footprint(INFO, 3.68, 10.)
        self.assertLess(footprint["maximum_spacing_m"], min(footprint["width_m"], footprint["length_m"]))
        self.assertFalse(footprint["occlusion_free_guarantee"])

    def test_actual_heading_changes_geometric_visibility_without_claiming_no_occlusion(self):
        result = plan(bounds_enu_m=[3., -1., 9., 1.])
        state = pose([0., 0., 3.5])
        east = potential_visible_samples(result, state)
        state["orientation_xyzw"] = [0., 0., 1., 0.]
        west = potential_visible_samples(result, state)
        self.assertTrue(east)
        self.assertEqual(west, [])

    def test_actual_mount_offset_and_target_corners_in_fov_at_each_requested_vehicle_point(self):
        result = plan(bounds_enu_m=[0., 0., 2., 2.], spacing_m=1., max_observations=4)
        first = result["views"][0]
        self.assertAlmostEqual(first["position_enu_m"][0], -3.406283784352713)
        self.assertAlmostEqual(first["position_enu_m"][1], .44)
        self.assertEqual(first["target_center_enu_m"], [.5, .5, 0.])
        for view in result["views"]:
            self.assertIn(view["id"], potential_visible_samples(result, view["planned_pose_enu"]))
            self.assertTrue(all(p["inside_fov_and_range"] for p in view["target_cell_corner_projections"]))
            self.assertTrue(all(0 <= p["pixel_uv"][0] < 640 and 0 <= p["pixel_uv"][1] < 480
                                for p in view["target_cell_corner_projections"]))
        # The original body-under-cell behavior cannot see its own target cell.
        under = pose([.5, .5, 3.5])
        self.assertNotIn(first["id"], potential_visible_samples(result, under))

    def test_flight_permission_applies_to_vehicle_and_insufficient_rear_permission_is_not_clipped(self):
        settings = {"bounds_enu_m": [0., 0., 2., 2.], "spacing_m": 1., "max_observations": 4}
        result = plan(allowed_bounds_enu_m=[-5., -2., -1., 3.], **settings)
        self.assertTrue(all(v["position_enu_m"][0] < 0 for v in result["views"]))
        with self.assertRaisesRegex(CoverageSearchError, "cannot clip"):
            plan(allowed_bounds_enu_m=[-2., -2., 3., 3.], **settings)
        with self.assertRaisesRegex(CoverageSearchError, "radius"):
            plan(bounds_enu_m=[0., 0., 2., 2.], spacing_m=1., max_observations=4, flight_radius_m=3.)

    def test_ninety_degree_heading_rotates_both_mount_leverarm_and_observation_offset(self):
        result = plan(bounds_enu_m=[0., 0., 2., 2.], spacing_m=1., planning_yaw_rad=math.pi/2)
        first = result["views"][0]
        self.assertAlmostEqual(first["position_enu_m"][0], .56)
        self.assertAlmostEqual(first["position_enu_m"][1], -3.406283784352713)
        self.assertIn(first["id"], potential_visible_samples(result, first["planned_pose_enu"]))

    def test_actual_acceptance_region_uses_permission_and_camera_geometry_not_target_truth(self):
        permission = navigation_goal_permission([-2.4, -7.4, 15.4, 7.4], 1., 1.2)
        allowed = permission["usable_goal_bounds_enu_m"]
        for region in ([0., 0., 2., 2.], [2., 0., 4., 2.]):
            with self.subTest(region=region), self.assertRaisesRegex(CoverageSearchError, "cannot clip"):
                plan(bounds_enu_m=region, spacing_m=1., max_observations=4,
                     allowed_bounds_enu_m=allowed, navigation_permission=permission)
        result = plan(bounds_enu_m=[3., 0., 5., 2.], spacing_m=1., max_observations=4,
                      allowed_bounds_enu_m=allowed, navigation_permission=permission)
        self.assertEqual(len(result["views"]), 4)
        self.assertAlmostEqual(result["views"][0]["position_enu_m"][0], -.406283784352713)
        self.assertAlmostEqual(result["views"][1]["position_enu_m"][0], .593716215647287)
        for view in result["views"]:
            self.assertIn(view["id"], potential_visible_samples(result, view["planned_pose_enu"]))
            self.assertTrue(all(p["inside_fov_and_range"] for p in view["target_cell_corner_projections"]))


class SearchProgressTests(unittest.TestCase):
    def test_actual_observer_outside_vehicle_permission_cannot_count_even_within_arrival_tolerance(self):
        result = plan(bounds_enu_m=[0., 0., 2., 2.], spacing_m=1., max_observations=4)
        first = result["views"][0]
        result["allowed_bounds_enu_m"][0] = first["position_enu_m"][0]
        search = CoverageSearch(result)
        search.record_navigation_result(first["id"], "succeeded", pose(first["position_enu_m"]))
        item = capture(first)
        item["body_pose_enu"]["position_m"][0] -= .1
        self.assertFalse(search.record_observation(first["id"], item)["accepted"])
        self.assertEqual(search.summary()["observed_views"], 0)
        self.assertIn("flight permission", search.summary()["reason"])

    def test_actual_camera_turning_away_does_not_count_a_clear_image_as_search_coverage(self):
        search = CoverageSearch(plan())
        current = search.next_view()
        search.record_navigation_result(current["id"], "succeeded", pose(current["position_enu_m"]))
        item = capture(current)
        item["body_pose_enu"]["orientation_xyzw"] = [0., 0., 1., 0.]
        self.assertFalse(search.record_observation(current["id"], item)["accepted"])
        self.assertEqual(search.summary()["observed_views"], 0)

    def test_successful_navigation_alone_does_not_count_as_observed(self):
        search = CoverageSearch(plan())
        current = search.next_view()
        search.record_navigation_result(current["id"], "succeeded", pose(current["position_enu_m"]))
        self.assertEqual(search.summary()["observed_views"], 0)
        self.assertEqual(search.summary()["view_sampling_fraction"], 0.)
        self.assertEqual(search.next_view()["id"], current["id"])

    def test_blocked_route_is_terminal_never_counted_or_skipped_as_covered(self):
        search = CoverageSearch(plan())
        current = search.next_view()
        search.record_navigation_result(current["id"], "failed", pose(current["position_enu_m"]))
        self.assertIsNone(search.next_view())
        report = search.summary()
        self.assertEqual(report["status"], "blocked")
        self.assertEqual(report["observed_views"], 0)
        self.assertFalse(report["occlusion_free_coverage_proven"])

    def test_fresh_clear_images_at_actual_points_complete_view_sampling(self):
        search = CoverageSearch(plan())
        for index in range(4):
            current = search.next_view()
            search.record_navigation_result(current["id"], "succeeded", pose(current["position_enu_m"]))
            result = search.record_observation(current["id"], capture(current, "capture" + str(index)))
            self.assertTrue(result["accepted"])
        report = search.summary()
        self.assertEqual(report["status"], "sampling_complete")
        self.assertEqual(report["view_sampling_fraction"], 1.)
        self.assertEqual(report["sampled_region_area_m2"], 24.)
        self.assertFalse(report["occlusion_free_coverage_proven"])

    def test_invalid_image_quality_timestamp_pose_or_duplicate_never_counts_view(self):
        for change in (lambda c: c.update(age_seconds=3.),
                       lambda c: c.update(measurement_age_seconds=.6),
                       lambda c: c["image_quality"].update(laplacian_variance=0.),
                       lambda c: c["image_quality"].update(saturated_fraction=.99),
                       lambda c: c["body_pose_enu"].update(stamp_sim_s=9.),
                       lambda c: c["body_pose_enu"].update(age_seconds=2.),
                       lambda c: c["body_pose_enu"].update(position_m=[12., 12., 3.5])):
            with self.subTest(change=change):
                search = CoverageSearch(plan())
                current = search.next_view()
                search.record_navigation_result(current["id"], "succeeded", pose(current["position_enu_m"]))
                item = capture(current)
                change(item)
                self.assertFalse(search.record_observation(current["id"], item)["accepted"])
                self.assertEqual(search.summary()["observed_views"], 0)
                self.assertIsNone(search.next_view())

    def test_wall_budget_and_operator_cancel_stop_without_issuing_actions(self):
        clock = [0.]
        search = CoverageSearch(plan(max_duration_wall_s=10.), clock=lambda: clock[0])
        clock[0] = 10.
        self.assertIsNone(search.next_view())
        self.assertEqual(search.summary()["status"], "budget_exhausted")
        search = CoverageSearch(plan())
        search.cancel()
        self.assertIsNone(search.next_view())
        self.assertEqual(search.summary()["status"], "cancelled")

    def test_target_measurements_must_belong_to_actual_capture(self):
        search = CoverageSearch(plan())
        current = search.next_view()
        search.record_navigation_result(current["id"], "succeeded", pose(current["position_enu_m"]))
        item = capture(current)
        wrong = measurement([5., 0., 0.], {"left": 'c'*64, "right": 'd'*64})
        self.assertFalse(search.record_observation(current["id"], item,
                                                 [{"label": "red container", "measurement": wrong}])["accepted"])
        self.assertEqual(search.summary()["targets"], [])

    def test_capture_epoch_change_and_target_epoch_mismatch_never_count_an_observation(self):
        epoch = {"vio_epoch": 0, "ekf_counters": {"xy": 0, "z": 0, "heading": 0}}
        search = CoverageSearch(plan())
        first = search.next_view()
        search.record_navigation_result(first["id"], "succeeded", pose(first["position_enu_m"]))
        item = capture(first, "first")
        item["coordinate_epoch"] = copy.deepcopy(epoch)
        self.assertTrue(search.record_observation(first["id"], item)["accepted"])
        second = search.next_view()
        search.record_navigation_result(second["id"], "succeeded", pose(second["position_enu_m"]))
        # Epoch evidence disappears, despite the camera and pose still looking valid.
        self.assertFalse(search.record_observation(second["id"], capture(second, "second"))["accepted"])
        self.assertEqual(search.summary()["observed_views"], 1)
        search = CoverageSearch(plan())
        current = search.next_view()
        search.record_navigation_result(current["id"], "succeeded", pose(current["position_enu_m"]))
        item = capture(current)
        item["coordinate_epoch"] = epoch
        target = measurement([5., 0., 0.], item["image_sha256"])
        self.assertFalse(search.record_observation(current["id"], item,
                                                 [{"label": "container", "measurement": target}])["accepted"])
        self.assertEqual(search.summary()["targets"], [])

    def test_canonical_same_epoch_target_is_recorded_without_scalar_fallback(self):
        epoch = {"vio_epoch": 0, "ekf_counters": {"xy": 0, "z": 0, "heading": 0}}
        for identity in (epoch, 0):
            with self.subTest(identity=identity):
                search = CoverageSearch(plan())
                current = search.next_view()
                search.record_navigation_result(current["id"], "succeeded", pose(current["position_enu_m"]))
                item = capture(current)
                item["coordinate_epoch"] = identity
                target = measurement([5., 0., 0.], item["image_sha256"])
                target["coordinate_epoch"] = identity
                result = search.record_observation(current["id"], item, [{"label": "container", "measurement": target}])
                self.assertEqual(result["accepted"], isinstance(identity, dict))
                if isinstance(identity, dict):
                    self.assertEqual(search.summary()["targets"][0]["coordinate_epoch"], identity)

    def test_invalid_later_target_does_not_leave_partial_report_memory(self):
        search = CoverageSearch(plan())
        current = search.next_view()
        search.record_navigation_result(current["id"], "succeeded", pose(current["position_enu_m"]))
        item = capture(current)
        valid = measurement([5., 0., 0.], item["image_sha256"])
        invalid = measurement([5., 1., 0.], item["image_sha256"], variance=-1.)
        result = search.record_observation(current["id"], item, [
            {"label": "container", "measurement": valid}, {"label": "barrel", "measurement": invalid}])
        self.assertFalse(result["accepted"])
        self.assertEqual(search.summary()["observed_views"], 0)
        self.assertEqual(search.summary()["targets"], [])


class TargetMemoryTests(unittest.TestCase):
    def test_region_membership_uses_uncertainty_envelope_without_claiming_calibrated_probability(self):
        bounds = [0., 0., 2., 2.]
        for point, variance, expected in (([1., 1., 0.], .01, "inside_with_configured_uncertainty_margin"),
                                          ([1., 1., 0.], 1., "uncertain_at_region_boundary"),
                                          ([2.1, 1., 0.], .01, "outside_region")):
            membership = target_region_membership(measurement(point, {"left": 'a'*64, "right": 'b'*64}, variance=variance), bounds)
            self.assertEqual(membership["status"], expected)
            self.assertFalse(membership["probability_calibrated"])

    def test_outside_region_high_uncertainty_or_wrong_query_two_view_targets_do_not_stop_search(self):
        for point, variance, label in (([7., 0., 0.], .01, "red container"),
                                       ([5., 0., 0.], 1., "red container"),
                                       ([5., 0., 0.], .01, "blue container")):
            with self.subTest(point=point, variance=variance, label=label):
                search = CoverageSearch(plan(), target_label="red container")
                for index in range(2):
                    current = search.next_view()
                    search.record_navigation_result(current["id"], "succeeded", pose(current["position_enu_m"]))
                    item = capture(current, "region"+str(index))
                    result = search.record_observation(current["id"], item, [{"label": label,
                        "measurement": measurement(point, item["image_sha256"], variance=variance)}])
                    self.assertTrue(result["accepted"])
                report = search.summary()
                self.assertEqual(report["status"], "running")
                self.assertEqual(report["observed_views"], 2)
                self.assertEqual(report["targets"][0]["distinct_viewpoint_count"], 2)
                self.assertFalse(report["target_identity_ground_truth_verified"])
                self.assertIsNotNone(search.next_view())

    def test_changed_or_lost_coordinate_evidence_cannot_mix_target_memory(self):
        known = {"vio_epoch": 0, "ekf_counters": {"xy": 0, "z": 0, "heading": 0}}
        changed = {"vio_epoch": 0, "ekf_counters": {"xy": 1, "z": 0, "heading": 0}}
        for first_epoch, second_epoch in ((None, known), (known, None), (known, changed), (known, 0)):
            with self.subTest(first=first_epoch, second=second_epoch):
                memory = TargetMemory()
                first = measurement([5., 0., 0.], {"left": 'a'*64, "right": 'b'*64})
                first["coordinate_epoch"] = first_epoch
                memory.observe("container", first, "first", pose([0., 0., 3.5]))
                before = memory.reports()
                second = measurement([5.1, 0., 0.], {"left": 'c'*64, "right": 'd'*64})
                second["coordinate_epoch"] = second_epoch
                with self.assertRaises(CoverageSearchError):
                    memory.observe("container", second, "second", pose([1., 0., 3.5]))
                self.assertEqual(memory.reports(), before)

    def test_multi_view_fusion_retains_shared_uncertainty_and_disagreement_scatter(self):
        memory = TargetMemory()
        hashes1, hashes2 = {"left": 'a'*64, "right": 'b'*64}, {"left": 'c'*64, "right": 'd'*64}
        memory.observe("red container", measurement([5., 0., 0.], hashes1), "first", pose([0., 0., 3.5]))
        memory.observe("red container", measurement([5.2, 0., 0.], hashes2), "second", pose([1., 0., 3.5]))
        target = memory.reports()[0]
        self.assertAlmostEqual(target["position_enu_m"][0], 5.1)
        self.assertEqual(target["distinct_viewpoint_count"], 2)
        self.assertGreater(target["covariance_enu_m2"][0][0], .04)
        self.assertEqual(target["covariance_enu_m2"][1][1], .04)
        self.assertFalse(target["error_statistics_calibrated"])
        self.assertFalse(target["generated_flight_goal"])

    def test_duplicate_and_stationary_repeated_frames_do_not_create_distinct_views(self):
        memory = TargetMemory()
        hashes = {"left": 'a'*64, "right": 'b'*64}
        item = measurement([5., 0., 0.], hashes)
        memory.observe("barrel", item, "first", pose([0., 0., 3.5]))
        self.assertFalse(memory.observe("barrel", item, "first", pose([0., 0., 3.5]))["accepted"])
        second = measurement([5., 0., 0.], {"left": 'c'*64, "right": 'd'*64})
        memory.observe("barrel", second, "second", pose([0., 0., 3.5]))
        self.assertEqual(memory.reports()[0]["distinct_viewpoint_count"], 1)

    def test_two_same_label_instances_in_one_frame_are_not_dropped_or_merged(self):
        memory = TargetMemory()
        hashes = {"left": 'a'*64, "right": 'b'*64}
        memory.observe("barrel", measurement([5., 0., 0.], hashes), "first", pose([0., 0., 3.5]))
        memory.observe("barrel", measurement([5., 1., 0.], hashes), "first", pose([0., 0., 3.5]))
        self.assertEqual(len(memory.reports()), 2)

    def test_non_stereo_fake_frame_stale_pose_and_memory_overflow_rejected(self):
        hashes = {"left": 'a'*64, "right": 'b'*64}
        for modify in (lambda m: m.update(method="gazebo_ground_truth"), lambda m: m.update(frame="gazebo_world")):
            item = measurement([5., 0., 0.], hashes)
            modify(item)
            with self.assertRaises(CoverageSearchError):
                TargetMemory().observe("barrel", item, "first", pose([0., 0., 3.5]))
        memory = TargetMemory(max_targets=1)
        memory.observe("barrel", measurement([5., 0., 0.], hashes), "first", pose([0., 0., 3.5]))
        with self.assertRaisesRegex(CoverageSearchError, "memory is full"):
            memory.observe("container", measurement([5., 0., 0.], hashes), "first", pose([0., 0., 3.5]))

    def test_requested_target_ends_after_required_real_viewpoints_and_preserves_unvisited_samples(self):
        search = CoverageSearch(plan(), target_label="red container")
        for index in range(2):
            current = search.next_view()
            search.record_navigation_result(current["id"], "succeeded", pose(current["position_enu_m"]))
            item = capture(current, "frame"+str(index))
            measured = measurement([5., 0., 0.], item["image_sha256"])
            accepted = search.record_observation(current["id"], item,
                                                 [{"label": "red container", "measurement": measured}])
            self.assertTrue(accepted["accepted"])
        report = search.summary()
        self.assertEqual(report["status"], "target_found")
        self.assertEqual(report["view_sampling_fraction"], .5)
        self.assertEqual(list(report["states"].values()).count("pending"), 2)
        self.assertIsNone(search.next_view())


if __name__ == "__main__":
    unittest.main()
