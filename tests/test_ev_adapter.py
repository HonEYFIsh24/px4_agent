"""Independent contracts for fresh VINS-to-body estimates and explicit resets."""
import copy
import importlib.util
import json
import math
import struct
import threading
import time
import unittest
from dataclasses import replace

from vio.ev_adapter import (AdapterConfig, ExternalVisionAdapter, VioSample,
                            conjugate, multiply, norm, rotate)
from vio.ev_bridge import TestFaultMonitor, parse_args, validate_fault
from vio.ev_transport import (Mavlink2OdometryEncoder, body_frd_to_local_frd_quaternion,
                              covariance_flu_to_frd, covariance_upper_triangle,
                              flu_to_frd, odometry_fields, packet_ros_fields)


T0 = 10.0
W0 = 100.0


def sample(t, p=(0, 0, .02), q=(0, 0, 0, 1), v=(0, 0, 0)):
    return VioSample(t, p, q, v)


def wall(t):
    return W0 + (t - T0) / .35


def fresh_adapter(config=None, omega=(0, 0, 0), q=(0, 0, 0, 1)):
    adapter = ExternalVisionAdapter(config)
    adapter.set_vehicle_status(True, True)
    adapter.update_tracking(True, T0, W0)
    adapter.accept_optimized(sample(T0, q=q), T0, W0)
    output = None
    for i in range(251):
        t = T0 + i * .004
        adapter.update_gyro(t, omega, "uav_imu_link", t, wall(t))
        if i % 25 == 0:
            adapter.update_tracking(True, t, wall(t))
            adapter.accept_optimized(sample(t, q=q), t, wall(t))
        output = adapter.accept_propagated(sample(t, q=q), t, wall(t)) or output
    assert output is not None
    return adapter, output, T0 + 1


class AdapterContracts(unittest.TestCase):
    def test_configuration_requires_finite_positive_real_covariance_assumptions(self):
        for changes in ({"position_variance_m2": (0, 0, 0)},
                        {"orientation_variance_rad2": (math.nan, 1, 1)},
                        {"world_to_local_yaw_rad": math.inf},
                        {"optimized_max_age_sim_s": -1},
                        {"imu_to_body_xyzw": (0, 0, 0, 2)},
                        {"rate_minimum_span_sim_s": 2},
                        {"healthy_quality": True}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                AdapterConfig(**changes)

    def test_initialization_waits_for_actual_ground_and_disarmed_status(self):
        a = ExternalVisionAdapter()
        a.update_tracking(True, T0, W0)
        self.assertTrue(a.accept_optimized(sample(T0), T0, W0))
        self.assertIsNone(a.anchor_body_world)
        a.set_vehicle_status(False, False)
        a.accept_optimized(sample(T0 + .1), T0 + .1, W0 + .3)
        self.assertIsNone(a.anchor_body_world)
        a.set_vehicle_status(True, True)
        a.accept_optimized(sample(T0 + .2), T0 + .2, W0 + .6)
        self.assertEqual(a.anchor_body_world, (0, 0, 0))

    def test_first_anchor_requires_stationary_vins_velocity(self):
        a = ExternalVisionAdapter()
        a.set_vehicle_status(True, True)
        a.update_tracking(True, T0, W0)
        a.accept_optimized(sample(T0, v=(.3, 0, 0)), T0, W0)
        self.assertIsNone(a.anchor_body_world)

    def test_new_imu_state_uses_original_measurement_stamp_not_receipt_or_old_pose(self):
        a, output, t = fresh_adapter()
        self.assertEqual(output.stamp_sim_s, t)
        self.assertNotEqual(output.stamp_sim_s, wall(t))
        old_count = a.accepted_count
        self.assertIsNone(a.accept_propagated(sample(t), t + .01, wall(t + .01)))
        self.assertEqual(a.accepted_count, old_count)
        self.assertEqual(a.last_output.stamp_sim_s, t)

    def test_low_frequency_optimized_odometry_cannot_be_republished_as_30hz(self):
        a = ExternalVisionAdapter()
        a.set_vehicle_status(True, True)
        result = None
        for i in range(15):
            t = T0 + i * .1
            a.update_tracking(True, t, wall(t))
            a.update_gyro(t, (0, 0, 0), "uav_imu_link", t, wall(t))
            a.accept_optimized(sample(t), t, wall(t))
            result = a.accept_propagated(sample(t), t, wall(t))
        self.assertIsNone(result)
        self.assertAlmostEqual(a.status(t, wall(t))["rate_sim_hz"], 10)

    def test_genuine_250_sim_hz_propagation_passes_rate_gate(self):
        a, output, t = fresh_adapter()
        status = a.status(t, wall(t))
        self.assertTrue(status["ready"], status)
        self.assertAlmostEqual(status["rate_sim_hz"], 250)
        self.assertEqual(output.quality, 1)
        self.assertEqual(output.source, "vins_imu_propagate")

    def test_position_lever_arm_tracks_body_reference_under_roll(self):
        a, _, t = fresh_adapter()
        angle = .12
        q = (math.sin(angle / 2), 0, 0, math.cos(angle / 2))
        # The body remains at its anchored position; only IMU offset rotates.
        t += .004
        imu_position = rotate(q, (0, 0, .02))
        a.update_gyro(t, (0, 0, 0), "uav_imu_link", t, wall(t))
        result = a.accept_propagated(sample(t, p=imu_position, q=q), t, wall(t))
        self.assertIsNotNone(result)
        for x in result.position_local_m:
            self.assertAlmostEqual(x, 0)

    def test_gyro_lever_arm_velocity_removes_rotation_of_imu_about_body(self):
        a, _, t = fresh_adapter(omega=(1, 0, 0))
        t += .004
        a.update_gyro(t, (1, 0, 0), "uav_imu_link", t, wall(t))
        result = a.accept_propagated(sample(t, v=(0, -.02, 0)), t, wall(t))
        self.assertIsNotNone(result)
        self.assertEqual(result.velocity_local_m_s, (0, 0, 0))
        self.assertEqual(result.angular_velocity_body_flu_rad_s, (1, 0, 0))

    def test_world_velocity_is_rotated_into_body_even_when_upstream_child_says_body(self):
        q = (0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        a, _, t = fresh_adapter(q=q)
        t += .004
        a.update_gyro(t, (0, 0, 0), "uav_imu_link", t, wall(t))
        result = a.accept_propagated(sample(t, q=q, v=(1, 0, 0)), t, wall(t))
        self.assertAlmostEqual(result.velocity_local_m_s[0], 1)
        self.assertAlmostEqual(result.linear_velocity_body_flu_m_s[0], 0, places=7)
        self.assertAlmostEqual(result.linear_velocity_body_flu_m_s[1], -1, places=7)

    def test_fixed_alignment_rotates_position_orientation_world_velocity_consistently(self):
        a, _, t = fresh_adapter(AdapterConfig(world_to_local_yaw_rad=math.pi / 2))
        t += .004
        a.update_gyro(t, (0, 0, 0), "uav_imu_link", t, wall(t))
        result = a.accept_propagated(sample(t, p=(.01, 0, .02), v=(1, 0, 0)), t, wall(t))
        self.assertAlmostEqual(result.position_local_m[1], .01)
        self.assertAlmostEqual(result.velocity_local_m_s[1], 1)
        self.assertAlmostEqual(result.linear_velocity_body_flu_m_s[0], 1)
        self.assertAlmostEqual(result.orientation_body_to_local_xyzw[2], math.sin(math.pi / 4))

    def test_fixed_imu_to_body_extrinsic_changes_body_orientation_and_sensor_gyro(self):
        q = (0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        a, _, t = fresh_adapter(AdapterConfig(imu_to_body_xyzw=q), q=q)
        t += .004
        a.update_gyro(t, (1, 0, 0), "uav_imu_link", t, wall(t))
        result = a.accept_propagated(sample(t, q=q), t, wall(t))
        self.assertAlmostEqual(result.orientation_body_to_local_xyzw[3], 1)
        self.assertAlmostEqual(result.angular_velocity_body_flu_rad_s[1], 1)

    def test_estimated_gyro_bias_is_subtracted_and_stale_bias_is_not_reused(self):
        a, _, t = fresh_adapter(omega=(.1, 0, 0))
        self.assertTrue(a.update_gyro_bias((.1, 0, 0), wall(t)))
        t += .004
        a.update_gyro(t, (.1, 0, 0), "uav_imu_link", t, wall(t))
        result = a.accept_propagated(sample(t), t, wall(t))
        self.assertEqual(result.angular_velocity_body_flu_rad_s, (0, 0, 0))
        self.assertFalse(a.update_gyro_bias((math.nan, 0, 0), wall(t)))

    def test_original_body_gyro_must_be_time_aligned_and_have_expected_frame(self):
        a, _, t = fresh_adapter()
        t += .024
        self.assertFalse(a.update_gyro(t, (0, 0, 0), "unknown", t, wall(t)))
        self.assertIsNone(a.accept_propagated(sample(t), t, wall(t)))
        self.assertEqual(a.last_rejection, "time_aligned_original_body_gyro_missing")

    def test_upstream_missing_or_all_zero_covariance_never_becomes_zero_uncertainty(self):
        _, output, _ = fresh_adapter()
        for covariance in (output.pose_covariance, output.twist_covariance_body):
            self.assertTrue(all(covariance[i * 7] > 0 for i in range(6)))
        self.assertIn("not_vins_covariance", output.covariance_source)

    def test_uncertainty_grows_as_propagation_separates_from_optimized_update(self):
        a, at_optimized, t = fresh_adapter()
        t += .08
        a.update_gyro(t, (0, 0, 0), "uav_imu_link", t, wall(t))
        result = a.accept_propagated(sample(t), t, wall(t))
        self.assertGreater(result.pose_covariance[0], at_optimized.pose_covariance[0])

    def test_nonfinite_position_velocity_and_bad_quaternion_stop_output_immediately(self):
        for changes in ({"position_imu_world_m": (math.nan, 0, 0)},
                        {"velocity_imu_world_m_s": (math.inf, 0, 0)},
                        {"orientation_imu_to_world_xyzw": (0, 0, 0, 0)},
                        {"orientation_imu_to_world_xyzw": (0, 0, 0, 2)}):
            a, _, t = fresh_adapter()
            t += .004
            with self.subTest(changes=changes):
                self.assertIsNone(a.accept_propagated(replace(sample(t), **changes), t, wall(t)))
                self.assertFalse(a.status(t, wall(t))["ready"])

    def test_timestamp_zero_nan_future_and_delayed_measurements_are_rejected(self):
        for stamp in (0, math.nan, 15, 9):
            a, _, t = fresh_adapter()
            with self.subTest(stamp=stamp):
                self.assertIsNone(a.accept_propagated(sample(stamp), t + .004, wall(t + .004)))
                self.assertFalse(a.status(t + .004, wall(t + .004))["ready"])

    def test_optimized_and_propagated_clocks_are_monotonic_independently(self):
        a, _, t = fresh_adapter()
        # An optimized result older than the newest IMU state is normal latency.
        self.assertTrue(a.accept_optimized(sample(t + .05), t + .09, wall(t + .09)))
        a.update_gyro(t + .09, (0, 0, 0), "uav_imu_link", t + .09, wall(t + .09))
        self.assertIsNotNone(a.accept_propagated(sample(t + .09), t + .09, wall(t + .09)))

    def test_real_measurement_clock_regression_latches_epoch(self):
        a, _, t = fresh_adapter()
        self.assertIsNone(a.accept_propagated(sample(t - .01), t, wall(t)))
        self.assertTrue(a.status(t, wall(t))["reset_latched"])
        self.assertEqual(a.epoch, 1)

    def test_simulation_clock_rollback_cannot_reuse_previous_epoch(self):
        a, _, t = fresh_adapter()
        status = a.status(1, wall(t) + 1)
        self.assertFalse(status["ready"])
        self.assertEqual(a.latched_reason, "simulation_clock_rollback")

    def test_monotonic_clock_rollback_stops_output(self):
        a, _, t = fresh_adapter()
        self.assertIsNone(a.accept_propagated(sample(t + .004), t + .004, W0))
        self.assertEqual(a.latched_reason, "monotonic_clock_rollback")

    def test_quaternion_sign_change_does_not_look_like_orientation_reset(self):
        a, _, t = fresh_adapter()
        t += .004
        a.update_gyro(t, (0, 0, 0), "uav_imu_link", t, wall(t))
        result = a.accept_propagated(sample(t, q=(0, 0, 0, -1)), t, wall(t))
        self.assertIsNotNone(result)
        self.assertIsNone(a.latched_reason)

    def test_unannounced_translation_or_rotation_jump_is_latched(self):
        for changes in ({"position_imu_world_m": (2, 0, .02)},
                        {"orientation_imu_to_world_xyzw": (0, 0, math.sin(.5), math.cos(.5))}):
            a, _, t = fresh_adapter()
            t += .004
            with self.subTest(changes=changes):
                self.assertIsNone(a.accept_propagated(replace(sample(t), **changes), t, wall(t)))
                self.assertEqual(a.latched_reason, "unannounced_estimator_frame_jump")

    def test_prop_only_dead_reckoning_does_not_keep_stale_visual_solution_healthy(self):
        a, _, t = fresh_adapter()
        for i in range(1, 150):
            now = t + i * .004
            a.update_tracking(True, now, wall(now))
            a.update_gyro(now, (0, 0, 0), "uav_imu_link", now, wall(now))
            output = a.accept_propagated(sample(now), now, wall(now))
        self.assertIsNone(output)
        self.assertIn("optimized_measurement_stale", a.status(now, wall(now))["reasons"])

    def test_healthy_tracking_bool_cannot_mask_stale_optimized_odometry(self):
        a, _, t = fresh_adapter()
        a.update_tracking(True, t + .6, wall(t + .6))
        self.assertFalse(a.status(t + .6, wall(t + .6))["ready"])

    def test_wall_receipt_ttl_expires_even_if_simulation_clock_is_paused(self):
        a, _, t = fresh_adapter()
        self.assertFalse(a.status(t, wall(t) + 4)["ready"])

    def test_tracking_failure_and_restart_require_explicit_ground_reanchor(self):
        for event in ("tracking", "restart"):
            a, _, t = fresh_adapter()
            a.set_vehicle_status(False, False)
            if event == "tracking":
                a.update_tracking(False, t, wall(t))
            else:
                a.note_restart(t, wall(t))
            epoch = a.epoch
            a.update_tracking(True, t, wall(t))
            self.assertIsNone(a.accept_propagated(sample(t + .004), t + .004, wall(t + .004)))
            self.assertFalse(a.reanchor(t + .004, wall(t + .004))[0])
            self.assertEqual(a.epoch, epoch)

    def test_fresh_stationary_post_reset_samples_allow_explicit_ground_reanchor(self):
        a, _, t = fresh_adapter()
        a.note_restart(t, wall(t))
        a.set_vehicle_status(True, True)
        t += .1
        self.assertIsNone(a.accept_propagated(sample(t), t, wall(t)))
        self.assertTrue(a.reanchor(t, wall(t))[0])
        self.assertIsNone(a.anchor_body_world)
        self.assertIsNone(a.last_output)
        self.assertEqual(a.epoch, 1)

    def test_reanchor_rejects_stale_or_moving_vins_data(self):
        a, _, t = fresh_adapter()
        self.assertFalse(a.reanchor(t + 1, wall(t + 1))[0])
        b, _, t = fresh_adapter()
        t += .004
        b.update_gyro(t, (0, 0, 0), "uav_imu_link", t, wall(t))
        b.accept_propagated(sample(t, v=(1, 0, 0)), t, wall(t))
        self.assertFalse(b.reanchor(t, wall(t))[0])

    def test_epoch_counter_wrap_is_explicit_and_never_resets_alignment_automatically(self):
        a, output, t = fresh_adapter()
        a.epoch = 256
        t += .004
        a.update_gyro(t, (0, 0, 0), "uav_imu_link", t, wall(t))
        result = a.accept_propagated(sample(t), t, wall(t))
        self.assertEqual(result.epoch, 256)
        self.assertEqual(result.reset_counter, 0)

    def test_diagnostics_never_claim_geographic_heading_or_estimator_uncertainty(self):
        a, output, t = fresh_adapter()
        status = a.status(t, wall(t))
        self.assertFalse(status["geographic_heading_calibrated"])
        self.assertIn("not_tracking_probability", status["quality_semantics"])
        self.assertFalse(any("gazebo" in x or "gnss" in x for x in status["estimator_inputs"]))
        json.dumps(output.as_dict(), allow_nan=False)


class TransportContracts(unittest.TestCase):
    def test_local_frd_is_same_heading_x_with_right_down_not_geographic_ned(self):
        self.assertEqual(flu_to_frd((1, 2, 3)), (1, -2, -3))
        self.assertEqual(body_frd_to_local_frd_quaternion((0, 0, 0, 1)), (1, 0, 0, 0))

    def test_yaw_is_transformed_between_both_world_and_body_bases(self):
        q = (0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        wxyz = body_frd_to_local_frd_quaternion(q)
        transformed_body_x = rotate((wxyz[1], wxyz[2], wxyz[3], wxyz[0]), (1, 0, 0))
        self.assertAlmostEqual(transformed_body_x[1], -1)

    def test_covariance_sign_change_preserves_anisotropy_cross_terms_and_diagonal(self):
        covariance = [0.0] * 36
        for i in range(6):
            covariance[i * 7] = i + 1
        covariance[1] = covariance[6] = .25
        covariance[2] = covariance[12] = -.1
        converted = covariance_flu_to_frd(covariance)
        self.assertEqual(converted[1], -.25)
        self.assertEqual(converted[2], .1)
        self.assertEqual([converted[i * 7] for i in range(6)], list(range(1, 7)))
        upper = covariance_upper_triangle(converted)
        self.assertEqual(len(upper), 21)
        self.assertEqual(upper[0], 1)
        self.assertEqual(upper[-1], 6)

    def test_covariance_rejects_zero_diagonal_and_asymmetric_values(self):
        _, output, _ = fresh_adapter()
        with self.assertRaises(ValueError):
            covariance_flu_to_frd([0] * 36)
        asymmetric = list(output.pose_covariance)
        asymmetric[1] = 10
        with self.assertRaises(ValueError):
            covariance_flu_to_frd(asymmetric)

    def test_raw_odometry_has_source_time_body_velocity_reset_and_binary_health(self):
        _, output, _ = fresh_adapter()
        output = replace(output, position_local_m=(1, 2, 3), linear_velocity_body_flu_m_s=(.1, .2, .3), epoch=257)
        fields = odometry_fields(output)
        self.assertEqual(fields["time_usec"], round(output.stamp_sim_s * 1e6))
        self.assertEqual((fields["frame_id"], fields["child_frame_id"]), (20, 12))
        self.assertEqual((fields["x"], fields["y"], fields["z"]), (1, -2, -3))
        self.assertEqual((fields["vx"], fields["vy"], fields["vz"]), (.1, -.2, -.3))
        self.assertEqual(fields["reset_counter"], 1)
        self.assertEqual(fields["quality"], 1)
        self.assertEqual(fields["estimator_type"], 3)

    def test_unhealthy_output_cannot_be_encoded(self):
        _, output, _ = fresh_adapter()
        for invalid in (replace(output, quality=0), replace(output, stamp_sim_s=math.nan), replace(output, epoch=-1)):
            with self.subTest(output=invalid), self.assertRaises(ValueError):
                odometry_fields(invalid)

    @unittest.skipUnless(importlib.util.find_spec("pymavlink"), "ROS Python pymavlink is required for packet round trip")
    def test_actual_v2_packet_round_trip_crc_extensions_sequence_and_ros_reconstruction(self):
        from pymavlink.dialects.v20 import common
        _, output, _ = fresh_adapter()
        output = replace(output, epoch=37)
        encoder = Mavlink2OdometryEncoder()
        for sequence in range(258):
            packet = encoder.encode(output)
            fields = packet_ros_fields(packet)
            parser = common.MAVLink(None)
            decoded = parser.parse_char(packet)
            self.assertEqual(decoded.get_type(), "ODOMETRY")
            self.assertEqual(decoded.reset_counter, 37)
            self.assertEqual(decoded.quality, 1)
            self.assertEqual(decoded.estimator_type, 3)
            self.assertEqual(decoded.time_usec, 11_000_000)
            self.assertEqual(fields["seq"], sequence % 256)
            self.assertEqual(fields["magic"], 253)
            self.assertEqual(fields["msgid"], 331)
            payload = struct.pack("<" + "Q" * len(fields["payload64"]), *fields["payload64"])[:fields["len"]]
            self.assertEqual(payload, packet[10:-2])
            self.assertEqual(fields["checksum"], int.from_bytes(packet[-2:], "little"))

    def test_ros_conversion_rejects_wrong_version_or_truncated_packet(self):
        for packet in (b"", b"\xfe" + b"\0" * 20, bytes([253, 20]) + b"\0" * 10):
            with self.subTest(packet=packet), self.assertRaises(ValueError):
                packet_ros_fields(packet)


class TestOnlyFaultContracts(unittest.TestCase):
    def payload(self, kind="drop", identifier="test_1", issued=None):
        issued = time.time() if issued is None else issued
        return {"id": identifier, "kind": kind, "duration_s": .5,
                "issued_wall_s": issued, "expires_wall_s": issued + .5,
                "synthetic": True, "simulation_only": True}

    def test_shadow_is_default_and_no_pymavlink_required_for_import(self):
        self.assertEqual(parse_args([]).mode, "shadow")
        self.assertFalse(parse_args([]).allow_test_faults)

    def test_default_fault_adapter_creates_no_thread_and_never_reads_parameter(self):
        monitor = TestFaultMonitor(reader=lambda: self.fail("default must not read ROS parameter"))
        self.assertIsNone(monitor.thread)
        self.assertIsNone(monitor.status())
        self.assertIsNone(monitor.active())

    def test_test_faults_require_verified_sitl(self):
        with self.assertRaises(RuntimeError):
            TestFaultMonitor(True, False)

    def test_fault_contract_rejects_false_labels_replay_overlong_ttl_and_future(self):
        now = time.time()
        for changes in ({"synthetic": False}, {"simulation_only": False}, {"duration_s": 30},
                        {"kind": "force_position"}, {"expires_wall_s": now + 30},
                        {"issued_wall_s": now + 30}, {"expires_wall_s": now - 1},
                        {"duration_s": True}, {"id": "a/../b"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_fault({**self.payload(issued=now), **changes}, now)

    def test_blocked_parameter_reader_cannot_block_fault_snapshot_or_control(self):
        entered = threading.Event()
        release = threading.Event()
        def reader():
            entered.set()
            release.wait(1)
            return {}
        monitor = TestFaultMonitor(True, True, reader)
        try:
            self.assertTrue(entered.wait(.5))
            began = time.monotonic()
            self.assertIsNone(monitor.active())
            self.assertIsNotNone(monitor.status())
            self.assertLess(time.monotonic() - began, .05)
        finally:
            release.set()
            monitor.close()

    def test_expired_fault_cannot_reactivate_from_unchanged_parameter(self):
        payload = self.payload()
        monitor = TestFaultMonitor(True, True, lambda: payload)
        try:
            deadline = time.monotonic() + 1
            while not monitor.active() and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertIsNotNone(monitor.active())
            monitor.applied(payload["id"])
            self.assertEqual(monitor.status()["applied_count"], 1)
            time.sleep(.6)
            self.assertIsNone(monitor.active())
            self.assertEqual(monitor.status()["applied_count"], 1)
        finally:
            monitor.close()

    def test_parameter_reader_errors_are_reported_without_killing_poll_worker(self):
        def reader():
            raise OSError("local master unavailable")
        monitor = TestFaultMonitor(True, True, reader)
        try:
            deadline = time.monotonic() + .5
            while monitor.status()["error"] is None and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertIn("unavailable", monitor.status()["error"])
            self.assertTrue(monitor.thread.is_alive())
            self.assertIsNone(monitor.active())
        finally:
            monitor.close()


if __name__ == "__main__":
    unittest.main()
