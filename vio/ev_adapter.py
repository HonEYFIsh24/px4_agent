"""Transport-independent, fail-closed VINS external-vision adapter (Python 3.8).

VINS publishes the IMU position/orientation and *world* linear velocity, even
when its Odometry child_frame_id says ``body``.  The adapter deliberately uses
that documented source contract rather than forwarding its ROS twist.

Only VINS and the original body IMU are estimator inputs.  Vehicle armed/landed
flags permit initialization/reanchoring; no GNSS, EKF pose or simulator truth is
used to align the estimate.  A configured yaw defines the local axes, and the
first stationary, disarmed optimized body position defines their origin.
"""
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple


Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # x, y, z, w


def finite_vector(value, size, name):
    if len(value) != size or not all(math.isfinite(float(x)) for x in value):
        raise ValueError(name + " must contain finite values")
    return tuple(float(x) for x in value)


def norm(value):
    return math.sqrt(sum(x * x for x in value))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def unit_quaternion(value, tolerance=0.02):
    q = finite_vector(value, 4, "quaternion")
    length = norm(q)
    if length <= 0 or abs(length - 1) > tolerance:
        raise ValueError("orientation is not a unit quaternion")
    return tuple(x / length for x in q)


def conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotation(q):
    x, y, z, w = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def matrix_vector(matrix, vector):
    return tuple(sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3))


def rotate(q, vector):
    return matrix_vector(rotation(q), vector)


def rotate_diagonal(variances, q):
    matrix = rotation(q)
    return tuple(sum(matrix[i][k] * variances[k] * matrix[j][k] for k in range(3))
                 for i in range(3) for j in range(3))


def covariance6(first, second):
    """Combine 3x3 translation and rotation covariances without false zeros on axes."""
    result = [0.0] * 36
    for i in range(3):
        for j in range(3):
            result[i * 6 + j] = first[i * 3 + j]
            result[(i + 3) * 6 + j + 3] = second[i * 3 + j]
    return tuple(result)


@dataclass(frozen=True)
class AdapterConfig:
    world_to_local_yaw_rad: float = 0.0
    # Vector from the body reference point to the original IMU, in body FLU.
    body_to_imu_flu_m: Vec3 = (0.0, 0.0, 0.02)
    imu_to_body_xyzw: Quat = (0.0, 0.0, 0.0, 1.0)
    source_world_frame: str = "world"
    source_imu_frame: str = "uav_imu_link"
    local_frame: str = "vio_local"
    body_frame: str = "base_link"
    optimized_max_age_sim_s: float = 0.5
    optimized_max_receipt_wall_s: float = 3.0
    propagated_max_age_sim_s: float = 0.10
    propagated_max_receipt_wall_s: float = 1.0
    tracking_max_receipt_wall_s: float = 2.0
    gyro_max_offset_sim_s: float = 0.02
    gyro_max_receipt_wall_s: float = 0.5
    future_tolerance_sim_s: float = 0.02
    max_speed_m_s: float = 4.0
    max_angular_speed_rad_s: float = 6.0
    max_position_norm_m: float = 100.0
    position_jump_floor_m: float = 0.35
    orientation_jump_floor_rad: float = 0.25
    stationary_speed_m_s: float = 0.2
    minimum_rate_sim_hz: float = 30.0
    rate_window_sim_s: float = 1.0
    rate_minimum_span_sim_s: float = 0.5
    # These are configured conservative noise assumptions, NOT VINS covariance.
    position_variance_m2: Vec3 = (0.04, 0.04, 0.09)
    orientation_variance_rad2: Vec3 = (0.0076154355, 0.0076154355, 0.0685389195)
    velocity_variance_m2_s2: Vec3 = (0.09, 0.09, 0.16)
    angular_velocity_variance_rad2_s2: Vec3 = (0.01, 0.01, 0.01)
    healthy_quality: int = 1  # binary validity proxy, never a tracking probability

    def __post_init__(self):
        if not math.isfinite(self.world_to_local_yaw_rad):
            raise ValueError("alignment yaw must be finite")
        finite_vector(self.body_to_imu_flu_m, 3, "lever arm")
        unit_quaternion(self.imu_to_body_xyzw)
        for name in ("position_variance_m2", "orientation_variance_rad2",
                     "velocity_variance_m2_s2", "angular_velocity_variance_rad2_s2"):
            if min(finite_vector(getattr(self, name), 3, name)) <= 0:
                raise ValueError(name + " must be strictly positive")
        for name in ("optimized_max_age_sim_s", "optimized_max_receipt_wall_s",
                     "propagated_max_age_sim_s", "propagated_max_receipt_wall_s",
                     "tracking_max_receipt_wall_s", "gyro_max_offset_sim_s",
                     "gyro_max_receipt_wall_s", "future_tolerance_sim_s",
                     "max_speed_m_s", "max_angular_speed_rad_s", "max_position_norm_m",
                     "position_jump_floor_m", "orientation_jump_floor_rad",
                     "stationary_speed_m_s", "minimum_rate_sim_hz", "rate_window_sim_s",
                     "rate_minimum_span_sim_s"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be finite and positive")
        if self.rate_minimum_span_sim_s > self.rate_window_sim_s:
            raise ValueError("rate minimum span exceeds rate window")
        if type(self.healthy_quality) is not int or not 1 <= self.healthy_quality <= 100:
            raise ValueError("healthy quality must be an integer from 1 to 100")
        if not all(isinstance(value, str) and value for value in
                   (self.source_world_frame, self.source_imu_frame, self.local_frame, self.body_frame)):
            raise ValueError("frame names must be nonempty")


@dataclass(frozen=True)
class VioSample:
    stamp_sim_s: float
    position_imu_world_m: Vec3
    orientation_imu_to_world_xyzw: Quat
    velocity_imu_world_m_s: Vec3
    frame_id: str = "world"


@dataclass(frozen=True)
class OutputMeasurement:
    stamp_sim_s: float
    position_local_m: Vec3
    orientation_body_to_local_xyzw: Quat
    velocity_local_m_s: Vec3
    linear_velocity_body_flu_m_s: Vec3
    angular_velocity_body_flu_rad_s: Vec3
    pose_covariance: Tuple[float, ...]
    twist_covariance_body: Tuple[float, ...]
    velocity_covariance_local: Tuple[float, ...]
    angular_velocity_covariance_body: Tuple[float, ...]
    epoch: int
    reset_counter: int
    quality: int
    optimized_stamp_sim_s: float
    source: str = "vins_imu_propagate"
    covariance_source: str = "configured_conservative_model_not_vins_covariance"

    def as_dict(self):
        from dataclasses import asdict
        return asdict(self)


class ExternalVisionAdapter:
    """Deterministic state machine; callers supply both measurement and wall clocks.

    Methods return a new output only on a distinct, accepted propagated IMU
    state. ``status`` never synthesizes or republishes an old measurement.
    Restart, tracking failure, clock rollback or a jump latch a stopped epoch;
    an explicit stationary/disarmed reanchor is needed before any new output.
    """
    def __init__(self, config=None):
        self.config = config or AdapterConfig()
        self.q_local_world = (0.0, 0.0, math.sin(self.config.world_to_local_yaw_rad / 2),
                              math.cos(self.config.world_to_local_yaw_rad / 2))
        self.q_body_imu = unit_quaternion(self.config.imu_to_body_xyzw)
        self.anchor_body_world = None
        self.epoch = 0
        self.latched_reason = None
        self.last_rejection = None
        self.accepted_count = 0
        self.rejected_count = 0
        self.tracking = None
        self.tracking_receipt_mono = None
        self.optimized = None
        self.optimized_receipt_mono = None
        self.propagated = None
        self.propagated_receipt_mono = None
        self.last_output = None
        self.last_sim_now = None
        self.last_wall_now = None
        self.disarmed = None
        self.landed = None
        self.gyro = deque(maxlen=100)
        self.bias_imu = (0.0, 0.0, 0.0)
        self.bias_receipt_mono = None
        self.rate_stamps = deque()
        self.latest_observation = None
        self.latest_observation_receipt_mono = None
        self.propagated_input_error = None
        self.optimized_input_error = None

    def set_vehicle_status(self, disarmed, landed):
        self.disarmed = bool(disarmed)
        self.landed = bool(landed)

    def _reject(self, reason):
        self.last_rejection = reason
        self.rejected_count += 1
        return None

    def _latch(self, reason):
        if self.latched_reason is None:
            self.epoch += 1
            self.latched_reason = reason
            self.rate_stamps.clear()
        return self._reject(reason)

    def _clock(self, now_sim, now_mono):
        if not math.isfinite(now_sim) or now_sim <= 0 or not math.isfinite(now_mono):
            self._reject("clock_not_valid")
            return False
        if self.last_sim_now is not None and now_sim < self.last_sim_now - 1e-6:
            self._latch("simulation_clock_rollback")
            self.last_sim_now = now_sim
            self.last_wall_now = now_mono
            return False
        if self.last_wall_now is not None and now_mono < self.last_wall_now:
            self._latch("monotonic_clock_rollback")
            return False
        self.last_sim_now = now_sim
        self.last_wall_now = now_mono
        return True

    def update_tracking(self, healthy, now_sim, now_mono):
        if not self._clock(now_sim, now_mono):
            return
        self.tracking = bool(healthy)
        self.tracking_receipt_mono = now_mono
        if not self.tracking and self.anchor_body_world is not None:
            self._latch("vins_tracking_failed_or_reset")

    def note_restart(self, now_sim, now_mono):
        self._clock(now_sim, now_mono)
        self.tracking = False
        self.tracking_receipt_mono = now_mono
        self._latch("explicit_vins_restart")

    def update_gyro_bias(self, bias_imu_rad_s, now_mono):
        try:
            value = finite_vector(bias_imu_rad_s, 3, "gyro bias")
            if norm(value) > 1 or not math.isfinite(now_mono):
                raise ValueError("gyro bias outside validity limit")
        except (TypeError, ValueError):
            return False
        self.bias_imu = value
        self.bias_receipt_mono = now_mono
        return True

    def update_gyro(self, stamp_sim_s, angular_velocity_imu_rad_s, frame_id, now_sim, now_mono):
        if not self._clock(now_sim, now_mono):
            return False
        try:
            omega = finite_vector(angular_velocity_imu_rad_s, 3, "IMU angular velocity")
            if frame_id != self.config.source_imu_frame:
                raise ValueError("unexpected IMU frame")
            if not math.isfinite(stamp_sim_s) or stamp_sim_s <= 0:
                raise ValueError("IMU stamp not valid")
            if not -self.config.future_tolerance_sim_s <= now_sim - stamp_sim_s <= self.config.propagated_max_age_sim_s:
                raise ValueError("IMU timestamp not fresh")
            if norm(omega) > self.config.max_angular_speed_rad_s:
                raise ValueError("IMU angular speed exceeds limit")
            if self.gyro and stamp_sim_s <= self.gyro[-1][0]:
                return False
        except (TypeError, ValueError):
            return False
        self.gyro.append((float(stamp_sim_s), omega, now_mono))
        return True

    def _validated_sample(self, sample, previous, now_sim, max_age):
        if sample.frame_id != self.config.source_world_frame:
            raise ValueError("unexpected_vins_world_frame")
        stamp = float(sample.stamp_sim_s)
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError("measurement_timestamp_not_valid")
        if not -self.config.future_tolerance_sim_s <= now_sim - stamp <= max_age:
            raise ValueError("measurement_timestamp_not_fresh")
        p = finite_vector(sample.position_imu_world_m, 3, "position")
        q = unit_quaternion(sample.orientation_imu_to_world_xyzw)
        v = finite_vector(sample.velocity_imu_world_m_s, 3, "velocity")
        if norm(p) > self.config.max_position_norm_m or norm(v) > self.config.max_speed_m_s:
            raise ValueError("measurement_exceeds_position_or_speed_limit")
        if previous is not None:
            delta_t = stamp - previous.stamp_sim_s
            if delta_t == 0:
                raise ValueError("duplicate_measurement_stamp")
            if delta_t < 0:
                self._latch("measurement_clock_regression")
                raise ValueError("measurement_clock_regression")
            displacement = norm(sub(p, previous.position_imu_world_m))
            orientation_delta = 2 * math.acos(min(1, abs(sum(a * b for a, b in
                         zip(q, previous.orientation_imu_to_world_xyzw)))))
            if (displacement > max(self.config.position_jump_floor_m,
                                   self.config.max_speed_m_s * delta_t)
                    or orientation_delta > max(self.config.orientation_jump_floor_rad,
                                               self.config.max_angular_speed_rad_s * delta_t)):
                self._latch("unannounced_estimator_frame_jump")
                raise ValueError("unannounced_estimator_frame_jump")
        return VioSample(stamp, p, q, v, sample.frame_id)

    def _body_pose(self, sample):
        q_world_body = multiply(sample.orientation_imu_to_world_xyzw, conjugate(self.q_body_imu))
        p_world_body = sub(sample.position_imu_world_m,
                           rotate(q_world_body, self.config.body_to_imu_flu_m))
        return p_world_body, q_world_body

    def accept_optimized(self, sample, now_sim, now_mono):
        if not self._clock(now_sim, now_mono):
            return False
        if self.latched_reason:
            self._observe_while_latched(sample, now_sim, now_mono, self.config.optimized_max_age_sim_s)
            return False
        try:
            sample = self._validated_sample(sample, self.optimized, now_sim,
                                             self.config.optimized_max_age_sim_s)
        except (TypeError, ValueError) as error:
            if str(error) != "duplicate_measurement_stamp":
                self.optimized_input_error = str(error)
            self._reject(str(error))
            return False
        self.optimized_input_error = None
        self.latest_observation = sample
        self.latest_observation_receipt_mono = now_mono
        self.optimized = sample
        self.optimized_receipt_mono = now_mono
        if (self.anchor_body_world is None and self.tracking is True
                and self.tracking_receipt_mono is not None
                and now_mono - self.tracking_receipt_mono <= self.config.tracking_max_receipt_wall_s
                and self.disarmed is True and self.landed is True
                and norm(sample.velocity_imu_world_m_s) <= self.config.stationary_speed_m_s):
            self.anchor_body_world = self._body_pose(sample)[0]
        return True

    def _observe_while_latched(self, sample, now_sim, now_mono, max_age):
        # A reset stops all output, but recent raw samples still establish that
        # a later explicitly requested ground reanchor is stationary.
        try:
            observed = self._validated_sample(sample, None, now_sim, max_age)
        except (TypeError, ValueError):
            return
        self.latest_observation = observed
        self.latest_observation_receipt_mono = now_mono

    def _gyro_for(self, stamp, now_mono):
        if not self.gyro:
            return None
        source = min(self.gyro, key=lambda entry: abs(entry[0] - stamp))
        if (abs(source[0] - stamp) > self.config.gyro_max_offset_sim_s
                or now_mono - source[2] > self.config.gyro_max_receipt_wall_s):
            return None
        # If VINS has not yet supplied a bias, raw gyro remains a valid sensor
        # input with the configured conservative angular covariance.
        bias = self.bias_imu if (self.bias_receipt_mono is not None and
                 now_mono - self.bias_receipt_mono <= self.config.optimized_max_receipt_wall_s) else (0, 0, 0)
        return rotate(self.q_body_imu, sub(source[1], bias))

    def _rate(self):
        if len(self.rate_stamps) < 2:
            return 0.0, 0.0
        span = self.rate_stamps[-1] - self.rate_stamps[0]
        return ((len(self.rate_stamps) - 1) / span if span > 0 else 0.0), span

    def _health_reasons(self, now_sim, now_mono):
        c = self.config
        reasons = []
        if self.latched_reason:
            reasons.append(self.latched_reason)
        if self.optimized_input_error:
            reasons.append("optimized_input_invalid: " + self.optimized_input_error)
        if self.propagated_input_error:
            reasons.append("propagated_input_invalid: " + self.propagated_input_error)
        if self.anchor_body_world is None:
            reasons.append("waiting_for_stationary_disarmed_initial_anchor")
        if self.tracking is not True:
            reasons.append("tracking_not_healthy")
        elif self.tracking_receipt_mono is None or now_mono - self.tracking_receipt_mono > c.tracking_max_receipt_wall_s:
            reasons.append("tracking_receipt_stale")
        if self.optimized is None:
            reasons.append("optimized_measurement_missing")
        elif (not -c.future_tolerance_sim_s <= now_sim - self.optimized.stamp_sim_s <= c.optimized_max_age_sim_s
              or now_mono - self.optimized_receipt_mono > c.optimized_max_receipt_wall_s):
            reasons.append("optimized_measurement_stale")
        if self.propagated is None:
            reasons.append("propagated_measurement_missing")
        elif (not -c.future_tolerance_sim_s <= now_sim - self.propagated.stamp_sim_s <= c.propagated_max_age_sim_s
              or now_mono - self.propagated_receipt_mono > c.propagated_max_receipt_wall_s):
            reasons.append("propagated_measurement_stale")
        rate, span = self._rate()
        if span < c.rate_minimum_span_sim_s or rate < c.minimum_rate_sim_hz:
            reasons.append("insufficient_fresh_propagated_rate")
        return reasons

    def accept_propagated(self, sample, now_sim, now_mono):
        if not self._clock(now_sim, now_mono):
            return None
        if self.latched_reason:
            self._observe_while_latched(sample, now_sim, now_mono, self.config.propagated_max_age_sim_s)
            return self._reject(self.latched_reason)
        try:
            sample = self._validated_sample(sample, self.propagated, now_sim,
                                             self.config.propagated_max_age_sim_s)
        except (TypeError, ValueError) as error:
            if str(error) != "duplicate_measurement_stamp":
                self.propagated_input_error = str(error)
            return self._reject(str(error))
        self.propagated_input_error = None
        self.latest_observation = sample
        self.latest_observation_receipt_mono = now_mono
        self.propagated = sample
        self.propagated_receipt_mono = now_mono
        self.rate_stamps.append(sample.stamp_sim_s)
        while self.rate_stamps and sample.stamp_sim_s - self.rate_stamps[0] > self.config.rate_window_sim_s:
            self.rate_stamps.popleft()
        reasons = self._health_reasons(now_sim, now_mono)
        if reasons:
            return self._reject(reasons[0])
        omega_body = self._gyro_for(sample.stamp_sim_s, now_mono)
        if omega_body is None:
            return self._reject("time_aligned_original_body_gyro_missing")
        c = self.config
        p_world_body, q_world_body = self._body_pose(sample)
        q_local_body = multiply(self.q_local_world, q_world_body)
        v_world_body = sub(sample.velocity_imu_world_m_s,
                           rotate(q_world_body, cross(omega_body, c.body_to_imu_flu_m)))
        p_local_body = rotate(self.q_local_world, sub(p_world_body, self.anchor_body_world))
        v_local_body = rotate(self.q_local_world, v_world_body)
        v_body = rotate(conjugate(q_local_body), v_local_body)
        propagation_age = max(0.0, sample.stamp_sim_s - self.optimized.stamp_sim_s)
        p_var = tuple(c.position_variance_m2[i] + propagation_age ** 2 * c.velocity_variance_m2_s2[i]
                      for i in range(3))
        angle_var = tuple(c.orientation_variance_rad2[i] + propagation_age ** 2 * c.angular_velocity_variance_rad2_s2[i]
                          for i in range(3))
        p_cov = rotate_diagonal(p_var, self.q_local_world)
        angle_cov = rotate_diagonal(angle_var, self.q_local_world)
        v_cov = rotate_diagonal(c.velocity_variance_m2_s2, self.q_local_world)
        v_body_cov = rotate_diagonal(c.velocity_variance_m2_s2, conjugate(q_world_body))
        omega_cov = rotate_diagonal(c.angular_velocity_variance_rad2_s2, (0, 0, 0, 1))
        result = OutputMeasurement(sample.stamp_sim_s, p_local_body, q_local_body, v_local_body,
                    v_body, omega_body, covariance6(p_cov, angle_cov),
                    covariance6(v_body_cov, omega_cov), v_cov, omega_cov,
                    self.epoch, self.epoch % 256, c.healthy_quality, self.optimized.stamp_sim_s)
        self.last_output = result
        self.last_rejection = None
        self.accepted_count += 1
        return result

    def reanchor(self, now_sim, now_mono):
        """Explicit ground operation. It never stitches a new airborne frame."""
        if self.disarmed is not True or self.landed is not True:
            return False, "reanchor_requires_landed_and_disarmed"
        if (self.latest_observation is None or self.latest_observation_receipt_mono is None
                or now_mono - self.latest_observation_receipt_mono > self.config.optimized_max_receipt_wall_s
                or not -self.config.future_tolerance_sim_s <= now_sim - self.latest_observation.stamp_sim_s <= self.config.optimized_max_age_sim_s):
            return False, "reanchor_requires_recent_vio_observation"
        if norm(self.latest_observation.velocity_imu_world_m_s) > self.config.stationary_speed_m_s:
            return False, "reanchor_requires_stationary_vio"
        if not math.isfinite(now_sim) or now_sim <= 0 or not math.isfinite(now_mono):
            return False, "reanchor_requires_valid_clock"
        if self.latched_reason is None:
            self.epoch += 1
        self.latched_reason = None
        self.anchor_body_world = None
        self.optimized = None
        self.optimized_receipt_mono = None
        self.propagated = None
        self.propagated_receipt_mono = None
        self.last_output = None
        self.gyro.clear()
        self.rate_stamps.clear()
        self.tracking = None
        self.tracking_receipt_mono = None
        self.last_sim_now = now_sim
        self.last_wall_now = now_mono
        self.last_rejection = None
        self.propagated_input_error = None
        self.optimized_input_error = None
        self.latest_observation = None
        self.latest_observation_receipt_mono = None
        return True, "waiting_for_new_healthy_stationary_vins_epoch"

    def status(self, now_sim, now_mono):
        clock_ok = self._clock(now_sim, now_mono)
        reasons = self._health_reasons(now_sim, now_mono)
        if not clock_ok:
            reasons.insert(0, self.last_rejection or "clock_not_valid")
        if self.propagated is not None and self._gyro_for(self.propagated.stamp_sim_s, now_mono) is None:
            reasons.append("time_aligned_original_body_gyro_missing")
        rate, span = self._rate()
        return {"ready": not reasons, "reasons": reasons, "epoch": self.epoch,
                "reset_counter": self.epoch % 256, "reset_latched": bool(self.latched_reason),
                "quality": self.config.healthy_quality if not reasons else 0,
                "quality_semantics": "binary_health_gate_not_tracking_probability",
                "local_frame": self.config.local_frame, "body_frame": self.config.body_frame,
                "coordinate_semantics": "gravity_aligned_local_FLU_with_configured_yaw",
                "geographic_heading_calibrated": False,
                "world_to_local_yaw_rad": self.config.world_to_local_yaw_rad,
                "body_to_imu_flu_m": self.config.body_to_imu_flu_m,
                "anchor_body_world_m": self.anchor_body_world,
                "optimized_stamp_sim_s": self.optimized.stamp_sim_s if self.optimized else None,
                "propagated_stamp_sim_s": self.propagated.stamp_sim_s if self.propagated else None,
                "last_output_stamp_sim_s": self.last_output.stamp_sim_s if self.last_output else None,
                "rate_sim_hz": rate, "rate_window_span_sim_s": span,
                "accepted_count": self.accepted_count, "rejected_count": self.rejected_count,
                "last_rejection": self.last_rejection,
                "estimator_inputs": ["vins_optimized_odometry", "vins_imu_propagate", "original_body_imu", "vins_gyro_bias"],
                "covariance_source": "configured_conservative_model_not_vins_covariance"}
