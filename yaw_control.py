"""Pure yaw-setpoint policy and arrival evidence; contains no flight I/O.

The bridge must still enforce its own navigation, VIO and mission gates. A
caller cannot treat the returned setpoint or a predicted pose as arrival.
"""
import math


def angle(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('yaw must be finite')
    return math.atan2(math.sin(value), math.cos(value))


def error(target, actual):
    return angle(angle(target)-angle(actual))


class YawArrival:
    """Bounded rotation and fresh-pose dwell tracking for one coordinate epoch."""
    def __init__(self, target_rad, identity, position, *, max_rate_rad_s=.5,
                 tolerance_rad=.1, dwell_s=1., max_drift_m=.35, max_pose_age_s=.5):
        self.target = angle(target_rad)
        if not isinstance(identity, str) or not identity:
            raise ValueError('coordinate identity is required')
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            raise ValueError('initial position is required')
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in position):
            raise ValueError('initial position must be finite')
        for v in (max_rate_rad_s, tolerance_rad, dwell_s, max_drift_m, max_pose_age_s):
            if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
                raise ValueError('yaw limits must be positive and finite')
        self.identity, self.position = identity, tuple(position)
        self.max_rate, self.tolerance = max_rate_rad_s, tolerance_rad
        self.dwell, self.max_drift, self.max_age = dwell_s, max_drift_m, max_pose_age_s
        self.last_stamp, self.last_wall, self.stable_since = None, None, None

    def next_setpoint(self, previous_rad, elapsed_s):
        if type(elapsed_s) not in (int, float) or not math.isfinite(elapsed_s) or not 0 <= elapsed_s <= .5:
            raise ValueError('setpoint interval must be between 0 and 0.5 seconds')
        delta = error(self.target, previous_rad)
        step = max(-self.max_rate*elapsed_s, min(self.max_rate*elapsed_s, delta))
        return angle(previous_rad+step)

    def observe(self, *, actual_rad, position, identity, stamp_s, received_age_s, wall_s):
        actual = angle(actual_rad)
        if identity != self.identity:
            raise ValueError('coordinate identity changed')
        if not isinstance(position, (list, tuple)) or len(position) != 3 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in position):
            raise ValueError('actual position must be finite')
        for v in (stamp_s, received_age_s, wall_s):
            if type(v) not in (int, float) or not math.isfinite(v):
                raise ValueError('pose timing must be finite')
        if (not 0 <= received_age_s <= self.max_age or
                self.last_stamp is not None and stamp_s <= self.last_stamp or
                self.last_wall is not None and (wall_s <= self.last_wall or wall_s-self.last_wall > self.max_age)):
            self.stable_since = None
            raise ValueError('pose is stale, repeated or discontinuous')
        if math.dist(position, self.position) > self.max_drift:
            self.stable_since = None
            raise ValueError('position drift exceeded rotation permission')
        self.last_stamp, self.last_wall = stamp_s, wall_s
        if abs(error(self.target, actual)) > self.tolerance:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = wall_s
        return wall_s-self.stable_since >= self.dwell


def run_rotation(policy, *, read_pose, set_heading, checkpoint, wait, clock, timeout_s=25.):
    """Execute against injected adapters, with no ROS dependency or hidden I/O.

    read_pose returns a dict with actual_rad, position, identity, stamp_s and
    received_age_s. set_heading must preserve the already-held position. Any
    failure propagates to the caller, which owns cancellation and safe hold.
    """
    if type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError('rotation timeout must be positive and finite')
    started = previous = clock()
    checkpoint()
    first = read_pose()
    policy.observe(**first, wall_s=started)
    commanded = angle(first['actual_rad'])
    while True:
        checkpoint()
        now = clock()
        if now < previous or now-started > timeout_s:
            raise ValueError('rotation timed out or clock moved backwards')
        commanded = policy.next_setpoint(commanded, min(now-previous, .5))
        set_heading(commanded)
        previous = now
        pose = read_pose()
        # The control loop can run faster than the sensor. Never count the
        # same sample twice, but still reject stale receipt or changed epoch.
        if pose['identity'] != policy.identity or not 0 <= pose['received_age_s'] <= policy.max_age:
            raise ValueError('coordinate identity changed or pose is stale')
        if pose['stamp_s'] != policy.last_stamp and policy.observe(**pose, wall_s=now):
            return {'status': 'reached', 'target_rad': policy.target,
                    'actual_rad': angle(pose['actual_rad']), 'stamp_s': pose['stamp_s'],
                    'elapsed_s': now-started}
        wait(.05)
