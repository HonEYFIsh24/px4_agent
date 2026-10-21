"""Explicit, bounded Synthetic sensor faults for local SITL reliability tests.

This module never writes a navigation grid, changes clearance, reads scene
objects, or controls aircraft motion. Only an explicitly enabled simulation
controller calls it. All modified samples remain labelled in local evidence.
"""
import copy
import math
import os
import threading
import time
import xmlrpc.client
from urllib.parse import urlsplit


FAULT_PARAMETER = "/uav_agent_executor/reliability_fault"
FAULT_KINDS = ("lidar_drop", "lidar_delay", "lidar_invalid", "lidar_near_obstacle",
               "vio_drop", "vio_tracking_loss", "vio_reset")
MAX_DURATION_S = 30.


class _BoundedTransport(xmlrpc.client.Transport):
    def __init__(self, timeout_s):
        super().__init__()
        self.timeout_s = timeout_s

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout_s
        return connection


def read_ros_fault_parameter(master_uri=None, timeout_s=.2):
    """Read only this local test parameter with a bounded socket timeout.

    Called solely by the fault poll worker, never from controller callbacks.
    A fresh proxy avoids sharing XMLRPC connections across ROS callbacks.
    """
    uri = master_uri or os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311")
    parsed = urlsplit(uri)
    if parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost", "::1"):
        raise RuntimeError("Synthetic fault polling requires a local ROS master")
    with xmlrpc.client.ServerProxy(uri, transport=_BoundedTransport(timeout_s)) as master:
        code, message, value = master.getParam("/uav_agent_executor", FAULT_PARAMETER)
    if code == -1:
        return {}
    if code != 1:
        raise RuntimeError("ROS test parameter read failed: " + str(message))
    return value


def _finite(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + " must be a finite number")
    if not low <= value <= high:
        raise ValueError(name + " is outside its permitted range")
    return float(value)


def fault_request(kind, duration_s, identifier, issued_at=None, range_m=.7,
                  bearing_rad=0., delay_s=2.):
    """Build the exact ROS parameter contract; it contains no obstacle truth."""
    issued_at = time.time() if issued_at is None else issued_at
    return {"id": identifier, "kind": kind, "duration_s": duration_s,
            "issued_at_wall_s": issued_at, "expires_at_wall_s": issued_at + duration_s + 5.,
            "range_m": range_m, "bearing_rad": bearing_rad, "delay_s": delay_s,
            "synthetic": True, "simulation_only": True}


def validate_request(value, wall_now):
    required = {"id", "kind", "duration_s", "issued_at_wall_s", "expires_at_wall_s",
                "range_m", "bearing_rad", "delay_s", "synthetic", "simulation_only"}
    if not isinstance(value, dict) or set(value) != required:
        raise ValueError("fault request must contain the exact test-only contract")
    identifier = value["id"]
    if (not isinstance(identifier, str) or not 1 <= len(identifier) <= 80
            or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in identifier)):
        raise ValueError("fault id must be a safe 1..80 character identifier")
    if value["kind"] not in FAULT_KINDS:
        raise ValueError("unknown Synthetic fault kind")
    if value["synthetic"] is not True or value["simulation_only"] is not True:
        raise ValueError("fault requires explicit Synthetic and simulation_only labels")
    result = copy.deepcopy(value)
    result["duration_s"] = _finite(value["duration_s"], "duration_s", .1, MAX_DURATION_S)
    issued = _finite(value["issued_at_wall_s"], "issued_at_wall_s", 0., 1e12)
    expiry = _finite(value["expires_at_wall_s"], "expires_at_wall_s", 0., 1e12)
    if issued > wall_now + 2. or expiry <= wall_now or not 0 < expiry-issued <= MAX_DURATION_S+5.:
        raise ValueError("fault request is expired, future dated, or has an unbounded TTL")
    result["range_m"] = _finite(value["range_m"], "range_m", .3, 1.)
    result["bearing_rad"] = _finite(value["bearing_rad"], "bearing_rad", -math.pi, math.pi)
    result["delay_s"] = _finite(value["delay_s"], "delay_s", .6, 5.)
    return result


def _label_range(value):
    return value if math.isfinite(value) else ("+inf" if value > 0 else "-inf" if value < 0 else "nan")


class ReliabilityFaults:
    """A ROS-free test adapter with monotonic expiry and replay protection."""

    def __init__(self, enabled=False, simulation_only=False, get_request=None,
                 emit=None, monotonic=None, wall_time=None, background_poll=False):
        self.enabled = bool(enabled)
        if self.enabled and simulation_only is not True:
            raise RuntimeError("Synthetic faults require a verified local simulation")
        self.get_request = get_request
        self.emit = emit or (lambda event, **fields: None)
        self.monotonic = monotonic or time.monotonic
        self.wall_time = wall_time or time.time
        self.lock = threading.RLock()
        self.active = None
        self.pending = None
        self.anchor = None
        self.started = self.deadline = None
        self.last_poll = -float("inf")
        self.last_rejected = None
        self.finished_ids = set()
        self.applied_count = self.dropped_count = 0
        self.error = None
        self.background_poll = bool(background_poll and self.enabled)
        self.cache_lock = threading.Lock()
        self.request_cache = {}
        self.cache_version = 0
        self.processed_cache_version = -1
        self.cache_received = None
        self.poll_duration = self.max_poll_duration = 0.
        self.poll_error = None
        self.poll_stop = threading.Event()
        self.poll_thread = None
        if self.background_poll:
            self.poll_thread = threading.Thread(target=self._poll_loop, name="reliability_parameter_poll", daemon=True)
            self.poll_thread.start()

    @classmethod
    def from_environment(cls, simulation_only, get_request, emit):
        return cls(os.environ.get("UAV_RELIABILITY_TESTING", "").lower() == "true",
                   simulation_only, get_request, emit, background_poll=True)

    def _poll_once(self):
        # No controller, adapter, or cache lock may span a network request.
        started = self.monotonic()
        try:
            value = self.get_request() if self.get_request else {}
            error = None
        except Exception as exc:
            value, error = None, str(exc)
        ended = self.monotonic()
        with self.cache_lock:
            self.poll_duration = max(0., ended-started)
            self.max_poll_duration = max(self.max_poll_duration, self.poll_duration)
            self.poll_error = error
            if error is None:
                self.request_cache = copy.deepcopy(value)
                self.cache_received = ended
                self.cache_version += 1

    def _poll_loop(self):
        while not self.poll_stop.is_set():
            self._poll_once()
            self.poll_stop.wait(.1)

    def close(self, timeout_s=.5):
        self.poll_stop.set()
        if self.poll_thread and self.poll_thread is not threading.current_thread():
            self.poll_thread.join(timeout=max(0., min(timeout_s, .5)))
        return self.poll_thread is None or not self.poll_thread.is_alive()

    def polling_snapshot(self):
        with self.cache_lock:
            return {"background_poll": self.background_poll,
                    "poll_duration_wall_s": self.poll_duration,
                    "max_poll_duration_wall_s": self.max_poll_duration,
                    "cache_age_wall_s": None if self.cache_received is None else max(0., self.monotonic()-self.cache_received),
                    "poll_error": self.poll_error,
                    "poll_thread_alive": bool(self.poll_thread and self.poll_thread.is_alive())}

    def _event(self, event, **fields):
        self.emit("reliability_" + event, synthetic=True, simulation_only=True,
                  wall_time_s=self.wall_time(), monotonic_s=self.monotonic(), **fields)

    def _end(self, reason):
        if self.active:
            self.finished_ids.add(self.active["id"])
            self._event("ended", fault_id=self.active["id"], kind=self.active["kind"],
                        reason=reason, elapsed_wall_s=self.monotonic()-self.started,
                        anchor_xy_enu_m=self.anchor)
        self.active = self.anchor = self.started = self.deadline = None

    def refresh(self, navigation_active=False):
        if not self.enabled:
            return None
        now = self.monotonic()
        if not self.background_poll and now-self.last_poll >= .1:
            self.last_poll = now
            self._poll_once()
        with self.cache_lock:
            value = copy.deepcopy(self.request_cache)
            version, poll_error = self.cache_version, self.poll_error
        with self.lock:
            now = self.monotonic()
            if self.active and now >= self.deadline:
                self._end("automatic_monotonic_expiry")
            if version != self.processed_cache_version and poll_error is None:
                self.processed_cache_version = version
                try:
                    if value in (None, {}):
                        self.pending = None
                        self._end("explicit_clear")
                        self.error = None
                    elif isinstance(value, dict) and value.get("id") in self.finished_ids:
                        self.pending = None
                        self.error = None
                    else:
                        request = validate_request(value, self.wall_time())
                        self.error = None
                        if request["id"] not in self.finished_ids:
                            if self.active and request["id"] != self.active["id"]:
                                self._end("replaced")
                            self.pending = request if self.active is None else None
                        else:
                            self.pending = None
                except Exception as exc:
                    self.error = str(exc)
                    marker = repr(value) if "value" in locals() else type(exc).__name__
                    if marker != self.last_rejected:
                        self.last_rejected = marker
                        self._event("request_rejected", error=self.error)
                    self.pending = None
            if self.active is None and self.pending:
                request = self.pending
                # Ground scans never activate a navigation fault. Pending
                # requests expire without creating a hidden future injection.
                if request["expires_at_wall_s"] <= self.wall_time():
                    self.pending = None
                elif navigation_active or not request["kind"].startswith("lidar_"):
                    self.active, self.pending = request, None
                    self.started = now
                    self.deadline = now + min(request["duration_s"],
                                              request["expires_at_wall_s"]-self.wall_time())
                    self.anchor = None
                    self._event("started", fault_id=request["id"], kind=request["kind"],
                                duration_s=self.deadline-now,
                                source="Synthetic test adapter; actual upstream ROS sensor remains unchanged")
            return copy.deepcopy(self.active)

    def snapshot(self, navigation_active=False):
        active = self.refresh(navigation_active)
        if not self.enabled:
            return None
        with self.lock:
            return {"enabled": True, "synthetic": True, "simulation_only": True,
                    "parameter": FAULT_PARAMETER,
                    "active_fault": active["kind"] if active else None,
                    "fault_id": active["id"] if active else None,
                    "pending_fault": self.pending["kind"] if self.pending else None,
                    "remaining_wall_s": max(0., self.deadline-self.monotonic()) if active else 0.,
                    "anchor_xy_enu_m": copy.copy(self.anchor),
                    "applied_count": self.applied_count, "dropped_count": self.dropped_count,
                    "configuration_error": self.error, **self.polling_snapshot()}

    def transform_scan(self, scan, sensor_pose=None, navigation_active=False):
        active = self.refresh(navigation_active)
        if not active or not active["kind"].startswith("lidar_") or not navigation_active:
            return scan
        with self.lock:
            kind = active["kind"]
            evidence = {"fault_id": active["id"], "kind": kind,
                        "source_topic": "/uav/lidar/scan", "source_stamp_sim_s": scan["stamp"],
                        "source_sample_count": len(scan["ranges"])}
            if kind == "lidar_drop":
                self.dropped_count += 1
                self._event("sample", **evidence, accepted_by_adapter=False, dropped=True)
                return None
            result = copy.deepcopy(scan)
            if kind == "lidar_delay":
                result["stamp"] = max(0., scan["stamp"]-active["delay_s"])
                evidence.update(effective_stamp_sim_s=result["stamp"], delay_s=active["delay_s"])
            elif kind == "lidar_invalid":
                result["ranges"][0] = -float("inf")
                evidence.update(changed_ray_indices=[0], original_ranges=[_label_range(scan["ranges"][0])],
                                effective_ranges=["-inf"])
            elif kind == "lidar_near_obstacle":
                if sensor_pose is None:
                    self._event("sample", **evidence, accepted_by_adapter=False,
                                reason="no time aligned aircraft sensor pose; no injection")
                    return scan
                values = (sensor_pose.get("position_m", [])
                          + sensor_pose.get("orientation_xyzw", []))
                if (len(values) != 7 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
                        or not .975 <= math.hypot(*values[3:]) <= 1.025
                        or abs(sensor_pose.get("stamp", -1.)-scan["stamp"]) > .05):
                    self._event("sample", **evidence, accepted_by_adapter=False,
                                reason="invalid/time-misaligned pose; original scan remains unmodified")
                    return scan
                norm = math.hypot(*values[3:])
                qx, qy, qz, qw = [value/norm for value in values[3:]]
                yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
                origin = sensor_pose["position_m"][:2]
                if self.anchor is None:
                    heading = yaw + active["bearing_rad"]
                    self.anchor = [origin[0]+active["range_m"]*math.cos(heading),
                                   origin[1]+active["range_m"]*math.sin(heading)]
                offset = [self.anchor[0]-origin[0], self.anchor[1]-origin[1]]
                distance = math.hypot(*offset)
                if not max(.3, scan["range_min"]) <= distance < min(5., scan["range_max"]):
                    self._event("sample", **evidence, accepted_by_adapter=False,
                                reason="fixed Synthetic endpoint no longer in valid scan range",
                                anchor_xy_enu_m=self.anchor, distance_m=distance)
                    return scan
                bearing = math.atan2(offset[1], offset[0])-yaw
                indices = sorted(range(len(scan["ranges"])), key=lambda i: abs(math.atan2(
                    math.sin(scan["angle_min"]+i*scan["angle_increment"]-bearing),
                    math.cos(scan["angle_min"]+i*scan["angle_increment"]-bearing))))[:3]
                for index in indices:
                    # A Synthetic return must not erase a nearer real obstacle.
                    result["ranges"][index] = min(scan["ranges"][index], distance)
                evidence.update(changed_ray_indices=indices,
                                original_ranges=[_label_range(scan["ranges"][i]) for i in indices],
                                effective_ranges=[_label_range(result["ranges"][i]) for i in indices],
                                anchor_xy_enu_m=copy.copy(self.anchor),
                                sensor_position_enu_m=copy.copy(sensor_pose["position_m"]),
                                orientation_xyzw=copy.copy(sensor_pose["orientation_xyzw"]),
                                pose_stamp_sim_s=sensor_pose["stamp"])
            self.applied_count += 1
            evidence.setdefault("effective_stamp_sim_s", result["stamp"])
            self._event("sample", **evidence, accepted_by_adapter=True)
            return result

    def vio_decision(self, source, stamp, navigation_active=False):
        active = self.refresh(navigation_active)
        if not active or not active["kind"].startswith("vio_"):
            return None
        kind = active["kind"]
        if kind == "vio_reset":
            # The script invokes the estimator's real /vins_restart callback;
            # its subsequent actual odometry and health are never synthesized.
            return None
        self.applied_count += 1
        dropped = kind == "vio_drop"
        if dropped:
            self.dropped_count += 1
        self._event("sample", fault_id=active["id"], kind=kind, source_topic=source,
                    source_stamp_sim_s=stamp, dropped=dropped,
                    forced_tracking_ok=False if kind == "vio_tracking_loss" else None,
                    used_for_flight_control=False)
        return "drop" if dropped else "tracking_false"
