#!/usr/bin/env python3
"""ROS wrapper for the independent external-vision adapter; shadow by default."""
import argparse
import json
import math
import os
import sys
import threading
import time
import xmlrpc.client
from collections import deque
from pathlib import Path
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from vio.ev_adapter import AdapterConfig, ExternalVisionAdapter, VioSample
from vio.ev_transport import Mavlink2OdometryEncoder, packet_ros_fields


FAULT_PARAMETER = "/uav_ev_bridge/test_fault"


class BoundedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 0.2
        return connection


def read_fault_parameter():
    uri = os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311")
    parsed = urlsplit(uri)
    if parsed.scheme != "http" or parsed.hostname not in ("localhost", "127.0.0.1", "::1"):
        raise RuntimeError("EV test fault polling requires a local ROS master")
    with xmlrpc.client.ServerProxy(uri, transport=BoundedTransport()) as master:
        code, message, value = master.getParam("/uav_ev_bridge", FAULT_PARAMETER)
    if code == -1:
        return {}
    if code != 1:
        raise RuntimeError("cannot read EV fault parameter: " + str(message))
    return value


def validate_fault(value, wall_now):
    required = {"id", "kind", "duration_s", "issued_wall_s", "expires_wall_s", "synthetic", "simulation_only"}
    if not isinstance(value, dict) or set(value) != required:
        raise ValueError("invalid EV fault payload contract")
    if value["kind"] not in ("drop", "delay") or value["synthetic"] is not True or value["simulation_only"] is not True:
        raise ValueError("EV fault must be explicitly Synthetic and simulation_only")
    identifier = value["id"]
    if (not isinstance(identifier, str) or not 1 <= len(identifier) <= 80
            or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in identifier)):
        raise ValueError("fault id is invalid")
    for name in ("duration_s", "issued_wall_s", "expires_wall_s"):
        x = value[name]
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
            raise ValueError(name + " must be a finite number")
    if not 0.1 <= value["duration_s"] <= 5:
        raise ValueError("fault duration must be between 0.1 and 5 wall seconds")
    issued, expiry = value["issued_wall_s"], value["expires_wall_s"]
    if not 0 <= issued <= wall_now + 0.5 or expiry <= wall_now or not 0 < expiry - issued <= 5:
        raise ValueError("fault TTL is expired, future or exceeds 5 wall seconds")
    if expiry - issued + 1e-6 < value["duration_s"]:
        raise ValueError("fault TTL is shorter than its duration")
    return dict(value)


class TestFaultMonitor:
    """Independent bounded XMLRPC worker; no network under adapter/callback locks."""
    def __init__(self, enabled=False, verified_sitl=False, reader=None):
        self.enabled = bool(enabled)
        if self.enabled and verified_sitl is not True:
            raise RuntimeError("EV test faults require verified local SITL")
        self.reader = reader or read_fault_parameter
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.request = None
        self.seen_ids = set()
        self.deadline = None
        self.applied_count = 0
        self.last_applied_wall_s = None
        self.error = None
        self.poll_duration_s = 0
        self.thread = None
        if self.enabled:
            self.thread = threading.Thread(target=self._poll, name="ev_test_parameter_poll", daemon=True)
            self.thread.start()

    def _poll(self):
        while not self.stop_event.is_set():
            started = time.monotonic()
            raw = None
            try:
                raw = self.reader()
                parsed = validate_fault(raw, time.time()) if raw else None
                error = None
            except Exception as exc:
                parsed, error = None, str(exc)
            ended = time.monotonic()
            with self.lock:
                self.poll_duration_s = ended - started
                self.error = error
                if parsed and parsed["id"] not in self.seen_ids:
                    self.seen_ids.add(parsed["id"])
                    self.request = parsed
                    self.deadline = ended + min(parsed["duration_s"], parsed["expires_wall_s"] - time.time())
                    self.applied_count = 0
                    self.last_applied_wall_s = None
                elif raw == {}:
                    self.request = None
                    self.deadline = None
            self.stop_event.wait(0.1)

    def active(self):
        if not self.enabled:
            return None
        with self.lock:
            if not self.request or self.deadline is None or time.monotonic() >= self.deadline:
                return None
            return dict(self.request)

    def applied(self, identifier):
        with self.lock:
            if self.request and self.request["id"] == identifier:
                self.applied_count += 1
                self.last_applied_wall_s = time.time()

    def status(self):
        if not self.enabled:
            return None
        with self.lock:
            active = bool(self.request and self.deadline is not None and time.monotonic() < self.deadline)
            return {"enabled": True, "synthetic": True, "simulation_only": True,
                    "active": active, "id": self.request["id"] if self.request else None,
                    "kind": self.request["kind"] if self.request else None,
                    "applied_count": self.applied_count, "last_applied_wall_s": self.last_applied_wall_s,
                    "remaining_wall_s": max(0, self.deadline - time.monotonic()) if self.deadline else 0,
                    "poll_duration_s": self.poll_duration_s, "error": self.error}

    def close(self):
        self.stop_event.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(0.5)


def verify_local_sitl(rospy):
    """Same endpoint/world guard as the existing executor, without estimator truth."""
    from gazebo_msgs.srv import GetWorldProperties
    if rospy.get_param("/use_sim_time", False) is not True:
        raise RuntimeError("external vision fusion requires /use_sim_time=true")
    if rospy.get_param("/mavros/fcu_url", "") != "udp://:14540@localhost:14557":
        raise RuntimeError("external vision fusion requires the project's local SITL endpoint")
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=15)
    result = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
    if not result.success or "iris" not in result.model_names:
        raise RuntimeError("external vision fusion requires the running Gazebo Iris")
    return True


def sample_from_ros(message):
    p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist.linear
    return VioSample(message.header.stamp.to_sec(), (p.x, p.y, p.z),
                     (q.x, q.y, q.z, q.w), (v.x, v.y, v.z), message.header.frame_id)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("shadow", "fuse"), default="shadow")
    parser.add_argument("--world-to-local-yaw-rad", type=float, default=0)
    parser.add_argument("--publish-rate-sim-hz", type=float, default=50)
    parser.add_argument("--allow-test-faults", action="store_true")
    parser.add_argument("--evidence-dir", type=Path)
    args = parser.parse_args(argv)
    if not math.isfinite(args.world_to_local_yaw_rad):
        parser.error("alignment yaw must be finite")
    if not math.isfinite(args.publish_rate_sim_hz) or not 30 <= args.publish_rate_sim_hz <= 100:
        parser.error("publish rate must be between 30 and 100 simulation Hz")
    return args


def main(argv=None):
    args = parse_args(argv)
    import rospy
    from mavros_msgs.msg import ExtendedState, Mavlink, State
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool, Float64MultiArray, String
    from std_srvs.srv import Trigger, TriggerResponse

    rospy.init_node("uav_ev_bridge", disable_signals=False)
    verified_sitl = verify_local_sitl(rospy) if args.mode == "fuse" or args.allow_test_faults else False
    encoder = Mavlink2OdometryEncoder() if args.mode == "fuse" else None
    adapter = ExternalVisionAdapter(AdapterConfig(world_to_local_yaw_rad=args.world_to_local_yaw_rad))
    faults = TestFaultMonitor(args.allow_test_faults, verified_sitl)
    rospy.on_shutdown(faults.close)
    lock = threading.RLock()
    state = {"armed": None, "landed": None, "connected": False, "state_mono": None,
             "forwarded_count": 0, "shadow_count": 0, "last_forwarded_sim_s": None,
             "last_forwarded_wall_s": None, "last_shadow_sim_s": None,
             "last_publish_sim_s": None, "transport_error": None, "fault_active": None}
    delayed_history = deque(maxlen=1000)
    shadow = rospy.Publisher("/uav/vio/ev_shadow", Odometry, queue_size=10)
    timing = rospy.Publisher("/uav/vio/ev_timing", String, queue_size=10)
    statuses = rospy.Publisher("/uav/vio/ev_status", String, queue_size=1, latch=True)
    transport = rospy.Publisher("/mavlink/to", Mavlink, queue_size=10) if encoder else None
    evidence = None
    if args.evidence_dir:
        args.evidence_dir.mkdir(parents=True, exist_ok=False)
        evidence = (args.evidence_dir / "adapter.jsonl").open("x", buffering=1)
        rospy.on_shutdown(evidence.close)

    def emit(kind, payload):
        if evidence:
            evidence.write(json.dumps({"event": kind, "wall_s": time.time(), **payload}, allow_nan=False) + "\n")

    def now():
        return rospy.Time.now().to_sec(), time.monotonic()

    def vehicle(message):
        with lock:
            state.update(armed=bool(message.armed), connected=bool(message.connected), state_mono=time.monotonic())
            adapter.set_vehicle_status(message.armed is False, state["landed"] is True)

    def extended(message):
        with lock:
            state["landed"] = message.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
            adapter.set_vehicle_status(state["armed"] is False, state["landed"])

    def tracking(message):
        with lock:
            adapter.update_tracking(message.data, *now())

    def restart(message):
        if message.data:
            with lock:
                adapter.note_restart(*now())
                emit("restart_latched", {"epoch": adapter.epoch})

    def bias(message):
        if len(message.data) == 6:
            with lock:
                adapter.update_gyro_bias(message.data[3:], time.monotonic())

    def gyro(message):
        w = message.angular_velocity
        with lock:
            adapter.update_gyro(message.header.stamp.to_sec(), (w.x, w.y, w.z), message.header.frame_id, *now())

    def optimized(message):
        with lock:
            adapter.accept_optimized(sample_from_ros(message), *now())

    def publish_shadow(output):
        message = Odometry()
        message.header.stamp = rospy.Time.from_sec(output.stamp_sim_s)
        message.header.frame_id = adapter.config.local_frame
        message.child_frame_id = adapter.config.body_frame
        p, q = message.pose.pose.position, message.pose.pose.orientation
        p.x, p.y, p.z = output.position_local_m
        q.x, q.y, q.z, q.w = output.orientation_body_to_local_xyzw
        message.pose.covariance = list(output.pose_covariance)
        v, w = message.twist.twist.linear, message.twist.twist.angular
        v.x, v.y, v.z = output.linear_velocity_body_flu_m_s
        w.x, w.y, w.z = output.angular_velocity_body_flu_rad_s
        message.twist.covariance = list(output.twist_covariance_body)
        publish_clock = rospy.Time.now().to_sec()
        publish_wall_before = time.time()
        publish_monotonic_before = time.monotonic()
        shadow.publish(message)
        publish_monotonic_after = time.monotonic()
        publish_wall_after = time.time()
        timing.publish(String(data=json.dumps({"stamp_sim_s": message.header.stamp.to_sec(),
            "stamp_ns": message.header.stamp.to_nsec(), "actual_publish_clock_sim_s": publish_clock,
            "measurement_age_at_publish_sim_s": publish_clock-message.header.stamp.to_sec(),
            "shadow_publish_wall_before_s": publish_wall_before,
            "shadow_publish_wall_after_s": publish_wall_after,
            "shadow_publish_monotonic_before_s": publish_monotonic_before,
            "shadow_publish_monotonic_after_s": publish_monotonic_after,
            "monotonic_clock_scope": "same_simulation_host_not_cross_machine",
            "publish_semantics": "rospy_publish_call_not_socket_delivery",
            "epoch": adapter.epoch, "mode": args.mode, "source": "producer_ros_clock_at_publish",
            "source_timestamp_unchanged": True}, allow_nan=False)))
        state["shadow_count"] += 1
        state["last_shadow_sim_s"] = output.stamp_sim_s

    def propagated(message):
        with lock:
            now_sim, now_mono = now()
            actual_sample = sample_from_ros(message)
            delayed_history.append(actual_sample)
            fault = faults.active()
            state["fault_active"] = fault
            sample = actual_sample
            if fault and fault["kind"] == "delay":
                # Deliver a historical real source state. Its source timestamp
                # remains untouched: this exercises rejection of actual stale data.
                candidates = [s for s in delayed_history if now_sim - s.stamp_sim_s >= 0.6]
                if candidates:
                    sample = candidates[-1]
                    faults.applied(fault["id"])
            output = adapter.accept_propagated(sample, now_sim, now_mono)
            if fault and fault["kind"] == "delay" and sample is not actual_sample:
                emit("fault_applied", {"id": fault["id"], "kind": "delay", "synthetic": True,
                        "actual_new_source_stamp_sim_s": actual_sample.stamp_sim_s,
                        "delivered_historical_source_stamp_sim_s": sample.stamp_sim_s,
                        "adapter_accepted": output is not None, "adapter_rejection": adapter.last_rejection})
            if output is None:
                return
            previous_publish = state["last_publish_sim_s"]
            if previous_publish is not None and output.stamp_sim_s - previous_publish < 1 / args.publish_rate_sim_hz - 1e-6:
                return
            state["last_publish_sim_s"] = output.stamp_sim_s
            publish_shadow(output)
            emit("measurement", output.as_dict())
            if fault and fault["kind"] == "drop":
                faults.applied(fault["id"])
                emit("fault_applied", {"id": fault["id"], "kind": "drop", "synthetic": True,
                        "actual_new_source_stamp_sim_s": output.stamp_sim_s, "transmitted_to_px4": False})
                return
            if encoder:
                if not state["connected"] or state["state_mono"] is None or now_mono - state["state_mono"] > 2:
                    state["transport_error"] = "FCU_connection_state_stale_or_disconnected"
                    return
                try:
                    fields = packet_ros_fields(encoder.encode(output))
                    raw = Mavlink()
                    raw.header.stamp = rospy.Time.from_sec(output.stamp_sim_s)
                    for key, value in fields.items():
                        setattr(raw, key, value)
                    transport.publish(raw)
                    state["forwarded_count"] += 1
                    state["last_forwarded_sim_s"] = output.stamp_sim_s
                    state["last_forwarded_wall_s"] = time.time()
                    state["transport_error"] = None
                    emit("forwarded", {"stamp_sim_s": output.stamp_sim_s, "seq": fields["seq"],
                                       "epoch": output.epoch, "reset_counter": output.reset_counter,
                                       "quality": output.quality})
                except Exception as exc:
                    state["transport_error"] = str(exc)
                    rospy.logerr_throttle(2, "EV transport encoding failed: %s", str(exc))

    def reanchor(_request):
        with lock:
            accepted, message = adapter.reanchor(*now())
            if accepted:
                delayed_history.clear()
                state["last_publish_sim_s"] = None
            emit("reanchor", {"accepted": accepted, "message": message, "epoch": adapter.epoch})
            return TriggerResponse(success=accepted, message=message)

    def status_callback(_event):
        with lock:
            payload = adapter.status(*now())
            payload.update(mode=args.mode, sitl_verified=verified_sitl,
                           imu_to_base_flu_m=[-x for x in adapter.config.body_to_imu_flu_m],
                           forwarded_count=state["forwarded_count"], shadow_count=state["shadow_count"],
                           last_forwarded_sim_s=state["last_forwarded_sim_s"],
                           last_forwarded_wall_s=state["last_forwarded_wall_s"],
                           last_shadow_sim_s=state["last_shadow_sim_s"],
                           transport_error=state["transport_error"],
                           configured_publish_rate_sim_hz=args.publish_rate_sim_hz)
            fault = faults.active()
            if fault:
                payload["ready"] = False
                payload["reasons"].append("synthetic_ev_transport_" + fault["kind"])
                payload["quality"] = 0
            if state["transport_error"]:
                payload["ready"] = False
                payload["reasons"].append(state["transport_error"])
                payload["quality"] = 0
            payload["forwarding"] = args.mode == "fuse" and payload["ready"]
            test_status = faults.status()
            if test_status is not None:
                payload["test_fault"] = test_status
            statuses.publish(String(data=json.dumps(payload, allow_nan=False)))
            emit("status", payload)

    subscribers = [
        rospy.Subscriber("/vins_estimator/odometry", Odometry, optimized, queue_size=10),
        rospy.Subscriber("/vins_estimator/imu_propagate", Odometry, propagated, queue_size=50),
        rospy.Subscriber("/vins_estimator/tracking_ok", Bool, tracking, queue_size=1),
        rospy.Subscriber("/vins_estimator/imu_bias", Float64MultiArray, bias, queue_size=1),
        rospy.Subscriber("/uav/imu/data_raw", Imu, gyro, queue_size=100),
        rospy.Subscriber("/vins_restart", Bool, restart, queue_size=1),
        rospy.Subscriber("/mavros/state", State, vehicle, queue_size=1),
        rospy.Subscriber("/mavros/extended_state", ExtendedState, extended, queue_size=1),
    ]
    service = rospy.Service("/uav/vio/reanchor", Trigger, reanchor)
    timer = rospy.Timer(rospy.Duration(0.05), status_callback)
    rospy.loginfo("VINS EV adapter mode=%s; fresh measurements only; FCU transport=%s", args.mode, bool(encoder))
    rospy.spin()
    timer.shutdown()
    for subscriber in subscribers:
        subscriber.unregister()
    service.shutdown()


if __name__ == "__main__":
    main()
