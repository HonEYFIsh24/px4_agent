#!/usr/bin/python3
"""ROS Noetic flight executor. Run independently of the language model."""
import argparse
import base64
import copy
import json
import hashlib
import math
import signal
import threading
import time
import uuid
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import rospy
import cv2
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetWorldProperties
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from mission import MAX_ALTITUDE, MAX_RADIUS, MAX_SPEED, MissionError, validate_mission
from scene_config import load_scene, validate_scene_trajectory
from obstacle_navigation import NavigationMap, NavigationError
from reliability_faults import ReliabilityFaults, read_ros_fault_parameter
from vio.quality import invalid_reason
from ev_health import ExternalVisionFlightGuard
from stereo_cache import StereoCache
from stereo_reports import StereoReports
from search_tools import coordinate_epoch


class Cancelled(Exception):
    pass


CAMERA_MAX_JPEG_BYTES = 2 * 1024 * 1024
CAMERA_POSE_MAX_OFFSET_S = 0.05
NAVIGATION_MAX_SPEED = .25


def camera_request_side(path):
    """Only the two fixed camera streams may be requested."""
    parsed = urlsplit(path)
    parameters = parse_qs(parsed.query, keep_blank_values=True)
    if parsed.path != "/camera/snapshot" or set(parameters) - {"side"}:
        raise ValueError("only /camera/snapshot?side=left or side=right is supported")
    sides = parameters.get("side", ["left"])
    if len(sides) != 1 or sides[0] not in ("left", "right"):
        raise ValueError("camera side must be left or right")
    return sides[0]


def quaternion_product(first, second):
    x, y, z, w = first
    a, b, c, d = second
    return [w*a + x*d + y*c - z*b, w*b - x*c + y*d + z*a,
            w*c + x*b - y*a + z*d, w*d - x*a - y*b - z*c]


def rotate_vector(quaternion, vector):
    rotated = quaternion_product(quaternion_product(quaternion, vector + [0.0]),
                                 [-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]])
    return rotated[:3]


def camera_world_pose(body_pose, side):
    """Actual SDF optical frame, relative to the Iris model/base_link origin."""
    if side not in ("left", "right"):
        raise ValueError("camera side must be left or right")
    q = body_pose["orientation_xyzw"]
    p = body_pose["position_m"]
    if (len(p) != 3 or len(q) != 4 or not all(math.isfinite(v) for v in p + q)
            or not 0.99 <= sum(v*v for v in q) <= 1.01):
        raise MissionError("camera body pose is invalid")
    norm = math.sqrt(sum(v*v for v in q))
    q = [v / norm for v in q]
    translation = [0.12, 0.06 if side == "left" else -0.06, 0.18]
    offset = rotate_vector(q, translation)
    # R_body_optical = R_y(0.35) * [[0,0,1],[-1,0,0],[0,-1,0]].
    optical_rotation = quaternion_product([0.0, math.sin(0.35/2), 0.0, math.cos(0.35/2)],
                                          [-0.5, 0.5, -0.5, 0.5])
    return {"position_m": [a + b for a, b in zip(p, offset)],
            "orientation_xyzw": quaternion_product(q, optical_rotation),
            "optical_axes": "x_right_y_down_z_forward"}


class Controller:
    def __init__(self, idle_land_seconds=120, world_path=None, simulation_only=False):
        self.scene = load_scene(world_path)
        guarded_world = (world_path and (
            Path(world_path).resolve().name in ("logistics_yard.world", "obstacle_yard.world")
            or Path(world_path).resolve().parent == (Path(__file__).parent / "scene/reliability").resolve()))
        if guarded_world and not self.scene["verified"]:
            raise MissionError("built-in world does not match its scene manifest; refusing unverified flight geometry")
        self.lock = threading.RLock()
        self.submit_lock = threading.RLock()
        self.command_lock = threading.Lock()
        self.state_msg = State()
        self.landed = ExtendedState.LANDED_STATE_UNDEFINED
        self.pose = None
        self.vio_message = None
        self.vio_time = 0.0
        self.vio_tracking_ok = False
        self.vio_tracking_time = 0.0
        self.ev_guard = ExternalVisionFlightGuard()
        self.ev_landing_requested = False
        self.gazebo_state = None
        self.gazebo_time = 0.0
        self.gazebo_history = deque(maxlen=512)
        self.local_pose_history = deque(maxlen=512)
        self.navigation_map = None
        self.navigation_active = False
        self.navigation_moving = False
        self.navigation_pausing = False
        self.navigation_gate_error = None
        self.navigation_gate_fault = None
        self.lidar_state = None
        self.lidar_error = None
        self.navigation_trace = []
        self.camera_state = {}
        self.camera_info = {"left": deque(maxlen=64), "right": deque(maxlen=64)}
        self.camera_attempts = {"left": 0.0, "right": 0.0}
        self.camera_encode_locks = {"left": threading.Lock(), "right": threading.Lock()}
        self.camera_bridge = CvBridge()
        stereo_bridge = CvBridge()
        def encode_stereo(message):
            pixels = stereo_bridge.imgmsg_to_cv2(message, "bgr8")
            ok, encoded = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not ok or not 0 < encoded.size <= CAMERA_MAX_JPEG_BYTES:
                raise ValueError("stereo JPEG encoding failed or exceeds size limit")
            from stereo_localization import image_quality_from_gray
            quality = image_quality_from_gray(cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE))
            return {"jpeg": encoded.tobytes(), "width": message.width, "height": message.height,
                    "frame_id": message.header.frame_id, "image_quality": quality}
        self.stereo_cache = StereoCache(encode_stereo, lambda: rospy.Time.now().to_sec())
        self.stereo_reports = StereoReports()
        camera_directory = Path(__file__).parent / "logs" / "vio"
        self.camera_paths = {"left": camera_directory / "camera_live.jpg",
                             "right": camera_directory / "camera_right.jpg"}
        self.state_time = self.pose_time = 0.0
        self.extended_state_time = None
        self.landing_mode_pending = False
        self.target = self.commanded = self.home = None
        self.yaw = 0.0
        self.mission = None
        self.cancel = threading.Event()
        self.worker = None
        self.idle_since = None
        self.idle_land_seconds = idle_land_seconds
        self.closed = threading.Event()
        self.log_path = Path(__file__).parent / "logs" / "flight.jsonl"
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_lock = threading.Lock()
        self.reliability_faults = ReliabilityFaults.from_environment(
            simulation_only=simulation_only,
            get_request=read_ros_fault_parameter, emit=self.log)
        self.publisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        rospy.Subscriber("/mavros/state", State, self.on_state)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.on_pose)
        rospy.Subscriber("/mavros/extended_state", ExtendedState, self.on_extended)
        rospy.Subscriber("/vins_estimator/odometry", Odometry, self.on_vio)
        rospy.Subscriber("/vins_estimator/tracking_ok", Bool, self.on_vio_tracking)
        rospy.Subscriber("/uav/vio/ev_status", String, self.on_ev_status, queue_size=1)
        rospy.Subscriber("/uav/vio/ekf_status", String, self.on_ekf_status, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_gazebo, queue_size=1)
        from sensor_msgs.msg import LaserScan
        rospy.Subscriber("/uav/lidar/scan", LaserScan, self.on_lidar, queue_size=1)
        for side in self.camera_paths:
            rospy.Subscriber("/uav/camera/{}/image_raw".format(side), Image, self.on_camera,
                             callback_args=side, queue_size=1, buff_size=2 * 640 * 480 * 3)
            rospy.Subscriber("/uav/camera/{}/camera_info".format(side), CameraInfo,
                             self.on_camera_info, callback_args=side, queue_size=5)
        rospy.wait_for_service("/mavros/set_mode", timeout=30)
        rospy.wait_for_service("/mavros/cmd/arming", timeout=30)
        self.mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        threading.Thread(target=self.publish_loop, daemon=True).start()
        threading.Thread(target=self.watchdog_loop, daemon=True).start()

    def on_state(self, message):
        with self.lock:
            self.state_msg, self.state_time = message, time.monotonic()

    def on_pose(self, message):
        with self.lock:
            self.pose, self.pose_time = message, time.monotonic()
            p = message.pose.position
            q = message.pose.orientation
            guard = getattr(self, 'ev_guard', None)
            barrier=getattr(self,'heading_alignment_pose_barrier_sim_s',None)
            if barrier is not None and message.header.stamp.to_sec()<barrier:
                return
            self.local_pose_history.append({"stamp_sim_s": message.header.stamp.to_sec(),
                                            "position_m": [p.x, p.y, p.z],
                                            "orientation_xyzw": [q.x, q.y, q.z, q.w],
                                            "received": self.pose_time,
                                            "coordinate_epoch": coordinate_epoch(guard.snapshot(self.pose_time)) if guard else None})

    def on_ev_status(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            with self.lock:
                self.ev_guard.observe_adapter(value, time.monotonic(), self.state_msg.armed)
        except (TypeError, ValueError):
            return

    def on_ekf_status(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            with self.lock:
                mission=self.mission or {}
                takeoff_window=(mission.get('status')=='running' and mission.get('action')=='takeoff'
                    and not self.navigation_active and not self.navigation_moving and self.navigation_map is None)
                alignment=self.ev_guard.observe_ekf(value, time.monotonic(), self.state_msg.armed,
                    allow_takeoff_mag_alignment=takeoff_window)
                if alignment:
                    delta=alignment['delta_heading_ned_rad']
                    self.yaw=(self.yaw-delta+math.pi)%(2*math.pi)-math.pi
                    self.heading_alignment_pose_barrier_sim_s=alignment['reset_source_stamp_sim_s']
                    self.local_pose_history.clear()
                    self.navigation_map=None
                    self.navigation_active=self.navigation_moving=False
                    self.log('takeoff_mag_heading_alignment',mission_id=mission.get('id'),
                        **alignment,commanded_yaw_enu_rad=self.yaw)
        except (TypeError, ValueError):
            return

    def on_lidar(self, message):
        received = time.monotonic()
        stamp = message.header.stamp.to_sec()
        with self.lock:
            mapper = self.navigation_map if self.navigation_active else None
            history = list(self.local_pose_history)
            world_history = list(self.gazebo_history)
        scan = {"stamp": stamp, "ranges": list(message.ranges), "angle_min": message.angle_min,
                "angle_increment": message.angle_increment, "range_min": message.range_min,
                "range_max": message.range_max, "frame_id": message.header.frame_id}
        faults = getattr(self, "reliability_faults", None)
        if faults and faults.enabled:
            sensor_pose = None
            if history:
                source_pose = min(history, key=lambda item: abs(item["stamp_sim_s"]-stamp))
                pose_values = source_pose["position_m"] + source_pose["orientation_xyzw"]
                if (abs(source_pose["stamp_sim_s"]-stamp) <= .05 and received-source_pose["received"] <= 1
                        and len(pose_values) == 7 and all(math.isfinite(v) for v in pose_values)
                        and .975 <= math.hypot(*pose_values[3:]) <= 1.025):
                    q = source_pose["orientation_xyzw"]
                    offset = rotate_vector(q, [0., 0., .35])
                    sensor_pose = {"position_m": [a+b for a, b in zip(source_pose["position_m"], offset)],
                                   "orientation_xyzw": list(q), "stamp": source_pose["stamp_sim_s"]}
            scan = faults.transform_scan(scan, sensor_pose, navigation_active=mapper is not None)
            if scan is None:
                return
            stamp = scan["stamp"]
        with self.lock:
            self.lidar_state = {"received": received, "stamp_sim_s": stamp,
                                "sample_count": len(scan["ranges"]), "frame_id": scan["frame_id"]}
            # Fetch histories at the actual alignment decision, after the
            # test adapter. Do not keep a pose snapshot from before processing.
            history = list(self.local_pose_history)
            world_history = list(self.gazebo_history)
        if mapper is None:
            return
        diagnostic = self.lidar_alignment_diagnostic(
            message.header.stamp.to_sec(), stamp, received, history, world_history)
        try:
            if scan["frame_id"] != "uav_lidar" or not history:
                raise NavigationError("LiDAR frame or aligned local pose is unavailable")
            pose = min(history, key=lambda item: abs(item["stamp_sim_s"]-stamp))
            if abs(pose["stamp_sim_s"]-stamp) > .05 or received-pose["received"] > 1:
                raise NavigationError("LiDAR and local ENU pose are not fresh/time aligned")
            if not -.05 <= rospy.Time.now().to_sec()-stamp <= .5:
                raise NavigationError("LiDAR source measurement is stale")
            cruise = self.scene["navigation"]["flight_altitude_m"]
            if abs(pose["position_m"][2]-cruise) > .25:
                raise NavigationError("LiDAR mapping requires fixed cruise height within .25 m")
            # PX4 local Z can drift relative to world height. Supervise only the
            # aircraft's own synchronized pose, never obstacle positions: at
            # this physical scan height, even a 5 m ray tilted by .20 rad stays
            # below the verified scene's 5.5 m walls.
            if not world_history:
                raise NavigationError("LiDAR physical-height supervision requires a fresh world body pose")
            world_pose = min(world_history, key=lambda item: abs(item["stamp_sim_s"]-stamp))
            if abs(world_pose["stamp_sim_s"]-stamp) > .05 or received-world_pose["received"] > 1:
                raise NavigationError("LiDAR and world body pose are not fresh/time aligned")
            world_q = world_pose["orientation_xyzw"]
            if len(world_q) != 4 or not all(math.isfinite(v) for v in world_q):
                raise NavigationError("LiDAR physical-height supervision has invalid world orientation")
            world_norm = math.sqrt(sum(v*v for v in world_q))
            if not .98 <= world_norm <= 1.02:
                raise NavigationError("LiDAR physical-height supervision has nonunit world orientation")
            world_offset = rotate_vector([v/world_norm for v in world_q], [0., 0., .35])
            physical_scan_height = world_pose["position_m"][2]+world_offset[2]
            if not 2.5 <= physical_scan_height <= 4.3:
                raise NavigationError("LiDAR physical scan height must remain within world Z [2.5, 4.3] m")
            q = pose["orientation_xyzw"]
            offset = rotate_vector(q, [0., 0., .35])
            aligned = {"position_m": [a+b for a, b in zip(pose["position_m"], offset)],
                       "orientation_xyzw": q, "stamp": pose["stamp_sim_s"]}
            # Limit planar ray clearing to 5 m. With tilt <= .20 rad and this
            # fixed-height scene's 5.5 m walls, distant beams cannot clear a
            # 2-D obstacle merely because they passed above its top.
            ranges = [min(float(value), 5.) if math.isfinite(value) and value >= scan["range_min"]
                      else (5. if value == float("inf") else value) for value in scan["ranges"]]
            mapper.update_scan({"ranges": ranges, "angle_min": scan["angle_min"],
                                "angle_increment": scan["angle_increment"], "range_min": scan["range_min"],
                                "range_max": min(scan["range_max"], 5.), "stamp": stamp}, aligned, received)
            with self.lock:
                self.lidar_error = None
        except (MissionError, KeyError, ValueError) as exc:
            with self.lock:
                self.lidar_error = str(exc)
                if self.navigation_moving or self.navigation_pausing:
                    self.navigation_gate_error = "latest LiDAR scan rejected: " + str(exc)
                    self.navigation_gate_fault = "sensor"
                    self.navigation_hold()
            self.log("lidar_scan_rejected", error=str(exc),
                     mission_id=(self.mission or {}).get("id"),
                     callback_elapsed_wall_s=time.monotonic()-received,
                     **diagnostic)

    def lidar_alignment_diagnostic(self, source_stamp, effective_stamp, received, history, world_history):
        """Evidence only; none of these fields grant navigation clearance."""
        def summary(items):
            if not items:
                return {"history_count": 0}
            nearest = min(items, key=lambda item: abs(item["stamp_sim_s"]-effective_stamp))
            return {"history_count": len(items),
                    "oldest_stamp_sim_s": min(item["stamp_sim_s"] for item in items),
                    "latest_stamp_sim_s": max(item["stamp_sim_s"] for item in items),
                    "nearest_stamp_sim_s": nearest["stamp_sim_s"],
                    "signed_offset_sim_s": nearest["stamp_sim_s"]-effective_stamp,
                    "absolute_offset_sim_s": abs(nearest["stamp_sim_s"]-effective_stamp),
                    "receipt_age_wall_s_at_callback": received-nearest["received"],
                    "receipt_age_wall_s_at_decision": time.monotonic()-nearest["received"]}
        faults = getattr(self, "reliability_faults", None)
        return {"source_scan_stamp_sim_s": source_stamp,
                "effective_scan_stamp_sim_s": effective_stamp,
                "source_measurement_age_sim_s_at_decision": rospy.Time.now().to_sec()-source_stamp,
                "effective_measurement_age_sim_s_at_decision": rospy.Time.now().to_sec()-effective_stamp,
                "local_pose_alignment": summary(history), "world_pose_alignment": summary(world_history),
                "reliability_poll": faults.polling_snapshot() if faults and faults.enabled else None}

    def navigation_snapshot(self):
        mapper = getattr(self, "navigation_map", None)
        result = mapper.diagnostics(time.monotonic(), self.position() if self.pose else None) if mapper else {
            "fresh": False, "scan_count": 0, "reason": "map starts after takeoff at the fixed navigation altitude"}
        source = getattr(self, "lidar_state", None)
        result.update(enabled=bool(self.scene.get("navigation", {}).get("enabled")),
                      active=getattr(self, "navigation_active", False), source_topic="/uav/lidar/scan",
                      latest_scan_error=getattr(self, "lidar_error", None),
                      lidar_available=bool(source and time.monotonic()-source["received"] <= 1),
                      lidar_sample_count=source.get("sample_count") if source else None,
                      setpoint_speed_m_s=NAVIGATION_MAX_SPEED,
                      scope="fixed-altitude static 2-D obstacle avoidance; no 3-D/dynamic protection")
        return result

    def on_extended(self, message):
        with self.lock:
            self.landed = message.landed_state
            self.extended_state_time = time.monotonic()

    def on_vio(self, message):
        faults = getattr(self, "reliability_faults", None)
        if faults and faults.enabled and faults.vio_decision(
                "/vins_estimator/odometry", message.header.stamp.to_sec(),
                getattr(self, "navigation_active", False)) == "drop":
            return
        with self.lock:
            self.vio_message, self.vio_time = message, time.monotonic()

    def on_vio_tracking(self, message):
        data = message.data
        faults = getattr(self, "reliability_faults", None)
        if faults and faults.enabled:
            decision = faults.vio_decision("/vins_estimator/tracking_ok", rospy.Time.now().to_sec(),
                                           getattr(self, "navigation_active", False))
            if decision == "drop":
                return
            if decision == "tracking_false":
                data = False
        with self.lock:
            self.vio_tracking_ok, self.vio_tracking_time = data, time.monotonic()

    def on_gazebo(self, message):
        if "iris" not in message.name:
            return
        index = message.name.index("iris")
        pose, twist = message.pose[index], message.twist[index]
        p, q, v, w = pose.position, pose.orientation, twist.linear, twist.angular
        with self.lock:
            self.gazebo_state = {
                "position_m": [p.x, p.y, p.z],
                "orientation_xyzw": [q.x, q.y, q.z, q.w],
                "linear_velocity_m_s": [v.x, v.y, v.z],
                "angular_velocity_rad_s": [w.x, w.y, w.z],
                "stamp_sim_s": rospy.Time.now().to_sec(),
                "source": "/gazebo/model_states",
            }
            self.gazebo_time = time.monotonic()
            self.gazebo_history.append({**self.gazebo_state, "received": self.gazebo_time})

    def on_camera_info(self, message, side):
        if side not in self.camera_info:
            return
        k = list(message.K)
        distortion = list(message.D)
        if (len(k) != 9 or message.width <= 0 or message.height <= 0 or k[0] <= 0 or k[4] <= 0
                or not all(math.isfinite(v) for v in k + distortion)):
            return
        calibration = {"stamp_sim_s": message.header.stamp.to_sec(),
                       "width": message.width, "height": message.height,
                       "fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5], "K": k,
                       "distortion_model": message.distortion_model,
                       "distortion_coefficients": distortion,
                       "R": list(message.R), "P": list(message.P),
                       "source": "/uav/camera/{}/camera_info".format(side)}
        with self.lock:
            self.camera_info[side].append(calibration)

    def on_camera(self, message, side):
        stereo = getattr(self, "stereo_cache", None)
        if stereo and side in ("left", "right"):
            stereo.push(side, message)
        if side not in self.camera_paths or not self.camera_encode_locks[side].acquire(False):
            return
        try:
            self.cache_camera(message, side)
        finally:
            self.camera_encode_locks[side].release()

    def cache_camera(self, message, side):
        now = time.monotonic()
        stamp = message.header.stamp.to_sec()
        stamp_age = rospy.Time.now().to_sec() - stamp
        with self.lock:
            if (now - self.camera_attempts[side] < 0.25 or stamp <= 0
                    or not -0.05 <= stamp_age <= 0.5
                    or stamp <= self.camera_state.get(side, {}).get("stamp", 0)):
                return
            self.camera_attempts[side] = now
        try:
            frame = self.camera_bridge.imgmsg_to_cv2(message, "bgr8")
            encoded_ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not encoded_ok:
                return
            jpeg = encoded.tobytes()
            if not 0 < len(jpeg) <= CAMERA_MAX_JPEG_BYTES:
                return
            path = self.camera_paths[side]
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(".tmp.jpg")
            temporary.write_bytes(jpeg)
            temporary.replace(path)
            with self.lock:
                self.camera_state[side] = {"stamp": stamp, "received": now, "jpeg": jpeg,
                                           "width": message.width, "height": message.height,
                                           "frame_id": message.header.frame_id}
        except Exception as exc:
            rospy.logwarn_throttle(10, "%s camera preview unavailable: %s", side, exc)

    def camera_snapshot(self):
        with self.lock:
            now, simulation_now = time.monotonic(), rospy.Time.now().to_sec()
            result = {}
            for side in self.camera_paths:
                frame = self.camera_state.get(side)
                age = now - frame["received"] if frame else None
                measurement_age = simulation_now - frame["stamp"] if frame else None
                result[side] = {
                    "available": bool(frame and age <= 2 and -0.05 <= measurement_age <= 0.5),
                    "width": frame["width"] if frame else None,
                    "height": frame["height"] if frame else None,
                    "stamp": frame["stamp"] if frame else None,
                    "age": age, "measurement_age": measurement_age,
                    "source_hz": 20, "preview_hz": 4,
                    "source": "/uav/camera/{}/image_raw".format(side),
                }
            return result

    def capture_camera(self, side="left"):
        """Return one immutable real frame with same-stamp calibration and aligned poses."""
        if side not in ("left", "right"):
            raise ValueError("camera side must be left or right")
        with self.lock:
            now, simulation_now = time.monotonic(), rospy.Time.now().to_sec()
            frame = self.camera_state.get(side)
            if not frame:
                raise MissionError("{} camera has not published a frame".format(side))
            age, measurement_age = now - frame["received"], simulation_now - frame["stamp"]
            if not 0 <= age <= 2 or not 0 <= measurement_age <= 0.5:
                raise MissionError("{} camera frame is stale or future-dated".format(side))
            jpeg = frame.get("jpeg")
            if not isinstance(jpeg, bytes) or not 0 < len(jpeg) <= CAMERA_MAX_JPEG_BYTES:
                raise MissionError("camera JPEG is unavailable or exceeds the size limit")
            calibration = next((info for info in reversed(self.camera_info[side])
                                if abs(info["stamp_sim_s"] - frame["stamp"]) < 1e-6
                                and (info["width"], info["height"]) == (frame["width"], frame["height"])), None)
            if calibration is None:
                raise MissionError("same-frame camera calibration is not yet available")
            if not self.gazebo_history:
                raise MissionError("camera world pose is not yet available")
            body_pose = min(self.gazebo_history, key=lambda pose: abs(pose["stamp_sim_s"] - frame["stamp"]))
            offset = body_pose["stamp_sim_s"] - frame["stamp"]
            if (abs(offset) > CAMERA_POSE_MAX_OFFSET_S or not 0 <= now - body_pose["received"] <= 2
                    or body_pose["stamp_sim_s"] > simulation_now):
                raise MissionError("camera world pose cannot be aligned within 0.05 simulation seconds")
            local_pose = min(self.local_pose_history, key=lambda pose: abs(pose["stamp_sim_s"] - frame["stamp"])) if self.local_pose_history else None
            local_offset = local_pose["stamp_sim_s"] - frame["stamp"] if local_pose else None
            local_valid = bool(local_pose and abs(local_offset) <= CAMERA_POSE_MAX_OFFSET_S
                               and 0 <= now - local_pose["received"] <= 1
                               and local_pose["stamp_sim_s"] <= simulation_now
                               and all(math.isfinite(v) for v in local_pose["position_m"]))
            payload = {"status": "ok", "side": side, "mime_type": "image/jpeg",
                       "width": frame["width"], "height": frame["height"],
                       "stamp_sim_s": frame["stamp"], "age_seconds": age,
                       "measurement_age_seconds": measurement_age,
                       "image_source": "/uav/camera/{}/image_raw".format(side),
                       "image_frame_id": frame["frame_id"],
                       "camera_intrinsics": copy.deepcopy(calibration),
                       "camera_pose_world": {**camera_world_pose(body_pose, side),
                                             "stamp_sim_s": body_pose["stamp_sim_s"],
                                             "time_alignment_error_s": abs(offset),
                                             "alignment_method": "nearest_gazebo_model_states_receipt_time",
                                             "approximate": True},
                       "drone_position_world_m": list(body_pose["position_m"]),
                       "drone_position_enu_m": list(local_pose["position_m"]) if local_valid else None,
                       "drone_position_enu_available": local_valid,
                       "drone_position_enu_pose_offset_sim_s": local_offset if local_valid else None,
                       "location_frame": "gazebo_world", "simulation_only": True,
                       "pose_alignment": {"method": "nearest_gazebo_model_states_receipt_time",
                                          "approximate": True, "pose_stamp_sim_s": body_pose["stamp_sim_s"],
                                          "pose_minus_image_sim_s": offset,
                                          "maximum_offset_sim_s": CAMERA_POSE_MAX_OFFSET_S,
                                          "note": "ModelStates has no source timestamp; receipt simulation time is used."},
                       "jpeg_size_bytes": len(jpeg)}
        payload["scene"] = copy.deepcopy(getattr(self, "scene", {"ground_plane": {"known": False}}))
        payload["image_base64"] = base64.b64encode(jpeg).decode("ascii")
        return payload

    def gazebo_snapshot(self):
        with self.lock:
            if self.gazebo_state is None:
                return {"available": False, "reason": "Gazebo has not published Iris state"}
            age = time.monotonic() - self.gazebo_time
            values = sum((self.gazebo_state[key] for key in (
                "position_m", "orientation_xyzw", "linear_velocity_m_s", "angular_velocity_rad_s")), [])
            if age > 2 or not all(math.isfinite(v) for v in values):
                return {"available": False, "age_wall_s": age,
                        "reason": "Gazebo state is stale or invalid"}
            return {"available": True, "age_wall_s": age, **copy.deepcopy(self.gazebo_state)}

    def capture_stereo(self):
        """Exact source-image pairing; metric transform uses timed EKF pose only."""
        try:
            frame = self.stereo_cache.snapshot()
        except ValueError as exc:
            raise MissionError(str(exc))
        with self.lock:
            now, simulation_now = time.monotonic(), rospy.Time.now().to_sec()
            stamp = frame["stamp_sim_s"]
            if not self.local_pose_history:
                raise MissionError("stereo has no aligned local EKF pose")
            body = min(self.local_pose_history, key=lambda item: abs(item["stamp_sim_s"]-stamp))
            if abs(body["stamp_sim_s"]-stamp) > .05 or now-body["received"] > 1:
                raise MissionError("stereo and EKF pose are not fresh and aligned")
            external = self.ev_guard.snapshot(now)
            if body.get('coordinate_epoch') != coordinate_epoch(external):
                raise MissionError("stereo pose belongs to an earlier coordinate epoch")
            result = {"status": "ok", "synchronized": True, "stamp_sim_s": stamp,
                      "now_sim_s": simulation_now, "baseline_m": .12,
                      "body_pose_enu": {"frame_id": "map", "body_reference": "base_link",
                          "position_m": list(body["position_m"]), "orientation_xyzw": list(body["orientation_xyzw"]),
                          "stamp_sim_s": body["stamp_sim_s"], "age_seconds": now-body["received"],
                          "source": "/mavros/local_position/pose", "pose_covariance_source": "configured_conservative_floor",
                          "position_covariance_m2": [[.09, 0, 0], [0, .09, 0], [0, 0, .16]],
                          "orientation_covariance_rad2": [[.008, 0, 0], [0, .008, 0], [0, 0, .069]]},
                      "external_vision": external,
                      "coordinate_epoch": body.get('coordinate_epoch')}
            result["image_sha256"] = {side: hashlib.sha256(frame[side]["jpeg"]).hexdigest() for side in ("left", "right")}
            for side in ("left", "right"):
                image = frame[side]
                info = next((item for item in reversed(self.camera_info[side])
                             if abs(item["stamp_sim_s"]-stamp) < 1e-6), None)
                if not info:
                    raise MissionError("stereo requires actual same-stamp CameraInfo")
                result[side] = {"status": "ok", "side": side, "mime_type": "image/jpeg", "width": image["width"],
                    "height": image["height"], "image_frame_id": image["frame_id"], "stamp_sim_s": stamp,
                    "age_seconds": now-frame["received"], "measurement_age_seconds": simulation_now-stamp,
                    "camera_intrinsics": copy.deepcopy(info), "image_base64": base64.b64encode(image["jpeg"]).decode("ascii")}
                result[side]["image_quality"] = copy.deepcopy(image["image_quality"])
        result["pair_id"] = self.stereo_reports.put(result)
        return result

    def vio_snapshot(self):
        with self.lock:
            guard = getattr(self, "ev_guard", None)
            used = bool(guard and guard.snapshot(time.monotonic())["used_for_flight_control"])
            if self.vio_message is None:
                return {"ready": False, "reason": "VINS-Fusion has not published an optimized pose"}
            message = self.vio_message
            age = time.monotonic() - self.vio_time
            p, q = message.pose.pose.position, message.pose.pose.orientation
            v = message.twist.twist.linear
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z]
            stamp_age = (rospy.Time.now() - message.header.stamp).to_sec()
            reason = invalid_reason(values, age, stamp_age, self.vio_tracking_ok,
                                    time.monotonic() - self.vio_tracking_time)
            if reason:
                return {"ready": False, "reason": reason, "used_for_flight_control": used}
            return {"ready": True, "age_seconds": age, "measurement_age_seconds": stamp_age,
                    "frame_id": message.header.frame_id,
                    "position_vio_m": values[:3], "orientation_xyzw": values[3:7],
                    "velocity_vio_m_s": values[7:],
                    "used_for_flight_control": used}

    def position(self):
        with self.lock:
            if self.pose is None:
                raise MissionError("local position is not available")
            p = self.pose.pose.position
            return [p.x, p.y, p.z]

    def ready(self):
        with self.lock:
            now = time.monotonic()
            if not self.state_msg.connected or now - self.state_time > 3:
                raise MissionError("MAVROS is disconnected or its state is stale")
            if self.pose is None or now - self.pose_time > 1:
                raise MissionError("local position is missing or stale")
            if not all(math.isfinite(v) for v in self.position()):
                raise MissionError("local position is invalid")

    def landing_ready(self):
        """FCU-owned landing needs fresh actual FCU state, independently of XY."""
        with self.lock:
            now = time.monotonic()
            state = self.state_msg
            if (type(getattr(state, "connected", None)) is not bool or state.connected is not True
                    or type(getattr(state, "armed", None)) is not bool
                    or not isinstance(getattr(state, "mode", None), str) or not 1 <= len(state.mode) <= 128):
                raise MissionError("landing requires valid connected FCU heartbeat and armed/mode fields")
            for name, received in (("heartbeat", getattr(self, "state_time", None)),
                                   ("ExtendedState", getattr(self, "extended_state_time", None))):
                if (type(received) not in (int, float) or not math.isfinite(received)
                        or not 0 <= now-received <= 3):
                    raise MissionError("landing requires fresh actual FCU {} receipt within 3 wall seconds".format(name))
            if type(self.landed) is not int or self.landed not in (1, 2, 3, 4):
                raise MissionError("landing requires a valid actual ExtendedState landed_state")

    def landing_snapshot(self):
        """Heartbeat alias evidence is not proof of PX4 AUTO_LAND versus DESCEND."""
        with self.lock:
            now = time.monotonic()
            try:
                self.landing_ready()
                ready, reason = True, None
            except MissionError as exc:
                ready, reason = False, str(exc)
            def age(value):
                return now-value if type(value) in (int, float) and math.isfinite(value) else None
            connected = getattr(self.state_msg, 'connected', None)
            armed = getattr(self.state_msg, 'armed', None)
            mode = getattr(self.state_msg, 'mode', None)
            return {"ready": ready, "reason": reason, "source": "actual_MAVROS_State_and_ExtendedState_receipts",
                    "connected": connected if type(connected) is bool else None,
                    "armed": armed if type(armed) is bool else None,
                    "mode": mode if isinstance(mode, str) else None,
                    "landed_state": self.landed if type(self.landed) is int else None,
                    "heartbeat_age_wall_s": age(getattr(self, "state_time", None)),
                    "extended_state_age_wall_s": age(getattr(self, "extended_state_time", None)),
                    "maximum_receipt_age_wall_s": 3., "xy_or_vio_readiness_required": False,
                    "px4_nav_state": None, "px4_nav_state_source": "not_present_in_MAVROS_State",
                    "mode_limitation": "AUTO.LAND heartbeat aliases PX4 AUTO_LAND and DESCEND; raw uORB evidence is separate"}

    def snapshot(self):
        with self.lock:
            try:
                self.ready()
                ready, reason = True, None
            except MissionError as exc:
                ready, reason = False, str(exc)
            position = self.position() if self.pose else None
            if position and not all(math.isfinite(v) for v in position):
                position = None
            result = {
                "ready": ready, "reason": reason, "simulation_only": True,
                "connected": self.state_msg.connected, "armed": self.state_msg.armed,
                "mode": self.state_msg.mode, "landed_state": self.landed,
                "position_enu_m": position,
                "home_enu_m": copy.copy(self.home), "target_enu_m": copy.copy(self.target),
                "limits": {"min_altitude_m": 1, "max_altitude_m": MAX_ALTITUDE,
                           "radius_m": MAX_RADIUS, "setpoint_speed_m_s": MAX_SPEED},
                "mission": copy.deepcopy(self.mission),
                "vio": self.vio_snapshot(),
                "gazebo": self.gazebo_snapshot(),
                "camera": self.camera_snapshot(),
                "scene": copy.deepcopy(self.scene),
                "navigation": self.navigation_snapshot(),
                "landing_readiness": self.landing_snapshot(),
            }
            faults = getattr(self, "reliability_faults", None)
            guard = getattr(self, "ev_guard", None)
            if guard:
                result["external_vision"] = copy.deepcopy(guard.snapshot(time.monotonic()))
            if faults and faults.enabled:
                result["reliability_test"] = faults.snapshot(self.navigation_active)
            return result

    def log(self, event, **fields):
        record = {"time": time.time(), "event": event, **fields}
        with self.log_lock, self.log_path.open("a", encoding="utf-8") as output:
            output.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")

    def publish_loop(self):
        previous = rospy.Time.now().to_sec()
        while not self.closed.wait(0.05) and not rospy.is_shutdown():
            now = time.monotonic()
            simulation_now = rospy.Time.now().to_sec()
            dt, previous = max(0.0, min(simulation_now - previous, 0.1)), simulation_now
            with self.lock:
                landing_pending = getattr(self, "landing_mode_pending", False)
                if (self.target is None or self.commanded is None
                        or now - self.state_time > 3 or (not landing_pending and now - self.pose_time > 2)):
                    continue
                if landing_pending:
                    try:
                        self.landing_ready()
                    except MissionError:
                        continue
                    if self.state_msg.mode == "AUTO.LAND":
                        continue
                    if (len(self.commanded) != 3 or not all(type(v) in (int, float) and math.isfinite(v)
                                                           for v in self.commanded)):
                        continue
                delta = [0., 0., 0.] if landing_pending else [a - b for a, b in zip(self.target, self.commanded)]
                distance = math.sqrt(sum(v * v for v in delta))
                speed_limit = min(MAX_SPEED, NAVIGATION_MAX_SPEED) if self.navigation_moving else MAX_SPEED
                ratio = min(1.0, speed_limit * dt / distance) if distance else 1.0
                if not landing_pending and self.navigation_moving and not self.navigation_map.segment_safe(self.position()[:2], self.target[:2], now):
                    # A model call is never on this feedback path. Freeze the
                    # setpoint immediately when the sensor/motion gate closes.
                    self.navigation_hold()
                    self.navigation_gate_error = "LiDAR motion clearance/freshness gate closed; setpoint frozen"
                    if self.lidar_error or not self.navigation_map.snapshot(now)["fresh"]:
                        self.navigation_gate_fault = "sensor"
                    elif self.navigation_gate_fault != "sensor":
                        self.navigation_gate_fault = "map"
                    delta, ratio = [0., 0., 0.], 0.
                self.commanded = [v + ratio * d for v, d in zip(self.commanded, delta)]
                message = PoseStamped()
                message.header.stamp = rospy.Time.now()
                message.header.frame_id = "map"
                message.pose.position.x, message.pose.position.y, message.pose.position.z = self.commanded
                message.pose.orientation.z = math.sin(self.yaw / 2)
                message.pose.orientation.w = math.cos(self.yaw / 2)
            self.publisher.publish(message)

    def checkpoint(self, require_offboard=False):
        if self.cancel.is_set():
            raise Cancelled()
        if rospy.is_shutdown() or self.closed.is_set():
            raise Cancelled()
        self.ready()
        with self.lock:
            if require_offboard and (self.state_msg.mode != "OFFBOARD" or not self.state_msg.armed):
                raise MissionError("vehicle left armed OFFBOARD mode; mission stopped")
            guard = getattr(self, "ev_guard", None)
            if require_offboard and guard and guard.reason(time.monotonic()):
                raise MissionError(guard.reason(time.monotonic()))
        if time.monotonic() > self.deadline:
            raise MissionError("mission exceeded 900 seconds")

    def landing_checkpoint(self):
        if self.cancel.is_set() or rospy.is_shutdown() or self.closed.is_set():
            raise Cancelled()
        self.landing_ready()
        if time.monotonic() > self.deadline:
            raise MissionError("mission exceeded 900 seconds")

    def wait_until(self, predicate, timeout, require_offboard=False, landing_only=False):
        if landing_only and require_offboard:
            raise MissionError("landing cannot require armed OFFBOARD mode")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if landing_only:
                self.landing_checkpoint()
            else:
                self.checkpoint(require_offboard)
            if predicate():
                return
            self.cancel.wait(0.05)
        raise MissionError("flight operation timed out after {} seconds".format(timeout))

    def switch_mode(self, mode, landing_only=False):
        if landing_only and mode != "AUTO.LAND":
            raise MissionError("landing-only readiness may request only AUTO.LAND")
        checkpoint = self.landing_checkpoint if landing_only else self.checkpoint
        for _ in range(5):
            checkpoint()
            with self.lock:
                if self.state_msg.mode == mode:
                    return
            with self.command_lock:
                checkpoint()
                response = self.mode_client(base_mode=0, custom_mode=mode)
            if response.mode_sent:
                try:
                    if landing_only:
                        self.wait_until(lambda: self.state_msg.mode == mode, 3, landing_only=True)
                    else:
                        self.wait_until(lambda: self.state_msg.mode == mode, 3)
                    return
                except MissionError:
                    checkpoint()
            self.cancel.wait(1)
        raise MissionError("PX4 rejected mode {}".format(mode))

    def set_target(self, target):
        with self.lock:
            self.checkpoint()
            if self.commanded is None:
                self.commanded = self.position()
            self.target = list(target)

    def hold_position(self):
        with self.lock:
            if self.target is not None:
                position = self.position() if self.pose is not None else None
                if (position is not None and 0 <= time.monotonic()-self.pose_time <= 1
                        and all(type(v) in (int, float) and math.isfinite(v) for v in position)):
                    self.target = self.commanded = position
                elif self.commanded is not None and len(self.commanded) == 3 and all(
                        type(v) in (int, float) and math.isfinite(v) for v in self.commanded):
                    self.target = list(self.commanded)

    def navigation_hold(self):
        """Freeze XY while retaining the commanded scan plane during a pause."""
        with self.lock:
            self.commanded = self.position()
            self.target = [*self.commanded[:2], self.scene["navigation"]["flight_altitude_m"]]

    def move_direct(self, target):
        self.checkpoint(require_offboard=True)
        self.set_target(target)
        settled_since = [None]

        def reached():
            distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(self.position(), target)))
            if distance > 0.25:
                settled_since[0] = None
                return False
            if settled_since[0] is None:
                settled_since[0] = time.monotonic()
            return time.monotonic() - settled_since[0] >= 1

        self.wait_until(reached, 100, require_offboard=True)

    def scene_body(self):
        stamp = self.pose.header.stamp.to_sec()
        if not self.gazebo_history or time.monotonic()-self.gazebo_time > 1:
            raise MissionError("cannot verify scene boundary without fresh Gazebo body pose")
        body = min(self.gazebo_history, key=lambda value: abs(value["stamp_sim_s"]-stamp))
        if abs(body["stamp_sim_s"]-stamp) > .05:
            raise MissionError("scene ENU/world poses are not sufficiently time aligned")
        q = self.pose.pose.orientation
        return body, [q.x, q.y, q.z, q.w]

    def start_navigation(self):
        # Map starts at cruise height: ground-height scans cannot justify an
        # airborne path. Boundaries contain no object poses or occupied cells.
        from scene_config import quaternion_yaw
        with self.lock:
            body, q = self.scene_body()
            yaw = quaternion_yaw(body["orientation_xyzw"])-quaternion_yaw(q)
            if abs(math.atan2(math.sin(yaw), math.cos(yaw))) > .15:
                raise MissionError("navigation ENU/world heading alignment exceeds .15 rad")
            c, s = math.cos(yaw), math.sin(yaw)
            p, w = self.position(), body["position_m"]
            ox, oy = w[0]-c*p[0]+s*p[1], w[1]-s*p[0]-c*p[1]
            xmin, ymin, xmax, ymax = self.scene["navigation"]["bounds_xy_m"]
            margin = self.scene["flight_region"]["margin_m"]
            wx, wy = (xmin+xmax)/2-ox, (ymin+ymax)/2-oy
            cx, cy = c*wx+s*wy, -s*wx+c*wy
            hx, hy = (xmax-xmin)/2-margin, (ymax-ymin)/2-margin
            # Conservative ENU rectangle inscribed in the permitted rotated
            # world rectangle; the separate supervisor checks each waypoint.
            ex, ey = hx*abs(c)-hy*abs(s), hy*abs(c)-hx*abs(s)
            self.navigation_map = NavigationMap([cx-ex, cy-ey, cx+ex, cy+ey],
                                               inflation=self.scene["navigation"]["minimum_clearance_m"])
            self.navigation_gate_error = self.navigation_gate_fault = None
            self.lidar_error = None
            self.navigation_active = True
        self.wait_until(lambda: (self.navigation_map.snapshot(time.monotonic())["fresh"]
                                and self.navigation_map.snapshot(time.monotonic())["scan_count"] >= 5),
                        5, require_offboard=True)

    def settle_navigation_hold(self):
        """Stop before replacing a segment invalidated by a new valid scan."""
        stable = [None]

        def stopped():
            if (self.navigation_gate_fault == "sensor" or self.lidar_error
                    or not self.navigation_map.snapshot(time.monotonic())["fresh"]):
                raise MissionError("LiDAR became invalid while stopped for replanning")
            with self.lock:
                body, _ = self.scene_body()
                speed = math.hypot(*body["linear_velocity_m_s"][:2])
                near_hold = math.dist(self.position()[:2], self.target[:2]) <= .15
            if speed > .15 or not near_hold:
                stable[0] = None
                return False
            stable[0] = time.monotonic() if stable[0] is None else stable[0]
            return time.monotonic()-stable[0] >= 1

        self.wait_until(stopped, 15, require_offboard=True)

    def wait_for_navigation_clearance(self):
        """Stay stopped while fresh rays resolve a blocked starting footprint."""
        stable = [None]

        def cleared():
            now = time.monotonic()
            if (self.navigation_gate_fault == "sensor" or self.lidar_error
                    or not self.navigation_map.snapshot(now)["fresh"]):
                raise MissionError("LiDAR became invalid while waiting for observed clearance")
            if not self.navigation_map.segment_safe(self.position()[:2], self.position()[:2], now):
                stable[0] = None
                return False
            stable[0] = now if stable[0] is None else stable[0]
            return now-stable[0] >= 1

        self.log("navigation_clearance_wait", mission_id=self.mission["id"],
                 position_enu_m=self.position(),
                 map=self.navigation_map.diagnostics(time.monotonic(), self.position(), include_map=True))
        started = time.monotonic()
        try:
            self.wait_until(cleared, 8, require_offboard=True)
        except MissionError as exc:
            self.log("navigation_clearance_wait_ended", mission_id=self.mission["id"],
                     outcome="failed", error=str(exc), elapsed_wall_s=time.monotonic()-started,
                     position_enu_m=self.position(),
                     map=self.navigation_map.diagnostics(time.monotonic(), self.position(), include_map=True))
            raise
        self.log("navigation_clearance_wait_ended", mission_id=self.mission["id"],
                 outcome="observed_clearance", elapsed_wall_s=time.monotonic()-started,
                 position_enu_m=self.position(),
                 map=self.navigation_map.diagnostics(time.monotonic(), self.position(), include_map=True))

    def move(self, target):
        if not self.scene.get("navigation", {}).get("enabled"):
            return self.move_direct(target)
        if self.navigation_map is None or not self.navigation_active:
            self.start_navigation()
        target = list(target)
        fixed_height = self.scene["navigation"]["flight_altitude_m"]
        if abs(target[2]-fixed_height) > .01:
            raise MissionError("2-D obstacle navigation only supports the scene's fixed cruise altitude")
        end = time.monotonic()+500
        replans = 0
        while time.monotonic() < end:
            self.checkpoint(require_offboard=True)
            plan = self.navigation_map.plan(self.position()[:2], target[:2], time.monotonic())
            replans += 1
            event = {"goal_enu_m": target, "plan": plan, "created_at": time.time(), "index": replans}
            with self.lock:
                self.mission.setdefault("navigation_plans", []).append(copy.deepcopy(event))
            self.log("navigation_planned", mission_id=self.mission["id"], **event)
            # Execute only the next visible safe segment, then replan from
            # current measurements. This also handles previously unknown turns.
            waypoint = [*plan["waypoints"][0], fixed_height]
            with self.lock:
                body, q = self.scene_body()
                validate_scene_trajectory(self.scene, [{"action": "goto", "x": waypoint[0], "y": waypoint[1],
                                                       "altitude": fixed_height}], self.position(), self.home, body, q)
            with self.lock:
                if self.lidar_error or self.navigation_gate_fault == "sensor":
                    raise MissionError("latest LiDAR scan is invalid: " +
                                       (self.lidar_error or "sensor fault latched during navigation pause"))
                self.navigation_gate_error = None
                self.navigation_gate_fault = None
                self.set_target(waypoint)
                self.navigation_moving = True
            replan_required = False
            try:
                stable = None
                segment_end = time.monotonic()+100
                while time.monotonic() < segment_end:
                    self.checkpoint(require_offboard=True)
                    current = self.position()
                    if self.navigation_gate_error or not self.navigation_map.segment_safe(current[:2], waypoint[:2], time.monotonic()):
                        with self.lock:
                            diagnostic = self.navigation_map.diagnostics(time.monotonic(), current, include_map=True)
                            sensor_fault = (self.lidar_error or not diagnostic["fresh"]
                                            or self.navigation_gate_fault == "sensor")
                            self.navigation_moving = False
                            self.navigation_pausing = True
                            self.navigation_hold()
                            pause = {"created_at": time.time(), "position_enu_m": current,
                                     "waypoint_enu_m": waypoint, "sensor_fault": bool(sensor_fault),
                                     "latest_scan_error": self.lidar_error,
                                     "map": {key: value for key, value in diagnostic.items() if key != "grid"}}
                            self.mission.setdefault("navigation_pauses", []).append(pause)
                        self.log("navigation_paused", mission_id=self.mission["id"],
                                 **{**pause, "map": diagnostic})
                        if sensor_fault:
                            raise MissionError("LiDAR navigation sensor/freshness gate closed; holding")
                        self.settle_navigation_hold()
                        if not self.navigation_map.segment_safe(self.position()[:2], self.position()[:2], time.monotonic()):
                            self.wait_for_navigation_clearance()
                        replan_required = True
                        break
                    if math.dist(current, waypoint) <= .25:
                        stable = time.monotonic() if stable is None else stable
                        if time.monotonic()-stable >= 1:
                            break
                    else:
                        stable = None
                    self.cancel.wait(.1)
                else:
                    raise MissionError("navigation segment timed out")
            finally:
                self.navigation_moving = False
                self.navigation_pausing = False
            if not replan_required and plan["route_complete"] and len(plan["waypoints"]) == 1:
                return
            if replans > 100:
                raise MissionError("navigation exceeded 100 replans without reaching the goal")
        raise MissionError("autonomous navigation exceeded 500 seconds")

    def takeoff(self, altitude):
        with self.lock:
            self.checkpoint()
            guard = getattr(self, "ev_guard", None)
            if guard and guard.reason(time.monotonic()):
                raise MissionError(guard.reason(time.monotonic()))
            if self.state_msg.armed or self.landed != ExtendedState.LANDED_STATE_ON_GROUND:
                raise MissionError("takeoff requires a disarmed, landed vehicle")
            self.home = self.position()
            q = self.pose.pose.orientation
            self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            # A map from a completed sortie is rebuilt after this takeoff and
            # its source-proved magnetic heading alignment has settled.
            self.navigation_map=None
            self.navigation_active=self.navigation_moving=False
            self.heading_alignment_pose_barrier_sim_s=None
        # Pre-stream a stationary setpoint before enabling OFFBOARD.
        self.set_target(self.position())
        self.pre_stream()
        self.switch_mode("OFFBOARD")
        for _ in range(5):
            self.checkpoint()
            with self.command_lock:
                self.checkpoint()
                armed = self.arm_client(value=True).success
            if armed:
                self.wait_until(lambda: self.state_msg.armed, 5)
                break
            self.cancel.wait(1)
        else:
            raise MissionError("PX4 rejected arming")
        self.move_direct([self.home[0], self.home[1], altitude])
        if guard and guard.required:
            self.wait_until(self.takeoff_heading_ready,15,require_offboard=True)
        if self.scene.get("navigation", {}).get("enabled"):
            settled = [None]
            def level_height():
                if abs(self.position()[2]-altitude) > .10:
                    settled[0] = None
                    return False
                if settled[0] is None:
                    settled[0] = time.monotonic()
                return time.monotonic()-settled[0] >= 1
            self.wait_until(level_height, 15, require_offboard=True)
            self.start_navigation()

    def takeoff_heading_ready(self):
        """Use post-reset local attitude before creating any horizontal map."""
        guard=getattr(self,'ev_guard',None)
        if not guard or not guard.required:
            return True
        with self.lock:
            barrier=getattr(self,'heading_alignment_pose_barrier_sim_s',None)
            source=self.pose.header.stamp.to_sec() if self.pose is not None else None
            return (not getattr(guard,'alignment_pending',False)
                and isinstance(guard.ekf,dict) and guard.ekf.get('heading_good_for_control') is True
                and (barrier is None or source is not None and source>=barrier))

    def pre_stream(self):
        end = time.monotonic() + 2
        while time.monotonic() < end:
            self.checkpoint()
            self.cancel.wait(0.05)

    def land(self):
        self.landing_checkpoint()
        if self.state_msg.armed is False and self.landed == ExtendedState.LANDED_STATE_ON_GROUND:
            with self.lock:
                self.navigation_active = self.navigation_moving = False
                self.target = self.commanded = None
            return
        with self.lock:
            self.landing_mode_pending = True
            if self.commanded is not None:
                self.target = list(self.commanded)
        try:
            self.switch_mode("AUTO.LAND", landing_only=True)
            self.landing_checkpoint()
            with self.lock:
                if self.state_msg.mode != "AUTO.LAND":
                    raise MissionError("AUTO.LAND was not confirmed by actual FCU heartbeat")
                self.navigation_active = self.navigation_moving = False
                self.target = self.commanded = None
                self.landing_mode_pending = False
            self.wait_until(lambda: self.landed == ExtendedState.LANDED_STATE_ON_GROUND
                            and self.state_msg.armed is False, 120, landing_only=True)
            self.landing_checkpoint()
        finally:
            with self.lock:
                self.landing_mode_pending = False

    def submit(self, steps, description="", parent_mission_id=None):
        with self.submit_lock, self.lock:
            if self.mission and self.mission["status"] == "running":
                raise MissionError("a mission is already running; stop it before submitting another")
            airborne = self.state_msg.armed and self.landed != ExtendedState.LANDED_STATE_ON_GROUND
            normalized = validate_mission(steps, airborne=airborne, home_known=self.home is not None)
            pure_land = normalized == [{"action": "land"}]
            if pure_land:
                self.landing_ready()
            else:
                self.ready()
            guard = getattr(self, "ev_guard", None)
            if any(step["action"] != "land" for step in normalized) and guard and guard.reason(time.monotonic()):
                raise MissionError(guard.reason(time.monotonic()))
            navigation = self.scene.get("navigation", {})
            if navigation.get("enabled"):
                for step in normalized:
                    if step["action"] in ("takeoff", "goto") and abs(step["altitude"]-navigation["flight_altitude_m"]) > .01:
                        raise MissionError("this 2-D autonomous scene requires fixed cruise altitude {} m".format(navigation["flight_altitude_m"]))
            if any(s["action"] == "takeoff" for s in normalized) and self.state_msg.armed:
                raise MissionError("takeoff requires a disarmed vehicle")
            if not pure_land and self.scene.get("flight_region"):
                stamp = self.pose.header.stamp.to_sec()
                if not self.gazebo_history or time.monotonic()-self.gazebo_time > 1:
                    raise MissionError("cannot verify scene flight path without fresh Gazebo body pose")
                body = min(self.gazebo_history, key=lambda value: abs(value["stamp_sim_s"]-stamp))
                if abs(body["stamp_sim_s"]-stamp) > .05:
                    raise MissionError("scene ENU/world poses are not sufficiently time aligned")
                q = self.pose.pose.orientation
                validate_scene_trajectory(self.scene, normalized, self.position(), self.home,
                                          body, [q.x, q.y, q.z, q.w])
            self.cancel = threading.Event()
            self.deadline = time.monotonic() + 900
            self.idle_since = None
            self.mission = {"id": uuid.uuid4().hex[:12], "status": "running", "step": 0,
                            "steps": normalized, "description": str(description)[:500],
                            "error": None, "started_at": time.time(), "events": []}
            if parent_mission_id is not None:
                self.mission['parent_mission_id']=parent_mission_id
            if pure_land:
                self.mission['landing_readiness_at_acceptance']=self.landing_snapshot()
            self.worker = threading.Thread(target=self.run_mission, daemon=True)
            self.worker.start()
            return {"mission": copy.deepcopy(self.mission)}

    def run_mission(self):
        mission_id = self.mission["id"]
        status, error = "succeeded", None
        try:
            self.log("mission_started", mission=copy.deepcopy(self.mission))
            for index, step in enumerate(self.mission["steps"]):
                if step["action"] == "land":
                    self.landing_checkpoint()
                else:
                    self.checkpoint()
                with self.lock:
                    self.mission["step"] = index + 1
                    self.mission["action"] = step["action"]
                action = step["action"]
                self.log("step_started", mission_id=mission_id, index=index + 1, step=step)
                if action == "takeoff":
                    self.takeoff(step["altitude"])
                elif action == "goto":
                    self.move([step["x"], step["y"], step["altitude"]])
                elif action == "return_home":
                    altitude = self.scene.get("navigation", {}).get("flight_altitude_m", self.position()[2])
                    self.move([self.home[0], self.home[1], altitude])
                elif action == "hold":
                    self.hold_position()
                    end = time.monotonic() + step["duration"]
                    self.wait_until(lambda: time.monotonic() >= end, step["duration"] + 1,
                                    require_offboard=True)
                elif action == "land":
                    self.land()
                if action == "land":
                    self.landing_checkpoint()
                    with self.lock:
                        if self.state_msg.armed is not False or self.landed != ExtendedState.LANDED_STATE_ON_GROUND:
                            raise MissionError("landing completion lacks fresh actual ground and disarmed evidence")
                        position = self.position() if self.pose is not None and 0 <= time.monotonic()-self.pose_time <= 1 else None
                        if position is not None and not all(type(v) in (int, float) and math.isfinite(v) for v in position):
                            position = None
                        landing_evidence = self.landing_snapshot()
                else:
                    position = self.position()
                    landing_evidence = None
                event = {"step": index + 1, "action": action,
                         "position_enu_m": position, "finished_at": time.time()}
                if landing_evidence is not None:
                    event['landing_fcu_evidence']=landing_evidence
                with self.lock:
                    self.mission["events"].append(event)
                self.log("step_completed", mission_id=mission_id, **event)
        except Cancelled:
            status = "cancelled"
            self.hold_position()
        except Exception as exc:
            status, error = "failed", str(exc)
            self.hold_position()
            rospy.logerr("Mission failed: %s", error)
        finally:
            with self.lock:
                self.mission.update(status=status, error=error, finished_at=time.time())
                self.idle_since = time.monotonic()
            self.log("mission_finished", mission_id=mission_id, status=status, error=error)

    def stop(self):
        with self.command_lock, self.lock:
            self.cancel.set()
            self.hold_position()
            holding = self.state_msg.armed and self.state_msg.mode == "OFFBOARD" and self.target is not None
            self.idle_since = time.monotonic()
        return {"requested": "cancel", "holding": holding, "state": self.snapshot()}

    def emergency_land(self, expected_mission_id=None):
        # Serialize the ownership check, cancellation and replacement with all
        # mission submissions. A recovery must never cancel a newer flight.
        with self.submit_lock:
            with self.lock:
                parent_mission_id=(self.mission or {}).get('id')
            if expected_mission_id is not None:
                if not isinstance(expected_mission_id,str) or not 1<=len(expected_mission_id)<=64:
                    raise MissionError('expected_mission_id must be a nonempty mission id')
                with self.lock:
                    if not self.mission or self.mission.get('id')!=expected_mission_id:
                        raise MissionError('mission changed; refusing recovery landing for another flight')
            self.stop()
            worker = self.worker
            if worker and worker.is_alive():
                worker.join(timeout=3)
            return self.submit([{"action": "land"}], "Operator requested landing",
                               parent_mission_id=parent_mission_id)

    def prepare_offboard(self):
        """Ground-only stationary stream for a truthful planned-mode prearm check."""
        with self.submit_lock:
            def grounded():
                self.ready()
                if self.state_msg.armed or self.landed != ExtendedState.LANDED_STATE_ON_GROUND:
                    raise MissionError("OFFBOARD preparation requires a grounded disarmed vehicle")
                if self.mission and self.mission["status"] == "running":
                    raise MissionError("OFFBOARD preparation cannot replace an active mission")
                reason = self.ev_guard.reason(time.monotonic())
                if reason:
                    raise MissionError(reason)
            with self.lock:
                grounded()
                self.commanded = self.target = self.position()
            started = time.monotonic()
            while time.monotonic()-started < 3.5:
                if self.closed.wait(.05):
                    raise MissionError("executor closed during OFFBOARD preparation")
                with self.lock:
                    grounded()
            deadline = time.monotonic()+8
            while time.monotonic()<deadline:
                with self.lock:
                    grounded()
                    if self.state_msg.mode == "OFFBOARD":
                        result = self.snapshot()
                        self.log("ground_offboard_prepared", position_enu_m=result["position_enu_m"], armed=False)
                        return {"status": "ok", "armed": False, "state": result}
                with self.command_lock:
                    self.mode_client(base_mode=0, custom_mode="OFFBOARD")
                self.closed.wait(.5)
            raise MissionError("PX4 rejected grounded OFFBOARD preparation")

    def watchdog_loop(self):
        while not self.closed.wait(.1) and not rospy.is_shutdown():
            with self.lock:
                idle = self.idle_since is not None and time.monotonic() - self.idle_since > self.idle_land_seconds
                should_land = idle and self.state_msg.armed and self.state_msg.mode == "OFFBOARD"
                guard = getattr(self, "ev_guard", None)
                reason = guard.reason(time.monotonic()) if guard else None
                ev_land = bool(reason and self.state_msg.armed and self.state_msg.mode == "OFFBOARD"
                               and not getattr(self, "ev_landing_requested", False))
                if ev_land:
                    self.ev_landing_requested = True
            if ev_land:
                try:
                    self.log("external_vision_safety_landing", reason=reason,
                             external_vision=guard.snapshot(time.monotonic()))
                    self.emergency_land()
                except Exception as exc:
                    # Heartbeat AUTO.LAND aliases two PX4 nav states. Preserve
                    # any rejection; requesting it never proves FCU fallback.
                    self.log("external_vision_safety_landing_rejected", error=str(exc), reason=reason)
                    rospy.logerr("VIO safety landing failed: %s", str(exc))
            if should_land:
                try:
                    self.submit([{"action": "land"}], "Automatic landing after idle timeout")
                except Exception as exc:
                    rospy.logwarn_throttle(5, "Idle landing failed: %s", str(exc))

    def shutdown(self):
        # Request an autopilot-owned landing before the setpoint publisher stops.
        if self.closed.is_set():
            return
        self.cancel.set()
        if self.state_msg.armed:
            try:
                self.mode_client(base_mode=0, custom_mode="AUTO.LAND")
            except Exception as exc:
                rospy.logerr("Shutdown landing request failed: %s", str(exc))
        self.closed.set()
        faults = getattr(self, "reliability_faults", None)
        if faults:
            faults.close()
        stereo = getattr(self, "stereo_cache", None)
        if stereo:
            stereo.close()


def simulation_guard():
    if not rospy.get_param("/use_sim_time", False):
        raise RuntimeError("simulation only: /use_sim_time must be true")
    if rospy.get_param("/mavros/fcu_url", "") != "udp://:14540@localhost:14557":
        raise RuntimeError("simulation only: MAVROS must use the provided local SITL endpoint")
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=30)
    world_client = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        world = world_client()
        if world.success and "iris" in world.model_names:
            return
        time.sleep(0.2)
    raise RuntimeError("simulation only: Gazebo iris model is missing")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--idle-land-seconds", type=int, default=120)
    parser.add_argument("--world-file", help="The actual Gazebo world used by this system invocation")
    args = parser.parse_args()
    if not 10 <= args.idle_land_seconds <= 600:
        parser.error("idle landing timeout must be between 10 and 600 seconds")
    rospy.init_node("uav_agent_executor", disable_signals=True)
    simulation_guard()
    controller = Controller(args.idle_land_seconds, args.world_file, simulation_only=True)
    rospy.on_shutdown(controller.shutdown)

    def terminate(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, terminate)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *arguments):
            pass

        def respond(self, status, payload):
            data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ("/state", "/health"):
                self.respond(200, controller.snapshot())
            elif self.path == "/camera/stereo":
                try:
                    self.respond(200, controller.capture_stereo())
                except (MissionError, ValueError) as exc:
                    self.respond(503, {"status": "error", "error": str(exc), "code": "stereo_unavailable"})
            elif urlsplit(self.path).path == "/camera/snapshot":
                try:
                    side = camera_request_side(self.path)
                except ValueError as exc:
                    self.respond(400, {"status": "error", "error": str(exc)})
                    return
                try:
                    self.respond(200, controller.capture_camera(side))
                except MissionError as exc:
                    self.respond(503, {"status": "error", "error": str(exc),
                                       "code": "camera_snapshot_unavailable"})
            else:
                self.respond(404, {"error": "unknown endpoint"})

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= 16384:
                    raise MissionError("request body must be 1..16384 bytes")
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise MissionError("request body must be an object")
                if self.path == "/mission":
                    result = controller.submit(payload.get("steps"), payload.get("description", ""))
                elif self.path == "/stop":
                    result = controller.stop()
                elif self.path == "/land":
                    if set(payload)-{'expected_mission_id'}:
                        raise MissionError('unknown landing request fields')
                    if 'expected_mission_id' in payload:
                        if not isinstance(payload['expected_mission_id'],str):
                            raise MissionError('expected_mission_id must be a mission id')
                        result = controller.emergency_land(payload['expected_mission_id'])
                    else:
                        result = controller.emergency_land()
                elif self.path == "/camera/stereo/localize":
                    result = controller.stereo_reports.localize(payload.get("pair_id"), payload.get("bboxes"))
                elif self.path == "/prepare-offboard":
                    if payload:
                        raise MissionError("OFFBOARD preparation accepts an empty object only")
                    result = controller.prepare_offboard()
                else:
                    self.respond(404, {"error": "unknown endpoint"})
                    return
                self.respond(200, result)
            except (ValueError, MissionError) as exc:
                self.respond(400, {"error": str(exc)})
            except Exception as exc:
                self.respond(503, {"error": str(exc)})

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.daemon_threads = True
    rospy.loginfo("UAV executor listening at http://127.0.0.1:%s (simulation only)", args.port)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        server.server_close()
        rospy.signal_shutdown("executor stopped")


if __name__ == "__main__":
    main()
