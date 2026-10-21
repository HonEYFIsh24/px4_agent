"""Camera evidence contracts; ROS callbacks are exercised without starting a master."""
import base64
import importlib.util
import math
import sys
import tempfile
import threading
import types
import unittest
from collections import deque
from pathlib import Path
from unittest import mock

from client import BridgeClient
from ev_health import ExternalVisionFlightGuard


def load_bridge_without_ros_master():
    modules = {}
    for package, names in {
        "cv_bridge": ["CvBridge"], "gazebo_msgs.msg": ["ModelStates"],
        "gazebo_msgs.srv": ["GetWorldProperties"], "geometry_msgs.msg": ["PoseStamped"],
        "mavros_msgs.msg": ["ExtendedState", "State"],
        "mavros_msgs.srv": ["CommandBool", "SetMode"], "nav_msgs.msg": ["Odometry"],
        "sensor_msgs.msg": ["CameraInfo", "Image"], "std_msgs.msg": ["Bool", "String"],
    }.items():
        if "." in package:
            parent = package.split(".")[0]
            modules.setdefault(parent, types.ModuleType(parent))
        module = modules[package] = types.ModuleType(package)
        for name in names:
            setattr(module, name, type(name, (), {}))
    modules["rospy"] = types.ModuleType("rospy")
    try:
        import cv2
    except ImportError:
        cv2 = types.ModuleType("cv2")
        cv2.IMWRITE_JPEG_QUALITY = 1
        cv2.imencode = mock.Mock()
        modules["cv2"] = cv2
    spec = importlib.util.spec_from_file_location(
        "_camera_contract_bridge", Path(__file__).resolve().parents[1] / "bridge.py")
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


bridge = load_bridge_without_ros_master()


class CameraCaptureTests(unittest.TestCase):
    def setUp(self):
        self.wall_time, self.sim_time = 1000.0, 10.12
        self.wall_patch = mock.patch.object(bridge.time, "monotonic", lambda: self.wall_time)
        self.wall_patch.start()
        self.addCleanup(self.wall_patch.stop)
        bridge.rospy.Time = types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_sec=lambda: self.sim_time))
        bridge.rospy.logwarn_throttle = mock.Mock()
        self.controller = bridge.Controller.__new__(bridge.Controller)
        self.controller.lock = threading.RLock()
        self.controller.camera_state = {}
        self.controller.camera_info = {side: deque(maxlen=64) for side in ("left", "right")}
        self.controller.camera_attempts = {"left": 0.0, "right": 0.0}
        self.controller.camera_encode_locks = {side: threading.Lock() for side in ("left", "right")}
        self.controller.gazebo_history = deque([{
            "stamp_sim_s": 10.004, "received": 999.95,
            "position_m": [1.0, 2.0, 3.0], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        }])
        self.controller.local_pose_history = deque([{
            "stamp_sim_s": 10.002, "received": 999.95, "position_m": [-1.0, -2.0, 2.8],
        }])
        self.frame("left", 10.0, b"\xff\xd8left-frame\xff\xd9")
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.controller.camera_paths = {side: Path(self.directory.name) / (side + ".jpg")
                                        for side in ("left", "right")}

    def frame(self, side, stamp, jpeg):
        self.controller.camera_state[side] = {
            "stamp": stamp, "received": 999.95, "jpeg": jpeg,
            "width": 640, "height": 480, "frame_id": "uav_stereo_optical",
        }
        self.controller.camera_info[side].append({
            "stamp_sim_s": stamp, "width": 640, "height": 480,
            "fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0,
            "K": [320.0, 0, 320.0, 0, 320.0, 240.0, 0, 0, 1],
            "distortion_model": "plumb_bob", "distortion_coefficients": [0.0] * 5,
            "source": "/uav/camera/" + side + "/camera_info",
        })

    def test_real_frame_and_calibration_remain_one_immutable_snapshot(self):
        result = self.controller.capture_camera()
        self.assertEqual(base64.b64decode(result["image_base64"]), b"\xff\xd8left-frame\xff\xd9")

        self.assertEqual(result["stamp_sim_s"], result["camera_intrinsics"]["stamp_sim_s"])
        self.assertEqual(result["camera_pose_world"]["position_m"], [1.12, 2.06, 3.18])
        self.assertEqual(result["drone_position_enu_m"], [-1.0, -2.0, 2.8])
        self.assertTrue(result["pose_alignment"]["approximate"])
        self.assertAlmostEqual(result["camera_pose_world"]["time_alignment_error_s"], 0.004)
        self.frame("left", 10.1, b"\xff\xd8newer-frame\xff\xd9")
        result["camera_intrinsics"]["K"][0] = -5
        result["drone_position_enu_m"][0] = -50
        self.assertEqual(self.controller.camera_info["left"][0]["K"][0], 320)
        self.assertEqual(self.controller.local_pose_history[0]["position_m"][0], -1)
        self.assertEqual(base64.b64decode(result["image_base64"]), b"\xff\xd8left-frame\xff\xd9")

    def test_stereo_rejects_historical_pose_after_coordinate_reset(self):
        self.controller.ev_guard = ExternalVisionFlightGuard()
        self.controller.ev_guard.observe_adapter({'mode':'shadow','epoch':0}, self.wall_time)
        self.controller.ev_guard.observe_ekf({'local_position_reset_counters':{'xy':2,'z':1,'heading':1}}, self.wall_time)
        pose=self.controller.local_pose_history[0]
        pose['coordinate_epoch']={'vio_epoch':0,'ekf_counters':{'xy':1,'z':1,'heading':1}}
        self.controller.stereo_cache=mock.Mock()
        self.controller.stereo_cache.snapshot.return_value={'stamp_sim_s':10.0}
        self.controller.stereo_reports=mock.Mock()
        with self.assertRaisesRegex(bridge.MissionError,'earlier coordinate epoch'):
            self.controller.capture_stereo()
        self.controller.stereo_reports.put.assert_not_called()

    def test_optical_axes_and_yaw_match_generated_sdf(self):
        pose = bridge.camera_world_pose({"position_m": [0, 0, 1], "orientation_xyzw": [0, 0, 0, 1]}, "left")
        q = pose["orientation_xyzw"]
        for axis, expected in (([1, 0, 0], [0, -1, 0]),
                               ([0, 1, 0], [-math.sin(.35), 0, -math.cos(.35)]),
                               ([0, 0, 1], [math.cos(.35), 0, -math.sin(.35)])):
            for actual, wanted in zip(bridge.rotate_vector(q, axis), expected):
                self.assertAlmostEqual(actual, wanted)
        yaw_pose = bridge.camera_world_pose({"position_m": [0, 0, 1],
                                             "orientation_xyzw": [0, 0, math.sqrt(.5), math.sqrt(.5)]}, "right")
        for actual, wanted in zip(yaw_pose["position_m"], [.06, .12, 1.18]):
            self.assertAlmostEqual(actual, wanted)

    def test_each_eye_has_its_own_frame_and_extrinsic(self):
        self.frame("right", 10.05, b"\xff\xd8right-frame\xff\xd9")
        result = self.controller.capture_camera("right")
        self.assertEqual(base64.b64decode(result["image_base64"]), b"\xff\xd8right-frame\xff\xd9")
        self.assertEqual(result["stamp_sim_s"], 10.05)
        self.assertEqual(result["camera_pose_world"]["position_m"], [1.12, 1.94, 3.18])

    def test_stale_future_and_oversize_frames_are_rejected(self):
        cases = [{"received": 997.9}, {"stamp": 9.6}, {"stamp": 10.13},
                 {"jpeg": b"x" * (bridge.CAMERA_MAX_JPEG_BYTES + 1)}]
        original = dict(self.controller.camera_state["left"])
        for change in cases:
            with self.subTest(change=list(change)):
                self.controller.camera_state["left"] = {**original, **change}
                with self.assertRaises(bridge.MissionError):
                    self.controller.capture_camera()

    def test_calibration_and_pose_alignment_are_required(self):
        self.controller.camera_info["left"][0]["stamp_sim_s"] = 9.95
        with self.assertRaisesRegex(bridge.MissionError, "same-frame"):
            self.controller.capture_camera()
        self.controller.camera_info["left"][0]["stamp_sim_s"] = 10.0
        self.controller.gazebo_history[0]["stamp_sim_s"] = 9.94
        with self.assertRaisesRegex(bridge.MissionError, "aligned"):
            self.controller.capture_camera()

    def test_unavailable_enu_does_not_invent_a_coordinate(self):
        self.controller.local_pose_history[0]["stamp_sim_s"] = 9.8
        result = self.controller.capture_camera()
        self.assertIsNone(result["drone_position_enu_m"])
        self.assertFalse(result["drone_position_enu_available"])

    def test_preview_file_and_model_bytes_are_encoded_once(self):
        jpeg = b"\xff\xd8encoded-callback-frame\xff\xd9"
        message = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: 10.1), frame_id="optical"),
            width=640, height=480)
        self.controller.camera_bridge = types.SimpleNamespace(imgmsg_to_cv2=mock.Mock(return_value=object()))
        encoded = types.SimpleNamespace(tobytes=lambda: jpeg)
        with mock.patch.object(bridge.cv2, "imencode", return_value=(True, encoded)) as encoder:
            self.controller.on_camera(message, "left")
        self.assertEqual(encoder.call_count, 1)
        self.assertEqual(self.controller.camera_paths["left"].read_bytes(), jpeg)
        self.assertEqual(self.controller.camera_state["left"]["jpeg"], jpeg)
        self.assertEqual(self.controller.camera_state["left"]["stamp"], 10.1)

    def test_invalid_routes_and_client_sides_are_blocked(self):
        self.assertEqual(bridge.camera_request_side("/camera/snapshot"), "left")
        self.assertEqual(bridge.camera_request_side("/camera/snapshot?side=right"), "right")
        for path in ("/camera/snapshot?side=/tmp/key", "/camera/snapshot?side=left&side=right",
                     "/camera/snapshot?path=/tmp/key", "/camera/snapshot?side="):
            with self.assertRaises(ValueError):
                bridge.camera_request_side(path)
        client = BridgeClient()
        client.request = mock.Mock(return_value={"status": "ok"})
        self.assertEqual(client.capture_camera("right"), {"status": "ok"})
        client.request.assert_called_once_with("/camera/snapshot?side=right")
        with self.assertRaises(ValueError):
            client.capture_camera("left&path=/tmp/key")


if __name__ == "__main__":
    unittest.main()
