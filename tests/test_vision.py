"""Vision evidence and geometry contracts; no provider, ROS, or flight calls."""
import base64
import copy
import importlib.util
import json
import math
import os
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

import vision


CONFIG = {"model": "qwen3-vl-plus", "base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
          "api_key": "secret-test-vision-key"}
# An actual, independently encoded 16x16 JPEG. No Pillow/OpenCV dependency at test time.
JPEG_B64 = (
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAYEBQYFBAYGBQYHBwYIChAKCgkJChQODwwQFxQYGBcU"
    "FhYaHSUfGhsjHBYWICwgIyYnKSopGR8tMC0oMCUoKSj/2wBDAQcHBwoIChMKChMoGhYaKCgoKCgo"
    "KCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCj/"
    "wAARCAAQABADASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgED"
    "AwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJ"
    "icoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5"
    "iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6"
    "/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEE"
    "BSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElK"
    "U1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6"
    "wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDgqKKK8Q/YD//Z"
)


def observation(targets=None):
    return {"summary": "当前画面可见红色目标", "targets": [target()] if targets is None else targets,
            "limitations": ["没有实测深度"]}


def target(basis="image_only", bbox=None):
    return {"label": "红色方块", "confidence": 0.8,
            "bbox": bbox or [0.4, 0.4, 0.6, 0.6], "surroundings": ["左侧可见地面"],
            "position_basis": basis}


def snapshot():
    return {"status": "ok", "side": "left", "image_base64": JPEG_B64,
            "mime_type": "image/jpeg", "width": 640, "height": 480,
            "stamp_sim_s": 42.0, "age_seconds": 0.1, "measurement_age_seconds": 0.02,
            "location_frame": "gazebo_world",
            "scene": {"id": "unit-known-flat", "verified": True,
                      "ground_plane": {"known": True, "z_m": 0.003,
                                       "bounds_xy_m": [-10, -10, 10, 10]}},
            "camera_intrinsics": {"fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0,
                                  "distortion_model": "plumb_bob", "distortion_coefficients": [0.0] * 5},
            # Optical x-right, y-down, z-forward; rotation around x points camera down.
            "camera_pose_world": {"position_m": [1.0, 2.0, 4.003],
                                  "orientation_xyzw": [1.0, 0.0, 0.0, 0.0],
                                  "optical_axes": "x_right_y_down_z_forward",
                                  "time_alignment_error_s": 0.01}}


class FakeClient:
    def __init__(self, captured=None, error=None):
        self.captured = snapshot() if captured is None else captured
        self.error = error
        self.calls = []

    def capture_camera(self, side):
        self.calls.append(side)
        if self.error:
            raise self.error
        return copy.deepcopy(self.captured)


class FakeModel:
    def __init__(self, content=None, error=None):
        self.content = json.dumps(observation(), ensure_ascii=False) if content is None else content
        self.error = error
        self.messages = []

    def invoke(self, messages):
        self.messages.append(copy.deepcopy(messages))
        if self.error:
            raise self.error
        return types.SimpleNamespace(content=self.content)


class VisionEvidenceTests(unittest.TestCase):
    def analyze(self, captured=None, content=None, error=None):
        client, model = FakeClient(captured), FakeModel(content, error)
        report = vision.VisionAnalyzer(client, CONFIG, model).analyze("寻找红色物体并报告周围环境")
        return report, client, model

    def test_exact_jpeg_bytes_are_sent_to_model_with_query(self):
        report, client, model = self.analyze()
        self.assertEqual(report["status"], "ok")
        self.assertEqual(client.calls, ["left"])
        self.assertEqual(len(model.messages), 1)
        messages = model.messages[0]
        self.assertEqual([m["role"] for m in messages], ["system", "user"])
        blocks = messages[1]["content"]
        self.assertIn("寻找红色物体", blocks[0]["text"])
        url = blocks[1]["image_url"]["url"]
        self.assertEqual(url, "data:image/jpeg;base64," + JPEG_B64)
        jpeg = base64.b64decode(url.split(",", 1)[1], validate=True)
        self.assertEqual(jpeg, base64.b64decode(JPEG_B64))
        self.assertIn(b"JFIF", jpeg)
        self.assertNotIn("image_base64", json.dumps(report))
        self.assertNotIn(CONFIG["api_key"], json.dumps(report))
        self.assertEqual(report["captured_at_sim_s"], 42.0)
        self.assertEqual(report["camera"]["stamp_sim_s"], 42.0)

    def test_stale_future_and_nonfinite_photos_never_reach_model(self):
        for key, age in (("age_seconds", 2.01), ("age_seconds", -0.01),
                         ("measurement_age_seconds", 0.501),
                         ("measurement_age_seconds", -0.051),
                         ("measurement_age_seconds", float("nan"))):
            with self.subTest(key=key, age=age):
                captured = snapshot()
                captured[key] = age
                report, client, model = self.analyze(captured)
                self.assertEqual(report["status"], "error")
                self.assertEqual(client.calls, ["left"])
                self.assertEqual(model.messages, [])

    def test_invalid_image_encoding_and_mime_never_reach_model(self):
        for change in ({"image_base64": "not base64!"}, {"mime_type": "image/png"},
                       {"image_base64": base64.b64encode(b"not a jpeg").decode()},
                       {"image_base64": "A" * 3000001}):
            with self.subTest(change=list(change)):
                captured = snapshot()
                captured.update(change)
                report, _, model = self.analyze(captured)
                self.assertEqual(report["status"], "error")
                self.assertEqual(model.messages, [])

    def test_capture_error_and_missing_config_do_not_call_model(self):
        model = FakeModel()
        client = FakeClient(error=OSError("camera disconnected"))
        report = vision.VisionAnalyzer(client, CONFIG, model).analyze("看看画面")
        self.assertEqual(report["status"], "error")
        self.assertEqual(model.messages, [])
        client = FakeClient()
        report = vision.VisionAnalyzer(client, None, model).analyze("看看画面")
        self.assertIn("未配置", report["error"])
        self.assertEqual(client.calls, [])

    def test_model_failure_preserves_diagnosis_and_redacts_secret(self):
        error = RuntimeError("provider rejected " + CONFIG["api_key"] + " data:image/jpeg;base64," + JPEG_B64)
        report, _, model = self.analyze(error=error)
        self.assertEqual(report["status"], "error")
        self.assertEqual(len(model.messages), 1)
        self.assertEqual(report["error_type"], "RuntimeError")
        self.assertNotIn(CONFIG["api_key"], json.dumps(report))
        self.assertNotIn(JPEG_B64, json.dumps(report))

    def test_http_errors_and_timeout_are_user_readable(self):
        for status, expected in ((400, "拒绝请求"), (401, "认证失败"), (403, "访问被拒绝"),
                                 (404, "模型不存在"), (429, "限流"), (503, "服务异常")):
            with self.subTest(status=status):
                error = RuntimeError("secret diagnostic " + CONFIG["api_key"])
                error.status_code, error.request_id = status, "request-123"
                report, _, _ = self.analyze(error=error)
                self.assertEqual(report["http_status"], status)
                self.assertEqual(report["request_id"], "request-123")
                self.assertIn(expected, report["error"])
                self.assertNotIn(CONFIG["api_key"], json.dumps(report))
        report, _, _ = self.analyze(error=TimeoutError("socket timed out"))
        self.assertIn("超时", report["error"])

    def test_request_identifier_also_redacts_credentials_and_images(self):
        error = RuntimeError("failed")
        error.request_id = CONFIG["api_key"] + " data:image/jpeg;base64," + JPEG_B64
        report, _, _ = self.analyze(error=error)
        self.assertNotIn(CONFIG["api_key"], json.dumps(report))
        self.assertNotIn("data:image/jpeg;base64", json.dumps(report))

    def test_empty_targets_are_a_successful_observation_not_a_failure(self):
        report, _, _ = self.analyze(content=json.dumps(observation([])))
        self.assertEqual(report["status"], "ok")
        self.assertEqual(report["targets"], [])
        self.assertTrue(any("未搜索视野外" in v for v in report["limitations"]))

    def test_model_cannot_supply_depth_or_world_coordinates(self):
        claimed = target()
        claimed.update(distance_m=4, world_position=[3, 5, 1],
                       approximate_location={"frame": "gazebo_world", "x_m": 99, "method": "measured_depth"})
        report, _, _ = self.analyze(content=json.dumps(observation([claimed])))
        found = report["targets"][0]
        self.assertNotIn("distance_m", found)
        self.assertNotIn("world_position", found)
        self.assertEqual(found["approximate_location"]["method"], "bearing_only")
        self.assertNotIn("x_m", found["approximate_location"])


class ObservationValidationTests(unittest.TestCase):
    def test_invalid_boxes_and_nonfinite_numbers_are_rejected(self):
        for bbox in ([0.3, 0.2, 0.3, 0.8], [0.7, 0.2, 0.3, 0.8],
                     [-0.1, 0.2, 0.3, 0.8], [0.1, 0.2, 1.1, 0.8],
                     [True, 0.2, 0.3, 0.8], [0.1, 0.2, float("nan"), 0.8],
                     [0.1, 0.2, float("inf"), 0.8], [0.1, 0.2, 0.3]):
            with self.subTest(bbox=bbox), self.assertRaises(ValueError):
                vision.parse_observation(json.dumps(observation([target(bbox=bbox)])))

    def test_invalid_confidence_basis_and_target_count_are_rejected(self):
        for change in ({"confidence": -0.01}, {"confidence": 1.01}, {"confidence": True},
                       {"confidence": float("nan")}, {"position_basis": "measured_depth"},
                       {"surroundings": "invented neighbours"}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                obj = target()
                obj.update(change)
                vision.parse_observation(json.dumps(observation([obj])))
        with self.assertRaises(ValueError):
            vision.parse_observation(json.dumps(observation([target()] * 9)))

    def test_non_object_non_json_and_missing_required_fields_are_rejected(self):
        for content in ("[]", "not JSON", "{}", '{"summary":"nothing","targets":null}'):
            with self.subTest(content=content), self.assertRaises(ValueError):
                vision.parse_observation(content)

    def test_text_blocks_and_unknown_confidence_are_supported(self):
        obj = target()
        obj["confidence"] = None
        raw = json.dumps(observation([obj]), ensure_ascii=False)
        parsed = vision.parse_observation([{"type": "text", "text": raw[:30]},
                                           {"type": "text", "text": raw[30:]}])
        self.assertIsNone(parsed["targets"][0]["confidence"])
        self.assertEqual(parsed["targets"][0]["label"], obj["label"])


class GroundProjectionTests(unittest.TestCase):
    def locate(self, obj=None, captured=None):
        obj = target("flat_ground_marking") if obj is None else obj
        vision.target_location(obj, snapshot() if captured is None else captured)
        return obj["approximate_location"]

    def test_known_downward_optical_pose_projects_center_to_floor(self):
        location = self.locate()
        self.assertEqual(location["method"], "ground_plane_projection")
        self.assertEqual([location[k] for k in ("x_m", "y_m", "z_m")], [1.0, 2.0, 0.003])
        self.assertEqual(location["anchor"], "bbox_center")
        self.assertTrue(any("不是 VIO 或实测深度" in a for a in location["assumptions"]))

    def test_visible_contact_uses_bottom_center_not_object_center(self):
        location = self.locate(target("visible_ground_contact"))
        self.assertEqual(location["anchor"], "bbox_bottom_center")
        self.assertEqual([location[k] for k in ("x_m", "y_m", "z_m")], [1.0, 1.4, 0.003])

    def test_off_axis_projection_respects_optical_x_right_y_down(self):
        location = self.locate(target("flat_ground_marking", [0.65, 0.55, 0.75, 0.65]))
        self.assertEqual([location[k] for k in ("x_m", "y_m")], [2.6, 1.4])
        # Standard level Iris optical rotation: x -> world -y, y -> world -z, z -> world +x.
        for axis, expected in (([1, 0, 0], [0, -1, 0]), ([0, 1, 0], [0, 0, -1]),
                               ([0, 0, 1], [1, 0, 0])):
            for value, wanted in zip(vision.rotate_vector([-0.5, 0.5, -0.5, 0.5], axis), expected):
                self.assertAlmostEqual(value, wanted)

    def test_actual_pitched_camera_optical_rotation_points_forward_and_down(self):
        # R_y(.35) times the standard optical rotation, independently expanded.
        s, c = math.sin(0.35/2), math.cos(0.35/2)
        q = [-(c+s)/2, (c+s)/2, (s-c)/2, (c-s)/2]
        ray = vision.rotate_vector(q, [0, 0, 1])
        self.assertAlmostEqual(ray[0], math.cos(.35))
        self.assertAlmostEqual(ray[1], 0)
        self.assertAlmostEqual(ray[2], -math.sin(.35))
        captured = snapshot()
        captured["camera_pose_world"].update(position_m=[0, 0, 1.003], orientation_xyzw=q)
        location = self.locate(captured=captured)
        self.assertEqual(location["method"], "ground_plane_projection")
        self.assertAlmostEqual(location["x_m"], 1/math.tan(.35), places=2)
        self.assertEqual(location["y_m"], 0)

    def test_near_horizon_and_upward_rays_only_provide_bearing(self):
        for angle in (0.0, 0.01, -0.2):
            with self.subTest(angle=angle):
                s, c = math.sin(angle/2), math.cos(angle/2)
                captured = snapshot()
                captured["camera_pose_world"]["orientation_xyzw"] = [-(c+s)/2, (c+s)/2, (s-c)/2, (c-s)/2]
                self.assertEqual(self.locate(captured=captured)["method"], "bearing_only")

    def test_clipped_targets_only_provide_bearing_even_with_contact_claim(self):
        for basis in ("flat_ground_marking", "visible_ground_contact"):
            for bbox in ([0, 0.4, 0.6, 0.6], [0.4, 0.4, 1, 0.6], [0.4, 0.4, 0.6, 1]):
                with self.subTest(basis=basis, bbox=bbox):
                    location = self.locate(target(basis, bbox))
                    self.assertEqual(location["method"], "bearing_only")
                    self.assertIn("裁切", location["reason"])

    def test_unsynchronized_nonfinite_and_invalid_poses_only_provide_bearing(self):
        for change in ({"time_alignment_error_s": 0.051}, {"time_alignment_error_s": -10},
                       {"time_alignment_error_s": float("nan")}, {"orientation_xyzw": [0, 0, 0, 0]},
                       {"orientation_xyzw": [0, 0, 0, 2]}, {"position_m": [1, 2, float("nan")]},
                       {"position_m": [1, 2, 0]}):
            with self.subTest(change=change):
                captured = snapshot()
                captured["camera_pose_world"].update(change)
                self.assertEqual(self.locate(captured=captured)["method"], "bearing_only")

    def test_missing_intrinsics_and_outside_known_floor_do_not_invent_position(self):
        captured = snapshot()
        captured.pop("camera_intrinsics")
        self.assertEqual(self.locate(captured=captured)["method"], "image_only")
        captured = snapshot()
        captured["camera_pose_world"]["position_m"] = [11, 0, 4.003]
        self.assertEqual(self.locate(captured=captured)["method"], "bearing_only")

    def test_unknown_coordinate_frames_and_nonzero_distortion_only_provide_bearing(self):
        for field, value in (("location_frame", "vio_world"), ("optical_axes", "body_flu"),
                             ("distortion_coefficients", [0.1, 0, 0, 0, 0]),
                             ("distortion_coefficients", [float("nan"), 0, 0, 0, 0])):
            with self.subTest(field=field):
                captured = snapshot()
                if field == "optical_axes":
                    captured["camera_pose_world"][field] = value
                elif field == "distortion_coefficients":
                    captured["camera_intrinsics"][field] = value
                else:
                    captured[field] = value
                self.assertEqual(self.locate(captured=captured)["method"], "bearing_only")


@unittest.skipUnless(importlib.util.find_spec("dotenv"), "optional python-dotenv is not installed")
class VisionConfigurationTests(unittest.TestCase):
    def test_provider_default_inherits_credentials_but_keeps_planning_model(self):
        primary = {**CONFIG, "model": "deepseek-v3.2"}
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(os.environ, {}, clear=True):
            env_file = Path(directory) / ".env"
            env_file.write_text("", encoding="utf-8")
            config = vision.vision_configuration(env_file, primary)
        self.assertEqual(config["model"], "qwen3-vl-plus")
        self.assertEqual(config["api_key"], primary["api_key"])
        self.assertEqual(primary["model"], "deepseek-v3.2")
        self.assertNotIn("api_key", vision.model_metadata(config))

    def test_explicit_vision_config_overrides_provider_and_missing_model_is_unconfigured(self):
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(os.environ, {}, clear=True):
            env_file = Path(directory) / ".env"
            env_file.write_text("VISION_MODEL=custom-vl\nVISION_API_KEY=vision-key\n"
                                "VISION_BASE_URL=https://vision.example/v1\n", encoding="utf-8")
            config = vision.vision_configuration(env_file, CONFIG)
            self.assertEqual(config, {"model": "custom-vl", "api_key": "vision-key",
                                      "base_url": "https://vision.example/v1"})
            env_file.write_text("", encoding="utf-8")
            self.assertIsNone(vision.vision_configuration(env_file, {
                "api_key": "primary-key", "base_url": "https://unknown.example/v1"}))


if __name__ == "__main__":
    unittest.main()
