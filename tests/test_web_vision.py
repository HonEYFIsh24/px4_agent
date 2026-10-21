"""Observe-mode integration using local fakes; no ROS or model service is contacted."""
import base64
import copy
import json
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

from test_web_server import MODEL, FakeBridge, application, completed, tool_result
from vision import VisionAnalyzer

VISION = {"model": "test-vision", "base_url": "https://vision.example/v1", "api_key": "private-vision-key"}
IMAGE_BYTES = b"\xff\xd8unit-test-private-camera-data\xff\xd9"
IMAGE_BASE64 = base64.b64encode(IMAGE_BYTES).decode()
OBSERVATION = {"summary": "画面左侧可见一个标记", "targets": [{"label": "标记", "confidence": .8,
               "bbox": [.1, .2, .3, .4], "surroundings": ["右侧可见地面"],
               "position_basis": "image_only"}], "limitations": ["未测量深度"]}


class CameraBridge(FakeBridge):
    def __init__(self):
        super().__init__()
        self.camera_reads = []

    def capture_camera(self, side="left"):
        self.camera_reads.append(side)
        return {"status": "ok", "side": side, "mime_type": "image/jpeg", "image_base64": IMAGE_BASE64,
                "width": 640, "height": 480, "stamp_sim_s": 42., "age_seconds": .1,
                "measurement_age_seconds": .04,
                "camera_intrinsics": {"fx": 320., "fy": 320., "cx": 320., "cy": 240., "D": [0]*5}}


class FakeVisionModel:
    def __init__(self, exception=None):
        self.exception = exception
        self.requests = []

    def invoke(self, messages):
        self.requests.append(copy.deepcopy(messages))
        if self.exception:
            raise self.exception
        return SimpleNamespace(content=json.dumps(OBSERVATION))


def assistant_result(payload, reply):
    return {"messages": [payload["messages"][0], {"role": "assistant", "content": reply}]}


class WebVisionTests(unittest.TestCase):
    def assert_public_is_private(self, command, state):
        encoded = json.dumps({"command": command, "state": state}, ensure_ascii=False)
        for private in (MODEL["api_key"], VISION["api_key"], IMAGE_BASE64,
                        "image_base64", "data:image/jpeg;base64,"):
            self.assertNotIn(private, encoded)

    def test_observe_blocks_all_agent_flight_actions_but_operator_controls_remain_available(self):
        ready, release = threading.Event(), threading.Event()
        denied = {}

        def script(client, payload, config):
            with ThreadPoolExecutor(max_workers=1) as worker:
                for path in ("/mission", "/land", "/stop"):
                    try:
                        worker.submit(client.request, path, {}).result()
                    except RuntimeError as exc:
                        denied[path] = str(exc)
            ready.set()
            release.wait(3)
            return assistant_result(payload, "观察完成")

        bridge = FakeBridge()
        app = application(bridge, script, vision_config=VISION)
        accepted = app.submit("只观察当前画面", mode="observe")
        try:
            self.assertTrue(ready.wait(1))
            self.assertEqual(set(denied), {"/mission", "/land", "/stop"})
            self.assertTrue(all("仅允许视觉观察" in text for text in denied.values()))
            self.assertEqual(bridge.calls, [])
            app.interrupt("/stop")
            app.interrupt("/land")
            self.assertEqual(bridge.calls, ["/stop", "/land"])
        finally:
            release.set()
        command = completed(app, accepted["id"])
        self.assertEqual(command["status"], "failed")
        self.assertIn("中断", command["reply"])

    def test_observe_cannot_claim_success_without_real_inspection_audit(self):
        def script(client, payload, config):
            # A plausible message alone is insufficient evidence that a snapshot was inspected.
            return {"messages": [payload["messages"][0],
                    tool_result("inspect_scene", {"status": "ok", "summary": "虚构的观察"}),
                    {"role": "assistant", "content": "视觉观察已成功完成"}]}

        app = application(FakeBridge(), script, vision_config=VISION)
        command = completed(app, app.submit("寻找标记", mode="observe")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertIn("没有实际调用视觉工具", command["error"])
        self.assertNotIn("已成功完成", command["reply"])
        self.assertEqual(command["perception"]["status"], "error")
        self.assertEqual(app.state()["latest_perception"], command["perception"])

    def test_inspection_error_is_failed_from_audit_without_tool_message(self):
        failure = {"status": "error", "query": "寻找标记", "model": VISION["model"],
                   "error": "相机快照已过期", "targets": [], "limitations": []}

        def script(client, payload, config):
            with ThreadPoolExecutor(max_workers=1) as worker:
                worker.submit(client.record, "inspect_scene", failure).result()
            return assistant_result(payload, "观察成功，已发现目标")

        app = application(FakeBridge(), script, vision_config=VISION)
        command = completed(app, app.submit("寻找标记", mode="observe")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual(command["error"], "相机快照已过期")
        self.assertIn("相机快照已过期", command["reply"])
        self.assertNotIn("观察成功", command["reply"])
        self.assertEqual(command["perception"], failure)
        self.assertEqual(app.state()["latest_perception"], failure)

    def test_success_exposes_observations_without_snapshot_or_credentials(self):
        bridge = CameraBridge()
        model = FakeVisionModel()

        def script(client, payload, config):
            analyzer = VisionAnalyzer(client, client.vision_config, model=model)
            with ThreadPoolExecutor(max_workers=1) as worker:
                observation = worker.submit(analyzer.analyze, payload["messages"][0]["content"]).result()
                worker.submit(client.record, "inspect_scene", observation).result()
            return assistant_result(payload, "目标坐标为 ENU (0.1, 0.2)，已确定精确距离")

        app = application(bridge, script, vision_config=VISION)
        command = completed(app, app.submit("寻找标记", mode="observe")["id"])
        self.assertEqual(command["status"], "succeeded")
        self.assertEqual(command["mode"], "observe")
        self.assertEqual(command["perception"]["status"], "ok")
        self.assertEqual(command["perception"]["targets"][0]["label"], "标记")
        self.assertEqual(command["perception"]["captured_at_sim_s"], 42.)
        self.assertNotIn("目标坐标为 ENU", command["reply"])
        self.assertNotIn("已确定精确距离", command["reply"])
        self.assertIn("未测量距离", command["reply"])
        self.assertIn("画面左侧上部", command["reply"])
        state = app.state()
        self.assertEqual(state["latest_perception"], command["perception"])
        self.assertEqual(state["vision_model"]["name"], VISION["model"])
        self.assertEqual(state["vision_model"]["provider"], "vision.example")
        self.assertEqual(bridge.camera_reads, ["left"])
        self.assertEqual(bridge.calls, [])
        self.assertIn(IMAGE_BASE64, json.dumps(model.requests))
        self.assert_public_is_private(command, state)

    def test_planner_exception_redacts_both_model_credentials(self):
        def script(*args):
            raise RuntimeError("planner failed " + MODEL["api_key"] + " " + VISION["api_key"])

        app = application(FakeBridge(), script, vision_config=VISION)
        command = completed(app, app.submit("描述当前场景", mode="observe")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertIn("planner failed", command["error"])
        self.assert_public_is_private(command, app.state())

    def test_vision_exception_redacts_credentials_and_encoded_image(self):
        failure = RuntimeError("vision failed " + VISION["api_key"] + " data:image/jpeg;base64," + IMAGE_BASE64)
        failure.request_id = "request-" + VISION["api_key"]
        model = FakeVisionModel(exception=failure)

        def script(client, payload, config):
            observation = VisionAnalyzer(client, client.vision_config, model=model).analyze("寻找标记")
            client.record("inspect_scene", observation)
            return assistant_result(payload, "观察已成功完成")

        app = application(CameraBridge(), script, vision_config=VISION)
        command = completed(app, app.submit("寻找标记", mode="observe")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertIn("vision failed", command["error"])
        self.assertIn("已隐藏密钥", command["perception"]["error"])
        self.assertIn("已隐藏图像数据", command["perception"]["error"])
        self.assert_public_is_private(command, app.state())

    def test_raw_inspection_error_does_not_expose_vision_key_in_public_perception(self):
        def script(client, payload, config):
            client.record("inspect_scene", {"status": "error", "error": "vision failed " + VISION["api_key"],
                                             "camera": {"image_base64": IMAGE_BASE64, "api_key": VISION["api_key"]},
                                             "diagnostic": "data:image/jpeg;base64," + IMAGE_BASE64,
                                             "targets": [], "limitations": []})
            return assistant_result(payload, "观察成功")

        app = application(FakeBridge(), script, vision_config=VISION)
        command = completed(app, app.submit("描述场景", mode="observe")["id"])
        self.assertEqual(command["status"], "failed")
        self.assert_public_is_private(command, app.state())


if __name__ == "__main__":
    unittest.main()
