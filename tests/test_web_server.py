import copy
import json
import tempfile
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from web_server import CommandBusy, WebApplication, make_server

MODEL = {"model": "test-model", "base_url": "https://provider.example/v1", "api_key": "secret-test-key"}


def tool_result(name, value):
    return {"role": "tool", "name": name, "content": json.dumps(value)}


class FakeBridge:
    def __init__(self):
        self.offline = False
        self.calls = []
        self.mission = {"id": "flight-1", "status": "succeeded", "step": 1, "steps": [{"action": "land"}]}

    def state(self):
        if self.offline:
            raise OSError("connection refused")
        return {"ready": True, "armed": False, "position_enu_m": [0, 0, 0],
                "gazebo": {"available": True, "position_m": [0, 0, 0]},
                "mission": copy.deepcopy(self.mission)}

    def request(self, path, payload=None):
        self.calls.append(path)
        if self.offline:
            raise OSError("connection refused")
        if path in ("/mission", "/land"):
            return {"mission": copy.deepcopy(self.mission)}
        return {"requested": "cancel", "holding": False, "state": self.state()}

    def wait_for(self, accepted, on_progress=None):
        return self.state()


class ScriptedAgent:
    def __init__(self, client, script):
        self.client = client
        self.script = script

    def invoke(self, payload, config):
        return self.script(self.client, payload, config)


def application(bridge, script, **kwargs):
    return WebApplication(bridge, MODEL, lambda client, model: ScriptedAgent(client, script), **kwargs)


def completed(app, command_id):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        result = app.command(command_id)
        if result["status"] != "running":
            return result
        time.sleep(0.01)
    raise AssertionError("test command did not finish")


class CommandTests(unittest.TestCase):
    def test_failed_flight_cannot_be_success_from_model_prose(self):
        bridge = FakeBridge()
        bridge.mission.update(status="failed", error="position timed out")

        def script(client, payload, config):
            state = client.execute([{"action": "land"}], "test")
            return {"messages": [payload["messages"][0], tool_result("execute_mission", state),
                                 {"role": "assistant", "content": "任务已完成"}]}

        app = application(bridge, script)
        command = completed(app, app.submit("降落")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual(command["error"], "position timed out")
        self.assertEqual(command["mission"]["status"], "failed")
        self.assertNotIn("任务已完成", command["reply"])
        self.assertIn("position timed out", command["reply"])

    def test_failed_mission_blocks_retry_but_allows_land_and_next_user_command(self):
        bridge = FakeBridge()
        bridge.mission.update(status="failed", error="takeoff timed out")
        attempts = []
        turns = []

        def script(client, payload, config):
            turns.append(payload)
            client.execute([{"action": "takeoff", "altitude": 2}], "test")
            if len(turns) == 1:
                try:
                    with ThreadPoolExecutor(max_workers=1) as executor:
                        executor.submit(client.execute, [{"action": "takeoff", "altitude": 2}], "retry").result()
                except RuntimeError as exc:
                    attempts.append(str(exc))
                client.request("/land", {})
            return {"messages": [payload["messages"][0], {"role": "assistant", "content": "全部成功"}]}

        app = application(bridge, script)
        command = completed(app, app.submit("起飞")["id"])
        self.assertEqual(bridge.calls, ["/mission", "/land"])
        self.assertEqual(len(attempts), 1)
        self.assertIn("禁止自动重试", attempts[0])
        self.assertEqual(command["status"], "failed")
        self.assertNotIn("全部成功", command["reply"])
        app.interrupt("/land")
        self.assertEqual(bridge.calls[-1], "/land")
        bridge.mission.update(status="succeeded", error=None)
        command = completed(app, app.submit("重新起飞")["id"])
        self.assertEqual(command["status"], "succeeded")
        self.assertEqual(bridge.calls, ["/mission", "/land", "/land", "/mission"])

    def test_flight_request_or_poll_exception_blocks_retry(self):
        for phase in ("request", "poll"):
            with self.subTest(phase=phase):
                bridge = FakeBridge()
                original_wait = bridge.wait_for
                failures = []
                if phase == "request":
                    bridge.offline = True
                else:
                    def fail_wait(*args):
                        raise OSError("telemetry unavailable")
                    bridge.wait_for = fail_wait

                def script(client, payload, config):
                    try:
                        client.execute([{"action": "takeoff", "altitude": 2}], "test")
                    except OSError:
                        pass
                    bridge.offline = False
                    bridge.wait_for = original_wait
                    try:
                        client.execute([{"action": "takeoff", "altitude": 2}], "retry")
                    except RuntimeError as exc:
                        failures.append(str(exc))
                    return {"messages": [payload["messages"][0], {"role": "assistant", "content": "全部成功"}]}

                app = application(bridge, script)
                command = completed(app, app.submit("起飞")["id"])
                self.assertEqual(bridge.calls, ["/mission"])
                self.assertEqual(len(failures), 1)
                self.assertEqual(command["status"], "failed")
                self.assertNotIn("全部成功", command["reply"])

    def test_previous_failed_turn_does_not_poison_current_command(self):
        def script(client, payload, config):
            return {"messages": [{"role": "user", "content": "previous"},
                                 tool_result("execute_mission", {"status": "error", "error": "old"}),
                                 payload["messages"][0], tool_result("get_drone_state", client.state()),
                                 {"role": "assistant", "content": "当前在地面"}]}

        app = application(FakeBridge(), script)
        command = completed(app, app.submit("查看状态")["id"])
        self.assertEqual(command["status"], "succeeded")
        self.assertIsNone(command["error"])

    def test_stop_is_immediate_and_blocks_future_actions_of_interrupted_agent(self):
        entered, release = threading.Event(), threading.Event()

        def script(client, payload, config):
            entered.set()
            release.wait(3)
            try:
                with ThreadPoolExecutor(max_workers=1) as executor:
                    result = executor.submit(client.execute, [{"action": "takeoff", "altitude": 2}], "test").result()
            except Exception as exc:
                result = {"status": "error", "error": str(exc)}
            return {"messages": [payload["messages"][0], tool_result("execute_mission", result),
                                 {"role": "assistant", "content": "已停止"}]}

        bridge = FakeBridge()
        app = application(bridge, script)
        accepted = app.submit("起飞")
        self.assertTrue(entered.wait(1))
        with self.assertRaises(CommandBusy):
            app.submit("另一个任务")
        app.interrupt("/stop")
        self.assertEqual(bridge.calls, ["/stop"])
        release.set()
        command = completed(app, accepted["id"])
        self.assertEqual(command["status"], "failed")
        self.assertIn("不会继续执行", command["reply"])
        self.assertEqual(bridge.calls, ["/stop"])

    def test_tool_worker_thread_records_authoritative_mission_result(self):
        bridge = FakeBridge()

        def script(client, payload, config):
            with ThreadPoolExecutor(max_workers=1) as executor:
                executor.submit(client.execute, [{"action": "land"}], "test").result()
            # Independent audit still works when no ToolMessage is exposed in the final graph result.
            return {"messages": [payload["messages"][0], {"role": "assistant", "content": "完成"}]}

        app = application(bridge, script)
        command = completed(app, app.submit("降落")["id"])
        self.assertEqual(command["status"], "succeeded")
        self.assertEqual(command["mission"]["id"], "flight-1")
        bridge.mission.update(status="cancelled")
        command = completed(app, app.submit("再次降落")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual(command["mission"]["status"], "cancelled")

    def test_offline_state_does_not_invent_positions_or_expose_key(self):
        bridge = FakeBridge()
        bridge.offline = True
        app = application(bridge, lambda *_: {})
        state = app.state()
        self.assertFalse(state["ready"])
        self.assertFalse(state["gazebo"]["available"])
        self.assertNotIn("position_enu_m", state)
        self.assertEqual(state["model"]["provider"], "provider.example")
        self.assertNotIn(MODEL["api_key"], json.dumps(state))

    def test_model_exception_redacts_key(self):
        def script(*_):
            raise RuntimeError("unauthorized " + MODEL["api_key"])

        app = application(FakeBridge(), script)
        command = completed(app, app.submit("状态")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertNotIn(MODEL["api_key"], json.dumps(command))


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.web = root / "web"
        self.web.mkdir()
        (self.web / "index.html").write_text("<html>local operator</html>")
        (root / "private.txt").write_text("private")
        self.camera = root / "camera.jpg"
        self.bridge = FakeBridge()
        self.app = application(self.bridge, lambda *_: {"messages": [{"role": "assistant", "content": "ok"}]},
                               web_root=self.web, camera_path=self.camera)
        self.server = make_server(self.app, 0)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = "http://127.0.0.1:{}".format(self.server.server_port)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        self.temp.cleanup()

    def request(self, path, body=None, headers=None):
        request = Request(self.url + path, data=json.dumps(body).encode() if body is not None else None,
                          headers={"Content-Type": "application/json", **(headers or {})})
        return urlopen(request, timeout=2)

    def test_http_accepts_command_and_rejects_cross_origin_mutation(self):
        with self.request("/api/commands", {"prompt": "查看状态"}, {"Origin": self.url}) as response:
            self.assertEqual(response.status, 202)
            accepted = json.load(response)
        completed(self.app, accepted["id"])
        with self.request("/api/commands/" + accepted["id"]) as response:
            self.assertEqual(json.load(response)["status"], "succeeded")
        with self.assertRaises(HTTPError) as caught:
            self.request("/api/land", {}, {"Origin": "https://untrusted.example"})
        self.assertEqual(caught.exception.code, 403)
        self.assertEqual(self.bridge.calls, [])

    def test_stop_and_land_do_not_wait_for_model(self):
        with self.request("/api/stop", {}) as response:
            self.assertEqual(json.load(response)["requested"], "cancel")
        with self.request("/api/land", {}) as response:
            self.assertEqual(json.load(response)["mission"]["id"], "flight-1")
        self.assertEqual(self.bridge.calls, ["/stop", "/land"])

    def test_camera_freshness_and_static_path_boundary(self):
        with self.request("/") as response:
            self.assertIn(b"local operator", response.read())
        with self.assertRaises(HTTPError) as caught:
            self.request("/%2e%2e/private.txt")
        self.assertEqual(caught.exception.code, 404)
        with self.assertRaises(HTTPError) as caught:
            self.request("/api/camera.jpg")
        self.assertEqual(caught.exception.code, 404)
        self.camera.write_bytes(b"fake-test-jpeg")
        with self.request("/api/camera.jpg") as response:
            self.assertEqual(response.headers["Content-Type"], "image/jpeg")
            self.assertEqual(response.headers["Cache-Control"], "no-store")
            self.assertEqual(response.read(), b"fake-test-jpeg")
        with self.request("/api/camera/left.jpg") as response:
            self.assertEqual(response.read(), b"fake-test-jpeg")
        right = self.camera.with_name("camera_right.jpg")
        right.write_bytes(b"independent-right-frame")
        with self.request("/api/camera/right.jpg") as response:
            self.assertEqual(response.read(), b"independent-right-frame")
        with self.assertRaises(HTTPError) as caught:
            self.request("/api/camera/unknown.jpg")
        self.assertEqual(caught.exception.code, 404)
        import os
        os.utime(self.camera, (time.time() - 5, time.time() - 5))
        with self.assertRaises(HTTPError) as caught:
            self.request("/api/camera.jpg")
        self.assertEqual(caught.exception.code, 404)
        with self.request("/api/camera/right.jpg") as response:
            self.assertEqual(response.read(), b"independent-right-frame")


if __name__ == "__main__":
    unittest.main()
