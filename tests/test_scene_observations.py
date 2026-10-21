"""Multi-stage observation contracts using local audit and flight fakes only."""
import copy
import json
import unittest

from test_web_server import MODEL, FakeBridge, application, completed


IMAGE_SECRET = "private-scene-camera-bytes"
VISION = {"model": "local-scene-vlm", "base_url": "https://vision.example/v1",
          "api_key": "private-scene-vision-key"}


def capture(capture_id):
    return {"status": "ok", "capture_id": capture_id, "architecture": "single_vlm",
            "camera": {"stamp_sim_s": 40.0, "image_sha256": "fake-source-hash"}}


def report(capture_id, summary):
    return {"status": "ok", "capture_id": capture_id, "architecture": "single_vlm",
            "summary": summary, "query": summary, "model": VISION["model"],
            "camera": {"stamp_sim_s": 40.0, "image_sha256": "fake-source-hash"},
            "targets": [], "limitations": ["仅当前视野"]}


def finish(payload, prose="两个观察阶段都完成了"):
    return {"messages": [payload["messages"][0], {"role": "assistant", "content": prose}]}


def scene_application(script, bridge=None):
    return application(bridge or FakeBridge(), script, vision_config=VISION,
                       agent_architecture="single_vlm")


class SceneObservationTests(unittest.TestCase):
    def run_task(self, script, minimum=1):
        app = scene_application(script)
        accepted = app.submit("按指定路线在两个位置观察，然后返航降落", min_observations=minimum)
        return app, completed(app, accepted["id"])

    def assert_private(self, command, state):
        value = json.dumps({"command": command, "state": state}, ensure_ascii=False)
        for secret in (MODEL["api_key"], VISION["api_key"], IMAGE_SECRET,
                       "image_base64", "data:image/jpeg;base64,"):
            self.assertNotIn(secret, value)

    def test_task_contract_cannot_succeed_from_prose_without_capture_and_report(self):
        app, command = self.run_task(lambda client, payload, config: finish(payload))
        self.assertEqual(command["status"], "failed")
        self.assertTrue(command["error"])
        self.assertEqual(command["perceptions"], [])
        self.assertNotEqual(command["reply"], "两个观察阶段都完成了")
        self.assert_private(command, app.state())

    def test_report_without_actual_capture_cannot_satisfy_task_contract(self):
        def script(client, payload, config):
            client.record("inspect_scene", report("invented-capture", "虚构阶段"))
            return finish(payload)

        _, command = self.run_task(script)
        self.assertEqual(command["status"], "failed")

    def test_single_report_does_not_satisfy_two_stage_contract(self):
        def script(client, payload, config):
            client.record("capture_scene", capture("one"))
            client.record("inspect_scene", report("one", "第一个位置"))
            return finish(payload)

        _, command = self.run_task(script, minimum=2)
        self.assertEqual(command["status"], "failed")
        self.assertEqual(len(command["perceptions"]), 1)

    def test_every_successful_capture_requires_its_own_valid_report(self):
        def script(client, payload, config):
            client.record("capture_scene", capture("one"))
            client.record("inspect_scene", report("one", "第一个位置"))
            client.record("capture_scene", capture("two"))
            return finish(payload)

        _, command = self.run_task(script, minimum=1)
        self.assertEqual(command["status"], "failed")
        self.assertEqual(command["perception"]["capture_id"], "one")

    def test_capture_alone_is_incomplete_even_when_no_minimum_is_declared(self):
        def script(client, payload, config):
            client.record("capture_scene", capture("unreported"))
            return finish(payload)

        _, command = self.run_task(script, minimum=0)
        self.assertEqual(command["status"], "failed")

    def test_repeated_same_capture_report_does_not_count_as_two_views(self):
        def script(client, payload, config):
            client.record("capture_scene", capture("one"))
            client.record("inspect_scene", report("one", "第一份报告"))
            client.record("inspect_scene", report("one", "重复同一帧"))
            return finish(payload)

        _, command = self.run_task(script, minimum=2)
        self.assertEqual(command["status"], "failed")

    def test_two_complete_reports_remain_ordered_and_last_report_is_ui_compatible(self):
        def script(client, payload, config):
            for capture_id, summary in (("first", "南侧可见路边车辆"), ("second", "北侧可见树木和座椅")):
                client.record("capture_scene", capture(capture_id))
                client.record("inspect_scene", report(capture_id, summary))
            return finish(payload)

        app, command = self.run_task(script, minimum=2)
        self.assertEqual(command["status"], "succeeded")
        self.assertEqual([item["capture_id"] for item in command["perceptions"]], ["first", "second"])
        self.assertEqual([item["summary"] for item in command["perceptions"]],
                         ["南侧可见路边车辆", "北侧可见树木和座椅"])
        self.assertEqual(command["perception"], command["perceptions"][-1])
        self.assertEqual(app.state()["latest_perception"], command["perception"])
        self.assert_private(command, app.state())

    def test_later_model_trace_does_not_rewrite_first_stage_report(self):
        saved = []

        def script(client, payload, config):
            client.record("capture_scene", capture("first"))
            client.record("inspect_scene", report("first", "第一阶段"))
            saved.append(copy.deepcopy(client.application.current_command["perceptions"][0]))
            client.record("vlm_call", {"status": "ok", "model": VISION["model"], "capture_ids": ["second"]})
            client.record("capture_scene", capture("second"))
            client.record("inspect_scene", report("second", "第二阶段"))
            return finish(payload)

        _, command = self.run_task(script, minimum=2)
        self.assertEqual(command["status"], "succeeded")
        self.assertEqual(command["perceptions"][0], saved[0])

    def test_all_stage_reports_are_redacted_in_public_state(self):
        def script(client, payload, config):
            for identifier in ("first", "second"):
                client.record("capture_scene", capture(identifier))
                value = report(identifier, "当前阶段")
                value["camera"].update(image_base64=IMAGE_SECRET, api_key=VISION["api_key"])
                value["nested"] = {"detail": MODEL["api_key"], "data": "data:image/jpeg;base64," + IMAGE_SECRET}
                client.record("inspect_scene", value)
            return finish(payload)

        app, command = self.run_task(script, minimum=2)
        self.assertEqual(command["status"], "succeeded")
        self.assertEqual(len(command["perceptions"]), 2)
        self.assert_private(command, app.state())

    def test_first_observation_failure_stays_failed_and_blocks_later_flight(self):
        denied = []
        bridge = FakeBridge()

        def script(client, payload, config):
            client.record("capture_scene", capture("first"))
            client.record("inspect_scene", {"status": "error", "capture_id": "first",
                "error": "首个观察阶段失败", "targets": [], "limitations": []})
            client.record("inspect_scene", report("first", "错误后的成功声明"))
            try:
                client.request("/mission", {"steps": [{"action": "goto", "x": 2, "y": 0, "altitude": 2}]})
            except RuntimeError as exc:
                denied.append(str(exc))
            return finish(payload)

        app = scene_application(script, bridge)
        command = completed(app, app.submit("观察后移动", min_observations=1)["id"])
        self.assertEqual(command["status"], "failed")
        self.assertTrue(command["error"])
        self.assertEqual([item["status"] for item in command["perceptions"]], ["error", "error"])
        self.assertEqual(command["perceptions"][0]["error"], "首个观察阶段失败")
        self.assertIn("首个观察阶段失败", command["perceptions"][-1]["error"])
        self.assertTrue(denied)
        self.assertEqual(bridge.calls, [])

    def test_first_flight_failure_cannot_be_erased_by_successful_reports(self):
        bridge = FakeBridge()
        bridge.mission.update(status="failed", error="首个飞行阶段失败")

        def script(client, payload, config):
            client.execute([{"action": "takeoff", "altitude": 2}], "first flight")
            client.record("capture_scene", capture("first"))
            client.record("inspect_scene", report("first", "后续报告"))
            return finish(payload)

        app = scene_application(script, bridge)
        command = completed(app, app.submit("起飞后观察", min_observations=1)["id"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual(command["error"], "首个飞行阶段失败")

    def test_saved_visual_report_is_preserved_when_return_navigation_fails_and_lands(self):
        class FailedReturnBridge(FakeBridge):
            def __init__(self):
                super().__init__()
                self.armed = False
                self.flight_count = 0

            def state(self):
                state = super().state()
                state.update(armed=self.armed, landed_state=2 if self.armed else 1)
                return state

            def request(self, path, payload=None):
                if path == "/mission":
                    self.flight_count += 1
                    self.armed = True
                    self.mission = {"id": "phase-{}".format(self.flight_count),
                        "status": "succeeded" if self.flight_count == 1 else "failed",
                        "step": 1, "steps": payload["steps"],
                        "error": None if self.flight_count == 1 else "start is blocked or lacks observed LiDAR clearance"}
                elif path == "/land":
                    self.armed = False
                    self.mission = {"id": "safety-land", "status": "succeeded", "step": 1,
                                    "steps": [{"action": "land"}], "error": None}
                return super().request(path, payload)

        bridge = FailedReturnBridge()

        def script(client, payload, config):
            client.execute([{"action": "takeoff", "altitude": 3.5}], "owned flight")
            client.record("capture_scene", capture("red-container"))
            client.record("inspect_scene", report("red-container", "已发现红色集装箱"))
            client.execute([{"action": "return_home"}], "autonomous return")
            return finish(payload, "观察、返航和降落均已完成")

        app = scene_application(script, bridge)
        command = completed(app, app.submit("起飞观察红箱，再返航降落", min_observations=1)["id"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual(command["perception"]["status"], "ok")
        self.assertEqual(command["perceptions"][0]["summary"], "已发现红色集装箱")
        self.assertIn("视觉观察报告已保存", command["reply"])
        self.assertIn("飞行任务失败", command["reply"])
        self.assertIn("start is blocked", command["reply"])
        self.assertNotIn("视觉观察未完成", command["reply"])
        self.assertNotIn("检查视觉报告和模型配置", command["reply"])
        self.assertIn("安全降落并上锁", command["reply"])
        self.assertEqual(command["safety_landing"]["status"], "ok")
        self.assertFalse(bridge.state()["armed"])
        self.assertEqual(bridge.calls, ["/mission", "/mission", "/land"])
        self.assert_private(command, app.state())

    def test_saved_report_does_not_hide_a_later_visual_failure(self):
        def script(client, payload, config):
            client.record("capture_scene", capture("first"))
            client.record("inspect_scene", report("first", "首份观察已保存"))
            client.record("capture_scene", capture("second"))
            client.record("inspect_scene", {"status": "error", "capture_id": "second",
                "error": "第二帧视觉报告解析失败", "targets": [], "limitations": []})
            # A rejected flight request is audited as a flight failure too.
            try:
                client.execute([{"action": "takeoff", "altitude": 3.5}], "blocked flight")
            except RuntimeError:
                pass
            return finish(payload)

        app = scene_application(script)
        command = completed(app, app.submit("两次观察后起飞", min_observations=2)["id"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual([value["status"] for value in command["perceptions"]], ["ok", "error"])
        self.assertIn("视觉观察未完成", command["reply"])
        self.assertNotIn("视觉观察报告已保存；飞行任务失败", command["reply"])

    def test_saved_report_and_flight_failure_do_not_hide_a_model_api_error(self):
        bridge = FakeBridge()
        bridge.mission.update(status="failed", error="return navigation blocked")

        def script(client, payload, config):
            client.record("capture_scene", capture("first"))
            client.record("inspect_scene", report("first", "首份观察已保存"))
            client.execute([{"action": "land"}], "failed flight")
            client.record("vlm_call", {"status": "error", "error": "provider disconnected"})
            raise RuntimeError("provider disconnected")

        app = scene_application(script, bridge)
        command = completed(app, app.submit("观察后返航", min_observations=1)["id"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual(command["model_calls"][-1]["status"], "error")
        self.assertEqual(command["perception"]["status"], "ok")
        self.assertIn("视觉观察未完成", command["reply"])
        self.assertNotIn("视觉观察报告已保存；飞行任务失败", command["reply"])

    def test_minimum_reports_accepts_only_integer_zero_to_eight(self):
        app = scene_application(lambda client, payload, config: finish(payload))
        for invalid in (True, False, -1, 9, 1.0, "1", [], {}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                app.submit("测试观察合同", min_observations=invalid)

    def test_observe_mode_defaults_to_one_required_report(self):
        app = scene_application(lambda client, payload, config: finish(payload))
        command = completed(app, app.submit("只观察", mode="observe")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual(command["perceptions"], [])

    def test_task_without_visual_request_or_capture_needs_no_report(self):
        app = scene_application(lambda client, payload, config: finish(payload, "当前飞机已上锁"))
        command = completed(app, app.submit("查询飞行状态")["id"])
        self.assertEqual(command["status"], "succeeded")
        self.assertEqual(command["perceptions"], [])


class FinalReplyAttributionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # These cases execute the actual LangChain graph and production flight
        # tools. The model/bridge remain offline stateful fixtures.
        try:
            import httpx
            from openai import APIConnectionError, APITimeoutError
            from langchain.agents import create_agent
            from langchain_openai import ChatOpenAI
        except ImportError as exc:
            raise unittest.SkipTest("Final-reply graph tests require Agent Python 3.11 dependencies") from exc
        # Import production code after checking only optional dependencies:
        # a product or fixture failure must remain a failing test.
        from test_agent_reliability import runner
        cls.runner = runner

    def checked_case(self, identifier):
        result = self.runner.run_case(identifier)
        self.assertTrue(result["passed"], json.dumps([a for a in result["assertions"] if not a["passed"]],
                                                    ensure_ascii=False, indent=2))
        command = result["observed"]["command"]
        self.assertEqual(command["status"], "failed")
        self.assertEqual(command["model_calls"][-1]["status"], "error")
        return result

    def test_completed_flight_and_final_connection_error_have_separate_outcomes(self):
        result = self.checked_case("final_reply_connection_after_completed_flight")
        command = result["observed"]["command"]
        self.assertEqual(command["mission"]["status"], "succeeded")
        self.assertIsNone(command["safety_landing"])
        self.assertEqual(result["observed"]["bridge_paths"], ["/mission"])

    def test_completed_return_keeps_saved_observation_after_final_model_error(self):
        result = self.checked_case("final_reply_error_after_saved_observation")
        command = result["observed"]["command"]
        self.assertEqual(command["perception"]["status"], "ok")
        self.assertIn("视觉观察报告已保存", command["reply"])
        self.assertNotIn("视觉观察未完成", command["reply"])

    def test_unfinished_failed_cancelled_and_unconfirmed_flights_never_claim_completion(self):
        for identifier in ("timeout_after_owned_takeoff", "final_reply_error_after_unconfirmed_acceptance",
                           "final_reply_error_after_failed_flight", "final_reply_error_after_cancelled_flight",
                           "operator_interrupt_preserves_authority", "connection_before_flight"):
            with self.subTest(case=identifier):
                result = self.checked_case(identifier)
                self.assertNotIn("飞行执行已完成", result["observed"]["command"]["reply"])


if __name__ == "__main__":
    unittest.main()
