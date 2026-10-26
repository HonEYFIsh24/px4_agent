"""Single VLM evidence boundaries with real LangChain graphs and local model fakes.

No ROS bridge or provider is contacted. Images intentionally exist in the fake model
requests; they must never appear in checkpoint state or public tool/audit reports.
"""
import copy
import json
import unittest
from typing import Any

try:
    from langchain.agents import create_agent  # Check v1 capability before importing the module.
    from langchain_core.language_models import BaseChatModel
    from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
    from langchain_core.outputs import ChatGeneration, ChatResult
    from langchain_core.tools import tool
    from pydantic import Field
except ImportError as exc:
    raise unittest.SkipTest("Single-VLM tests require Agent Python3.11 LangChain environment") from exc

import single_vlm
from test_vision import CONFIG, JPEG_B64, observation, snapshot, target
from test_web_server import FakeBridge, completed
from web_server import WebApplication


def encoded(value):
    return json.dumps(value, ensure_ascii=False,
                      default=lambda item: item.model_dump() if hasattr(item, "model_dump") else str(item))


def tool_call(name, args, call_id="local-call"):
    return AIMessage(content="", tool_calls=[{"name": name, "args": args,
                                             "id": call_id, "type": "tool_call"}])


def tool_payload(messages, name):
    for message in reversed(messages):
        if isinstance(message, ToolMessage) and message.name == name:
            return json.loads(message.content)
    raise AssertionError("No {} ToolMessage".format(name))


def capture_request(messages=None):
    return tool_call("capture_scene", {"query": "寻找红色标记及周围环境", "side": "left"}, "capture-call")


def report_request(messages):
    capture = tool_payload(messages, "capture_scene")
    return tool_call("report_scene", {"capture_id": capture["capture_id"], **observation()}, "report-call")


def image_urls(messages):
    return [block["image_url"]["url"] for message in messages
            if isinstance(message.content, list) for block in message.content
            if isinstance(block, dict) and block.get("type") == "image_url"]


class LocalScriptModel(BaseChatModel):
    """Only local Python runs, while create_agent owns messages and tool execution."""
    script: list[Any]
    calls: list[Any] = Field(default_factory=list)
    bindings: list[Any] = Field(default_factory=list)

    @property
    def _llm_type(self):
        return "local-single-vlm-test"

    def bind_tools(self, tools, **kwargs):
        self.bindings.append({"names": [getattr(item, "name", str(item)) for item in tools],
                              **kwargs})
        return self.bind(**kwargs)

    def _generate(self, messages, stop=None, run_manager=None, **kwargs):
        self.calls.append(copy.deepcopy(messages))
        if not self.script:
            raise AssertionError("Unexpected extra model call")
        step = self.script.pop(0)
        if isinstance(step, Exception):
            raise step
        response = step(messages) if callable(step) else step
        return ChatResult(generations=[ChatGeneration(message=response)])


class CameraClient(FakeBridge):
    def __init__(self, captured=None, error=None):
        super().__init__()
        self.captured = snapshot() if captured is None else captured
        self.error = error
        self.camera_reads = []
        self.records = []

    def capture_camera(self, side="left"):
        self.camera_reads.append(side)
        if self.error:
            raise self.error
        value = copy.deepcopy(self.captured)
        value["side"] = side
        return value

    def record(self, name, value):
        self.records.append((name, copy.deepcopy(value)))
        return value


class FlightCameraClient(CameraClient):
    """Local flight state transitions used to prove recovery waits for disarming."""
    def __init__(self, armed=False):
        super().__init__()
        self.armed = armed
        self.waited_missions = []

    def state(self):
        value = super().state()
        value.update(armed=self.armed, landed_state=2 if self.armed else 1)
        return value

    def request(self, path, payload=None):
        if path in ("/mission", "/land"):
            steps = (payload or {}).get("steps", []) if path == "/mission" else [{"action": "land"}]
            self.armed = not bool(steps and steps[-1]["action"] == "land")
            self.mission = {"id": "local-flight-{}".format(len(self.calls) + 1), "status": "succeeded",
                            "step": len(steps), "steps": copy.deepcopy(steps)}
        return super().request(path, payload)

    def wait_for(self, accepted, on_progress=None):
        self.waited_missions.append(accepted["mission"]["id"])
        return self.state()


class StoreTests(unittest.TestCase):
    def test_stereo_capture_keeps_acquisition_coordinate_epoch_and_source(self):
        client=CameraClient()
        client.supports_stereo_localization=True
        client.capture_stereo=lambda:{'status':'ok','pair_id':'real-owned-pair',
            'left':snapshot(),'body_pose_enu':{'position_m':[1.,2.,3.5]},
            'image_sha256':{'left':'left-hash','right':'right-hash'},
            'external_vision':{'adapter':{'epoch':2},'ekf':{'local_position_reset_counters':{'xy':3,'z':1,'heading':4}}}}
        store=single_vlm.FrameStore(client,CONFIG)
        capture=store.capture(('thread','turn'),'inspect')
        saved=store.get(('thread','turn'),capture['capture_id'])['snapshot']
        self.assertEqual(saved['coordinate_epoch'],{'vio_epoch':2,'ekf_counters':{'xy':3,'z':1,'heading':4}})
        self.assertEqual(saved['location_frame'],'local_enu')
        self.assertEqual(saved['stereo_pair_id'],'real-owned-pair')

    def setUp(self):
        self.client = CameraClient()
        self.now = [100.0]
        self.owner = ("thread-a", "turn-a")
        self.store = single_vlm.FrameStore(self.client, CONFIG, ttl_seconds=20, max_frames=2,
                                          clock=lambda: self.now[0])

    def assert_private(self, value):
        text = encoded(value)
        for secret in (CONFIG["api_key"], JPEG_B64, "image_base64", "data:image/jpeg;base64,"):
            self.assertNotIn(secret, text)

    def test_capture_exposes_metadata_and_keeps_exact_image_internal(self):
        public = self.store.capture(self.owner, "寻找标记")
        self.assertEqual(public["status"], "ok")
        self.assert_private(public)
        frame = self.store.get(self.owner, public["capture_id"])
        self.assertEqual(frame["image_base64"], JPEG_B64)
        self.assertNotIn("image_base64", frame["snapshot"])
        self.assertFalse(frame["delivered"])
        self.assertEqual(frame["snapshot"]["stamp_sim_s"], 42.0)
        self.assertEqual(self.client.camera_reads, ["left"])

    def test_owner_is_current_human_id_and_thread_id(self):
        messages = [HumanMessage("old", id="old-turn"), AIMessage("old reply"),
                    HumanMessage("new", id="new-turn"), capture_request()]
        owner = single_vlm.turn_owner(messages, {"configurable": {"thread_id": "thread-a"}})
        self.assertEqual(owner, ("thread-a", "new-turn"))
        with self.assertRaises((ValueError, RuntimeError)):
            single_vlm.turn_owner([HumanMessage("missing id")],
                                  {"configurable": {"thread_id": "thread-a"}})

    def test_unknown_capture_and_cross_turn_or_cross_thread_access_are_rejected(self):
        capture_id = self.store.capture(self.owner, "寻找标记")["capture_id"]
        for owner, identifier in ((self.owner, "invented-capture"),
                                  (("thread-a", "turn-b"), capture_id),
                                  (("thread-b", "turn-a"), capture_id)):
            with self.subTest(owner=owner), self.assertRaises((ValueError, RuntimeError, KeyError)):
                self.store.get(owner, identifier)

    def test_report_requires_successful_image_delivery(self):
        capture_id = self.store.capture(self.owner, "寻找标记")["capture_id"]
        with self.assertRaises((ValueError, RuntimeError)):
            self.store.get(self.owner, capture_id, require_delivered=True)
        self.store.mark_delivered(self.owner, capture_id, {"model": CONFIG["model"]})
        self.assertTrue(self.store.get(self.owner, capture_id, require_delivered=True)["delivered"])

    def test_expired_and_evicted_frames_are_not_reusable(self):
        first = self.store.capture(self.owner, "first")["capture_id"]
        self.store.capture(self.owner, "second")
        self.store.capture(self.owner, "third")
        with self.assertRaises((ValueError, RuntimeError, KeyError)):
            self.store.get(self.owner, first)
        fresh = self.store.capture(self.owner, "fresh")["capture_id"]
        self.now[0] += 20.01
        with self.assertRaises((ValueError, RuntimeError, KeyError)):
            self.store.get(self.owner, fresh)

    def test_saved_current_turn_report_no_longer_requests_its_expired_image(self):
        capture = self.store.capture(self.owner, "look")
        messages = [HumanMessage("look", id=self.owner[1]),
                    ToolMessage(encoded(capture), name="capture_scene", tool_call_id="capture"),
                    ToolMessage(encoded({"status": "ok", "capture_id": capture["capture_id"]}),
                                name="report_scene", tool_call_id="report")]
        self.now[0] += 20.01
        owner, frame = self.store.frame_for_request(messages, {"configurable": {"thread_id": self.owner[0]}})
        self.assertEqual(owner, self.owner)
        self.assertIsNone(frame)
        with self.assertRaises(RuntimeError):
            self.store.get(self.owner, capture["capture_id"])

    def test_unreported_or_failed_report_does_not_bypass_frame_expiry(self):
        capture = self.store.capture(self.owner, "look")
        base = [HumanMessage("look", id=self.owner[1]),
                ToolMessage(encoded(capture), name="capture_scene", tool_call_id="capture")]
        reports = [[],
                   [ToolMessage(encoded({"status": "error", "capture_id": capture["capture_id"]}),
                                name="report_scene", tool_call_id="failed-report")],
                   [ToolMessage(encoded({"status": "ok", "capture_id": capture["capture_id"]}),
                                name="report_scene", tool_call_id="transport-error", status="error")]]
        self.now[0] += 20.01
        for report in reports:
            with self.subTest(report=report), self.assertRaises(RuntimeError):
                self.store.frame_for_request(base + report, {"configurable": {"thread_id": self.owner[0]}})

    def test_old_report_and_assistant_content_do_not_consume_a_new_capture(self):
        first = self.store.capture(self.owner, "first")
        second = self.store.capture(self.owner, "second")
        messages = [HumanMessage("look twice", id=self.owner[1]),
                    ToolMessage(encoded(first), name="capture_scene", tool_call_id="capture-first"),
                    ToolMessage(encoded({"status": "ok", "capture_id": first["capture_id"]}),
                                name="report_scene", tool_call_id="report-first"),
                    ToolMessage(encoded(second), name="capture_scene", tool_call_id="capture-second"),
                    ToolMessage(encoded({"status": "ok", "capture_id": first["capture_id"]}),
                                name="report_scene", tool_call_id="old-report"),
                    AIMessage(encoded({"status": "ok", "capture_id": second["capture_id"]}), name="report_scene")]
        config = {"configurable": {"thread_id": self.owner[0]}}
        _, frame = self.store.frame_for_request(messages, config)
        self.assertEqual(frame["capture_id"], second["capture_id"])
        self.now[0] += 20.01
        with self.assertRaises(RuntimeError):
            self.store.frame_for_request(messages, config)

    def test_previous_turn_report_cannot_consume_current_turn_image(self):
        capture = self.store.capture(self.owner, "look")
        messages = [HumanMessage("previous", id="previous-turn"),
                    ToolMessage(encoded({"status": "ok", "capture_id": capture["capture_id"]}),
                                name="report_scene", tool_call_id="previous-report"),
                    HumanMessage("look", id=self.owner[1]),
                    ToolMessage(encoded(capture), name="capture_scene", tool_call_id="current-capture")]
        _, frame = self.store.frame_for_request(messages, {"configurable": {"thread_id": self.owner[0]}})
        self.assertEqual(frame["capture_id"], capture["capture_id"])
        self.assertFalse(frame["delivered"])
        with self.assertRaises(RuntimeError):
            self.store.frame_for_request(messages, {"configurable": {"thread_id": "another-thread"}})

    def test_stale_future_and_nonfinite_measurements_fail_before_image_use(self):
        for change in ({"age_seconds": 2.01}, {"age_seconds": -.01},
                       {"measurement_age_seconds": .501}, {"measurement_age_seconds": -.051},
                       {"measurement_age_seconds": float("nan")}, {"stamp_sim_s": float("inf")}):
            with self.subTest(change=change):
                captured = snapshot()
                captured.update(change)
                store = single_vlm.FrameStore(CameraClient(captured), CONFIG)
                with self.assertRaises((ValueError, RuntimeError)):
                    store.capture(self.owner, "look")
                self.assertEqual(store.frames, {})

    def test_bad_image_encoding_mime_and_oversized_payload_fail(self):
        for change in ({"image_base64": "not base64!"}, {"mime_type": "image/png"},
                       {"image_base64": "bm90IGEganBlZw=="}, {"image_base64": "A" * 3000001}):
            with self.subTest(change=list(change)):
                captured = snapshot()
                captured.update(change)
                store = single_vlm.FrameStore(CameraClient(captured), CONFIG)
                with self.assertRaises((ValueError, RuntimeError)):
                    store.capture(self.owner, "look")
                self.assertEqual(store.frames, {})

    def test_capture_error_keeps_frame_store_empty(self):
        failure = RuntimeError("camera rejected " + CONFIG["api_key"] + " data:image/jpeg;base64," + JPEG_B64)
        store = single_vlm.FrameStore(CameraClient(error=failure), CONFIG)
        with self.assertRaises(RuntimeError):
            store.capture(self.owner, "look")
        self.assertEqual(store.frames, {})

    def test_visual_target_rejects_boolean_and_string_numbers_before_coercion(self):
        for change in ({"confidence": True}, {"confidence": "0.8"},
                       {"bbox": [.1, .2, True, .8]}, {"bbox": ["0.1", .2, .3, .4]}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                value = target()
                value.update(change)
                single_vlm.VisualTarget(**value)


class GraphTests(unittest.TestCase):
    def build(self, script, client=None, store=None, flight_tools=None):
        client = client or CameraClient()
        model = LocalScriptModel(script=list(script))
        store = store or single_vlm.FrameStore(client, CONFIG)
        graph = single_vlm.build_single_vlm_agent(client, CONFIG, flight_tools or [],
                                                 "Only observe true camera evidence; do not fly.",
                                                 model=model, store=store)
        return graph, model, store, client

    def invoke(self, graph, prompt="寻找标记", turn="turn-a", thread="thread-a"):
        config = {"configurable": {"thread_id": thread}, "recursion_limit": 15, "max_concurrency": 1}
        result = graph.invoke({"messages": [HumanMessage(prompt, id=turn)]}, config)
        return result, config

    def assert_private(self, value):
        for secret in (CONFIG["api_key"], JPEG_B64, "image_base64", "data:image/jpeg;base64,"):
            self.assertNotIn(secret, encoded(value))

    def test_same_model_runs_text_capture_image_report_and_final_without_checkpoint_image(self):
        graph, model, store, client = self.build([capture_request, report_request, AIMessage("观察完成")])
        result, config = self.invoke(graph)
        self.assertEqual(len(model.calls), 3)
        self.assertEqual(image_urls(model.calls[0]), [])
        self.assertEqual(image_urls(model.calls[1]), ["data:image/jpeg;base64," + JPEG_B64])
        self.assertEqual(image_urls(model.calls[2]), [])
        capture = tool_payload(result["messages"], "capture_scene")
        report = tool_payload(result["messages"], "report_scene")
        self.assertEqual(report["status"], "ok")
        self.assertEqual(report["targets"][0]["label"], "红色方块")
        self.assertEqual(report["captured_at_sim_s"], 42.0)
        self.assertTrue(store.get(("thread-a", "turn-a"), capture["capture_id"], require_delivered=True))
        self.assertEqual(client.camera_reads, ["left"])
        self.assertEqual(client.calls, [])
        self.assert_private(result)
        self.assert_private(graph.get_state(config).values)
        self.assert_private(client.records)
        self.assert_private(store.calls(("thread-a", "turn-a")))
        self.assertEqual(result["messages"][-1].content, "观察完成")

    def test_saved_report_survives_return_flight_longer_than_image_ttl(self):
        client, now = CameraClient(), [100.]
        store = single_vlm.FrameStore(client, CONFIG, clock=lambda: now[0])

        @tool
        def return_home() -> dict:
            """Finish a locally simulated long return flight."""
            now[0] += 181.
            return {"status": "ok", "landed_state": 1, "armed": False}

        graph, model, _, _ = self.build([capture_request, report_request,
            tool_call("return_home", {}, "long-return"), AIMessage("报告已保存，已返航降落")],
            client=client, store=store, flight_tools=[return_home])
        result, config = self.invoke(graph)
        self.assertEqual(result["messages"][-1].content, "报告已保存，已返航降落")
        self.assertEqual(tool_payload(result["messages"], "report_scene")["status"], "ok")
        self.assertEqual(len(model.calls), 4)
        self.assertEqual([len(image_urls(messages)) for messages in model.calls], [0, 1, 0, 0])
        self.assert_private(graph.get_state(config).values)
        capture_id = tool_payload(result["messages"], "capture_scene")["capture_id"]
        with self.assertRaises(RuntimeError):
            store.get(("thread-a", "turn-a"), capture_id)

    def test_unreported_image_expiring_during_return_still_fails_preparation(self):
        client, now = CameraClient(), [100.]
        store = single_vlm.FrameStore(client, CONFIG, clock=lambda: now[0])

        @tool
        def return_home() -> dict:
            """Finish a locally simulated long return flight."""
            now[0] += 181.
            return {"status": "ok"}

        graph, model, _, _ = self.build([capture_request,
            tool_call("return_home", {}, "return-without-report"), AIMessage("must not run")],
            client=client, store=store, flight_tools=[return_home])
        with self.assertRaisesRegex(RuntimeError, "已过期"):
            self.invoke(graph)
        self.assertEqual(len(model.calls), 2)
        failures = [value for name, value in client.records if name == "inspect_scene" and value.get("status") == "error"]
        self.assertEqual(failures[-1]["phase"], "image_preparation")

    def test_sdk_request_keeps_system_and_complete_tool_reply_before_image(self):
        graph, model, _, _ = self.build([capture_request, report_request, AIMessage("done")])
        self.invoke(graph)
        request = model.calls[1]
        self.assertEqual(request[0].type, "system")
        tool_index = next(i for i, message in enumerate(request) if isinstance(message, ToolMessage))
        image_index = next(i for i, message in enumerate(request) if image_urls([message]))
        self.assertLess(tool_index, image_index)
        self.assertEqual(request[tool_index].tool_call_id, "capture-call")
        self.assertIsInstance(request[image_index], HumanMessage)
        self.assertTrue(all(binding.get("parallel_tool_calls") is False for binding in model.bindings))

    def test_new_user_turn_does_not_receive_previous_image(self):
        graph, model, _, _ = self.build([capture_request, report_request, AIMessage("first done"),
                                         AIMessage("second answer")])
        self.invoke(graph)
        result, config = self.invoke(graph, "当前状态", turn="turn-b")
        self.assertEqual(len(model.calls), 4)
        self.assertEqual(image_urls(model.calls[-1]), [])
        self.assert_private(result)
        self.assert_private(graph.get_state(config).values)

    def test_forged_previous_turn_capture_cannot_produce_new_report(self):
        old = {}
        def forged_report(messages):
            return tool_call("report_scene", {"capture_id": old["capture_id"], **observation()}, "forged-call")
        graph, model, _, _ = self.build([capture_request, report_request, AIMessage("first done"),
                                         forged_report, AIMessage("claim success")])
        first, _ = self.invoke(graph)
        old.update(tool_payload(first['messages'], 'capture_scene'))
        result, _ = self.invoke(graph, "重新观察", turn="turn-b")
        reports = [json.loads(message.content) for message in result["messages"]
                   if isinstance(message, ToolMessage) and message.name == "report_scene"]
        self.assertEqual(reports[-1]["status"], "error")
        self.assertEqual(image_urls(model.calls[-2]), [])

    def test_report_with_unknown_capture_id_is_error_without_camera_or_image(self):
        graph, model, _, client = self.build([
            tool_call("report_scene", {"capture_id": "invented-id", **observation()}, "bad-report"),
            AIMessage("claim success")])
        result, _ = self.invoke(graph)
        self.assertEqual(tool_payload(result["messages"], "report_scene")["status"], "error")
        self.assertEqual(client.camera_reads, [])
        self.assertTrue(all(not image_urls(messages) for messages in model.calls))

    def test_preseeded_capture_cannot_report_without_successful_image_model_call(self):
        client = CameraClient()
        store = single_vlm.FrameStore(client, CONFIG)
        capture_id = store.capture(("thread-a", "turn-a"), "preseeded")["capture_id"]
        graph, model, _, _ = self.build([
            tool_call("report_scene", {"capture_id": capture_id, **observation()}, "not-seen"),
            AIMessage("claim success")], client=client, store=store)
        result, _ = self.invoke(graph)
        self.assertEqual(tool_payload(result["messages"], "report_scene")["status"], "error")
        self.assertFalse(store.get(("thread-a", "turn-a"), capture_id)["delivered"])
        self.assertTrue(all(not image_urls(messages) for messages in model.calls))

    def test_another_thread_cannot_report_or_receive_same_turn_id_image(self):
        client = CameraClient()
        store = single_vlm.FrameStore(client, CONFIG)
        capture_id = store.capture(("thread-a", "shared-turn-id"), "other thread")["capture_id"]
        store.mark_delivered(("thread-a", "shared-turn-id"), capture_id, {})
        graph, model, _, _ = self.build([
            tool_call("report_scene", {"capture_id": capture_id, **observation()}, "foreign-id"),
            AIMessage("claim success")], client=client, store=store)
        result, _ = self.invoke(graph, turn="shared-turn-id", thread="thread-b")
        self.assertEqual(tool_payload(result["messages"], "report_scene")["status"], "error")
        self.assertTrue(all(not image_urls(messages) for messages in model.calls))

    def test_failed_image_model_call_does_not_mark_frame_delivered(self):
        failure = RuntimeError("provider failed " + CONFIG["api_key"] + " data:image/jpeg;base64," + JPEG_B64)
        graph, model, store, client = self.build([capture_request, failure])
        config = {"configurable": {"thread_id": "thread-a"}, "recursion_limit": 15}
        try:
            graph.invoke({"messages": [HumanMessage("look", id="turn-a")]}, config)
        except Exception as exc:
            self.assert_private(str(exc))
        else:
            self.fail("Model failure should terminate the graph invocation")
        state = graph.get_state(config).values
        capture_id = tool_payload(state["messages"], "capture_scene")["capture_id"]
        self.assertFalse(store.get(("thread-a", "turn-a"), capture_id)["delivered"])
        self.assert_private(state)
        self.assert_private(client.records)
        traces = store.calls(("thread-a", "turn-a"))
        self.assertEqual(traces[-1]["status"], "error")
        self.assertEqual(traces[-1]["input_modalities"], ["text", "image"])
        self.assertEqual(traces[-1]["capture_ids"], [capture_id])
        self.assert_private(traces)
        failure = [result for name, result in client.records if name == "inspect_scene"][-1]
        self.assertEqual(failure["capture_id"], capture_id)
        self.assertEqual(failure["captured_at_sim_s"], 42.0)
        self.assertEqual(failure["camera"]["image_sha256"],
                         tool_payload(state["messages"], "capture_scene")["camera"]["image_sha256"])
        self.assertEqual(failure["vlm_calls"][-1], traces[-1])
        self.assertEqual(len(model.calls), 2)

    def test_capture_tool_error_redacts_credentials_and_image_without_model_image_input(self):
        failure = RuntimeError("camera failed " + CONFIG["api_key"] + " data:image/jpeg;base64," + JPEG_B64)
        graph, model, _, client = self.build([capture_request, AIMessage("claim success")],
                                            client=CameraClient(error=failure))
        result, config = self.invoke(graph)
        self.assertEqual(tool_payload(result["messages"], "capture_scene")["status"], "error")
        self.assertTrue(all(not image_urls(messages) for messages in model.calls))
        self.assert_private(result)
        self.assert_private(graph.get_state(config).values)
        self.assert_private(client.records)

    def test_report_computes_projection_from_captured_pose_and_never_trusts_claimed_depth(self):
        def projected_report(messages):
            capture_id = tool_payload(messages, "capture_scene")["capture_id"]
            return tool_call("report_scene", {"capture_id": capture_id,
                             **observation([target("flat_ground_marking")])}, "projection-report")
        graph, _, _, _ = self.build([capture_request, projected_report, AIMessage("done")])
        result, _ = self.invoke(graph)
        location = tool_payload(result["messages"], "report_scene")["targets"][0]["approximate_location"]
        self.assertEqual(location["method"], "ground_plane_projection")
        self.assertEqual(location["frame"], "gazebo_world")
        self.assertEqual([location[key] for key in ("x_m", "y_m", "z_m")], [1., 2., .003])

    def test_invalid_bbox_is_error_instead_of_observation(self):
        def invalid_report(messages):
            capture_id = tool_payload(messages, "capture_scene")["capture_id"]
            return tool_call("report_scene", {"capture_id": capture_id,
                             **observation([target(bbox=[.6, .4, .4, .6])])}, "invalid-report")
        graph, _, _, _ = self.build([capture_request, invalid_report, AIMessage("claim success")])
        result, _ = self.invoke(graph)
        self.assertEqual(tool_payload(result["messages"], "report_scene")["status"], "error")


class WebGuardTests(unittest.TestCase):
    def application(self, script, flight_tools_factory=None, bridge=None):
        bridge = bridge or CameraClient()
        model = LocalScriptModel(script=list(script))
        def factory(client, config):
            flight_tools = flight_tools_factory(client) if flight_tools_factory else []
            return single_vlm.build_single_vlm_agent(client, config, flight_tools,
                                                     "Observe only; no flight authorization.", model=model)
        app = WebApplication(bridge, CONFIG, factory, vision_config=CONFIG, agent_architecture="single_vlm")
        return app, bridge, model

    @staticmethod
    def basic_flight_tools(client):
        @tool
        def execute_mission() -> dict:
            """Take off for this explicitly requested flight phase."""
            try:
                return client.execute([{"action": "takeoff", "altitude": 2}], "owned flight")
            except Exception as exc:
                return {"status": "error", "error": str(exc)}

        @tool
        def land_drone() -> dict:
            """Land the aircraft and wait for disarming."""
            return client.wait_for(client.request("/land", {}))
        return [execute_mission, land_drone]

    def test_real_single_vlm_graph_observe_mode_reports_evidence_without_flying(self):
        app, bridge, model = self.application([capture_request, report_request,
                                              AIMessage("ENU position and measured depth are exact")])
        command = completed(app, app.submit("寻找红色标记", mode="observe")["id"])
        self.assertEqual(command["status"], "succeeded")
        self.assertEqual(command["perception"]["status"], "ok")
        self.assertIn("未测量距离", command["reply"])
        self.assertNotIn("measured depth are exact", command["reply"])
        self.assertEqual(bridge.calls, [])
        self.assertEqual(bridge.camera_reads, ["left"])
        self.assertEqual(len(model.calls), 3)
        GraphTests().assert_private({"command": command, "state": app.state()})

    def test_observe_still_blocks_agent_mission_stop_and_land_while_operator_controls_work(self):
        for path in ("/mission", "/stop", "/land"):
            with self.subTest(path=path):
                def tools_factory(client):
                    @tool
                    def unsafe_action() -> dict:
                        """Try an agent flight action."""
                        try:
                            return client.request(path, {})
                        except Exception as exc:
                            return {"status": "error", "error": str(exc)}
                    return [unsafe_action]
                app, bridge, _ = self.application([tool_call("unsafe_action", {}, "unsafe-call"),
                                                   AIMessage("flight succeeded")], tools_factory)
                command = completed(app, app.submit("只观察当前图像", mode="observe")["id"])
                self.assertEqual(command["status"], "failed")
                self.assertEqual(bridge.calls, [])
                self.assertNotIn("flight succeeded", command["reply"])
                app.interrupt("/stop")
                app.interrupt("/land")
                self.assertEqual(bridge.calls, ["/stop", "/land"])

    def test_provider_error_is_reported_as_failure_and_redacted_in_public_web_state(self):
        failure = RuntimeError("provider failed " + CONFIG["api_key"] + " data:image/jpeg;base64," + JPEG_B64)
        app, bridge, model = self.application([capture_request, failure])
        command = completed(app, app.submit("寻找标记", mode="observe")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertIn("provider failed", command["error"])
        self.assertEqual(bridge.calls, [])
        self.assertEqual(len(model.calls), 2)
        trace = command["model_calls"][-1]
        self.assertEqual(trace["status"], "error")
        self.assertEqual(trace["input_modalities"], ["text", "image"])
        self.assertEqual(trace["capture_ids"], [command["perception"]["capture_id"]])
        self.assertTrue(command["perception"]["camera"]["image_sha256"])
        self.assertEqual(command["perception"]["captured_at_sim_s"], 42.0)
        GraphTests().assert_private({"command": command, "state": app.state()})

    def test_schema_failure_blocks_new_mission_but_preserves_requested_safety_land(self):
        def invalid_report(messages):
            capture_id = tool_payload(messages, "capture_scene")["capture_id"]
            malformed = target()
            malformed["confidence"] = True
            return tool_call("report_scene", {"capture_id": capture_id,
                             **observation([malformed])}, "schema-error")

        def flight_tools(client):
            @tool
            def execute_mission() -> dict:
                """Submit a new flight mission."""
                try:
                    return client.execute([{"action": "takeoff", "altitude": 2}], "new mission")
                except Exception as exc:
                    return {"status": "error", "error": str(exc)}

            @tool
            def land_drone() -> dict:
                """Finish the previously requested safety landing."""
                return client.wait_for(client.request("/land", {}))
            return [execute_mission, land_drone]

        app, bridge, _ = self.application([capture_request, invalid_report,
                                          tool_call("execute_mission", {}, "blocked-mission"),
                                          tool_call("land_drone", {}, "safe-land"),
                                          AIMessage("all succeeded")], flight_tools)
        command = completed(app, app.submit("观察当前场景，然后降落", mode="task")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual(command["perception"]["status"], "error")
        self.assertEqual(bridge.calls, ["/land"])
        self.assertNotIn("all succeeded", command["reply"])
        GraphTests().assert_private({"command": command, "state": app.state()})

    def test_success_after_failed_report_cannot_unlock_new_mission(self):
        def invalid_report(messages):
            capture_id = tool_payload(messages, "capture_scene")["capture_id"]
            return tool_call("report_scene", {"capture_id": capture_id,
                             **observation([target(bbox=[.6, .4, .4, .6])])}, "first-error")

        def later_report(messages):
            captures = [json.loads(message.content) for message in messages
                        if isinstance(message, ToolMessage) and message.name == "capture_scene"]
            capture_id = next(result["capture_id"] for result in captures if result.get("status") == "ok")
            return tool_call("report_scene", {"capture_id": capture_id, **observation()}, "discarded-report")

        app, bridge, _ = self.application([
            capture_request, invalid_report,
            tool_call("capture_scene", {"query": "retry failed observation", "side": "left"}, "retry-capture"),
            later_report,
            tool_call("execute_mission", {}, "still-blocked"),
            tool_call("land_drone", {}, "explicit-safe-land"), AIMessage("all succeeded")],
            self.basic_flight_tools, bridge=FlightCameraClient())
        command = completed(app, app.submit("观察场景，然后降落", mode="task")["id"])
        self.assertEqual(command["perception"]["status"], "error")
        self.assertIn("不再采纳", command["perception"]["error"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual(bridge.calls, ["/land"])
        self.assertEqual(bridge.camera_reads, ["left"])
        self.assertIsNone(command["safety_landing"])
        self.assertFalse(bridge.state()["armed"])
        self.assertNotIn("all succeeded", command["reply"])
        self.assertNotIn("vision_failure", command)

    def test_model_failure_after_owned_takeoff_runs_independent_land_and_waits_for_disarming(self):
        app, bridge, model = self.application([
            tool_call("execute_mission", {}, "takeoff-first"), capture_request,
            RuntimeError("provider disconnected " + CONFIG["api_key"])],
            self.basic_flight_tools, bridge=FlightCameraClient())
        command = completed(app, app.submit("起飞到2米，观察场景，然后降落", mode="task")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual(bridge.calls, ["/mission", "/land"])
        self.assertFalse(bridge.state()["armed"])
        self.assertEqual(bridge.state()["landed_state"], 1)
        self.assertEqual(len(bridge.waited_missions), 2)
        self.assertEqual(command["safety_landing"]["status"], "ok")
        self.assertEqual(command["safety_landing"]["mission"]["status"], "succeeded")
        self.assertIn("安全降落并上锁", command["reply"])
        self.assertEqual(len(model.calls), 3)
        GraphTests().assert_private({"command": command, "state": app.state()})

    def test_failed_observation_does_not_land_an_already_airborne_drone(self):
        app, bridge, _ = self.application([capture_request, RuntimeError("provider failed")],
                                         bridge=FlightCameraClient(armed=True))
        command = completed(app, app.submit("只观察画面", mode="observe")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertTrue(bridge.state()["armed"])
        self.assertEqual(bridge.calls, [])
        self.assertEqual(bridge.waited_missions, [])
        self.assertIsNone(command["safety_landing"])

    def test_operator_interrupt_after_owned_takeoff_does_not_trigger_automatic_landing(self):
        app_holder = {}
        def interrupt_then_fail(messages):
            app_holder["app"].interrupt("/stop")
            raise RuntimeError("provider failed after operator stopped")
        app, bridge, _ = self.application([
            tool_call("execute_mission", {}, "takeoff-first"), interrupt_then_fail],
            self.basic_flight_tools, bridge=FlightCameraClient())
        app_holder["app"] = app
        command = completed(app, app.submit("起飞并观察", mode="task")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual(bridge.calls, ["/mission", "/stop"])
        self.assertTrue(bridge.state()["armed"])
        self.assertIsNone(command["safety_landing"])
        self.assertIn("已中断", command["reply"])

    def test_failed_task_without_its_own_flight_does_not_take_over_existing_airborne_drone(self):
        app, bridge, _ = self.application([capture_request, RuntimeError("provider failed")],
                                         self.basic_flight_tools, bridge=FlightCameraClient(armed=True))
        command = completed(app, app.submit("分析当前画面", mode="task")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertTrue(bridge.state()["armed"])
        self.assertEqual(bridge.calls, [])
        self.assertEqual(bridge.waited_missions, [])
        self.assertIsNone(command["safety_landing"])

    def test_flight_schema_error_is_json_and_blocks_retry_while_allowing_land(self):
        from agent import MissionStep

        def typed_tools(client):
            @tool
            def execute_mission(steps: list[MissionStep], description: str) -> dict:
                """Execute the explicitly requested flight phase."""
                try:
                    return client.execute([step.model_dump(exclude_none=True) for step in steps], description)
                except Exception as exc:
                    return {"status": "error", "error": str(exc)}

            @tool
            def land_drone() -> dict:
                """Land immediately and wait for disarming."""
                return client.wait_for(client.request("/land", {}))
            return [execute_mission, land_drone]

        description = "this description belongs to tool arguments, never model trace"
        app, bridge, _ = self.application([
            tool_call("execute_mission", {"steps": [{"action": "land"}]}, "missing-description"),
            tool_call("execute_mission", {"steps": [{"action": "takeoff", "altitude": 2}],
                                          "description": description}, "blocked-retry"),
            tool_call("land_drone", {}, "allowed-land"), AIMessage("all succeeded")],
            typed_tools, bridge=FlightCameraClient())
        command = completed(app, app.submit("起飞后降落", mode="task")["id"])
        messages = app.agent.get_state({"configurable": {"thread_id": app.agent_thread_id}}).values["messages"]
        failures = [json.loads(message.content) for message in messages
                    if isinstance(message, ToolMessage) and message.name == "execute_mission"]
        self.assertEqual([result["status"] for result in failures], ["error", "error"])
        self.assertIn("description", failures[0]["error"])
        self.assertIn("禁止自动重试", failures[1]["error"])
        self.assertEqual(bridge.calls, ["/land"])
        self.assertEqual(command["status"], "failed")
        self.assertNotIn("无法解析", command["error"])
        self.assertEqual(command["tool_errors"][0]["tool"], "execute_mission")
        self.assertNotIn(description, encoded(command["model_calls"]))
        self.assertEqual(command["model_calls"][0]["called_tools"][0],
                         {"name": "execute_mission", "argument_fields": ["steps"], "mission_actions": ["land"]})

    def test_unknown_tool_name_is_json_and_public_error_identifies_actual_tool(self):
        unknown = "invented_drone_tool"
        app, bridge, _ = self.application([
            tool_call(unknown, {}, "unknown-tool"), tool_call("land_drone", {}, "allowed-land"),
            AIMessage("all succeeded")], self.basic_flight_tools, bridge=FlightCameraClient())
        command = completed(app, app.submit("执行任务然后降落", mode="task")["id"])
        state = app.agent.get_state({"configurable": {"thread_id": app.agent_thread_id}}).values
        failure = tool_payload(state["messages"], unknown)
        self.assertEqual(failure["status"], "error")
        self.assertIn(unknown, failure["error"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual(command["tool_errors"][0]["tool"], unknown)
        self.assertNotIn("无法解析", command["error"])
        self.assertEqual(bridge.calls, ["/land"])
        self.assertNotIn("all succeeded", command["reply"])
        GraphTests().assert_private({"command": command, "state": app.state()})


if __name__ == "__main__":
    unittest.main()
