#!/usr/bin/env python3
"""Offline fault injection through the real Agent graph and web execution guard.

Run with the Agent's Python 3.11 environment. This deliberately reuses the local
model/camera fixtures used by the existing graph regression tests. No provider,
ROS, Gazebo, or HTTP server is contacted. A successful case means the expected
failure/recovery contract passed, not that its simulated flight task succeeded.
"""
import argparse
import copy
import importlib.metadata
import json
import os
import sys
import time
import uuid
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from unittest import mock

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))
sys.path.insert(0, str(PROJECT_DIR / "tests"))

try:
    import httpx
    from openai import APIConnectionError, APITimeoutError
    import agent
    import single_vlm
    from test_single_vlm import (AIMessage, FlightCameraClient, LocalScriptModel,
                                ToolMessage, capture_request, encoded, image_urls,
                                report_request, tool_call)
    from test_vision import JPEG_B64
    from test_web_server import completed
    from web_server import WebApplication
    from langchain_core.tools import tool
except ImportError as exc:
    raise RuntimeError("Use the Agent Python 3.11 environment with LangChain installed") from exc


SOURCE = "offline_fault_injection_real_langchain_graph"
MODEL_CONFIG = {"model": "offline-scripted-vlm", "base_url": "https://offline-fault.invalid/v1",
                "api_key": "synthetic-offline-key-never-sent"}


def flight(steps, call_id, description="offline authorized phase"):
    return tool_call("execute_mission", {"steps": steps, "description": description}, call_id)


TAKEOFF = [{"action": "takeoff", "altitude": 3.5}]
GOTO = [{"action": "goto", "x": 13., "y": 0., "altitude": 3.5}]
RETURN_LAND = [{"action": "return_home"}, {"action": "land"}]


class EvidenceBridge(FlightCameraClient):
    """Only bridge I/O is fake; production tools/audit/recovery own all decisions."""
    def __init__(self, now, armed=False, camera_error=None, fail_mission=None, failure_status="failed"):
        super().__init__(armed=armed)
        self.now = now
        self.error = camera_error
        self.fail_mission = fail_mission
        self.failure_status = failure_status
        self.requests = []
        self.waits = []
        self.mission_number = 0

    def request(self, path, payload=None):
        self.requests.append({"path": path, "payload": copy.deepcopy(payload)})
        result = super().request(path, payload)
        if path == "/mission":
            self.mission_number += 1
            if self.mission_number == self.fail_mission:
                self.mission.update(status=self.failure_status, error="offline injected mission failure: blocked passage")
                result = {"mission": copy.deepcopy(self.mission)}
        return result

    def wait_for(self, accepted, on_progress=None):
        self.waits.append(copy.deepcopy(accepted["mission"]))
        request = self.requests[-1]
        if (request["payload"] or {}).get("description") == "advance offline clock":
            self.now[0] += 181.
        return super().wait_for(accepted, on_progress)


def connection_error():
    return APIConnectionError(message="offline injected API connection failure",
                              request=httpx.Request("POST", MODEL_CONFIG["base_url"] + "/chat/completions"))


def timeout_error():
    return APITimeoutError(request=httpx.Request("POST", MODEL_CONFIG["base_url"] + "/chat/completions"))


CASES = {
    "connection_before_flight": {
        "injection": "APIConnectionError on first text request, before any flight tool",
        "expected": "Fail command; no bridge motion or landing; retain one error model audit",
        "expected_paths": [], "model_calls": 1,
    },
    "timeout_after_owned_takeoff": {
        "injection": "APITimeoutError after this command's authorized takeoff completes",
        "expected": "Fail command; independently invoke /land and wait for disarming; no new movement",
        "expected_paths": ["/mission", "/land"], "model_calls": 2,
    },
    "connection_on_image_after_owned_takeoff": {
        "injection": "APIConnectionError on request containing the captured JPEG",
        "expected": "Frame remains undelivered; no successful observation; independent landing completes",
        "expected_paths": ["/mission", "/land"], "model_calls": 3,
    },
    "camera_delivery_failure_blocks_movement": {
        "injection": "Camera transport OSError, followed by a model attempt to move",
        "expected": "Reject capture and subsequent movement; no image enters a model request; land owned flight",
        "expected_paths": ["/mission", "/land"], "model_calls": 4,
    },
    "unreported_frame_expired": {
        "injection": "Advance local frame clock by 181 seconds after delivery without a saved report",
        "expected": "Reject next model preparation before calling model; preserve TTL; independently land",
        "expected_paths": ["/mission", "/mission", "/land"], "model_calls": 3,
    },
    "saved_report_survives_frame_expiry": {
        "injection": "Advance local frame clock by 181 seconds during return after report_scene succeeded",
        "expected": "Saved report and final text succeed; expired image is not attached again; no recovery landing",
        "expected_paths": ["/mission", "/mission"], "model_calls": 5, "status": "succeeded",
    },
    "invalid_tool_parameters_block_retry": {
        "injection": "Extra altitude field on return_home, followed by a valid model-requested goto retry",
        "expected": "Reject complete invalid submission; latch flight failure; block retry; independently land",
        "expected_paths": ["/mission", "/land"], "model_calls": 4,
    },
    "failed_mission_blocks_retry": {
        "injection": "Second fake bridge mission reports failed; model attempts to resubmit movement",
        "expected": "Keep initial mission failure; reject automatic retry; independently land owned flight",
        "expected_paths": ["/mission", "/mission", "/land"], "model_calls": 4,
    },
    "unowned_airborne_observation_failure": {
        "injection": "Image APIConnectionError while aircraft was already airborne before observe-only command",
        "expected": "Fail observation; issue no flight or landing; preserve ownership of existing flight",
        "expected_paths": [], "model_calls": 2, "armed": True,
    },
    "operator_interrupt_preserves_authority": {
        "injection": "Operator /stop after owned takeoff, then injected model timeout",
        "expected": "Keep command interrupted; issue no automatic landing or later movement",
        "expected_paths": ["/mission", "/stop"], "model_calls": 2, "armed": True,
    },
    "final_reply_connection_after_completed_flight": {
        "injection": "Text APIConnectionError after a full submitted sortie completed and disarmed",
        "expected": "Keep command failed and API audit; state flight completed and final reply failed; no duplicate landing",
        "expected_paths": ["/mission"], "model_calls": 2,
    },
    "final_reply_error_after_saved_observation": {
        "injection": "Text APIConnectionError after saved observation and waited return/landing",
        "expected": "Preserve saved report; explicitly distinguish completed flight from failed final model response",
        "expected_paths": ["/mission", "/mission"], "model_calls": 5,
    },
    "final_reply_error_after_unconfirmed_acceptance": {
        "injection": "Text APIConnectionError after submission without executing a wait for its result",
        "expected": "Accepted request alone must not claim flight execution completed",
        "expected_paths": ["/mission"], "model_calls": 2,
    },
    "final_reply_error_after_failed_flight": {
        "injection": "Text APIConnectionError after a submitted full sortie reports failed",
        "expected": "Preserve authoritative flight failure; do not attribute all failure to final response",
        "expected_paths": ["/mission"], "model_calls": 2,
    },
    "final_reply_error_after_cancelled_flight": {
        "injection": "Text APIConnectionError after a submitted full sortie reports cancelled",
        "expected": "Cancelled flight must not be called completed despite a later model exception",
        "expected_paths": ["/mission"], "model_calls": 2,
    },
}


def _script_for(case_id, holder):
    takeoff = flight(TAKEOFF, "owned-takeoff")
    move = flight(GOTO, "forbidden-moving-retry")
    false_success = AIMessage("offline false success claim: all flight phases succeeded")
    if case_id == "connection_before_flight":
        return [connection_error()]
    if case_id == "timeout_after_owned_takeoff":
        return [takeoff, timeout_error()]
    if case_id == "connection_on_image_after_owned_takeoff":
        return [takeoff, capture_request, connection_error()]
    if case_id == "camera_delivery_failure_blocks_movement":
        return [takeoff, capture_request, move, false_success]
    if case_id == "unreported_frame_expired":
        return [takeoff, capture_request,
                flight([{"action": "hold", "duration": duration} for duration in (60., 60., 60., 1.)],
                       "wait-without-report", "advance offline clock"),
                AIMessage("must not reach model after frame expiry")]
    if case_id == "saved_report_survives_frame_expiry":
        return [takeoff, capture_request, report_request,
                flight(RETURN_LAND, "return-after-report", "advance offline clock"),
                AIMessage("offline saved report retained; return and landing completed")]
    if case_id == "invalid_tool_parameters_block_retry":
        return [takeoff, flight([{"action": "return_home", "altitude": 3.5}], "invalid-return"), move,
                false_success]
    if case_id == "failed_mission_blocks_retry":
        return [takeoff, flight(GOTO, "failed-moving-phase"), move, false_success]
    if case_id == "unowned_airborne_observation_failure":
        return [capture_request, connection_error()]
    if case_id == "operator_interrupt_preserves_authority":
        def interrupt_then_fail(messages):
            holder["app"].interrupt("/stop")
            raise timeout_error()
        return [takeoff, interrupt_then_fail]
    if case_id in {"final_reply_connection_after_completed_flight", "final_reply_error_after_failed_flight",
                   "final_reply_error_after_cancelled_flight"}:
        return [flight(TAKEOFF + GOTO + [{"action": "hold", "duration": 2.}] + RETURN_LAND,
                       "completed-full-sortie"), connection_error()]
    if case_id == "final_reply_error_after_saved_observation":
        return [takeoff, capture_request, report_request, flight(RETURN_LAND, "waited-return-and-land"), connection_error()]
    if case_id == "final_reply_error_after_unconfirmed_acceptance":
        return [tool_call("submit_without_wait", {}, "unconfirmed-submission"), connection_error()]
    raise ValueError("Unknown reliability case: " + case_id)


def _network_guard():
    stack = ExitStack()
    def forbidden(*args, **kwargs):
        raise AssertionError("Offline reliability test attempted network transport")
    stack.enter_context(mock.patch("socket.create_connection", side_effect=forbidden))
    stack.enter_context(mock.patch("httpx.Client.send", side_effect=forbidden))
    stack.enter_context(mock.patch("httpx.AsyncClient.send", side_effect=forbidden))
    # A manager's capture-evidence setting must not make this offline run write
    # synthetic images into any actual-flight evidence directory.
    stack.enter_context(mock.patch.dict(os.environ, {"UAV_CAPTURE_EVIDENCE_DIR": ""}))
    return stack


def run_case(case_id):
    spec = CASES[case_id]
    started = time.monotonic()
    now, holder = [100.], {}
    bridge = EvidenceBridge(now, armed=case_id == "unowned_airborne_observation_failure",
                            camera_error=OSError("offline injected camera delivery failure")
                            if case_id == "camera_delivery_failure_blocks_movement" else None,
                            fail_mission=2 if case_id == "failed_mission_blocks_retry" else 1
                            if case_id in {"final_reply_error_after_failed_flight", "final_reply_error_after_cancelled_flight"}
                            else None,
                            failure_status="cancelled" if case_id == "final_reply_error_after_cancelled_flight" else "failed")
    model = LocalScriptModel(script=_script_for(case_id, holder))
    def factory(client, config):
        store = single_vlm.FrameStore(client, config, clock=lambda: now[0])
        holder["store"] = store
        flight_tools = agent.flight_tools(client)
        if case_id == "final_reply_error_after_unconfirmed_acceptance":
            @tool
            def submit_without_wait() -> dict:
                """Offline negative control: submit flight but do not poll or await its outcome."""
                return client.request("/mission", {"steps": TAKEOFF + RETURN_LAND,
                                                   "description": "unconfirmed accepted submission"})
            flight_tools.append(submit_without_wait)
        return single_vlm.build_single_vlm_agent(client, config, flight_tools,
            "Offline test: obey this user's authorized flight phases and actual tool results.", model=model, store=store)
    mode = "observe" if case_id == "unowned_airborne_observation_failure" else "task"
    minimum = 1 if case_id == "saved_report_survives_frame_expiry" else 0
    with _network_guard():
        app = WebApplication(bridge, MODEL_CONFIG, factory, vision_config=MODEL_CONFIG,
                             agent_architecture="single_vlm")
        holder["app"] = app
        initial_state = bridge.state()
        accepted = app.submit("离线故障测试：按此指令起飞、观察、等待、返航和降落", mode=mode,
                              min_observations=minimum)
        command = completed(app, accepted["id"])
        checkpoint = app.agent.get_state({"configurable": {"thread_id": app.agent_thread_id}}).values
        final_state = bridge.state()
    messages = checkpoint.get("messages", [])
    tools = []
    for message in messages:
        if not isinstance(message, ToolMessage):
            continue
        try:
            payload = json.loads(message.content)
        except (TypeError, ValueError):
            payload = {"error": str(message.content)[:500]}
        tools.append({"name": message.name, "tool_call_id": message.tool_call_id,
                      "transport_status": message.status, "result": payload})
    request_counts = [len(image_urls(request)) for request in model.calls]
    frame_delivery = {identifier: value["delivered"] for identifier, value in holder["store"].frames.items()}
    observed = {"command": command, "bridge_requests": bridge.requests,
                "bridge_paths": bridge.calls, "bridge_waits": bridge.waits,
                "tool_messages": tools, "model_request_image_counts": request_counts,
                "model_request_count": len(model.calls), "frame_delivery": frame_delivery,
                "elapsed_frame_clock_seconds": now[0] - 100.,
                "initial_state": {"armed": initial_state["armed"], "landed_state": initial_state["landed_state"]},
                "final_state": {"armed": final_state["armed"], "landed_state": final_state["landed_state"]}}
    assertions = []
    def check(name, actual, expected):
        assertions.append({"name": name, "passed": actual == expected,
                           "expected": expected, "actual": actual})
    check("command_status", command["status"], spec.get("status", "failed"))
    check("exact_bridge_paths_no_extra_movement", bridge.calls, spec["expected_paths"])
    check("exact_model_request_count", len(model.calls), spec["model_calls"])
    check("initial_flight_ownership", initial_state["armed"], case_id == "unowned_airborne_observation_failure")
    check("final_armed_state", final_state["armed"], spec.get("armed", False))
    check("private_payload_absent_from_checkpoint_and_report",
          any(secret in encoded({"checkpoint": checkpoint, "observed": observed}) for secret in
              (MODEL_CONFIG["api_key"], JPEG_B64, "image_base64", "data:image/jpeg;base64,")), False)
    failed = spec.get("status", "failed") == "failed"
    check("failure_has_explicit_error", bool(command["error"]), failed)
    if failed:
        check("false_success_prose_is_not_published", "offline false success claim" in command["reply"], False)
    independent_landing = case_id in {
        "timeout_after_owned_takeoff", "connection_on_image_after_owned_takeoff",
        "camera_delivery_failure_blocks_movement", "unreported_frame_expired",
        "invalid_tool_parameters_block_retry", "failed_mission_blocks_retry"}
    if independent_landing:
        check("independent_landing_audited", (command["safety_landing"] or {}).get("status"), "ok")
        check("landing_waited_for_completion", bridge.waits[-1]["steps"], [{"action": "land"}])
        check("landing_disarmed_and_grounded", observed["final_state"], {"armed": False, "landed_state": 1})
        check("recovery_landing_does_not_claim_complete_original_flight", "飞行执行已完成" in command["reply"], False)
    else:
        check("no_unowned_or_duplicate_recovery_landing", command["safety_landing"], None)
    if case_id in {"connection_before_flight", "timeout_after_owned_takeoff",
                   "connection_on_image_after_owned_takeoff", "unowned_airborne_observation_failure"}:
        check("provider_fault_audit", command["model_calls"][-1]["status"], "error")
        expected_type = "APITimeoutError" if case_id == "timeout_after_owned_takeoff" else "APIConnectionError"
        check("provider_fault_type", command["model_calls"][-1].get("error_type"), expected_type)
    if case_id in {"connection_on_image_after_owned_takeoff", "unowned_airborne_observation_failure"}:
        check("failed_request_used_image", request_counts[-1], 1)
        check("failed_image_not_marked_delivered", list(frame_delivery.values()), [False])
        check("failed_image_has_no_successful_report", any(p.get("status") == "ok" for p in command["perceptions"]), False)
    if case_id == "camera_delivery_failure_blocks_movement":
        check("camera_failure_never_attached_image", sum(request_counts), 0)
        check("movement_tool_rejected_after_camera_failure", tools[-1]["result"].get("status"), "error")
        check("movement_rejection_keeps_vision_failure", "视觉分析已失败" in tools[-1]["result"].get("error", ""), True)
    if case_id == "unreported_frame_expired":
        check("frame_expiry_error", "已过期" in command["error"], True)
        check("failure_phase_is_image_preparation",
              (command["perceptions"][-1] if command["perceptions"] else {}).get("phase"), "image_preparation")
        check("expiry_has_no_false_provider_error", all(t.get("status") == "ok" for t in command["model_calls"]), True)
    if case_id == "saved_report_survives_frame_expiry":
        check("exactly_one_successful_saved_report", [p.get("status") for p in command["perceptions"]], ["ok"])
        check("image_only_on_delivery_request", request_counts, [0, 0, 1, 0, 0])
        check("clock_really_exceeds_image_ttl", now[0] - 100. > holder["store"].ttl_seconds, True)
        check("final_text_reached_model", command["reply"], "offline saved report retained; return and landing completed")
    if case_id == "invalid_tool_parameters_block_retry":
        flight_errors = [t["result"].get("error", "") for t in tools
                         if t["name"] == "execute_mission" and t["result"].get("status") == "error"]
        check("invalid_field_is_explicit", "Extra inputs" in flight_errors[0] and "altitude" in flight_errors[0], True)
        check("automatic_retry_is_explicitly_blocked", "禁止自动重试" in flight_errors[-1], True)
        check("invalid_steps_never_reach_bridge", bridge.requests[0]["payload"]["steps"], TAKEOFF)
    if case_id == "failed_mission_blocks_retry":
        check("first_flight_failure_preserved", command["error"], "offline injected mission failure: blocked passage")
        check("automatic_retry_rejected", any("禁止自动重试" in t["result"].get("error", "") for t in tools), True)
    if case_id == "operator_interrupt_preserves_authority":
        check("operator_interrupt_wins", command["error"], "该指令已由操作员中断")
    if case_id in {"final_reply_connection_after_completed_flight", "final_reply_error_after_saved_observation"}:
        check("completed_flight_is_explicit", "飞行执行已完成；模型最终回复生成失败" in command["reply"], True)
        check("completed_mission_preserved_in_public_command", (command.get("mission") or {}).get("status"), "succeeded")
        check("model_failure_keeps_api_error_audit", command["model_calls"][-1].get("error_type"), "APIConnectionError")
        check("final_model_failure_is_text_only", request_counts[-1], 0)
        check("failure_not_misattributed_to_flight_or_vision",
              "飞行任务失败" in command["reply"] or "视觉观察未完成" in command["reply"], False)
        if case_id == "final_reply_error_after_saved_observation":
            check("successful_report_preserved", [p.get("status") for p in command["perceptions"]], ["ok"])
            check("successful_report_is_acknowledged", "视觉观察报告已保存" in command["reply"], True)
    if case_id in {"final_reply_error_after_unconfirmed_acceptance", "final_reply_error_after_failed_flight",
                   "final_reply_error_after_cancelled_flight", "operator_interrupt_preserves_authority"}:
        check("uncertainty_or_cancel_does_not_claim_completed_flight", "飞行执行已完成" in command["reply"], False)
    if case_id == "final_reply_error_after_unconfirmed_acceptance":
        check("negative_control_really_never_waited", bridge.waits, [])
    return {"id": case_id, "source": SOURCE, "injection": spec["injection"],
            "expected_behavior": spec["expected"], "passed": all(a["passed"] for a in assertions),
            "assertions": assertions, "observed": observed,
            "duration_wall_seconds": round(time.monotonic() - started, 4)}


def run_suite(case_ids=None):
    started = time.monotonic()
    results = []
    for case_id in case_ids or CASES:
        try:
            results.append(run_case(case_id))
        except Exception as exc:
            results.append({"id": case_id, "source": SOURCE, "passed": False,
                            "injection": CASES[case_id]["injection"],
                            "expected_behavior": CASES[case_id]["expected"],
                            "harness_error": {"type": type(exc).__name__, "message": str(exc)},
                            "assertions": []})
    return {"schema_version": 1, "source": SOURCE,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "scope": {"model": "local scripted BaseChatModel; exceptions injected offline",
                      "framework": "real LangChain graph, single_vlm middleware, WebApplication and AuditedClient",
                      "bridge": "in-process stateful fixture; no ROS or real flight",
                      "transport": "network send paths blocked; credentials not read",
                      "case_pass_means": "expected failure or success contract verified"},
            "environment": {"python": sys.version.split()[0],
                            "langchain": importlib.metadata.version("langchain"),
                            "langgraph": importlib.metadata.version("langgraph")},
            "passed": bool(results) and all(r["passed"] for r in results),
            "case_count": len(results), "passed_count": sum(r["passed"] for r in results),
            "duration_wall_seconds": round(time.monotonic() - started, 4), "cases": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="New JSON report path (existing files are never overwritten)")
    parser.add_argument("--case", action="append", choices=list(CASES), help="Run selected case(s)")
    args = parser.parse_args()
    output = args.output or PROJECT_DIR / "logs/reliability" / (
        "agent_offline_" + datetime.now().strftime("%Y%m%d-%H%M%S") + "_" + uuid.uuid4().hex[:8] + ".json")
    if output.exists():
        parser.error("Refusing to overwrite existing evidence: " + str(output))
    report = run_suite(args.case)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps({"passed": report["passed"], "cases": report["case_count"],
                      "passed_cases": report["passed_count"], "report": str(output),
                      "source": SOURCE}, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
