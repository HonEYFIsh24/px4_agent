"""Serial, read-only provider diagnostics; dry-run unless --execute is explicit.

No Agent graph is constructed and no returned tool call is ever executed. Camera
input is a fresh Bridge GET snapshot, with the exact JPEG preserved as evidence.
"""
import argparse
import base64
import hashlib
import json
import math
import os
import re
import socket
import ssl
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

DEFAULT_ENV = (ROOT / ".env" if (ROOT / ".env").is_file()
               else Path("/home/dut/ros_agent/mcp-client-langchain/.env"))
READ_ONLY_SCHEMA = {"type": "function", "function": {
    "name": "get_drone_state",
    "description": "Read drone telemetry without changing state. Takes no arguments.",
    "parameters": {"type": "object", "properties": {}, "additionalProperties": False}}}


def safe_uri(value):
    """Retain only transport/host/port, never credentials, path or query."""
    if not value:
        return None
    try:
        parsed = urlsplit(value if "://" in value else "http://" + value)
        host = parsed.hostname
        if not host:
            return "<invalid URI>"
        host = "[{}]".format(host) if ":" in host else host
        return "{}://{}{}".format(parsed.scheme, host,
                                  ":{}".format(parsed.port) if parsed.port else "")
    except (TypeError, ValueError):
        return "<invalid URI>"


def clean_text(value, secrets=(), limit=1800):
    text = str(value)
    for secret in sorted((str(item) for item in secrets if item), key=len, reverse=True):
        text = text.replace(secret, "<redacted>")
    text = re.sub(r"[a-z][a-z0-9+.-]*://[^\s<>\"']+", lambda match: safe_uri(match.group(0)), text, flags=re.I)
    text = re.sub(r"(?i)(bearer\s+)[^\s,;]+", r"\1<redacted>", text)
    text = re.sub(r"(?i)(api[_-]?key|access[_-]?token|authorization)(\s*[=:]\s*)[^\s,;]+",
                  r"\1\2<redacted>", text)
    return text[:limit]


def safe_network_environment(environ):
    proxy_keys = ("HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "http_proxy", "https_proxy", "all_proxy")
    return {"proxies": {key: {"configured": bool(environ.get(key)),
                             "endpoint": safe_uri(environ.get(key))} for key in proxy_keys},
            "no_proxy_configured": bool(environ.get("NO_PROXY") or environ.get("no_proxy")),
            "custom_ca_configured": {key: bool(environ.get(key)) for key in
                                      ("SSL_CERT_FILE", "SSL_CERT_DIR", "REQUESTS_CA_BUNDLE")},
            "httpx_trust_env": True}


def exception_record(exc, secrets=()):
    chain, seen, current = [], set(), exc
    while current is not None and id(current) not in seen and len(chain) < 10:
        seen.add(id(current))
        chain.append({"type": type(current).__name__, "message": clean_text(current, secrets)})
        current = current.__cause__ or (None if current.__suppress_context__ else current.__context__)
    names = {item["type"] for item in chain}
    status = getattr(exc, "status_code", None)
    if any("Timeout" in name for name in names):
        category = "timeout"
    elif status is not None:
        category = ("server_error" if status >= 500 else "rate_limit" if status == 429
                    else "authentication" if status in (401, 403) else "request_error")
    elif names & {"gaierror"}:
        category = "dns_error"
    elif any("SSL" in name or "Certificate" in name for name in names):
        category = "tls_error"
    elif names & {"APIConnectionError", "ConnectError", "ConnectionError", "ConnectionRefusedError"}:
        category = "connection_error"
    else:
        category = "other_error"
    result = {"category": category, "chain": chain,
              "provider_fault_proven": False,
              "http_server_failure_observed": category == "server_error"}
    if status is not None:
        result["http_status"] = status
    return result


def network_checks(base_url, secrets=()):
    """Unauthenticated DNS/TLS checks, separate from the configured HTTP proxy path."""
    parsed = urlsplit(base_url or "https://api.openai.com/v1")
    host, port = parsed.hostname, parsed.port or (443 if parsed.scheme == "https" else 80)
    result = {"endpoint": safe_uri(base_url), "scope": "direct DNS/TLS; not the HTTP proxy request path"}
    try:
        addresses = socket.getaddrinfo(host, port, type=socket.SOCK_STREAM)
        result["dns"] = {"ok": True, "addresses": sorted({item[4][0] for item in addresses})}
    except Exception as exc:
        result["dns"] = {"ok": False, "error": exception_record(exc, secrets)}
        return result
    if parsed.scheme != "https":
        result["tls"] = {"attempted": False}
        return result
    started = time.monotonic()
    try:
        with socket.create_connection((host, port), timeout=5) as raw:
            with ssl.create_default_context().wrap_socket(raw, server_hostname=host) as secured:
                result["tls"] = {"attempted": True, "ok": True, "protocol": secured.version(),
                                 "cipher": secured.cipher()[0], "certificate_verified": True}
    except Exception as exc:
        result["tls"] = {"attempted": True, "ok": False, "error": exception_record(exc, secrets)}
    result["tls"]["duration_s"] = round(time.monotonic() - started, 3)
    return result


def validate_snapshot(snapshot):
    if snapshot.get("status") != "ok" or snapshot.get("mime_type") != "image/jpeg":
        raise ValueError("Bridge did not provide a successful JPEG snapshot")
    age, measurement = snapshot["age_seconds"], snapshot["measurement_age_seconds"]
    stamp = snapshot["stamp_sim_s"]
    if (not all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v)
                for v in (age, measurement, stamp))
            or not 0 <= age <= 2 or not 0 <= measurement <= .5):
        raise ValueError("Snapshot is stale, future-dated or has an invalid clock")
    image = snapshot["image_base64"]
    if not isinstance(image, str) or len(image) > 3000000:
        raise ValueError("Snapshot JPEG encoding exceeds its limit")
    raw = base64.b64decode(image, validate=True)
    if not raw.startswith(b"\xff\xd8") or not raw.endswith(b"\xff\xd9"):
        raise ValueError("Snapshot bytes are not a JPEG")
    calibration = snapshot["camera_intrinsics"]
    if (abs(calibration["stamp_sim_s"] - stamp) > 1e-6
            or calibration["width"] != snapshot["width"]
            or calibration["height"] != snapshot["height"]):
        raise ValueError("Snapshot calibration does not belong to the image frame")
    offset = snapshot["pose_alignment"]["pose_minus_image_sim_s"]
    if not math.isfinite(offset) or abs(offset) > .05:
        raise ValueError("Snapshot pose is not aligned with the image")
    metadata = {key: value for key, value in snapshot.items() if key != "image_base64"}
    metadata.update(image_sha256=hashlib.sha256(raw).hexdigest(), jpeg_size_bytes=len(raw))
    return raw, image, metadata


def telemetry_summary(state):
    return {"armed": state.get("armed"), "landed_state": state.get("landed_state"),
            "position_enu_m": state.get("position_enu_m"),
            "mission_id": (state.get("mission") or {}).get("id"),
            "mission_status": (state.get("mission") or {}).get("status")}


def response_record(message, secrets=(), schema_profile="empty_object"):
    # This deliberately parses JSON fields only; callable tools are never installed.
    calls = getattr(message, "tool_calls", []) or []
    valid = len(calls) == 1 and calls[0].get("name") == "get_drone_state"
    arguments = calls[0].get("args") if calls else None
    if schema_profile == "typed_view":
        valid = valid and isinstance(arguments, dict) and set(arguments) == {"view"} and arguments["view"] in ("flight", "sensors", "all")
    else:
        valid = valid and arguments == {}
    return {"text": clean_text(getattr(message, "content", ""), secrets),
            "tool_calls": [{"name": clean_text(item.get("name"), secrets),
                            "arguments_json": clean_text(json.dumps(item.get("args"), ensure_ascii=False), secrets)}
                           for item in calls],
            "read_only_schema_valid": valid,
            "tool_calls_executed": 0,
            "invalid_tool_calls": clean_text(json.dumps(getattr(message, "invalid_tool_calls", []) or [],
                                                         ensure_ascii=False), secrets),
            "finish_reason": getattr(message, "response_metadata", {}).get("finish_reason"),
            "token_usage": getattr(message, "usage_metadata", None)}


class ReadOnlySDKModel:
    """Direct SDK counterpart, solely for isolating LangChain from provider behavior."""
    def __init__(self, config, options):
        from openai import OpenAI
        self.client = OpenAI(api_key=config["api_key"], base_url=config.get("base_url"),
                             timeout=options["timeout"], max_retries=0)
        self.request_options = {key: value for key, value in options.items()
                                if key not in ("timeout", "max_retries")}
        self.request_options["model"] = config["model"]
        self.bound = {}

    def bind_tools(self, schemas, tool_choice):
        self.bound = {"tools": schemas, "tool_choice": (tool_choice if tool_choice == "auto" else
                       {"type": "function", "function": {"name": tool_choice}})}
        return self

    def invoke(self, messages):
        raw = self.client.chat.completions.create(
            **self.request_options, **self.bound,
            messages=[{"role": "system" if message.type == "system" else "user", "content": message.content}
                      for message in messages])
        calls, invalid = [], []
        for call in raw.choices[0].message.tool_calls or []:
            try:
                calls.append({"name": call.function.name, "args": json.loads(call.function.arguments)})
            except (ValueError, TypeError):
                invalid.append({"name": call.function.name, "args": call.function.arguments})
        return SimpleNamespace(content=raw.choices[0].message.content or "", tool_calls=calls,
                               invalid_tool_calls=invalid, usage_metadata=raw.usage.model_dump() if raw.usage else None,
                               response_metadata={"finish_reason": raw.choices[0].finish_reason})


def run_diagnostics(config, kinds, repeats, output, bridge, model_factory=None,
                    failure_callback=None, include_network_checks=True, tool_choice_mode="named",
                    tool_prompt_profile="basic", tool_schema_profile="empty_object", transport="langchain"):
    """Inject model/Bridge in unit tests; production model is the existing ChatOpenAI stack."""
    from langchain_core.messages import HumanMessage, SystemMessage
    if model_factory is None:
        from langchain_openai import ChatOpenAI
        model_factory = ChatOpenAI
    directory = Path(output)
    directory.mkdir(parents=True, exist_ok=False)
    secrets = [config.get("api_key")]
    for key in ("HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "http_proxy", "https_proxy", "all_proxy"):
        try:
            parsed = urlsplit(os.environ.get(key, ""))
            secrets.extend([parsed.username, parsed.password])
        except ValueError:
            pass
    options = {"temperature": 0, "timeout": 45, "max_retries": 0, "max_tokens": 2200}
    if (urlsplit(config.get("base_url") or "").hostname or "").endswith("aliyuncs.com"):
        options["extra_body"] = {"enable_thinking": False}
    schema = json.loads(json.dumps(READ_ONLY_SCHEMA))
    if tool_schema_profile == "omit_parameters":
        del schema["function"]["parameters"]
    elif tool_schema_profile == "typed_view":
        schema["function"].update(description="Read one current telemetry view without changing state.", parameters={
            "type": "object", "properties": {"view": {"type": "string", "enum": ["flight", "sensors", "all"],
            "description": "flight: flight state; sensors: sensor health; all: both."}},
            "required": ["view"], "additionalProperties": False})
    report = {"schema_version": 1, "started_at_wall_s": time.time(), "actual_provider_calls": True,
              "actual_flight": False, "tool_calls_executed": 0,
              "independent_trials": True, "automatic_model_retries": 0,
              "model": {"name": config["model"], "endpoint": safe_uri(config.get("base_url")),
                        "api_key_configured": bool(config.get("api_key")), "options": options},
              "read_only_tool_schema": schema, "tool_choice_mode": tool_choice_mode,
              "tool_prompt_profile": tool_prompt_profile,
              "tool_schema_profile": tool_schema_profile,
              "transport": transport,
              "environment": safe_network_environment(os.environ), "trials": [],
              "scope_note": "Read-only standalone LangChain model diagnostics; does not validate full Agent task completion."}
    path = directory / "diagnostics.json"

    def save():
        # Directory is exclusively owned by this run; replace only its own checkpoint.
        temporary = directory / ".diagnostics.pending"
        temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(path)

    save()
    if include_network_checks:
        report["network_checks"] = network_checks(config.get("base_url"), secrets)
        save()
    model = (ReadOnlySDKModel(config, options) if transport == "openai_sdk"
             else model_factory(**config, **options))
    for kind in kinds:
        for iteration in range(1, repeats + 1):
            trial = {"kind": kind, "iteration": iteration, "started_at_wall_s": time.time(),
                     "provider_request_sent": False, "tool_calls_executed": 0}
            started = time.monotonic()
            try:
                selected_model = model
                messages = [SystemMessage(content="你正在参加只读接口诊断。不要建议或执行飞行，不要生成额外工具。")]
                if kind == "text":
                    messages.append(HumanMessage(content="请只回答：文本接口正常。"))
                elif kind == "image":
                    before = telemetry_summary(bridge.state())
                    if before["armed"] is not False or before["landed_state"] != 1:
                        raise RuntimeError("Image diagnostics require the simulator to be disarmed and landed")
                    raw, image, metadata = validate_snapshot(bridge.capture_camera("left"))
                    prefix = "image_{:02d}".format(iteration)
                    with (directory / (prefix + ".jpg")).open("xb") as stream:
                        stream.write(raw)
                    with (directory / (prefix + ".json")).open("x", encoding="utf-8") as stream:
                        json.dump(metadata, stream, ensure_ascii=False, indent=2)
                    trial.update(camera=metadata, state_before_capture=before,
                                 image_evidence_file=prefix + ".jpg")
                    messages.append(HumanMessage(content=[
                        {"type": "text", "text": "请用一两句话描述这张无人机相机实际图像中可见的颜色和物体；不要猜测米制位置。"},
                        {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + image}}]))
                elif kind == "tool":
                    selected_model = model.bind_tools([schema], tool_choice=(
                        "get_drone_state" if tool_choice_mode == "named" else "auto"))
                    if tool_schema_profile == "typed_view":
                        messages.append(HumanMessage(content="请调用 get_drone_state，读取完整遥测。只传参数 view='all'，不要其他字段。只解析不执行工具。"))
                    elif tool_prompt_profile == "exact_empty":
                        messages[0] = SystemMessage(content=(
                            "这是只读工具接口契约验证，不执行工具，不进行飞行。你必须调用 get_drone_state 一次。"
                            "该函数没有任何参数。function.arguments 必须严格等于 JSON 空对象 {}。"
                            "禁止任何额外键，包括 example_parameter_1、example_parameter_2；禁止把示例参数填入调用。"
                            "不要生成解释文本或其他函数调用。唯一正确的参数对象是 {}。"))
                        messages.append(HumanMessage(content="只调用 get_drone_state，arguments 必须 EXACT {}。不要示例参数。"))
                    else:
                        messages.append(HumanMessage(content="请调用 get_drone_state 读取状态；参数必须为空对象。此诊断只解析调用，不执行工具。"))
                else:
                    raise ValueError("Unknown diagnostic kind: " + kind)
                trial["provider_request_sent"] = True
                message = selected_model.invoke(messages)
                trial["response"] = response_record(message, secrets, tool_schema_profile)
                trial["passed"] = (trial["response"]["read_only_schema_valid"] if kind == "tool"
                                   else bool(getattr(message, "content", "")))
                if kind == "image":
                    trial["state_after_response"] = telemetry_summary(bridge.state())
                if not trial["passed"]:
                    trial["contract_error"] = "Model did not return the required text or read-only schema"
                    if failure_callback:
                        failure_callback({"kind": kind, "iteration": iteration,
                                          "provider_request_sent": trial["provider_request_sent"],
                                          "category": "model_contract_error",
                                          "response": trial["response"], "contract_error": trial["contract_error"]})
            except Exception as exc:
                trial["passed"] = False
                trial["error"] = exception_record(exc, secrets)
                if failure_callback:
                    failure_callback({"kind": kind, "iteration": iteration,
                                      "provider_request_sent": trial["provider_request_sent"], "error": trial["error"]})
            trial["duration_s"] = round(time.monotonic() - started, 3)
            report["trials"].append(trial)
            save()
            print(json.dumps({"kind": kind, "iteration": iteration, "passed": trial["passed"],
                              "duration_s": trial["duration_s"], "error": trial.get("error")}, ensure_ascii=False), flush=True)
    report.update(finished_at_wall_s=time.time(),
                  passed=sum(item["passed"] for item in report["trials"]),
                  total=len(report["trials"]))
    report["all_passed"] = report["passed"] == report["total"]
    save()
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="Authorize actual provider requests; default is zero-I/O dry-run")
    parser.add_argument("--env-file", default=os.environ.get("UAV_ENV_FILE", str(DEFAULT_ENV)))
    parser.add_argument("--bridge-url", default="http://127.0.0.1:8765")
    parser.add_argument("--kind", action="append", choices=("text", "image", "tool"))
    parser.add_argument("--repeats", type=int, choices=(1, 2, 3), default=2)
    parser.add_argument("--tool-choice", choices=("named", "auto"), default="named",
                        help="Separate experiments can compare named forced calls and production-style auto")
    parser.add_argument("--tool-prompt", choices=("basic", "exact_empty"), default="basic",
                        help="Explicitly separate strengthened-prompt trials from the original contract failures")
    parser.add_argument("--tool-schema", choices=("empty_object", "omit_parameters", "typed_view"), default="empty_object",
                        help="Read-only schema experiment; never changes production tool signatures")
    parser.add_argument("--transport", choices=("langchain", "openai_sdk"), default="langchain")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    kinds = args.kind or ["text", "image", "tool"]
    if len(set(kinds)) != len(kinds):
        parser.error("Each diagnostic kind may be specified once")
    if not args.execute:
        print(json.dumps({"dry_run": True, "actual_provider_calls": False, "actual_flight": False,
                          "kinds": kinds, "repeats": args.repeats, "tool_calls_executed": 0,
                          "tool_choice_mode": args.tool_choice,
                          "tool_prompt_profile": args.tool_prompt,
                          "tool_schema_profile": args.tool_schema,
                          "transport": args.transport,
                          "output_created": False}, ensure_ascii=False))
        return 0
    if args.output is None:
        parser.error("--execute requires a new --output directory")
    from agent_config import runtime_configuration
    from client import BridgeClient
    from model_network import configure_agent_network
    configure_agent_network(args.env_file)
    architecture, config, _ = runtime_configuration(args.env_file)
    if architecture != "single_vlm":
        parser.error("These diagnostics target the configured single_vlm; do not change configuration")
    report = run_diagnostics(config, kinds, args.repeats, args.output, BridgeClient(args.bridge_url),
                             failure_callback=lambda failure: print("MODEL_DIAGNOSTIC_FAILURE " +
                                                                   json.dumps(failure, ensure_ascii=False), flush=True),
                             tool_choice_mode=args.tool_choice, tool_prompt_profile=args.tool_prompt,
                             tool_schema_profile=args.tool_schema, transport=args.transport)
    return 0 if report["all_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
