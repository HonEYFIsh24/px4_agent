"""Read-only diagnostics must preserve failures and never dispatch model tools."""
import base64
import contextlib
import importlib.util
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/diagnose_model_connection.py"
SPEC = importlib.util.spec_from_file_location("model_diagnostics", SCRIPT)
diagnostics = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(diagnostics)

try:
    import langchain_core.messages
    HAS_LANGCHAIN = True
except ImportError:
    HAS_LANGCHAIN = False


def snapshot():
    return {"status": "ok", "mime_type": "image/jpeg", "width": 640, "height": 480,
            "age_seconds": .1, "measurement_age_seconds": .03, "stamp_sim_s": 20.,
            "image_base64": base64.b64encode(b"\xff\xd8exact-frame\xff\xd9").decode(),
            "camera_intrinsics": {"width": 640, "height": 480, "stamp_sim_s": 20.},
            "pose_alignment": {"pose_minus_image_sim_s": .02}, "simulation_only": True}


class ReadOnlyBridge:
    def __init__(self):
        self.reads = []

    def state(self):
        self.reads.append("/state")
        return {"armed": False, "landed_state": 1, "mission": {"id": "existing"}}

    def capture_camera(self, side):
        self.reads.append("/camera/snapshot?side=" + side)
        return snapshot()

    def execute(self, *args, **kwargs):
        raise AssertionError("diagnostics attempted flight")

    def request(self, *args, **kwargs):
        raise AssertionError("diagnostics bypassed the read-only Bridge API")


class DummyModel:
    def __init__(self, responses):
        self.responses = iter(responses)
        self.requests = []
        self.schemas = []

    def bind_tools(self, schemas, tool_choice):
        self.schemas.append((schemas, tool_choice))
        return self

    def invoke(self, messages):
        self.requests.append(messages)
        response = next(self.responses)
        if isinstance(response, Exception):
            raise response
        return response


class DiagnosticsTests(unittest.TestCase):
    def test_typed_view_requires_exact_legal_enum_and_rejects_extra_fields(self):
        for args, expected in (({"view": "all"}, True), ({"view": "sensors"}, True),
                               ({"view": "shell"}, False), ({"view": "all", "extra": 1}, False), ({}, False)):
            with self.subTest(args=args):
                response = diagnostics.response_record(SimpleNamespace(content="", tool_calls=[{
                    "name": "get_drone_state", "args": args}]), schema_profile="typed_view")
                self.assertEqual(response["read_only_schema_valid"], expected)

    def test_redacts_exact_keys_bearer_and_uri_credentials_queries(self):
        error = "credential123 Bearer credential456 https://user:pass@provider.example/v1?api_key=credential789"
        result = diagnostics.clean_text(error, ["credential123"])
        for sensitive in ("credential123", "credential456", "credential789", "user:", ":pass@"):
            self.assertNotIn(sensitive, result)
        self.assertIn("https://provider.example", result)

    def test_environment_records_presence_without_proxy_credentials_or_custom_paths(self):
        result = diagnostics.safe_network_environment({
            "HTTPS_PROXY": "http://username:password@proxy.example:8080/private?token=secret",
            "NO_PROXY": "private.company", "SSL_CERT_FILE": "/secret/private/cert.pem"})
        serialized = json.dumps(result)
        for value in ("username", "password", "/private", "secret", "private.company"):
            self.assertNotIn(value, serialized)
        self.assertEqual(result["proxies"]["HTTPS_PROXY"]["endpoint"], "http://proxy.example:8080")

    def test_exception_keeps_causal_class_but_does_not_assert_provider_fault(self):
        try:
            try:
                raise ConnectionRefusedError("https://u:p@provider.example/v1?key=secret refused")
            except ConnectionRefusedError as inner:
                raise ConnectionError("outer secret") from inner
        except ConnectionError as error:
            record = diagnostics.exception_record(error, ["secret"])
        self.assertEqual([item["type"] for item in record["chain"]],
                         ["ConnectionError", "ConnectionRefusedError"])
        self.assertEqual(record["category"], "connection_error")
        self.assertFalse(record["provider_fault_proven"])
        self.assertNotIn("secret", json.dumps(record))

    def test_rejects_calibration_from_different_frame_and_unaligned_pose(self):
        item = snapshot()
        item["camera_intrinsics"]["stamp_sim_s"] += .01
        with self.assertRaisesRegex(ValueError, "image frame"):
            diagnostics.validate_snapshot(item)
        item = snapshot()
        item["pose_alignment"]["pose_minus_image_sim_s"] = .051
        with self.assertRaisesRegex(ValueError, "not aligned"):
            diagnostics.validate_snapshot(item)

    def test_dry_run_never_creates_output_or_calls_provider(self):
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "not-created"
            with contextlib.redirect_stdout(io.StringIO()) as text:
                code = diagnostics.main(["--output", str(output)])
            self.assertEqual(code, 0)
            self.assertFalse(output.exists())
            self.assertFalse(json.loads(text.getvalue())["actual_provider_calls"])

    @unittest.skipUnless(HAS_LANGCHAIN, "Live model orchestration uses Agent Python")
    def test_records_initial_error_without_retry_and_never_executes_tool_calls(self):
        model = DummyModel([ConnectionError("keysecret failed"),
                            SimpleNamespace(content="", tool_calls=[{"name": "execute_mission", "args": {"steps": []}}]),
                            SimpleNamespace(content="", tool_calls=[{"name": "get_drone_state", "args": {}}])])
        options = []
        bridge = ReadOnlyBridge()
        failures = []
        def factory(**kwargs):
            options.append(kwargs)
            return model
        with tempfile.TemporaryDirectory() as root, contextlib.redirect_stdout(io.StringIO()):
            output = Path(root) / "new-run"
            result = diagnostics.run_diagnostics(
                {"model": "qwen3-vl-plus", "base_url": "https://provider.aliyuncs.com/v1", "api_key": "keysecret"},
                ["tool"], 3, output, bridge, factory, failures.append, False)
            evidence = json.loads((output / "diagnostics.json").read_text())
        self.assertEqual(len(model.requests), 3)
        self.assertEqual(options[0]["max_retries"], 0)
        self.assertFalse(options[0]["extra_body"]["enable_thinking"])
        self.assertEqual([trial["passed"] for trial in result["trials"]], [False, False, True])
        self.assertEqual(len(failures), 2)
        self.assertEqual(failures[1]["category"], "model_contract_error")
        self.assertEqual(result["tool_calls_executed"], 0)
        self.assertEqual(bridge.reads, [])
        self.assertNotIn("keysecret", json.dumps(evidence))
        self.assertEqual(model.schemas[0][0], [diagnostics.READ_ONLY_SCHEMA])

    @unittest.skipUnless(HAS_LANGCHAIN, "Live model orchestration uses Agent Python")
    def test_auto_tool_selection_is_explicit_and_still_requires_empty_arguments(self):
        model = DummyModel([SimpleNamespace(content="", tool_calls=[{
            "name": "get_drone_state", "args": {"unexpected": 1}}])])
        with tempfile.TemporaryDirectory() as root, contextlib.redirect_stdout(io.StringIO()):
            result = diagnostics.run_diagnostics(
                {"model": "test-vlm", "api_key": "private-key"}, ["tool"], 1,
                Path(root) / "new-run", ReadOnlyBridge(), lambda **kwargs: model,
                include_network_checks=False, tool_choice_mode="auto")
        self.assertEqual(model.schemas[0][1], "auto")
        self.assertFalse(result["all_passed"])

    @unittest.skipUnless(HAS_LANGCHAIN, "Live model orchestration uses Agent Python")
    def test_strengthened_prompt_still_does_not_drop_unexpected_parameters(self):
        model = DummyModel([SimpleNamespace(content="", tool_calls=[{
            "name": "get_drone_state", "args": {"example_parameter_1": ""}}])])
        with tempfile.TemporaryDirectory() as root, contextlib.redirect_stdout(io.StringIO()):
            result = diagnostics.run_diagnostics(
                {"model": "test-vlm", "api_key": "private-key"}, ["tool"], 1,
                Path(root) / "new-run", ReadOnlyBridge(), lambda **kwargs: model,
                include_network_checks=False, tool_prompt_profile="exact_empty")
        self.assertEqual(result["tool_prompt_profile"], "exact_empty")
        self.assertIn("example_parameter_1", model.requests[0][0].content)
        self.assertFalse(result["all_passed"])

    @unittest.skipUnless(HAS_LANGCHAIN, "Live model orchestration uses Agent Python")
    def test_omitted_parameters_trial_does_not_mutate_default_schema_or_accept_extra_keys(self):
        model = DummyModel([SimpleNamespace(content="", tool_calls=[{
            "name": "get_drone_state", "args": {"unexpected": 1}}])])
        with tempfile.TemporaryDirectory() as root, contextlib.redirect_stdout(io.StringIO()):
            result = diagnostics.run_diagnostics(
                {"model": "test-vlm", "api_key": "private-key"}, ["tool"], 1,
                Path(root) / "new-run", ReadOnlyBridge(), lambda **kwargs: model,
                include_network_checks=False, tool_schema_profile="omit_parameters")
        self.assertNotIn("parameters", model.schemas[0][0][0]["function"])
        self.assertIn("parameters", diagnostics.READ_ONLY_SCHEMA["function"])
        self.assertFalse(result["all_passed"])

    @unittest.skipUnless(HAS_LANGCHAIN, "Live model orchestration uses Agent Python")
    def test_image_input_matches_exact_saved_bytes_and_read_only_snapshot(self):
        model = DummyModel([SimpleNamespace(content="visible image", tool_calls=[])])
        bridge = ReadOnlyBridge()
        with tempfile.TemporaryDirectory() as root, contextlib.redirect_stdout(io.StringIO()):
            output = Path(root) / "new-run"
            result = diagnostics.run_diagnostics(
                {"model": "test-vlm", "base_url": "https://example.test/v1", "api_key": "private-key"},
                ["image"], 1, output, bridge, lambda **kwargs: model, include_network_checks=False)
            raw = (output / "image_01.jpg").read_bytes()
            image_input = model.requests[0][-1].content[1]["image_url"]["url"].split(",", 1)[1]
            self.assertEqual(base64.b64decode(image_input), raw)
            self.assertEqual(result["trials"][0]["camera"]["jpeg_size_bytes"], len(raw))
        self.assertEqual(bridge.reads, ["/state", "/camera/snapshot?side=left", "/state"])
        self.assertTrue(result["all_passed"])

    @unittest.skipUnless(HAS_LANGCHAIN, "Live model orchestration uses Agent Python")
    def test_existing_evidence_directory_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as output:
            with self.assertRaises(FileExistsError):
                diagnostics.run_diagnostics({}, [], 0, output, ReadOnlyBridge(), lambda **kwargs: None)


if __name__ == "__main__":
    unittest.main()
