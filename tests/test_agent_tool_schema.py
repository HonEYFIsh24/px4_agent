"""Model-facing flight schema and strict tool boundaries, with local fakes only."""
import copy
import json
import unittest
from unittest import mock

try:
    from langchain.agents import create_agent
    from langchain_core.utils.function_calling import convert_to_openai_tool
    from pydantic import ValidationError
    import agent
except ImportError as exc:
    raise unittest.SkipTest("Tool schema tests require the Agent Python3.11 LangChain environment") from exc


VALID_STEPS = [
    {"action": "takeoff", "altitude": 3.5},
    {"action": "goto", "x": 13., "y": 0., "altitude": 3.5},
    {"action": "hold", "duration": 2.},
    {"action": "return_home"},
    {"action": "land"},
]


class AgentToolSchemaTests(unittest.TestCase):
    def test_telemetry_and_safety_tools_reject_fabricated_parameters_before_execution(self):
        client = mock.Mock()
        for t in agent.flight_tools(client):
            if t.name in ('get_drone_state', 'stop_drone', 'land_drone'):
                schema = convert_to_openai_tool(t)['function']['parameters']
                self.assertIs(schema['additionalProperties'], False)
                with self.assertRaises(ValidationError):
                    t.invoke({'example_parameter_1': '', 'example_parameter_2': ''})
        client.state.assert_not_called()
        client.request.assert_not_called()

    def test_telemetry_view_has_real_semantics_and_default_compatibility(self):
        client = mock.Mock()
        client.state.return_value = {'armed': False, 'vio': {'ready': True}, 'camera': {}, 'mission': {}}
        t = agent.flight_tools(client)[0]
        self.assertEqual(t.invoke({}), client.state.return_value)
        self.assertIn('vio', t.invoke({'view': 'sensors'}))
        self.assertNotIn('mission', t.invoke({'view': 'sensors'}))
        with self.assertRaises(ValidationError):
            t.invoke({'view': 'unsupported'})

    def setUp(self):
        self.client = mock.Mock()
        self.client.execute.return_value = {"mission": {"status": "succeeded"}}
        self.execute = next(tool for tool in agent.flight_tools(self.client) if tool.name == "execute_mission")

    def invoke(self, steps):
        return self.execute.invoke({"steps": steps, "description": "schema contract test"})

    def test_provider_schema_exposes_only_each_actions_required_fields(self):
        schema = convert_to_openai_tool(self.execute)["function"]["parameters"]
        item = schema["properties"]["steps"]["items"]
        self.assertEqual(item["discriminator"]["propertyName"], "action")
        variants = {variant["properties"]["action"]["const"]: variant for variant in item["oneOf"]}
        self.assertEqual(set(variants), {step["action"] for step in VALID_STEPS})
        for step in VALID_STEPS:
            with self.subTest(action=step["action"]):
                variant = variants[step["action"]]
                self.assertEqual(set(variant["properties"]), set(step))
                self.assertEqual(set(variant["required"]), set(step))
                self.assertIs(variant["additionalProperties"], False)
                for field in variant["properties"].values():
                    self.assertNotIn("anyOf", field)
        # LangChain inlines definitions; the emitted API schema must not retain
        # discriminator references to definitions it has removed.
        self.assertNotIn("#/$defs/", json.dumps(schema))
        self.assertNotIn("$ref", json.dumps(schema))

    def test_missionstep_legacy_construction_reads_and_flat_export_still_work(self):
        for raw in VALID_STEPS:
            with self.subTest(action=raw["action"]):
                constructed = agent.MissionStep(**raw)
                validated = agent.MissionStep.model_validate(raw)
                self.assertEqual(constructed.action, raw["action"])
                self.assertEqual(constructed.model_dump(), raw)
                self.assertEqual(validated.model_dump(exclude_none=True), raw)
                self.assertEqual(constructed.altitude, raw.get("altitude"))
                self.assertEqual(constructed.x, raw.get("x"))
                self.assertEqual(constructed.duration, raw.get("duration"))

    def test_valid_full_sortie_reaches_client_with_exact_action_fields(self):
        steps = copy.deepcopy(VALID_STEPS)
        result = self.invoke(steps)
        self.assertEqual(result["mission"]["status"], "succeeded")
        forwarded, description, progress = self.client.execute.call_args.args
        self.assertEqual(forwarded, VALID_STEPS)
        self.assertEqual(description, "schema contract test")
        self.assertTrue(callable(progress))
        self.assertEqual(steps, VALID_STEPS)
        self.assertEqual(forwarded[-2:], [{"action": "return_home"}, {"action": "land"}])

    def test_missing_required_fields_are_rejected_before_any_flight_call(self):
        for raw in VALID_STEPS:
            for field in raw:
                with self.subTest(action=raw["action"], missing=field):
                    invalid = {key: value for key, value in raw.items() if key != field}
                    with self.assertRaises(ValidationError):
                        self.invoke([invalid])
        self.client.execute.assert_not_called()

    def test_irrelevant_fields_are_rejected_even_when_null_without_stripping(self):
        fields = {"x", "y", "altitude", "duration", "unused"}
        for raw in VALID_STEPS:
            for field in fields - set(raw):
                for value in (None, 1.):
                    with self.subTest(action=raw["action"], extra=field, value=value):
                        invalid = {**raw, field: value}
                        original = copy.deepcopy(invalid)
                        with self.assertRaises(ValidationError):
                            self.invoke([invalid])
                        self.assertEqual(invalid, original)
        self.client.execute.assert_not_called()

    def test_numeric_strings_booleans_nonfinite_and_null_are_not_flight_numbers(self):
        for raw in VALID_STEPS:
            for field in set(raw) - {"action"}:
                for value in ("2", True, float("nan"), float("inf"), None):
                    with self.subTest(action=raw["action"], field=field, value=value):
                        with self.assertRaises(ValidationError):
                            self.invoke([{**raw, field: value}])
        self.client.execute.assert_not_called()

    def test_invalid_later_return_step_rejects_the_whole_tool_submission(self):
        invalid = copy.deepcopy(VALID_STEPS)
        invalid[-2]["altitude"] = 3.5
        with self.assertRaises(ValidationError):
            self.invoke(invalid)
        self.client.execute.assert_not_called()
        self.assertEqual(invalid[-2], {"action": "return_home", "altitude": 3.5})

    def test_single_vlm_inherits_exact_action_fields_and_no_null_instruction(self):
        with mock.patch("single_vlm.build_single_vlm_agent", return_value=object()) as build:
            agent.build_agent(self.client, {"model": "fake", "api_key": "fake"}, architecture="single_vlm")
        prompt = build.call_args.args[3]
        for example in ('{"action":"takeoff","altitude":3.5}',
                        '{"action":"goto","x":13,"y":0,"altitude":3.5}',
                        '{"action":"hold","duration":2}',
                        '{"action":"return_home"}', '{"action":"land"}'):
            self.assertIn(example, prompt)
        self.assertIn("禁止用 null 填充任何多余字段", prompt)

    def test_schema_failure_is_audited_and_blocks_model_retry_but_allows_land(self):
        import test_single_vlm as local_graph
        from test_web_server import completed

        fixture = local_graph.WebGuardTests()
        app, bridge, _ = fixture.application([
            local_graph.tool_call("execute_mission", {"steps": [{"action": "return_home", "altitude": 3.5}],
                "description": "invalid return"}, "bad-return"),
            local_graph.tool_call("execute_mission", {"steps": [{"action": "takeoff", "altitude": 3.5}],
                "description": "forbidden automatic retry"}, "blocked-retry"),
            local_graph.tool_call("land_drone", {}, "explicit-land"),
            local_graph.AIMessage("all succeeded")], agent.flight_tools, bridge=local_graph.FlightCameraClient())
        command = completed(app, app.submit("返航后降落", mode="task")["id"])
        self.assertEqual(command["status"], "failed")
        self.assertIn("altitude", command["error"])
        self.assertEqual(bridge.calls, ["/land"])
        self.assertNotIn("all succeeded", command["reply"])
        messages = app.agent.get_state({"configurable": {"thread_id": app.agent_thread_id}}).values["messages"]
        failures = [json.loads(message.content) for message in messages
                    if isinstance(message, local_graph.ToolMessage) and message.name == "execute_mission"]
        self.assertEqual([value["status"] for value in failures], ["error", "error"])
        self.assertIn("Extra inputs", failures[0]["error"])
        self.assertIn("禁止自动重试", failures[1]["error"])


if __name__ == "__main__":
    unittest.main()
