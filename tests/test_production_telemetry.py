"""Pure read-only production telemetry boundaries; no model requests."""
import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
SPEC=importlib.util.spec_from_file_location('production_telemetry_contract',ROOT/'scripts/test_model_telemetry.py')
telemetry=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(telemetry)


def state():
    return {'simulation_only':True,'armed':False,'landed_state':1,'mission':{'status':'succeeded'},
            'vio':{'ready':True,'used_for_flight_control':False},'external_vision':{'used_for_flight_control':True}}


def messages():
    result=state()
    reply={'armed':False,'landed_state':1,'vio_ready':True,'used_for_flight_control':True}
    return [{'type':'ai','tool_calls':[{'name':'get_drone_state','args':{'view':'all'}}],'content':''},
            {'type':'tool','name':'get_drone_state','content':json.dumps(result)},
            {'type':'ai','tool_calls':[],'content':json.dumps(reply)}]


class ProductionTelemetryTests(unittest.TestCase):
    def test_only_real_local_state_get_reaches_base_transport(self):
        client=telemetry.ReadonlyBridge()
        with mock.patch.object(telemetry.BridgeClient,'request',return_value=state()) as actual:
            self.assertEqual(client.state(),state())
            actual.assert_called_once_with('/state')
        self.assertEqual(client.transport_events[0]['method'],'GET')
        self.assertFalse(client.supports_stereo_localization)

    def test_every_mutation_wait_and_camera_path_blocks_before_transport(self):
        client=telemetry.ReadonlyBridge()
        operations=[lambda:client.request('/state',{}),lambda:client.request('/land',{}),
            lambda:client.request('/stop',{}),lambda:client.execute([{'action':'takeoff','altitude':3.5}]),
            lambda:client.wait_for({'mission':{'id':'original'}}),lambda:client.capture_camera(),
            lambda:client.capture_stereo(),lambda:client.request('/camera/snapshot?side=left')]
        with mock.patch.object(telemetry.BridgeClient,'request',side_effect=AssertionError('transport forbidden')) as actual:
            for operation in operations:
                with self.assertRaises(RuntimeError):operation()
            actual.assert_not_called()
        self.assertEqual(len(client.blocked),len(operations))
        self.assertTrue(all(event['blocked_before_transport'] for event in client.blocked))

    def test_nonlocal_or_nonhttp_bridge_refused(self):
        for url in ('https://127.0.0.1','http://example.com','file:///tmp/state'):
            with self.assertRaises(ValueError):telemetry.ReadonlyBridge(url)

    def test_ground_requires_real_simulation_unarmed_landed_and_no_running_worker(self):
        telemetry.require_ground(state())
        for field,value in (('simulation_only',False),('armed',True),('landed_state',2),('mission',{'status':'running'})):
            bad=state();bad[field]=value
            with self.assertRaises(RuntimeError):telemetry.require_ground(bad)

    def test_complete_actual_tool_and_exact_reply_pass(self):
        client=telemetry.ReadonlyBridge()
        result=telemetry.validate_messages(messages(),client)
        self.assertTrue(result['passed'])
        self.assertIs(result['parsed_reply']['used_for_flight_control'],True)

    def test_independent_vio_false_must_not_replace_external_vision_true(self):
        value=messages();reply=json.loads(value[-1]['content']);reply['used_for_flight_control']=False
        value[-1]['content']=json.dumps(reply)
        self.assertFalse(telemetry.validate_messages(value,telemetry.ReadonlyBridge())['passed'])

    def test_missing_external_vision_requires_null_not_independent_vio_false(self):
        value=messages();actual=json.loads(value[1]['content']);del actual['external_vision']
        value[1]['content']=json.dumps(actual)
        reply=json.loads(value[-1]['content']);reply['used_for_flight_control']=False
        value[-1]['content']=json.dumps(reply)
        self.assertFalse(telemetry.validate_messages(value,telemetry.ReadonlyBridge())['passed'])
        reply['used_for_flight_control']=None;value[-1]['content']=json.dumps(reply)
        self.assertTrue(telemetry.validate_messages(value,telemetry.ReadonlyBridge())['passed'])

    def test_no_tool_duplicate_tool_or_model_claim_without_real_result_fails(self):
        for value in ([messages()[-1]],messages()+[messages()[1]],messages()[:1]+messages()[2:]):
            self.assertFalse(telemetry.validate_messages(value,telemetry.ReadonlyBridge())['passed'])

    def test_schema_fabrications_or_unrequested_blocked_action_fail(self):
        value=messages();value[0]['tool_calls'][0]['args']['example_parameter_1']=''
        self.assertFalse(telemetry.validate_messages(value,telemetry.ReadonlyBridge())['passed'])
        client=telemetry.ReadonlyBridge();client.blocked.append({'operation':'takeoff','blocked_before_transport':True})
        self.assertFalse(telemetry.validate_messages(messages(),client)['passed'])

    def test_boolean_fields_cannot_be_substituted_by_numbers_or_strings(self):
        for field,value in (('armed',0),('landed_state',True),('vio_ready','true'),('used_for_flight_control',1)):
            msg=messages();reply=json.loads(msg[-1]['content']);reply[field]=value;msg[-1]['content']=json.dumps(reply)
            self.assertFalse(telemetry.validate_messages(msg,telemetry.ReadonlyBridge())['passed'])

    def test_credential_and_url_userinfo_path_redacted_recursively(self):
        raw={'error':'Bearer very-secret https://user:pass@example.com/private?token=abc',
             'nested':[{'key':'very-secret'}]}
        text=json.dumps(telemetry.sanitize(raw,['very-secret']))
        for forbidden in ('very-secret','user:pass','/private','token=abc'):
            self.assertNotIn(forbidden,text)
        self.assertIn('https://example.com',text)

    def test_default_plan_has_zero_model_bridge_or_write_calls(self):
        with mock.patch.object(telemetry.BridgeClient,'request',side_effect=AssertionError('HTTP forbidden')), \
                mock.patch.object(Path,'mkdir',side_effect=AssertionError('write forbidden')), \
                contextlib.redirect_stdout(io.StringIO()) as capture:
            self.assertEqual(telemetry.main([]),0)
        result=json.loads(capture.getvalue())
        self.assertFalse(result['actual_provider_calls']);self.assertEqual(result['automatic_retries'],0)


if __name__=='__main__':unittest.main()
