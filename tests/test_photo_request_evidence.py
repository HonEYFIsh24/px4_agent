"""SDK evidence is opt-in, image-referenced, private and behavior-preserving."""
import base64
import hashlib
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage
from langchain_openai import ChatOpenAI
import photo_request_evidence as module
from test_vision import JPEG_B64


class RequestEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        patch=mock.patch.object(module,'_ROOT',self.root);patch.start();self.addCleanup(patch.stop)
        self.directory=self.root/'logs'/'requests'
        env=mock.patch.dict(os.environ,{'UAV_PHOTO_REQUEST_EVIDENCE_DIR':str(self.directory)})
        env.start();self.addCleanup(env.stop)
        self.config=dict(api_key='private-test-key',model='qwen3.7-plus',temperature=0,max_retries=0)
        self.model=module.EvidenceChatOpenAI(**self.config)
        self.messages=[SystemMessage('system'),HumanMessage(content=[
            dict(type='text',text='current photo'),
            dict(type='image_url',image_url=dict(url='data:image/jpeg;base64,'+JPEG_B64))])]

    def scope(self, enabled=True):
        return module.request_evidence(enabled,('thread','owner'),'capture',self.config)

    def test_payload_preserved_and_reconstructable_from_hash_verified_image(self):
        parent=ChatOpenAI(**self.config)._get_request_payload(self.messages,parallel_tool_calls=False)
        with self.scope() as state:
            payload=self.model._get_request_payload(self.messages,parallel_tool_calls=False)
            module.record_response(state,[AIMessage('choice',tool_calls=[dict(name='rotate_photo_view',args={'direction':'right','angle_deg':15},id='call')])])
        self.assertEqual(payload,parent)
        request=next(self.directory.glob('*.request.json')); raw=request.read_text();record=json.loads(raw)
        self.assertNotIn(JPEG_B64,raw);self.assertNotIn(self.config['api_key'],raw)
        self.assertTrue(record['exact_payload_reconstructable'])
        self.assertFalse(record['flight_execution_authority'])
        url=record['payload']['messages'][-1]['content'][-1]['image_url']['url']
        digest=hashlib.sha256(base64.b64decode(JPEG_B64)).hexdigest()
        self.assertEqual(url,'local-image-sha256:'+digest)
        record['payload']['messages'][-1]['content'][-1]['image_url']['url']='data:image/jpeg;base64,'+JPEG_B64
        self.assertEqual(record['payload'],parent)
        self.assertEqual(hashlib.sha256(module.canonical(parent).encode()).hexdigest(),record['payload_sha256'])
        self.assertEqual(request.stat().st_mode & 0o777,0o600)
        response=json.loads(next(self.directory.glob('*.response.json')).read_text())
        self.assertEqual(response['request_id'],record['id']);self.assertFalse(response['failed'])
        self.assertEqual(state['status'],'recorded')

    def test_disabled_nested_or_outside_scope_records_nothing(self):
        self.model._get_request_payload(self.messages)
        with self.scope(False):self.model._get_request_payload(self.messages)
        with self.scope():
            with self.scope(False):self.model._get_request_payload(self.messages)
        self.assertFalse(self.directory.exists())

    def test_sensitive_text_and_tokens_redacted_without_altering_outbound_payload(self):
        messages=[HumanMessage('private-test-key Bearer another-secret https://user:password@host/path '+
                              '{"ticket":"serialized-ticket", "api_key":"serialized-key"}')]
        with self.scope():
            payload=self.model._get_request_payload(messages,extra_body={'ticket':'live-ticket','api_key':'other-key'})
        self.assertIn('private-test-key',payload['messages'][0]['content'])
        record=json.loads(next(self.directory.glob('*.request.json')).read_text());text=json.dumps(record)
        for secret in ('private-test-key','another-secret','user:password','live-ticket','other-key','serialized-ticket','serialized-key'):
            self.assertNotIn(secret,text)
        self.assertFalse(record['exact_payload_reconstructable'])

    def test_disk_failure_or_unsupported_directory_does_not_change_model_parameters(self):
        parent=ChatOpenAI(**self.config)._get_request_payload(self.messages)
        with self.scope() as state,mock.patch.object(module,'write_private',side_effect=OSError('full')):
            self.assertEqual(self.model._get_request_payload(self.messages),parent)
            module.record_response(state,[AIMessage('answer')])
        self.assertEqual(state['status'],'unavailable')
        with mock.patch.dict(os.environ,{'UAV_PHOTO_REQUEST_EVIDENCE_DIR':str(self.root/'outside')}),self.scope() as state:
            self.assertEqual(self.model._get_request_payload(self.messages),parent)
        self.assertEqual(state['status'],'unavailable');self.assertFalse(self.directory.exists())

    def test_failed_provider_response_is_recorded_without_exception_or_credentials(self):
        with self.scope() as state:
            self.model._get_request_payload(self.messages)
            module.record_response(state,failed=True)
        response=json.loads(next(self.directory.glob('*.response.json')).read_text())
        self.assertTrue(response['failed']);self.assertEqual(response['response'],[])

    def test_real_agent_middleware_records_only_photo_selection_and_pairs_response(self):
        from typing import Any
        from test_photo_task import PhotoFlightClient, PROMPT, review
        from test_single_vlm import LocalScriptModel, capture_request, report_request, tool_call
        from single_vlm import FrameStore, build_single_vlm_agent
        from agent import flight_tools
        from test_vision import CONFIG
        class PayloadScriptModel(LocalScriptModel):
            recorder: Any
            def _generate(self,messages,stop=None,run_manager=None,**kwargs):
                self.recorder._get_request_payload(messages,stop=stop,**kwargs)
                return super()._generate(messages,stop=stop,run_manager=run_manager,**kwargs)
        client=PhotoFlightClient(armed=True);store=FrameStore(client,CONFIG)
        model=PayloadScriptModel(recorder=self.model,script=[capture_request,report_request,
            review('no','yes','调整'),tool_call('rotate_photo_view',dict(direction='right',angle_deg=15.,reason='fixture')),
            capture_request,report_request,review(),AIMessage('done')])
        graph=build_single_vlm_agent(client,CONFIG,flight_tools(client),'',model=model,store=store)
        graph.invoke({'messages':[HumanMessage(PROMPT,id='photo')]},
                     {'configurable':{'thread_id':'trace'},'recursion_limit':60,'max_concurrency':1})
        files=list(self.directory.glob('*.request.json'));self.assertEqual(len(files),1)
        record=json.loads(files[0].read_text())
        self.assertEqual(record['owner'],['trace','photo'])
        self.assertEqual(record['capture_id'],store.photo_tasks[('trace','photo')].observations[0]['capture_id'])
        self.assertEqual(len(record['images']),1)
        response=json.loads(next(self.directory.glob('*.response.json')).read_text())
        self.assertEqual(response['response'][0]['tool_calls'][0]['name'],'rotate_photo_view')
        traces=[c for c in store.calls(('trace','photo')) if c['image_purpose']=='photo_view_selection']
        self.assertEqual(traces[0]['context_memory']['photo_request_evidence'][0]['status'],'recorded')
        self.assertEqual(client.calls,['/navigation/photo-action'])


if __name__=='__main__':unittest.main()
