import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from search_diagnostics import stage
from viewpoint_geometry import RemoteViewpointEvaluator


class SearchDiagnosticsTests(unittest.TestCase):
    def test_exact_query_response_is_recorded_without_mutation_or_reusable_ticket(self):
        response = {'results': [{'execution_ticket': 'live-authority', 'position_enu_m': [1., 2., 3.]}]}
        client = mock.Mock();client.request.return_value = response
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR': directory}):
            backend = RemoteViewpointEvaluator(client, 'actual-pair', {'owner': ['thread','turn']}, [[1., 2., 0.]])
            value = backend.query('paths', [0., 0.], {'epoch': 3}, [{'id': 'view_1', 'position_enu_m': [1., 2., 3.]}])
            records = [json.loads(line) for line in next(Path(directory).glob('*.jsonl')).read_text().splitlines()]
            self.assertIs(value, response)
            self.assertEqual(value['results'][0]['execution_ticket'], 'live-authority')
            self.assertEqual(records[0]['inputs'], client.request.call_args.args[1])
            self.assertEqual(records[1]['result']['results'][0]['execution_ticket'], '[redacted-ticket]')
            self.assertEqual(records[0]['id'], records[1]['id'])

    def test_original_failure_is_preserved_with_callsite_stack(self):
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR': directory}):
            with self.assertRaises(TypeError):
                with stage('confirmation_fixture', {'shape': 'scalar'}):
                    list(1.5)
            records = [json.loads(line) for line in next(Path(directory).glob('*.jsonl')).read_text().splitlines()]
            self.assertEqual(records[-1]['exception_type'], 'TypeError')
            self.assertEqual(records[-1]['frames'][-1]['module'], 'test_search_diagnostics.py')
            self.assertTrue(all(set(f) == {'module','function','line'} for f in records[-1]['frames']))

    def test_diagnostic_io_failure_cannot_mask_original_error(self):
        with mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR':'/nonexistent/diagnostic/path'}):
            with self.assertRaisesRegex(RuntimeError, 'original safety failure'):
                with stage('fixture', {}):
                    raise RuntimeError('original safety failure')

    def test_replay_uses_exact_archived_queries_and_rejects_modified_input(self):
        from dataclasses import asdict
        from semantic_map import SemanticMap
        from viewpoint_geometry import StaticViewpointEvaluator
        from viewpoint_planner import ViewpointConfig, ViewpointPlanner
        from test_semantic_map import navigator, observation, OWNER
        from scripts.replay_nbv_diagnostics import replay
        nav = navigator(); frame, report, geometry = observation(nav)
        m = SemanticMap(OWNER, geometry); m.update_observation(frame, report, geometry)
        snapshot = frame['snapshot']
        local = StaticViewpointEvaluator(nav, snapshot['body_pose_enu'], snapshot['camera_intrinsics'], snapshot['coordinate_epoch'])
        client = mock.Mock()
        client.request.side_effect = lambda path, p, timeout: local.query(p['operation'], p['origin_xy_enu_m'], p['expected_identity'], p['candidates'])
        planner = ViewpointPlanner(ViewpointConfig(mode='geometry'))
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR': directory}), \
                mock.patch('socket.create_connection', side_effect=AssertionError('replay must remain offline')):
            with stage('planner_evaluate', {'context': {'batch_id':'fixture'}, 'config':asdict(planner.config),
                        'semantic_map':m.snapshot(), 'confirmation_goal':None}) as diagnostic:
                diagnostic['result'] = planner.evaluate(m, RemoteViewpointEvaluator(client, 'fixture-pair'))
            path = next(Path(directory).glob('*.jsonl'))
            # Disable new recording while replaying the immutable archived batch.
            with mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR':''}):
                result = replay(path)
                self.assertEqual(result['replay_outcome'], 'completed', result)
                self.assertTrue(result['matches_recorded_prediction'])
                self.assertTrue(result['all_recorded_queries_used'])
                events = [json.loads(x) for x in path.read_text().splitlines()]
                next(e for e in events if e['stage']=='remote_viewpoints' and e['event']=='begin')['inputs']['origin_xy_enu_m']=[1.,0.]
                modified = Path(directory)/'modified.jsonl'
                modified.write_text('\n'.join(json.dumps(e) for e in events))
                rejected = replay(modified)
                self.assertEqual(rejected['replay_outcome'], 'error')
                self.assertIn('differs', rejected['message'])
