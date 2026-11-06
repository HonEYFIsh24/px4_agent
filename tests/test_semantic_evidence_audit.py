"""Controlled temporary archives test audit failures; not real-flight acceptance."""
import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from scripts.audit_semantic_map_evidence import audit
from semantic_map import SemanticMap, SemanticMapError
from test_semantic_map import navigator, observation, target, OWNER


class EvidenceAuditTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.captures = self.root / 'captures'
        self.captures.mkdir()
        self.frame, self.report, self.geometry = observation(navigator())
        self.identifier = self.frame['capture_id']
        fixture = Path(__file__).parent / 'fixtures'
        hashes = {}
        for side, suffix in (('left', '.jpg'), ('right', '.right.jpg')):
            content = (fixture / ('stereo_' + side + '.jpg')).read_bytes()
            (self.captures / (self.identifier + suffix)).write_bytes(content)
            hashes[side] = hashlib.sha256(content).hexdigest()
        self.frame['snapshot']['stereo_image_sha256'] = hashes
        self.geometry['image_sha256'] = hashes
        self.report['targets'] = [target(self.frame, [5., 1., 1.3])]
        semantic_map = SemanticMap(OWNER, self.geometry, query='蓝色油桶', instruction='寻找红色集装箱附近的蓝色油桶')
        semantic_map.update_observation(self.frame, self.report, self.geometry)
        self.online = semantic_map.snapshot()
        self.record = {'capture_directory': str(self.captures), 'flight': {'armed': False, 'landed_state': 1},
            'command': {'id': OWNER[1], 'status': 'succeeded', 'prompt': '寻找红色集装箱附近的蓝色油桶',
                'perceptions': [self.report], 'semantic_map_events': [dict(semantic_map.get_context(), capture_id=self.identifier)],
                'model_calls': [{'status': 'ok', 'input_modalities': ['text', 'image'], 'capture_ids': [self.identifier]}]}}
        for suffix, data in [('.json', {'snapshot': self.frame['snapshot'], 'query': '蓝色油桶'}),
                             ('.geometry.json', self.geometry), ('.semantic-map.json', self.online)]:
            (self.captures / (self.identifier + suffix)).write_text(json.dumps(data))
        self.result = self.root / 'result.json'
        self.save_record()

    def save_record(self):
        self.result.write_text(json.dumps(self.record))

    def test_staged_pixel_association_replays_the_real_backend_and_original_bytes(self):
        import base64
        m = SemanticMap(OWNER, self.geometry, query='蓝色油桶', instruction=self.record['command']['prompt'])
        m.enable_joint_association()
        self.frame['image_base64'] = base64.b64encode((self.captures/(self.identifier+'.jpg')).read_bytes()).decode()
        m.update_observation(self.frame, self.report, self.geometry)
        self.online = m.snapshot()
        (self.captures/(self.identifier+'.semantic-map.json')).write_text(json.dumps(self.online))
        report = audit(self.result, self.root/'staged-replay')
        self.assertTrue(report['checks']['online_replay_equal'], report['evidence'])
        self.assertEqual(self.online['association_backend'], 'cpu_harris_patch_affine_v1')

    def test_complete_archive_proves_integrity_but_single_view_is_not_multiview(self):
        report = audit(self.result, self.root / 'audit')
        checks = report['checks']
        for key in ('online_replay_equal', 'duplicate_rejected_without_mutation',
                    'replay_has_all_successful_reports', 'explicit_semantic_field_active'):
            self.assertTrue(checks[key], key)
        self.assertFalse(checks['real_multiview_geometric_evidence'])
        self.assertFalse(checks['unlocalized_clues_have_no_xyz_and_have_bearing'])
        self.assertFalse(report['semantic_labels_independently_verified'])

    def test_missing_report_event_does_not_pass_coverage_of_audit_chain(self):
        self.record['command']['semantic_map_events'][0]['capture_id'] = 'another-capture'
        self.save_record()
        report = audit(self.result, self.root / 'wrong-event')
        self.assertFalse(report['checks']['map_events_cover_successful_reports'])

    def test_hash_tamper_or_missing_model_delivery_is_rejected(self):
        path = self.captures / (self.identifier + '.jpg')
        original = path.read_bytes()
        path.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'hash mismatch'):
            audit(self.result, self.root / 'bad-hash')
        path.write_bytes(original)
        self.record['command']['model_calls'] = []
        self.save_record()
        with self.assertRaisesRegex(SemanticMapError, 'owned delivered'):
            audit(self.result, self.root / 'not-delivered')

    def test_missing_geometry_is_uncovered_instead_of_guessed(self):
        (self.captures / (self.identifier + '.geometry.json')).unlink()
        report = audit(self.result, self.root / 'missing-geometry')
        self.assertFalse(report['checks']['replay_has_all_successful_reports'])
        self.assertNotIn('online_replay_equal', report['checks'])

    def test_online_divergence_and_failed_task_are_not_silently_passed(self):
        changed = copy.deepcopy(self.online)
        changed['cells'][0]['coverage'] = .7
        (self.captures / (self.identifier + '.semantic-map.json')).write_text(json.dumps(changed))
        self.record['command']['status'] = 'failed'
        self.record['flight']['armed'] = True
        self.save_record()
        report = audit(self.result, self.root / 'diverged')
        for key in ('online_replay_equal', 'task_succeeded', 'finished_grounded_disarmed'):
            self.assertFalse(report['checks'][key], key)

    def test_only_semantic_score_roundoff_is_tolerated_and_reported(self):
        changed = copy.deepcopy(self.online)
        index = changed['eligible_cell_ids'][0]
        changed['cells'][index]['semantic_score'] += 2e-16
        path = self.captures / (self.identifier + '.semantic-map.json')
        path.write_text(json.dumps(changed))
        report = audit(self.result, self.root / 'roundoff')
        self.assertTrue(report['checks']['online_replay_equal'])
        self.assertFalse(report['evidence']['online_replay_exact_comparison']['cells'])
        self.assertGreater(report['evidence']['semantic_score_max_abs_error'], 0.)
        changed['cells'][index]['semantic_score'] += .01
        path.write_text(json.dumps(changed))
        self.assertFalse(audit(self.result, self.root / 'meaningful-change')['checks']['online_replay_equal'])
