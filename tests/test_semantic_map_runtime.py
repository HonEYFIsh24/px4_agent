"""Passive runtime and cached geometry provenance integration."""
import copy
import unittest
from unittest import mock
from semantic_map_runtime import SemanticMapRuntime
from test_semantic_map import navigator, observation, OWNER, EPOCH
from autonomous_search import AdaptiveSearch
from test_autonomous_search import AdaptiveBridge


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.n = navigator()
        self.frame, self.report, self.geometry = observation(self.n)
        self.client = mock.Mock()
        self.state = {'scene': {'world_sha256': self.n.world_sha256},
            'navigation': {'observation_geometry_supported': True, 'registration_id': self.n.registration_id},
            'external_vision': {'adapter': {'epoch': 0}, 'ekf': {'local_position_reset_counters': EPOCH['ekf_counters']}}}
        self.client.state.return_value = self.state
        self.client.request.return_value = self.geometry
        self.runtime = SemanticMapRuntime(self.client)
        self.frame['snapshot']['navigation_registration_id'] = self.n.registration_id
        self.session = {'origin': [0., 0., 3.], 'query': '油桶', 'epoch_bound': True,
            'epoch': copy.deepcopy(EPOCH), 'scene': self.n.world_sha256,
            'binding': {'registration_id': self.n.registration_id, 'stamp_sim_s': 0.}}

    def test_saved_report_updates_map_and_exports_replay_evidence(self):
        import tempfile, os, json
        from pathlib import Path
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR': tmp}):
            result = self.runtime.observe(OWNER, self.frame, self.report, self.session)
            self.assertEqual(result['observation_count'], 1)
            self.assertTrue(result['passive_only'])
            self.client.request.assert_called_once_with('/navigation/observation-geometry',
                {'pair_id': 'capture-1-pair', 'origin_xy_enu_m': [0., 0.]})
            exported = json.loads((Path(tmp)/'capture-1.geometry.json').read_text())
            self.assertEqual(exported, self.geometry)
            self.assertEqual(self.session['spatial_map_identity']['status'], 'valid')
            self.assertNotIn('cells', self.session)

    def test_planning_revision_is_immutable_and_does_not_replace_capture_map(self):
        import tempfile, os, json
        from pathlib import Path
        from viewpoint_planner import map_revision
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(os.environ, {'UAV_CAPTURE_EVIDENCE_DIR': tmp}):
            self.runtime.observe(OWNER, self.frame, self.report, self.session)
            acquired = Path(tmp)/'capture-1.semantic-map.json'
            original = acquired.read_bytes()
            before = self.runtime.checkpoint(OWNER)
            self.runtime.maps[OWNER].set_query('红色集装箱')
            after = self.runtime.checkpoint(OWNER)
            self.assertNotEqual(before, after)
            self.assertEqual(acquired.read_bytes(), original)
            self.assertEqual(after, map_revision(self.runtime.maps[OWNER]))
            version = Path(tmp)/('map-'+after+'.snapshot.json')
            self.assertEqual(json.loads(version.read_text())['query'], '红色集装箱')
            version.write_text('{}')
            with self.assertRaisesRegex(ValueError, 'modified'):self.runtime.checkpoint(OWNER)

    def test_changed_registration_invalidates_and_blocks_new_search_moves(self):
        self.runtime.observe(OWNER, self.frame, self.report, self.session)
        self.state['navigation']['registration_id'] = 'new-sortie'
        result = self.runtime.observe(OWNER, self.frame, self.report, self.session)
        self.assertEqual(result['status'], 'invalid')
        bridge = AdaptiveBridge()
        search = AdaptiveSearch(bridge)
        search.begin(OWNER, '油桶')
        search.sessions[OWNER]['spatial_map_identity'] = self.session['spatial_map_identity']
        with self.assertRaisesRegex(ValueError, '地图坐标身份失效'):
            search.validate_state(OWNER, bridge.state())
        # A model call cannot acquire operator/safety authority through land.
        with self.assertRaises(ValueError):
            search.check_mission(OWNER, [{'action': 'land'}])

    def test_unsupported_backend_does_not_request_or_change_session(self):
        self.state['navigation']['observation_geometry_supported'] = False
        self.assertEqual(self.runtime.observe(OWNER, self.frame, self.report, self.session)['status'], 'unavailable')
        self.client.request.assert_not_called()
        self.assertNotIn('spatial_map_identity', self.session)

    def test_unbound_or_prebinding_image_cannot_seed_map(self):
        for field, value in (('epoch_bound', False), ('binding', {
                'registration_id': self.n.registration_id, 'stamp_sim_s': 2.})):
            session = copy.deepcopy(self.session);session[field] = value
            with self.assertRaisesRegex(ValueError, '绑定'):
                self.runtime.observe(OWNER, self.frame, self.report, session)
            self.assertEqual(self.runtime.maps, {})
        self.client.request.assert_not_called()

    def test_pose_or_calibration_mismatch_does_not_commit_observation(self):
        for key in ('body_pose_enu', 'camera_intrinsics'):
            geometry = copy.deepcopy(self.geometry)
            geometry[key] = {}
            self.client.request.return_value = geometry
            runtime = SemanticMapRuntime(self.client)
            with self.assertRaisesRegex(ValueError, 'another actual image'):
                runtime.observe(OWNER, self.frame, self.report, self.session)
            self.assertEqual(runtime.maps[OWNER].get_context()['observation_count'], 0)
