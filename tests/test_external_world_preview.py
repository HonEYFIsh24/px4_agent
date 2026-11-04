import hashlib
import json
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

import preview_external_world as preview


class ExternalWorldPreviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def put(self, name, text):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        return path

    def source(self):
        world = self.put('source/original.world', '<sdf version="1.6"><world name="default">'
                         '<state/><plugin filename="libJoyPlugin.so"/>'
                         '<model name="building"><static>true</static></model></world></sdf>')
        self.put('source/.source_revision.json', json.dumps({'commit': 'revision-1'}))
        return world, {'directory': 'source', 'commit': 'revision-1', 'world': 'original.world',
                       'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
                       'omit_preview_plugins': ['libJoyPlugin.so']}

    def test_prepare_preserves_upstream_and_records_preview_changes(self):
        original, spec = self.source()
        before = original.read_bytes()
        with patch.object(preview, 'ASSETS', self.root):
            result = preview.prepare('example', spec)
        self.assertEqual(original.read_bytes(), before)
        self.assertNotIn(b'<plugin', result.read_bytes())
        self.assertNotIn(b'<state', result.read_bytes())
        self.assertIn(b'building', result.read_bytes())
        audit = json.loads((result.parent / 'provenance.json').read_text())
        self.assertFalse(audit['flight_approved'])
        self.assertEqual(len(audit['changes']), 2)

    def test_changed_source_is_rejected(self):
        original, spec = self.source()
        original.write_text(original.read_text() + ' ')
        with patch.object(preview, 'ASSETS', self.root), self.assertRaisesRegex(RuntimeError, 'changed'):
            preview.prepare('example', spec)

    def test_wrong_revision_is_rejected(self):
        _, spec = self.source()
        spec['commit'] = 'another-revision'
        with patch.object(preview, 'ASSETS', self.root), self.assertRaisesRegex(RuntimeError, 'revision mismatch'):
            preview.prepare('example', spec)

    def test_unreviewed_world_plugin_is_not_executed(self):
        _, spec = self.source()
        spec['omit_preview_plugins'] = []
        with patch.object(preview, 'ASSETS', self.root), self.assertRaisesRegex(RuntimeError, 'unreviewed'):
            preview.prepare('example', spec)

    def test_nested_model_dependencies_and_plugins_are_checked(self):
        world = self.put('world.world', '<sdf><world><include><uri>model://outer</uri></include></world></sdf>')
        self.put('models/outer/model.config', '<model><sdf>model.sdf</sdf></model>')
        self.put('models/outer/model.sdf', '<sdf><model><plugin filename="unknown.so"/>'
                 '<include><uri>model://absent</uri></include></model></sdf>')
        result = preview.check_resources(world, [self.root / 'models'])
        self.assertEqual(result['missing_resources'], ['model://absent'])
        self.assertEqual(result['unreviewed_plugins'], ['unknown.so'])

    def test_heightmap_texture_and_resource_search_paths(self):
        world = self.put('world.world', '<sdf><world><model><visual><geometry><heightmap>'
            '<texture><diffuse>model://terrain/missing.jpg</diffuse>'
            '<normal>file://media/normal.png</normal></texture>'
            '</heightmap></geometry></visual></model></world></sdf>')
        self.put('resources/media/normal.png', 'fixture')
        result = preview.check_resources(world, [self.root / 'models'], [self.root / 'resources'])
        self.assertEqual(result['missing_resources'], ['model://terrain/missing.jpg'])

    def test_busy_port_does_not_start_or_stop_any_process(self):
        with socket.socket() as bound:
            bound.bind(('127.0.0.1', 0))
            port = bound.getsockname()[1]
            with patch.object(preview.subprocess, 'Popen') as popen, self.assertRaises(OSError):
                preview.preview(self.root / 'unused', [], [], 'example', port, 1)
            popen.assert_not_called()

    def test_client_start_failure_cleans_up_owned_server(self):
        server = Mock()
        server.poll.return_value = None
        with socket.socket() as bound:
            bound.bind(('127.0.0.1', 0))
            port = bound.getsockname()[1]
        world = self.put('world.world', '<sdf/>')
        with patch.object(preview, 'ROOT', self.root), \
                patch.object(preview.socket, 'create_connection'), \
                patch.object(preview.subprocess, 'Popen', side_effect=[server, OSError('client failed')]), \
                patch.object(preview, 'write_json'):
            with self.assertRaisesRegex(OSError, 'client failed'):
                preview.preview(world, [], [], 'example', port, 1)
        server.send_signal.assert_called_once_with(signal.SIGINT)
        server.wait.assert_called_once()

    def test_unresponsive_owned_process_has_bounded_cleanup(self):
        process = Mock()
        process.poll.return_value = None
        process.wait.side_effect = [subprocess.TimeoutExpired('owned', 8),
                                    subprocess.TimeoutExpired('owned', 5), 0]
        preview.stop(process, signal.SIGTERM)
        process.terminate.assert_called_once()
        process.kill.assert_called_once()


if __name__ == '__main__':
    unittest.main()
