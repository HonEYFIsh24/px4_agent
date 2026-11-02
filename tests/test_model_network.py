import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from model_network import configure_agent_network


class ModelNetworkTests(unittest.TestCase):
    def test_explicit_proxy_overrides_inherited_desktop_socks_fallback(self):
        import httpx
        with tempfile.TemporaryDirectory() as directory:
            env_file = Path(directory)/'.env'
            env_file.write_text('UAV_MODEL_PROXY=http://127.0.0.1:7897\n')
            with patch.dict(os.environ, {'ALL_PROXY': 'socks://127.0.0.1:7897',
                                         'all_proxy': 'socks://127.0.0.1:7897',
                                         'NO_PROXY': 'example.internal'}, clear=True):
                configure_agent_network(env_file)
                for name in ('HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY', 'http_proxy', 'https_proxy', 'all_proxy'):
                    self.assertEqual(os.environ[name], 'http://127.0.0.1:7897')
                self.assertIn('example.internal', os.environ['NO_PROXY'])
                for host in ('localhost', '127.0.0.1', '::1'):
                    self.assertIn(host, os.environ['NO_PROXY'].split(','))
                # Client construction reproduces the desktop failure without
                # sending a request or requiring a SOCKS dependency.
                with httpx.Client():
                    pass

    def test_without_opt_in_inherited_network_configuration_is_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            env_file = Path(directory)/'.env'
            env_file.write_text('')
            with patch.dict(os.environ, {'ALL_PROXY': 'socks://127.0.0.1:7897'}, clear=True):
                before = dict(os.environ)
                configure_agent_network(env_file)
                self.assertEqual(dict(os.environ), before)

    def test_environment_proxy_takes_precedence_over_env_file(self):
        with tempfile.TemporaryDirectory() as directory:
            env_file = Path(directory)/'.env'
            env_file.write_text('UAV_MODEL_PROXY=http://127.0.0.1:7897\n')
            with patch.dict(os.environ, {'UAV_MODEL_PROXY': 'http://127.0.0.1:7999'}, clear=True):
                configure_agent_network(env_file)
                self.assertEqual(os.environ['ALL_PROXY'], 'http://127.0.0.1:7999')
