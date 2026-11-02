"""Transport failures must be distinguished from provider authentication safely."""
import json
import socket
import ssl
import unittest

from vision import safe_error


class APIConnectionError(Exception):
    pass


def wrapped(inner):
    error = APIConnectionError('outer failure')
    error.__cause__ = inner
    return error


class ModelErrorStageTests(unittest.TestCase):
    def test_tls_eof_chain_is_not_reported_as_an_api_key_error(self):
        inner = Exception('[SSL: UNEXPECTED_EOF_WHILE_READING] secret-key data:image/jpeg;base64,payload')
        error = wrapped(wrapped(inner))
        result = safe_error(error, {'api_key': 'secret-key'})
        self.assertEqual(result['failure_stage'], 'tls_connection')
        self.assertIn('TLS', result['error'])
        self.assertIsNone(result['http_status'])
        for secret in ('secret-key', 'payload'):
            self.assertNotIn(secret, json.dumps(result))

    def test_http_authentication_status_takes_precedence(self):
        error = wrapped(ssl.SSLError('old connection detail'))
        error.status_code = 401
        result = safe_error(error, {})
        self.assertEqual(result['failure_stage'], 'http_response')
        self.assertIn('认证失败', result['error'])
        self.assertIn('HTTP 401', result['error'])

    def test_network_stages_are_specific_and_do_not_echo_exception_text(self):
        for inner, stage in [(socket.gaierror('private-host'), 'dns'),
                             (ConnectionRefusedError('private-proxy'), 'tcp_connection_refused'),
                             (TimeoutError('private-request'), 'timeout'),
                             (ssl.SSLCertVerificationError('private-cert'), 'tls_certificate')]:
            with self.subTest(stage=stage):
                result = safe_error(wrapped(inner), {})
                self.assertEqual(result['failure_stage'], stage)
                self.assertNotIn('private-', json.dumps(result))

    def test_cyclic_chain_is_bounded_and_unknown_error_is_preserved(self):
        error = APIConnectionError('private')
        error.__cause__ = error
        self.assertEqual(safe_error(error, {})['failure_stage'], 'connection')
        result = safe_error(ValueError('bad visual bbox'), {})
        self.assertIsNone(result['failure_stage'])
        self.assertEqual(result['error'], 'bad visual bbox')

    def test_proxy_failure_is_distinct_from_authentication(self):
        proxy_error = type('ProxyError', (Exception,), {})('proxy-token')
        result = safe_error(wrapped(proxy_error), {})
        self.assertEqual(result['failure_stage'], 'proxy_connection')
        self.assertNotIn('proxy-token', json.dumps(result))

    def test_plain_validation_error_text_does_not_become_a_transport_diagnosis(self):
        result = safe_error(ValueError('bad observation UNEXPECTED_EOF_WHILE_READING'), {})
        self.assertIsNone(result['failure_stage'])
