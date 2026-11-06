"""NBV transport must obey the same command gates as ordinary flight."""
import copy
import threading
import unittest
from types import SimpleNamespace

from web_server import AuditedClient


class Transport:
    def __init__(self):
        self.calls = []
        self.fail = False

    def request(self, path, payload=None, **kwargs):
        self.calls.append((path, copy.deepcopy(payload)))
        if self.fail:
            raise OSError('controlled transport failure')
        return {'mission': {'id': 'nbv-flight', 'status': 'running',
                            'steps': [{'action': 'goto', 'x': 2., 'y': 0., 'altitude': 3.}]}}

    def wait_for(self, accepted, on_progress=None):
        return {'mission': {**accepted['mission'], 'status': 'succeeded'}}


class NBVAuditTests(unittest.TestCase):
    def setUp(self):
        self.transport = Transport()
        self.command = {'id': 'task', 'prompt': '寻找前方目标，报告后返航降落',
                        'mode': 'task', 'interrupted': False, 'vision_failure': None,
                        'flight_failure': None, 'tool_results': []}
        self.app = SimpleNamespace(client=self.transport, current_command=self.command,
            lock=threading.RLock(), control_lock=threading.RLock(),
            clean_error=str, clean_perception=copy.deepcopy)
        self.client = AuditedClient(self.app)

    def execute(self):
        return self.client.execute_viewpoint(('thread', 'task'), 'a'*32, 'b'*64,
                                             'view_001', 'c'*32)

    def test_observation_interrupt_and_prior_failures_block_ticket_before_transport(self):
        for field, value, message in [('mode', 'observe', '仅允许视觉观察'),
                                     ('interrupted', True, '操作员已中断'),
                                     ('vision_failure', 'bad image', '视觉分析已失败'),
                                     ('flight_failure', 'lost position', '禁止自动重试')]:
            with self.subTest(field=field):
                self.setUp()
                self.command[field] = value
                with self.assertRaisesRegex(RuntimeError, message):
                    self.execute()
                self.assertEqual(self.transport.calls, [])

    def test_accepted_and_completed_ticket_share_existing_mission_audit_contract(self):
        result = self.execute()
        self.assertEqual(result['mission']['status'], 'succeeded')
        self.assertEqual([name for name, _ in self.command['tool_results']],
                         ['/mission', 'execute_mission'])
        self.assertEqual(self.command['flight_missions'][0]['mission']['id'], 'nbv-flight')
        accepted = self.command['tool_results'][0][1]
        self.assertEqual(accepted['accepted_via'], '/navigation/execute-viewpoint')
        self.assertNotIn('ticket', accepted)
        self.assertNotIn('x', self.transport.calls[0][1])
        self.assertEqual(len(self.command['flight_mission_results']), 1)
        self.assertEqual(self.command['flight_mission_results'][0]['mission']['status'], 'succeeded')
        self.assertEqual(self.command['flight_missions'][0]['accepted_via'], '/navigation/execute-viewpoint')

    def test_repeated_tool_audits_do_not_duplicate_owned_completed_plans(self):
        result = self.execute()
        self.client.record('execute_mission', result)
        self.assertEqual(len(self.command['flight_mission_results']), 1)
        foreign = copy.deepcopy(result)
        foreign['mission']['id'] = 'foreign-flight'
        self.client.record('execute_mission', foreign)
        self.assertEqual(len(self.command['flight_mission_results']), 1)

    def test_ambiguous_ticket_transport_failure_blocks_retries_but_allows_landing(self):
        self.transport.fail = True
        with self.assertRaises(OSError):
            self.execute()
        self.assertTrue(self.command['flight_failure'])
        self.transport.fail = False
        with self.assertRaisesRegex(RuntimeError, '禁止自动重试'):
            self.execute()
        self.client.request('/land', {})
        self.assertEqual([path for path, _ in self.transport.calls],
                         ['/navigation/execute-viewpoint', '/land'])

    def test_read_only_prediction_failure_does_not_claim_a_flight_was_attempted(self):
        self.transport.fail = True
        with self.assertRaises(OSError):
            self.client.request('/navigation/viewpoints', {'operation': 'context'}, timeout=30.)
        self.assertIsNone(self.command['flight_failure'])
        self.assertEqual(self.command['tool_results'], [])


if __name__ == '__main__':
    unittest.main()
