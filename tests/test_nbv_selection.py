"""Selection freshness on synthetic camera evidence and real local geometry; no network."""
import unittest

from autonomous_search import AdaptiveSearch
from semantic_map_runtime import SemanticMapRuntime
from single_vlm import FrameStore
from test_nbv_search_integration import ScopedFixtureBridge
from test_vision import CONFIG

OWNER = ('selection-test', 'task')


class SelectionTests(unittest.TestCase):
    def setUp(self):
        self.client = ScopedFixtureBridge()
        self.client.execute = lambda steps, description='': self.client.wait_for(
            self.client.request('/mission', {'steps': steps, 'description': description}))
        self.client.execute_viewpoint = lambda owner, batch_id, revision, candidate_id, ticket, description='': self.client.wait_for(
            self.client.request('/navigation/execute-viewpoint', {'owner': list(owner), 'batch_id': batch_id,
                'map_revision': revision, 'candidate_id': candidate_id, 'ticket': ticket, 'description': description}))
        self.maps = SemanticMapRuntime(self.client)
        self.now = 0.
        self.search = AdaptiveSearch(self.client, clock=lambda: self.now,
            search_mode='nbv_geometry', semantic_maps=self.maps, nbv_selector='vlm')
        self.search.begin(OWNER, '蓝色油桶')
        self.search.execute_pending(OWNER)
        store = FrameStore(self.client, CONFIG)
        captured = store.capture(OWNER, '蓝色油桶')
        # Explicit synthetic delivery fixture, never provider/recognition evidence.
        store.mark_delivered(OWNER, captured['capture_id'], {})
        frame = store.get(OWNER, captured['capture_id'])
        report = {'status': 'ok', 'capture_id': captured['capture_id'], 'summary': 'synthetic fixture', 'targets': []}
        self.search.consume(OWNER, frame, report)
        self.maps.observe(OWNER, frame, report, self.search.sessions[OWNER])
        self.offered = self.search.next_view(OWNER, 'auto', 'fixture')

    def select(self):
        return self.search.select_nbv(OWNER, self.offered['batch_id'], self.offered['candidates'][0]['id'], 'fixture')

    def test_repeated_next_does_not_refresh_ticket_batch_or_move_before_choice(self):
        tickets = set(self.client.authority.tickets)
        again = self.search.next_view(OWNER, 'rise', 'do not reshuffle')
        self.assertEqual(again['batch_id'], self.offered['batch_id'])
        self.assertEqual(set(self.client.authority.tickets), tickets)
        with self.assertRaisesRegex(ValueError, '先选择'):
            self.search.execute_pending(OWNER)
        self.assertEqual(self.client.ticket_executions, 0)

    def test_expired_choice_cannot_select_or_renew_batch(self):
        self.now = 60.001
        with self.assertRaisesRegex(ValueError, '超时'): self.select()
        with self.assertRaisesRegex(ValueError, '超时'): self.search.next_view(OWNER, 'auto', 'renew')
        self.assertIsNone(self.search.sessions[OWNER]['pending'])

    def test_map_update_prevents_old_choice(self):
        self.maps.maps[OWNER].cells[0]['semantic_score'] += .1
        with self.assertRaisesRegex(ValueError, '地图已经更新'): self.select()
        self.assertEqual(self.client.ticket_executions, 0)

    def test_epoch_change_prevents_selection(self):
        self.client.telemetry['external_vision']['adapter']['epoch'] += 1
        with self.assertRaises(ValueError): self.select()
        self.assertIsNone(self.search.sessions[OWNER]['pending'])

    def test_source_motion_and_sortie_changes_prevent_selection(self):
        self.client.telemetry['position_enu_m'][0] += .36
        with self.assertRaisesRegex(ValueError, '源位置或架次'): self.select()
        self.client.telemetry['position_enu_m'][0] -= .36
        self.client.nav.registration_id = 'new-sortie'
        with self.assertRaisesRegex(ValueError, '源位置或架次'): self.select()

    def test_one_choice_cannot_be_changed_or_replayed(self):
        selected = self.select()
        self.assertEqual(selected['pending']['candidate_id'], self.offered['candidates'][0]['id'])
        with self.assertRaisesRegex(ValueError, '没有等待'): self.select()
        self.search.execute_pending(OWNER)
        self.assertEqual(self.client.ticket_executions, 1)


if __name__ == '__main__':
    unittest.main()
