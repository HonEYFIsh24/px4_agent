import copy
import unittest

from audit_confirmation_retry import audit_record


class RetryAuditTests(unittest.TestCase):
    def setUp(self):
        self.events = [
            {'outcome': 'unconfirmed', 'source_supported': False, 'source_uncertain': True,
             'current_supported': False, 'candidate': {'source_capture_id': 'a', 'current_capture_id': 'b'}},
            {'outcome': 'candidate', 'candidate': {'source_capture_id': 'a', 'review_attempts': 1}},
            {'outcome': 'unconfirmed', 'candidate': {'source_capture_id': 'a', 'current_capture_id': 'c', 'review_attempts': 1}},
        ]
        self.record = {'command': {'target_confirmation_events': self.events, 'nbv_events': [
            {'source_capture_id': 'b', 'batch_id': 'batch', 'planning_mode': 'confirmation',
             'confirmation_goal': {'source_capture_id': 'a'}}]}}
        self.nbv = {'passed': True, 'moves': [{'source_capture_id': 'b', 'next_capture_id': 'c', 'batch_id': 'batch'}]}
        self.confirmation = {'passed': True}
        self.map = {'observations': [{'capture_id': 'a', 'stamp_sim_s': 1., 'position_enu_m': [0., 0., 3.]},
                                     {'capture_id': 'b', 'stamp_sim_s': 2., 'position_enu_m': [2., 0., 3.]},
                                     {'capture_id': 'c', 'stamp_sim_s': 3., 'position_enu_m': [0., 2., 3.]}]}

    def audit(self, record=None, nbv=None, snapshot=None):
        return audit_record(record or self.record, nbv or self.nbv, self.confirmation, lambda _: snapshot or self.map)

    def test_owned_newer_distinct_reobservation_is_required(self):
        self.assertTrue(self.audit()['passed'])
        for fault in ('missing_retain', 'wrong_source', 'same_image', 'old_image', 'same_position', 'wrong_batch'):
            with self.subTest(fault=fault):
                record, nbv, snapshot = copy.deepcopy(self.record), copy.deepcopy(self.nbv), copy.deepcopy(self.map)
                if fault == 'missing_retain': record['command']['target_confirmation_events'].pop(1)
                if fault == 'wrong_source': record['command']['target_confirmation_events'][2]['candidate']['source_capture_id'] = 'other'
                if fault == 'same_image': record['command']['target_confirmation_events'][2]['candidate']['current_capture_id'] = 'b'
                if fault == 'old_image': snapshot['observations'][2]['stamp_sim_s'] = 1.5
                if fault == 'same_position': snapshot['observations'][2]['position_enu_m'] = [2., 0., 3.]
                if fault == 'wrong_batch': nbv['moves'][0]['batch_id'] = 'another'
                self.assertFalse(self.audit(record, nbv, snapshot)['passed'])


if __name__ == '__main__': unittest.main()
