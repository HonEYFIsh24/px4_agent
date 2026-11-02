import copy
import unittest

from audit_yaw_pose_drop_sitl import audit


class PoseDropAuditTests(unittest.TestCase):
    def setUp(self):
        request = {'id': 'fault1', 'kind': 'pose_drop', 'synthetic': True, 'simulation_only': True,
                   'duration_s': 4., 'issued_at_wall_s': 100.}
        sample = {'fault_id': 'fault1', 'kind': 'pose_drop', 'dropped': True, 'synthetic': True,
                  'simulation_only': True, 'source_topic': '/mavros/local_position/pose',
                  'monotonic_s': 11., 'source_stamp_sim_s': 20.}
        self.record = {'status': 'passed', 'checks': {'capture_blocked_after_restart': True},
            'owned_mission_ids': ['takeoff', 'rotate', 'land'],
            'injection': {'request': request, 'readback': copy.deepcopy(request)},
            'motion_before_fault': {'initial_vio_yaw_rad': 0., 'observed_vio_yaw_rad': .08,
                                    'delta_rad': .08, 'age_seconds': .1},
            'pose_fault_lifecycle': [{'event': 'reliability_started', 'fault_id': 'fault1',
                                      'kind': 'pose_drop', 'monotonic_s': 10., 'duration_s': 4.}],
            'pose_fault_samples': [sample, {**sample, 'monotonic_s': 12., 'source_stamp_sim_s': 21.}],
            'rotation_terminal_events': [{'mission_id': 'rotate', 'status': 'failed', 'error': 'pose is stale'}],
            'safety_landing_events': [{'event': 'rotation_pose_safety_landing', 'mission_id': 'rotate',
                                       'time': 101.}],
            'last_state': {'armed': False, 'landed_state': 1, 'mission': {'id': 'land',
                'parent_mission_id': 'rotate', 'steps': [{'action': 'land'}], 'status': 'succeeded'}},
            'fault_clear': {'cleared': True, 'confirmed_absent': True}}

    def test_valid_and_wrong_fault_or_late_source_are_distinguished(self):
        self.assertTrue(audit(self.record)['passed'])
        for change in ('wrong_id', 'late_source', 'same_stamp', 'unowned_landing'):
            record = copy.deepcopy(self.record)
            if change == 'wrong_id': record['pose_fault_samples'][1]['fault_id'] = 'other'
            if change == 'late_source': record['pose_fault_samples'][1]['monotonic_s'] = 15.
            if change == 'same_stamp': record['pose_fault_samples'][1]['source_stamp_sim_s'] = 20.
            if change == 'unowned_landing': record['last_state']['mission']['parent_mission_id'] = 'other'
            with self.subTest(change=change):
                self.assertFalse(audit(record)['passed'])


if __name__ == '__main__':
    unittest.main()
