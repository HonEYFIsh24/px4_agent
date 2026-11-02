import copy
import json
import tempfile
import unittest
from pathlib import Path
from audit_yaw_ev_drop_sitl import audit


class EVDropAuditTests(unittest.TestCase):
    def test_requires_actual_owned_drop_and_advancing_vio_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); path = root/'result.json'
            request = {'id': 'fault1', 'kind': 'drop', 'synthetic': True, 'simulation_only': True,
                       'duration_s': 4., 'issued_wall_s': 10., 'expires_wall_s': 14.5}
            record = {'status': 'passed', 'owned_mission_ids': ['takeoff', 'turn', 'landing'],
                'checks': {'capture_blocked_after_restart': True},
                'motion_before_fault': {'initial_vio_yaw_rad': 0., 'observed_vio_yaw_rad': .1, 'delta_rad': .1, 'age_seconds': .02},
                'injection': {'synthetic': True, 'simulation_only': True, 'request': request},
                'pre_fault_shadow_stamp_sim_s': 2., 'adapter_fault_evidence': [dict(request, active=True, observed_wall_s=11., applied_count=2, shadow_stamp_sim_s=3.)],
                'rotation_terminal_events': [{'event': 'mission_finished', 'mission_id': 'turn', 'status': 'cancelled'}],
                'safety_landing_events': [{'event': 'external_vision_safety_landing', 'time': 11., 'reason': 'drop'}],
                'last_state': {'armed': False, 'landed_state': 1, 'mission': {'id': 'landing',
                    'parent_mission_id': 'turn', 'steps': [{'action': 'land'}], 'status': 'succeeded'}},
                'fault_clear': {'cleared': True, 'confirmed_absent': True}}
            path.write_text(json.dumps(record)); self.assertTrue(audit(path, root/'good')['passed'])
            changed = copy.deepcopy(record); changed['adapter_fault_evidence'][0]['id'] = 'other'
            path.write_text(json.dumps(changed)); self.assertFalse(audit(path, root/'wrong-id')['checks']['adapter_applied_drop'])
            changed = copy.deepcopy(record); changed['adapter_fault_evidence'][0]['shadow_stamp_sim_s'] = 2.
            path.write_text(json.dumps(changed)); self.assertFalse(audit(path, root/'stale-source')['checks']['source_continued_during_drop'])
            changed = copy.deepcopy(record); changed['adapter_fault_evidence'][0]['active'] = False
            path.write_text(json.dumps(changed)); self.assertFalse(audit(path, root/'fault-expired')['checks']['source_continued_during_drop'])
