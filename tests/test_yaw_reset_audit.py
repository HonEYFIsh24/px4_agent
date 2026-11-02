import copy
import json
import tempfile
import unittest
from pathlib import Path
from audit_yaw_reset_sitl import audit


class ResetAuditTests(unittest.TestCase):
    def test_rejects_unowned_landing_or_a_completed_rotation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); path = root/'result.json'
            record = {'status': 'passed', 'owned_mission_ids': ['takeoff', 'turn', 'landing'],
                'checks': {'capture_blocked_after_restart': True, 'landed_disarmed': True},
                'motion_before_fault': {'initial_vio_yaw_rad': 0., 'observed_vio_yaw_rad': .1,
                    'delta_rad': .1, 'age_seconds': .02},
                'injection': {'simulation_only': True, 'estimator_restart_published': True,
                              'request': {'kind': 'vio_reset', 'issued_at_wall_s': 10.}},
                'rotation_terminal_events': [{'event': 'mission_finished', 'mission_id': 'turn', 'status': 'cancelled'}],
                'safety_landing_events': [{'event': 'external_vision_safety_landing', 'time': 11., 'reason': 'reset'}],
                'last_state': {'armed': False, 'landed_state': 1, 'mission': {'id': 'landing',
                    'parent_mission_id': 'turn', 'steps': [{'action': 'land'}], 'status': 'succeeded'}}}
            path.write_text(json.dumps(record)); self.assertTrue(audit(path, root/'good')['passed'])
            changed = copy.deepcopy(record); changed['last_state']['mission']['parent_mission_id'] = 'other'
            path.write_text(json.dumps(changed)); self.assertFalse(audit(path, root/'wrong-owner')['checks']['owned_landing_completed'])
            changed = copy.deepcopy(record); changed['rotation_terminal_events'][0]['status'] = 'succeeded'
            path.write_text(json.dumps(changed)); self.assertFalse(audit(path, root/'wrong-status')['checks']['rotation_stopped'])
