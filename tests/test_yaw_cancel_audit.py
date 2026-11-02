import copy
import json
import tempfile
import unittest
from pathlib import Path
from audit_yaw_cancel_sitl import audit


class CancellationAuditTests(unittest.TestCase):
    def test_only_owned_cancelled_rotation_and_own_landing_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = {'status': 'passed', 'owned_mission_ids': ['takeoff', 'turn', 'landing'],
                'checks': {'capture_blocked_after_cancel': True, 'landed_disarmed': True},
                'rotation_terminal': {'id': 'turn', 'status': 'cancelled', 'action': 'rotate',
                                      'steps': [{'action': 'rotate', 'yaw_rad': 1.}]},
                'landing_terminal': {'id': 'landing', 'parent_mission_id': 'turn',
                                     'status': 'succeeded', 'steps': [{'action': 'land'}]}}
            path = root/'result.json'; path.write_text(json.dumps(record))
            self.assertTrue(audit(path, root/'good')['passed'])
            broken = copy.deepcopy(record); broken['landing_terminal']['parent_mission_id'] = 'other'
            path.write_text(json.dumps(broken))
            self.assertFalse(audit(path, root/'wrong-owner')['checks']['independent_landing_completed'])
            broken = copy.deepcopy(record); broken['rotation_terminal']['status'] = 'succeeded'
            path.write_text(json.dumps(broken))
            self.assertFalse(audit(path, root/'not-cancelled')['checks']['rotation_cancelled'])
            broken = copy.deepcopy(record)
            broken['actual_rotation_before_cancel'] = {'source': '/vins_estimator/odometry',
                'delta_rad': .01, 'age_seconds': .1}
            path.write_text(json.dumps(broken))
            self.assertFalse(audit(path, root/'no-actual-turn')['checks']['actual_turn_before_cancel'])
