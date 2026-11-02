import hashlib
import json
import math
import tempfile
import unittest
from pathlib import Path
from audit_yaw_sitl import audit


class YawAuditTests(unittest.TestCase):
    def test_actual_camera_heading_and_image_bytes_are_independently_checked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            before, after = b'controlled-before', b'controlled-after'
            (root/'before.jpg').write_bytes(before); (root/'after.jpg').write_bytes(after)
            def image(raw, stamp, heading):
                return {'sha256': hashlib.sha256(raw).hexdigest(), 'stamp_sim_s': stamp,
                        'coordinate_epoch': {'epoch': 1}, 'body_pose_enu': {'position_m': [0, 0, 3],
                        'orientation_xyzw': [0, 0, math.sin(heading/2), math.cos(heading/2)]}}
            record = {'status': 'passed', 'owned_mission_ids': ['a', 'b', 'c'], 'checks': {'landed_disarmed': True},
                      'rotation': {'target_rad': .8, 'actual_rad': .8, 'stamp_s': 2.},
                      'images': {'before': image(before, 1., 0.), 'after': image(after, 3., .8)}}
            path = root/'result.json'; path.write_text(json.dumps(record))
            self.assertTrue(audit(path, root/'pass')['passed'])
            record['images']['after']['body_pose_enu']['orientation_xyzw'] = [0, 0, 0, 1]
            path.write_text(json.dumps(record))
            wrong = audit(path, root/'wrong-heading')
            self.assertFalse(wrong['checks']['post_image_actual_yaw'])
            (root/'after.jpg').write_bytes(b'tampered')
            self.assertFalse(audit(path, root/'wrong-image')['checks']['after_image_hash'])
