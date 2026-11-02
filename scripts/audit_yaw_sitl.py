#!/usr/bin/env python3
"""Audit saved yaw SITL evidence, including the actual post-rotation camera pose."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from yaw_control import error


def yaw(q):
    if not isinstance(q, list) or len(q) != 4 or any(type(v) not in (int, float) or not math.isfinite(v) for v in q):
        raise ValueError('Invalid actual camera body quaternion')
    length = math.sqrt(sum(v*v for v in q))
    if not .99 <= length <= 1.01:
        raise ValueError('Actual camera quaternion is not normalized')
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def audit(result_path, output):
    record = json.loads(result_path.read_text())
    before, after = record['images']['before'], record['images']['after']
    rotation = record['rotation']
    checks = {'task_reported_passed': record['status'] == 'passed',
              'all_runtime_checks_passed': bool(record['checks']) and all(v is True for v in record['checks'].values()),
              'three_distinct_owned_missions': len(set(record['owned_mission_ids'])) == 3,
              'same_coordinate_epoch': before['coordinate_epoch'] == after['coordinate_epoch'],
              'actual_pose_time_order': before['stamp_sim_s'] < rotation['stamp_s'] < after['stamp_sim_s'],
              'actual_arrival_yaw': abs(error(rotation['target_rad'], rotation['actual_rad'])) <= .1,
              'post_image_actual_yaw': abs(error(rotation['target_rad'], yaw(after['body_pose_enu']['orientation_xyzw']))) <= .12,
              'vehicle_turned': abs(error(yaw(after['body_pose_enu']['orientation_xyzw']),
                                          yaw(before['body_pose_enu']['orientation_xyzw']))) >= math.radians(30.),
              'position_held': math.dist(before['body_pose_enu']['position_m'], after['body_pose_enu']['position_m']) <= .35}
    for name, image in (('before', before), ('after', after)):
        checks[name+'_image_hash'] = hashlib.sha256((result_path.parent/(name+'.jpg')).read_bytes()).hexdigest() == image['sha256']
    result = {'passed': all(checks.values()), 'checks': checks,
              'limitation': 'Audits saved actual camera poses and executor results; does not prove continuous trajectory.'}
    output.mkdir(parents=True, exist_ok=False)
    (output/'yaw-audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.result, args.output)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    sys.exit(0 if result['passed'] else 2)
