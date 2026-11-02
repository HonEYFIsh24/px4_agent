#!/usr/bin/env python3
"""Audit recorded owned rotation cancellation and independent landing evidence."""
import argparse
import json
import math
import sys
from pathlib import Path


def audit(result_path, output):
    record = json.loads(result_path.read_text())
    identifiers = record['owned_mission_ids']
    rotation, landing = record['rotation_terminal'], record['landing_terminal']
    ids = (identifiers+[None, None, None])[:3]
    checks = {'reported_passed': record['status'] == 'passed',
              'runtime_checks_passed': bool(record['checks']) and all(v is True for v in record['checks'].values()),
              'three_distinct_owned_missions': len(identifiers) == 3 and len(set(identifiers)) == 3,
              'rotation_cancelled': rotation.get('id') == ids[1] and rotation.get('status') == 'cancelled'
                  and rotation.get('action') == 'rotate' and rotation.get('steps')
                  and rotation['steps'][0].get('action') == 'rotate',
              'independent_landing_completed': landing.get('id') == ids[2]
                  and landing.get('parent_mission_id') == ids[1]
                  and landing.get('steps') == [{'action': 'land'}] and landing.get('status') == 'succeeded',
              'capture_blocked': record['checks'].get('capture_blocked_after_cancel') is True,
              'actual_grounded_disarmed': record['checks'].get('landed_disarmed') is True}
    checks = {k: bool(v) for k, v in checks.items()}
    if 'actual_rotation_before_cancel' in record:
        observed = record['actual_rotation_before_cancel']
        values = [observed.get(k) for k in ('initial_vio_yaw_rad', 'observed_vio_yaw_rad', 'delta_rad', 'age_seconds')]
        valid = all(type(v) in (int, float) and math.isfinite(v) for v in values)
        if valid:
            initial, final, delta, age = values
            measured = math.atan2(math.sin(final-initial), math.cos(final-initial))
            valid = abs(measured-delta) <= 1e-9 and abs(delta) >= .06 and 0 <= age <= 1.
        epoch = observed.get('coordinate_epoch') or {}
        checks['actual_turn_before_cancel'] = (observed.get('source') == '/vins_estimator/odometry'
            and valid and type(epoch.get('vio_epoch')) is int and isinstance(epoch.get('ekf_counters'), dict))
    result = {'passed': all(checks.values()), 'checks': checks,
              'limitation': 'Audits recorded cancellation and landing; does not independently prove continuous heading or trajectory.'}
    output.mkdir(parents=True, exist_ok=False)
    (output/'cancel-audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.result, args.output)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    sys.exit(0 if result['passed'] else 2)
