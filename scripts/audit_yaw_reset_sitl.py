#!/usr/bin/env python3
"""Audit archived actual VINS restart, owned rotation stop and automatic landing."""
import argparse
import json
import math
import sys
from pathlib import Path


def audit(result_path, output):
    record = json.loads(result_path.read_text()); ids = (record.get('owned_mission_ids', [])+[None]*3)[:3]
    injection = record.get('injection') or {}; request = injection.get('request') or {}
    motion = record.get('motion_before_fault') or {}; final = record.get('last_state') or {}
    landing = final.get('mission') or {}; terminal = record.get('rotation_terminal_events') or []
    safety = record.get('safety_landing_events') or []
    values = [motion.get(k) for k in ('initial_vio_yaw_rad', 'observed_vio_yaw_rad', 'delta_rad', 'age_seconds')]
    valid = all(type(v) in (int, float) and math.isfinite(v) for v in values)
    if valid:
        initial, final_yaw, delta, age = values
        valid = abs(math.atan2(math.sin(final_yaw-initial), math.cos(final_yaw-initial))-delta) <= 1e-9 and abs(delta) >= .06 and 0 <= age <= 1.
    issued = request.get('issued_at_wall_s')
    checks = {'reported_passed': record.get('status') == 'passed',
              'all_runtime_checks_passed': bool(record.get('checks')) and all(v is True for v in record['checks'].values()),
              'actual_motion_before_fault': valid,
              'actual_restart_in_simulation': injection.get('simulation_only') is True
                  and injection.get('estimator_restart_published') is True and request.get('kind') == 'vio_reset',
              'rotation_stopped': bool(terminal) and terminal[-1].get('mission_id') == ids[1]
                  and terminal[-1].get('status') in ('failed', 'cancelled'),
              'owned_landing_completed': landing.get('id') == ids[2] and landing.get('parent_mission_id') == ids[1]
                  and landing.get('steps') == [{'action': 'land'}] and landing.get('status') == 'succeeded'
                  and final.get('armed') is False and final.get('landed_state') == 1,
              'safety_event_after_fault': type(issued) in (int, float) and any(
                  e.get('event') == 'external_vision_safety_landing' and e.get('reason')
                  and e.get('time', 0) >= issued for e in safety),
              'capture_blocked': record.get('checks', {}).get('capture_blocked_after_restart') is True,
              'no_fallback': not record.get('fallback_landing_requested', False)}
    checks = {k: bool(v) for k, v in checks.items()}
    result = {'passed': all(checks.values()), 'checks': checks,
              'limitation': 'Checks recorded VIO restart and safety chain; does not prove continuous trajectory or all fault types.'}
    output.mkdir(parents=True, exist_ok=False)
    (output/'yaw-reset-audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); result = audit(args.result, args.output)
    print(json.dumps(result, ensure_ascii=False, indent=2)); sys.exit(0 if result['passed'] else 2)
