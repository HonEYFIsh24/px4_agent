#!/usr/bin/env python3
"""Independently audit a bounded controller-local pose receipt loss experiment."""
import argparse
import json
import math
import sys
from pathlib import Path


def audit(record):
    ids = (record.get('owned_mission_ids', []) + [None] * 3)[:3]
    injection = record.get('injection') or {}
    request = injection.get('request') or {}
    motion = record.get('motion_before_fault') or {}
    terminal = record.get('rotation_terminal_events') or []
    landing = (record.get('last_state') or {}).get('mission') or {}
    final = record.get('last_state') or {}
    lifecycle = record.get('pose_fault_lifecycle') or []
    starts = [e for e in lifecycle if e.get('event') == 'reliability_started'
              and e.get('fault_id') == request.get('id') and e.get('kind') == 'pose_drop']
    start = starts[0].get('monotonic_s') if len(starts) == 1 else None
    duration = starts[0].get('duration_s') if len(starts) == 1 else None
    valid_number = lambda v: type(v) in (int, float) and math.isfinite(v)
    samples = [s for s in record.get('pose_fault_samples', []) if s.get('fault_id') == request.get('id')
               and s.get('kind') == 'pose_drop' and s.get('dropped') is True
               and s.get('synthetic') is True and s.get('simulation_only') is True
               and s.get('source_topic') == '/mavros/local_position/pose'
               and valid_number(s.get('monotonic_s')) and valid_number(start) and valid_number(duration)
               and start <= s['monotonic_s'] <= start + duration and valid_number(s.get('source_stamp_sim_s'))]
    stamps = [s['source_stamp_sim_s'] for s in samples]
    values = [motion.get(k) for k in ('initial_vio_yaw_rad', 'observed_vio_yaw_rad', 'delta_rad', 'age_seconds')]
    moved = all(valid_number(v) for v in values)
    if moved:
        a, b, delta, age = values
        moved = abs(math.atan2(math.sin(b-a), math.cos(b-a))-delta) < 1e-9 and abs(delta) >= .06 and 0 <= age <= 1
    clear = record.get('fault_clear') or {}
    checks = {
        'runtime_passed': record.get('status') == 'passed' and bool(record.get('checks'))
                          and all(v is True for v in record['checks'].values()),
        'actual_yaw_motion': moved,
        'bounded_owned_fault': injection.get('readback') == request and request.get('kind') == 'pose_drop'
            and request.get('synthetic') is True and request.get('simulation_only') is True
            and valid_number(request.get('duration_s')) and .1 <= request['duration_s'] <= 4,
        'upstream_pose_continued_while_receipts_dropped': len(stamps) >= 2 and max(stamps) > min(stamps),
        'rotation_failed': bool(terminal) and terminal[-1].get('mission_id') == ids[1]
            and terminal[-1].get('status') == 'failed' and 'stale' in (terminal[-1].get('error') or '').lower(),
        'owned_automatic_landing': landing.get('id') == ids[2] and landing.get('parent_mission_id') == ids[1]
            and landing.get('steps') == [{'action': 'land'}] and landing.get('status') == 'succeeded'
            and final.get('armed') is False and final.get('landed_state') == 1,
        'pose_watchdog': any(e.get('event') == 'rotation_pose_safety_landing' and e.get('mission_id') == ids[1]
            and valid_number(e.get('time')) and valid_number(request.get('issued_at_wall_s'))
            and e['time'] >= request['issued_at_wall_s'] for e in record.get('safety_landing_events', [])),
        'capture_blocked': record.get('checks', {}).get('capture_blocked_after_restart') is True,
        'fault_cleared': clear.get('cleared') is True and clear.get('confirmed_absent') is True,
        'no_fallback': not record.get('fallback_landing_requested', False),
    }
    return {'passed': all(checks.values()), 'checks': checks,
            'limitation': 'Synthetic drop is local to the controller callback, not an actual upstream ROS publisher failure.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(json.loads(args.result.read_text()))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'yaw-pose-drop-audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
