#!/usr/bin/env python3
"""Audit archived bounded EV transport drop during an owned yaw rotation."""
import argparse
import json
import math
import sys
from pathlib import Path


def audit(result_path, output):
    r = json.loads(result_path.read_text()); ids = (r.get('owned_mission_ids', [])+[None]*3)[:3]
    injection = r.get('injection') or {}; request = injection.get('request') or {}
    motion = r.get('motion_before_fault') or {}; final = r.get('last_state') or {}; landing = final.get('mission') or {}
    terminal = r.get('rotation_terminal_events') or []; safety = r.get('safety_landing_events') or []
    values = [motion.get(k) for k in ('initial_vio_yaw_rad', 'observed_vio_yaw_rad', 'delta_rad', 'age_seconds')]
    valid = all(type(v) in (int, float) and math.isfinite(v) for v in values)
    if valid:
        a, b, delta, age = values
        valid = abs(math.atan2(math.sin(b-a), math.cos(b-a))-delta) <= 1e-9 and abs(delta) >= .06 and 0 <= age <= 1.
    applied = r.get('adapter_fault_evidence') or []; baseline = r.get('pre_fault_shadow_stamp_sim_s')
    issued = request.get('issued_wall_s'); expiry = request.get('expires_wall_s'); clear = r.get('fault_clear') or {}
    active_samples = [x for x in applied if x.get('id') == request.get('id') and x.get('kind') == 'drop'
        and x.get('active') is True and type(issued) in (int, float) and type(expiry) in (int, float)
        and type(x.get('observed_wall_s')) in (int, float) and issued <= x['observed_wall_s'] <= expiry]
    checks = {'reported_passed': r.get('status') == 'passed',
        'all_runtime_checks_passed': bool(r.get('checks')) and all(v is True for v in r['checks'].values()),
        'actual_motion_before_fault': valid,
        'bounded_owned_fault': injection.get('synthetic') is True and injection.get('simulation_only') is True
            and request.get('kind') == 'drop' and request.get('synthetic') is True and request.get('simulation_only') is True
            and type(request.get('duration_s')) in (int, float) and .5 <= request['duration_s'] <= 4.,
        'adapter_applied_drop': any(x.get('synthetic') is True and x.get('simulation_only') is True
            and x.get('applied_count', 0) > 0 for x in active_samples),
        'source_continued_during_drop': type(baseline) in (int, float) and any(
            type(x.get('shadow_stamp_sim_s')) in (int, float) and x['shadow_stamp_sim_s'] > baseline for x in active_samples),
        'rotation_stopped': bool(terminal) and terminal[-1].get('mission_id') == ids[1]
            and terminal[-1].get('status') in ('failed', 'cancelled'),
        'owned_landing_completed': landing.get('id') == ids[2] and landing.get('parent_mission_id') == ids[1]
            and landing.get('steps') == [{'action': 'land'}] and landing.get('status') == 'succeeded'
            and final.get('armed') is False and final.get('landed_state') == 1,
        'safety_event_after_fault': type(issued) in (int, float) and any(
            e.get('event') == 'external_vision_safety_landing' and e.get('reason') and e.get('time', 0) >= issued for e in safety),
        'capture_blocked': r.get('checks', {}).get('capture_blocked_after_restart') is True,
        'owned_fault_cleared': clear.get('cleared') is True and clear.get('confirmed_absent') is True,
        'no_fallback': not r.get('fallback_landing_requested', False)}
    checks = {k: bool(v) for k, v in checks.items()}
    result = {'passed': all(checks.values()), 'checks': checks,
              'limitation': 'Checks a single bounded synthetic EV transport fault; it does not prove pose-topic expiry or continuous trajectory.'}
    output.mkdir(parents=True, exist_ok=False)
    (output/'yaw-ev-drop-audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); result = audit(args.result, args.output)
    print(json.dumps(result, ensure_ascii=False, indent=2)); sys.exit(0 if result['passed'] else 2)
