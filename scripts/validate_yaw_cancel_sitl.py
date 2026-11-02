#!/usr/bin/env python3
"""Opt-in SITL test of owned cancellation during an actual yaw rotation."""
import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from client import BridgeClient
from search_tools import coordinate_epoch
from yaw_control import error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        print('Dry run; no simulator commands. Add --execute for a SITL cancellation flight.')
        return 0
    bridge = BridgeClient(); state = bridge.state(); ev = state.get('external_vision') or {}
    if not (state.get('simulation_only') is True and state.get('ready') is True
            and state.get('armed') is False and state.get('landed_state') == 1
            and state.get('experimental_yaw_enabled') is True
            and state.get('navigation', {}).get('method') == 'static_3d'
            and ev.get('healthy') is True and ev.get('used_for_flight_control') is True):
        raise RuntimeError('Requires grounded/disarmed static_3d SITL, active VIO and explicitly enabled yaw')
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'run': (ROOT/'logs/system/latest').resolve().name,
              'owned_mission_ids': [], 'checks': {}}
    def save():
        tmp = args.output/'result.tmp'; tmp.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        tmp.replace(args.output/'result.json')
    def submit(steps):
        accepted = bridge.request('/mission', {'steps': steps, 'description': 'owned yaw cancellation validation'})
        report['owned_mission_ids'].append(accepted['mission']['id']); save()
        return accepted
    save()
    try:
        takeoff = bridge.wait_for(submit([{'action': 'takeoff', 'altitude': 3.}]))
        if takeoff['mission']['status'] != 'succeeded':
            raise RuntimeError('Takeoff failed: '+str(takeoff['mission'].get('error')))
        pair = bridge.capture_stereo(); q = pair['body_pose_enu']['orientation_xyzw']
        initial = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]*q[1]+q[2]*q[2]))
        before_vio = bridge.state().get('vio') or {}
        if before_vio.get('ready') is not True or before_vio.get('age_seconds', 99) > 1.:
            raise RuntimeError('Fresh actual VIO attitude is required before rotation')
        def heading(values):
            if not isinstance(values, list) or len(values) != 4 or any(
                    type(v) not in (int, float) or not math.isfinite(v) for v in values):
                raise RuntimeError('VIO attitude is invalid')
            x, y, z, w = values
            return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        initial_vio = heading(before_vio.get('orientation_xyzw'))
        target = math.atan2(math.sin(initial+2.5), math.cos(initial+2.5))
        accepted = submit([{'action': 'rotate', 'yaw_rad': target}]); rotation_id = accepted['mission']['id']
        deadline = time.monotonic()+15
        while True:
            current = bridge.state(); mission = current.get('mission') or {}
            if mission.get('id') != rotation_id or mission.get('status') != 'running':
                raise RuntimeError('Rotation ended or ownership changed before cancellation')
            vio = current.get('vio') or {}
            if coordinate_epoch(current) != pair['coordinate_epoch']:
                raise RuntimeError('Coordinate identity changed before cancellation')
            actual_vio = heading(vio.get('orientation_xyzw')) if vio.get('ready') is True and vio.get('age_seconds', 99) <= 1. else None
            if mission.get('action') == 'rotate' and actual_vio is not None and abs(error(actual_vio, initial_vio)) >= .06:
                report['actual_rotation_before_cancel'] = {'source': '/vins_estimator/odometry',
                    'initial_vio_yaw_rad': initial_vio, 'observed_vio_yaw_rad': actual_vio,
                    'delta_rad': error(actual_vio, initial_vio), 'age_seconds': vio['age_seconds'],
                    'coordinate_epoch': pair['coordinate_epoch']}
                report['checks']['actual_rotation_observed_before_cancel'] = True
                save()
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('Fresh VIO did not show an actual turn of at least 0.06 rad before cancellation')
            time.sleep(.05)
        stop = bridge.request('/stop', {'expected_mission_id': rotation_id})
        report['checks']['scoped_stop_requested_while_rotating'] = stop.get('requested') == 'cancel'
        report['checks']['offboard_position_hold_requested'] = stop.get('holding') is True
        cancelled = bridge.wait_for(accepted)
        report['rotation_terminal'] = cancelled['mission']
        report['checks']['owned_rotation_cancelled'] = cancelled['mission']['status'] == 'cancelled'
        try:
            bridge.capture_stereo()
            report['checks']['capture_blocked_after_cancel'] = False
        except RuntimeError as exc:
            report['checks']['capture_blocked_after_cancel'] = 'rotation' in str(exc)
        save()
        land = bridge.request('/land', {'expected_mission_id': rotation_id})
        report['owned_mission_ids'].append(land['mission']['id']); save()
        landed = bridge.wait_for(land)
        report['landing_terminal'] = landed['mission']
        report['checks']['landed_disarmed'] = (landed['mission']['status'] == 'succeeded'
            and landed.get('armed') is False and landed.get('landed_state') == 1)
        report['status'] = 'passed' if all(report['checks'].values()) else 'failed'
    except Exception as exc:
        report.update(status='failed', error=str(exc))
        try:
            state = bridge.state(); current = (state.get('mission') or {}).get('id')
            if state.get('armed') is True and current in report['owned_mission_ids']:
                accepted = bridge.request('/land', {'expected_mission_id': current})
                result = bridge.wait_for(accepted)
                report['recovery'] = {'status': result['mission']['status'], 'armed': result.get('armed'),
                                      'landed_state': result.get('landed_state')}
        except Exception as recovery_error:
            report['recovery'] = {'error': str(recovery_error)}
    save(); print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
