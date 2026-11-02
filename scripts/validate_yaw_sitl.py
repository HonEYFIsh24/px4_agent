#!/usr/bin/env python3
"""Opt-in PX4 SITL rotation validation; makes no model requests."""
import argparse
import base64
import hashlib
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from client import BridgeClient
from yaw_control import error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        print('Dry run: no simulator commands. Add --execute for a real SITL flight.')
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
              'owned_mission_ids': [], 'checks': {}, 'images': {}}
    def save():
        tmp = args.output/'result.tmp'
        tmp.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        tmp.replace(args.output/'result.json')
    def mission(steps):
        accepted = bridge.request('/mission', {'steps': steps, 'description': 'yaw validation'})
        report['owned_mission_ids'].append(accepted['mission']['id']); save()
        result = bridge.wait_for(accepted)
        if result['mission']['status'] != 'succeeded':
            raise RuntimeError('Flight mission failed: '+str(result['mission'].get('error')))
        return result
    def photo(name, after=None):
        end = time.monotonic()+12
        while True:
            try:
                pair = bridge.capture_stereo()
                if after is not None and pair['stamp_sim_s'] <= after:
                    raise RuntimeError('Image is not newer than actual rotation arrival')
                break
            except RuntimeError as exc:
                if time.monotonic() >= end:
                    raise RuntimeError('Could not obtain fresh stereo image: '+str(exc))
                time.sleep(.2)
        raw = base64.b64decode(pair['left']['image_base64'], validate=True)
        digest = hashlib.sha256(raw).hexdigest()
        if digest != pair['image_sha256']['left']:
            raise RuntimeError('Actual image hash mismatch')
        (args.output/(name+'.jpg')).write_bytes(raw)
        report['images'][name] = {'sha256': digest, 'stamp_sim_s': pair['stamp_sim_s'],
                                  'body_pose_enu': pair['body_pose_enu'], 'coordinate_epoch': pair['coordinate_epoch']}
        save()
        return pair
    save()
    try:
        mission([{'action': 'takeoff', 'altitude': 3.}])
        before = photo('before')
        q = before['body_pose_enu']['orientation_xyzw']
        initial = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]*q[1]+q[2]*q[2]))
        target = math.atan2(math.sin(initial+math.pi/4), math.cos(initial+math.pi/4))
        accepted = bridge.request('/mission', {'steps': [{'action': 'rotate', 'yaw_rad': target}],
                                                'description': 'yaw validation'})
        report['owned_mission_ids'].append(accepted['mission']['id']); save()
        try:
            bridge.capture_stereo()
            report['checks']['capture_blocked_during_rotation'] = False
        except RuntimeError as exc:
            report['checks']['capture_blocked_during_rotation'] = 'rotation' in str(exc)
        result = bridge.wait_for(accepted)
        if result['mission']['status'] != 'succeeded':
            raise RuntimeError('Rotation failed: '+str(result['mission'].get('error')))
        rotation = result['mission']['events'][-1]['rotation']; report['rotation'] = rotation
        report['checks']['actual_arrival_within_tolerance'] = abs(error(target, rotation['actual_rad'])) <= .1
        after = photo('after', rotation['stamp_s'])
        report['checks']['after_image_newer_than_arrival'] = after['stamp_sim_s'] > rotation['stamp_s']
        report['checks']['same_epoch'] = before['coordinate_epoch'] == after['coordinate_epoch']
        landed = mission([{'action': 'return_home'}, {'action': 'land'}])
        report['checks']['landed_disarmed'] = landed.get('armed') is False and landed.get('landed_state') == 1
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
