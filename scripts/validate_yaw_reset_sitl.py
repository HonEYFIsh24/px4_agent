#!/usr/bin/env python3
"""Opt-in VINS reset, EV drop or controller pose receipt drop during SITL rotation."""
import argparse
import json
import math
import subprocess
import sys
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from client import BridgeClient
from search_tools import coordinate_epoch
from yaw_control import error


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true'); p.add_argument('--output', type=Path, required=True)
    p.add_argument('--fault', choices=('reset', 'drop', 'pose'), default='reset')
    args = p.parse_args()
    if not args.execute:
        print('Dry run: no flight or fault. Add --execute for explicitly enabled local SITL only.')
        return 0
    client = BridgeClient(); state = client.state(); ev = state.get('external_vision') or {}
    if not (state.get('simulation_only') is True and state.get('ready') is True
            and state.get('armed') is False and state.get('landed_state') == 1
            and state.get('experimental_yaw_enabled') is True and state.get('navigation', {}).get('method') == 'static_3d'
            and ((args.fault in ('reset', 'pose') and (state.get('reliability_test') or {}).get('enabled') is True)
                 or (args.fault == 'drop' and ((ev.get('adapter') or {}).get('test_fault') or {}).get('enabled') is True))
            and ev.get('healthy') is True and ev.get('used_for_flight_control') is True):
        raise RuntimeError('Requires grounded local SITL, fused VIO and explicit yaw and reliability test flags')
    args.output.mkdir(parents=True, exist_ok=False)
    log = ROOT/'logs/flight.jsonl'; offset = log.stat().st_size if log.exists() else 0
    report = {'status': 'running', 'fault_kind': args.fault, 'run': (ROOT/'logs/system/latest').resolve().name,
              'owned_mission_ids': [], 'checks': {}, 'actual_model_call': False, 'synthetic_fault': True}
    parameters, owned_parameter = None, None
    def save():
        tmp = args.output/'result.tmp'; tmp.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        tmp.replace(args.output/'result.json')
    def submit(steps):
        result = client.request('/mission', {'steps': steps, 'description': 'owned yaw VINS restart validation'})
        report['owned_mission_ids'].append(result['mission']['id']); save(); return result
    def events():
        if not log.exists(): return []
        with log.open('rb') as stream:
            stream.seek(offset); rows = stream.read().splitlines()
        return [json.loads(line) for line in rows if line.strip()]
    save()
    try:
        result = client.wait_for(submit([{'action': 'takeoff', 'altitude': 3.}]))
        if result['mission']['status'] != 'succeeded': raise RuntimeError('Takeoff did not complete')
        pair = client.capture_stereo(); q = pair['body_pose_enu']['orientation_xyzw']
        initial = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]*q[1]+q[2]*q[2]))
        before_vio = client.state().get('vio') or {}
        if before_vio.get('ready') is not True or before_vio.get('age_seconds', 99) > 1.:
            raise RuntimeError('Fresh VIO attitude is required')
        def heading(values):
            if not isinstance(values, list) or len(values) != 4 or any(
                    type(v) not in (int, float) or not math.isfinite(v) for v in values):
                raise RuntimeError('Invalid VIO attitude')
            x, y, z, w = values
            return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        initial_vio = heading(before_vio.get('orientation_xyzw'))
        target = math.atan2(math.sin(initial+2.5), math.cos(initial+2.5))
        accepted = submit([{'action': 'rotate', 'yaw_rad': target}]); rotation_id = accepted['mission']['id']
        deadline = time.monotonic()+15
        while True:
            current = client.state(); mission = current.get('mission') or {}
            if mission.get('id') != rotation_id or mission.get('status') != 'running':
                raise RuntimeError('Rotation ended or owner changed before restart')
            vio = current.get('vio') or {}
            if coordinate_epoch(current) != pair['coordinate_epoch']:
                raise RuntimeError('Coordinate identity changed before test injection')
            actual = heading(vio.get('orientation_xyzw')) if vio.get('ready') is True and vio.get('age_seconds', 99) <= 1. else None
            if mission.get('action') == 'rotate' and actual is not None and abs(error(actual, initial_vio)) >= .06:
                report['motion_before_fault'] = {'initial_vio_yaw_rad': initial_vio, 'observed_vio_yaw_rad': actual,
                    'delta_rad': error(actual, initial_vio), 'age_seconds': vio['age_seconds'],
                    'coordinate_epoch': pair['coordinate_epoch']}
                break
            if time.monotonic() >= deadline: raise RuntimeError('Fresh VIO did not show actual rotation before fault')
            time.sleep(.05)
        # Existing local-SITL injector checks ROS master, simulation, opt-in,
        # and subscriber before publishing the real upstream restart message.
        identifier = 'yaw-'+args.fault+'-'+uuid.uuid4().hex[:12]
        if args.fault == 'reset':
            command = ['bash', '-c', 'source "$1/scripts/setup_ros.bash"; shift; exec "$@"',
                       'yaw_reset', str(ROOT), '/usr/bin/python3', str(ROOT/'scripts/inject_reliability_fault.py'),
                       '--kind', 'vio_reset', '--duration', '3', '--id', identifier]
            result = subprocess.run(command, capture_output=True, text=True, timeout=12)
            if result.returncode: raise RuntimeError('Restart injection failed: '+result.stderr[-500:])
            injection = json.loads(result.stdout.strip().splitlines()[-1]); report['injection'] = injection
            report['checks']['real_restart_published'] = injection.get('estimator_restart_published') is True
        elif args.fault == 'drop':
            from test_ev_faults import Parameters, fault_request
            report['pre_fault_shadow_stamp_sim_s'] = ((current.get('external_vision') or {}).get('adapter') or {}).get('last_shadow_sim_s')
            parameters = Parameters(); owned_parameter = fault_request('drop', 4., identifier, time.time())
            readback = parameters.install(owned_parameter)
            report['injection'] = {'synthetic': True, 'simulation_only': True, 'request': owned_parameter, 'readback': readback}
        else:
            from test_ev_faults import Parameters
            from reliability_faults import FAULT_PARAMETER, fault_request
            class PoseParameters(Parameters):
                def install(self, value):
                    if self.get(FAULT_PARAMETER) not in (None, {}):
                        raise RuntimeError('Another reliability fault exists; refusing overwrite')
                    code, message, _ = self.master.setParam('/yaw_pose_test', FAULT_PARAMETER, value)
                    if code != 1 or self.get(FAULT_PARAMETER) != value:
                        raise RuntimeError('Pose fault installation or readback failed: '+str(message))
                    return value
                def clear_owned(self, value):
                    if self.get(FAULT_PARAMETER) != value:
                        return {'cleared': False, 'reason': 'request changed or absent; refusing to clear'}
                    code, message, _ = self.master.deleteParam('/yaw_pose_test', FAULT_PARAMETER)
                    if code != 1: raise RuntimeError('Cannot clear pose fault: '+str(message))
                    return {'cleared': True, 'confirmed_absent': self.get(FAULT_PARAMETER) is None}
            parameters = PoseParameters(); owned_parameter = fault_request('pose_drop', 4., identifier, time.time())
            readback = parameters.install(owned_parameter)
            report['injection'] = {'synthetic': True, 'simulation_only': True, 'request': owned_parameter, 'readback': readback}
        try:
            client.capture_stereo()
            report['checks']['capture_blocked_after_restart'] = False
        except RuntimeError as exc:
            report['checks']['capture_blocked_after_restart'] = 'rotation' in str(exc)
        save(); deadline = time.monotonic()+90; saw_unhealthy = False; landing_id = None; applied = []
        while True:
            current = client.state(); mission = current.get('mission') or {}; guard = current.get('external_vision') or {}
            saw_unhealthy |= guard.get('healthy') is False
            fault = ((guard.get('adapter') or {}).get('test_fault') or {})
            if (args.fault == 'drop' and fault.get('id') == identifier and fault.get('kind') == 'drop'
                    and fault.get('synthetic') is True and fault.get('simulation_only') is True
                    and fault.get('applied_count', 0) > 0):
                applied.append({**fault, 'observed_wall_s': time.time(),
                                'shadow_stamp_sim_s': (guard.get('adapter') or {}).get('last_shadow_sim_s')})
            if mission.get('id') != rotation_id:
                if mission.get('parent_mission_id') != rotation_id or mission.get('steps') != [{'action': 'land'}]:
                    raise RuntimeError('Mission replaced by an unrelated task; refusing to intervene')
                landing_id = mission.get('id')
                if landing_id not in report['owned_mission_ids']: report['owned_mission_ids'].append(landing_id)
            report['last_state'] = {k: current.get(k) for k in ('armed', 'landed_state', 'mission', 'external_vision')}
            if current.get('armed') is False and current.get('landed_state') == 1 and mission.get('status') != 'running':
                break
            if time.monotonic() >= deadline:
                # A guarded fallback is allowed only while the original owned
                # task is still current; never replace an unknown task.
                if mission.get('id') == rotation_id and current.get('armed') is True:
                    response = client.request('/land', {'expected_mission_id': rotation_id})
                    report['fallback_landing_requested'] = True; save()
                    client.wait_for(response)
                raise TimeoutError('Automatic VIO safety landing did not complete within 90 seconds')
            save(); time.sleep(.25)
        journal = events(); original = [e for e in journal if e.get('event') == 'mission_finished' and e.get('mission_id') == rotation_id]
        safety_kind = 'rotation_pose_safety_landing' if args.fault == 'pose' else 'external_vision_safety_landing'
        safety = [e for e in journal if e.get('event') == safety_kind and e.get('reason')]
        report['rotation_terminal_events'] = original; report['safety_landing_events'] = safety
        if args.fault == 'drop':
            report['adapter_fault_evidence'] = applied
            report['checks']['ev_drop_applied'] = bool(applied)
            baseline = report.get('pre_fault_shadow_stamp_sim_s')
            report['checks']['vio_source_advanced_during_drop'] = type(baseline) in (int, float) and any(
                type(f.get('shadow_stamp_sim_s')) in (int, float) and f['shadow_stamp_sim_s'] > baseline for f in applied)
        if args.fault == 'pose':
            samples = [e for e in journal if e.get('event') == 'reliability_sample' and e.get('fault_id') == identifier
                       and e.get('kind') == 'pose_drop' and e.get('dropped') is True]
            report['pose_fault_samples'] = samples
            report['pose_fault_lifecycle'] = [e for e in journal if e.get('event') in ('reliability_started', 'reliability_ended')
                                              and e.get('fault_id') == identifier]
            stamps = [e.get('source_stamp_sim_s') for e in samples if type(e.get('source_stamp_sim_s')) in (int, float)]
            report['checks']['pose_drop_source_advanced'] = len(stamps) >= 2 and max(stamps) > min(stamps)
        else:
            report['checks']['vio_unhealthy_observed'] = saw_unhealthy
        report['checks'].update(
            original_rotation_stopped=bool(original) and original[-1].get('status') in ('cancelled', 'failed'),
            automatic_safety_landing=bool(safety) and landing_id is not None and mission.get('id') == landing_id,
            landed_disarmed=current.get('armed') is False and current.get('landed_state') == 1,
            no_fallback=not report.get('fallback_landing_requested', False))
        report['status'] = 'passed' if all(report['checks'].values()) else 'failed'
    except Exception as exc:
        report.update(status='failed', error=str(exc))
        try:
            current = client.state(); mission = current.get('mission') or {}; identifier = mission.get('id')
            if current.get('armed') is True and identifier in report['owned_mission_ids']:
                if mission.get('steps') == [{'action': 'land'}] and mission.get('status') == 'running':
                    report['recovery'] = 'existing owned landing continues'
                else:
                    accepted = client.request('/land', {'expected_mission_id': identifier})
                    result = client.wait_for(accepted)
                    report['recovery'] = {'status': result['mission']['status'], 'armed': result.get('armed'),
                                          'landed_state': result.get('landed_state')}
        except Exception as recovery_error:
            report['recovery_error'] = str(recovery_error)
    if parameters is not None:
        try:
            report['fault_clear'] = parameters.clear_owned(owned_parameter)
        except Exception as exc:
            report.update(status='failed', fault_clear_error=str(exc))
    save(); print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
