#!/usr/bin/env python3
"""Ground-only profiles for the uniquely owned local PX4 simulation.

Every mutation is journalled and read back. This does not establish fusion;
record_ev_fusion.py provides the independent, actual EKF evidence.
"""
import argparse
import json
import math
import os
import re
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from client import BridgeClient

PARAMETERS = ('EKF2_HGT_REF', 'EKF2_BARO_CTRL', 'EKF2_GPS_CTRL', 'EKF2_EV_CTRL',
              'EKF2_EV_QMIN', 'EKF2_EV_NOISE_MD', 'EKF2_EV_DELAY',
              'EKF2_EVP_NOISE', 'EKF2_EVV_NOISE', 'EKF2_EVA_NOISE',
              'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z',
              'COM_OBL_RC_ACT', 'COM_OF_LOSS_T', 'SDLOG_MODE', 'SDLOG_PROFILE')


def target_profile(name, original):
    if name not in ('shadow', 'gps_fused', 'no_gps'):
        raise ValueError('unknown profile')
    return dict(EKF2_HGT_REF=3 if name == 'no_gps' else 0, EKF2_BARO_CTRL=1,
                EKF2_GPS_CTRL=0 if name == 'no_gps' else 7,
                EKF2_EV_CTRL=0 if name == 'shadow' else (7 if name == 'no_gps' else 5),
                EKF2_EV_QMIN=1, EKF2_EV_NOISE_MD=0, EKF2_EV_DELAY=0,
                EKF2_EVP_NOISE=.20, EKF2_EVV_NOISE=.30, EKF2_EVA_NOISE=.26,
                EKF2_EV_POS_X=0, EKF2_EV_POS_Y=0, EKF2_EV_POS_Z=0,
                COM_OBL_RC_ACT=4 if name == 'no_gps' else original['COM_OBL_RC_ACT'],
                COM_OF_LOSS_T=1.0 if name == 'no_gps' else original['COM_OF_LOSS_T'],
                SDLOG_MODE=1, SDLOG_PROFILE=int(original['SDLOG_PROFILE']) | 128)


def ground_guard(state):
    if (state.get('simulation_only') is not True or state.get('connected') is not True
            or state.get('armed') is not False or state.get('landed_state') != 1
            or not state.get('gazebo', {}).get('available')):
        raise RuntimeError('profile changes require connected, grounded and disarmed owned SITL')
    if (state.get('mission') or {}).get('status') == 'running':
        raise RuntimeError('a mission is running; profile change refused')


def parse_value(stdout, name):
    match = re.search(r'\b' + re.escape(name) + r'\b[^:\n]*:\s*([+-]?[\d.]+)', stdout)
    if not match:
        raise RuntimeError('missing actual parameter ' + name)
    value = float(match.group(1))
    if not math.isfinite(value):
        raise RuntimeError('invalid actual parameter ' + name)
    return int(value) if value.is_integer() else value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('shadow', 'gps_fused', 'no_gps'))
    parser.add_argument('--restore', type=Path, help='Prior profile original.json')
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--run-directory', type=Path, default=ROOT/'logs/system/latest')
    parser.add_argument('--px4-directory', type=Path, default=Path(os.getenv('PX4_DIR', '/home/dut/PX4-Autopilot')))
    args = parser.parse_args(argv)
    if bool(args.profile) == bool(args.restore):
        parser.error('choose one of --profile or --restore')
    if not args.apply:
        print(json.dumps({'apply': False, 'profile': args.profile, 'restore': str(args.restore) if args.restore else None,
                          'ground_only': True, 'fusion_proven': False, 'parameters': PARAMETERS}))
        return 0
    if args.output is None:
        parser.error('--apply requires --output NEW_DIRECTORY')
    from run_reliability_suite import process_identity, owned_px4_process
    from record_ev_fusion import bounded_command, stable_px4_identity
    client = BridgeClient()
    binary = args.px4_directory.resolve()/'build/px4_sitl_default/bin/px4-param'
    manager = process_identity(args.run_directory)
    if manager is None:
        raise RuntimeError('no live owned simulation manager')
    px4 = stable_px4_identity(owned_px4_process(manager, binary))

    def guard():
        if (process_identity(args.run_directory) != manager
                or stable_px4_identity(owned_px4_process(manager, binary)) != px4):
            raise RuntimeError('manager/PX4 identity changed')
        ground_guard(client.state())

    guard()
    args.output.mkdir(parents=True, exist_ok=False)
    stream = (args.output/'commands.jsonl').open('x', buffering=1)

    def command(*words):
        guard()
        record = bounded_command([str(binary), *words], 2)
        record['wall_s'] = time.time()
        stream.write(json.dumps(record) + '\n')
        if record.get('return_code') != 0 or record.get('error'):
            raise RuntimeError('parameter command failed: ' + ' '.join(words))
        return record['stdout']

    original = {name: parse_value(command('show', name), name) for name in PARAMETERS}
    metadata = {'manager': manager, 'px4': px4, 'parameters': original}
    (args.output/'original.json').write_text(json.dumps(metadata, indent=2)+'\n')
    target = (json.loads(args.restore.read_text())['parameters'] if args.restore
              else target_profile(args.profile, original))
    if set(target) != set(PARAMETERS) or any(type(x) not in (float, int) or not math.isfinite(x) for x in target.values()):
        raise RuntimeError('restore parameter contract is invalid')
    result = {'profile': args.profile, 'restored_from': str(args.restore) if args.restore else None,
              'target': target, 'manager': manager, 'px4': px4, 'fusion_proven': False,
              'no_gps_scope': '3D position and velocity from VIO; vision height reference, barometer backup and magnetometer retained',
              'success': False}
    try:
        # Non-GNSS HGT_REF precedes GPS_CTRL=0: v1.14 otherwise reenables GPS height.
        for name in PARAMETERS:
            if original[name] != target[name]:
                command('set', name, str(target[name]))
            observed = parse_value(command('show', name), name)
            if not math.isclose(observed, target[name], rel_tol=1e-4, abs_tol=1e-4):
                raise RuntimeError('parameter readback mismatch: ' + name)
        result['actual'] = {name: parse_value(command('show', name), name) for name in PARAMETERS}
        if result['actual'] != target:
            raise RuntimeError('final profile changed after application')
        result['success'] = True
    except Exception as exc:
        result.update(error_type=type(exc).__name__, error=str(exc))
        try:
            # Do not conceal original failure or restore over an armed vehicle.
            for name in PARAMETERS:
                command('set', name, str(original[name]))
            result['rollback_readback'] = {name: parse_value(command('show', name), name) for name in PARAMETERS}
            result['rolled_back'] = result['rollback_readback'] == original
        except Exception as rollback:
            result['rollback_error'] = str(rollback)
    finally:
        stream.close()
        (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
