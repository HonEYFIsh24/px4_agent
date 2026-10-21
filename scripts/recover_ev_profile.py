#!/usr/bin/env python3
"""Explicit, ground-only recovery from a stopped manager's actual EV originals.

Only the existing fixed EV parameter list can be restored. Old evidence is
never modified. This does not authorize arming, establish fusion, or alter the
profile that start_system.sh applies after successful recovery.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from scripts.configure_ev_profile import PARAMETERS, ground_guard, parse_value


def read_record(path):
    raw = Path(path).read_bytes()
    if not 1 <= len(raw) <= 32768:
        raise RuntimeError('profile recovery record has invalid size')
    def unique_pairs(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise RuntimeError('duplicate recovery record key: ' + key)
            result[key] = value
        return result
    value = json.loads(raw, object_pairs_hook=unique_pairs)
    if not isinstance(value, dict):
        raise RuntimeError('profile recovery record must be an object')
    return value, hashlib.sha256(raw).hexdigest(), raw


def process_record(pid):
    directory = Path('/proc') / str(pid)
    try:
        fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
        return {'start_time': fields[19], 'state': fields[0], 'uid': directory.stat().st_uid}
    except FileNotFoundError:
        return None


def assert_stopped(identity, label, probe=process_record):
    current = probe(identity['pid'])
    if current is None:
        return
    if current.get('start_time') != identity['start_time'] or current.get('uid') != os.getuid():
        raise RuntimeError('old ' + label + ' PID identity/owner is ambiguous')
    if current.get('state') != 'Z':
        raise RuntimeError('old ' + label + ' is still alive; recovery refused')


def validate_source(source, px4_directory, system_root=None, probe=process_record):
    source = Path(source).resolve(strict=True)
    root = Path(system_root or ROOT / 'logs/system').resolve()
    old_run = source.parent.parent
    if source.name != 'original.json' or source.parent.name != 'ev-profile' or old_run.parent != root:
        raise RuntimeError('recover source must be an owned logs/system/RUN/ev-profile/original.json')
    if source.stat().st_uid != os.getuid():
        raise RuntimeError('old original record belongs to another OS user')
    original, original_hash, _ = read_record(source)
    if set(original) != {'manager', 'px4', 'parameters'}:
        raise RuntimeError('old original record structure is invalid')
    manager, px4, parameters = (original[k] for k in ('manager', 'px4', 'parameters'))
    if not isinstance(manager, dict) or not isinstance(px4, dict) or not isinstance(parameters, dict):
        raise RuntimeError('old original identities/parameters must be objects')
    if (set(manager) != {'pid', 'start_time', 'run_directory', 'script'}
            or type(manager.get('pid')) is not int or manager['pid'] <= 0
            or not isinstance(manager.get('start_time'), str)
            or not re.fullmatch(r'[1-9][0-9]*', manager['start_time'])
            or manager.get('run_directory') != str(old_run)
            or manager.get('script') != str(ROOT / 'scripts/start_system.sh')):
        raise RuntimeError('old manager identity does not match this archived run')
    controller, controller_hash, _ = read_record(old_run / 'controller.json')
    if controller != {k: manager[k] for k in ('pid', 'start_time', 'script')}:
        raise RuntimeError('old controller identity differs from original record')
    expected_binary = str((Path(px4_directory).resolve() / 'build/px4_sitl_default/bin/px4').resolve())
    ancestors = px4.get('ancestor_pids')
    if (set(px4) - {'pid', 'parent_pid', 'start_time', 'ancestor_pids', 'binary', 'state'}
            or type(px4.get('pid')) is not int or px4['pid'] <= 0
            or type(px4.get('parent_pid')) is not int or px4['parent_pid'] <= 0
            or not isinstance(px4.get('start_time'), str)
            or not re.fullmatch(r'[1-9][0-9]*', px4['start_time'])
            or px4.get('binary') != expected_binary
            or not isinstance(ancestors, list) or len(ancestors) < 2
            or any(type(x) is not int or x <= 0 for x in ancestors)
            or len(set(ancestors)) != len(ancestors)
            or ancestors[0] != px4['pid'] or ancestors[1] != px4['parent_pid']
            or ancestors[-1] != manager['pid']):
        raise RuntimeError('old PX4 binary/ancestry identity is invalid')
    if (set(parameters) != set(PARAMETERS)
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in parameters.values())):
        raise RuntimeError('old fixed parameter contract is invalid')
    assert_stopped(manager, 'manager', probe)
    assert_stopped(px4, 'PX4', probe)
    return {'source': str(source), 'source_sha256': original_hash,
            'controller_sha256': controller_hash, 'old_run_directory': str(old_run),
            'old_manager': manager, 'old_px4': px4, 'parameters': parameters}


def fresh_ground_guard(state):
    ground_guard(state)
    if (state.get('gazebo') or {}).get('available') is not True:
        raise RuntimeError('recovery requires actual Gazebo availability')
    if type(state.get('landed_state')) is not int or state['landed_state'] != 1:
        raise RuntimeError('actual landed_state must be integer ON_GROUND')
    evidence = state.get('landing_readiness') or {}
    if (evidence.get('ready') is not True
            or evidence.get('source') != 'actual_MAVROS_State_and_ExtendedState_receipts'
            or evidence.get('connected') is not True or evidence.get('armed') is not False
            or type(evidence.get('landed_state')) is not int or evidence['landed_state'] != 1):
        raise RuntimeError('recovery requires fresh actual FCU ground/disarmed receipts')
    for field in ('heartbeat_age_wall_s', 'extended_state_age_wall_s'):
        age = evidence.get(field)
        if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= 3:
            raise RuntimeError('recovery FCU receipt is stale or invalid: ' + field)


def recover(source, run_directory, px4_directory, output, *, client, manager_reader,
            px4_reader, command_runner, probe=process_record, system_root=None):
    source_evidence = validate_source(source, px4_directory, system_root, probe)
    run = Path(run_directory).resolve()
    if run.parent != Path(system_root or ROOT / 'logs/system').resolve() or run == Path(source_evidence['old_run_directory']):
        raise RuntimeError('recovery requires a distinct new startup run')
    if (run / 'ev-profile/original.json').exists():
        raise RuntimeError('current profile was already captured; recovery is startup-only')
    if (run / 'ev-profile.txt').read_text().strip() not in ('shadow', 'gps_fused'):
        raise RuntimeError('startup recovery cannot alter a pre-EKF no_gps experiment')
    manager = manager_reader(run)
    if manager is None:
        raise RuntimeError('no live new owned simulation manager')
    if (manager.get('run_directory') != str(run)
            or manager.get('script') != str(ROOT / 'scripts/start_system.sh')
            or manager == source_evidence['old_manager']):
        raise RuntimeError('new manager identity is invalid')
    binary = Path(px4_directory).resolve() / 'build/px4_sitl_default/bin/px4-param'
    def stable(value):
        return {k: v for k, v in value.items() if k != 'state'}
    px4 = stable(px4_reader(manager, binary))
    if px4 == source_evidence['old_px4']:
        raise RuntimeError('recovery must use a fresh owned PX4 daemon')

    def guard():
        if validate_source(source, px4_directory, system_root, probe) != source_evidence:
            raise RuntimeError('old recovery source changed')
        if manager_reader(run) != manager or stable(px4_reader(manager, binary)) != px4:
            raise RuntimeError('new manager/PX4 identity changed')
        fresh_ground_guard(client.state())

    guard()
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    # Preserve the exact old record bytes alongside its provenance, never edit it.
    _, _, raw = read_record(source_evidence['source'])
    with (output / 'recovered-original.json').open('xb') as stream:
        stream.write(raw)
    result = {'schema_version': 1, 'source': 'explicit_stopped_manager_profile_recovery',
              'source_evidence': source_evidence, 'manager': manager, 'px4': px4,
              'target': source_evidence['parameters'], 'success': False,
              'arming_requested': False, 'arming_checks_changed': False,
              'current_profile_target_changed': False, 'fusion_proven': False}
    original = None
    with (output / 'commands.jsonl').open('x', buffering=1) as journal:
        def command(*words):
            guard()
            record = command_runner([str(binary), *words], 2)
            record['wall_s'] = time.time()
            journal.write(json.dumps(record, allow_nan=False) + '\n')
            if record.get('return_code') != 0 or record.get('error'):
                raise RuntimeError('recovery parameter command failed: ' + ' '.join(words))
            guard()
            return record.get('stdout', '')
        try:
            original = {name: parse_value(command('show', name), name) for name in PARAMETERS}
            with (output / 'original.json').open('x') as stream:
                json.dump({'manager': manager, 'px4': px4, 'parameters': original}, stream, indent=2)
                stream.write('\n')
            for name in PARAMETERS:
                value = source_evidence['parameters'][name]
                if original[name] != value:
                    command('set', name, str(value))
                observed = parse_value(command('show', name), name)
                if not math.isclose(observed, value, rel_tol=1e-4, abs_tol=1e-4):
                    raise RuntimeError('recovery parameter readback mismatch: ' + name)
            result['actual'] = {name: parse_value(command('show', name), name) for name in PARAMETERS}
            if result['actual'] != result['target']:
                raise RuntimeError('recovery final readback differs from old actual originals')
            result['success'] = True
        except Exception as exc:
            result.update(error_type=type(exc).__name__, error=str(exc))
            if original is not None:
                try:
                    for name in PARAMETERS:
                        command('set', name, str(original[name]))
                    result['rollback_readback'] = {name: parse_value(command('show', name), name) for name in PARAMETERS}
                    result['rolled_back'] = result['rollback_readback'] == original
                except Exception as rollback:
                    result['rollback_error'] = str(rollback)
        finally:
            with (output / 'result.json').open('x') as stream:
                json.dump(result, stream, indent=2, allow_nan=False)
                stream.write('\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--validate-source', action='store_true', help='Read-only stopped-owner/record validation')
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--run-directory', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--px4-directory', type=Path, default=Path(os.getenv('PX4_DIR', '/home/dut/PX4-Autopilot')))
    args = parser.parse_args(argv)
    if args.validate_source and args.apply:
        parser.error('choose --validate-source or --apply')
    if not args.apply:
        evidence = validate_source(args.source, args.px4_directory)
        print(json.dumps({'apply': False, 'source_validated': True, **evidence}, indent=2))
        return 0
    if args.run_directory is None or args.output is None:
        parser.error('--apply requires --run-directory and new --output')
    from client import BridgeClient
    from scripts.run_reliability_suite import process_identity, owned_px4_process
    from scripts.record_ev_fusion import bounded_command
    result = recover(args.source, args.run_directory, args.px4_directory, args.output,
                     client=BridgeClient(), manager_reader=process_identity,
                     px4_reader=owned_px4_process, command_runner=bounded_command)
    print(json.dumps(result, indent=2))
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
