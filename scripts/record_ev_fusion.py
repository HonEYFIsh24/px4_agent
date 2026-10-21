#!/usr/bin/env python3
"""Read-only, bounded PX4 SITL evidence recording. Default only prints a plan.

The recorder neither publishes ROS/MAVLink nor changes parameters or flight.
EV reception and control-status flags alone are never labelled fusion proof:
actual aid-source samples with advancing time_last_fuse are recorded separately.
Topics are sampled asynchronously; raw stdout and individual wall times remain
in the JSONL evidence. Polls do not establish a continuous-time safety guarantee.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PX4 = Path('/home/dut/PX4-Autopilot')
TOPICS = (
    'vehicle_visual_odometry', 'estimator_status_flags', 'estimator_selector_status',
    'estimator_aid_src_ev_pos', 'estimator_aid_src_ev_vel',
    'estimator_aid_src_ev_hgt', 'estimator_aid_src_ev_yaw',
    'estimator_event_flags', 'vehicle_local_position', 'vehicle_attitude',
    'vehicle_status', 'failsafe_flags', 'timesync_status',
)
COMPONENTS = {'pos': ('cs_ev_pos', 'estimator_aid_src_ev_pos'),
              'vel': ('cs_ev_vel', 'estimator_aid_src_ev_vel'),
              'hgt': ('cs_ev_hgt', 'estimator_aid_src_ev_hgt'),
              'yaw': ('cs_ev_yaw', 'estimator_aid_src_ev_yaw')}
MULTI_INSTANCE_TOPICS = {'estimator_status_flags', 'estimator_event_flags',
                        *(topic for flag,topic in COMPONENTS.values())}
BOOL_FIELDS = {'fusion_enabled', 'innovation_rejected', 'fused', 'xy_valid', 'z_valid',
               'v_xy_valid', 'v_z_valid', 'xy_global', 'z_global', 'dead_reckoning',
               'heading_good_for_control', 'vision_data_stopped', 'failsafe',
               'angular_velocity_invalid', 'attitude_invalid', 'local_altitude_invalid',
               'local_position_invalid', 'local_velocity_invalid', 'offboard_control_signal_lost'}


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def stable_px4_identity(identity):
    # Process scheduling state R/S changes legitimately between reads.
    return {key: value for key, value in identity.items() if key != 'state'}


def topic_instance(topic, estimator_instance):
    """PX4 selector/vehicle outputs are global; EKF diagnostics are per instance."""
    return estimator_instance if topic in MULTI_INSTANCE_TOPICS else 0


def usable_topic(record, estimator_instance):
    expected=topic_instance(record.get('topic'),estimator_instance)
    actual=record.get('instance',expected)
    return (type(record.get('return_code')) is int and record['return_code']==0
            and record.get('published') is True and isinstance(record.get('fields'),dict)
            and type(actual) is int and actual==expected)


def scalar(value):
    value = value.strip()
    if value.lower() in ('true', 'false'):
        return value.lower() == 'true'
    first = value.split()[0] if value else ''
    try:
        number = float(first)
        if not math.isfinite(number):
            return None
        return int(first) if re.fullmatch(r'[+-]?\d+', first) else number
    except ValueError:
        return value


def parse_listener(stdout):
    """Parse generated PX4 listener values while retaining raw output elsewhere."""
    text = re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', stdout)
    fields = {}
    for line in text.splitlines():
        match = re.match(r'^\s*([a-z][a-z0-9_]*):\s*(.*?)\s*$', line)
        if not match:
            continue
        key, raw = match.groups()
        if raw.startswith('[') and ']' in raw:
            fields[key] = [scalar(v) for v in raw[1:raw.index(']')].split(',')]
        else:
            value = scalar(raw)
            if ((key.startswith(('cs_', 'fs_', 'reject_')) or key in BOOL_FIELDS)
                    and isinstance(value, (int, float)) and value in (0, 1)):
                value = bool(value)
            fields[key] = value
    return {'published': bool(fields) and 'never published' not in text.lower(),
            'fields': fields}


def parse_parameters(stdout):
    # PX4 param show: "x + EKF2_EV_CTRL [136,145] : 5". Preserve unknown
    # formats in stdout rather than inventing defaults.
    values = {}
    for line in stdout.splitlines():
        match = re.search(r'\b(EKF2_[A-Z0-9_]+)\b[^:]*:\s*(\S+)', line)
        if match:
            values[match.group(1)] = scalar(match.group(2))
    return values


def bounded_command(command, timeout, runner=subprocess.run):
    if not 0 < timeout <= 2:
        raise ValueError('Read-only subprocess timeout must be within (0, 2] seconds')
    started = time.monotonic()
    record = {'command': list(map(str, command)), 'started_at_wall_s': time.time(),
              'timeout_wall_s': timeout, 'return_code': None, 'stdout': '', 'stderr': ''}
    try:
        result = runner(record['command'], stdin=subprocess.DEVNULL, capture_output=True,
                        text=True, timeout=timeout, check=False, cwd=str(ROOT))
        record.update(return_code=result.returncode, stdout=result.stdout[-160000:],
                      stderr=result.stderr[-16000:])
    except (OSError, subprocess.TimeoutExpired) as exc:
        def decoded(value):
            return value.decode(errors='replace') if isinstance(value, bytes) else value or ''
        record.update(error_type=type(exc).__name__, error=str(exc),
                      stdout=decoded(getattr(exc, 'stdout', ''))[-160000:],
                      stderr=decoded(getattr(exc, 'stderr', ''))[-16000:])
    record.update(completed_at_wall_s=time.time(), duration_wall_s=time.monotonic()-started)
    return record


def summary(records, instance=0):
    """Summarize observed evidence without upgrading configuration to fusion."""
    by_topic = {}
    for record in records:
        if record.get('kind') == 'topic':
            by_topic.setdefault(record['topic'], []).append(record)
    result = {'source': 'actual_owned_local_px4_listener', 'read_only': True,
              'actual_model_call': False, 'flight_command_sent': False,
              'estimator_instance_requested': instance, 'components': {},
              'topic_statistics': {}, 'fusion_proof_definition':
              'EV control flag observed and at least two distinct successful aid-source samples '
              'with increasing time_last_fuse; asynchronous samples, no continuous-time guarantee'}
    for topic, entries in by_topic.items():
        published = [r for r in entries if usable_topic(r,instance)]
        stamps = {r['fields'].get('timestamp') for r in published}
        stamps.discard(None)
        result['topic_statistics'][topic] = {'poll_count': len(entries), 'published_count': len(published),
            'distinct_timestamp_count': len(stamps),
            'timeout_or_error_count': sum(r.get('error_type') is not None or r.get('return_code') != 0 for r in entries)}
    selector_entries=[r for r in by_topic.get('estimator_selector_status',[]) if usable_topic(r,instance)]
    selector_instances = sorted({r['fields']['primary_instance'] for r in selector_entries
        if type(r['fields'].get('primary_instance')) is int and 0<=r['fields']['primary_instance']<4})
    result['invalid_published_selector_count']=sum(type(r['fields'].get('primary_instance')) is not int
        or not 0<=r['fields']['primary_instance']<4 for r in selector_entries)
    result['primary_estimator_instances_observed'] = selector_instances
    result['requested_instance_is_observed_primary'] = (bool(selector_instances) and selector_instances == [instance]
        and result['invalid_published_selector_count']==0)
    flag_entries = [r for r in by_topic.get('estimator_status_flags', []) if usable_topic(r,instance)]
    for component, (flag, topic) in COMPONENTS.items():
        entries = [r for r in by_topic.get(topic, []) if usable_topic(r,instance)]
        unique = {}
        for entry in entries:
            fields = entry['fields']
            stamp, last_fuse = fields.get('timestamp_sample'), fields.get('time_last_fuse')
            if (type(stamp) is int and stamp > 0 and type(last_fuse) is int
                    and last_fuse > 0 and fields.get('fusion_enabled') is True
                    and fields.get('fused') is True and fields.get('innovation_rejected') is False):
                unique[stamp] = last_fuse
        ordered = sorted(unique.items())
        advancing = sum(b[1] > a[1] for a, b in zip(ordered, ordered[1:]))
        flag_seen = any(r.get('return_code') == 0 and r.get('published') and r.get('fields', {}).get(flag) is True
                        for r in flag_entries)
        result['components'][component] = {'control_flag_observed': flag_seen,
            'distinct_successful_aid_sample_count': len(unique), 'advancing_time_last_fuse_count': advancing,
            'innovation_rejected_poll_count': sum(r['fields'].get('innovation_rejected') is True for r in entries),
            'fusion_observed_in_requested_instance': flag_seen and advancing > 0,
            'fusion_observed_in_selected_instance': flag_seen and advancing > 0
                and result['requested_instance_is_observed_primary']}
    for flag in ('cs_gps', 'cs_gps_hgt', 'cs_baro_hgt', 'cs_mag_hdg', 'cs_mag_3d', 'cs_fake_pos',
                 'cs_fake_hgt', 'cs_inertial_dead_reckoning'):
        values = [r['fields'][flag] for r in flag_entries if flag in r.get('fields', {})]
        result[flag] = {'observed_true': any(v is True for v in values),
                        'observed_false': any(v is False for v in values), 'poll_count': len(values)}
    result['visual_odometry_reset_counters'] = sorted({r['fields']['reset_counter']
        for r in by_topic.get('vehicle_visual_odometry', []) if usable_topic(r,instance)
        and type(r['fields'].get('reset_counter')) is int})
    received = [r['fields'] for r in by_topic.get('vehicle_visual_odometry', [])
                if usable_topic(r,instance)]
    ages = [(r['timestamp']-r['timestamp_sample'])/1e6 for r in received
            if isinstance(r.get('timestamp'), (int, float)) and isinstance(r.get('timestamp_sample'), (int, float))]
    result['ev_reception_minus_measurement_age_sim_s'] = {'count': len(ages),
        'minimum': min(ages) if ages else None, 'maximum': max(ages) if ages else None,
        'negative_count': sum(age < 0 for age in ages),
        'limitation': 'PX4 timesync substitutes receipt time before convergence; near-zero age alone is not proof of source-time preservation.'}
    parameter_records = [r for r in records if r.get('kind') == 'parameters']
    result['parameters_observed'] = {r['phase']: r.get('values', {}) for r in parameter_records}
    result['local_position_resets'] = {key: sorted({r['fields'][key]
        for r in by_topic.get('vehicle_local_position', []) if usable_topic(r,instance)
        and type(r['fields'].get(key)) is int})
        for key in ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter', 'vxy_reset_counter', 'vz_reset_counter')}
    return result


def plan(args):
    px4 = args.px4_directory.resolve() / 'build/px4_sitl_default/bin'
    return {'schema_version': 1, 'read_only': True, 'default_dry_run': True,
            'record_requested': args.record, 'flight_command_sent': False,
            'duration_wall_s': args.duration, 'requested_poll_interval_wall_s': args.interval,
            'subprocess_timeout_wall_s': args.timeout, 'estimator_instance': args.instance,
            'run_directory': str(args.run_directory.resolve()),
            'topic_instances':{topic:topic_instance(topic,args.instance) for topic in args.topic or TOPICS},
            'commands': [[str(px4/'px4-listener'), topic, '-i', str(topic_instance(topic,args.instance)), '-n', '1']
                         for topic in args.topic or TOPICS],
            'parameter_commands': [[str(px4/'px4-param'), 'show', 'EKF2_*']],
            'limitations': ['Topics are sampled asynchronously.',
                'Default estimator instance 0 may differ from selected primary; selector evidence is required.',
                'Receiving EV or enabling parameters alone does not demonstrate fusion.',
                'No API, ROS, MAVLink send, takeoff, landing, parameter mutation or simulator start.']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', action='store_true', help='Explicitly run read-only local PX4 listener recording')
    parser.add_argument('--output', type=Path, help='New directory; existing paths are rejected')
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--interval', type=float, default=1.)
    parser.add_argument('--timeout', type=float, default=1.5)
    parser.add_argument('--instance', type=int, choices=range(4), default=0)
    parser.add_argument('--topic', action='append', choices=TOPICS)
    parser.add_argument('--px4-directory', type=Path, default=Path(os.environ.get('PX4_DIR') or DEFAULT_PX4))
    parser.add_argument('--run-directory', type=Path, default=ROOT/'logs/system/latest')
    args = parser.parse_args(argv)
    if (not math.isfinite(args.duration) or not 0 < args.duration <= 7200
            or not math.isfinite(args.interval) or not .5 <= args.interval <= 60
            or not math.isfinite(args.timeout) or not 0 < args.timeout <= 2):
        parser.error('duration: (0,7200], interval: [0.5,60], timeout: (0,2], finite wall seconds')
    specification = plan(args)
    if not args.record:
        print(json.dumps(specification, ensure_ascii=False, indent=2))
        return 0
    if args.output is None:
        parser.error('--record requires --output NEW_DIRECTORY')
    # Ownership validation is imported only for recording; dry-run performs no
    # filesystem/proc/daemon access and the module has no launch side effects.
    from run_reliability_suite import process_identity, owned_px4_process, SafetyError
    manager = process_identity(args.run_directory)
    if manager is None:
        raise SafetyError('No live project manager; recorder does not start one')
    binary_dir = args.px4_directory.resolve()/'build/px4_sitl_default/bin'
    px4_identity = stable_px4_identity(owned_px4_process(manager, binary_dir/'px4-listener'))
    args.output.mkdir(parents=True, exist_ok=False)
    metadata = dict(specification, source='actual_owned_local_px4_listener', started_at_utc=utc_now(),
                    manager=manager, px4_process=px4_identity)
    metadata['px4_source_evidence'] = []
    for relative in ('src/modules/mavlink/mavlink_receiver.cpp', 'src/lib/timesync/Timesync.cpp',
                     'src/modules/mavlink/mavlink_receiver.h','src/modules/ekf2/EKF2.hpp','src/modules/ekf2/EKF2Selector.hpp',
                     'src/modules/ekf2/ekf2_params.c', 'msg/EstimatorStatusFlags.msg',
                     'msg/EstimatorAidSource1d.msg', 'msg/EstimatorAidSource2d.msg', 'msg/EstimatorAidSource3d.msg'):
        source = args.px4_directory/relative
        if source.is_file():
            metadata['px4_source_evidence'].append({'path': str(source.resolve()),
                'sha256': hashlib.sha256(source.read_bytes()).hexdigest()})
    args.output.joinpath('metadata.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False)+'\n')
    records = []
    started = time.monotonic()
    deadline = started+args.duration
    try:
        with args.output.joinpath('samples.jsonl').open('x') as stream, ThreadPoolExecutor(max_workers=4) as workers:
            def append(record):
                records.append(record)
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
                stream.flush()
            parameters = bounded_command([str(binary_dir/'px4-param'), 'show', 'EKF2_*'], args.timeout)
            parameters.update(kind='parameters', phase='start', values=parse_parameters(parameters['stdout']))
            append(parameters)
            cycle = 0
            while time.monotonic() < deadline:
                current_manager = process_identity(args.run_directory)
                if current_manager != manager or stable_px4_identity(owned_px4_process(manager, binary_dir/'px4-listener')) != px4_identity:
                    raise SafetyError('Manager or PX4 daemon changed during read-only recording')
                cycle_started = time.monotonic()
                def poll_topic(topic):
                    selected_instance=topic_instance(topic,args.instance)
                    result = bounded_command([str(binary_dir/'px4-listener'), topic, '-i', str(selected_instance), '-n', '1'], args.timeout)
                    result.update(kind='topic', cycle=cycle, topic=topic, instance=selected_instance,
                                  **parse_listener(result['stdout']))
                    return result
                for record in workers.map(poll_topic, args.topic or TOPICS):
                    append(record)
                cycle += 1
                time.sleep(min(max(0., args.interval-(time.monotonic()-cycle_started)), max(0., deadline-time.monotonic())))
            parameters = bounded_command([str(binary_dir/'px4-param'), 'show', 'EKF2_*'], args.timeout)
            parameters.update(kind='parameters', phase='end', values=parse_parameters(parameters['stdout']))
            append(parameters)
    except (Exception, KeyboardInterrupt) as exc:
        metadata.update(error_type=type(exc).__name__, error=str(exc))
    result = summary(records, args.instance)
    result.update(started_at_utc=metadata['started_at_utc'], finished_at_utc=utc_now(),
                  duration_wall_s=time.monotonic()-started, error=metadata.get('error'),
                  samples_sha256=hashlib.sha256(args.output.joinpath('samples.jsonl').read_bytes()).hexdigest())
    args.output.joinpath('summary.json').write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 1 if result['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
