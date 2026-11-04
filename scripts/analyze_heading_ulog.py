#!/usr/bin/env python3
"""Read PX4 ULog heading-reset evidence; never connects to or controls a vehicle."""
import argparse
import hashlib
import json
from pathlib import Path


def value(data, key, index):
    item = data[key][index]
    return item.item() if hasattr(item, 'item') else item


def changes(data, field):
    values = data[field]
    return [i for i in range(1, len(values)) if values[i] != values[i-1]]


def previous_sample(data, stamp_us):
    # A status sample may lag the estimator output; never use a future sample.
    import bisect
    stamps = data['timestamp_sample']
    if any(b < a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('flag source timestamps are not monotonic')
    index = bisect.bisect_right(stamps, stamp_us)-1
    return index if index >= 0 else None


def compare_visual_yaw_to_groundtruth(visual, truth):
    """Compare relative yaw after removing the median offset; offline truth only."""
    import numpy as np
    def yaw(data):
        w, x, y, z = (np.asarray(data['q[{}]'.format(i)], dtype=float) for i in range(4))
        return np.unwrap(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    vt = np.asarray(visual['timestamp_sample'], dtype=float)/1e6
    gt = np.asarray(truth['timestamp'], dtype=float)/1e6
    if len(gt) < 2 or np.any(np.diff(gt) <= 0) or np.any(np.diff(vt) <= 0):
        raise ValueError('visual or groundtruth timestamps are not strictly increasing')
    mask = ((vt > 10) & (vt >= gt[0]) & (vt <= gt[-1])
            & (np.asarray(visual['quality']) > 0))
    if not np.any(mask):
        raise ValueError('no overlapping healthy visual/groundtruth samples')
    relative = np.unwrap(yaw(visual)[mask]-np.interp(vt[mask], gt, yaw(truth)))
    if not np.all(np.isfinite(relative)):
        raise ValueError('nonfinite yaw comparison')
    median = float(np.median(relative))
    residual = np.abs(relative-median)
    return {'samples': int(mask.sum()), 'first_source_s': float(vt[mask][0]),
            'last_source_s': float(vt[mask][-1]), 'median_offset_rad': median,
            'absolute_residual_p95_rad': float(np.percentile(residual,95)),
            'maximum_absolute_residual_rad': float(np.max(residual)),
            'scope': 'offline Gazebo truth; healthy samples only; median offset removed; not absolute calibration'}


def replay_ground_heading_reference(visual, truth, attitude, status, flags):
    """Offline experiment only: derive offset on ground; score later truth samples."""
    import numpy as np
    def yaw(data):
        w, x, y, z = (np.asarray(data['q[{}]'.format(i)], dtype=float) for i in range(4))
        return np.unwrap(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    t = np.asarray(visual['timestamp_sample'], dtype=float)/1e6
    tg = np.asarray(truth['timestamp'], dtype=float)/1e6
    ta = np.asarray(attitude['timestamp_sample'], dtype=float)/1e6
    ts = np.asarray(status['timestamp'], dtype=float)/1e6
    tf = np.asarray(flags['timestamp_sample'], dtype=float)/1e6
    if any(len(x)<2 or np.any(np.diff(x)<=0) for x in (t,tg,ta,ts,tf)):
        raise ValueError('required timestamps must be strictly increasing')
    si=np.searchsorted(ts,t,side='right')-1
    fi=np.searchsorted(tf,t,side='right')-1
    ai=np.searchsorted(ta,t,side='right')-1
    valid=(si>=0)&(fi>=0)&(ai>=0)&(t<=ta[-1])&(np.asarray(visual['quality'])>0)
    si=np.maximum(si,0);fi=np.maximum(fi,0);ai=np.maximum(ai,0)
    # The calibration only sees PX4 attitude already published at the visual
    # source time. A later sample must never leak future information into it.
    valid &= (t-ts[si]<1)&(t-tf[fi]<2)&(t-ta[ai]<=.05)
    ground=(np.asarray(status['arming_state'])[si]==1)&(~np.asarray(flags['cs_in_air'],dtype=bool)[fi])
    aligned=np.asarray(flags['cs_yaw_align'],dtype=bool)[fi]
    magnetic=(np.asarray(flags['cs_mag_hdg'],dtype=bool)[fi]|np.asarray(flags['cs_mag_3d'],dtype=bool)[fi])
    magnetic_healthy=(~np.asarray(flags['cs_mag_fault'],dtype=bool)[fi]
                      & ~np.asarray(flags['cs_mag_field_disturbed'],dtype=bool)[fi])
    calibration=valid&ground&aligned&magnetic&magnetic_healthy&(t>=10)&(t<=15)
    heldout=valid&(t>15)&(t>=tg[0])&(t<=tg[-1])
    if np.count_nonzero(calibration)<20 or np.count_nonzero(heldout)<20:
        raise ValueError('insufficient independent ground or held-out samples')
    if (len(set(np.asarray(visual['reset_counter'])[calibration])) != 1
            or len(set(np.asarray(attitude['quat_reset_counter'])[ai[calibration]])) != 1):
        raise ValueError('coordinate reset during ground calibration')
    vision_yaw=yaw(visual)
    px4_yaw=yaw(attitude)[ai]
    # Ground truth is intentionally not used to derive the calibration.
    offset=float(np.median(np.unwrap(px4_yaw[calibration]-vision_yaw[calibration])))
    truth_yaw=np.interp(t,tg,yaw(truth))
    errors=(vision_yaw[heldout]+offset-truth_yaw[heldout]+np.pi)%(2*np.pi)-np.pi
    if not np.isfinite(offset) or not np.all(np.isfinite(errors)):
        raise ValueError('nonfinite ground calibration or held-out error')
    return {'ground_reference_samples':int(calibration.sum()), 'heldout_samples':int(heldout.sum()),
            'ground_px4_minus_visual_yaw_rad':offset,
            'heldout_absolute_truth_error_p95_rad':float(np.percentile(np.abs(errors),95)),
            'heldout_absolute_truth_error_max_rad':float(np.max(np.abs(errors))),
            'heldout_contains_airborne':bool(np.any(np.asarray(flags['cs_in_air'],dtype=bool)[fi[heldout]])),
            'scope':'offline diagnostic only; ground truth scores held-out samples; no online fusion or safety validation'}


def summarize(path):
    try:
        from pyulog import ULog
    except ImportError as exc:
        raise RuntimeError('pyulog is required; use the project ROS Python environment') from exc
    path = Path(path)
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    log = ULog(str(path))
    local = log.get_dataset('vehicle_local_position').data
    attitude = log.get_dataset('vehicle_attitude').data
    flags = log.get_dataset('estimator_status_flags').data
    visual = log.get_dataset('vehicle_visual_odometry').data
    resets = []
    for i in changes(local, 'heading_reset_counter'):
        stamp = int(value(local, 'timestamp_sample', i))
        j = previous_sample(flags, stamp)
        events = [k for k in changes(attitude, 'quat_reset_counter')
                  if 0 <= int(value(attitude, 'timestamp_sample', k))-stamp <= 100_000]
        record = {'source_time_s': stamp/1e6,
                  'old_heading_reset_counter': int(value(local, 'heading_reset_counter', i-1)),
                  'new_heading_reset_counter': int(value(local, 'heading_reset_counter', i)),
                  'delta_heading_rad': float(value(local, 'delta_heading', i)),
                  'heading_before_rad': float(value(local, 'heading', i-1)),
                  'heading_after_rad': float(value(local, 'heading', i)),
                  'local_z_m': float(value(local, 'z', i)),
                  'attitude_reset_source_time_s': int(value(attitude, 'timestamp_sample', events[0]))/1e6 if events else None,
                  'attitude_reset_quaternion_wxyz': [float(value(attitude, 'delta_q_reset[{}]'.format(k), events[0]))
                                                     for k in range(4)] if events else None}
        keys = ('cs_in_air','cs_yaw_align','cs_mag_hdg','cs_mag_3d','cs_ev_yaw','cs_mag_aligned_in_flight')
        record['latest_earlier_flags'] = ({k: bool(value(flags, k, j)) for k in keys} if j is not None else None)
        record['flags_source_time_s'] = int(value(flags, 'timestamp_sample', j))/1e6 if j is not None else None
        resets.append(record)
    stamps = [int(v) for v in visual['timestamp_sample']]
    diffs = [(b-a)/1e6 for a,b in zip(stamps,stamps[1:])]
    try:
        truth_comparison = compare_visual_yaw_to_groundtruth(
            visual, log.get_dataset('vehicle_attitude_groundtruth').data)
    except (KeyError, IndexError, ValueError) as exc:
        truth_comparison = {'available': False, 'reason': str(exc)}
    try:
        ground_reference = replay_ground_heading_reference(visual,
            log.get_dataset('vehicle_attitude_groundtruth').data, attitude,
            log.get_dataset('vehicle_status').data, flags)
    except (KeyError, IndexError, ValueError) as exc:
        ground_reference = {'available': False, 'reason': str(exc)}
    return {'ulog_sha256': digest.hexdigest(), 'duration_s': (log.last_timestamp-log.start_timestamp)/1e6,
            'dropout_count': len(log.dropouts),
            'parameters': {k: log.initial_parameters.get(k) for k in ('EKF2_MAG_TYPE','EKF2_EV_CTRL','EKF2_EV_DELAY','EKF2_GPS_CTRL','EKF2_HGT_REF')},
            'heading_resets': resets,
            'vision': {'sample_count':len(stamps),'nonincreasing_source_timestamps':sum(d<=0 for d in diffs),
                       'maximum_source_gap_s':max(diffs) if diffs else None,
                       'reset_counters': sorted(set(int(v) for v in visual['reset_counter']))},
            'visual_yaw_groundtruth_comparison': truth_comparison,
            'ground_heading_reference_replay': ground_reference,
            'limitations': ['Recorded messages are samples; flag times can lag the actual internal EKF event.',
                            'No inference of semantic truth, a safe reconfiguration, or the internal reset call site.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ulog', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.ulog)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # A new file protects historical diagnostics from accidental overwrite.
    with args.output.open('x', encoding='utf-8') as output:
        json.dump(result, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write('\n')
    print(json.dumps({'output': str(args.output), 'heading_reset_count': len(result['heading_resets']),
                      'dropout_count': result['dropout_count']}))


if __name__ == '__main__':
    main()
