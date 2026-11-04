#!/usr/bin/env python3
"""Offline landing diagnostic. Gazebo truth is evaluation-only, never flight input."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def down_acceleration(accel_frd, quaternion_wxyz):
    """Rotate specific force from body FRD to NED, then add gravity down."""
    a = np.asarray(accel_frd, dtype=float)
    q = np.asarray(quaternion_wxyz, dtype=float)
    if a.shape[-1] != 3 or q.shape[-1] != 4 or not np.isfinite(a).all() or not np.isfinite(q).all():
        raise ValueError('invalid IMU/attitude')
    if not np.allclose(np.linalg.norm(q, axis=-1), 1., atol=.01):
        raise ValueError('invalid quaternion norm')
    w, x, y, z = np.moveaxis(q, -1, 0)
    return (2*(x*z-w*y)*a[..., 0] + 2*(y*z+w*x)*a[..., 1]
            + (1-2*(x*x+y*y))*a[..., 2] + 9.80665)


def analyze(path, start, end, output):
    if not start < end:
        raise ValueError('invalid time interval')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    u = ULog(str(path))
    datasets = {name: u.get_dataset(name).data for name in (
        'sensor_combined', 'vehicle_attitude', 'vehicle_local_position',
        'vehicle_local_position_groundtruth', 'vehicle_visual_odometry',
        'estimator_aid_src_ev_vel', 'estimator_status_flags')}
    imu, attitude, local, truth, visual, aid, flags = datasets.values()
    mask = lambda d: (d['timestamp']/1e6 >= start) & (d['timestamp']/1e6 <= end)
    im = mask(imu)
    indices = np.flatnonzero(im)
    if not len(indices):
        raise ValueError('no IMU samples in window')
    stamp = imu['timestamp'].astype(np.int64)+imu['accelerometer_timestamp_relative'].astype(np.int64)
    qi = np.searchsorted(attitude['timestamp'], stamp, side='right')-1
    if np.any(qi[im] < 0) or np.max((stamp-attitude['timestamp'][qi])[im]) > 50000:
        raise ValueError('no fresh causal attitude for IMU samples')
    acc = np.column_stack([imu['accelerometer_m_s2[%d]' % j] for j in range(3)])
    quats = np.column_stack([attitude['q[%d]' % j][qi] for j in range(4)])
    down = down_acceleration(acc, quats)
    events = []
    for field in ('heading_reset_counter', 'vxy_reset_counter', 'vz_reset_counter'):
        for i in np.flatnonzero(np.diff(local[field].astype(int)) != 0)+1:
            when = float(local['timestamp'][i])/1e6
            if not start <= when <= end:
                continue
            j = np.searchsorted(aid['timestamp'], local['timestamp'][i], side='right')-1
            before = max(0, j-1)
            events.append(dict(field=field, stamp_sim_s=when,
                before=int(local[field][i-1]), after=int(local[field][i]),
                delta_vz_m_s=float(local['delta_vz'][i]),
                previous_aid_sample_s=float(aid['timestamp_sample'][before])/1e6,
                previous_last_fuse_s=float(aid['time_last_fuse'][before])/1e6,
                previous_vertical_innovation=float(aid['innovation[2]'][before]),
                previous_vertical_test_ratio=float(aid['test_ratio[2]'][before])))
    am = mask(aid)
    truth_v = np.interp(aid['timestamp_sample'][am], truth['timestamp'], truth['vz'])
    ev_error = aid['observation[2]'][am]-truth_v
    peak = indices[np.argmax(down[im])]
    result = dict(ulog=str(Path(path).resolve()), sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        px4_version=u.msg_info_dict.get('ver_sw'), window_sim_s=[start, end],
        ulog_dropout_count=len(u.dropouts),
        imu_peak_down_acceleration_m_s2=float(down[peak]), imu_peak_stamp_sim_s=float(stamp[peak])/1e6,
        imu_clipping_values=[int(x) for x in np.unique(imu['accelerometer_clipping'][im])],
        imu_logged_interval_fraction=float(np.sum(imu['accelerometer_integral_dt'][im])/1e6/(end-start)),
        imu_max_log_gap_s=float(np.max(np.diff(stamp[im]))/1e6),
        ev_vertical_velocity_truth_rmse_m_s=float(np.sqrt(np.mean(ev_error**2))),
        ev_vertical_velocity_truth_p95_abs_m_s=float(np.percentile(np.abs(ev_error),95)),
        ev_reset_counter_values=[int(x) for x in np.unique(visual['reset_counter'][mask(visual)])],
        visual_quality_values=[int(x) for x in np.unique(visual['quality'][mask(visual)])],
        visual_max_source_gap_s=float(np.max(np.diff(visual['timestamp_sample'][mask(visual)]))/1e6),
        velocity_innovation_peak=float(np.max(aid['innovation[2]'][am])),
        velocity_test_ratio_peak=float(np.max(aid['test_ratio[2]'][am])),
        reset_events=events,
        limits=['ULog no dropout does not mean all raw IMU samples were logged; do not integrate as complete contact impulse.',
                'Gazebo truth used only offline. Source timestamps aligned for EV comparison; truth velocity has limited publication precision.',
                'An IMU/contact inconsistency does not uniquely identify a plugin or physics-engine defect.'])
    (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    with (output/'imu-truth-timeline.csv').open('w') as f:
        writer = csv.writer(f)
        writer.writerow(['imu_stamp_sim_s','imu_down_accel_m_s2','truth_vz_ned_m_s','px4_vz_ned_m_s'])
        for i in indices:
            writer.writerow([float(stamp[i])/1e6,float(down[i]),
                float(np.interp(stamp[i],truth['timestamp'],truth['vz'])),
                float(np.interp(stamp[i],local['timestamp'],local['vz']))])
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ulog', type=Path, required=True)
    p.add_argument('--start', type=float, required=True)
    p.add_argument('--end', type=float, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(analyze(a.ulog,a.start,a.end,a.output),indent=2))
