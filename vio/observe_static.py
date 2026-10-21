"""Verify 30 simulation seconds of initialized, stationary VIO without ground truth input."""
import json
import math
import time
from pathlib import Path
from urllib.request import urlopen

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray


def main():
    rospy.init_node('uav_vio_static_verification', anonymous=True)
    poses, biases, health = [], [], []
    started = [False]
    def pose_callback(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        poses.append([msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    def health_callback(msg):
        if msg.data:
            started[0] = True
        if started[0]:
            health.append([rospy.Time.now().to_sec(), bool(msg.data)])
    subscribers = [
        rospy.Subscriber('/vins_estimator/odometry', Odometry, pose_callback, queue_size=1000),
        rospy.Subscriber('/vins_estimator/imu_bias', Float64MultiArray,
                         lambda msg: biases.append([rospy.Time.now().to_sec(), *msg.data]), queue_size=1000),
        rospy.Subscriber('/vins_estimator/tracking_ok', Bool, health_callback, queue_size=1000),
    ]
    deadline = time.monotonic() + 60
    while not (started[0] and poses) and time.monotonic() < deadline:
        time.sleep(.05)
    if not (started[0] and poses):
        raise RuntimeError('VINS did not initialize within 60 wall seconds')
    start_sim = poses[-1][0]
    start_wall = time.monotonic()
    while rospy.Time.now().to_sec() - start_sim < 30 and time.monotonic() - start_wall < 180:
        time.sleep(.05)
    for subscriber in subscribers:
        subscriber.unregister()
    poses = np.asarray([p for p in poses if p[0] >= start_sim])
    bias_array = np.asarray([b for b in biases if b[0] >= start_sim])
    invalid = sum(not h[1] for h in health if h[0] >= start_sim)
    state = json.load(urlopen('http://127.0.0.1:8765/state', timeout=5))
    drift = np.linalg.norm(poses[:, 1:4] - poses[0, 1:4], axis=1)
    duration = float(poses[-1, 0] - poses[0, 0])
    report = {
        'source': '/vins_estimator/odometry',
        'imu_source': '/uav/imu/data_raw (original Iris plugin, in-process ROS forwarding)',
        'start_sim_s': float(start_sim),
        'duration_sim_s': duration,
        'duration_wall_s': time.monotonic() - start_wall,
        'optimized_poses': int(len(poses)),
        'optimized_rate_hz': float((len(poses) - 1) / duration) if duration else 0,
        'maximum_stationary_drift_m': float(drift.max()),
        'tracking_invalid_messages': invalid,
        'acceleration_bias_max_norm_m_s2': float(np.linalg.norm(bias_array[:, 1:4], axis=1).max()) if len(bias_array) else None,
        'gyro_bias_max_norm_rad_s': float(np.linalg.norm(bias_array[:, 4:7], axis=1).max()) if len(bias_array) else None,
        'agent_vio_ready_after_observation': state['vio']['ready'],
        'armed_after_observation': state['armed'],
        'ground_truth_used_as_estimator_input': False,
    }
    report['validation_passed'] = bool(duration >= 29.5 and len(poses) > 200
        and np.all(np.isfinite(poses)) and drift.max() < .3 and invalid == 0
        and state['vio']['ready'] and not state['armed'])
    root = Path(__file__).resolve().parent.parent / 'logs/vio'
    np.savetxt(root / 'static_poses.csv', poses, delimiter=',', header='sim_s,x,y,z,qx,qy,qz,qw', comments='')
    if len(bias_array):
        np.savetxt(root / 'static_biases.csv', bias_array, delimiter=',', header='sim_s,bax,bay,baz,bgx,bgy,bgz', comments='')
    (root / 'static_validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if not report['validation_passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
