"""Record a landed 30-second camera/native-IMU VIO stability check."""
import json
import sys
import time
from pathlib import Path

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from client import BridgeClient


def main():
    rospy.init_node('vio_stationary_validation', anonymous=True)
    client = BridgeClient()
    assert client.state()['ready'] and not client.state()['armed'], 'land and disarm first'
    samples = {'vio': [], 'bias': [], 'tracking': []}

    def pose(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        samples['vio'].append({'t': message.header.stamp.to_sec(),
            'received_at': rospy.Time.now().to_sec(),
            'p': [p.x, p.y, p.z], 'q': [q.x, q.y, q.z, q.w]})

    def bias(message):
        samples['bias'].append({'t': rospy.Time.now().to_sec(), 'bias': list(message.data)})

    def tracking(message):
        samples['tracking'].append({'t': rospy.Time.now().to_sec(), 'ok': message.data})

    subscriptions = [rospy.Subscriber('/vins_estimator/odometry', Odometry, pose, queue_size=100),
        rospy.Subscriber('/vins_estimator/imu_bias', Float64MultiArray, bias, queue_size=100),
        rospy.Subscriber('/vins_estimator/tracking_ok', Bool, tracking, queue_size=100)]
    deadline = time.monotonic() + 300
    while not samples['vio'] or not client.state()['vio']['ready']:
        assert time.monotonic() < deadline, 'VIO failed to initialize'
        time.sleep(.1)
    start = rospy.Time.now().to_sec()
    print('Observing initialized landed VIO for 30 simulation seconds', flush=True)
    while rospy.Time.now().to_sec() - start < 30:
        assert time.monotonic() < deadline, 'simulation clock or VIO stalled'
        assert not client.state()['armed'], 'aircraft armed during stationary check'
        time.sleep(.2)
    for subscription in subscriptions:
        subscription.unregister()
    selected = [sample for sample in samples['vio'] if sample['t'] >= start]
    positions = np.array([sample['p'] for sample in selected])
    displacement = np.linalg.norm(positions - positions[0], axis=1)
    invalid = sum(not sample['ok'] for sample in samples['tracking'] if sample['t'] >= start)
    report = {'duration_sim_s': float(selected[-1]['t'] - selected[0]['t']),
        'optimized_poses': len(selected), 'max_stationary_displacement_m': float(displacement.max()),
        'final_stationary_displacement_m': float(displacement[-1]),
        'tracking_invalid_messages': invalid, 'agent_vio_ready': client.state()['vio']['ready'],
        'last_bias_acceleration_m_s2_then_gyro_rad_s': samples['bias'][-1]['bias'] if samples['bias'] else None,
        'inputs': ['/uav/camera/left/image_raw', '/uav/camera/right/image_raw', '/uav/imu/data_raw']}
    report['validation_passed'] = bool(len(selected) >= 250 and np.isfinite(positions).all()
        and displacement.max() < .3 and invalid == 0 and report['agent_vio_ready'])
    output = ROOT / 'logs' / 'vio'
    (output / 'stationary_samples.json').write_text(json.dumps(samples))
    (output / 'stationary_validation.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    assert report['validation_passed'], 'stationary VIO validation failed'


if __name__ == '__main__':
    main()
