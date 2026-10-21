"""Execute a camera/IMU flight and compare independent VIO against Gazebo ground truth."""
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import quaternion_matrix, euler_from_quaternion

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from client import BridgeClient


def main():
    rospy.init_node('vio_flight_validation', anonymous=True)
    lock = threading.Lock()
    samples = {'ground_truth': [], 'vio': [], 'tracking': []}
    previous_truth_time = [0.0]

    def record_truth(message):
        if 'iris' not in message.name:
            return
        stamp = rospy.Time.now().to_sec()
        if stamp - previous_truth_time[0] < 0.02:
            return
        previous_truth_time[0] = stamp
        pose = message.pose[message.name.index('iris')]
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        # The original body IMU is 2 cm above the model origin.
        p = np.array([pose.position.x, pose.position.y, pose.position.z]) + quaternion_matrix(q)[:3, :3] @ [0, 0, 0.02]
        with lock:
            samples['ground_truth'].append({'t': stamp, 'p': p.tolist(), 'q': q})

    def record_vio(message):
        pose = message.pose.pose
        with lock:
            samples['vio'].append({'t': message.header.stamp.to_sec(),
                                   'received_at': rospy.Time.now().to_sec(),
                                   'p': [pose.position.x, pose.position.y, pose.position.z],
                                   'q': [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]})

    def record_tracking(message):
        with lock:
            samples['tracking'].append({'t': rospy.Time.now().to_sec(), 'ok': message.data})

    subscribers = [rospy.Subscriber('/gazebo/model_states', ModelStates, record_truth, queue_size=1),
                   rospy.Subscriber('/vins_estimator/odometry', Odometry, record_vio, queue_size=100),
                   rospy.Subscriber('/vins_estimator/tracking_ok', Bool, record_tracking, queue_size=100)]
    client = BridgeClient()
    state = client.state()
    assert state['ready'] and not state['armed'], 'start with a ready, landed simulator'
    initialization_deadline = time.monotonic() + 20
    while not state['vio']['ready']:
        assert time.monotonic() < initialization_deadline, 'VIO must initialize before the stereo flight test: ' + str(state['vio'])
        time.sleep(.2)
        state = client.state()
    time.sleep(1)
    steps = [{'action': 'takeoff', 'altitude': 2},
             {'action': 'goto', 'x': 1.5, 'y': 0, 'altitude': 2},
             {'action': 'goto', 'x': 1.5, 'y': 1.5, 'altitude': 2},
             {'action': 'goto', 'x': -1, 'y': 1.5, 'altitude': 2.5},
             {'action': 'goto', 'x': -1, 'y': -1, 'altitude': 2},
             {'action': 'goto', 'x': 1.5, 'y': -1, 'altitude': 2.5},
             {'action': 'return_home'}, {'action': 'hold', 'duration': 2}, {'action': 'land'}]
    try:
        result = client.execute(steps, 'Stereo cameras + existing body IMU VIO validation',
                                lambda m: print('Flight step', m['step'], m.get('action'), flush=True))
        assert result['mission']['status'] == 'succeeded', result
        landing_time = rospy.Time.now().to_sec()
        print('Landed and disarmed; observing stationary VIO for 30 simulation seconds', flush=True)
        deadline = time.monotonic() + 300
        while rospy.Time.now().to_sec() - landing_time < 30:
            assert time.monotonic() < deadline, 'simulation clock stopped after landing'
            time.sleep(.2)
    finally:
        if client.state()['armed']:
            client.wait_for(client.request('/land', {}))
        for subscriber in subscribers:
            subscriber.unregister()
        output = ROOT / 'logs' / 'vio'
        (output / 'trajectory_samples.json').write_text(json.dumps(samples))
    assert len(samples['vio']) >= 50, 'VIO did not initialize and produce enough optimized poses'
    truth = sorted(samples['ground_truth'], key=lambda s: s['t'])
    vio = [s for s in samples['vio'] if truth[0]['t'] <= s['t'] <= truth[-1]['t']]
    gt_t = np.array([s['t'] for s in truth])
    gt_p = np.array([s['p'] for s in truth])
    times = np.array([s['t'] for s in vio])
    reference = np.column_stack([np.interp(times, gt_t, gt_p[:, index]) for index in range(3)])
    estimated = np.array([s['p'] for s in vio])
    first_truth = truth[int(np.argmin(np.abs(gt_t - times[0])))]
    yaw = euler_from_quaternion(first_truth['q'])[2] - euler_from_quaternion(vio[0]['q'])[2]
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    # Align only the unobservable initial yaw and position. Do not correct scale.
    aligned = (estimated - estimated[0]) @ rotation.T + reference[0]
    errors = np.linalg.norm(aligned - reference, axis=1)
    ground_mask = times >= landing_time
    ground_errors = errors[ground_mask]
    tracking_after_init = [s for s in samples['tracking'] if s['t'] >= times[0] + 0.5]
    tracking_resets = sum(not s['ok'] for s in tracking_after_init)
    report = {'mission_id': result['mission']['id'], 'mission_status': result['mission']['status'],
              'vio_optimized_poses': len(vio), 'duration_sim_s': float(times[-1] - times[0]),
              'optimized_rate_hz': float(1 / np.median(np.diff(times))),
              'median_output_lag_s': float(np.median([s['received_at'] - s['t'] for s in vio])),
              'position_rmse_m': float(np.sqrt(np.mean(errors ** 2))),
              'position_max_error_m': float(errors.max()), 'final_position_error_m': float(errors[-1]),
              'post_landing_observation_s': 30,
              'post_landing_max_error_m': float(ground_errors.max()) if len(ground_errors) else None,
              'post_landing_drift_m': float(np.linalg.norm(aligned[-1] - aligned[np.flatnonzero(ground_mask)[0]])) if ground_mask.any() else None,
              'tracking_invalid_messages_after_initialization': tracking_resets,
              'agent_vio_ready_after_observation': client.state()['vio']['ready'],
              'alignment': 'initial yaw and translation only; no scale correction',
              'vio_inputs': ['/uav/camera/left/image_raw', '/uav/camera/right/image_raw', '/uav/imu/data_raw'],
              'ground_truth_used_only_for_evaluation': True, 'armed_after_landing': result['armed']}
    report['validation_passed'] = bool(np.isfinite(errors).all()
        and report['position_rmse_m'] < 0.5 and report['position_max_error_m'] < 1.0
        and len(ground_errors) > 200 and tracking_resets == 0
        and report['post_landing_drift_m'] < 0.3 and report['agent_vio_ready_after_observation'])
    (output / 'validation.json').write_text(json.dumps(report, indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].plot(reference[:, 0], reference[:, 1], label='Gazebo reference')
    axes[0].plot(aligned[:, 0], aligned[:, 1], label='VINS-Fusion VIO')
    axes[0].set(xlabel='East / m', ylabel='North / m', title='Camera + body IMU trajectory')
    axes[0].axis('equal')
    axes[0].legend()
    axes[1].plot(times - times[0], errors)
    axes[1].axvline(landing_time - times[0], color='gray', linestyle='--', label='Landed / disarmed')
    axes[1].legend()
    axes[1].set(xlabel='Simulation time / s', ylabel='Position error / m', title='No scale correction')
    for axis in axes:
        axis.grid(alpha=0.3)
    figure.tight_layout()
    figure.savefig(str(output / 'trajectory_validation.png'), dpi=180)
    print(json.dumps(report, indent=2), flush=True)
    assert report['validation_passed'], 'VIO trajectory, stationary drift, or tracking health validation failed'
    print('PASS: independent stereo-inertial VIO, metric trajectory and completed landing', flush=True)


if __name__ == '__main__':
    main()
