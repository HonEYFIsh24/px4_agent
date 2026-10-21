"""Check synchronized stereo images and the original body IMU before VINS."""
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, Imu


def check():
    rospy.init_node('vio_input_check', anonymous=True)
    topics = {'left': '/uav/camera/left/image_raw', 'right': '/uav/camera/right/image_raw',
              'imu': '/uav/imu/data_raw'}
    infos = {}
    samples = {name: [] for name in topics}
    offsets = {name: [] for name in topics}
    ignored_stale = {name: 0 for name in topics}
    ignored_order = {name: 0 for name in topics}
    latest = {}

    def collect(name, message):
        age = rospy.Time.now().to_sec() - message.header.stamp.to_sec()
        # Gazebo can send the last cached image to a new subscriber first.
        if not -0.05 <= age < 0.1:
            ignored_stale[name] += 1
            return
        if samples[name] and message.header.stamp.to_sec() <= samples[name][-1]:
            ignored_order[name] += 1
            return
        latest[name] = message
        samples[name].append(message.header.stamp.to_sec())
        offsets[name].append(age)

    subscribers = [rospy.Subscriber(topic, Imu if name == 'imu' else Image,
                    lambda message, key=name: collect(key, message), queue_size=100 if name == 'imu' else 5)
                   for name, topic in topics.items()]
    # Keep image subscriptions alive while awaiting CameraInfo: Gazebo cameras
    # publish on demand, so waiting for CameraInfo alone can never activate them.
    subscribers += [rospy.Subscriber('/uav/camera/' + name + '/camera_info', CameraInfo,
                     lambda message, key=name: infos.__setitem__(key, message), queue_size=1)
                    for name in ('left', 'right')]
    input_deadline = time.monotonic() + 30
    while len(latest) < 3 or len(infos) < 2:
        if time.monotonic() > input_deadline:
            raise RuntimeError('sensor inputs missing: images/IMU={}, CameraInfo={}'.format(list(latest), list(infos)))
        time.sleep(.05)
    sample_start = rospy.Time.now().to_sec()
    sample_start_wall = time.monotonic()
    deadline = sample_start_wall + 60
    while rospy.Time.now().to_sec() - sample_start < 5:
        if time.monotonic() > deadline:
            raise RuntimeError('simulation clock did not advance by 5 seconds within 60 seconds')
        time.sleep(.05)
    for subscriber in subscribers:
        subscriber.unregister()
    report = {}
    for name in topics:
        message = latest[name]
        if name == 'imu':
            report[name] = {'frame_id': message.header.frame_id,
                'acceleration': [message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z],
                'angular_velocity': [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z]}
        else:
            info = infos[name]
            report[name] = {'width': message.width, 'height': message.height,
                'encoding': message.encoding, 'frame_id': message.header.frame_id,
                'K': list(info.K), 'D': list(info.D), 'P': list(info.P)}
            assert (message.width, message.height) == (640, 480)
            assert np.allclose([info.K[0], info.K[4], info.K[2], info.K[5]], [320, 320, 320, 240], atol=.01)
        stamps = samples[name]
        report[name].update(count=len(stamps), ignored_stale_messages=ignored_stale[name],
            ignored_nonmonotonic_messages=ignored_order[name],
            rate_sim_hz=float((len(stamps) - 1) / (stamps[-1] - stamps[0])) if len(stamps) > 1 else 0,
            monotonic=bool(np.all(np.diff(stamps) > 0)),
            median_stamp_age_s=float(np.median(offsets[name])) if offsets[name] else None)
    paired = set(samples['left']) & set(samples['right'])
    report['stereo'] = {'exactly_synchronized_pairs': len(paired),
        'paired_fraction': len(paired) / max(1, len(samples['left']), len(samples['right'])),
        'baseline_m': -infos['right'].P[3] / infos['right'].P[0]}
    report['timing'] = {'sample_duration_sim_s': rospy.Time.now().to_sec() - sample_start,
                        'sample_duration_wall_s': time.monotonic() - sample_start_wall}
    output = Path(__file__).resolve().parent.parent / 'logs' / 'vio' / 'inputs.json'
    output.write_text(json.dumps(report, indent=2))
    bridge = CvBridge()
    left = bridge.imgmsg_to_cv2(latest['left'], 'bgr8')
    right = bridge.imgmsg_to_cv2(latest['right'], 'bgr8')
    cv2.imwrite(str(output.parent / 'camera_preview.png'), left)
    cv2.imwrite(str(output.parent / 'stereo_preview.png'), np.hstack([left, right]))
    print(json.dumps(report, indent=2), flush=True)
    for name in topics:
        assert report[name]['count'] >= (100 if name == 'imu' else 10)
        assert report[name]['monotonic'], 'sensor timestamps are not monotonic'
        assert ignored_order[name] <= .02 * report[name]['count'], 'frequent timestamp disorder'
        assert abs(report[name]['median_stamp_age_s']) < .1, 'sensors do not share simulation time'
        minimum, maximum = (180, 300) if name == 'imu' else (15, 22)
        assert minimum <= report[name]['rate_sim_hz'] <= maximum, 'unsuitable sensor frequency'
    assert np.isclose(report['stereo']['baseline_m'], .12)
    assert len(paired) >= 20 and report['stereo']['paired_fraction'] > .9, 'stereo images are not synchronized'
    print('PASS: stereo calibration, 12 cm baseline, synchronized images and body IMU', flush=True)


if __name__ == '__main__':
    check()
