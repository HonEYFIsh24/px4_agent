import math
import unittest
from yaw_control import YawArrival, error, run_rotation


class YawControlTests(unittest.TestCase):
    def test_short_rotation_crosses_pi_with_rate_limit(self):
        state = YawArrival(math.radians(-179), 'epoch', [0, 0, 3])
        start = math.radians(179)
        result = state.next_setpoint(start, .1)
        self.assertLess(abs(error(math.radians(-179), result)), abs(error(math.radians(-179), start)))
        self.assertLessEqual(abs(error(result, start)), .05)

    def test_arrival_requires_repeated_fresh_actual_poses_within_tolerance(self):
        state = YawArrival(1., 'epoch', [0, 0, 3])
        samples = []
        for i in range(7):
            samples.append(state.observe(actual_rad=1.02, position=[0, 0, 3], identity='epoch',
                stamp_s=10+i*.2, received_age_s=.01, wall_s=20+i*.2))
        self.assertEqual(samples[:5], [False]*5)
        self.assertTrue(samples[-1])

    def test_reset_stale_pose_and_drift_are_rejected(self):
        for changes in ({'identity': 'new'}, {'received_age_s': 1.}, {'position': [.36, 0, 3]}):
            state = YawArrival(1., 'epoch', [0, 0, 3])
            sample = dict(actual_rad=1., position=[0, 0, 3], identity='epoch', stamp_s=1., received_age_s=.01, wall_s=1.)
            sample.update(changes)
            with self.assertRaises(ValueError): state.observe(**sample)

    def test_repeated_pose_cannot_satisfy_dwell(self):
        state = YawArrival(1., 'epoch', [0, 0, 3])
        state.observe(actual_rad=1., position=[0, 0, 3], identity='epoch', stamp_s=1., received_age_s=0., wall_s=1.)
        with self.assertRaises(ValueError):
            state.observe(actual_rad=1., position=[0, 0, 3], identity='epoch', stamp_s=1., received_age_s=0., wall_s=1.2)

    def test_runner_waits_for_measured_heading_not_commanded_heading(self):
        now = [0.]; command = [0.]; actual = [0.]; count = [0]
        def read():
            return dict(actual_rad=actual[0], position=[0., 0., 3.], identity='epoch',
                        stamp_s=now[0]+1., received_age_s=0.)
        def wait(dt):
            now[0] += dt; count[0] += 1
            if count[0] > 10:
                actual[0] = command[0]
        policy = YawArrival(.3, 'epoch', [0, 0, 3], dwell_s=.2)
        result = run_rotation(policy, read_pose=read, set_heading=lambda v: command.__setitem__(0, v),
                              checkpoint=lambda: None, wait=wait, clock=lambda: now[0])
        self.assertGreater(count[0], 10)
        self.assertLess(abs(error(.3, result['actual_rad'])), .1)

    def test_runner_rejects_stale_or_reset_even_with_repeated_stamp(self):
        now = [0.]; calls = [0]
        def read():
            calls[0] += 1
            return dict(actual_rad=0., position=[0., 0., 3.], identity='epoch' if calls[0] < 3 else 'reset',
                        stamp_s=1., received_age_s=0.)
        def wait(dt): now[0] += dt
        with self.assertRaisesRegex(ValueError, 'identity'):
            run_rotation(YawArrival(.5, 'epoch', [0, 0, 3]), read_pose=read, set_heading=lambda v: None,
                         checkpoint=lambda: None, wait=wait, clock=lambda: now[0])

    def test_runner_cannot_report_arrival_if_vehicle_does_not_turn(self):
        now = [0.]
        def read():
            return dict(actual_rad=0., position=[0., 0., 3.], identity='epoch',
                        stamp_s=1.+now[0], received_age_s=0.)
        def wait(dt): now[0] += dt
        with self.assertRaisesRegex(ValueError, 'timed out'):
            run_rotation(YawArrival(.8, 'epoch', [0, 0, 3]), read_pose=read, set_heading=lambda v: None,
                         checkpoint=lambda: None, wait=wait, clock=lambda: now[0], timeout_s=2.)
