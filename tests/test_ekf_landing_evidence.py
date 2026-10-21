"""Only raw, typed PX4 observations may identify its landing state."""
import copy
import unittest

from vio.ekf_monitor import autopilot_observation_fields, TOPICS


class AutopilotEvidenceTests(unittest.TestCase):
    def status(self, nav=18):
        return {'timestamp': 11000000, 'nav_state_timestamp': 10000000,
                'nav_state': nav, 'arming_state': 2, 'failsafe': False}

    def flags(self):
        return {'timestamp': 11000000, 'angular_velocity_invalid': False,
                'attitude_invalid': False, 'local_altitude_invalid': False,
                'local_position_invalid': True, 'local_velocity_invalid': True,
                'offboard_control_signal_lost': True}

    def test_raw_navigation_separates_same_heartbeat_label(self):
        land = autopilot_observation_fields(self.status(), self.flags())
        descend = autopilot_observation_fields(self.status(12), self.flags())
        self.assertEqual(land['navigation_state_kind'], 'position_landing')
        self.assertEqual(descend['navigation_state_kind'], 'descend_without_horizontal_control')
        self.assertTrue(descend['flags']['local_position_invalid'])
        self.assertFalse(descend['flags']['local_altitude_invalid'])
        self.assertEqual(descend['status_publish_stamp_sim_s'], 11.)
        self.assertEqual(descend['navigation_state_activated_sim_s'], 10.)
        self.assertTrue(land['heartbeat_auto_land_label_does_not_distinguish_descend'])

    def test_termination_cannot_be_labelled_a_landing(self):
        self.assertEqual(autopilot_observation_fields(self.status(13), {})[
            'navigation_state_kind'], 'flight_termination')
        self.assertIsNone(autopilot_observation_fields(self.status(14), {})[
            'navigation_state_kind'])

    def test_missing_fields_do_not_invent_a_healthy_fallback(self):
        result = autopilot_observation_fields({}, {})
        for key in ('navigation_state', 'navigation_state_kind', 'arming_state',
                    'failsafe', 'status_publish_stamp_sim_s', 'failsafe_publish_stamp_sim_s'):
            self.assertIsNone(result[key])
        self.assertTrue(all(value is None for value in result['flags'].values()))

    def test_wrong_types_and_zero_timestamps_are_not_coerced(self):
        for value in (True, '12', 12., -1, 23, None):
            status = self.status(); status['nav_state'] = value
            self.assertIsNone(autopilot_observation_fields(status, {})['navigation_state'])
        for value in (True, 0, -1, '1000000', 1000000., None):
            status = self.status(); status['timestamp'] = value
            self.assertIsNone(autopilot_observation_fields(status, {})['status_publish_stamp_sim_s'])
        for value in (0, 1, 'false', None):
            status = self.status(); status['failsafe'] = value
            flags = self.flags(); flags['attitude_invalid'] = value
            result = autopilot_observation_fields(status, flags)
            self.assertIsNone(result['failsafe'])
            self.assertIsNone(result['flags']['attitude_invalid'])

    def test_observation_does_not_mutate_raw_inputs(self):
        status, flags = self.status(12), self.flags()
        before = copy.deepcopy((status, flags))
        autopilot_observation_fields(status, flags)
        self.assertEqual((status, flags), before)
        self.assertIn('vehicle_status', TOPICS)
        self.assertIn('failsafe_flags', TOPICS)


if __name__ == '__main__':
    unittest.main()
