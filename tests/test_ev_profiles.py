import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1]/'scripts/configure_ev_profile.py'
spec = importlib.util.spec_from_file_location('ev_profiles', path)
profiles = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profiles)


class ProfilesTest(unittest.TestCase):
    def test_gps_disabled_does_not_use_gps_height_reference(self):
        p = profiles.target_profile('no_gps', {'SDLOG_PROFILE': 3})
        self.assertEqual((p['EKF2_GPS_CTRL'], p['EKF2_HGT_REF'], p['EKF2_EV_CTRL']), (0, 3, 7))
        self.assertEqual(p['EKF2_BARO_CTRL'], 1)
        self.assertEqual(p['SDLOG_PROFILE'], 131)
        self.assertEqual(p['COM_OBL_RC_ACT'], 4)
        self.assertEqual(p['COM_OF_LOSS_T'], 1.)
        self.assertLess(profiles.PARAMETERS.index('EKF2_HGT_REF'), profiles.PARAMETERS.index('EKF2_GPS_CTRL'))

    def test_shadow_retains_gps_without_ev(self):
        p = profiles.target_profile('shadow', {'SDLOG_PROFILE': 131, 'COM_OBL_RC_ACT': 3, 'COM_OF_LOSS_T': 2.})
        self.assertEqual((p['EKF2_EV_CTRL'], p['EKF2_GPS_CTRL']), (0, 7))
        self.assertEqual(p['COM_OBL_RC_ACT'], 3)
        self.assertEqual(p['COM_OF_LOSS_T'], 2.)

    def test_gps_fused_preserves_observed_offboard_loss_action(self):
        p = profiles.target_profile('gps_fused', {'SDLOG_PROFILE': 3, 'COM_OBL_RC_ACT': 5, 'COM_OF_LOSS_T': .8})
        self.assertEqual(p['COM_OBL_RC_ACT'], 5)
        self.assertEqual(p['COM_OF_LOSS_T'], .8)
        with self.assertRaises(KeyError):
            profiles.target_profile('gps_fused', {'SDLOG_PROFILE': 3})

    def test_ground_guard_rejects_armed_and_active_mission(self):
        state = dict(simulation_only=True, connected=True, armed=False, landed_state=1, gazebo={'available': True})
        profiles.ground_guard(state)
        for bad in ({'armed': True}, {'landed_state': 2}, {'mission': {'status': 'running'}}, {'connected': False}):
            with self.assertRaises(RuntimeError):
                profiles.ground_guard(dict(state, **bad))

    def test_readback_requires_real_name_and_number(self):
        self.assertEqual(profiles.parse_value('x + EKF2_EV_CTRL [123,456] : 5', 'EKF2_EV_CTRL'), 5)
        with self.assertRaises(RuntimeError):
            profiles.parse_value('EKF2_GPS_CTRL : 7', 'EKF2_EV_CTRL')


if __name__ == '__main__':
    unittest.main()
