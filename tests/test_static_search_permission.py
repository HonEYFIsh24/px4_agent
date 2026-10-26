import unittest
from search_tools import _navigation_permission, CoverageSearchError
import test_search_tools as fixtures


class StaticSearchPermissionTests(unittest.TestCase):
    def state(self):
        state=fixtures.MetadataBridge().state()
        state['position_enu_m']=[0.,0.,8.]
        state['navigation'].update(method='static_3d',source='verified_static_collision_map_3d',
                                   map_registered=True,scan_count=0)
        return state

    def test_uses_current_observation_altitude_without_requiring_lidar_samples(self):
        _,altitude,_=_navigation_permission(self.state())
        self.assertEqual(altitude,8.)

    def test_unregistered_map_and_out_of_range_altitude_are_rejected(self):
        state=self.state(); state['navigation']['map_registered']=False
        with self.assertRaises(CoverageSearchError):_navigation_permission(state)
        state=self.state(); state['position_enu_m'][2]=11.
        with self.assertRaises(CoverageSearchError):_navigation_permission(state)

    def test_vio_epoch_and_reset_counter_evidence_are_still_required(self):
        state=self.state(); state['external_vision']={'required':True}
        with self.assertRaises(CoverageSearchError):_navigation_permission(state)
