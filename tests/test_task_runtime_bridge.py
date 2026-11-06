"""Actual controller stage-transition method, with local ROS message doubles."""
import copy
import unittest
from unittest import mock

from mission import MissionError
from test_semantic_map import navigator, EPOCH
from viewpoint_geometry import registered_identity
import test_bridge_navigation as fixtures


class ControllerStageTests(unittest.TestCase):
    def setUp(self):
        self.fixture=fixtures.BridgeNavigationTests('test_auto_land_recovers_without_any_lidar_scan')
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.c=self.fixture.controller
        self.c.scene['navigation']['method']='static_3d'
        self.c.navigation_map=navigator()
        self.c.ready=mock.Mock()
        self.c.mission={'status':'succeeded','id':'owned-completed-flight'}
        self.state={'external_vision':{'adapter':{'epoch':EPOCH['vio_epoch']},
                    'ekf':{'local_position_reset_counters':copy.deepcopy(EPOCH['ekf_counters'])}}}
        self.c.snapshot=lambda:copy.deepcopy(self.state)
        self.owner=['parent','turn']
        self.context={'owner':self.owner,'map_revision':'b'*64,'batch_id':'a'*32}
        self.identity=registered_identity(self.c.navigation_map,EPOCH)
        self.authority=self.c.search_authority()
        self.authority.bind(self.context,'pair-1',{'stamp_sim_s':1.,'body_pose_enu':{'position_m':[0.,0.,3.]}},
                            self.identity,[0.,0.])

    def test_actual_controller_changes_stage_without_creating_flight(self):
        before=self.authority.constraint()
        result=self.c.transition_search_stage(self.owner,'photo',1)
        self.assertEqual(result['status'],'ok')
        self.assertEqual(self.c.mission['id'],'owned-completed-flight')
        self.assertEqual(before['origin_xy_enu_m'],self.authority.constraint()['origin_xy_enu_m'])
        self.assertEqual(before['search_radius_m'],self.authority.constraint()['search_radius_m'])
        self.assertIsNone(self.c.worker)

    def test_executor_running_or_coordinate_reset_cannot_advance_stage(self):
        self.c.mission['status']='running'
        with self.assertRaises(MissionError):self.c.transition_search_stage(self.owner,'photo',1)
        self.c.mission['status']='succeeded'
        self.state['external_vision']['adapter']['epoch']+=1
        with self.assertRaises(MissionError):self.c.transition_search_stage(self.owner,'photo',1)
        self.assertEqual(self.authority.active['batch_id'],self.context['batch_id'])

    def test_operator_revoke_remains_in_force(self):
        self.authority.revoke()
        with self.assertRaises(MissionError):self.c.transition_search_stage(self.owner,'photo',1)
        self.assertIn(tuple(self.owner),self.authority.revoked_owners)


if __name__=='__main__':unittest.main()
