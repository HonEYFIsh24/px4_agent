import unittest
from unittest import mock
from ev_health import ExternalVisionFlightGuard
import test_bridge_camera as camera_contract
import test_bridge_navigation as navigation_contract


class GroundPreparationTests(unittest.TestCase):
    def setUp(self):
        fixture=navigation_contract.BridgeNavigationTests('test_pose_callback_preserves_full_orientation_for_scan_alignment')
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.controller=fixture.controller
        self.clock=fixture.fixture
        self.controller.ev_guard=ExternalVisionFlightGuard()
        self.controller.closed=mock.Mock()
        def wait(duration):
            self.clock.wall_time+=duration
            return False
        self.controller.closed.wait.side_effect=wait
        self.controller.arm_client=mock.Mock()
        self.controller.mode_client=mock.Mock()
        self.controller.snapshot=mock.Mock(return_value={'position_enu_m':self.controller.position(),'armed':False})
        camera_contract.bridge.ExtendedState.LANDED_STATE_ON_GROUND=1

    def test_real_stationary_stream_prepares_mode_without_arming(self):
        def set_mode(**kwargs):
            self.assertEqual(kwargs['custom_mode'],'OFFBOARD')
            self.controller.state_msg.mode='OFFBOARD'
        self.controller.mode_client.side_effect=set_mode
        position=self.controller.position()
        result=self.controller.prepare_offboard()
        self.assertEqual(result['status'],'ok')
        self.assertEqual(self.controller.commanded,position)
        self.assertEqual(self.controller.target,position)
        self.controller.arm_client.assert_not_called()
        self.assertFalse(self.controller.state_msg.armed)

    def test_active_worker_and_airborne_vehicle_are_rejected_before_mode_change(self):
        for armed,mission in ((True,None),(False,{'status':'running'})):
            self.controller.state_msg.armed=armed
            self.controller.mission=mission
            with self.assertRaises(camera_contract.bridge.MissionError):
                self.controller.prepare_offboard()
        self.controller.mode_client.assert_not_called()
        self.controller.arm_client.assert_not_called()

    def test_rejected_mode_is_preserved_as_failure_without_arming(self):
        with self.assertRaisesRegex(camera_contract.bridge.MissionError,'rejected'):
            self.controller.prepare_offboard()
        self.controller.arm_client.assert_not_called()
