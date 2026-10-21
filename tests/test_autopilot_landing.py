"""Landing-only FCU receipts and ownership contracts; no actual ROS/services."""
import types
import unittest
from unittest import mock

import test_bridge_navigation as navigation_fixture

bridge = navigation_fixture.bridge


class AutopilotLandingTests(unittest.TestCase):
    def setUp(self):
        fixture = navigation_fixture.BridgeNavigationTests('test_pose_callback_preserves_full_orientation_for_scan_alignment')
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.fixture, self.controller, self.clock = fixture, fixture.controller, fixture.fixture
        controller = self.controller
        controller.ready = bridge.Controller.ready.__get__(controller)
        controller.mode_client, controller.arm_client = mock.Mock(), mock.Mock()
        controller.mode_client.return_value = types.SimpleNamespace(mode_sent=True)
        controller.deadline = self.clock.wall_time+900.
        controller.landing_mode_pending = False
        bridge.rospy.logerr = mock.Mock()
        self.heartbeat(True,'OFFBOARD')
        controller.on_extended(types.SimpleNamespace(landed_state=2))

    def heartbeat(self, armed, mode='AUTO.LAND'):
        self.controller.on_state(types.SimpleNamespace(connected=True,armed=armed,mode=mode))

    def remove_xy(self, kind):
        if kind=='missing':self.controller.pose=None
        elif kind=='stale':self.controller.pose_time=self.clock.wall_time-5.
        elif kind=='nonfinite':self.controller.pose.pose.position.x=float('nan')
        self.controller.gazebo_history.clear();self.controller.gazebo_time=0.
        self.controller.navigation_map=None
        self.controller.ev_guard=mock.Mock()
        self.controller.ev_guard.reason.side_effect=AssertionError('landing must not query EV/XY readiness')

    def install_landing_callbacks(self, cancel=False):
        controller=self.controller
        old=[0.,0.,3.5]
        controller.target,controller.commanded=[5.,0.,3.5],list(old)
        controller.navigation_active=controller.navigation_moving=True
        seen=[]
        def service(**payload):
            self.assertEqual(payload,{'base_mode':0,'custom_mode':'AUTO.LAND'})
            self.assertTrue(controller.landing_mode_pending)
            self.assertEqual(controller.commanded,old)
            self.assertEqual(controller.target,old)
            self.assertTrue(controller.navigation_active)
            seen.append('service_ack_only')
            return types.SimpleNamespace(mode_sent=True)
        controller.mode_client.side_effect=service
        def wait(seconds):
            self.clock.wall_time+=seconds
            if controller.target is not None:
                self.assertTrue(controller.navigation_active)
                self.assertEqual(controller.commanded,old)
                if cancel:
                    controller.cancel.set();seen.append('cancelled')
                else:
                    self.heartbeat(True);seen.append('actual_AUTO_LAND_heartbeat')
            else:
                self.assertFalse(controller.navigation_active)
                self.assertFalse(controller.navigation_moving)
                controller.on_extended(types.SimpleNamespace(landed_state=1))
                self.heartbeat(False);seen.append('fresh_ground_disarmed')
            return controller.cancel.is_set()
        controller.cancel.wait=mock.Mock(side_effect=wait)
        return seen

    def test_only_validated_pure_land_accepts_missing_stale_nonfinite_xy_without_scene_or_ev(self):
        for kind in ('missing','stale','nonfinite'):
            with self.subTest(kind=kind):
                self.fixture.set_pose([0.,0.,3.5]);self.controller.pose_time=self.clock.wall_time
                self.remove_xy(kind);self.controller.mission=None
                with mock.patch.object(bridge.threading,'Thread') as thread,\
                     mock.patch.object(bridge,'validate_scene_trajectory',side_effect=AssertionError('pure land must skip XY geometry')):
                    accepted=self.controller.submit([{'action':'land'}])['mission']
                    thread.return_value.start.assert_called_once()
                self.assertEqual(accepted['steps'],[{'action':'land'}])
                self.assertTrue(accepted['landing_readiness_at_acceptance']['ready'])
                self.assertFalse(accepted['landing_readiness_at_acceptance']['xy_or_vio_readiness_required'])
                self.assertIsNone(accepted['landing_readiness_at_acceptance']['px4_nav_state'])
                self.controller.mission=None
                with mock.patch.object(bridge.threading,'Thread') as thread,self.assertRaises(bridge.MissionError):
                    self.controller.submit([{'action':'goto','x':1.,'y':0.,'altitude':3.5},{'action':'land'}])
                thread.assert_not_called()
                with self.assertRaises(bridge.MissionError):self.controller.submit([{'action':'land','x':1.}])

    def test_missing_stale_future_nonfinite_or_invalid_actual_fcu_fields_reject_even_ground(self):
        changes=({'state_time':None},{'state_time':float('nan')},{'state_time':self.clock.wall_time+1.},
                 {'state_time':self.clock.wall_time-3.001},{'extended_state_time':None},
                 {'extended_state_time':float('inf')},{'extended_state_time':self.clock.wall_time+1.},
                 {'extended_state_time':self.clock.wall_time-3.001}, {'landed':0},{'landed':False},
                 {'landed':float('nan')},{'state_msg':types.SimpleNamespace(connected=False,armed=False,mode='AUTO.LAND')},
                 {'state_msg':types.SimpleNamespace(connected=1,armed=False,mode='AUTO.LAND')},
                 {'state_msg':types.SimpleNamespace(connected=True,armed=0,mode='AUTO.LAND')},
                 {'state_msg':types.SimpleNamespace(connected=True,armed=False,mode='')})
        for change in changes:
            with self.subTest(change=change):
                self.heartbeat(False);self.controller.on_extended(types.SimpleNamespace(landed_state=1))
                for key,value in change.items():setattr(self.controller,key,value)
                with self.assertRaises(bridge.MissionError):self.controller.land()
                self.controller.mode_client.assert_not_called();self.controller.arm_client.assert_not_called()
                self.heartbeat(False);self.controller.on_extended(types.SimpleNamespace(landed_state=1))
        self.clock.wall_time+=3.
        self.controller.landing_ready()  # Fixed inclusive 3 wall-second boundary.
        self.clock.wall_time+=.001
        with self.assertRaises(bridge.MissionError):self.controller.landing_ready()

    def test_actual_extended_callback_sets_receipt_and_snapshot_explains_mode_alias(self):
        self.clock.wall_time+=.2
        self.controller.on_extended(types.SimpleNamespace(landed_state=4))
        self.assertEqual(self.controller.extended_state_time,self.clock.wall_time)
        evidence=self.controller.landing_snapshot()
        self.assertTrue(evidence['ready'])
        self.assertEqual(evidence['extended_state_age_wall_s'],0.)
        self.assertIn('DESCEND',evidence['mode_limitation'])
        self.assertIsNone(evidence['px4_nav_state'])
        self.assertEqual(evidence['px4_nav_state_source'],'not_present_in_MAVROS_State')
        self.controller.extended_state_time=None
        evidence=self.controller.landing_snapshot()
        self.assertFalse(evidence['ready']);self.assertIsNone(evidence['extended_state_age_wall_s'])
        self.controller.state_msg=types.SimpleNamespace()
        evidence=self.controller.landing_snapshot()
        self.assertFalse(evidence['ready']);self.assertIsNone(evidence['armed'])

    def test_auto_land_preserves_hold_until_actual_heartbeat_then_stops_stream_and_waits_fresh_ground(self):
        self.remove_xy('missing')
        seen=self.install_landing_callbacks()
        self.controller.land()
        self.assertEqual(seen,['service_ack_only','actual_AUTO_LAND_heartbeat','fresh_ground_disarmed'])
        self.assertIsNone(self.controller.target);self.assertIsNone(self.controller.commanded)
        self.assertFalse(self.controller.navigation_active);self.assertFalse(self.controller.navigation_moving)
        self.assertFalse(self.controller.landing_mode_pending)
        self.controller.arm_client.assert_not_called()
        self.controller.ev_guard.reason.assert_not_called()

    def test_mode_ack_without_actual_auto_land_heartbeat_never_clears_hold_or_claims_completion(self):
        self.remove_xy('missing')
        self.controller.target=[5.,0.,3.5];self.controller.commanded=[0.,0.,3.5]
        def wait(seconds):
            self.clock.wall_time+=seconds
            self.heartbeat(True,'OFFBOARD')
            self.controller.on_extended(types.SimpleNamespace(landed_state=2))
            return False
        self.controller.cancel.wait=mock.Mock(side_effect=wait)
        with self.assertRaisesRegex(bridge.MissionError,'rejected mode AUTO.LAND'):
            self.controller.land()
        self.assertIsNotNone(self.controller.target);self.assertIsNotNone(self.controller.commanded)
        self.assertTrue(self.controller.navigation_active)
        self.assertEqual(self.controller.state_msg.mode,'OFFBOARD')
        self.assertEqual(self.controller.mode_client.call_count,5)
        self.controller.arm_client.assert_not_called()

    def test_ground_disarmed_flags_cannot_finish_when_actual_extended_or_heartbeat_receipt_stales(self):
        for stale in ('state_time','extended_state_time'):
            with self.subTest(stale=stale):
                self.controller.cancel.clear();self.controller.mission=None
                self.heartbeat(True);self.controller.on_extended(types.SimpleNamespace(landed_state=2))
                self.controller.pose=None
                def wait(seconds):
                    self.clock.wall_time+=seconds
                    self.heartbeat(False);self.controller.on_extended(types.SimpleNamespace(landed_state=1))
                    setattr(self.controller,stale,self.clock.wall_time-3.001)
                    return False
                self.controller.cancel.wait=mock.Mock(side_effect=wait)
                with self.assertRaisesRegex(bridge.MissionError,'fresh actual FCU'):
                    self.controller.land()
                self.controller.arm_client.assert_not_called()

    def test_pure_land_worker_completes_without_xy_and_retains_actual_fcu_event_evidence(self):
        self.remove_xy('missing');self.install_landing_callbacks()
        self.controller.mission={'id':'owned-land','status':'running','step':0,'events':[],'steps':[{'action':'land'}]}
        self.controller.run_mission()
        self.assertEqual(self.controller.mission['status'],'succeeded')
        event,=self.controller.mission['events']
        self.assertIsNone(event['position_enu_m'])
        self.assertTrue(event['landing_fcu_evidence']['ready'])
        self.assertEqual(event['landing_fcu_evidence']['landed_state'],1)
        self.assertFalse(event['landing_fcu_evidence']['armed'])
        self.assertIn('DESCEND',event['landing_fcu_evidence']['mode_limitation'])
        self.controller.arm_client.assert_not_called()

    def test_landing_checkpoint_preserves_cancellation_shutdown_deadline_and_other_checkpoint_xy_gates(self):
        self.remove_xy('missing')
        self.controller.landing_checkpoint()
        with self.assertRaisesRegex(bridge.MissionError,'local position'):self.controller.checkpoint()
        for condition in ('cancel','closed','shutdown','deadline'):
            with self.subTest(condition=condition):
                self.controller.cancel.clear();self.controller.closed.clear()
                self.controller.deadline=self.clock.wall_time+900.
                if condition=='cancel':self.controller.cancel.set()
                elif condition=='closed':self.controller.closed.set()
                elif condition=='deadline':self.controller.deadline=self.clock.wall_time-1.
                with mock.patch.object(bridge.rospy,'is_shutdown',return_value=condition=='shutdown'),\
                     self.assertRaises(bridge.MissionError if condition=='deadline' else bridge.Cancelled):
                    self.controller.landing_checkpoint()
        self.controller.cancel.clear();self.controller.closed.clear();self.controller.deadline=self.clock.wall_time+900.
        with self.assertRaises(bridge.MissionError):self.controller.switch_mode('OFFBOARD',landing_only=True)
        self.controller.mode_client.assert_not_called()

    def test_cancellation_during_mode_handshake_preserves_command_ownership_and_makes_no_new_mode(self):
        self.remove_xy('missing');seen=self.install_landing_callbacks(cancel=True)
        with self.assertRaises(bridge.Cancelled):self.controller.land()
        self.assertEqual(seen,['service_ack_only','cancelled'])
        self.controller.mode_client.assert_called_once_with(base_mode=0,custom_mode='AUTO.LAND')
        self.controller.arm_client.assert_not_called()
        self.assertIsNotNone(self.controller.commanded)
        self.assertFalse(self.controller.landing_mode_pending)
        self.controller.mission={'id':'operator-flight','status':'running'}
        self.controller.stop=mock.Mock()
        with self.assertRaisesRegex(bridge.MissionError,'another flight'):
            self.controller.emergency_land('owned-flight')
        self.controller.stop.assert_not_called()

    def test_publisher_pending_landing_holds_last_finite_setpoint_despite_xy_loss_and_stops_on_mode(self):
        self.remove_xy('missing')
        self.controller.target=[5.,0.,3.5];self.controller.commanded=[.5,.2,3.5]
        self.controller.navigation_moving=True;self.controller.landing_mode_pending=True
        self.controller.pose_time=self.clock.wall_time-10.
        messages=self.fixture.run_publisher_iterations()
        self.assertEqual(len(messages),2)
        for message in messages:
            self.assertEqual([message.pose.position.x,message.pose.position.y,message.pose.position.z],[.5,.2,3.5])
        self.heartbeat(True)
        messages=self.fixture.run_publisher_iterations()
        self.assertEqual(messages,[])

    def test_stop_hold_never_overwrites_last_finite_command_with_invalid_or_stale_xy(self):
        for kind in ('missing','stale','nonfinite'):
            with self.subTest(kind=kind):
                self.fixture.set_pose([8.,8.,3.5]);self.controller.pose_time=self.clock.wall_time
                self.remove_xy(kind)
                self.controller.target=[9.,9.,3.5];self.controller.commanded=[.5,.2,3.5]
                self.controller.hold_position()
                self.assertEqual(self.controller.target,[.5,.2,3.5])
                self.assertEqual(self.controller.commanded,[.5,.2,3.5])


if __name__=='__main__':
    unittest.main()
