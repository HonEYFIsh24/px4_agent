"""Recovery must act on the accepted flight; no ROS or flight requests."""
import threading
import unittest
from unittest import mock

import test_bridge_navigation as navigation_contract
from test_web_server import FakeBridge,application,completed
bridge=navigation_contract.bridge


class BridgeRecoveryTests(unittest.TestCase):
    def setUp(self):
        fixture=navigation_contract.BridgeNavigationTests('test_pose_callback_preserves_full_orientation_for_scan_alignment')
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.controller=fixture.controller
        self.controller.submit_lock=threading.RLock()
        self.controller.mission={'id':'owned-flight','status':'running'}
        self.controller.stop=mock.Mock()
        self.controller.submit=mock.Mock(return_value={'mission':{'id':'landing-flight'}})

    def test_changed_mission_rejects_before_cancellation_or_new_submission(self):
        with self.assertRaises(bridge.MissionError):
            self.controller.emergency_land('old-flight')
        self.controller.stop.assert_not_called()
        self.controller.submit.assert_not_called()

    def test_invalid_owner_id_rejects_before_cancellation(self):
        for identifier in ('',False,123,'x'*65):
            with self.subTest(identifier=identifier),self.assertRaises(bridge.MissionError):
                self.controller.emergency_land(identifier)
        self.controller.stop.assert_not_called()

    def test_ownership_check_cancellation_and_landing_are_serialized(self):
        started=threading.Event(); acquired=threading.Event()
        def competing_submission():
            started.set()
            with self.controller.submit_lock:
                acquired.set()
        thread=threading.Thread(target=competing_submission)
        def stop():
            thread.start()
            self.assertTrue(started.wait(1))
            self.assertFalse(acquired.wait(.05))
        self.controller.stop.side_effect=stop
        def submit(*args,**kwargs):
            # A composed landing reenters the production submission lock.
            with self.controller.submit_lock:
                self.assertFalse(acquired.is_set())
                self.controller.mission={'id':'landing-flight','parent_mission_id':kwargs.get('parent_mission_id')}
                return {'mission':dict(self.controller.mission)}
        self.controller.submit.side_effect=submit
        accepted=self.controller.emergency_land('owned-flight')['mission']
        self.assertEqual(accepted['id'],'landing-flight')
        self.assertEqual(accepted['parent_mission_id'],'owned-flight')
        self.assertEqual(self.controller.mission['parent_mission_id'],'owned-flight')
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertTrue(acquired.is_set())


class WebRecoveryTests(unittest.TestCase):
    def test_failed_command_cancels_its_running_takeoff_before_arming(self):
        bridge_client=FakeBridge()
        bridge_client.mission.update(status='running',steps=[{'action':'takeoff','altitude':3.5}])
        original_request=bridge_client.request
        def request(path,payload=None):
            if path=='/land':
                self.assertEqual(payload,{'expected_mission_id':'flight-1'})
                bridge_client.mission={'id':'landing-1','parent_mission_id':'flight-1',
                    'status':'succeeded','steps':[{'action':'land'}]}
            return original_request(path,payload)
        bridge_client.request=request
        def script(client,payload,config):
            client.request('/mission',{'steps':[{'action':'takeoff','altitude':3.5}]})
            raise TimeoutError('owned takeoff worker has not armed yet')
        app=application(bridge_client,script,agent_architecture='single_vlm')
        command=completed(app,app.submit('take off')['id'])
        self.assertEqual(command['status'],'failed')
        self.assertEqual(bridge_client.calls,['/mission','/land'])
        self.assertEqual(command['safety_landing']['mission']['parent_mission_id'],'flight-1')

    def test_failed_command_waits_for_its_existing_land_without_resubmission(self):
        bridge_client=FakeBridge();bridge_client.mission['status']='running'
        def wait_for(accepted,on_progress=None):
            self.assertEqual(accepted['mission']['id'],'flight-1')
            bridge_client.mission['status']='succeeded'
            return bridge_client.state()
        bridge_client.wait_for=mock.Mock(side_effect=wait_for)
        def script(client,payload,config):
            client.request('/mission',{'steps':[{'action':'land'}]})
            raise TimeoutError('land worker still running')
        app=application(bridge_client,script,agent_architecture='single_vlm')
        command=completed(app,app.submit('land')['id'])
        self.assertEqual(command['status'],'failed')
        self.assertEqual(bridge_client.calls,['/mission'])
        bridge_client.wait_for.assert_called_once()
        self.assertEqual(command['safety_landing']['status'],'ok')

    def test_failed_command_does_not_recover_its_finished_grounded_takeoff(self):
        bridge_client=FakeBridge()
        bridge_client.mission.update(status='failed',steps=[{'action':'takeoff','altitude':3.5}])
        def script(client,payload,config):
            client.request('/mission',{'steps':[{'action':'takeoff','altitude':3.5}]})
            raise RuntimeError('preflight refused')
        app=application(bridge_client,script,agent_architecture='single_vlm')
        command=completed(app,app.submit('take off')['id'])
        self.assertEqual(bridge_client.calls,['/mission'])
        self.assertIsNone(command['safety_landing'])

    def test_running_command_exposes_actual_accepted_mission_audit(self):
        bridge_client=FakeBridge(); release=threading.Event(); accepted=threading.Event()
        def script(client,payload,config):
            client.request('/mission',{'steps':[{'action':'land'}]})
            accepted.set(); release.wait(2)
            return {'messages':[{'role':'assistant','content':'done'}]}
        app=application(bridge_client,script,agent_architecture='single_vlm')
        identifier=app.submit('test')['id']
        try:
            self.assertTrue(accepted.wait(1))
            audit=app.command(identifier)['flight_missions']
            self.assertEqual(audit[0]['path'],'/mission')
            self.assertEqual(audit[0]['mission']['id'],'flight-1')
            self.assertEqual(audit[0]['mission']['steps'],[{'action':'land'}])
        finally:
            release.set()
            completed(app,identifier)

    def test_guarded_interrupt_forwards_mission_id_only_after_matching_command(self):
        bridge_client=FakeBridge(); app=application(bridge_client,lambda *args:None)
        app.commands['owned-command']={'interrupted':False,'flight_missions':[{'path':'/mission','mission':{'id':'flight-1'}}]}
        app.active_id='owned-command'
        bridge_client.request=mock.Mock(return_value={'mission':{'id':'landing'}})
        app.interrupt('/land',{'expected_mission_id':'flight-1','expected_command_id':'owned-command'})
        bridge_client.request.assert_called_once_with('/land',{'expected_mission_id':'flight-1'})
        self.assertTrue(app.commands['owned-command']['interrupted'])

    def test_changed_command_or_mission_rejects_without_interrupting_other_work(self):
        bridge_client=FakeBridge(); app=application(bridge_client,lambda *args:None)
        app.commands['new-command']={'interrupted':False,'flight_missions':[]}
        app.commands['owned-command']={'interrupted':False,'flight_missions':[]}
        app.active_id='new-command'
        bridge_client.request=mock.Mock()
        for payload in ({'expected_command_id':'owned-command','expected_mission_id':'flight-1'},
                        {'expected_command_id':'new-command','expected_mission_id':'old-flight'},
                        {'expected_mission_id':'flight-1'}):
            with self.subTest(payload=payload),self.assertRaises(ValueError):
                app.interrupt('/land',payload)
        bridge_client.request.assert_not_called()
        self.assertFalse(app.commands['new-command']['interrupted'])

    def test_failed_model_does_not_land_a_newer_bridge_mission(self):
        bridge_client=FakeBridge()
        def state():
            return {'armed':True,'mission':{'id':'new-flight'}}
        bridge_client.state=state
        def script(client,payload,config):
            client.request('/mission',{'steps':[{'action':'land'}]})
            raise RuntimeError('provider failed')
        app=application(bridge_client,script,agent_architecture='single_vlm')
        command=completed(app,app.submit('test')['id'])
        self.assertEqual(command['status'],'failed')
        self.assertEqual(bridge_client.calls,['/mission'])
        self.assertIsNone(command['safety_landing'])

    def test_matching_command_id_cannot_claim_unaudited_current_flight(self):
        bridge_client=FakeBridge(); app=application(bridge_client,lambda *args:None)
        app.commands['old-command']={'interrupted':False,'flight_missions':[]}
        app.active_id=None
        bridge_client.request=mock.Mock()
        with self.assertRaises(ValueError):
            app.interrupt('/land',{'expected_command_id':'old-command','expected_mission_id':'flight-1'})
        bridge_client.request.assert_not_called()
        self.assertFalse(app.commands['old-command']['interrupted'])


if __name__=='__main__':
    unittest.main()
