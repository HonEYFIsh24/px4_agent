"""Actual EV fault evidence contracts; offline and no ROS/flight/model calls."""
import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
SPEC=importlib.util.spec_from_file_location('ev_fault_safety_contracts',ROOT/'scripts/test_ev_faults.py')
faults=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(faults)


def healthy_ground():
    return {'simulation_only':True,'ready':True,'armed':False,'landed_state':1,'mode':'OFFBOARD',
            'mission':{'status':'succeeded'},'external_vision':{'required':True,'healthy':True,
            'used_for_flight_control':True,'adapter':{'mode':'fuse','ready':True,'forwarding':True,'sitl_verified':True},
            'ekf':{'source':'actual_owned_px4_uorb','ready':True,'fusion_active':True,'local_position_valid':True}}}


def evidence(kind='drop'):
    original={'id':'original','status':'cancelled','steps':faults.STEPS,'description':faults.DESCRIPTION}
    landing={'id':'landing','parent_mission_id':'original','status':'succeeded',
             'description':'Operator requested landing','steps':[{'action':'land'}]}
    injection={'id':'injected','kind':kind,'issued_wall_s':10.,'duration_s':3.,
               'trigger':{'armed':True,'mode':'OFFBOARD'},'adapter_before':{'epoch':0,'rejected_count':0},
               'estimator_restart_published':kind=='reset','vins_target':{'pid':45}}
    states=[{'wall_time':10.,'state':{'armed':True,'mode':'OFFBOARD','landed_state':2,'mission':original}},
            {'wall_time':11.,'state':{'armed':True,'mode':'AUTO.LAND','landed_state':2,'mission':landing}},
            {'wall_time':14.,'state':{'armed':False,'mode':'AUTO.LAND','landed_state':1,'mission':landing}}]
    events=[{'time':10.2,'event':'external_vision_safety_landing','reason':'VIO adapter stopped'},
            {'time':10.3,'event':'mission_finished','mission_id':'original','status':'cancelled'},
            {'time':10.4,'event':'mission_started','mission':landing},
            {'time':13.9,'event':'step_completed','mission_id':'landing','action':'land'},
            {'time':14.,'event':'mission_finished','mission_id':'landing','status':'succeeded'}]
    sensors=[]
    for stamp in (10.5,11.5):
        payload={'epoch':1 if kind=='reset' else 0,'reset_latched':kind=='reset','forwarded_count':100,
                 'rejected_count':5,'last_rejection':'source stamp moved backwards',
                 'test_fault':{'id':'injected','kind':kind,'applied_count':30,'synthetic':True,'simulation_only':True}}
        sensors.append({'kind':'status','receipt_wall_s':stamp,'value':payload})
    if kind!='reset':sensors += [{'kind':'propagated','receipt_wall_s':10.6,'stamp_sim_s':100.},
                                {'kind':'propagated','receipt_wall_s':11.6,'stamp_sim_s':101.}]
    if kind=='reset':sensors += [{'kind':'restart','receipt_wall_s':10.1,'value':True},
                                {'kind':'tracking','receipt_wall_s':10.15,'value':False}]
    records=[{'topic':'vehicle_status','started_at_wall_s':11.6,'cycle':1,
              'published':True,'return_code':0,'fields':{'timestamp':102000000,
              'nav_state_timestamp':101000000,'nav_state':18,'arming_state':2,'failsafe':False}},
             {'topic':'failsafe_flags','started_at_wall_s':11.6,'cycle':1,
              'published':True,'return_code':0,'fields':{'timestamp':102000000,
              'local_altitude_invalid':False,'attitude_invalid':False,
              'angular_velocity_invalid':False,'local_position_invalid':False}}]
    for wall,localstamp in ((10.6,101000000),(11.6,102000000)):
        records.append({'topic':'vehicle_visual_odometry','started_at_wall_s':wall,'published':True,'return_code':0,
                        'fields':{'timestamp':100000000,'timestamp_sample':99000000}})
        records.append({'topic':'vehicle_local_position','started_at_wall_s':wall,'published':True,'return_code':0,
                        'cycle':1 if wall==11.6 else 0,'fields':{'timestamp':localstamp}})
    return {'kind':kind,'states':states,'sensor_events':sensors,'px4_records':records,'flight_events':events,
            'missions':{'original':original,'landing':landing},'original_id':'original','injection':injection,
            'trajectory':{'minimum_interpolated_segment_clearance_m':3.}}


def recovery_state(kind='original',ground=False,status=None):
    value=evidence()
    mission=copy.deepcopy(value['missions']['original' if kind=='original' else 'landing'])
    mission['status']=status or ('succeeded' if ground else ('failed' if kind=='original' else 'running'))
    return {'armed':not ground,'landed_state':1 if ground else 2,
            'mode':'AUTO.LAND' if kind!='original' else 'OFFBOARD','mission':mission}


class EvFaultRecoveryOwnershipTests(unittest.TestCase):
    def setup_recovery(self,states,web=None):
        client=mock.Mock(spec=faults.BridgeClient)
        operations=mock.Mock()
        operations.states.side_effect=[(state,copy.deepcopy(web or {})) for state in states]
        owner=mock.Mock()
        report={'flight_submission_attempted':True,'error':'original EV fault failure',
                'requested_mission_completed':False,'expected_safety_contract_pass':False}
        return client,operations,owner,report

    def invoke(self,client,operations,owner,report,**kwargs):
        return faults.recover_owned_fault_flight(client,operations,owner,'original',report,
            sleep=mock.Mock(),**kwargs)

    def test_original_failed_airborne_has_one_guarded_land_and_real_ground_evidence(self):
        initial=recovery_state();child=recovery_state('landing');ground=recovery_state('landing',True)
        client,operations,owner,report=self.setup_recovery([initial,child,ground])
        client.request.return_value={'mission':copy.deepcopy(child['mission'])}
        captured=[]
        result=self.invoke(client,operations,owner,report,on_state=captured.append)
        client.request.assert_called_once_with('/land',{'expected_mission_id':'original'})
        self.assertTrue(result['requested']);self.assertTrue(result['confirmed_ground_disarmed'])
        self.assertEqual(result['landing_id'],'landing');self.assertEqual(result['final_state'],ground)
        self.assertEqual(captured,[initial,child,ground]);self.assertTrue(report['fallback_landing_acknowledged'])
        self.assertEqual(report['error'],'original EV fault failure')
        self.assertFalse(report['requested_mission_completed']);self.assertFalse(report['expected_safety_contract_pass'])

    def test_already_started_parent_proven_watchdog_land_is_waited_without_cancellation(self):
        child=recovery_state('landing');ground=recovery_state('landing',True)
        client,operations,owner,report=self.setup_recovery([child,ground])
        result=self.invoke(client,operations,owner,report)
        client.request.assert_not_called();self.assertFalse(result['requested'])
        self.assertTrue(result['confirmed_ground_disarmed']);self.assertEqual(result['landing_id'],'landing')
        # Ownership permits waiting before journal delivery. It cannot replace
        # the actual watchdog/event evidence required to pass the fault sortie.
        self.assertFalse(faults.legitimate_safety_landing(child['mission'],[],evidence()['injection'],'original'))

    def test_identity_change_before_or_after_state_or_before_post_refuses_land(self):
        for successful_checks in (0,1,2):
            client,operations,owner,report=self.setup_recovery([recovery_state()])
            owner.side_effect=[None]*successful_checks+[RuntimeError('Pinned PX4 identity changed')]
            with self.subTest(successful_checks=successful_checks):
                with self.assertRaisesRegex(RuntimeError,'identity changed'):
                    self.invoke(client,operations,owner,report)
                client.request.assert_not_called()
                self.assertNotIn('fallback_landing_submission_attempted',report)

    def test_no_parent_foreign_parent_or_nonlanding_worker_never_claimed_even_on_ground(self):
        for change in ('missing_parent','foreign_parent','other_steps','same_id','foreign_original'):
            state=recovery_state('landing',True)
            if change=='missing_parent':state['mission'].pop('parent_mission_id')
            if change=='foreign_parent':state['mission']['parent_mission_id']='operator-flight'
            if change=='other_steps':state['mission']['steps']=[{'action':'hold','duration':20.}]
            if change=='same_id':state['mission']['id']='original'
            if change=='foreign_original':
                state=recovery_state('original',True);state['mission']['description']='Operator flight'
            client,operations,owner,report=self.setup_recovery([state])
            with self.subTest(change=change):
                with self.assertRaisesRegex(RuntimeError,'proven owned lineage'):
                    self.invoke(client,operations,owner,report)
                client.request.assert_not_called()

    def test_missing_original_ack_id_cannot_claim_by_description(self):
        client,operations,owner,report=self.setup_recovery([recovery_state()])
        with self.assertRaisesRegex(RuntimeError,'proven owned lineage'):
            faults.recover_owned_fault_flight(client,operations,owner,None,report)
        client.request.assert_not_called()

    def test_active_operator_and_known_replacement_refuse_actions(self):
        client,operations,owner,report=self.setup_recovery([recovery_state()],{'active_command_id':'operator'})
        with self.assertRaisesRegex(RuntimeError,'Operator command'):
            self.invoke(client,operations,owner,report)
        client.request.assert_not_called()
        for submitted,replaced in ((False,False),(True,True)):
            client,operations,owner,report=self.setup_recovery([])
            report['flight_submission_attempted']=submitted
            result=self.invoke(client,operations,owner,report,operator_replacement=replaced)
            self.assertFalse(result['confirmed_ground_disarmed']);self.assertIn('skipped',result)
            client.request.assert_not_called();operations.states.assert_not_called();owner.assert_not_called()

    def test_guard_refused_after_foreign_replacement_is_not_retried(self):
        foreign=recovery_state('landing');foreign['mission']['parent_mission_id']='operator'
        client,operations,owner,report=self.setup_recovery([recovery_state(),foreign])
        client.request.side_effect=RuntimeError('mission changed; refusing recovery landing for another flight')
        with self.assertRaisesRegex(RuntimeError,'proven owned lineage'):
            self.invoke(client,operations,owner,report)
        client.request.assert_called_once_with('/land',{'expected_mission_id':'original'})
        self.assertTrue(report['fallback_landing_submission_attempted'])
        self.assertIn('mission changed',report['fallback_landing_request_error'])
        self.assertEqual(report['error'],'original EV fault failure')

    def test_guard_race_with_watchdog_child_waits_without_second_post(self):
        client,operations,owner,report=self.setup_recovery(
            [recovery_state(),recovery_state('landing'),recovery_state('landing',True)])
        client.request.side_effect=RuntimeError('mission changed; watchdog already submitted landing')
        result=self.invoke(client,operations,owner,report)
        client.request.assert_called_once_with('/land',{'expected_mission_id':'original'})
        self.assertTrue(result['confirmed_ground_disarmed']);self.assertEqual(result['landing_id'],'landing')
        self.assertEqual(report['error'],'original EV fault failure')

    def test_lost_ack_can_only_be_reconciled_from_actual_owned_ground_state(self):
        client,operations,owner,report=self.setup_recovery(
            [recovery_state(),recovery_state('landing'),recovery_state('landing',True)])
        client.request.side_effect=TimeoutError('HTTP acknowledgment lost')
        result=self.invoke(client,operations,owner,report)
        self.assertTrue(result['confirmed_ground_disarmed']);self.assertEqual(result['landing_id'],'landing')
        self.assertNotIn('fallback_landing_acknowledged',report)
        self.assertEqual(report['fallback_landing_request_error'],'HTTP acknowledgment lost')
        client.request.assert_called_once_with('/land',{'expected_mission_id':'original'})

    def test_mid_wait_operator_identity_or_foreign_replacement_cannot_claim_completion(self):
        for change in ('operator','identity','foreign','new_child'):
            initial=recovery_state('landing');ground=recovery_state('landing',True)
            if change=='foreign':ground['mission']['parent_mission_id']='operator'
            if change=='new_child':ground['mission']['id']='different-child'
            client,operations,owner,report=self.setup_recovery([initial,ground])
            if change=='operator':operations.states.side_effect=[(initial,{}),(ground,{'active_command_id':'operator'})]
            if change=='identity':owner.side_effect=[None,None,None,RuntimeError('Pinned manager changed')]
            with self.subTest(change=change):
                with self.assertRaises(RuntimeError):self.invoke(client,operations,owner,report)
                client.request.assert_not_called()

    def test_ack_and_unknown_ground_fields_or_worker_status_cannot_prove_ground(self):
        for missing in ('armed','landed_state','stopped','boolean_landed','float_landed'):
            initial=recovery_state();unproven=recovery_state('landing',True)
            if missing=='stopped':unproven['mission']['status']=None
            elif missing=='boolean_landed':unproven['landed_state']=True
            elif missing=='float_landed':unproven['landed_state']=1.
            else:unproven.pop(missing)
            client,operations,owner,report=self.setup_recovery([initial,unproven])
            client.request.return_value={'mission':recovery_state('landing',True)['mission']}
            ticks=iter((0.,2.))
            with self.subTest(missing=missing):
                with self.assertRaisesRegex(TimeoutError,'grounded, disarmed and stopped'):
                    self.invoke(client,operations,owner,report,timeout=1.,clock=lambda:next(ticks))
                client.request.assert_called_once_with('/land',{'expected_mission_id':'original'})
                self.assertTrue(report['fallback_landing_acknowledged'])

    def test_stopped_original_on_ground_needs_no_new_worker_but_cannot_pass_fault_contract(self):
        client,operations,owner,report=self.setup_recovery([recovery_state('original',True,status='failed')])
        result=self.invoke(client,operations,owner,report)
        self.assertTrue(result['confirmed_ground_disarmed']);self.assertIsNone(result['landing_id'])
        client.request.assert_not_called();self.assertFalse(report['expected_safety_contract_pass'])

    def test_parent_id_is_required_by_complete_contract_and_final_ground_evidence(self):
        for change in ('missing_parent','foreign_parent','foreign_final_only'):
            value=evidence()
            if change=='foreign_final_only':
                value['states'][-1]['state']['mission']=copy.deepcopy(value['missions']['landing'])
                value['states'][-1]['state']['mission']['parent_mission_id']='operator'
            elif change=='missing_parent':value['missions']['landing'].pop('parent_mission_id')
            else:value['missions']['landing']['parent_mission_id']='operator'
            result=faults.evaluate(**value)
            with self.subTest(change=change):
                self.assertFalse(result['passed'])
                checks={item['name']:item['passed'] for item in result['checks']}
                self.assertFalse(checks['final_ground_disarmed_no_running_worker'])


class EvFaultSafetyTests(unittest.TestCase):
    def test_ground_preparation_is_empty_payload_bounded_and_never_arms(self):
        client=mock.Mock(spec=faults.BridgeClient)
        prepared={'status':'ok','armed':False,'state':healthy_ground()}
        client.request.return_value=prepared
        operations=mock.Mock();operations.states.return_value=(healthy_ground(),{})
        operations.preflight_check.return_value={'passed':True,'stdout':'Preflight check: OK'}
        calls=mock.Mock();calls.attach_mock(client,'client');calls.attach_mock(operations,'operations')
        report={'flight_submission_attempted':False}
        faults.prepare_ground_preflight(client,operations,{'pid':45},report)
        self.assertEqual(calls.mock_calls,[mock.call.operations.states(),
            mock.call.client.request('/prepare-offboard',{},timeout=15),
            mock.call.operations.preflight_check({'pid':45},2.)])
        self.assertEqual(report['ground_offboard_preparation'],prepared)
        self.assertFalse(report['flight_submission_attempted']);client.execute.assert_not_called()

    def test_active_worker_or_operator_command_refused_before_prepare_transport(self):
        for active in ('worker','operator'):
            ground=healthy_ground();web={}
            if active=='worker':ground['mission']['status']='running'
            else:web['active_command_id']='operator-new-command'
            operations=mock.Mock();operations.states.return_value=(ground,web)
            client=mock.Mock(spec=faults.BridgeClient)
            with self.assertRaises(RuntimeError):faults.prepare_ground_preflight(client,operations,{'pid':45},{})
            client.request.assert_not_called();operations.preflight_check.assert_not_called()

    def test_prepare_disarm_or_ground_contract_failure_blocks_commander_and_flight(self):
        for altered in ('ack_armed','state_armed','worker','mode'):
            prepared={'status':'ok','armed':False,'state':healthy_ground()}
            if altered=='ack_armed':prepared['armed']=True
            if altered=='state_armed':prepared['state']['armed']=True
            if altered=='worker':prepared['state']['mission']['status']='running'
            if altered=='mode':prepared['state']['mode']='AUTO.LOITER'
            client=mock.Mock(spec=faults.BridgeClient);client.request.return_value=prepared
            operations=mock.Mock();operations.states.return_value=(healthy_ground(),{})
            report={'flight_submission_attempted':False}
            with self.assertRaises(RuntimeError):faults.prepare_ground_preflight(client,operations,{'pid':45},report)
            client.request.assert_called_once_with('/prepare-offboard',{},timeout=15)
            operations.preflight_check.assert_not_called();client.execute.assert_not_called()
            self.assertEqual(report['ground_offboard_preparation'],prepared)
            self.assertFalse(report['flight_submission_attempted'])

    def test_failed_actual_prearm_preserves_evidence_without_flight_submission(self):
        client=mock.Mock(spec=faults.BridgeClient)
        client.request.return_value={'status':'ok','armed':False,'state':healthy_ground()}
        failed={'passed':False,'stdout':'Preflight check: FAILED','reason':'required position estimate invalid'}
        operations=mock.Mock();operations.states.return_value=(healthy_ground(),{})
        operations.preflight_check.return_value=failed
        report={'flight_submission_attempted':False}
        with self.assertRaisesRegex(RuntimeError,'preflight'):
            faults.prepare_ground_preflight(client,operations,{'pid':45},report)
        self.assertEqual(report['preflight'],failed)
        self.assertFalse(report['flight_submission_attempted']);client.execute.assert_not_called()
        client.request.assert_called_once_with('/prepare-offboard',{},timeout=15)

    def test_dry_plan_does_not_issue_http_or_create_directory(self):
        with mock.patch.object(faults.BridgeClient,'request',side_effect=AssertionError('HTTP forbidden')), \
                mock.patch.object(Path,'mkdir',side_effect=AssertionError('write forbidden')), \
                contextlib.redirect_stdout(io.StringIO()) as capture:
            self.assertEqual(faults.main(['--kind','drop','--output','/tmp/no-write']),0)
        value=json.loads(capture.getvalue());self.assertFalse(value['actual_requests'])
        self.assertEqual(value['ev_parameter'],'/uav_ev_bridge/test_fault')
        self.assertTrue(value['requires_real_selected_ekf_fusion'])

    def test_drop_delay_request_has_bounded_ttl_and_exact_adapter_contract(self):
        for kind in ('drop','delay'):
            request=faults.fault_request(kind,3.,'case_1',100.)
            self.assertEqual(set(request),{'id','kind','duration_s','issued_wall_s','expires_wall_s','synthetic','simulation_only'})
            self.assertEqual(request['expires_wall_s']-request['issued_wall_s'],3.5)
            self.assertTrue(request['synthetic']);self.assertTrue(request['simulation_only'])
        for kind,duration in (('reset',3.),('drop',6.),('drop',float('nan'))):
            with self.assertRaises(ValueError):faults.fault_request(kind,duration,'case_1',100.)

    def test_all_three_evidence_complete_contracts_pass_but_original_never_completed(self):
        for kind in ('drop','delay','reset'):
            result=faults.evaluate(**evidence(kind))
            self.assertTrue(result['expected_safety_contract_pass'],result['checks'])
            self.assertFalse(result['requested_mission_completed'])
            self.assertEqual(result['original_mission_status'],'cancelled')

    def test_heartbeat_land_without_raw_state_cannot_prove_autopilot_landing(self):
        value=evidence();value['px4_records']=[r for r in value['px4_records'] if r['topic']!='vehicle_status']
        result=faults.evaluate(**value)
        self.assertFalse(result['passed'])
        self.assertEqual(result['autopilot_landing_states'],[])

    def test_legitimate_no_xy_descend_is_distinct_from_position_landing(self):
        value=evidence();value['px4_records'][0]['fields'].update(nav_state=12,failsafe=True)
        value['px4_records'][1]['fields']['local_position_invalid']=True
        result=faults.evaluate(**value)
        self.assertTrue(result['passed'],result['checks'])
        self.assertEqual(result['autopilot_landing_states'][0]['kind'],'descend_without_horizontal_control')

    def test_bad_stale_or_missing_altitude_attitude_gyro_cannot_prove_safe_descend(self):
        for field, altered in (('local_altitude_invalid',True),('attitude_invalid',True),
                               ('angular_velocity_invalid',True),('timestamp',100000000),
                               ('local_altitude_invalid',0),('attitude_invalid',None)):
            value=evidence();value['px4_records'][0]['fields'].update(nav_state=12,failsafe=True)
            value['px4_records'][1]['fields'][field]=altered
            self.assertFalse(faults.evaluate(**value)['passed'],(field,altered))

    def test_termination_wrong_cycle_and_nonfailsafe_descend_do_not_pass(self):
        for change in ('termination','wrong_cycle','not_failsafe'):
            value=evidence()
            if change=='termination':value['px4_records'][0]['fields']['nav_state']=13
            elif change=='wrong_cycle':value['px4_records'][1]['cycle']=2
            else:value['px4_records'][0]['fields'].update(nav_state=12,failsafe=False)
            self.assertFalse(faults.evaluate(**value)['passed'])

    def test_failed_original_then_watchdog_child_can_pass_each_fault_contract(self):
        for kind in ('drop','delay','reset'):
            value=evidence(kind);value['missions']['original']['status']='failed'
            value['missions']['original']['error']='Actual EV input stopped'
            value['flight_events'][1].update(status='failed',error='Actual EV input stopped')
            result=faults.evaluate(**value)
            self.assertTrue(result['passed'],result['checks'])
            self.assertFalse(result['requested_mission_completed'])
            self.assertEqual(result['original_mission_status'],'failed')
            self.assertEqual(value['missions']['original']['error'],'Actual EV input stopped')

    def test_merely_changed_health_flag_does_not_prove_real_ev_dropout(self):
        value=evidence();value['px4_records'][-2]['fields']['timestamp_sample']+=100
        value['px4_records'][-2]['fields']['timestamp']+=100
        result=faults.evaluate(**value)
        self.assertFalse(result['passed'])
        checks={c['name']:c['passed'] for c in result['checks']}
        self.assertFalse(checks['actual_px4_ev_input_stopped'])

    def test_published_packet_missing_actual_timestamp_is_not_freeze_evidence(self):
        value=evidence()
        for record in value['px4_records']:
            if record['topic']=='vehicle_visual_odometry':record['fields']={'quality':1}
        self.assertFalse(faults.evaluate(**value)['passed'])

    def test_no_actual_fault_applied_or_other_fault_id_is_rejected(self):
        for altered in ('id','applied'):
            value=evidence()
            for event in value['sensor_events']:
                if event['kind']!='status':continue
                if altered=='id':event['value']['test_fault']['id']='someone_else'
                else:event['value']['test_fault']['applied_count']=0
            self.assertFalse(faults.evaluate(**value)['passed'])

    def test_forwarded_count_continues_cannot_pass_drop(self):
        value=evidence();value['sensor_events'][1]['value']['forwarded_count']=110
        self.assertFalse(faults.evaluate(**value)['passed'])

    def test_reset_requires_true_upstream_publication_epoch_and_tracking_evidence(self):
        for missing in ('publication','restart','epoch','tracking'):
            value=evidence('reset')
            if missing=='publication':value['injection']['estimator_restart_published']=False
            if missing=='restart':value['sensor_events']=[e for e in value['sensor_events'] if e['kind']!='restart']
            if missing=='tracking':value['sensor_events']=[e for e in value['sensor_events'] if e['kind']!='tracking']
            if missing=='epoch':
                for e in value['sensor_events']:
                    if e['kind']=='status':e['value']['epoch']=0
            self.assertFalse(faults.evaluate(**value)['passed'],missing)

    def test_delay_requires_actual_rejection_increase(self):
        value=evidence('delay')
        for e in value['sensor_events']:
            if e['kind']=='status':e['value']['rejected_count']=0
        self.assertFalse(faults.evaluate(**value)['passed'])

    def test_drop_cannot_pass_when_entire_vins_source_stopped(self):
        value=evidence();value['sensor_events']=[e for e in value['sensor_events'] if e['kind']!='propagated']
        self.assertFalse(faults.evaluate(**value)['passed'])

    def test_landing_receipt_without_completed_land_event_cannot_pass(self):
        value=evidence();value['flight_events']=[e for e in value['flight_events'] if e['event']!='step_completed']
        self.assertFalse(faults.evaluate(**value)['passed'])

    def test_landing_description_without_watchdog_time_chain_not_ownership(self):
        value=evidence();landing=value['missions']['landing']
        self.assertTrue(faults.legitimate_safety_landing(landing,value['flight_events'],value['injection'],'original'))
        for events in ([],value['flight_events'][1:]):
            self.assertFalse(faults.legitimate_safety_landing(landing,events,value['injection'],'original'))
        late=copy.deepcopy(value['flight_events']);late[2]['time']=16.
        self.assertFalse(faults.legitimate_safety_landing(landing,late,value['injection'],'original'))

    def test_original_completed_ground_receipt_is_not_fault_safety_pass(self):
        value=evidence();value['missions']['original']['status']='succeeded'
        result=faults.evaluate(**value)
        self.assertFalse(result['passed']);self.assertTrue(result['requested_mission_completed'])
        value=evidence();value['states'][-1]['state']['armed']=True
        self.assertFalse(faults.evaluate(**value)['passed'])

    def test_stopped_px4_clock_or_less_than_one_metre_clearance_is_not_pass(self):
        value=evidence()
        local=[r for r in value['px4_records'] if r['topic']=='vehicle_local_position']
        local[-1]['fields']['timestamp']=local[0]['fields']['timestamp']
        self.assertFalse(faults.evaluate(**value)['passed'])
        value=evidence();value['trajectory']['minimum_interpolated_segment_clearance_m']=.99
        self.assertFalse(faults.evaluate(**value)['passed'])

    def test_parameter_clear_removes_only_this_exact_owned_request(self):
        value=faults.fault_request('drop',3.,'our_id',100.)
        parameters=object.__new__(faults.Parameters);parameters.master=mock.Mock()
        parameters.get=mock.Mock(side_effect=[value,None]);parameters.master.deleteParam.return_value=(1,'ok',0)
        self.assertTrue(parameters.clear_owned(value)['confirmed_absent'])
        parameters.master.deleteParam.assert_called_once()
        parameters.master.reset_mock();parameters.get=mock.Mock(return_value=dict(value,id='other_id'))
        self.assertFalse(parameters.clear_owned(value)['cleared']);parameters.master.deleteParam.assert_not_called()

    def test_parameter_install_refuses_to_overwrite_existing_request(self):
        parameters=object.__new__(faults.Parameters);parameters.master=mock.Mock();parameters.get=mock.Mock(return_value={'id':'other'})
        with self.assertRaises(RuntimeError):parameters.install(faults.fault_request('drop',3.,'own',100.))
        parameters.master.setParam.assert_not_called()

    def test_partial_flight_json_line_is_deferred_without_losing_original_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'flight.jsonl';path.write_text('')
            journal=faults.FlightJournal(path)
            start={'event':'mission_started','time':1.,'mission':{'id':'original','status':'running'}}
            finish={'event':'mission_finished','time':2.,'mission_id':'original','status':'failed','error':'EV stopped'}
            path.write_text(json.dumps(start)+'\n'+json.dumps(finish)[:20]);journal.read()
            self.assertEqual(journal.missions['original']['status'],'running')
            path.write_text(json.dumps(start)+'\n'+json.dumps(finish)+'\n');journal.read()
            self.assertEqual(journal.missions['original']['status'],'failed')
            self.assertEqual(journal.missions['original']['error'],'EV stopped')


if __name__=='__main__':unittest.main()
