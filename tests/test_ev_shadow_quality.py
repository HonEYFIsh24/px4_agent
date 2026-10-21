"""Independent evaluation contracts. No ROS, flight, HTTP, or model calls."""
import contextlib
import copy
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
SPEC=importlib.util.spec_from_file_location('independent_ev_shadow',ROOT/'scripts/test_ev_shadow.py')
shadow=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(shadow)


def point(kind,stamp,p=(0.,0.,0.),q=(0.,0.,0.,1.),v=(0.,0.,0.)):
    return {'kind':kind,'stamp_sim_s':stamp,'receipt_sim_s':stamp+.001,
            'p':list(p),'q':list(q),'v':list(v),'w':[0.,0.,0.]}


def series(duration=.4):
    rows=[]
    for i in range(round(duration/.02)+1):
        stamp=1.+i*.02
        rows.extend([point('truth',stamp),point('shadow',stamp),point('optimized',stamp,(0.,0.,.02)),
                     point('propagated',stamp,(0.,0.,.02))])
    rows.append({'kind':'status','value':{'epoch':0,'ready':True,'shadow_count':20,'reset_latched':False}})
    rows.append({'kind':'tracking','value':True})
    return rows


def tilt_series(degrees,duration=.4):
    rows=series(duration)
    q=[math.sin(math.radians(degrees)/2),0.,0.,math.cos(math.radians(degrees)/2)]
    for sample in rows:
        if sample['kind']=='shadow':
            sample['q']=list(q)
            sample['p']=[10.,20.,30.]  # Once translation aligns this perfectly.
    return rows


def quaternion_product(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z]


class ShadowEvaluationTests(unittest.TestCase):
    def test_shadow_gravity_gate_rejects_11deg_despite_perfect_translation_but_accepts_3deg(self):
        for degrees in (3.,11.):
            rows=tilt_series(degrees);original=copy.deepcopy(rows)
            result=shadow.evaluate(rows,[1.,1.4],require_full_ground=False)
            checks={r['name']:r for r in result['checks']}
            self.assertTrue(checks['shadow_position_rmse']['passed'])
            self.assertTrue(checks['shadow_maximum_position_error']['passed'])
            self.assertTrue(checks['shadow_source_timestamp_fresh']['passed'])
            self.assertEqual(checks['shadow_gravity_axis_within_6deg']['passed'],degrees==3.)
            self.assertEqual(result['passed'],degrees==3.)
            metric=result['source_metrics']['shadow']
            self.assertAlmostEqual(metric['maximum_position_error_m'],0.)
            self.assertAlmostEqual(math.degrees(metric['maximum_gravity_axis_error_rad']),degrees)
            self.assertEqual(metric['alignment']['translation_m'],[-10.,-20.,-30.])
            self.assertNotIn('roll_rad',metric['alignment']);self.assertNotIn('pitch_rad',metric['alignment'])
            self.assertEqual(result['additional_independent_criterion']['maximum_error_deg'],6.)
            self.assertEqual(result['additional_independent_criterion']['scope'],
                             'independent_simulation_evaluation_not_FCU_control_proof')
            self.assertFalse(result['additional_independent_criterion']['ground_truth_feedback'])
            self.assertEqual(result['quality_evaluation_version'],'shadow_quality_gravity_axis_v2')
            self.assertEqual(rows,original)

    def test_same_gravity_criterion_applies_to_full_ground_quality_and_short_preliminary(self):
        for full in (False,True):
            duration=30. if full else 2.
            result=shadow.evaluate(tilt_series(11.,duration),[1.,1.+duration],require_full_ground=full)
            checks={r['name']:r for r in result['checks']}
            self.assertFalse(result['passed'])
            self.assertFalse(checks['shadow_gravity_axis_within_6deg']['passed'])
            if full:self.assertTrue(checks['ground_30_sim_seconds_drift']['passed'])
            self.assertEqual(result['additional_independent_criterion'],shadow.GRAVITY_CRITERION)

    def test_arbitrary_global_and_changing_level_yaw_are_not_gravity_failures(self):
        rows=series()
        for sample in rows:
            if sample['kind']=='shadow':
                angle=2.1+(sample['stamp_sim_s']-1.)*2.
                sample['q']=[0.,0.,math.sin(angle/2),math.cos(angle/2)]
        result=shadow.evaluate(rows,[1.,1.4],False)
        self.assertTrue(result['passed'])
        metrics=result['source_metrics']['shadow']
        self.assertAlmostEqual(metrics['maximum_gravity_axis_error_rad'],0.)
        self.assertGreater(metrics['maximum_orientation_geodesic_error_rad'],.5)

    def test_dynamic_actual_roll_pitch_is_compared_to_truth_instead_of_level_world(self):
        rows=series()
        yaw=.8;frame_yaw=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)]
        for sample in rows:
            if sample['kind'] not in ('truth','shadow'):continue
            t=sample['stamp_sim_s']-1.
            roll=math.radians(12.)+t*.2;pitch=math.radians(-9.)+t*.1
            q=quaternion_product([0.,math.sin(pitch/2),0.,math.cos(pitch/2)],
                                 [math.sin(roll/2),0.,0.,math.cos(roll/2)])
            sample['q']=q if sample['kind']=='truth' else quaternion_product(frame_yaw,q)
        result=shadow.evaluate(rows,[1.,1.4],False)
        self.assertTrue(result['passed'])
        self.assertLess(result['source_metrics']['shadow']['maximum_gravity_axis_error_rad'],1e-7)
        self.assertAlmostEqual(result['source_metrics']['shadow']['alignment']['yaw_rad'],-yaw)

    def test_missing_nonfinite_invalid_or_zero_orientation_fails_closed_without_normalizing_it_valid(self):
        for kind in ('truth','shadow'):
            for quaternion in (None,[],[0.,0.,0.,0.],[0.,0.,0.,2.],
                               [0.,float('nan'),0.,1.],[0.,0.,float('inf'),1.],[False,0.,0.,1.]):
                with self.subTest(kind=kind,quaternion=quaternion):
                    rows=series()
                    target=next(s for s in rows if s['kind']==kind)
                    target['q']=quaternion
                    result=shadow.evaluate(rows,[1.,1.4],False)
                    checks={r['name']:r for r in result['checks']}
                    self.assertFalse(result['passed'])
                    self.assertFalse(checks['shadow_gravity_axis_within_6deg']['passed'])
        rows=[s for s in series() if s['kind']!='shadow']
        result=shadow.evaluate(rows,[1.,1.4],False)
        check=next(c for c in result['checks'] if c['name']=='shadow_gravity_axis_within_6deg')
        self.assertFalse(check['passed']);self.assertIsNone(check['observed']['maximum_error_rad'])

    def test_truth_interpolation_normalizes_valid_endpoints_separately_and_refuses_invalid_orientation(self):
        truth=[point('truth',1.,q=(0.,0.,0.,1.02)),
               point('truth',1.02,q=(math.sqrt(.5)*.98,0.,0.,math.sqrt(.5)*.98))]
        result=shadow.interpolate_truth(truth,[1.,1.02],1.01)
        self.assertAlmostEqual(result['q'][0],math.sin(math.pi/8))
        self.assertAlmostEqual(result['q'][3],math.cos(math.pi/8))
        truth[0]['q']=[0.,0.,0.,2.]
        self.assertIsNone(shadow.interpolate_truth(truth,[1.,1.02],1.01))
        self.assertIsNone(shadow.interpolate_truth(truth,[1.,1.02],1.))
        q=[math.sin(.2/2)*1.02,0.,0.,math.cos(.2/2)*1.02]
        self.assertAlmostEqual(shadow.quaternion_yaw(q),0.)

    def test_real_runner_preliminary_gravity_failure_refuses_all_prepare_arm_and_mission_requests(self):
        # Run the production run() preflight branch with simulated callbacks
        # and pinned fake process owners. No service/HTTP method may be called.
        with tempfile.TemporaryDirectory() as temporary:
            output=Path(temporary)/'new';layout_path=Path(temporary)/'layout.json';layout_path.write_text('{}')
            args=SimpleNamespace(px4_directory=Path(temporary),layout=layout_path,output=output,fly=True,
                                 timeout=100.,expected_mode='shadow',observe_sim=30.,ground_duration_sim=30.)
            now=[100.];sim=[1.]
            all_rows=tilt_series(11.,2.2)
            class Collector:
                def __init__(self):
                    self.events=[];self.error=None;self.closed=False
                    self.recording_session=SimpleNamespace(cleanup_errors=[],evidence=lambda:{'fake':True,'closed':self.closed})
                def flush(self,stream):
                    self.events=[copy.deepcopy(s) for s in all_rows if 'stamp_sim_s' not in s or s['stamp_sim_s']<=sim[0]]
                def snapshot(self):return {}
                def clock(self):return sim[0]
                def close(self):self.closed=True
            collector=Collector()
            manager={'run_directory':str(Path(temporary)/'manager')}
            state={'ready':True,'simulation_only':True,'armed':False,'landed_state':1,
                   'position_enu_m':[0.,0.,0.], 'gazebo':{'available':True,'position_m':[0.,0.,0.]},
                   'scene':{'verified':True,'world_sha256':'world','navigation':{'minimum_clearance_m':1.}}}
            operations=mock.Mock();operations.initial_manager.return_value=manager
            operations.current_manager.return_value=manager;operations.states.return_value=(state,state)
            reliability=ModuleType('run_reliability_suite')
            reliability.SystemOperations=lambda:operations;reliability.safe_ground=mock.Mock()
            reliability.process_identity=lambda _:manager;reliability.owned_px4_process=lambda *a:{'pid':123}
            fusion=ModuleType('record_ev_fusion');fusion.stable_px4_identity=lambda value:value
            avoidance=ModuleType('test_avoidance_scene')
            avoidance.load_ground_truth=lambda _ :({'world_sha256':'world'},[{'collision_aabb_world_m':[100,100,101,101]}])
            avoidance.segment_rectangle_distance=lambda *a:{'distance_m':10.}
            avoidance.trajectory_evaluation=lambda *a:{'minimum_interpolated_segment_clearance_m':10.}
            client=mock.Mock();client.request.side_effect=AssertionError('control HTTP forbidden')
            def sleep(seconds):now[0]+=seconds;sim[0]+=seconds
            with mock.patch.dict(sys.modules,{'run_reliability_suite':reliability,'record_ev_fusion':fusion,
                                             'test_avoidance_scene':avoidance}),\
                 mock.patch.object(shadow,'RosObservations',return_value=collector),\
                 mock.patch.object(shadow,'BridgeClient',return_value=client),\
                 mock.patch.object(shadow,'require_shadow_health'),\
                 mock.patch.object(shadow.time,'monotonic',side_effect=lambda:now[0]),\
                 mock.patch.object(shadow.time,'sleep',side_effect=sleep):
                result=shadow.run(args)
            client.request.assert_not_called();client.state.assert_not_called()
            operations.preflight_check.assert_not_called()
            self.assertFalse(result['flight_submission_attempted']);self.assertFalse(result['actual_worker_acknowledged'])
            self.assertFalse(result['validation_passed']);self.assertTrue(collector.closed)
            self.assertIn('shadow_gravity_axis_within_6deg',result['error'])
            self.assertFalse(result['preliminary_quality']['passed'])
            self.assertNotIn('ground_offboard_preparation',result)
            for quality in (result['preliminary_quality'],result['quality']):
                check=next(c for c in quality['checks'] if c['name']=='shadow_gravity_axis_within_6deg')
                self.assertFalse(check['passed'])
            self.assertTrue((output/'task_validation.json').exists())

    def timing_rows(self):
        sample=point('shadow',2.)
        sample.update(receipt_sim_s=1.96,receipt_monotonic_s=100.,stamp_ns=2000000000)
        timing={'kind':'timing','value':{'stamp_sim_s':2.,'stamp_ns':2000000000,'actual_publish_clock_sim_s':2.,
            'measurement_age_at_publish_sim_s':0.,'source':'producer_ros_clock_at_publish',
            'source_timestamp_unchanged':True}}
        clock={'kind':'clock','stamp_sim_s':2.,'receipt_monotonic_s':100.008}
        return [sample,timing,clock]

    def publication_rows(self):
        rows=self.timing_rows()
        rows[1]['value'].update(shadow_publish_monotonic_before_s=99.98,
            shadow_publish_monotonic_after_s=99.99,
            monotonic_clock_scope='same_simulation_host_not_cross_machine',
            publish_semantics='rospy_publish_call_not_socket_delivery')
        return rows

    def test_same_host_publication_metadata_reports_actual_maxima_without_modifying_ages(self):
        rows=self.publication_rows()
        second=copy.deepcopy(rows)
        second[0].update(stamp_sim_s=2.02,stamp_ns=2020000000,receipt_sim_s=1.98,receipt_monotonic_s=100.04)
        second[1]['value'].update(stamp_sim_s=2.02,stamp_ns=2020000000,actual_publish_clock_sim_s=2.02,
            shadow_publish_monotonic_before_s=100.009,shadow_publish_monotonic_after_s=100.025)
        second[2].update(stamp_sim_s=2.02,receipt_monotonic_s=100.045)
        rows+=second
        original=copy.deepcopy(rows)
        result=shadow.timestamp_assessment(rows)
        self.assertTrue(result['passed'])
        delivery=result['producer_to_recorder_delivery']
        self.assertEqual(delivery['same_host_monotonic_interval_count'],2)
        self.assertAlmostEqual(delivery['maximum_callback_minus_publish_before_wall_s'],.031)
        self.assertAlmostEqual(delivery['maximum_callback_minus_publish_after_wall_s'],.015)
        self.assertAlmostEqual(delivery['maximum_rospy_publish_call_duration_wall_s'],.016)
        self.assertTrue(delivery['consumer_age_not_corrected'])
        self.assertEqual(delivery['semantics'],'recorder_callback_after_deserialization_not_socket_or_fcu_receipt')
        self.assertAlmostEqual(result['raw_consumer']['maximum_receipt_minus_source_age_sim_s'],-.04)
        self.assertEqual(rows,original)

    def test_callback_may_precede_publish_return_without_invalidating_same_host_metadata(self):
        rows=self.publication_rows()
        rows[1]['value'].update(shadow_publish_monotonic_before_s=99.99,shadow_publish_monotonic_after_s=100.02)
        result=shadow.timestamp_assessment(rows)
        self.assertTrue(result['passed'])
        delivery=result['producer_to_recorder_delivery']
        self.assertAlmostEqual(delivery['maximum_callback_minus_publish_before_wall_s'],.01)
        self.assertAlmostEqual(delivery['maximum_callback_minus_publish_after_wall_s'],-.02)
        self.assertAlmostEqual(delivery['maximum_rospy_publish_call_duration_wall_s'],.03)

    def test_valid_delivery_diagnostics_never_correct_positive_stale_consumer_age(self):
        rows=self.publication_rows()
        rows[0]['receipt_sim_s']=2.15
        rows[1]['value'].update(shadow_publish_monotonic_before_s=99.82,shadow_publish_monotonic_after_s=99.85)
        result=shadow.timestamp_assessment(rows)
        self.assertFalse(result['passed'])
        self.assertEqual(result['invalid_count'],0)
        self.assertAlmostEqual(result['producer_maximum_age_sim_s'],0.)
        self.assertAlmostEqual(result['raw_consumer']['maximum_receipt_minus_source_age_sim_s'],.15)
        self.assertTrue(result['producer_to_recorder_delivery']['consumer_age_not_corrected'])

    def test_partial_nonfinite_wrong_scope_or_semantics_publication_metadata_cannot_pass(self):
        for field,value in (('shadow_publish_monotonic_before_s',None),
                            ('shadow_publish_monotonic_after_s',None),
                            ('shadow_publish_monotonic_before_s',float('nan')),
                            ('shadow_publish_monotonic_after_s',float('inf')),
                            ('shadow_publish_monotonic_before_s',True),
                            ('shadow_publish_monotonic_after_s','99.99'),
                            ('monotonic_clock_scope','cross_machine'),
                            ('publish_semantics','socket_delivery')):
            with self.subTest(field=field,value=value):
                rows=self.publication_rows();rows[1]['value'][field]=value
                result=shadow.timestamp_assessment(rows)
                self.assertFalse(result['passed'])
                self.assertGreater(result['invalid_count'],0)
                self.assertEqual(result['producer_to_recorder_delivery']['same_host_monotonic_interval_count'],0)
        for missing in ('shadow_publish_monotonic_before_s','shadow_publish_monotonic_after_s',
                        'monotonic_clock_scope','publish_semantics'):
            with self.subTest(missing=missing):
                rows=self.publication_rows();rows[1]['value'].pop(missing)
                self.assertFalse(shadow.timestamp_assessment(rows)['passed'])
        for null_pair in (False,True):
            with self.subTest(partial_scope_only_or_null_pair=null_pair):
                rows=self.publication_rows()
                for key in ('shadow_publish_monotonic_before_s','shadow_publish_monotonic_after_s'):
                    if null_pair:rows[1]['value'][key]=None
                    else:rows[1]['value'].pop(key)
                self.assertFalse(shadow.timestamp_assessment(rows)['passed'])

    def test_out_of_order_publication_interval_or_callback_before_publish_start_cannot_pass(self):
        for before,after in ((100.01,99.99),(100.01,100.02)):
            with self.subTest(before=before,after=after):
                rows=self.publication_rows()
                rows[1]['value'].update(shadow_publish_monotonic_before_s=before,shadow_publish_monotonic_after_s=after)
                result=shadow.timestamp_assessment(rows)
                self.assertFalse(result['passed'])
                self.assertGreater(result['invalid_count'],0)

    def test_absent_optional_publication_metadata_preserves_legacy_fixture_contract(self):
        result=shadow.timestamp_assessment(self.timing_rows())
        self.assertTrue(result['passed'])
        delivery=result['producer_to_recorder_delivery']
        self.assertEqual(delivery['same_host_monotonic_interval_count'],0)
        self.assertIsNone(delivery['maximum_callback_minus_publish_before_wall_s'])
        self.assertIsNone(delivery['maximum_callback_minus_publish_after_wall_s'])
        self.assertIsNone(delivery['maximum_rospy_publish_call_duration_wall_s'])
        self.assertTrue(delivery['consumer_age_not_corrected'])

    def test_clock_reordering_needs_producer_and_actual_delivery_evidence(self):
        rows=self.timing_rows()
        result=shadow.timestamp_assessment(rows)
        self.assertTrue(result['passed'])
        self.assertAlmostEqual(result['raw_consumer']['minimum_receipt_minus_source_age_sim_s'],-.04)
        self.assertFalse(shadow.timestamp_assessment(rows[:-1])['passed'])
        rows[-1]['receipt_monotonic_s']=100.2
        self.assertFalse(shadow.timestamp_assessment(rows)['passed'])

    def test_actual_future_or_stale_source_is_rejected_despite_later_clock(self):
        for age in (-.03,.2):
            rows=self.timing_rows()
            rows[1]['value']['actual_publish_clock_sim_s']=2.+age
            rows[1]['value']['measurement_age_at_publish_sim_s']=age
            self.assertFalse(shadow.timestamp_assessment(rows)['passed'])

    def test_malformed_conflicting_or_duplicate_integer_timing_cannot_pass(self):
        for field,value in (('stamp_sim_s',None),('stamp_sim_s','2'),('stamp_ns',2000000001)):
            rows=self.timing_rows();rows[1]['value'][field]=value
            self.assertFalse(shadow.timestamp_assessment(rows)['passed'])
        rows=self.timing_rows();rows.append(copy.deepcopy(rows[1]))
        self.assertFalse(shadow.timestamp_assessment(rows)['passed'])

    def test_initial_yaw_translation_align_once(self):
        q=[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)]
        truth=[point('truth',1,(2,3,4)),point('truth',1.02,(3,3,4))]
        vio=[point('shadow',1,(10,10,10),q),point('shadow',1.02,(10,11,10),q)]
        result=shadow.evaluate_source(vio,truth,'shadow',None)
        self.assertAlmostEqual(result['position_rmse_m'],0.,places=12)
        self.assertAlmostEqual(result['alignment']['yaw_rad'],-math.pi/2)
        self.assertEqual(result['alignment']['scale'],1.)

    def test_scale_error_not_fitted_away(self):
        truth=[point('truth',1,(0,0,0)),point('truth',1.02,(1,0,0)),point('truth',1.04,(2,0,0))]
        vio=[point('shadow',1,(5,0,0)),point('shadow',1.02,(7,0,0)),point('shadow',1.04,(9,0,0))]
        result=shadow.evaluate_source(vio,truth,'shadow',None)
        self.assertEqual(result['maximum_position_error_m'],2.)
        self.assertEqual(result['alignment']['translation_m'],[-5.,0.,0.])

    def test_interpolation_requires_close_bracket_no_extrapolation(self):
        truth=[point('truth',1,(0,0,0)),point('truth',1.02,(2,0,0))]
        result=shadow.interpolate_truth(truth,[1,1.02],1.01)
        self.assertAlmostEqual(result['p'][0],1.)
        self.assertIsNone(shadow.interpolate_truth(truth,[1,1.02],.9))
        self.assertIsNone(shadow.interpolate_truth(truth,[1,1.02],1.1))
        far=[point('truth',1),point('truth',2)]
        self.assertIsNone(shadow.interpolate_truth(far,[1,2],1.5))

    def test_body_to_imu_rotating_lever_arm_independent_reference(self):
        q=[math.sin(math.pi/4),0.,0.,math.cos(math.pi/4)]
        truth=[point('truth',1),point('truth',1.02,q=q)]
        vio=[point('optimized',1,(0.,0.,.02)),point('optimized',1.02,(0.,-.02,0.),q=q)]
        result=shadow.evaluate_source(vio,truth,'optimized',None)
        self.assertAlmostEqual(result['maximum_position_error_m'],0.,places=12)
        self.assertEqual(result['alignment']['reference_point'],'original_imu')

    def test_short_observation_explicitly_has_no_full_ground_claim(self):
        result=shadow.evaluate(series(),[1,1.4],require_full_ground=False)
        self.assertTrue(result['passed'])
        self.assertFalse(result['full_ground_30_sim_seconds_assessment'])
        self.assertNotIn('ground_30_sim_seconds_drift',[r['name'] for r in result['checks']])

    def test_full_stationary_ground_has_real_30_sim_seconds_evidence(self):
        result=shadow.evaluate(series(30),[1,31],require_full_ground=True)
        self.assertTrue(result['passed'])
        self.assertAlmostEqual(result['source_metrics']['shadow']['ground_drift']['span_sim_s'],30.)

    def test_short_record_cannot_pass_full_ground_contract(self):
        result=shadow.evaluate(series(),[1,1.4],require_full_ground=True)
        checks={r['name']:r['passed'] for r in result['checks']}
        self.assertFalse(result['passed'])
        self.assertFalse(checks['ground_30_sim_seconds_observed'])

    def test_truth_same_receipt_stamp_deduplicates_but_rewind_fails(self):
        rows=series();rows.insert(1,copy.deepcopy(rows[0]))
        result=shadow.evaluate(rows,[1,1.4],False)
        self.assertTrue(result['passed'])
        self.assertEqual(result['truth_duplicate_receipt_stamp_count'],1)
        rows=series();rows.insert(8,point('truth',.9))
        result=shadow.evaluate(rows,[1,1.4],False)
        self.assertFalse(result['passed'])
        self.assertEqual(result['source_metrics']['shadow']['matched_count'],0)

    def test_stale_timestamp_duplicate_or_invalid_values_reject_quality(self):
        rows=series()
        for row in rows:
            if row['kind']=='shadow':
                row['receipt_sim_s']=row['stamp_sim_s']+.2
        self.assertFalse(shadow.evaluate(rows,[1,1.4],False)['passed'])
        rows=series();rows.insert(2,copy.deepcopy(rows[1]))
        self.assertFalse(shadow.evaluate(rows,[1,1.4],False)['passed'])
        rows=series();rows[1]['p'][0]=None
        result=shadow.evaluate(rows,[1,1.4],False)
        self.assertFalse(result['passed'])
        self.assertEqual(result['source_metrics']['shadow']['invalid_sample_count'],1)

    def test_reset_cannot_be_hidden_by_realigning_each_epoch(self):
        rows=series()+[{'kind':'status','value':{'epoch':1,'ready':True,'shadow_count':30,'reset_latched':False}}]
        result=shadow.evaluate(rows,[1,1.4],False)
        self.assertFalse(result['passed'])
        self.assertEqual(result['adapter_epochs'],[0,1])
        rows=series();rows[-2]['value']['reset_latched']=True
        self.assertFalse(shadow.evaluate(rows,[1,1.4],False)['passed'])

    def test_no_ros_http_or_output_write_in_default_plan(self):
        with mock.patch.object(shadow.BridgeClient,'request',side_effect=AssertionError('HTTP forbidden')), \
                mock.patch.object(Path,'mkdir',side_effect=AssertionError('write forbidden')), \
                contextlib.redirect_stdout(io.StringIO()) as capture:
            self.assertEqual(shadow.main(['--output','/tmp/no-write']),0)
        result=json.loads(capture.getvalue())
        self.assertFalse(result['actual_requests'])
        self.assertFalse(result['truth_feedback_to_adapter_or_control'])
        self.assertEqual(result['flight_steps_if_requested'],shadow.STEPS)

    def test_observation_never_sends_landing(self):
        client=mock.Mock()
        result=shadow.independent_land(client,None,False,mock.Mock(),mock.Mock())
        client.assert_not_called();client.request.assert_not_called();client.state.assert_not_called()
        self.assertFalse(result['requested'])

    def test_failed_original_worker_has_separate_landing_and_no_retry(self):
        client=mock.Mock()
        client.state.side_effect=[{'armed':True,'landed_state':2,'mission':{'id':'original','status':'failed'}},
                                 {'armed':False,'landed_state':1,'mission':{'id':'landing','status':'succeeded'}}]
        client.request.return_value={'mission':{'id':'landing'}}
        result=shadow.independent_land(client,'original',True,mock.Mock(),mock.Mock())
        client.request.assert_called_once_with('/land',{'expected_mission_id':'original'})
        self.assertTrue(result['confirmed_ground_disarmed'])
        self.assertEqual(result['landing_mission']['id'],'landing')

    def test_replaced_operator_flight_not_cancelled(self):
        client=mock.Mock()
        client.state.return_value={'armed':True,'landed_state':2,'mission':{'id':'operator','status':'running'}}
        with self.assertRaises(RuntimeError):
            shadow.independent_land(client,'original',True,mock.Mock(),mock.Mock())
        client.request.assert_not_called()

    def test_missing_ack_requires_worker_description_and_steps_for_recovery(self):
        client=mock.Mock()
        client.state.return_value={'armed':True,'landed_state':2,'mission':{'id':'operator','status':'running'}}
        with self.assertRaises(RuntimeError):
            shadow.independent_land(client,None,True,mock.Mock(),mock.Mock())
        client.request.assert_not_called()
        client.state.side_effect=[{'armed':False,'landed_state':1,'mission':{'id':'owned','status':'running',
                                    'description':'Independent EV quality short route','steps':shadow.STEPS}},
                                 {'armed':False,'landed_state':1,'mission':{'id':'landing','status':'succeeded'}}]
        result=shadow.independent_land(client,None,True,mock.Mock(),mock.Mock())
        client.request.assert_called_once_with('/land',{'expected_mission_id':'owned'})
        self.assertTrue(result['confirmed_ground_disarmed'])

    def test_foreign_terminal_mission_is_not_cancelled_with_known_or_missing_ack(self):
        for original_id in ('original',None):
            with self.subTest(original_id=original_id):
                client=mock.Mock()
                client.state.return_value={'armed':True,'landed_state':2,
                    'mission':{'id':'operator','status':'failed','description':'operator flight',
                               'steps':[{'action':'hold','duration':1.}]}}
                with self.assertRaises(RuntimeError):
                    shadow.independent_land(client,original_id,True,mock.Mock(),mock.Mock(),timeout=.001)
                client.request.assert_not_called()

    def test_owner_identity_failure_before_state_or_land_makes_no_flight_request(self):
        for before_state in (True,False):
            with self.subTest(before_state=before_state):
                client=mock.Mock()
                client.state.return_value={'armed':True,'landed_state':2,'mission':{'id':'original','status':'failed'}}
                owner=mock.Mock(side_effect=RuntimeError('different manager') if before_state else
                                [None,RuntimeError('different manager')])
                with self.assertRaisesRegex(RuntimeError,'different manager'):
                    shadow.independent_land(client,'original',True,mock.Mock(),mock.Mock(),owner_check=owner)
                client.request.assert_not_called()
                self.assertEqual(client.state.call_count,0 if before_state else 1)

    def test_existing_proven_parent_child_landing_is_waited_without_another_post(self):
        client=mock.Mock();owner=mock.Mock()
        child={'id':'child-land','parent_mission_id':'original','steps':[{'action':'land'}]}
        client.state.side_effect=[{'armed':True,'landed_state':2,'mission':{**child,'status':'running'}},
                                  {'armed':False,'landed_state':1,'mission':{**child,'status':'succeeded'}}]
        result=shadow.independent_land(client,'original',True,mock.Mock(),mock.Mock(),owner_check=owner)
        client.request.assert_not_called()
        self.assertTrue(result['existing_owned_landing'])
        self.assertEqual(result['landing_mission_id'],'child-land')
        self.assertTrue(result['confirmed_ground_disarmed'])
        self.assertGreaterEqual(owner.call_count,2)

    def test_freshness_mode_quality_and_lidar_gate(self):
        latest={'status':{'receipt_monotonic_s':1.,'value':{'mode':'shadow','ready':True,'quality':1,'reset_latched':False}},
                'shadow':{'stamp_sim_s':10.,'receipt_monotonic_s':1.},
                'lidar':{'stamp_sim_s':10.,'receipt_monotonic_s':1.,'sample_count':360,'negative_infinite_count':0}}
        shadow.require_shadow_health(latest,'shadow',10.02,1.02)
        for case in ('mode','quality','stamp','lidar'):
            bad=copy.deepcopy(latest)
            if case=='mode':bad['status']['value']['mode']='fuse'
            if case=='quality':bad['status']['value']['quality']=0
            if case=='stamp':bad['shadow']['stamp_sim_s']=9.
            if case=='lidar':bad['lidar']['sample_count']=359
            with self.assertRaises(RuntimeError):shadow.require_shadow_health(bad,'shadow',10.02,1.02)


if __name__=='__main__':
    unittest.main()
