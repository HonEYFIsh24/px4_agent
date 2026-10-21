"""Offline positive acceptance evidence, not actual provider/flight testing."""
import contextlib
import copy
import io
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from scripts import test_stereo_positive as positive
from scripts import test_stereo_search as search
from stereo_localization import optical_to_enu,triangulate_pixel
import test_stereo_search_acceptance as search_fixture


def evidence():
    identifier='a'*32;hashes={'left':'b'*64,'right':'c'*64}
    calibration={'fx':320.,'fy':320.,'cx':320.,'cy':240.,'baseline_m':.12}
    body={'frame_id':'map','body_reference':'base_link','position_m':[0.,0.,3.5],
          'orientation_xyzw':[0.,0.,0.,1.],'stamp_sim_s':10.,'age_seconds':.1}
    optical,camera_cov=triangulate_pixel(320.,240.,7.68,calibration,sigma_disparity_px=.5)
    point,covariance,assumed=optical_to_enu(optical,camera_cov,body)
    metric={'status':'ok','method':'stereo_sgbm_robust_surface','frame':'local_enu',
            'image_stamp_sim_s':10.,'body_pose_stamp_sim_s':10.,'position_camera_optical_m':optical,
            'position_enu_m':point,'covariance_enu_m2':covariance,'sigma_enu_m':[covariance[i][i]**.5 for i in range(3)],
            'historical_observation':True,'usable_as_flight_waypoint':False,'uncertainty_statistics_calibrated':False,
            'coordinate_epoch':search_fixture.EPOCH,'image_sha256':hashes,'capture_pair_id':'pair-a','calibration':calibration,
            'quality':{'roi_pixels':100,'raw_valid_pixels':90,'lr_valid_pixels':80,'robust_inlier_pixels':70,
                       'valid_ratio':.8,'lr_consistent_ratio':80/90,'texture_laplacian_variance':100.,
                       'representative_pixel_uv':[320.,240.],'median_disparity_px':7.68,'disparity_mad_px':.1,
                       'sigma_disparity_px':.5,'depth_sigma_m':camera_cov[2][2]**.5}}
    report={'status':'ok','architecture':'single_vlm','model':positive.MODEL,'capture_id':identifier,
            'created_at':10.,'finished_at':11.,'captured_at_sim_s':10.,'camera':{'image_sha256':hashes['left'],'width':640,'height':480},
            'targets':[{'label':'actual model-selected entity','bbox':[.4,.4,.6,.6],'stereo_location':metric}]}
    info={'stamp_sim_s':10.,'width':640,'height':480,'K':[320.,0.,320.,0.,320.,240.,0.,0.,1.],
          'D':[0.]*5,'R':[1.,0.,0.,0.,1.,0.,0.,0.,1.],
          'P':[320.,0.,320.,0.,0.,320.,240.,0.,0.,0.,1.,0.]}
    right_info=copy.deepcopy(info);right_info['P'][3]=-38.4
    paired={'schema_version':1,'pair_id':'pair-a','synchronized':True,'stamp_sim_s':10.,'baseline_m':.12,
            'image_sha256':hashes,'coordinate_epoch':search_fixture.EPOCH,'body_pose_enu':body,
            'model_input_side':'left','ground_truth_included':False,
            **{side:{'side':side,'mime_type':'image/jpeg','width':640,'height':480,'stamp_sim_s':10.,
                      'image_sha256':hashes[side],'camera_intrinsics':info if side=='left' else right_info,
                      'jpeg_filename':identifier+('.jpg' if side=='left' else '.right.jpg')}
               for side in ('left','right')}}
    metadata={identifier:{'capture_id':identifier,'camera':report['camera'],'stereo_evidence':paired,
                         'verified_archive_image_sha256':copy.deepcopy(hashes),
                         'snapshot':{'stamp_sim_s':10.,'stereo_pair_id':'pair-a','body_pose_enu':body,
                                     'coordinate_epoch':search_fixture.EPOCH,'stereo_image_sha256':hashes,
                                     'camera_intrinsics':info}}}
    command={'status':'succeeded','perceptions':[report],'model_calls':[
        {'status':'ok','model':positive.MODEL,'input_modalities':['text'],
         'called_tools':[{'name':'get_drone_state'},{'name':'capture_scene'}]},
        {'status':'ok','model':positive.MODEL,'input_modalities':['text','image'],'capture_ids':[identifier],
         'called_tools':[{'name':'report_scene'}]}]}
    def mission(identifier,action,started,finished,point,**parameters):
        return {'id':identifier,'status':'succeeded','started_at':started,'steps':[{'action':action,**parameters}],
                'events':[{'action':action,'step':1,'finished_at':finished,'position_enu_m':point}]}
    missions={m['id']:m for m in (mission('up','takeoff',1.,2.,[0.,0.,3.5],altitude=3.5),
        mission('home','return_home',12.,13.,[0.,0.,3.5]),mission('land','land',14.,15.,[0.,0.,0.]))}
    command['flight_missions']=[{'path':'/mission','mission':m} for m in missions.values()]
    initial=search_fixture.fusion_state();final=search_fixture.fusion_state()
    samples=[{'wall_time':10.,'state':{'armed':True,'position_enu_m':[0.,0.,3.5]}},
             {'wall_time':13.,'state':{'armed':True,'position_enu_m':[0.,0.,3.5]}}]
    # Real capture metadata is loaded separately from public report JSON.
    return copy.deepcopy((command,initial,final,samples,missions,positive.MODEL,'hover',copy.deepcopy(metadata)))


class PositiveStereoTests(unittest.TestCase):
    def test_archive_copies_exact_owned_bytes_exclusively_and_rejects_hash_mismatch(self):
        fixtures=Path(__file__).resolve().parent/'fixtures'
        raw=(fixtures/'stereo_left.jpg').read_bytes();right_raw=(fixtures/'stereo_right.jpg').read_bytes()
        values=evidence();command=values[0];metadata=next(iter(values[-1].values()));identifier=metadata['capture_id']
        metadata['camera']['image_sha256']=hashlib.sha256(raw).hexdigest()
        actual_hashes={'left':hashlib.sha256(raw).hexdigest(),'right':hashlib.sha256(right_raw).hexdigest()}
        metadata.pop('verified_archive_image_sha256')
        metadata['snapshot']['stereo_image_sha256']=copy.deepcopy(actual_hashes)
        metadata['stereo_evidence']['image_sha256']=copy.deepcopy(actual_hashes)
        for side in ('left','right'):metadata['stereo_evidence'][side]['image_sha256']=actual_hashes[side]
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)/'manager'/'captures';directory.mkdir(parents=True)
            (directory/(identifier+'.jpg')).write_bytes(raw)
            (directory/(identifier+'.right.jpg')).write_bytes(right_raw)
            (directory/(identifier+'.json')).write_text(json.dumps(metadata))
            owner=mock.Mock();owner.manager={'run_directory':str(directory.parent)}
            output=Path(temporary)/'evidence';output.mkdir()
            retained=positive.archive_captures(command,owner,output)
            self.assertEqual((output/'captures'/(identifier+'.jpg')).read_bytes(),raw)
            self.assertEqual((output/'captures'/(identifier+'.right.jpg')).read_bytes(),right_raw)
            self.assertEqual(retained[identifier]['verified_archive_image_sha256'],actual_hashes)
            self.assertEqual(retained[identifier]['capture_id'],identifier)
            self.assertEqual(owner.check.call_count,2)
            with self.assertRaises(FileExistsError):positive.archive_captures(command,owner,output)
            other=Path(temporary)/'mismatch';other.mkdir()
            (directory/(identifier+'.jpg')).write_bytes(raw+b'changed')
            with self.assertRaises(ValueError):positive.archive_captures(command,owner,other)

    def test_positive_requires_actual_right_archive_with_matching_stamp_calibration_and_hash(self):
        for change in ('absent','stamp','camera_info_stamp','right_hash','unsafe_filename','unverified_bytes'):
            values=evidence();frame=next(iter(values[-1].values()));pair=frame['stereo_evidence']
            if change=='absent':frame.pop('stereo_evidence')
            elif change=='stamp':pair['right']['stamp_sim_s']+=.01
            elif change=='camera_info_stamp':pair['right']['camera_intrinsics']['stamp_sim_s']+=.01
            elif change=='right_hash':pair['right']['image_sha256']='d'*64
            elif change=='unsafe_filename':pair['right']['jpeg_filename']='../../foreign.jpg'
            else:frame.pop('verified_archive_image_sha256')
            with self.subTest(change=change),self.assertRaises(ValueError):positive.audit_positive(*values)

    def test_positive_actual_model_box_metric_provenance_and_geometry_without_truth_claim(self):
        result=positive.audit_positive(*evidence())
        self.assertTrue(result['positive_stereo_surface_contract_passed'])
        self.assertFalse(result['object_identity_ground_truth_verified'])
        self.assertFalse(result['metric_accuracy_ground_truth_verified'])
        self.assertEqual(len(result['metric_results']),1)

    def test_no_target_or_quality_rejected_metric_is_not_positive_success(self):
        for targets in ([],[{'label':'unknown','bbox':[.4,.4,.6,.6],
                            'stereo_location':{'status':'unavailable','reason':'low texture'}}]):
            values=evidence();values[0]['perceptions'][0]['targets']=targets
            with self.subTest(targets=targets),self.assertRaisesRegex(ValueError,'NOT passed'):
                positive.audit_positive(*values)

    def test_wrong_hash_epoch_depth_ratio_or_transformed_position_cannot_pass(self):
        for change in ('hash','epoch','depth','position','image_input','ratio'):
            values=evidence();command=values[0];metric=command['perceptions'][0]['targets'][0]['stereo_location']
            if change=='hash':metric['image_sha256']['right']='d'*64
            elif change=='epoch':metric['coordinate_epoch']['ekf_counters']['heading']+=1
            elif change=='depth':metric['quality']['depth_sigma_m']=100.
            elif change=='position':metric['position_enu_m'][0]+=1.
            elif change=='image_input':command['model_calls'][1]['input_modalities']=['text']
            else:metric['quality']['valid_ratio']=1.
            with self.subTest(change=change),self.assertRaises(ValueError):positive.audit_positive(*values)

    def test_live_flight_scope_refuses_extra_target_waypoint_and_hover_horizontal_motion(self):
        for preset in ('hover','single_view'):
            command={'flight_missions':[{'mission':{'id':'flight','steps':[{'action':'goto','x':8.,'y':0.,'altitude':3.5}]}}]}
            with self.subTest(preset=preset),self.assertRaisesRegex(ValueError,'unapproved'):
                positive.check_live(command,preset)
        positive.check_live({'flight_missions':[{'mission':{'id':'flight','steps':[{'action':'goto','x':1.,'y':0.,'altitude':3.5}]}}]},'single_view')

    def test_default_plan_makes_no_http_model_flight_or_file_write(self):
        with mock.patch.object(search,'request',side_effect=AssertionError('HTTP forbidden')),\
             mock.patch.object(positive,'execute',side_effect=AssertionError('execution forbidden')),\
             mock.patch.object(Path,'mkdir',side_effect=AssertionError('write forbidden')),\
             contextlib.redirect_stdout(io.StringIO()) as stream:
            self.assertEqual(positive.main([]),0)
        result=json.loads(stream.getvalue());self.assertFalse(result['actual_requests'])
        self.assertTrue(result['metric_required_for_positive_pass'])

    def test_bounded_wrapper_keeps_exclusive_owner_and_guarded_recovery_on_failed_model(self):
        owner=search_fixture.FakeOwner();transport=search_fixture.RecoveryTransport(owner)
        with tempfile.TemporaryDirectory() as temporary:
            output=Path(temporary)/'new'
            result=positive.execute('unused',output,transport=transport,ownership_factory=lambda *args:owner)
            self.assertFalse(result['validation_passed'])
            self.assertTrue(result['recovery_landing_confirmed'])
            submission=[payload for path,payload in transport.paths if path=='/api/commands'][0]
            self.assertTrue(submission['prompt'].startswith(positive.PRESETS['hover']['prompt']))
            self.assertEqual([payload for path,payload in transport.paths if path=='/api/land'],
                             [{'expected_command_id':'task-a','expected_mission_id':'owned-mission'}])
            with self.assertRaises(FileExistsError):positive.execute('unused',output,transport=transport)

    def test_positive_wrapper_cancels_owned_prearm_worker_and_requires_fresh_fcu_cleanup(self):
        for stale in (False, True):
            owner=search_fixture.FakeOwner(); original=search_fixture.RecoveryTransport(owner)
            clock=search_fixture.FakeClock()
            def transport(base,path,payload=None):
                result=original(base,path,payload)
                if path=='/api/commands':
                    original.state.update(armed=False,landed_state=1,ready=False,position_enu_m=None,vio={'ready':False})
                    original.state['mission']['status']='running'
                    original.command['flight_missions'][0]['mission']['status']='running'
                if path=='/api/state' and original.state.get('mission',{}).get('id')=='owned-landing' and stale:
                    result['landing_readiness'].update(ready=False,extended_state_age_wall_s=4.)
                return result
            with self.subTest(stale=stale),tempfile.TemporaryDirectory() as temporary:
                result=positive.execute('unused',Path(temporary)/'exclusive',transport=transport,
                                        clock=clock,sleep=clock.sleep,ownership_factory=lambda *args:owner)
                self.assertFalse(result['validation_passed'])
                self.assertEqual(result['recovery_landing_confirmed'],not stale)
                self.assertEqual([payload for path,payload in original.paths if path=='/api/land'],
                                 [{'expected_command_id':'task-a','expected_mission_id':'owned-mission'}])
                self.assertEqual(sum(path=='/api/commands' for path,_ in original.paths),1)
                self.assertFalse(result['actual_model_call'])


if __name__=='__main__':
    unittest.main()
