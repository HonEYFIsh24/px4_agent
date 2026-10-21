"""Actual encoded JPEG archive provenance; no ROS, HTTP or provider calls."""
import base64
import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

try:
    import single_vlm
except ImportError as exc:
    raise unittest.SkipTest("Stereo FrameStore needs Agent LangChain Python") from exc

import test_single_vlm as graph_fixture


FIXTURES = Path(__file__).resolve().parent / "fixtures"
RAW = {side: (FIXTURES/("stereo_"+side+".jpg")).read_bytes() for side in ("left","right")}
EPOCH = {"vio_epoch":2,"ekf_counters":{"xy":3,"z":1,"heading":4}}


def pair():
    frames={}
    for side in ('left','right'):
        frames[side]={'status':'ok','side':side,'mime_type':'image/jpeg','width':640,'height':480,
                      'image_frame_id':'uav_'+side+'_optical','stamp_sim_s':42.,'age_seconds':.1,
                      'measurement_age_seconds':.02,'image_base64':base64.b64encode(RAW[side]).decode(),
                      'camera_intrinsics':{'stamp_sim_s':42.,'width':640,'height':480,
                          'K':[320.,0.,320.,0.,320.,240.,0.,0.,1.], 'D':[0.]*5,
                          'R':[1.,0.,0.,0.,1.,0.,0.,0.,1.],
                          'P':[320.,0.,320.,0. if side=='left' else -38.4,0.,320.,240.,0.,0.,0.,1.,0.],
                          'fx':320.,'fy':320.,'cx':320.,'cy':240.,'distortion_model':'plumb_bob',
                          'source':'/uav/camera/'+side+'/camera_info'}}
    return {'status':'ok','synchronized':True,'stamp_sim_s':42.,'baseline_m':.12,'pair_id':'same-frame-pair',
            'body_pose_enu':{'frame_id':'map','body_reference':'base_link','position_m':[1.,2.,3.5],
                             'orientation_xyzw':[0.,0.,0.,1.],'stamp_sim_s':42.,'age_seconds':.1},
            'image_sha256':{side:hashlib.sha256(RAW[side]).hexdigest() for side in RAW},
            'external_vision':{'adapter':{'epoch':2},'ekf':{'local_position_reset_counters':EPOCH['ekf_counters']}},
            **frames}


class StereoEvidenceArchiveTests(unittest.TestCase):
    def test_real_local_graph_gets_only_left_while_right_is_archived_without_an_extra_capture(self):
        client=graph_fixture.CameraClient();client.supports_stereo_localization=True
        client.capture_stereo=mock.Mock(return_value=pair())
        client.localize_stereo=mock.Mock(return_value={'status':'ok','targets':[]})
        def report_empty(messages):
            capture=graph_fixture.tool_payload(messages,'capture_scene')
            return graph_fixture.tool_call('report_scene',{'capture_id':capture['capture_id'],
                'summary':'No independently identified target','targets':[], 'limitations':['single view']},'report')
        fixture=graph_fixture.GraphTests()
        graph,model,store,_=fixture.build([graph_fixture.capture_request,report_empty,
                                        graph_fixture.AIMessage('Observation saved')],client=client)
        with tempfile.TemporaryDirectory() as temporary,mock.patch.dict(os.environ,{'UAV_CAPTURE_EVIDENCE_DIR':temporary}):
            result,config=fixture.invoke(graph)
            urls=[graph_fixture.image_urls(messages) for messages in model.calls]
            self.assertEqual(urls,[[],['data:image/jpeg;base64,'+base64.b64encode(RAW['left']).decode()],[]])
            capture=graph_fixture.tool_payload(result['messages'],'capture_scene')
            self.assertTrue((Path(temporary)/(capture['capture_id']+'.right.jpg')).exists())
            checkpoint=graph_fixture.encoded(graph.get_state(config).values)
            self.assertNotIn('data:image/',checkpoint)
        client.capture_stereo.assert_called_once_with();self.assertEqual(client.camera_reads,[])
        self.assertEqual(client.calls,[])

    def client_store(self,value):
        client=mock.Mock();client.supports_stereo_localization=True
        client.capture_stereo.return_value=copy.deepcopy(value)
        return client,single_vlm.FrameStore(client,graph_fixture.CONFIG)

    def test_actual_right_bytes_and_same_pair_metadata_are_saved_left_only_to_model(self):
        value=pair()
        # Unknown truth/debug additions must never enter the new paired archive.
        value['left']['camera_pose_world']={'position_m':[99.,99.,99.]}
        value['right']['scene_objects']=[{'secret_object_position':[99.,99.,99.]}]
        client,store=self.client_store(value)
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(os.environ,{'UAV_CAPTURE_EVIDENCE_DIR':temporary}):
            result=store.capture(('thread','turn'),'look at actual frame');identifier=result['capture_id']
            folder=Path(temporary)
            self.assertEqual((folder/(identifier+'.jpg')).read_bytes(),RAW['left'])
            self.assertEqual((folder/(identifier+'.right.jpg')).read_bytes(),RAW['right'])
            archived=json.loads((folder/(identifier+'.json')).read_text());evidence=archived['stereo_evidence']
            self.assertEqual(evidence['coordinate_epoch'],EPOCH)
            self.assertEqual(evidence['pair_id'],'same-frame-pair')
            self.assertEqual(evidence['left']['stamp_sim_s'],evidence['right']['stamp_sim_s'])
            self.assertEqual(evidence['right']['camera_intrinsics']['P'][3],-38.4)
            self.assertEqual(evidence['body_pose_enu'],archived['snapshot']['body_pose_enu'])
            self.assertEqual(evidence['image_sha256'],value['image_sha256'])
            encoded=json.dumps(archived)
            for key in ('image_base64','camera_pose_world','scene_objects','secret_object_position','api_key'):
                self.assertNotIn(key,encoded)
            frame=store.get(('thread','turn'),identifier)
            self.assertEqual(base64.b64decode(frame['image_base64']),RAW['left'])
            self.assertNotIn(base64.b64encode(RAW['right']).decode(),json.dumps(frame))
            self.assertNotIn('stereo_evidence',result)
        client.capture_stereo.assert_called_once_with();client.capture_camera.assert_not_called()

    def test_invalid_advertised_hash_stamp_calibration_mime_or_jpeg_fails_before_any_archive(self):
        for failure in ('right_hash','left_hash','stamp','info_stamp','mime','jpeg','oversize'):
            value=pair()
            if failure in ('right_hash','left_hash'):value['image_sha256'][failure.split('_')[0]]='a'*64
            elif failure=='stamp':value['right']['stamp_sim_s']+=.01
            elif failure=='info_stamp':value['right']['camera_intrinsics']['stamp_sim_s']+=.01
            elif failure=='mime':value['right']['mime_type']='image/png'
            elif failure=='jpeg':value['right']['image_base64']=base64.b64encode(b'broken image').decode()
            else:value['right']['image_base64']='a'*3000001
            client,store=self.client_store(value)
            with self.subTest(failure=failure),tempfile.TemporaryDirectory() as temporary,\
                    mock.patch.dict(os.environ,{'UAV_CAPTURE_EVIDENCE_DIR':temporary}):
                with self.assertRaises((ValueError,KeyError)):store.capture(('thread','turn'),'inspect')
                self.assertEqual(list(Path(temporary).iterdir()),[])
                self.assertEqual(store.frames,{})
            client.capture_stereo.assert_called_once_with();client.capture_camera.assert_not_called()

    def test_disabled_evidence_does_not_require_new_right_archive_fields_or_change_capture(self):
        value=pair();del value['right'];del value['synchronized']
        client,store=self.client_store(value)
        with mock.patch.dict(os.environ,{'UAV_CAPTURE_EVIDENCE_DIR':''}),\
                mock.patch.object(Path,'mkdir',side_effect=AssertionError('archive disabled')):
            result=store.capture(('thread','turn'),'inspect')
        self.assertEqual(result['status'],'ok');client.capture_stereo.assert_called_once_with()

    def test_legacy_mono_archive_is_unchanged_and_collision_does_not_overwrite(self):
        client=graph_fixture.CameraClient();store=single_vlm.FrameStore(client,graph_fixture.CONFIG)
        with tempfile.TemporaryDirectory() as temporary,mock.patch.dict(os.environ,{'UAV_CAPTURE_EVIDENCE_DIR':temporary}),\
                mock.patch.object(single_vlm.uuid,'uuid4',return_value=mock.Mock(hex='f'*32)):
            result=store.capture(('thread','turn'),'legacy mono');identifier=result['capture_id'];folder=Path(temporary)
            first=(folder/(identifier+'.json')).read_bytes()
            self.assertNotIn('stereo_evidence',json.loads(first))
            self.assertFalse((folder/(identifier+'.right.jpg')).exists())
            with self.assertRaises(FileExistsError):store.capture(('thread','turn'),'would overwrite')
            self.assertEqual((folder/(identifier+'.json')).read_bytes(),first)


if __name__=='__main__':
    unittest.main()
