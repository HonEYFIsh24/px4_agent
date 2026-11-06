"""Read-only workbench evidence handling with small synthetic archives."""
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from desktop_search_data import (search_snapshot, observation_image, current_prediction, status_text,
                                 telemetry_matches, static_obstacle_outlines, landmark_uncertainty_ellipse)


class WorkbenchEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        self.image=b'fixture image';self.digest=hashlib.sha256(self.image).hexdigest();(self.root/'frame.jpg').write_bytes(self.image)
        self.snap={'status':'valid','owner':['thread','command'],'world_sha256':'world','sortie_id':'sortie','coordinate_epoch':{'vio_epoch':1},
                   'grid':{'rows':48,'cols':48,'radius_m':12.,'resolution_m':.5},'cells':[{}]*2304,'observations':[{'capture_id':'frame','image_sha256':{'left':self.digest}}]}
        (self.root/'frame.semantic-map.json').write_text(json.dumps(self.snap))
        self.command={'id':'command','perceptions':[{'status':'ok','capture_id':'frame','camera':{'image_sha256':self.digest}}],
                      'semantic_map_events':[{'status':'valid','capture_id':'frame','world_sha256':'world','sortie_id':'sortie'}]}

    def test_snapshot_is_command_owned_and_latest_invalid_event_is_not_hidden(self):
        self.assertEqual(search_snapshot(self.command,self.root),self.snap)
        self.command['id']='another';self.assertIsNone(search_snapshot(self.command,self.root))
        self.command['id']='command';self.command['semantic_map_events'].append({'status':'invalid'})
        self.assertIsNone(search_snapshot(self.command,self.root))

    def test_wider_grid_requires_matching_dimensions_and_cell_count(self):
        self.snap['grid']={'radius_m':30.,'resolution_m':1.,'rows':60,'cols':60}
        self.snap['cells']=[{}]*3600
        (self.root/'frame.semantic-map.json').write_text(json.dumps(self.snap))
        self.assertEqual(search_snapshot(self.command,self.root),self.snap)
        self.snap['cells']=[{}]*2304
        (self.root/'frame.semantic-map.json').write_text(json.dumps(self.snap))
        self.assertIsNone(search_snapshot(self.command,self.root))

    def test_historical_image_requires_report_and_matching_hash(self):
        self.assertEqual(observation_image(self.command,'frame',self.root),self.image)
        (self.root/'frame.jpg').write_bytes(b'modified')
        self.assertIsNone(observation_image(self.command,'frame',self.root))
        self.assertIsNone(observation_image(self.command,'../frame',self.root))

    def test_candidate_must_match_source_and_coordinate_identity(self):
        event={'owner':['thread','command'],'source_capture_id':'frame',
               'map_revision':hashlib.sha256(json.dumps(self.snap,ensure_ascii=False,sort_keys=True,allow_nan=False).encode()).hexdigest(),
               'identity':{'world_sha256':'world','sortie_id':'sortie','coordinate_epoch':{'vio_epoch':1}}}
        self.command['nbv_events']=[event]
        self.assertEqual(current_prediction(self.command,self.snap),event)
        event['source_capture_id']='older';self.assertIsNone(current_prediction(self.command,self.snap))
        event['source_capture_id']='frame';event['identity']['sortie_id']='different';self.assertIsNone(current_prediction(self.command,self.snap))

    def test_single_frame_never_described_as_multiview_confirmation(self):
        self.command.update(status='succeeded',search_events=[{'result':{'search_status':'target_found','search_phase':'confirmed','target_confirmation_mode':'single_frame'}}])
        self.assertIn('没有多视角确认记录',status_text(self.command,{}))
        self.command['search_events'][0]['result']['target_confirmation_mode']='multiview'
        self.command['target_confirmation_events']=[{'outcome':'confirmed','owner':['thread','command']}]
        self.assertEqual(status_text(self.command,{}),'多视角确认完成')
        self.assertEqual(status_text(self.command,{'mission':{'status':'running','action':'land'}}),'正在返航降落')
        self.assertEqual(status_text(self.command,{'mission':{'status':'running','action':'takeoff'}}),'起飞与导航初始化')

    def test_target_approach_reacquisition_and_exhaustion_are_distinct(self):
        progress={'search_status':'searching','search_phase':'confirming','target_focus_state':'approaching'}
        self.command.update(status='running',search_events=[{'result':progress}])
        self.assertIn('正在接近',status_text(self.command,{}))
        progress['target_focus_state']='reacquiring'
        self.assertIn('重新观察',status_text(self.command,{}))
        progress['search_status']='budget_exhausted'
        self.assertEqual(status_text(self.command,{}),'搜索预算已用尽')
        progress['search_status']='no_reachable_view'
        self.assertIn('正在收尾',status_text(self.command,{}))

    def test_live_position_requires_matching_world_sortie_and_epoch(self):
        self.snap['coordinate_epoch']={'vio_epoch':1,'ekf_counters':{'xy':1,'z':2,'heading':3}}
        state={'ready':True,'scene':{'world_sha256':'world'},'navigation':{'registration_id':'sortie'},
               'external_vision':{'adapter':{'epoch':1},'ekf':{'local_position_reset_counters':{'xy':1,'z':2,'heading':3}}}}
        self.assertTrue(telemetry_matches(state,self.snap))
        state['external_vision']['adapter']['epoch']=2
        self.assertFalse(telemetry_matches(state,self.snap))
        state['external_vision']['adapter']['epoch']=1;state['navigation']['registration_id']='other'
        self.assertFalse(telemetry_matches(state,self.snap))

    def test_obstacle_outlines_require_hash_verified_geometry(self):
        world=Path(__file__).resolve().parents[1]/'scene/test_environments/courtyard_delivery.world'
        snapshot={'world_sha256':hashlib.sha256(world.read_bytes()).hexdigest(),
                  'registration':{'world_from_enu_origin_m':[0.,0.,0.],'world_from_enu_yaw_rad':0.}}
        outlines=static_obstacle_outlines(snapshot,str(world))
        self.assertTrue(outlines)
        self.assertTrue(all(len(poly)==4 for poly in outlines))
        snapshot['world_sha256']='0'*64
        self.assertIsNone(static_obstacle_outlines(snapshot,str(world)))

    def test_uncertainty_ellipse_is_not_invented_for_invalid_covariance(self):
        ellipse=landmark_uncertainty_ellipse([[4.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
        self.assertEqual(ellipse,(4.,2.,0.))
        for invalid in (None, [[-1.,0.],[0.,1.]], [[float('nan'),0.],[0.,1.]]):
            self.assertIsNone(landmark_uncertainty_ellipse(invalid))


if __name__=='__main__':unittest.main()
