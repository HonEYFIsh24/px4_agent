"""Controlled geometry/evidence fixtures; no simulator truth used as semantic observations."""
import copy
import hashlib
import math
import unittest

from semantic_map import SemanticMap, SemanticMapError
from static_navigation import StaticNavigator
from coverage_search import project_ground_point

OWNER = ('semantic-test', 'actual-turn')
INFO = {'width': 640, 'height': 480, 'K': [320.,0.,320.,0.,320.,240.,0.,0.,1.]}
EPOCH = {'vio_epoch': 0, 'ekf_counters': {'xy': 1, 'z': 1, 'heading': 1}}


def navigator(boxes=None):
    n = StaticNavigator(boxes or [([30.,30.,0.],[31.,31.,1.])], [[-13.,-13.,.8],[13.,13.,10.4]])
    n.world_sha256 = 'a'*64
    n.bind([0.,0.,0.], [0.,0.,0.,1.], {'position_m':[0.,0.,0.], 'orientation_xyzw':[0.,0.,0.,1.]})
    return n


def observation(n, identifier='capture-1', position=None, stamp=1., targets=None):
    pose = {'frame_id':'map', 'body_reference':'base_link','position_m':position or [0.,0.,3.],
            'orientation_xyzw':[0.,0.,0.,1.], 'stamp_sim_s':stamp, 'age_seconds':.1}
    hashes = {s: hashlib.sha256((identifier+s).encode()).hexdigest() for s in ('left','right')}
    snapshot = {'stamp_sim_s':stamp, 'age_seconds':.1, 'measurement_age_seconds':.01,
                'body_pose_enu':pose, 'coordinate_epoch':copy.deepcopy(EPOCH),
                'image_quality':{'laplacian_variance':100.,'mean_luma':120.,'saturated_fraction':.1},
                'camera_intrinsics':copy.deepcopy(INFO), 'stereo_image_sha256':hashes, 'stereo_pair_id':identifier+'-pair'}
    frame = {'owner':OWNER,'delivered':True,'capture_id':identifier,'snapshot':snapshot}
    report = {'status':'ok','capture_id':identifier,'summary':'Controlled saved report','targets':targets or []}
    geometry = n.observation_visibility(pose, INFO, [0.,0.])
    geometry.update(capture_pair_id=snapshot['stereo_pair_id'], coordinate_epoch=copy.deepcopy(EPOCH),
                    stamp_sim_s=stamp,image_sha256=hashes, body_pose_enu=copy.deepcopy(pose),camera_intrinsics=copy.deepcopy(INFO))
    return frame,report,geometry


def target(frame, point, label='红色集装箱', box=None, variance=.04):
    snapshot=frame['snapshot']
    return {'label':label,'bbox':box or [.05,.05,.95,.95], 'confidence':.9,
            'stereo_location':{'status':'ok','method':'stereo_sgbm_robust_surface','frame':'local_enu',
                'position_enu_m':point,'covariance_enu_m2':[[variance,0.,0.],[0.,variance,0.],[0.,0.,variance]],
                'image_stamp_sim_s':snapshot['stamp_sim_s'],'image_sha256':snapshot['stereo_image_sha256'],
                'coordinate_epoch':snapshot['coordinate_epoch']}}


class VisibilityTests(unittest.TestCase):
    def test_blocked_ground_stays_unobserved_and_raw_los_is_not_flight_inflation(self):
        n=navigator([([2.9,-1.,0.],[3.6,1.,4.])])
        frame,report,g=observation(n)
        behind=24*48+34  # (5.25,.25), directly behind the wall.
        side=30*48+34    # (5.25,3.25), clear of the raw wall, inside inflated flight exclusion.
        self.assertIn(behind,g['eligible_cell_ids'])
        self.assertNotIn(behind,g['visible_cell_ids'])
        self.assertIn(side,g['visible_cell_ids'])
        self.assertFalse(n.segment_safe(g['camera_origin_enu_m'],[5.25,3.25,.01]))
        m=SemanticMap(OWNER,g)
        m.update_observation(frame,report,g)
        self.assertEqual(m.cells[behind]['coverage'],0.)
        self.assertEqual(m.cells[side]['coverage'],1.)
        self.assertGreater(g['occluded_cell_count'],0)

    def test_visibility_query_does_not_itself_update_coverage(self):
        n=navigator();_,_,g=observation(n)
        m=SemanticMap(OWNER,g)
        self.assertEqual(m.get_context()['geometric_coverage_fraction'],0.)
        self.assertGreater(m.coverage_gain(g['visible_cell_ids']),0.)
        self.assertFalse(m.get_context()['coverage_is_recognition_probability'])


class SemanticEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.n=navigator();self.first=observation(self.n)
        self.m=SemanticMap(OWNER,self.first[2],query='蓝色油桶',instruction='寻找红色集装箱附近的蓝色油桶')

    def test_no_report_bad_quality_and_cross_turn_do_not_change_map(self):
        for field, bad in [('delivered',False),('owner',('other','turn'))]:
            frame=copy.deepcopy(self.first[0]);frame[field]=bad
            with self.assertRaises(SemanticMapError):self.m.update_observation(frame,*self.first[1:])
        frame=copy.deepcopy(self.first[0]);frame['snapshot']['image_quality']['laplacian_variance']=0.
        with self.assertRaises(ValueError):self.m.update_observation(frame,*self.first[1:])
        self.assertEqual(self.m.get_context()['observation_count'],0)

    def test_duplicate_photo_is_rejected_and_same_position_does_not_increment_distinct_count(self):
        self.m.update_observation(*self.first)
        with self.assertRaises(SemanticMapError):self.m.update_observation(*self.first)
        second=observation(self.n,'capture-2',stamp=2.)
        self.m.update_observation(*second)
        self.assertEqual(max(c['observation_count'] for c in self.m.cells),1)
        frame,report,g=observation(self.n,'copy',stamp=3.)
        frame['snapshot']['stereo_image_sha256']=self.first[0]['snapshot']['stereo_image_sha256']
        g['image_sha256']=frame['snapshot']['stereo_image_sha256']
        with self.assertRaises(SemanticMapError):self.m.update_observation(frame,report,g)

    def test_two_distinct_views_fuse_surface_evidence_without_collapsing_uncertainty(self):
        f,r,g=self.first;r['targets']=[target(f,[5.,-1.,1.3])]
        self.m.update_observation(f,r,g)
        f2,r2,g2=observation(self.n,'capture-2',[0.,1.,3.],2.)
        r2['targets']=[target(f2,[5.,2.,1.3])]
        self.m.update_observation(f2,r2,g2)
        self.assertEqual(len(self.m.memory.targets),1)
        landmark=self.m.landmarks['target_01']
        self.assertEqual(landmark.status,'confirmed')
        self.assertEqual(landmark.attributes['distinct_viewpoint_count'],2)
        self.assertGreater(landmark.covariance_enu_m2[1][1],2.)
        self.assertFalse(landmark.semantic_class_independently_verified)

    def test_nearby_same_class_objects_are_separate_then_only_consistent_one_is_associated(self):
        def tight(frame,point):
            p=project_ground_point(INFO,frame['snapshot']['body_pose_enu'],point,20.)['pixel_uv']
            return [p[0]/640-.02,p[1]/480-.02,p[0]/640+.02,p[1]/480+.02]
        f,r,g=self.first
        a,b=[5.,-.5,1.],[5.,.5,1.]
        r['targets']=[target(f,a,'交通锥',tight(f,a)),target(f,b,'交通锥',tight(f,b))]
        self.m.update_observation(f,r,g)
        f2,r2,g2=observation(self.n,'capture-2',[0.,1.,3.],2.)
        r2['targets']=[target(f2,a,'交通锥',tight(f2,a))]
        self.m.update_observation(f2,r2,g2)
        self.assertEqual(len(self.m.memory.targets),2)
        self.assertEqual(sorted(x['measurement_count'] for x in self.m.memory.targets),[1,2])

    def test_failed_localization_keeps_bearing_without_inventing_position(self):
        f,r,g=self.first;r['targets']=[{'label':'蓝色油桶','bbox':[.4,.4,.6,.6],
                                     'stereo_location':{'status':'unavailable','reason':'not enough texture'}}]
        self.m.update_observation(f,r,g)
        landmark=self.m.landmarks['bearing_001']
        self.assertIsNone(landmark.position_enu_m)
        self.assertEqual(landmark.status,'unlocalized')
        ray=landmark.observations[0]['bearing_direction_enu']
        self.assertAlmostEqual(sum(v*v for v in ray),1.)
        self.assertFalse(self.m.get_context()['semantic_field_active'])

    def test_explicit_relation_activates_semantic_regions_without_using_covariance_as_scale(self):
        f,r,g=self.first;r['targets']=[target(f,[5.,0.,1.])]
        self.m.update_observation(f,r,g)
        self.assertTrue(self.m.get_context()['semantic_field_active'])
        self.assertGreater(len(self.m.semantic_regions()),0)
        self.assertGreater(self.m.semantic_gain(self.m.insufficiently_observed_cells(),'蓝色油桶'),0.)
        for instruction in ('寻找蓝色油桶','蓝色油桶不在红色集装箱附近'):
            other=SemanticMap(OWNER,g,query='蓝色油桶',instruction=instruction)
            other.update_observation(f,r,g)
            self.assertFalse(other.get_context()['semantic_field_active'])

    def test_epoch_sortie_and_world_changes_invalidate_and_disable_queries(self):
        for key,new in [('world_sha256','b'*64),('sortie_id','new-sortie'),
                        ('coordinate_epoch',{'vio_epoch':1,'ekf_counters':EPOCH['ekf_counters']})]:
            m=SemanticMap(OWNER,self.first[2]);bad=copy.deepcopy(self.first[2]);bad[key]=new
            with self.assertRaises(SemanticMapError):m.update_observation(self.first[0],self.first[1],bad)
            self.assertEqual(m.status,'invalid')
            with self.assertRaises(SemanticMapError):m.coverage_gain([])

    def test_target_batch_is_transactional_and_rejects_wrong_capture_measurement(self):
        f,r,g=self.first
        good=target(f,[5.,0.,1.]);bad=copy.deepcopy(good);bad['stereo_location']['image_stamp_sim_s']=99.
        r['targets']=[good,bad]
        before=self.m.snapshot()
        with self.assertRaises(SemanticMapError):self.m.update_observation(f,r,g)
        self.assertEqual(self.m.snapshot(),before)
