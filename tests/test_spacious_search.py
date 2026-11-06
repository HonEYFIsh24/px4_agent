"""30m permission, real authored scene geometry and bearing-only local approach.

All target observations below are explicit synthetic fixtures, not Gazebo truth
fed to the Agent. Scene authoring checks remain offline.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from mission import MissionError
from scene_config import load_scene
from search_limits import validate_grid
from semantic_map import SemanticMap, SemanticMapError
from static_navigation import StaticNavigator
from target_confirmation import confirmation_goal
from viewpoint_authority import ViewpointAuthority
from viewpoint_geometry import StaticViewpointEvaluator, registered_identity
from viewpoint_planner import ViewpointConfig, ViewpointPlanner
from test_semantic_map import navigator, observation, OWNER, EPOCH
from coverage_search import project_ground_point

ROOT=Path(__file__).resolve().parents[1]
WORLD=ROOT/'scene/test_environments/courtyard_spacious.world'


def fixture(boxes=None):
    nav=navigator(boxes);nav.bounds=[[-33.,-33.,.8],[33.,33.,20.4]]
    frame,report,geometry=observation(nav)
    geometry.update(nav.observation_visibility(frame['snapshot']['body_pose_enu'],
        frame['snapshot']['camera_intrinsics'],[0.,0.],radius=30.,resolution=1.))
    u,v=project_ground_point(frame['snapshot']['camera_intrinsics'],frame['snapshot']['body_pose_enu'],[8.,0.,1.],20.)['pixel_uv']
    report['targets']=[{'label':'白色货车','bbox':[u/640-.08,v/480-.08,u/640+.08,v/480+.08],
                        'stereo_location':{'status':'unavailable','reason':'controlled missing metric depth'}}]
    semantic_map=SemanticMap(OWNER,geometry,query='白色货车');semantic_map.update_observation(frame,report,geometry)
    goal=confirmation_goal(semantic_map,next(iter(semantic_map.landmarks)),frame['capture_id'])
    goal['purpose']='depth_acquisition'
    return nav,frame,semantic_map,goal


class SpaciousWorldTests(unittest.TestCase):
    def test_registered_scene_has_real_room_around_vehicle_without_semantic_leakage(self):
        scene=load_scene(WORLD,navigation_mode='static_3d')
        self.assertTrue(scene['verified'])
        self.assertEqual(scene['world_sha256'],hashlib.sha256(WORLD.read_bytes()).hexdigest())
        self.assertEqual(scene['ground_plane']['bounds_xy_m'],[-38.,-38.,38.,38.])
        self.assertEqual(scene['flight_region']['bounds_min_m'][:2],[-34.,-34.])
        self.assertEqual(scene['flight_region']['bounds_max_m'][:2],[34.,34.])
        for name in ('white_delivery_van','courtyard_screen','target_ids','collision_bounds'):
            self.assertNotIn(name,json.dumps(scene))
        def bounds(name):
            data=json.loads((WORLD.parent/name).read_text())
            return {o['id']:o['collision_bounds'] for o in data['objects']}
        def gap(a,b):
            return math.sqrt(sum(max(0.,a[axis][0]-b[axis][1],b[axis][0]-a[axis][1])**2 for axis in ('x','y')))
        old=bounds('courtyard_delivery_layout.json');new=bounds('courtyard_spacious_layout.json')
        self.assertGreater(gap(new['white_delivery_van'],new['courtyard_screen']),9.)
        self.assertGreater(gap(new['white_delivery_van'],new['courtyard_screen']),gap(old['white_delivery_van'],old['courtyard_screen']))
        self.assertGreater(gap(new['white_delivery_van'],new['brick_warehouse']),8.)
        self.assertGreater(gap(new['white_delivery_van'],new['red_shipping_container']),10.)

    def test_new_world_generation_does_not_rewrite_old_baseline(self):
        import sys
        sys.path.insert(0,str(ROOT/'scene'))
        from generate_spacious_courtyard import generate
        baseline=WORLD.parent/'courtyard_delivery.world';before=baseline.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            new=generate(directory)
            self.assertEqual(new.read_bytes(),WORLD.read_bytes())
        self.assertEqual(baseline.read_bytes(),before)

    def test_actual_spacious_geometry_permits_25m_goal_and_rejects_30m_overflow(self):
        scene=load_scene(WORLD,navigation_mode='static_3d')
        nav=StaticNavigator.from_scene(WORLD,scene)
        nav.bind([0.,0.,0.],[0.,0.,0.,1.],{'position_m':[0.,0.,0.],'orientation_xyzw':[0.,0.,0.,1.]})
        plan=nav.preview_path([0.,0.,3.],[0.,25.,3.],[0.,0.],30.)
        self.assertEqual(plan['search_radius_m'],30.)
        self.assertTrue(plan['full_route_within_search_volume'])
        with self.assertRaises(MissionError):nav.preview_path([0.,0.,3.],[30.01,0.,3.],[0.,0.],30.)


class ThirtyMetreMapTests(unittest.TestCase):
    def test_map_indices_area_and_legacy_replay_are_consistent(self):
        _,_,semantic_map,_=fixture()
        self.assertEqual(len(semantic_map.cells),3600)
        self.assertEqual(semantic_map.grid['cols'],60)
        self.assertEqual(semantic_map.centre(0)[:2],[-29.5,-29.5])
        self.assertEqual(semantic_map.centre(3599)[:2],[29.5,29.5])
        fresh=semantic_map.insufficiently_observed_cells()
        self.assertEqual(semantic_map.coverage_gain(fresh[:10]),10.)
        with self.assertRaises(SemanticMapError):semantic_map.coverage_gain([3600])
        self.assertEqual(validate_grid({'radius_m':12.,'resolution_m':.5,'rows':48,'cols':48}),2304)
        for invalid in ({'radius_m':31.,'resolution_m':1.,'rows':62,'cols':62},
                        {'radius_m':30.,'resolution_m':1.,'rows':60000,'cols':60000}):
            with self.assertRaises(ValueError):validate_grid(invalid)

    def test_executor_ticket_keeps_same_30m_scope_and_cannot_widen_it(self):
        nav,frame,semantic_map,_=fixture();pose=frame['snapshot']
        identity=registered_identity(nav,EPOCH)
        evaluator=StaticViewpointEvaluator(nav,pose['body_pose_enu'],pose['camera_intrinsics'],EPOCH,
                                           search_radius_m=30.,grid_resolution_m=1.)
        authority=ViewpointAuthority();context={'owner':list(OWNER),'map_revision':'b'*64,'batch_id':'a'*32}
        metadata={'stamp_sim_s':1.,'body_pose_enu':pose['body_pose_enu']}
        authority.bind(context,'pair',metadata,identity,[0.,0.],search_radius_m=30.)
        response=evaluator.query('paths',[0.,0.],identity,[{'id':'view_001','position_enu_m':[25.,0.,3.]}])
        authority.issue(context,response)
        value=authority.consume(OWNER,context['batch_id'],context['map_revision'],'view_001',
            response['results'][0]['execution_ticket'],identity,[0.,0.,3.],0.)
        self.assertEqual(value['goal_enu_m'],[25.,0.,3.]);self.assertEqual(value['search_radius_m'],30.)
        with self.assertRaises(MissionError):
            authority.bind(context,'pair',metadata,identity,[0.,0.],search_radius_m=31.)
        with self.assertRaises(MissionError):
            authority.bind(context,'pair',metadata,identity,[0.,0.],search_radius_m=12.)
        response['search_radius_m']=31.
        with self.assertRaises(MissionError):authority.issue(context,response)


class BearingApproachTests(unittest.TestCase):
    def evaluate(self,nav,frame,semantic_map,goal,source=None):
        snapshot=frame['snapshot'];pose=copy.deepcopy(snapshot['body_pose_enu'])
        if source is not None:pose['position_m']=source
        backend=StaticViewpointEvaluator(nav,pose,snapshot['camera_intrinsics'],EPOCH,
            probe_points_enu_m=goal['probe_points_enu_m'],search_radius_m=30.,grid_resolution_m=1.)
        planner=ViewpointPlanner(ViewpointConfig(mode='geometry',max_path_evaluations=8))
        return planner,planner.evaluate(semantic_map,backend,goal)

    def test_missing_depth_selects_small_forward_observation_without_inventing_target_xyz(self):
        nav,frame,semantic_map,goal=fixture();before=semantic_map.snapshot()
        planner,result=self.evaluate(nav,frame,semantic_map,goal)
        selected=planner.select(semantic_map)
        self.assertIsNotNone(selected,result)
        dx,dy,dz=[a-b for a,b in zip(selected['position_enu_m'],frame['snapshot']['body_pose_enu']['position_m'])]
        self.assertGreater(dx,0.);self.assertLessEqual(math.hypot(dx,dy),2.5);self.assertLessEqual(abs(dz),.5)
        self.assertGreater(selected['score_terms']['bearing_approach'],0.)
        self.assertIsNone(selected['predicted_target_range_m'])
        self.assertEqual(selected['observation_range_score'],0.)
        self.assertEqual(semantic_map.snapshot(),before)
        self.assertTrue(all(l.position_enu_m is None for l in semantic_map.landmarks.values()))
        self.assertFalse(selected['execution_authorized'])

    def test_static_obstacle_blocks_approach_despite_positive_bearing(self):
        nav,frame,semantic_map,goal=fixture([([1.2,-2.,0.],[1.6,2.,7.])])
        _,result=self.evaluate(nav,frame,semantic_map,goal)
        self.assertTrue(any(c['stage']=='endpoint' for c in result['rejected']))
        self.assertTrue(all(c['bearing_advance_score']==0. for c in result['ranked_candidates']))

    def test_stale_view_and_large_close_box_do_not_reward_blind_forward_motion(self):
        nav,frame,semantic_map,goal=fixture()
        for source,box in (([0.,3.,3.],goal['source_bbox']),([0.,0.,3.],[.2,.2,.8,.8])):
            with self.subTest(source=source,box=box):
                adjusted=copy.deepcopy(goal);adjusted['source_bbox']=box
                planner,result=self.evaluate(nav,frame,semantic_map,adjusted,source)
                self.assertFalse(planner.acquire_depth)
                self.assertTrue(all(c['bearing_advance_score']==0. for c in result['ranked_candidates']))


class CurrentPairProtocolTests(unittest.TestCase):
    def test_current_visible_bearing_can_start_local_approach_without_claiming_same_identity(self):
        import test_target_confirmation as fixtures
        harness=fixtures.ConfirmationTests('test_bearing_only_multiview_is_hypothesis_not_confirmation')
        harness.setUp()
        first=harness.observe(metric=False,point=[8.,0.,1.])
        harness.next_confirmation();current=harness.observe(metric=False,point=[8.,0.,1.])
        with mock.patch('target_confirmation.pair_geometry',return_value={
                'compatible':False,'association_basis':'none','reason':'controlled ambiguous bearing intersection'}):
            result=harness.review(source=True,current=True,same=True)
        self.assertEqual(result['confirmation_outcome'],'unconfirmed')
        self.assertEqual(result['search_status'],'searching')
        active=result['confirmation']
        self.assertEqual(active['source_capture_id'],current['capture_id'])
        self.assertTrue(active['identity_with_previous_unresolved'])
        self.assertEqual(active['previous_source_capture_id'],first['capture_id'])
        self.assertTrue(all(l.position_enu_m is None for l in harness.runtime.maps[fixtures.OWNER].landmarks.values()))
        harness.search.next_view(fixtures.OWNER,'auto','try actual current bearing')
        prediction=harness.search.sessions[fixtures.OWNER]['nbv_public']
        self.assertEqual(prediction['confirmation_goal']['purpose'],'depth_acquisition')
        self.assertEqual(prediction['confirmation_goal']['source_capture_id'],current['capture_id'])

    def test_actual_mixed_id_counterexample_is_rejected_and_error_does_not_count_as_observation(self):
        import test_nbv_search_integration as fixtures
        from test_single_vlm import tool_call
        from coverage_search import project_ground_point
        harness=fixtures.NBVGraphTests('test_multiview_graph_requires_paired_delivery_and_distinct_confirmation_move')
        bridge=fixtures.ScopedFixtureBridge(search_radius=30.,grid_resolution=1.)
        bridge.localize_stereo=lambda pair_id,boxes:fixtures.synthetic_metric_target(bridge,pair_id,boxes)
        def report(messages):
            capture=json.loads(next(m for m in reversed(messages)
                if getattr(m,'name',None)=='observe_target_search_view').content)
            pair=next(reversed(bridge.pairs.values()))
            u,v=project_ground_point(pair['left']['camera_intrinsics'],pair['body_pose_enu'],[5.,0.,1.],20.)['pixel_uv']
            return tool_call('report_scene',{'capture_id':capture['capture_id'],'summary':'controlled candidate',
                'targets':[{'label':'蓝色油桶','bbox':[u/640-.06,v/480-.07,u/640+.06,v/480+.07],
                            'surroundings':[],'position_basis':'image_only'}],
                'limitations':['synthetic fixture']},'report-'+capture['capture_id'])
        def wrong_review(messages):
            progress=json.loads(next(m for m in reversed(messages) if getattr(m,'name',None)=='report_scene').content)['search_progress']
            active=progress['confirmation']
            return tool_call('review_target_candidate',{
                'source_capture_id':'ff9f932b2e614f939a67ce799440922c', # Original real failed-model value.
                'current_capture_id':active['current_capture_id'],'current_target_id':active['current_target_ids'][0],
                'source_supported':True,'current_supported':True,'same_object':True,
                'source_description':'controlled source','current_description':'controlled current','reason':'controlled wrong-ID case'},'wrong-review')
        script=harness.prefix()[:-1]+[report,tool_call('observe_target_search_view',{},'second'),report,wrong_review]
        result,_,_=harness.run_task(bridge,script,auto_prepare=True,confirmation='multiview')
        self.assertEqual(result['status'],'failed')
        self.assertIn('IDs do not match',result['error'])
        self.assertFalse(result['task_outcome']['user_goal_completed'])
        self.assertEqual(result['search_outcome']['observation_count'],2)
        self.assertEqual(len([p for p in result['perceptions'] if p.get('status')=='ok']),2)
        self.assertFalse(bridge.telemetry['armed'])
        self.assertEqual(result['safety_landing']['status'],'ok')


if __name__=='__main__':unittest.main()
