"""The user-authorized 20m limit agrees across tool, planner and execution contracts."""
import unittest
from pathlib import Path
from pydantic import ValidationError
from agent import TakeoffStep, GotoStep
from mission import MAX_ALTITUDE, MissionError, validate_mission
from scene_config import load_scene
from static_navigation import StaticNavigator
from viewpoint_planner import ViewpointPlanner, ViewpointConfig, ViewpointError

ROOT = Path(__file__).resolve().parents[1]


class AltitudeLimitTests(unittest.TestCase):
    def test_twenty_metres_accepted_and_excess_rejected_in_both_contracts(self):
        self.assertEqual(MAX_ALTITUDE, 20.)
        for height in (10., 15., 20.):
            self.assertEqual(TakeoffStep(action='takeoff', altitude=height).altitude, height)
            GotoStep(action='goto',x=1,y=0,altitude=height)
            validate_mission([{'action':'takeoff','altitude':height},
                              {'action':'goto','x':1,'y':0,'altitude':height}])
        for height in (.99,20.001):
            with self.assertRaises(ValidationError): TakeoffStep(action='takeoff',altitude=height)
            with self.assertRaises(ValidationError): GotoStep(action='goto',x=0,y=0,altitude=height)
            with self.assertRaises(MissionError): validate_mission([{'action':'takeoff','altitude':height}])

    def test_registered_static_scene_exposes_twenty_metres_without_changing_horizontal_scope(self):
        world=ROOT/'scene/test_environments/forest_rescue.world'
        scene=load_scene(world,navigation_mode='static_3d')
        self.assertEqual(scene['navigation']['max_altitude_m'],20.)
        nav=StaticNavigator.from_scene(world,scene)
        nav.bind([0,0,.333],[0,0,0,1],{'position_m':[0,0,.333],'orientation_xyzw':[0,0,0,1]})
        self.assertTrue(nav.vertical_safe([0,0,.333],[0,0,20.]))
        path=nav.plan([0,0,18.],[0,0,20.],search_origin_xy=[0,0])
        self.assertTrue(path['full_route_within_search_volume'])
        with self.assertRaises(MissionError):nav.plan([0,0,18.],[0,0,20.01],search_origin_xy=[0,0])
        with self.assertRaises(MissionError):nav.plan([0,0,18.],[13,0,18.],search_origin_xy=[0,0])

    def test_twenty_metre_limit_does_not_override_overhead_collision(self):
        nav=StaticNavigator([([-1,-1,14],[1,1,16])],[[-5,-5,.8],[5,5,20.4]],clearance=1.)
        nav.bind([0,0,0],[0,0,0,1],{'position_m':[0,0,0],'orientation_xyzw':[0,0,0,1]})
        self.assertFalse(nav.vertical_safe([0,0,.3],[0,0,20.]))

    def test_nbv_preserves_half_metre_margin_and_search_radius(self):
        planner=ViewpointPlanner(ViewpointConfig(heading_mode='discrete'))
        candidates=planner.generate_candidates({'position_m':[0,0,20.]},[0,0],0.)
        self.assertTrue(candidates)
        self.assertTrue(all(1.5<=x['position_enu_m'][2]<=19.5 for x in candidates))
        with self.assertRaises(ViewpointError):planner.generate_candidates({'position_m':[0,0,20.1]},[0,0],0.)
