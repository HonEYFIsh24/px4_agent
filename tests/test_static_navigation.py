"""Known-map 3-D collision and frame contracts; no ROS or flight."""
import hashlib
import math
from pathlib import Path
import tempfile
import unittest
from static_navigation import StaticNavigator, collision_boxes, intersects
from scene_config import load_scene, OBSTACLE_WORLD
from mission import MissionError


class StaticPlanningTests(unittest.TestCase):
    def navigator(self, boxes=()):
        n=StaticNavigator(list(boxes),[[-3,-5,1],[12,5,10.4]],clearance=.5)
        n.bind([0,0,0],[0,0,0,1],{'position_m':[0,0,0],'orientation_xyzw':[0,0,0,1]})
        return n

    def test_clear_flight_can_change_altitude_and_reach_ten_metres(self):
        n=self.navigator()
        p=n.plan([0,0,2],[2,0,10])
        self.assertEqual(p['waypoints'],[[2.,0.,10.]])
        self.assertEqual(p['source'],'verified_static_collision_map_3d')

    def test_measured_height_near_target_is_not_confused_with_command_limits(self):
        n=StaticNavigator([], [[-3,-5,.8],[12,5,10.4]],clearance=.5)
        n.bind([0,0,0],[0,0,0,1],{'position_m':[0,0,0],'orientation_xyzw':[0,0,0,1]})
        self.assertTrue(n.segment_safe([0,0,.95],[1,0,1.]))
        self.assertTrue(n.segment_safe([0,0,10.1],[1,0,10.]))

    def test_obstacle_crossing_gets_continuously_checked_3d_detour(self):
        n=self.navigator([([3,-4,0],[4,4,5.5])])
        start,goal=[0,0,3],[8,0,3]
        self.assertFalse(n.segment_safe(start,goal))
        plan=n.plan(start,goal)
        for a,b in zip([start]+plan['waypoints'],plan['waypoints']):
            self.assertTrue(n.segment_safe(a,b))
        self.assertGreater(len(plan['waypoints']),1)
        self.assertEqual(plan['waypoints'][-1],goal)

    def test_wall_does_not_block_clear_air_above_its_inflated_top(self):
        n=self.navigator([([3,-4,0],[4,4,5.5])])
        self.assertTrue(n.segment_safe([0,0,8],[8,0,8]))
        self.assertFalse(n.segment_safe([0,0,6],[8,0,6]))

    def test_occupied_goal_and_out_of_region_goal_are_rejected(self):
        n=self.navigator([([3,-1,0],[4,1,6])])
        for goal in ([3.5,0,3],[2,0,11]):
            with self.assertRaises(MissionError): n.plan([0,0,3],goal)

    def test_registration_rotates_and_translates_without_using_world_pose_as_control(self):
        n=self.navigator()
        n.bind([1,2,3],[0,0,0,1],{'position_m':[4,5,6],
                 'orientation_xyzw':[0,0,math.sin(.3),math.cos(.3)]})
        p=[2,-1,7]
        for a,b in zip(n.to_local(n.to_world(p)),p): self.assertAlmostEqual(a,b)

    def test_public_local_bounds_are_inside_world_bounds_at_arbitrary_heading(self):
        n=self.navigator()
        n.bind([0,0,0],[0,0,0,1],{'position_m':[0,0,0],
               'orientation_xyzw':[0,0,math.sin(.6),math.cos(.6)]})
        x0,y0,x1,y1=n.diagnostics()['bounds_xy_enu_m']
        for x in (x0,x1):
            for y in (y0,y1):
                w=n.to_world([x,y,3])
                self.assertTrue(all(n.bounds[0][i]-1e-9 <= w[i] <= n.bounds[1][i]+1e-9 for i in range(3)))

    def test_vertical_takeoff_and_descent_must_clear_overhead_geometry(self):
        n=self.navigator([([-1,-1,4],[1,1,5])])
        self.assertTrue(n.vertical_safe([0,0,0],[0,0,3]))
        self.assertFalse(n.vertical_safe([0,0,0],[0,0,8]))
        self.assertFalse(n.vertical_safe([0,0,8],[0,0,0]))

    def test_slab_intersection_catches_thin_obstacles_and_tangent_contact(self):
        self.assertTrue(intersects([0,0,3],[10,0,3],[4.001,-1,2],[4.002,1,4]))
        self.assertTrue(intersects([0,1,3],[10,1,3],[4,-1,2],[5,1,4]))
        self.assertFalse(intersects([0,2,3],[10,2,3],[4,-1,2],[5,1,4]))

    def test_real_verified_world_supports_static_mode_without_exposing_boxes(self):
        scene=load_scene(OBSTACLE_WORLD,navigation_mode='static_3d')
        n=StaticNavigator.from_scene(OBSTACLE_WORLD,scene)
        self.assertGreater(len(n.boxes),0)
        self.assertEqual(scene['navigation']['altitude_mode'],'variable')
        self.assertNotIn('boxes',scene)
        self.assertNotIn('boxes',n.diagnostics())
        self.assertEqual(load_scene(OBSTACLE_WORLD)['navigation']['method'],'lidar_grid')

    def test_hash_mismatch_and_unsupported_geometry_fail_closed(self):
        with self.assertRaises(MissionError): collision_boxes(OBSTACLE_WORLD,'wrong hash')
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'scene.world'
            p.write_text('<sdf><world><model name="unknown"><static>true</static><link><collision><geometry><mesh/></geometry></collision></link></model></world></sdf>')
            with self.assertRaises(MissionError): collision_boxes(p,hashlib.sha256(p.read_bytes()).hexdigest())


if __name__=='__main__': unittest.main()
