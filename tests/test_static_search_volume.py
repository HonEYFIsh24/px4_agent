"""Full-route search-volume checks must constrain A*, not just candidate goals."""
import copy
import math
import unittest

from mission import MissionError
from static_navigation import StaticNavigator


class SearchVolumeTests(unittest.TestCase):
    def navigator(self, half_width):
        nav = StaticNavigator([([-.2, -half_width, 0.], [.2, half_width, 11.])],
                              [[-6., -6., .8], [6., 6., 10.4]], clearance=.1)
        nav.bind([0., 0., 0.], [0., 0., 0., 1.],
                 {'position_m': [0., 0., 0.], 'orientation_xyzw': [0., 0., 0., 1.]})
        return nav

    def test_endpoint_inside_disk_is_insufficient_when_only_detour_leaves_disk(self):
        nav = self.navigator(4.1)
        start, goal = [-3., 0., 3.], [3., 0., 3.]
        unrestricted = nav.plan(start, goal)
        self.assertTrue(any(math.hypot(*p[:2]) > 4 for p in unrestricted['waypoints']))
        executing = copy.deepcopy(nav.last_plan)
        with self.assertRaises(MissionError):
            nav.preview_path(start, goal, [0., 0.], search_radius_m=4.)
        self.assertEqual(nav.last_plan, executing)

    def test_in_disk_detour_has_continuous_clearance_and_preserves_executing_plan(self):
        nav = self.navigator(1.1)
        nav.last_plan = {'executing': 'preserved'}
        start, goal = [-3., 0., 3.], [3., 0., 3.]
        result = nav.preview_path(start, goal, [0., 0.], search_radius_m=4.)
        self.assertTrue(result['full_route_within_search_volume'])
        points = [start, *result['waypoints']]
        self.assertTrue(all(math.hypot(*p[:2]) <= 4.+1e-9 for p in points))
        self.assertTrue(all(nav.segment_safe(a, b) for a, b in zip(points, points[1:])))
        self.assertEqual(nav.last_plan, {'executing': 'preserved'})

    def test_invalid_radius_and_out_of_volume_start_or_goal_are_rejected(self):
        nav = self.navigator(1.)
        for radius in (True, 0., -1., 13., float('nan')):
            with self.assertRaises(ValueError):
                nav.preview_path([-3., 0., 3.], [3., 0., 3.], [0., 0.], radius)
        for start, goal in (([-5., 0., 3.], [3., 0., 3.]),
                            ([-3., 0., 3.], [5., 0., 3.]),
                            ([-3., 0., .9], [3., 0., 3.])):
            with self.assertRaises(MissionError):
                nav.preview_path(start, goal, [0., 0.], 4.)

    def test_rotated_translated_registration_preserves_valid_height_endpoints(self):
        for offset in (.0669734795094401, .6765, .706):
            nav = StaticNavigator([], [[-20., -20., .8], [20., 20., 20.]])
            nav.bind([0., 0., 0.], [0., 0., 0., 1.],
                     {'position_m': [.15, -.11, offset],
                      'orientation_xyzw': [0., 0., math.sin(.047), math.cos(.047)]})
            for height in (1., 10.):
                result = nav.preview_path([0., 0., height], [1., 0., height], [0., 0.])
                self.assertTrue(result['full_route_within_search_volume'])
                self.assertLess(math.dist(result['waypoints'][-1], [1., 0., height]), 1e-8)
