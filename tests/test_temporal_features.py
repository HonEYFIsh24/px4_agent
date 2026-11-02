"""Known geometry checks; temporal matches remain experimental, not class truth."""
import unittest
from coverage_search import project_ground_point
from test_semantic_map import navigator, observation
from replay_temporal_features import geometry, spatially_distinct


class TemporalGeometryTests(unittest.TestCase):
    def test_consistent_projection_has_parallax_but_wrong_pixel_fails(self):
        nav = navigator(); views = []
        for pos in ([0., 0., 3.], [0., 2., 3.]):
            frame, _, data = observation(nav, position=pos)
            snap = frame['snapshot']
            views.append({'camera_info': snap['camera_intrinsics'], 'body_pose_enu': snap['body_pose_enu'],
                          'bearing_origin_enu_m': data['camera_origin_enu_m']})
        pixels = [project_ground_point(v['camera_info'], v['body_pose_enu'], [5., 0., 1.], 20.)['pixel_uv'] for v in views]
        result = geometry(*views, *pixels)
        self.assertGreater(result['parallax_deg'], 3.)
        self.assertLess(result['ray_gap_deg'], .01)
        self.assertIsNone(geometry(*views, pixels[0], [30., 30.]))
        self.assertIsNone(geometry(views[0], views[0], pixels[0], pixels[0]))

    def test_repeated_features_are_suppressed_in_either_image(self):
        def point(a, b, residual):
            return {'source_pixel': a, 'current_pixel': b, 'geometry': {'ray_gap_deg': residual}}
        points = [point([0, 0], [0, 0], .1), point([1, 1], [30, 30], .2),
                  point([30, 30], [1, 1], .3), point([30, 30], [30, 30], .4)]
        self.assertEqual(spatially_distinct(points), [points[0], points[3]])
