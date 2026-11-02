import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import scene_config
from mission import MissionError
from static_navigation import StaticNavigator, collision_boxes

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scene'))
from generate_test_environments import generate


class AlternateEnvironmentTests(unittest.TestCase):
    def test_generated_scene_is_reproducible_and_baseline_is_unchanged(self):
        original = hashlib.sha256(scene_config.OBSTACLE_WORLD.read_bytes()).hexdigest()
        with tempfile.TemporaryDirectory() as d:
            path = generate(Path(d))
            first = path.read_bytes()
            generate(Path(d))
            self.assertEqual(first, path.read_bytes())
        self.assertEqual(original, hashlib.sha256(scene_config.OBSTACLE_WORLD.read_bytes()).hexdigest())

    def test_registered_scene_uses_static_map_without_leaking_target_geometry(self):
        path = ROOT/'scene/test_environments/courtyard_delivery.world'
        scene = scene_config.load_scene(path, navigation_mode='static_3d')
        self.assertTrue(scene['verified'])
        self.assertEqual(scene['navigation']['method'], 'static_3d')
        public = json.dumps(scene)
        for forbidden in ('target_ids', 'collision_bounds', 'white_delivery_van', 'traffic_cone_1'):
            self.assertNotIn(forbidden, public)
        self.assertGreater(len(collision_boxes(path, scene['world_sha256'])), 10)
        n = StaticNavigator.from_scene(path, scene)
        n.bind([0., 0., .333], [0., 0., 0., 1.],
               {'position_m': [0., 0., .333], 'orientation_xyzw': [0., 0., 0., 1.]})
        self.assertTrue(n.vertical_safe([0., 0., .333], [0., 0., 10.]))
        self.assertFalse(n.segment_safe([4., 0., 2.], [4., 0., 2.]))

    def test_low_obstacles_require_static_profile_and_unknown_scene_is_not_trusted(self):
        path = ROOT/'scene/test_environments/courtyard_delivery.world'
        with self.assertRaises(MissionError): scene_config.load_scene(path)
        with tempfile.TemporaryDirectory() as d:
            generated = generate(Path(d))
            self.assertFalse(scene_config.load_scene(generated, navigation_mode='static_3d')['verified'])
            with mock.patch.object(scene_config, 'STATIC_TEST_SCENE_ROOT', Path(d)):
                self.assertTrue(scene_config.load_scene(generated, navigation_mode='static_3d')['verified'])
                generated.write_text(generated.read_text().replace('courtyard_screen', 'tampered_screen'))
                self.assertFalse(scene_config.load_scene(generated, navigation_mode='static_3d')['verified'])
