import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock
import scene_config
from static_navigation import StaticNavigator
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scene'))
from generate_forest_world import generate
from generate_test_environments import generate as courtyard

class ForestEnvironmentTests(unittest.TestCase):
    def test_reproducibility_registry_and_semantic_separation(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); courtyard(root); world=generate(root); first=world.read_bytes()
            generate(root); self.assertEqual(first,world.read_bytes())
            courtyard(root)
            self.assertEqual(len(json.loads((root/'index.json').read_text())['scenes']),2)
            with mock.patch.object(scene_config,'STATIC_TEST_SCENE_ROOT',root):
                s=scene_config.load_scene(world,navigation_mode='static_3d')
                self.assertTrue(s['verified'])
                self.assertNotIn('orange_rescue_tent',json.dumps(s))
                n=StaticNavigator.from_scene(world,s)
                n.bind([0,0,.333],[0,0,0,1],{'position_m':[0,0,.333],'orientation_xyzw':[0,0,0,1]})
                self.assertTrue(n.vertical_safe([0,0,.333],[0,0,10]))
                self.assertFalse(n.segment_safe([4,1,1],[4,1,1]))
                # Distinct geometrically safe observation region; no target coordinates enter runtime.
                self.assertTrue(n.segment_safe([0,-3,3],[5,-3,3]))
                world.write_bytes(first+b'\n')
                self.assertFalse(scene_config.load_scene(world,navigation_mode='static_3d')['verified'])
