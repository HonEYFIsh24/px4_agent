import copy
import hashlib
import tempfile
import unittest
from pathlib import Path

from scene_config import load_scene
from static_navigation import StaticNavigator
from flat_landing import flat_support_proof, LandingUnavailable

WORLD = Path(__file__).resolve().parents[1]/'scene/test_environments/courtyard_spacious.world'


class FlatLandingTests(unittest.TestCase):
    def setUp(self):
        self.scene = load_scene(WORLD, navigation_mode='static_3d')
        self.nav = StaticNavigator.from_scene(WORLD, self.scene)
        self.nav.bind([0., 0., 0.], [0., 0., 0., 1.],
                      {'position_m': [0., 0., 0.], 'orientation_xyzw': [0., 0., 0., 1.]})

    def test_authorized_flat_support_and_whole_descent_have_positive_real_geometry_proof(self):
        proof = flat_support_proof(WORLD, self.scene, self.nav, [-5., 0., 3.])
        self.assertTrue(proof['horizontal_support_verified'])
        self.assertTrue(proof['continuous_descent_verified'])
        self.assertEqual(proof['existing_collision_inflation_m'], 1.)
        self.assertFalse(proof['dynamic_objects_supported'])
        self.assertIsNone(self.nav.last_plan)

    def test_unregistered_world_and_outside_support_are_rejected(self):
        for edit in ('verified', 'ground'):
            scene = copy.deepcopy(self.scene)
            if edit == 'verified':scene['verified'] = False
            else:scene['ground_plane']['known'] = False
            with self.assertRaises(LandingUnavailable):flat_support_proof(WORLD, scene, self.nav, [-5., 0., 3.])
        with self.assertRaises(LandingUnavailable):flat_support_proof(WORLD, self.scene, self.nav, [37.5, 0., 3.])

    def test_obstacle_column_is_rejected_even_if_the_upper_hover_is_clear(self):
        # Synthetic obstacle has no semantic label; upper hover alone is clear.
        self.nav.boxes.append(([-6., -1., 0.], [-4., 1., 1.]))
        self.assertTrue(self.nav.segment_safe([-5., 0., 3.], [-5., 0., 3.]))
        with self.assertRaises(LandingUnavailable):flat_support_proof(WORLD, self.scene, self.nav, [-5., 0., 3.])

    def test_slope_or_missing_actual_support_cannot_be_replaced_by_manifest_claim(self):
        raw = WORLD.read_bytes()
        # Remove the actual ground collision while keeping a declarative plane.
        import xml.etree.ElementTree as ET
        tree = ET.fromstring(raw)
        for model in tree.find('world').findall('model'):
            for link in model.findall('link'):
                for collision in list(link.findall('collision')):
                    box = collision.find('geometry/box/size')
                    if box is not None and box.text == '76 76 0.1':link.remove(collision)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'no-support.world';path.write_bytes(ET.tostring(tree))
            scene = copy.deepcopy(self.scene);scene['world_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
            self.nav.world_sha256 = scene['world_sha256']
            with self.assertRaises(LandingUnavailable):flat_support_proof(path, scene, self.nav, [-5., 0., 3.])


if __name__ == '__main__':unittest.main()
