import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scene'))
from import_orchard_world import triangle_boxes, flatten_ground, NS, load_dump
import scene_config
from mission import MissionError
from static_navigation import StaticNavigator


class OrchardGeometryTests(unittest.TestCase):
    def test_triangle_interiors_crossing_bins_are_enclosed(self):
        tri = np.array([[[.1, .1, .1], [5.9, .1, 2.], [.1, 5.9, 1.]]])
        boxes = triangle_boxes(tri)
        # Interior points in cells containing no original vertex must be covered.
        for a in np.linspace(0, 1, 20):
            for b in np.linspace(0, 1-a, 20):
                p = a*tri[0, 0]+b*tri[0, 1]+(1-a-b)*tri[0, 2]
                self.assertTrue(any((p >= lo).all() and (p <= hi).all() for lo, hi in boxes))

    def test_negative_cells_and_vertical_triangle_are_enclosed(self):
        tri = np.array([[[-4., -2., 0.], [-4., -2., 3.], [-4., 2., 1.]]])
        boxes = triangle_boxes(tri)
        for p in tri.reshape(-1, 3):
            self.assertTrue(any((p >= lo).all() and (p <= hi).all() for lo, hi in boxes))
        self.assertTrue(all((hi > lo).all() for lo, hi in boxes))

    def test_invalid_mesh_is_rejected(self):
        for t in [[], [[[0, 0, float('nan')]]*3], [[0, 0, 0]]]:
            with self.assertRaises(ValueError):
                triangle_boxes(t)

    def test_imported_assets_cannot_be_missing_modified_or_escape(self):
        source = ROOT/'scene/test_environments/forest_rescue_manifest.json'
        original_world = ROOT/'scene/test_environments/forest_rescue.world'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            world = root/'test.world'
            world.write_bytes(original_world.read_bytes())
            manifest = json.loads(source.read_text())
            asset = root/'texture.png'
            asset.write_bytes(b'original fixture')
            manifest['external_assets_sha256'] = {'texture.png': hashlib.sha256(asset.read_bytes()).hexdigest()}
            mp = root/'manifest.json'
            def save():
                mp.write_text(json.dumps(manifest))
                (root/'index.json').write_text(json.dumps({'schema_version': 1, 'scenes': [{
                    'world': world.name, 'manifest': mp.name, 'world_sha256': manifest['world_sha256'],
                    'asset_sha256': {'world': manifest['world_sha256'], 'manifest': hashlib.sha256(mp.read_bytes()).hexdigest()}}]}))
            save()
            with patch.object(scene_config, 'ROOT', root), patch.object(scene_config, 'STATIC_TEST_SCENE_ROOT', root):
                self.assertTrue(scene_config.load_scene(world, navigation_mode='static_3d')['verified'])
                asset.write_bytes(b'changed')
                with self.assertRaisesRegex(MissionError, 'hash mismatch'):
                    scene_config.load_scene(world, navigation_mode='static_3d')
                asset.unlink()
                with self.assertRaisesRegex(MissionError, 'missing'):
                    scene_config.load_scene(world, navigation_mode='static_3d')
                manifest['external_assets_sha256'] = {'../texture.png': '0'*64}
                save()
                with self.assertRaisesRegex(MissionError, 'identity'):
                    scene_config.load_scene(world, navigation_mode='static_3d')


class InstalledOrchardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = ROOT/'scene/test_environments/orchard_flat_test.world'
        required = [cls.world, ROOT/'scene/external_worlds/prepared/orchard_flat/flat_ground.dae',
                    ROOT/'scene/external_worlds/clearpath-pinned/cpr_orchard_gazebo/meshes/orchard_world.dae']
        if not all(p.exists() for p in required):
            raise unittest.SkipTest('external orchard assets have not been imported')

    def test_registered_scene_has_safe_pad_routes_and_no_semantic_truth(self):
        scene = scene_config.load_scene(self.world, navigation_mode='static_3d')
        self.assertTrue(scene['verified'])
        self.assertNotIn('BaseStation', json.dumps(scene))
        nav = StaticNavigator.from_scene(self.world, scene)
        nav.bind([0, 0, .333], [0, 0, 0, 1], {'position_m': [0, 0, .333], 'orientation_xyzw': [0, 0, 0, 1]})
        self.assertTrue(nav.vertical_safe([0, 0, .333], [0, 0, 10]))
        result = nav.plan([0, 0, 3.5], [4, 0, 3.5], search_origin_xy=[0, 0])
        self.assertTrue(result['full_route_within_search_volume'])
        self.assertFalse(nav.segment_safe([0, 11, 1], [0, 11, 1]))
        self.assertFalse(nav.segment_safe([-5, 0, 4], [-5, 0, 4]))

    def test_flight_region_is_supported_by_flat_collision_ground(self):
        manifest = json.loads(self.world.with_name('orchard_flat_test_manifest.json').read_text())
        ground = manifest['ground_plane']['bounds_xy_m']
        bounds = manifest['navigation']['bounds_xy_m']
        self.assertTrue(all(bounds[i] > ground[i] for i in (0, 1)))
        self.assertTrue(all(bounds[i] < ground[i] for i in (2, 3)))
        root = ET.parse(self.world)
        collision = root.find('.//model[@name="yard_ground"]/link/collision')
        pose = np.fromstring(collision.findtext('pose'), sep=' ')
        size = np.fromstring(collision.findtext('geometry/box/size'), sep=' ')
        self.assertAlmostEqual(pose[2]+size[2]/2, 0.)
        self.assertTrue(np.allclose(pose[:2]-size[:2]/2, ground[:2]))
        self.assertTrue(np.allclose(pose[:2]+size[:2]/2, ground[2:]))
        # Whole-scene rotation preserves the source tripod's relative placement.
        visual = root.find('.//visual[@name="orchard_world_link_visual_3"]')
        if visual is None:
            visual = next(v for v in root.findall('.//visual')
                          if 'BaseStationWithTripod.dae' in v.findtext('geometry/mesh/uri', ''))
        self.assertTrue(np.allclose(np.fromstring(visual.findtext('pose'), sep=' ')[:3], [0, 11, 0]))

    def test_flat_ground_preserves_original_uvs_and_does_not_modify_source(self):
        src = ROOT/'scene/external_worlds/clearpath-pinned/cpr_orchard_gazebo/meshes/orchard_world.dae'
        before = src.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'flat.dae'
            flatten_ground(src, output)
            old, new = ET.parse(src), ET.parse(output)
            for array in old.findall('.//c:float_array', NS):
                if 'map' in array.get('id', ''):
                    corresponding = new.find('.//c:float_array[@id="'+array.get('id')+'"]', NS)
                    self.assertEqual(array.text, corresponding.text)
            positions = new.find('.//c:source[@id="Plane-mesh-positions"]/c:float_array', NS)
            self.assertTrue((np.fromstring(positions.text, sep=' ').reshape(-1, 3)[:, 2] == 0).all())
        self.assertEqual(src.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
