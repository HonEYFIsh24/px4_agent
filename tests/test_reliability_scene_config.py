"""Registered-scene trust boundaries, independent of world generation or ROS."""
import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import scene_config
from mission import MissionError


ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN_PUBLIC = ("objects", "collision_aabb", "feasibility_witness", "seed",
                    "expected_outcome", "pillar_positions", "layout", "tasks", "waypoints")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class CheckedInReliabilityScenesTests(unittest.TestCase):
    def test_all_four_registered_assets_enable_navigation_without_exposing_evaluation_truth(self):
        directory = ROOT / "scene/reliability"
        index = json.loads((directory / "index.json").read_text())
        self.assertEqual(len(index["scenes"]), 4)
        self.assertEqual({item["id"] for item in index["scenes"]}, {
            "reliability_seed_17", "reliability_seed_73", "reliability_seed_211", "reliability_blocked_17"})
        for item in index["scenes"]:
            with self.subTest(scene=item["id"]):
                public = scene_config.load_scene(directory / item["world"])
                self.assertTrue(public["verified"])
                self.assertTrue(public["navigation"]["enabled"])
                self.assertEqual(public["navigation"]["method"], "lidar_grid")
                self.assertGreaterEqual(public["navigation"]["minimum_clearance_m"], 1.)
                self.assertEqual(public["navigation"]["flight_altitude_m"], 3.5)
                self.assertEqual(public["world_sha256"], digest(directory / item["world"]))
                # The scene identifier may mention its seed; only public keys
                # are checked here so normal identity remains usable.
                self.assertTrue(set(public).isdisjoint(FORBIDDEN_PUBLIC))
                self.assertEqual(set(public), {"id", "world_sha256", "verified", "ground_plane",
                                               "flight_region", "navigation", "takeoff_pad"})
                self.assertNotIn("collision_aabb", json.dumps(public))
                self.assertNotIn("feasibility_witness", json.dumps(public))


class ReliabilitySceneRegistryTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.outer = Path(temporary.name)
        self.directory = self.outer / "registry"
        self.directory.mkdir()
        patch = mock.patch.object(scene_config, "RELIABILITY_SCENE_ROOT", self.directory)
        patch.start()
        self.addCleanup(patch.stop)
        self.world = self.directory / "registered.world"
        self.world.write_text("<sdf version='1.6'><world name='registered'/></sdf>")
        self.manifest_path = self.directory / "registered_manifest.json"
        self.manifest = {
            "id": "unit_registered", "world_sha256": digest(self.world),
            "ground_plane": {"known": True, "z_m": .003, "bounds_xy_m": [-20., -20., 20., 20.]},
            "flight_region": {"frame": "gazebo_world", "bounds_min_m": [-2., -3., -.2],
                              "bounds_max_m": [10., 3., 6.], "margin_m": .6},
            "navigation": {"enabled": True, "method": "lidar_grid", "bounds_xy_m": [-2., -3., 10., 3.],
                           "flight_altitude_m": 3.5, "minimum_clearance_m": 1.},
            "takeoff_pad": {"center_world_m": [0., 0., .003], "radius_m": 1., "vertical_clearance_m": 6.}}
        self.manifest_path.write_text(json.dumps(self.manifest))
        self.entry = {"world": self.world.name, "manifest": self.manifest_path.name,
                      "world_sha256": digest(self.world),
                      "asset_sha256": {"world": digest(self.world), "manifest": digest(self.manifest_path)}}
        self.registry = {"schema_version": 1, "scenes": [self.entry]}
        self.write_index()

    def write_index(self):
        (self.directory / "index.json").write_text(json.dumps(self.registry))

    def update_manifest(self):
        self.manifest_path.write_text(json.dumps(self.manifest))
        self.entry["asset_sha256"]["manifest"] = digest(self.manifest_path)
        self.write_index()

    def assert_closed(self, world=None, manifest=None):
        try:
            result = scene_config.load_scene(world or self.world, manifest)
        except (MissionError, ValueError, TypeError, AttributeError, KeyError, OSError):
            return
        self.assertIs(result["verified"], False)
        self.assertIs(result["ground_plane"]["known"], False)
        self.assertFalse(result.get("flight_region"))
        self.assertFalse(result.get("navigation", {}).get("enabled"))

    def test_hash_registered_minimal_scene_enables_only_public_geometry(self):
        self.manifest.update(objects=[{"id": "private_obstacle"}], collision_aabb_world_m=[1, 2],
                             feasibility_witness_world_m=[[8, 2]], seed=123, layout="private-layout.json")
        self.update_manifest()
        result = scene_config.load_scene(self.world)
        self.assertTrue(result["verified"])
        self.assertEqual(result["navigation"]["minimum_clearance_m"], 1.)
        self.assertEqual(result["navigation"]["bounds_xy_m"], [-2., -3., 10., 3.])
        self.assertEqual(result["takeoff_pad"]["center_world_m"], [0., 0., .003])
        self.assertNotIn("private_obstacle", json.dumps(result))
        self.assertTrue(set(result).isdisjoint(FORBIDDEN_PUBLIC))

    def test_unknown_same_directory_world_cannot_borrow_registered_manifest(self):
        other = self.directory / "unregistered.world"
        other.write_bytes(self.world.read_bytes())
        self.assert_closed(other, self.manifest_path)

    def test_world_outside_registered_directory_cannot_borrow_identical_hash_or_manifest(self):
        external = self.outer / self.world.name
        external.write_bytes(self.world.read_bytes())
        self.assert_closed(external, self.manifest_path)

    def test_external_identical_manifest_is_not_the_registered_manifest(self):
        external = self.outer / self.manifest_path.name
        external.write_bytes(self.manifest_path.read_bytes())
        self.assert_closed(self.world, external)

    def test_missing_registry_deactivates_all_registered_geometry(self):
        (self.directory / "index.json").unlink()
        self.assert_closed()

    def test_schema_version_requires_integer_one(self):
        for version in (True, False, "1", 0, 2, None):
            with self.subTest(version=version):
                self.registry["schema_version"] = version
                self.write_index()
                self.assert_closed()

    def test_registry_scene_collection_must_be_a_list(self):
        for collection in ({"world": self.world.name}, "scene", None):
            with self.subTest(collection=collection):
                self.registry["scenes"] = collection
                self.write_index()
                self.assert_closed()

    def test_absolute_or_escaping_registered_paths_fail_closed(self):
        for key in ("world", "manifest"):
            original = self.entry[key]
            for replacement in (str(self.directory / original), "../" + original, "nested/" + original):
                with self.subTest(key=key, value=replacement):
                    self.entry[key] = replacement
                    self.write_index()
                    self.assert_closed()
            self.entry[key] = original

    def test_paths_with_dot_segments_are_not_direct_relative_children(self):
        (self.directory / "nested").mkdir()
        for key in ("world", "manifest"):
            original = self.entry[key]
            for replacement in ("./" + original, "nested/../" + original,
                                "../" + self.directory.name + "/" + original):
                with self.subTest(key=key, value=replacement):
                    self.entry[key] = replacement
                    self.write_index()
                    self.assert_closed()
            self.entry[key] = original

    def test_manifest_symlink_outside_registry_cannot_confer_trust(self):
        outside = self.outer / "external_manifest.json"
        outside.write_bytes(self.manifest_path.read_bytes())
        link = self.directory / "linked_manifest.json"
        link.symlink_to(outside)
        self.entry["manifest"] = link.name
        self.write_index()
        self.assert_closed()

    def test_duplicate_world_registration_is_ambiguous_even_with_identical_hashes(self):
        self.registry["scenes"].append(copy.deepcopy(self.entry))
        self.write_index()
        self.assert_closed()

    def test_world_byte_mutation_deactivates_navigation(self):
        self.world.write_bytes(self.world.read_bytes() + b"<!-- changed geometry -->")
        self.assert_closed()

    def test_manifest_byte_mutation_deactivates_navigation(self):
        self.manifest_path.write_bytes(self.manifest_path.read_bytes() + b"\n")
        self.assert_closed()

    def test_each_registered_world_and_manifest_hash_is_required_and_matched(self):
        original = copy.deepcopy(self.entry)
        for changed in ("world_sha256", "asset_world", "asset_manifest", "missing_asset_world",
                        "missing_asset_manifest", "missing_asset_sha256"):
            with self.subTest(changed=changed):
                self.entry.clear()
                self.entry.update(copy.deepcopy(original))
                if changed == "world_sha256":
                    self.entry[changed] = "0" * 64
                elif changed.startswith("asset_"):
                    self.entry["asset_sha256"][changed[6:]] = "0" * 64
                elif changed == "missing_asset_sha256":
                    del self.entry["asset_sha256"]
                else:
                    del self.entry["asset_sha256"][changed[14:]]
                self.write_index()
                self.assert_closed()

    def test_manifest_internal_world_hash_must_also_match(self):
        self.manifest["world_sha256"] = "0" * 64
        self.update_manifest()
        self.assert_closed()

    def test_clearance_cannot_be_reduced_even_after_manifest_reregistration(self):
        for value in (.999, 0., True, float("nan"), float("inf")):
            with self.subTest(clearance=value):
                self.manifest["navigation"]["minimum_clearance_m"] = value
                self.update_manifest()
                self.assert_closed()

    def test_navigation_contract_cannot_switch_to_unknown_method_or_altitude(self):
        for field, value in (("enabled", False), ("method", "ground_truth"), ("flight_altitude_m", 5.1),
                             ("bounds_xy_m", [-2., -3., 11., 3.])):
            with self.subTest(field=field):
                original = copy.deepcopy(self.manifest["navigation"])
                self.manifest["navigation"][field] = value
                self.update_manifest()
                self.assert_closed()
                self.manifest["navigation"] = original


if __name__ == "__main__":
    unittest.main()
