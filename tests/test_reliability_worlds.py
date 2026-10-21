"""Independent geometry and reproducibility checks; no ROS or flight requests."""
import copy
import hashlib
import importlib.util
import itertools
import json
import math
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def import_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


generator = import_file("reliability_fixture_generator", ROOT/"scene/generate_reliability_worlds.py")
evaluator = import_file("reliability_independent_geometry", ROOT/"scripts/test_avoidance_scene.py")


def normalized_xml(element):
    """Ignore indentation only; retain every physical/visual/material value."""
    element = copy.deepcopy(element)
    for child in element.iter():
        if child.text is not None and not child.text.strip():
            child.text = None
        child.tail = None
    return generator.ET.tostring(element, encoding="utf-8")


def physical_changed_model_box(model):
    """Read physical fixture geometry directly; all changed shapes are vertical.

    This deliberately does not call the generator's rotation/bounds functions.
    Boxes are validated through their eight corners; cylinders through their
    exact axis-aligned outer limits. Unsupported poses fail this test.
    """
    pose = [float(value) for value in model.findtext("pose").split()]
    if any(abs(angle) > 1e-12 for angle in pose[3:]):
        raise AssertionError("Reliability obstacle is unexpectedly rotated")
    points = []
    for link in model.findall("link"):
        lp = [float(value) for value in link.findtext("pose", "0 0 0 0 0 0").split()]
        if any(abs(angle) > 1e-12 for angle in lp[3:]):
            raise AssertionError("Changed obstacle link is unexpectedly rotated")
        for collision in link.findall("collision"):
            cp = [float(value) for value in collision.findtext("pose", "0 0 0 0 0 0").split()]
            if any(abs(angle) > 1e-12 for angle in cp[3:]):
                raise AssertionError("Changed obstacle collision is unexpectedly rotated")
            center = [pose[k]+lp[k]+cp[k] for k in range(3)]
            box = collision.find("geometry/box")
            if box is not None:
                size = [float(value) for value in box.findtext("size").split()]
                points.extend([[center[k]+signs[k]*size[k]/2 for k in range(3)]
                               for signs in itertools.product((-1, 1), repeat=3)])
            else:
                cylinder = collision.find("geometry/cylinder")
                if cylinder is None:
                    raise AssertionError("Unexpected changed collision geometry")
                radius, length = float(cylinder.findtext("radius")), float(cylinder.findtext("length"))
                for axis, half in enumerate((radius, radius, length/2)):
                    for sign in (-1, 1):
                        point = list(center)
                        point[axis] += sign*half
                        points.append(point)
    return {axis: [min(point[k] for point in points), max(point[k] for point in points)]
            for k, axis in enumerate(("x", "y", "z"))}


class ReliabilityWorldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.output = Path(cls.temporary.name)
        cls.index = generator.generate(cls.output)
        cls.scenes = cls.index["scenes"]
        cls.layouts = {scene["id"]: json.loads((cls.output/scene["layout"]).read_text())
                       for scene in cls.scenes}

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_asset_registry_has_three_reproducible_seeds_and_one_physical_negative_trial(self):
        self.assertEqual(self.index["schema_version"], 1)
        self.assertEqual([scene["seed"] for scene in self.scenes[:3]], [17, 73, 211])
        self.assertEqual([scene["expected_outcome"] for scene in self.scenes],
                         ["success", "success", "success", "no_route"])
        for scene in self.scenes:
            for kind in ("world", "manifest", "layout", "tasks"):
                self.assertEqual(Path(scene[kind]).name, scene[kind])
                self.assertEqual(hashlib.sha256((self.output/scene[kind]).read_bytes()).hexdigest(),
                                 scene["asset_sha256"][kind])

    def test_regeneration_is_byte_reproducible_and_matches_checked_in_assets(self):
        with tempfile.TemporaryDirectory() as directory:
            generator.generate(Path(directory))
            for original in self.output.iterdir():
                self.assertEqual(original.read_bytes(), (Path(directory)/original.name).read_bytes())
                self.assertEqual(original.read_bytes(), (generator.OUTPUT/original.name).read_bytes())

    def test_generation_preserves_all_baseline_assets(self):
        paths = [generator.BASE_WORLD, generator.BASE_MANIFEST, generator.BASE_LAYOUT,
                 generator.BASE_TASKS, generator.ROOT/"worlds/logistics_yard.world",
                 generator.ROOT/"runtime_manifest.json"]
        before = {path: path.read_bytes() for path in paths}
        with tempfile.TemporaryDirectory() as directory:
            generator.generate(Path(directory))
        self.assertEqual(before, {path: path.read_bytes() for path in paths})
        self.assertEqual(hashlib.sha256(generator.BASE_WORLD.read_bytes()).hexdigest(),
                         "1f5ee08c08a3ac0e25e4ca55d5c14e29bdf4b1058c3fa5de81eb8b9567a2d72c")

    def test_only_tall_obstacles_change_and_materials_physics_ground_remain_exact(self):
        original = generator.ET.parse(str(generator.BASE_WORLD)).getroot().find("world")
        old = {model.attrib["name"]: model for model in original.findall("model")}
        for scene in self.scenes:
            current = generator.ET.parse(str(self.output/scene["world"])).getroot().find("world")
            for tag in ("include", "scene", "physics", "gui"):
                self.assertEqual(normalized_xml(original.find(tag)), normalized_xml(current.find(tag)))
            for model in current.findall("model"):
                if model.attrib["name"] not in generator.CHANGED_NAMES:
                    self.assertEqual(normalized_xml(model), normalized_xml(old[model.attrib["name"]]))

    def test_collision_truth_matches_serialized_changed_boxes_and_cylinders(self):
        baseline = {item["id"]: item for item in json.loads(generator.BASE_LAYOUT.read_text())["objects"]}
        for scene in self.scenes:
            models = {model.attrib["name"]: model for model in
                      generator.ET.parse(str(self.output/scene["world"])).findall("./world/model")}
            for obj in self.layouts[scene["id"]]["objects"]:
                expected = (physical_changed_model_box(models[obj["id"]]) if obj["id"] in generator.CHANGED_NAMES
                            else baseline[obj["id"]]["collision_aabb_world_m"])
                for axis in ("x", "y", "z"):
                    for actual, value in zip(obj["collision_aabb_world_m"][axis], expected[axis]):
                        self.assertAlmostEqual(actual, value, places=6)
                if obj["inside_navigation_xy"]:
                    self.assertLessEqual(obj["collision_aabb_world_m"]["z"][0], .003001)
                    self.assertGreaterEqual(obj["collision_aabb_world_m"]["z"][1], 5.5)

    def test_sampled_clearance_certificate_is_bounded_by_independent_exact_segment_geometry(self):
        for scene in self.scenes[:3]:
            layout = self.layouts[scene["id"]]
            witness = layout["feasibility_witness_world_m"]
            self.assertEqual(witness[0], [0., 0., 3.5])
            self.assertEqual(witness[-1], [13., 0., 3.5])
            minimum = min(evaluator.segment_rectangle_distance(first, second,
                          obj["collision_aabb_world_m"])["distance_m"]
                          for first, second in zip(witness, witness[1:]) for obj in layout["objects"])
            claimed = layout["validation"]["certified_horizontal_clearance_lower_bound_m"]
            self.assertLessEqual(claimed, minimum+1e-6)
            self.assertGreaterEqual(claimed, 1.6)
            self.assertTrue(layout["validation"]["feasible_path_exists"])
            for point in witness:
                self.assertEqual(point[2], 3.5)
                self.assertGreaterEqual(min(point[0]+3, 16-point[0], point[1]+8, 8-point[1]), 2.)

    def test_direct_goal_requires_two_wall_detours_and_turning_gap_exceeds_4_5m(self):
        fixtures = []
        for scene in self.scenes[:3]:
            layout = self.layouts[scene["id"]]
            walls = {obj["id"]: obj["collision_aabb_world_m"] for obj in layout["objects"]
                     if obj["id"] in ("obstacle_wall_1", "obstacle_wall_2")}
            self.assertGreater(walls["obstacle_wall_2"]["x"][0]-walls["obstacle_wall_1"]["x"][1], 4.5)
            for box in walls.values():
                self.assertLess(evaluator.segment_rectangle_distance([0., 0.], [13., 0.], box)["distance_m"], 1.)
            witness = layout["feasibility_witness_world_m"]
            self.assertGreater(witness[2][1], walls["obstacle_wall_1"]["y"][1]+1.6)
            self.assertLess(witness[3][1], walls["obstacle_wall_2"]["y"][0]-1.6)
            fixtures.append(json.dumps(layout["randomization"], sort_keys=True))
        self.assertEqual(len(set(fixtures)), 3)

    def test_changed_obstacles_do_not_overlap_other_physical_objects(self):
        for scene in self.scenes:
            objects = self.layouts[scene["id"]]["objects"]
            for first, second in itertools.combinations(objects, 2):
                if first["id"] not in generator.CHANGED_NAMES and second["id"] not in generator.CHANGED_NAMES:
                    continue
                a, b = first["collision_aabb_world_m"], second["collision_aabb_world_m"]
                self.assertFalse(all(min(a[axis][1], b[axis][1])-max(a[axis][0], b[axis][0]) > 1e-6
                                     for axis in ("x", "y", "z")), (scene["id"], first["id"], second["id"]))

    def test_negative_trial_is_a_full_width_collision_barrier_with_no_fabricated_witness(self):
        layout = self.layouts[self.scenes[-1]["id"]]
        self.assertEqual(layout["feasibility_witness_world_m"], [])
        self.assertFalse(layout["validation"]["feasible_path_exists"])
        box = next(obj["collision_aabb_world_m"] for obj in layout["objects"] if obj["id"] == "obstacle_wall_1")
        self.assertLess(0., box["x"][0])
        self.assertLess(box["x"][1], 13.)
        self.assertLessEqual(box["y"][0], -8.)
        self.assertGreaterEqual(box["y"][1], 8.)
        self.assertEqual(evaluator.segment_rectangle_distance([0., 0.], [13., 0.], box)["distance_m"], 0.)
        self.assertFalse(any(obj["id"] == "south_boundary_cargo_stack" for obj in layout["objects"]))

    def test_runtime_manifests_and_prompts_never_receive_collision_truth_or_witness(self):
        original = json.loads(generator.BASE_MANIFEST.read_text())
        baseline_prompt = json.loads(generator.BASE_TASKS.read_text())["tasks"][0]["prompt"]
        for scene in self.scenes:
            public = json.loads((self.output/scene["manifest"]).read_text())
            self.assertEqual(set(public), set(original))
            self.assertEqual(public["world_sha256"], scene["world_sha256"])
            for key in ("ground_plane", "flight_region", "takeoff_pad", "navigation"):
                self.assertEqual(public[key], original[key])
            tasks = json.loads((self.output/scene["tasks"]).read_text())["tasks"]
            self.assertEqual(len(tasks), 1)
            self.assertEqual(tasks[0]["goal_enu_m"], [13, 0, 3.5])
            self.assertEqual(tasks[0]["prompt"], baseline_prompt)
            for forbidden in ("objects", "collision_aabb", "feasibility_witness", "blocked_path_certificate",
                              "pillar_positions", "first_wall_upper", "waypoints"):
                self.assertNotIn(forbidden, json.dumps(public))
                self.assertNotIn(forbidden, tasks[0]["prompt"])

    def test_certificate_rejects_an_obstacle_crossing_witness(self):
        layout = self.layouts[self.scenes[0]["id"]]
        invalid = [[0., 0., 3.5], [13., 0., 3.5]]
        with self.assertRaisesRegex(ValueError, "Insufficient"):
            generator.certify(layout["objects"], invalid, blocked=False)


if __name__ == "__main__":
    unittest.main()
