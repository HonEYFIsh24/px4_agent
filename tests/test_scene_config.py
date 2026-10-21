"""Scene identity, ENU/world flight volume and projection tests without ROS."""
import hashlib
import json
import math
import tempfile
import threading
import types
import unittest
from pathlib import Path
from unittest import mock

import scene_config
from mission import MissionError
import test_bridge_camera as camera_contract
from test_vision import snapshot, target
from vision import target_location


PROJECT_DIR = Path(__file__).resolve().parents[1]
WORLD = PROJECT_DIR / "scene/worlds/logistics_yard.world"
MANIFEST = PROJECT_DIR / "scene/runtime_manifest.json"
IDENTITY = [0.0, 0.0, 0.0, 1.0]
bridge = camera_contract.bridge


def yaw_quaternion(yaw):
    return [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]


def known_scene():
    return {"id": "unit-test-yard", "world_sha256": "test-only-hash", "verified": True,
            "ground_plane": {"known": True, "z_m": 0.003,
                             "bounds_xy_m": [-20.0, -20.0, 20.0, 20.0]},
            "flight_region": {"frame": "gazebo_world", "bounds_min_m": [-3.0, -4.0, -0.2],
                              "bounds_max_m": [8.0, 4.0, 6.0], "margin_m": 0.6}}


def world_pose(position=None, orientation=None):
    return {"position_m": position or [0.0, 0.0, 0.12],
            "orientation_xyzw": orientation or list(IDENTITY)}


class SceneIdentityTests(unittest.TestCase):
    def test_builtin_world_and_trusted_manifest_activate_known_ground_and_volume(self):
        scene = scene_config.load_scene(WORLD, MANIFEST)
        self.assertEqual(scene["id"], "logistics_yard")
        self.assertEqual(scene["world_sha256"], hashlib.sha256(WORLD.read_bytes()).hexdigest())
        self.assertTrue(scene["ground_plane"]["known"])
        self.assertEqual(scene["ground_plane"]["bounds_xy_m"], [-20, -20, 20, 20])
        self.assertEqual(scene["flight_region"]["frame"], "gazebo_world")
        self.assertEqual(scene["flight_region"]["bounds_min_m"], [-3, -4, -0.2])
        self.assertEqual(scene["flight_region"]["bounds_max_m"], [8, 4, 6])

    def test_builtin_default_manifest_is_resolved_without_external_ground_truth(self):
        scene = scene_config.load_scene(WORLD)
        self.assertTrue(scene["ground_plane"]["known"])
        encoded = json.dumps(scene)
        for ground_truth in ("objects", "red_shipping_container", "white_delivery_van", "aabb_world_m"):
            self.assertNotIn(ground_truth, encoded)

    def test_unknown_world_never_inherits_legacy_floor_or_flight_region(self):
        with tempfile.TemporaryDirectory() as directory:
            unknown = Path(directory) / "external.world"
            unknown.write_text("<sdf version='1.6'><world name='external'/></sdf>")
            scene = scene_config.load_scene(unknown)
            self.assertFalse(scene["ground_plane"]["known"])
            self.assertFalse(scene.get("flight_region"))
            self.assertEqual(scene["world_sha256"], hashlib.sha256(unknown.read_bytes()).hexdigest())

    def test_old_vio_world_is_not_assumed_to_be_the_active_yard(self):
        scene = scene_config.load_scene(PROJECT_DIR / "vio/worlds/vio.world")
        self.assertFalse(scene["ground_plane"]["known"])
        self.assertFalse(scene.get("flight_region"))

    def test_external_world_cannot_activate_ground_by_copying_trusted_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            copied = Path(directory) / "copied.world"
            copied.write_bytes(WORLD.read_bytes())
            scene = scene_config.load_scene(copied, MANIFEST)
            self.assertFalse(scene["ground_plane"]["known"])
            self.assertFalse(scene.get("flight_region"))

    def test_untrusted_manifest_cannot_activate_even_matching_builtin_world_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            copied = Path(directory) / "forged_manifest.json"
            copied.write_bytes(MANIFEST.read_bytes())
            scene = scene_config.load_scene(WORLD, copied)
            self.assertFalse(scene["ground_plane"]["known"])
            self.assertFalse(scene.get("flight_region"))

    def test_hash_mismatch_deactivates_known_projection_and_flight_volume(self):
        original = Path.read_bytes

        def changed_world(path):
            data = original(path)
            return data + b"\n<!-- changed geometry -->\n" if path.resolve() == WORLD.resolve() else data

        with mock.patch.object(Path, "read_bytes", changed_world):
            scene = scene_config.load_scene(WORLD, MANIFEST)
        self.assertFalse(scene["ground_plane"]["known"])
        self.assertFalse(scene.get("flight_region"))


class SceneTrajectoryTests(unittest.TestCase):
    def check(self, steps, start=None, home=None, pose=None, local_orientation=None):
        return scene_config.validate_scene_trajectory(known_scene(), steps,
            start or [0.0, 0.0, 0.0], home or [0.0, 0.0, 0.0],
            pose or world_pose(), local_orientation or list(IDENTITY))

    def test_complete_sortie_stays_within_convex_clear_volume(self):
        self.check([{"action": "takeoff", "altitude": 3.0},
                    {"action": "goto", "x": 6.0, "y": 2.0, "altitude": 3.0},
                    {"action": "goto", "x": 2.0, "y": -2.0, "altitude": 3.0},
                    {"action": "hold", "duration": 2.0},
                    {"action": "return_home"}, {"action": "land"}])

    def test_waypoint_margin_and_world_altitude_are_enforced(self):
        for step in ({"action": "goto", "x": 7.5, "y": 0.0, "altitude": 3.0},
                     {"action": "goto", "x": 0.0, "y": 3.5, "altitude": 3.0},
                     {"action": "goto", "x": -2.5, "y": 0.0, "altitude": 3.0},
                     {"action": "goto", "x": 0.0, "y": 0.0, "altitude": 6.0}):
            with self.subTest(step=step), self.assertRaises(MissionError):
                self.check([step])

    def test_real_start_must_be_inside_clear_volume_before_valid_waypoint_is_accepted(self):
        with self.assertRaises(MissionError):
            self.check([{"action": "goto", "x": 2.0, "y": 0.0, "altitude": 3.0}],
                       pose=world_pose([-3.0, 0.0, 0.12]))

    def test_return_home_target_is_checked_at_current_height(self):
        with self.assertRaises(MissionError):
            self.check([{"action": "return_home"}], start=[0.0, 0.0, 3.0],
                       home=[10.0, 0.0, 0.0], pose=world_pose([0.0, 0.0, 3.12]))

    def test_takeoff_vertical_endpoint_is_checked(self):
        with self.assertRaises(MissionError):
            self.check([{"action": "takeoff", "altitude": 6.0}])

    def test_enu_world_translation_is_applied_instead_of_comparing_raw_enu_coordinates(self):
        self.check([{"action": "goto", "x": 8.0, "y": 0.0, "altitude": 3.0}],
                   start=[10.0, 0.0, 0.0], home=[10.0, 0.0, 0.0],
                   pose=world_pose([7.0, 0.0, 0.12]))

    def test_yaw_difference_rotates_local_route_into_world_before_bounds_check(self):
        with self.assertRaises(MissionError):
            self.check([{"action": "goto", "x": 4.0, "y": 0.0, "altitude": 3.0}],
                       pose=world_pose(orientation=yaw_quaternion(math.pi / 2)))
        self.check([{"action": "goto", "x": 6.0, "y": 0.0, "altitude": 3.0}],
                   pose=world_pose(orientation=yaw_quaternion(math.pi / 2)),
                   local_orientation=yaw_quaternion(math.pi / 2))

    def test_local_heading_is_subtracted_from_world_heading(self):
        with self.assertRaises(MissionError):
            self.check([{"action": "goto", "x": 4.0, "y": 0.0, "altitude": 3.0}],
                       local_orientation=yaw_quaternion(math.pi / 2))

    def test_nonfinite_telemetry_cannot_validate_a_scene_trajectory(self):
        for position in ([float("nan"), 0.0, 0.12], [0.0, float("inf"), 0.12]):
            with self.subTest(position=position), self.assertRaises(MissionError):
                self.check([{"action": "takeoff", "altitude": 2.0}], pose=world_pose(position))


class SceneGroundProjectionTests(unittest.TestCase):
    def locate(self, scene):
        photo = snapshot()
        photo.pop("scene", None)
        if scene is not None:
            photo["scene"] = scene
        result = target("flat_ground_marking")
        target_location(result, photo)
        return result["approximate_location"]

    def test_missing_or_unknown_scene_does_not_receive_legacy_ground_projection(self):
        for scene in (None, {"id": "external", "ground_plane": {"known": False}},
                      {"id": "external", "ground_plane": {"z_m": 0.003,
                                                         "bounds_xy_m": [-10, -10, 10, 10]}}):
            with self.subTest(scene=scene):
                self.assertEqual(self.locate(scene)["method"], "bearing_only")

    def test_active_scene_ground_height_is_used_instead_of_old_floor_constant(self):
        scene = known_scene()
        scene["ground_plane"].update(z_m=1.5, bounds_xy_m=[-4.0, -4.0, 4.0, 4.0])
        location = self.locate(scene)
        self.assertEqual(location["method"], "ground_plane_projection")
        self.assertEqual(location["frame"], "gazebo_world")
        self.assertEqual([location[key] for key in ("x_m", "y_m", "z_m")], [1.0, 2.0, 1.5])
        self.assertIn("1.5", " ".join(location["assumptions"]))

    def test_active_scene_ground_bounds_are_enforced(self):
        scene = known_scene()
        scene["ground_plane"]["bounds_xy_m"] = [-0.5, -0.5, 0.5, 0.5]
        self.assertEqual(self.locate(scene)["method"], "bearing_only")

    def test_invalid_ground_metadata_falls_back_to_direction(self):
        for invalid in ({"known": True, "z_m": float("nan"), "bounds_xy_m": [-20, -20, 20, 20]},
                        {"known": True, "z_m": 0.003, "bounds_xy_m": [20, -20, -20, 20]},
                        {"known": True, "z_m": 0.003, "bounds_xy_m": [-20, -20, 20]}):
            with self.subTest(invalid=invalid):
                scene = known_scene()
                scene["ground_plane"] = invalid
                self.assertEqual(self.locate(scene)["method"], "bearing_only")


class SceneBridgeTests(unittest.TestCase):
    def setUp(self):
        # Reuse a local ROS-stub fixture, retaining its cleanup without starting a master.
        fixture = camera_contract.CameraCaptureTests("test_real_frame_and_calibration_remain_one_immutable_snapshot")
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.controller = fixture.controller
        self.controller.scene = known_scene()
        self.controller.gazebo_time = 999.95
        self.controller.submit_lock = threading.Lock()
        self.controller.ready = lambda: None
        self.controller.position = lambda: [0.0, 0.0, 0.0]
        self.controller.home = self.controller.target = self.controller.mission = None
        self.controller.state_msg = types.SimpleNamespace(connected=True, armed=False, mode="POSCTL")
        self.controller.landed = 1
        bridge.ExtendedState.LANDED_STATE_ON_GROUND = 1
        self.controller.pose = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: 10.0)),
            pose=types.SimpleNamespace(orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))

    def test_capture_carries_immutable_scene_geometry_without_object_ground_truth(self):
        value = self.controller.capture_camera()
        self.assertEqual(value["scene"], self.controller.scene)
        self.assertNotIn("objects", json.dumps(value["scene"]))
        value["scene"]["ground_plane"]["z_m"] = 999
        self.assertEqual(self.controller.scene["ground_plane"]["z_m"], 0.003)

    def test_state_publishes_the_same_scene_geometry(self):
        self.controller.vio_snapshot = lambda: {}
        self.controller.gazebo_snapshot = lambda: {}
        self.controller.camera_snapshot = lambda: {}
        value = self.controller.snapshot()
        self.assertEqual(value["scene"], self.controller.scene)
        value["scene"]["flight_region"]["margin_m"] = 0
        self.assertEqual(self.controller.scene["flight_region"]["margin_m"], 0.6)

    def test_outside_plan_is_rejected_before_any_worker_can_start(self):
        with mock.patch.object(bridge.threading, "Thread") as worker:
            with self.assertRaises(bridge.MissionError):
                self.controller.submit([{"action": "takeoff", "altitude": 2.0},
                    {"action": "goto", "x": 7.0, "y": 0.0, "altitude": 2.0}])
            worker.assert_not_called()
        self.assertIsNone(self.controller.mission)

    def test_scene_validation_rejects_stale_or_unaligned_world_pose(self):
        for stale in (True, False):
            with self.subTest(stale=stale), mock.patch.object(bridge.threading, "Thread") as worker:
                self.controller.gazebo_time = 998.0 if stale else 999.95
                self.controller.gazebo_history[0]["stamp_sim_s"] = 10.004 if stale else 9.9
                with self.assertRaises(bridge.MissionError):
                    self.controller.submit([{"action": "takeoff", "altitude": 2.0}])
                worker.assert_not_called()

    def test_validated_plan_is_accepted_with_only_a_mock_worker(self):
        with mock.patch.object(bridge.threading, "Thread") as worker:
            result = self.controller.submit([{"action": "takeoff", "altitude": 2.0},
                                             {"action": "land"}])
            self.assertEqual(result["mission"]["status"], "running")
            worker.assert_called_once()
            worker.return_value.start.assert_called_once()

if __name__ == "__main__":
    unittest.main()
