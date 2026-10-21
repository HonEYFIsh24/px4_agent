"""LiDAR geometry and preserved-flight-sensor contracts; no simulator is started."""
import importlib.util
import math
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = Path("/home/dut/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf")


def pose(element):
    return [float(value) for value in (element.findtext("pose") or "0 0 0 0 0 0").split()]


class LidarSensorContractTests(unittest.TestCase):
    def setUp(self):
        self.model = ET.parse(ROOT / "vio/models/iris_vio/iris_vio.sdf").getroot().find("model")
        self.lidar = self.model.find("link[@name='uav_lidar']")
        self.assertIsNotNone(self.lidar)
        self.sensor = self.lidar.find("sensor[@name='uav_horizontal_lidar']")
        self.assertIsNotNone(self.sensor)

    def test_all_original_px4_model_elements_remain_identical(self):
        native = ET.parse(UPSTREAM).getroot().find("model")
        for element in native:
            with self.subTest(tag=element.tag, name=element.get("name")):
                candidates = [candidate for candidate in self.model
                              if candidate.tag == element.tag and candidate.get("name") == element.get("name")]
                self.assertEqual(len(candidates), 1)
                self.assertEqual(ET.tostring(candidates[0]), ET.tostring(element))

    def test_stereo_and_original_body_imu_contracts_are_preserved(self):
        camera_link = self.model.find("link[@name='vio_camera_link']")
        self.assertEqual(pose(camera_link), [0.12, 0.06, 0.18, 0, 0.35, 0])
        camera = camera_link.find("sensor")
        self.assertEqual(camera.get("type"), "multicamera")
        self.assertEqual(float(camera.findtext("update_rate")), 20)
        for eye in camera.findall("camera"):
            self.assertEqual([int(eye.findtext("image/"+axis)) for axis in ("width", "height")], [640, 480])
            self.assertAlmostEqual(float(eye.findtext("horizontal_fov")), math.pi/2)
        self.assertEqual(pose(camera.find("camera[@name='right']")), [0, -0.12, 0, 0, 0, 0])
        original_body_imu = self.model.find("plugin[@name='uav_original_body_imu_ros']")
        self.assertEqual(original_body_imu.get("filename"), "libuav_body_imu_plugin.so")
        self.assertEqual(pose(self.model.find("link[@name='/imu_link']")), [0, 0, 0.02, 0, 0, 0])

    def test_real_horizontal_rays_have_bounded_cost_angles_and_range(self):
        self.assertEqual(self.sensor.get("type"), "ray")
        self.assertEqual(self.sensor.findtext("always_on"), "true")
        rate = float(self.sensor.findtext("update_rate"))
        horizontal = self.sensor.find("ray/scan/horizontal")
        count = int(horizontal.findtext("samples"))
        self.assertEqual(rate, 10)
        self.assertEqual(count, 360)
        self.assertEqual(count*rate, 3600)
        self.assertEqual(float(horizontal.findtext("resolution")), 1)
        self.assertAlmostEqual(float(horizontal.findtext("min_angle")), -math.pi)
        self.assertAlmostEqual(float(horizontal.findtext("max_angle")), math.pi)
        self.assertAlmostEqual(2*math.pi/(count-1), 0.01750190893476208)
        vertical = self.sensor.find("ray/scan/vertical")
        self.assertEqual(int(vertical.findtext("samples")), 1)
        self.assertEqual([float(vertical.findtext(name)) for name in ("min_angle", "max_angle")], [0, 0])
        self.assertEqual([float(self.sensor.findtext("ray/range/"+name)) for name in ("min", "max")], [.2, 20])

    def test_fixed_lidar_frame_uses_base_flu_axes_and_documented_extrinsic(self):
        self.assertEqual(pose(self.lidar), [0, 0, .35, 0, 0, 0])
        self.assertEqual(pose(self.sensor), [0, 0, 0, 0, 0, 0])
        joint = self.model.find("joint[@name='uav_lidar_joint']")
        self.assertEqual(joint.get("type"), "fixed")
        self.assertEqual(joint.findtext("parent"), "base_link")
        self.assertEqual(joint.findtext("child"), "uav_lidar")
        plugin = self.sensor.find("plugin")
        self.assertEqual(plugin.get("filename"), "libgazebo_ros_laser.so")
        self.assertEqual(plugin.findtext("topicName"), "/uav/lidar/scan")
        self.assertEqual(plugin.findtext("frameName"), "uav_lidar")
        self.assertEqual(plugin.findtext("robotNamespace"), "/")

    def test_native_colliders_and_lidar_housing_cannot_intersect_body_horizontal_scan_plane(self):
        scan_height = pose(self.lidar)[2]+pose(self.sensor)[2]
        highest = -math.inf
        for link in self.model.findall("link"):
            for collision in link.findall("collision"):
                link_pose, collision_pose = pose(link), pose(collision)
                self.assertEqual(link_pose[3:5], [0, 0])
                self.assertEqual(collision_pose[3:5], [0, 0])
                box = collision.find("geometry/box")
                cylinder = collision.find("geometry/cylinder")
                if box is not None:
                    half_height = float(box.findtext("size").split()[2])/2
                else:
                    self.assertIsNotNone(cylinder, "New native collision shape needs explicit self-return review")
                    half_height = float(cylinder.findtext("length"))/2
                highest = max(highest, link_pose[2]+collision_pose[2]+half_height)
        self.assertAlmostEqual(highest, .055)
        self.assertGreater(scan_height-highest, .2)
        self.assertEqual(self.lidar.findall("collision"), [])
        housing = self.lidar.find("visual")
        housing_top = scan_height+pose(housing)[2]+float(housing.findtext("geometry/cylinder/length"))/2
        self.assertLess(housing_top, scan_height)

    def test_model_only_generation_does_not_overwrite_world_textures_or_vins_config(self):
        spec = importlib.util.spec_from_file_location("_lidar_assets", ROOT / "vio/generate_assets.py")
        generator = importlib.util.module_from_spec(spec)
        # This execution path must not call either image/numeric dependency.
        with mock.patch.dict("sys.modules", {"cv2": types.ModuleType("cv2"), "numpy": types.ModuleType("numpy")}):
            spec.loader.exec_module(generator)
        with tempfile.TemporaryDirectory() as directory:
            generator.ROOT = Path(directory) / "vio"
            protected = [generator.ROOT / "worlds/vio.world", generator.ROOT / "config/stereo_imu.yaml",
                         generator.ROOT / "materials/textures/features.png"]
            for path in protected:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"protected-existing-asset")
            generator.generate(model_only=True)
            for path in protected:
                self.assertEqual(path.read_bytes(), b"protected-existing-asset")
            model = ET.parse(generator.ROOT / "models/iris_vio/iris_vio.sdf").getroot().find("model")
            self.assertIsNotNone(model.find("link[@name='uav_lidar']/sensor[@type='ray']"))


if __name__ == "__main__":
    unittest.main()
