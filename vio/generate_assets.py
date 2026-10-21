"""Generate an Iris with stereo, original body IMU and a horizontal ROS LiDAR."""
import argparse
import math
import os
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
PX4 = Path('/home/dut/PX4-Autopilot')
PITCH = 0.35
WIDTH, HEIGHT = 640, 480
LIDAR_HEIGHT = 0.35
LIDAR_SAMPLES = 360
LIDAR_RATE = 10


def _sdf_transform(element):
    """Return a pose in its enclosing SDF frame; never fit measured poses."""
    pose = element.find('pose')
    if pose is not None and pose.get('relative_to'):
        raise ValueError('Rigid camera conversion requires explicit enclosing-frame poses')
    values = [float(value) for value in (pose.text if pose is not None else '0 0 0 0 0 0').split()]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError('SDF pose must contain six finite values')
    x, y, z, roll, pitch, yaw = values
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = [x, y, z]
    return transform


def _write_sdf_transform(element, transform):
    rotation = transform[:3, :3]
    angles = [math.atan2(rotation[2, 1], rotation[2, 2]),
              math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0])),
              math.atan2(rotation[1, 0], rotation[0, 0])]
    pose = element.find('pose')
    if pose is None:
        pose = ET.SubElement(element, 'pose')
    pose.attrib.clear()
    pose.text = ' '.join('%.17g' % value for value in list(transform[:3, 3]) + angles)


def _inertial_in_parent(link, parent_T_link):
    inertial = link.find('inertial')
    mass = float(inertial.findtext('mass'))
    inertia = inertial.find('inertia')
    values = {name: float(inertia.findtext(name, '0')) for name in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')}
    tensor = np.array([[values['ixx'], values['ixy'], values['ixz']],
                       [values['ixy'], values['iyy'], values['iyz']],
                       [values['ixz'], values['iyz'], values['izz']]])
    if not math.isfinite(mass) or mass <= 0 or not np.all(np.isfinite(tensor)) or np.min(np.linalg.eigvalsh(tensor)) <= 0:
        raise ValueError('Rigid camera merge requires positive mass and physical inertia')
    parent_T_inertial = parent_T_link @ _sdf_transform(inertial)
    rotation = parent_T_inertial[:3, :3]
    return mass, parent_T_inertial[:3, 3], rotation @ tensor @ rotation.T


def mount_camera_on_base(model, camera):
    """Replace the camera's ODE fixed joint with a rigid base-link sensor.

    Preserve the DECLARED payload mass/COM/inertia, not a fitted image or link
    pose. Sensor/visual frames stay geometrically identical to the ideal old
    rigid assembly. The body frame, original IMU and calibration are unchanged.
    """
    base = model.find("link[@name='base_link']")
    if base is None or base.find("sensor[@name='vio_camera']") is not None:
        raise ValueError('Expected one base_link without an existing VIO sensor')
    model_T_base = _sdf_transform(base)
    model_T_camera = _sdf_transform(camera)
    base_T_camera = np.linalg.inv(model_T_base) @ model_T_camera
    base_mass, base_com, base_inertia = _inertial_in_parent(base, np.eye(4))
    camera_mass, camera_com, camera_inertia = _inertial_in_parent(camera, base_T_camera)
    combined_mass = base_mass + camera_mass
    combined_com = (base_mass*base_com + camera_mass*camera_com) / combined_mass
    combined_inertia = base_inertia + camera_inertia
    for mass, center in ((base_mass, base_com), (camera_mass, camera_com)):
        displacement = center - combined_com
        combined_inertia += mass * (np.dot(displacement, displacement)*np.eye(3) - np.outer(displacement, displacement))
    inertial = base.find('inertial')
    inertial.find('mass').text = '%.17g' % combined_mass
    base_T_inertial = np.eye(4)
    base_T_inertial[:3, 3] = combined_com
    _write_sdf_transform(inertial, base_T_inertial)
    tensor = inertial.find('inertia')
    for name, row, column in [('ixx', 0, 0), ('iyy', 1, 1), ('izz', 2, 2), ('ixy', 0, 1), ('ixz', 0, 2), ('iyz', 1, 2)]:
        entry = tensor.find(name)
        if entry is None:
            entry = ET.SubElement(tensor, name)
        entry.text = '%.17g' % combined_inertia[row, column]
    # Move complete original XML children: camera rate, two local camera poses,
    # image formats, intrinsics and ROS plugin/topic fields are not rewritten.
    for child in list(camera):
        if child.tag in ('sensor', 'visual'):
            base_T_child = base_T_camera @ _sdf_transform(child)
            _write_sdf_transform(child, base_T_child)
            camera.remove(child)
            base.append(child)
    if camera in list(model):
        model.remove(camera)
    for joint in list(model.findall('joint')):
        if joint.findtext('child') == camera.get('name'):
            if joint.get('type') != 'fixed' or joint.findtext('parent') != 'base_link':
                raise ValueError('Unexpected camera attachment; refusing a non-rigid conversion')
            model.remove(joint)


def add_lidar(model):
    """Add a real Gazebo CPU ray sensor in the base-link FLU coordinate system.

    The scan plane clears Iris's highest native collider (base z=0.055 m).
    A visual-only housing below the scan plane cannot create self returns.
    """
    lidar = ET.fromstring('''<link name="uav_lidar">
      <pose>0 0 0.35 0 0 0</pose>
      <inertial><pose>0 0 -0.035 0 0 0</pose><mass>0.01</mass>
        <inertia><ixx>0.000001</ixx><iyy>0.000001</iyy><izz>0.000001</izz></inertia></inertial>
      <visual name="lidar_housing"><pose>0 0 -0.035 0 0 0</pose>
        <geometry><cylinder><radius>0.04</radius><length>0.025</length></cylinder></geometry>
        <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.08 0.08 0.08 1</diffuse></material></visual>
      <sensor name="uav_horizontal_lidar" type="ray">
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on><update_rate>10</update_rate><visualize>false</visualize>
        <ray><scan><horizontal><samples>360</samples><resolution>1</resolution>
          <min_angle>-3.141592653589793</min_angle><max_angle>3.141592653589793</max_angle></horizontal>
          <vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical></scan>
          <range><min>0.2</min><max>20</max><resolution>0.01</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
        </ray>
        <plugin name="uav_lidar_ros" filename="libgazebo_ros_laser.so">
          <robotNamespace>/</robotNamespace><topicName>/uav/lidar/scan</topicName><frameName>uav_lidar</frameName>
        </plugin>
      </sensor>
    </link>''')
    lidar.find('pose').text = '0 0 {} 0 0 0'.format(LIDAR_HEIGHT)
    lidar.find('sensor/update_rate').text = str(LIDAR_RATE)
    lidar.find('sensor/ray/scan/horizontal/samples').text = str(LIDAR_SAMPLES)
    model.append(lidar)
    model.append(ET.fromstring('''<joint name="uav_lidar_joint" type="fixed">
      <parent>base_link</parent><child>uav_lidar</child></joint>'''))


def generate(model_only=False):
    model_dir = ROOT / 'models' / 'iris_vio'
    texture_dir = ROOT / 'materials' / 'textures'
    script_dir = ROOT / 'materials' / 'scripts'
    for directory in (model_dir, texture_dir, script_dir, ROOT / 'config', ROOT / 'worlds'):
        directory.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(PX4 / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf')
    model = tree.getroot().find('model')
    camera = ET.fromstring('''
    <link name="vio_camera_link">
      <pose>0.12 0.06 0.18 0 0.35 0</pose>
      <inertial><mass>0.01</mass><inertia><ixx>0.000001</ixx><iyy>0.000001</iyy><izz>0.000001</izz></inertia></inertial>
      <visual name="camera_housing"><pose>0 -0.06 0 0 0 0</pose><geometry><box><size>0.03 0.16 0.03</size></box></geometry>
        <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>
      <sensor name="vio_camera" type="multicamera">
        <always_on>true</always_on><update_rate>20</update_rate><visualize>false</visualize>
        <camera name="left"><horizontal_fov>1.5707963267948966</horizontal_fov>
          <image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>50</far></clip>
        </camera>
        <camera name="right"><pose>0 -0.12 0 0 0 0</pose><horizontal_fov>1.5707963267948966</horizontal_fov>
          <image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>50</far></clip>
        </camera>
        <plugin name="vio_camera_ros" filename="libgazebo_ros_multicamera.so">
          <robotNamespace>/</robotNamespace><cameraName>uav/camera</cameraName>
          <imageTopicName>image_raw</imageTopicName><cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <frameName>uav_stereo_optical</frameName><updateRate>0</updateRate><hackBaseline>0.12</hackBaseline>
          <Cx>320</Cx><Cy>240</Cy><focalLength>320</focalLength>
          <distortionK1>0</distortionK1><distortionK2>0</distortionK2><distortionK3>0</distortionK3>
          <distortionT1>0</distortionT1><distortionT2>0</distortionT2>
        </plugin>
      </sensor>
    </link>''')
    camera.find('pose').text = '0.12 0.06 0.18 0 {} 0'.format(PITCH)
    mount_camera_on_base(model, camera)
    model.append(ET.fromstring('''<plugin name="uav_original_body_imu_ros"
        filename="libuav_body_imu_plugin.so"/>'''))
    add_lidar(model)
    tree.write(str(model_dir / 'iris_vio.sdf'), encoding='utf-8', xml_declaration=True)
    if model_only:
        print('Generated stereo/body-IMU/LiDAR Iris model only in', model_dir)
        return

    rng = np.random.RandomState(20260930)
    texture = np.full((2048, 2048, 3), 190, dtype=np.uint8)
    for _ in range(6000):
        x, y = rng.randint(0, 2048, 2)
        radius = int(rng.randint(3, 14))
        color = tuple(int(v) for v in rng.randint(10, 240, 3))
        if rng.rand() < 0.5:
            cv2.rectangle(texture, (x, y), (x + radius * 2, y + radius), color, -1)
        else:
            cv2.circle(texture, (x, y), radius, color, -1)
    cv2.imwrite(str(texture_dir / 'features.png'), texture)
    wall_texture = np.full((512, 512, 3), 220, dtype=np.uint8)
    for _ in range(450):
        x, y = rng.randint(0, 512, 2)
        width, height = rng.randint(4, 16, 2)
        color = tuple(int(v) for v in rng.randint(10, 180, 3))
        cv2.rectangle(wall_texture, (x, y), (x + width, y + height), color, -1)
    cv2.imwrite(str(texture_dir / 'wall_features.png'), wall_texture)
    (script_dir / 'vio.material').write_text('''material VIO/Features
{
  technique
  {
    pass
    {
      ambient 0.8 0.8 0.8 1
      diffuse 0.8 0.8 0.8 1
      specular 0 0 0 1
      texture_unit
      {
        texture features.png
      }
    }
  }
}
''' + '''material VIO/Wall
{
  technique
  {
    pass
    {
      ambient 0.9 0.9 0.9 1
      diffuse 0.9 0.9 0.9 1
      texture_unit
      {
        texture wall_features.png
      }
    }
  }
}
''')
    material = '''<material><script><uri>file://{}</uri><uri>file://{}</uri>
        <name>VIO/Features</name></script></material>'''.format(script_dir, texture_dir)
    world = ET.parse(PX4 / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world')
    world_node = world.getroot().find('world')
    for include in list(world_node.findall('include')):
        if include.findtext('uri') == 'model://asphalt_plane':
            world_node.remove(include)
    world_node.append(ET.fromstring('''<model name="vio_feature_floor"><static>true</static>
      <pose>0 0 0.003 0 0 0</pose><link name="surface">
      <visual name="features"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
      %s</visual></link></model>''' % material))
    # Different depths prevent the visual scene from being entirely planar.
    for index, (x, y, height) in enumerate(((8, -3, 3), (8, 0, 4), (8, 3, 2), (5, -5, 2), (5, 5, 3))):
        world_node.append(ET.fromstring('''<model name="vio_landmark_%d"><static>true</static>
          <pose>%s %s %s 0 0 0</pose><link name="box">
          <collision name="collision"><geometry><box><size>0.8 0.8 %s</size></box></geometry></collision>
          <visual name="texture"><geometry><box><size>0.8 0.8 %s</size></box></geometry>%s</visual>
          </link></model>''' % (index, x, y, height / 2, height, height, material)))
    world_node.append(ET.fromstring('''<model name="vio_feature_wall"><static>true</static>
      <pose>5 0 2.5 0 0 0</pose><link name="wall">
      <collision name="collision"><geometry><box><size>0.1 12 5</size></box></geometry></collision>
      <visual name="features"><geometry><box><size>0.1 12 5</size></box></geometry>
      %s</visual></link></model>''' % material.replace('VIO/Features', 'VIO/Wall')))
    world.write(str(ROOT / 'worlds' / 'vio.world'), encoding='utf-8', xml_declaration=True)

    calibration = '''%YAML:1.0
model_type: PINHOLE
camera_name: uav_camera
image_width: 640
image_height: 480
distortion_parameters:
   k1: 0.0
   k2: 0.0
   p1: 0.0
   p2: 0.0
projection_parameters:
   fx: 320.0
   fy: 320.0
   cx: 320.0
   cy: 240.0
'''
    (ROOT / 'config' / 'camera.yaml').write_text(calibration)
    s, c = math.sin(PITCH), math.cos(PITCH)
    # body_T_cam maps optical x-right/y-down/z-forward to the original IMU's FLU axes.
    matrix = [0, -s, c, .12, -1, 0, 0, .06, 0, -c, -s, .16, 0, 0, 0, 1]
    right_matrix = list(matrix)
    right_matrix[7] = -.06
    native_log_dir = Path(os.environ.get('UAV_VIO_LOG_DIR') or str(ROOT.parent / 'logs' / 'vio')).expanduser().resolve()
    visual_diagnostics = os.environ.get('PX4_AGENT_VINS_VISUAL_DIAGNOSTICS', '0').strip()
    if visual_diagnostics not in ('0', '1'):
        raise ValueError('PX4_AGENT_VINS_VISUAL_DIAGNOSTICS must be 0 or 1')
    diagnostic_env = os.environ.get('PX4_AGENT_VINS_BIAS_DIAGNOSTICS_DIR', '').strip()
    diagnostic_dir = str(Path(diagnostic_env).expanduser().resolve()) if diagnostic_env else ''
    if diagnostic_dir:
        Path(diagnostic_dir).mkdir(parents=True, exist_ok=True)
    native_log_dir.mkdir(parents=True, exist_ok=True)
    (native_log_dir / 'pose_graph').mkdir(parents=True, exist_ok=True)
    config = '''%%YAML:1.0
imu: 1
num_of_cam: 2
imu_topic: "/uav/imu/data_raw"
image0_topic: "/uav/camera/left/image_raw"
image1_topic: "/uav/camera/right/image_raw"
output_path: %s
cam0_calib: "camera.yaml"
cam1_calib: "camera.yaml"
image_width: 640
image_height: 480
estimate_extrinsic: 0
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [%s]
body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [%s]
multiple_thread: 1
max_cnt: 180
min_dist: 20
freq: 20
F_threshold: 1.0
show_track: 1
flow_back: 1
# SITL budget experiment: at simulation speed .35, 10 sim Hz optimization
# allows .286 wall seconds per image. OLD uses 80%% of this wall-time budget.
# Measurement stamps and all source-freshness/quality gates stay unchanged.
max_solver_time: 0.12
max_num_iterations: 12
# Empty by default. Only the explicit task environment opts into matrix dumps.
# Experimental exact derivative of existing frozen prior residual (no new constraint).
exact_marginalization_pose_jacobian: 1
# Coordinate gauge only: first XYZ/yaw fixed, roll/pitch remain sensor-estimated.
# Experimental V8 coordinate gauge: free RP with bounded smooth retraction.
fix_first_pose_xyz_yaw: 1
bias_information_diagnostics_dir: %s
bias_information_period_s: 1.0
bias_information_startup_dense_s: 3.0
keyframe_parallax: 10.0
# VINS native discrete covariance convention at original IMU dt=0.004 s:
# N = Gazebo density / sqrt(dt) per sample; W = OU sigma_step / dt.
# Native preintegration omits OU bias leak and shared-midpoint cross-step
# correlation; these values match nominal one-step variance, not a full model.
acc_n: 0.029409182239566
gyr_n: 0.0029511956263521
acc_w: 0.094867697353033
gyr_w: 0.00061324346876081
acc_bias_prior_sigma: 0.196
gyr_bias_prior_sigma: 0.0087
# Experimental simulation ZUPT: requires original IMU and near tracked stereo
# support; never uses Gazebo truth, GNSS, PX4 landed flags or frozen odometry.
enable_zupt: 1
# V9 SITL startup: wait for every real window frame/interval stationary.
# Seeds source IMU means once as an initial guess; adds no new measurement.
stationary_initialization: 1
# Default OFF: bounded feature lifecycle snapshots, no change to physical rules.
visual_lifecycle_diagnostics: %s
# Retain real IMU/bias transmission during prolonged low-parallax stillness.
# Upstream default 0 disables this optional policy; SITL uses 1 actual second.
max_imu_preintegration_span_s: 1.0
zupt_velocity_sigma: 0.03
zupt_position_sigma: 0.02
zupt_camera_fx: 320.0
# Physical IMU->body center lever arm, used only by refresh health checking.
health_imu_to_body_flu: [0.0, 0.0, -0.02]
g_norm: 9.8066
estimate_td: 0
td: 0.0
load_previous_pose_graph: 0
pose_graph_save_path: %s
save_image: 0
''' % (json.dumps(str(native_log_dir)), ', '.join('%.12g' % v for v in matrix), ', '.join('%.12g' % v for v in right_matrix), json.dumps(diagnostic_dir), visual_diagnostics, json.dumps(str(native_log_dir / 'pose_graph')))
    (ROOT / 'config' / 'stereo_imu.yaml').write_text(config)
    (ROOT.parent / 'logs' / 'vio').mkdir(parents=True, exist_ok=True)
    print('Generated camera model, feature world and VINS-Fusion configuration in', ROOT)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-only', action='store_true', help='Update only Iris SDF, leaving world, textures and VINS config untouched')
    generate(model_only=parser.parse_args().model_only)
