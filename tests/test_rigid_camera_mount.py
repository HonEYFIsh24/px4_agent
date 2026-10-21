"""Rigid sensor topology, preserved payload mechanics and unchanged calibration."""
import copy
import importlib.util
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
ORIGINAL=ROOT/'logs/ev/rigid_camera_mount_design_01/original/vio/models/iris_vio/iris_vio.sdf'


def generator():
    spec=importlib.util.spec_from_file_location('rigid_assets',ROOT/'vio/generate_assets.py')
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value


def inertia(element):
    tensor=element.find('inertial/inertia')
    xx,yy,zz,xy,xz,yz=[float(tensor.findtext(key,'0'))for key in ['ixx','iyy','izz','ixy','ixz','iyz']]
    return np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])


def pose(element):
    # Independent homogeneous transform in the SDF enclosing frame.
    vals=[float(v)for v in element.findtext('pose','0 0 0 0 0 0').split()]
    x,y,z,r,p,yaw=vals
    rx=np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    ry=np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    rz=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    t=np.eye(4);t[:3,:3]=rz.dot(ry).dot(rx);t[:3,3]=[x,y,z];return t


class RigidCameraMountTests(unittest.TestCase):
    def old_model(self):
        if not ORIGINAL.exists():self.skipTest('preserved original camera SDF required')
        return ET.parse(ORIGINAL).getroot().find('model')

    def test_actual_mass_com_tensor_and_other_physics_preserved(self):
        original=self.old_model();model=copy.deepcopy(original)
        camera=model.find("link[@name='vio_camera_link']")
        generator().mount_camera_on_base(model,camera)
        base=model.find("link[@name='base_link']")
        self.assertIsNone(model.find("link[@name='vio_camera_link']"))
        self.assertIsNone(model.find("joint[@name='vio_camera_joint']"))
        self.assertAlmostEqual(float(base.findtext('inertial/mass')),1.51,places=14)
        # The payload's old COM is at the LEFT mount [.12,.06,.18], not
        # at housing center [.12,0,.18]. Preserve the declared 10 grams.
        center=pose(base.find('inertial'))[:3,3]
        np.testing.assert_allclose(center,np.array([.12,.06,.18])*.01/1.51,rtol=0,atol=1e-15)
        old_tensor=np.diag([.029125,.029125,.055225]);point=np.array([.12,.06,.18])
        expected=old_tensor+np.eye(3)*1e-6+(1.5*.01/1.51)*(np.dot(point,point)*np.eye(3)-np.outer(point,point))
        np.testing.assert_allclose(inertia(base),expected,rtol=0,atol=1e-15)
        np.testing.assert_allclose(pose(base.find('inertial'))[:3,:3],np.eye(3),atol=1e-15)
        self.assertTrue(np.all(np.linalg.eigvalsh(inertia(base))>0))
        old_mass=sum(float(p.findtext('inertial/mass','0'))for p in original.findall('link'))
        new_mass=sum(float(p.findtext('inertial/mass','0'))for p in model.findall('link'))
        self.assertAlmostEqual(old_mass,new_mass,places=14)
        for old_child in original:
            if old_child is original.find("link[@name='base_link']") or old_child.tag=='link' and old_child.get('name')=='vio_camera_link' or old_child.tag=='joint' and old_child.get('name')=='vio_camera_joint':continue
            matches=[new for new in model if new.tag==old_child.tag and new.get('name')==old_child.get('name')]
            self.assertEqual(len(matches),1)
            self.assertEqual(ET.tostring(old_child),ET.tostring(matches[0]))
        # The geometric base origin/collision stay fixed when its COM moves.
        original_base=original.find("link[@name='base_link']")
        self.assertEqual(ET.tostring(base.find('collision')),ET.tostring(original_base.find('collision')))
        np.testing.assert_allclose(pose(base),pose(original_base),atol=0)

    def test_camera_world_geometry_topics_rate_and_calibration_unchanged(self):
        original=self.old_model();model=copy.deepcopy(original)
        old_camera=original.find("link[@name='vio_camera_link']")
        old_sensor=old_camera.find('sensor');old_visual=old_camera.find('visual')
        generator().mount_camera_on_base(model,model.find("link[@name='vio_camera_link']"))
        base=model.find("link[@name='base_link']");sensor=base.find("sensor[@name='vio_camera']")
        visual=base.find("visual[@name='camera_housing']")
        self.assertIsNotNone(sensor)
        expected=pose(old_camera).dot(pose(old_sensor))
        np.testing.assert_allclose(pose(base).dot(pose(sensor)),expected,rtol=0,atol=1e-15)
        np.testing.assert_allclose(pose(base).dot(pose(visual)),pose(old_camera).dot(pose(old_visual)),rtol=0,atol=1e-15)
        for name in ['left','right']:
            old_eye=old_sensor.find("camera[@name='%s']"%name);new_eye=sensor.find("camera[@name='%s']"%name)
            self.assertEqual(ET.tostring(old_eye),ET.tostring(new_eye))
            np.testing.assert_allclose(pose(base).dot(pose(sensor)).dot(pose(new_eye)),pose(old_camera).dot(pose(old_sensor)).dot(pose(old_eye)),atol=1e-15)
        for tag in ['update_rate','always_on','visualize','plugin']:
            self.assertEqual(ET.tostring(sensor.find(tag)),ET.tostring(old_sensor.find(tag)))
        # Independently derive optical->IMU FLU from unchanged sensor geometry.
        optical=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        imu=original.find("link[@name='/imu_link']")
        imu_T_base=np.linalg.inv(pose(imu))
        import cv2
        fs=cv2.FileStorage(str(ROOT/'vio/config/stereo_imu.yaml'),cv2.FILE_STORAGE_READ)
        try:
            for index,name in enumerate(['left','right']):
                eye=sensor.find("camera[@name='%s']"%name)
                expected_optical=imu_T_base.dot(pose(base)).dot(pose(sensor)).dot(pose(eye))
                expected_optical[:3,:3]=expected_optical[:3,:3].dot(optical)
                np.testing.assert_allclose(expected_optical,fs.getNode('body_T_cam%d'%index).mat(),rtol=0,atol=1e-12)
        finally:fs.release()
        left=pose(sensor).dot(pose(sensor.find("camera[@name='left']")))[:3,3]
        right=pose(sensor).dot(pose(sensor.find("camera[@name='right']")))[:3,3]
        self.assertAlmostEqual(np.linalg.norm(left-right),.12,places=14)

    def test_rotated_inertial_frames_and_parallel_axis(self):
        # Hand-solvable nonisotropic/rotated-frame fixture, independent of
        # the actual isotropic camera case which cannot exercise rotations.
        model=ET.fromstring('''<model name="fixture"><link name="base_link"><inertial>
          <pose>.2 -.3 .4 0 0 1.5707963267948966</pose><mass>2</mass><inertia>
          <ixx>.2</ixx><iyy>.3</iyy><izz>.4</izz></inertia></inertial></link>
          <link name="vio_camera_link"><pose>1 0 0 0 0 1.5707963267948966</pose><inertial>
          <pose>.1 .2 .3 0 1.5707963267948966 0</pose><mass>.5</mass><inertia>
          <ixx>.004</ixx><iyy>.005</iyy><izz>.006</izz></inertia></inertial>
          <sensor name="vio_camera" type="multicamera"/></link>
          <joint name="vio_camera_joint" type="fixed"><parent>base_link</parent><child>vio_camera_link</child></joint></model>''')
        generator().mount_camera_on_base(model,model.find("link[@name='vio_camera_link']"))
        base=model.find("link[@name='base_link']")
        np.testing.assert_allclose(pose(base.find('inertial'))[:3,3],[.32,-.22,.38],rtol=0,atol=1e-15)
        expected=np.array([[.373,-.096,.024],[-.096,.354,.016],[.024,.016,.612]])
        np.testing.assert_allclose(inertia(base),expected,rtol=0,atol=1e-15)

    def test_generated_model_parser_and_default_diagnostics_off(self):
        module=generator()
        with tempfile.TemporaryDirectory(prefix='rigid_camera_assets_')as tmp:
            module.ROOT=Path(tmp)/'vio'
            with patch.dict(os.environ,{'UAV_VIO_LOG_DIR':str(Path(tmp)/'native'),'PX4_AGENT_VINS_BIAS_DIAGNOSTICS_DIR':''}):
                with patch.dict(os.environ):
                    os.environ.pop('PX4_AGENT_VINS_VISUAL_DIAGNOSTICS',None)
                    module.generate()
            model_path=module.ROOT/'models/iris_vio/iris_vio.sdf'
            model=ET.parse(model_path).getroot().find('model')
            self.assertIsNotNone(model.find("link[@name='base_link']/sensor[@name='vio_camera']"))
            self.assertIsNone(model.find("link[@name='vio_camera_link']"))
            self.assertIn('visual_lifecycle_diagnostics: 0',(module.ROOT/'config/stereo_imu.yaml').read_text())
            if shutil.which('gz'):
                environment=os.environ.copy()
                # The unchanged Iris mesh URIs must resolve to the ACTUAL local
                # PX4 model tree; do not wait on Gazebo's remote model database.
                environment['GAZEBO_MODEL_DATABASE_URI']=''
                environment['GAZEBO_MODEL_PATH']=str(module.PX4/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models')
                checked=subprocess.run(['gz','sdf','-k',str(model_path)],capture_output=True,text=True,timeout=30,env=environment)
                self.assertEqual(checked.returncode,0,checked.stdout+checked.stderr)
                print('Actual generated Gazebo SDF validation:',checked.stdout.strip())
            print('Generated actual base mass:',model.findtext("link[@name='base_link']/inertial/mass"))

if __name__=='__main__':unittest.main()
