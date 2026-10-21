"""Actual captured-prior derivative, Ceres prior ownership and free-pose tests."""
import gzip
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'vio_ws/src/VINS-Fusion'
LIB=ROOT/'vio_ws/devel/lib'

class ExactPriorTests(unittest.TestCase):
    def compile_native(self,source,executable):
        common=['g++','-std=c++11','-O0','-g0','-I'+str(SOURCE/'vins_estimator/src/factor'),
                '-I/usr/include/eigen3','-I/usr/include/opencv4','-I/opt/ros/noetic/include']
        link=['-L'+str(LIB),'-Wl,-rpath,'+str(LIB),'-L/opt/ros/noetic/lib','-Wl,-rpath,/opt/ros/noetic/lib',
              '-lvins_lib','-lceres','-lglog','-lgflags','-lrosconsole','-lroscpp','-lspqr','-lcholmod',
              '-lccolamd','-lcamd','-lcolamd','-lamd','-llapack','-lblas','-lsuitesparseconfig','-lrt',
              '-lmetis','-lcxsparse','-lgomp','-ltbb','-ltbbmalloc','-lpthread']
        result=subprocess.run(common+[str(source)]+link+['-o',str(executable)],
                              capture_output=True,text=True,timeout=120)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_captured_native_prior_residual_and_cost_finite_difference(self):
        if not shutil.which('g++') or not (LIB/'libvins_lib.so').exists():
            self.skipTest('Actual built native VINS required')
        fixtures=json.loads(gzip.decompress((ROOT/'tests/data/vins_prior_rotation_captured.json.gz').read_bytes()))['fixtures']
        with tempfile.TemporaryDirectory(prefix='vins_exact_prior_') as tmp:
            directory=Path(tmp);files=[]
            for i,data in enumerate(fixtures):
                path=directory/('fixture_%d.txt'%i);files.append(str(path))
                with path.open('w') as stream:
                    stream.write('%d %d %d\n'%(len(data['jacobian']),len(data['jacobian'][0]),len(data['parameter_blocks'])))
                    for block in data['parameter_blocks']:
                        stream.write('%d %d %d %s\n'%(block['global_size'],block['column'],block['constant_in_actual_problem'],block['label']))
                        stream.write(' '.join(map(repr,block['linearization_global_values']))+'\n')
                        stream.write(' '.join(map(repr,block['global_values']))+'\n')
                    for row in data['jacobian']:stream.write(' '.join(map(repr,row))+'\n')
                    stream.write(' '.join(map(repr,data['stored_linearized_residual']))+'\n')
                    stream.write(' '.join(map(repr,data['expected_current_residual']))+'\n')
            executable=directory/'native_prior'
            self.compile_native(ROOT/'tests/vins_prior_pose_jacobian_test.cpp',executable)
            result=subprocess.run([str(executable)]+files,capture_output=True,text=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertEqual(result.stdout.count('PASS:'),2,result.stdout)
            self.assertIn('nondifferentiable',result.stdout)
            print(result.stdout,end='')
    def test_actual_full_graph_wrong_tilt_free_rp_and_relative_heading(self):
        if not shutil.which('g++') or not (LIB/'libvins_lib.so').exists():
            self.skipTest('Actual built native VINS required')
        with tempfile.TemporaryDirectory(prefix='vins_exact_prior_fullgraph_') as tmp:
            executable=Path(tmp)/'full_graph'
            self.compile_native(ROOT/'tests/vins_exact_prior_full_graph_test.cpp',executable)
            result=subprocess.run([str(executable)],capture_output=True,text=True,timeout=90)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('once-prior through ten OLD',result.stdout)
            self.assertIn('first RP2 and subsequent6 free poses',result.stdout)
            print(result.stdout,end='')
    def test_fixed_gauge_actual_plus_basis_and_diagnostic_mapping(self):
        if not shutil.which('g++') or not (LIB/'libvins_lib.so').exists():
            self.skipTest('Actual built native VINS required')
        with tempfile.TemporaryDirectory(prefix='vins_fixed_gauge_') as tmp:
            executable=Path(tmp)/'fixed_gauge';snapshot=Path(tmp)/'snapshot.json'
            self.compile_native(ROOT/'tests/vins_fixed_yaw_parameterization_test.cpp',executable)
            result=subprocess.run([str(executable),str(snapshot)],capture_output=True,text=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            data=json.loads(snapshot.read_text());self.assertEqual(data['schema_version'],3)
            block=data['parameter_blocks'][0]
            self.assertEqual(block['local_size'],6);self.assertEqual(block['actual_optimizer_local_size'],2)
            expected=[[0,0],[0,0],[0,0],[.001,0],[0,.001],[0,0]]
            self.assertEqual(block['optimizer_to_matrix_local'],expected)
            self.assertEqual(block['physical_pose_tangent_from_optimizer'],expected)
            print(result.stdout,end='')
    def test_optin_is_immutable_and_only_sitl_config_enabled(self):
        header=(SOURCE/'vins_estimator/src/factor/marginalization_factor.h').read_text()
        self.assertIn('const bool exact_pose_jacobian;',header)
        self.assertIn('bool exact_pose_derivative = false',header)
        parameters=(SOURCE/'vins_estimator/src/estimator/parameters.cpp').read_text()
        self.assertIn('EXACT_MARGINALIZATION_POSE_JACOBIAN = 0;',parameters)
        for path in [ROOT/'vio/config/stereo_imu.yaml',ROOT/'vio/generate_assets.py']:
            self.assertIn('exact_marginalization_pose_jacobian: 1',path.read_text())
        self.assertIn('FIX_FIRST_POSE_XYZ_YAW = 0;',parameters)
        for path in [ROOT/'vio/config/stereo_imu.yaml',ROOT/'vio/generate_assets.py']:
            self.assertIn('fix_first_pose_xyz_yaw: 1',path.read_text())
        for name in ['bias_native_diagnostics.h','bias_snapshot.h','fixed_yaw_pose_parameterization.h']:
            self.assertEqual((ROOT/'vio/patches'/name).read_bytes(),(SOURCE/'vins_estimator/src/factor'/name).read_bytes())
if __name__=='__main__':unittest.main()
