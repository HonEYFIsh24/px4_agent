"""Targeted actual-native diagnostic math and array ownership regression."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'vio_ws/src/VINS-Fusion'
LIB=ROOT/'vio_ws/devel/lib'

class BiasDiagnosticsTests(unittest.TestCase):
    def test_actual_native_imu_pose_projection_and_asan(self):
        if not shutil.which('g++') or not (LIB/'libvins_lib.so').exists():
            self.skipTest('Actual built VINS native library unavailable')
        with tempfile.TemporaryDirectory(prefix='vins_bias_diagnostics_') as tmp:
            out=Path(tmp)
            common=['g++','-std=c++11','-O0','-g0','-I'+str(SOURCE/'vins_estimator/src/factor'),
                    '-I/usr/include/eigen3','-I/usr/include/opencv4','-I/opt/ros/noetic/include']
            link=['-L'+str(LIB),'-Wl,-rpath,'+str(LIB),'-L/opt/ros/noetic/lib',
                  '-Wl,-rpath,/opt/ros/noetic/lib','-lvins_lib','-lceres','-lglog','-lgflags','-lrosconsole','-lroscpp',
                  '-lspqr','-lcholmod','-lccolamd','-lcamd','-lcolamd','-lamd',
                  '-llapack','-lblas','-lsuitesparseconfig','-lrt','-lmetis',
                  '-lcxsparse','-lgomp','-ltbb','-ltbbmalloc','-lpthread']
            executable=out/'projection'
            result=subprocess.run(common+[str(ROOT/'tests/vins_bias_diagnostics_test.cpp')]+link+['-o',str(executable)],
                                  capture_output=True,text=True,timeout=120)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            result=subprocess.run([str(executable),str(out/'snapshot.json')],capture_output=True,text=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('actual full IMU/pose nuisance',result.stdout)
            data=json.loads((out/'snapshot.json').read_text())
            self.assertEqual(len(data['jacobian']),len(data['residual']))
            self.assertTrue(data['diagnostic_only'])
            asan=out/'asan'
            result=subprocess.run(common+['-fsanitize=address','-fno-omit-frame-pointer',str(ROOT/'tests/vins_marginalization_asan_test.cpp')]+link+['-o',str(asan)],
                                  capture_output=True,text=True,timeout=60)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            env=dict(os.environ,ASAN_OPTIONS='detect_leaks=0:alloc_dealloc_mismatch=1')
            result=subprocess.run([str(asan)],env=env,capture_output=True,text=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('100 iterations',result.stdout)

if __name__=='__main__':unittest.main()
