"""Bounded native visual diagnostics: actual deletion and actual solve snapshots."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'vio_ws/src/VINS-Fusion'
LIB=ROOT/'vio_ws/devel/lib'


def compile_native(source, executable):
    includes=['-I'+str(SOURCE/'vins_estimator/src/estimator'),
              '-I'+str(SOURCE/'vins_estimator/src/factor'),
              '-I'+str(SOURCE/'camera_models/include'), '-I/usr/include/eigen3',
              '-I/usr/include/opencv4','-I/opt/ros/noetic/include']
    links=['-L'+str(LIB),'-Wl,-rpath,'+str(LIB),'-L/opt/ros/noetic/lib',
           '-Wl,-rpath,/opt/ros/noetic/lib','-lvins_lib','-lceres','-lglog','-lgflags',
           '-lrosconsole','-lroscpp','-lrostime','-lopencv_core','-lspqr','-lcholmod',
           '-lccolamd','-lcamd','-lcolamd','-lamd','-llapack','-lblas',
           '-lsuitesparseconfig','-lrt','-lmetis','-lcxsparse','-lgomp','-ltbb','-ltbbmalloc','-lpthread']
    return subprocess.run(['g++','-std=c++11','-O0','-g0']+includes+[str(source)]+links+
                          ['-o',str(executable)],capture_output=True,text=True,timeout=120)


class NativeVisualLifecycleTests(unittest.TestCase):
    def setUp(self):
        if not shutil.which('g++') or not (LIB/'libvins_lib.so').exists():
            self.skipTest('actual built native VINS library required')

    def test_actual_outlier_deletion_caps_and_disabled_physics(self):
        with tempfile.TemporaryDirectory(prefix='vins_visual_outliers_') as tmp:
            executable=Path(tmp)/'visual'
            compiled=compile_native(ROOT/'tests/vins_visual_lifecycle_test.cpp',executable)
            self.assertEqual(compiled.returncode,0,compiled.stdout+compiled.stderr)
            result=subprocess.run([str(executable)],capture_output=True,text=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('PASS: actual visual lifecycle',result.stdout)
            print(result.stdout,end='')

    def test_actual_first_solve_and_post_outlier_source_phases(self):
        with tempfile.TemporaryDirectory(prefix='vins_visual_solve_') as tmp:
            executable=Path(tmp)/'initialization'
            compiled=compile_native(ROOT/'tests/vins_stationary_initialization_test.cpp',executable)
            self.assertEqual(compiled.returncode,0,compiled.stdout+compiled.stderr)
            result=subprocess.run([str(executable),str(ROOT/'vio/config/camera.yaml'),'--visual-diagnostics'],
                                  capture_output=True,text=True,timeout=90)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            evidence=Path(re.search(r'evidence=([^\s]+)',result.stdout).group(1))
            rows=[json.loads(line) for line in (evidence/'native_health.jsonl').read_text().splitlines()]
            waiting=[r for r in rows if r['initialization']['seed_count']==0]
            self.assertTrue(waiting)
            self.assertTrue(all(not r['visual_lifecycle_diagnostics']['sampled'] for r in waiting))
            solved=[r for r in rows if r['visual_lifecycle_diagnostics']['sampled']]
            self.assertGreaterEqual(len(solved),6)
            first=solved[0]['visual_lifecycle_diagnostics']
            self.assertEqual(first['outcome'],'initial_solve_without_outlier_stage')
            self.assertFalse(first['outliers_evaluated'])
            self.assertEqual(first['solve_sequence'],1)
            for row in solved:
                diag=row['visual_lifecycle_diagnostics']
                self.assertEqual(diag['source_stamp_sim_s'],row['stamp'])
                headers=diag['window_headers_sim_s']
                self.assertEqual(len(headers),11)
                self.assertTrue(all(b>a for a,b in zip(headers,headers[1:])))
                self.assertEqual(headers[-1],diag['source_stamp_sim_s'])
                self.assertEqual(len(diag['original_imu_spans_s']),10)
                self.assertTrue(all(abs(s-(b-a))<1e-8 for s,a,b in zip(diag['original_imu_spans_s'],headers,headers[1:])))
                for key,phase in [('before_solve','before_solve'),('post_solve_pre_outlier','post_solve_pre_outlier')]:
                    snapshot=diag[key]
                    self.assertEqual(snapshot['phase'],phase)
                    self.assertEqual(snapshot['total_features'],len(snapshot['features']))
                    self.assertEqual(snapshot['eligible_features'],sum(f['observation_count']>=4 for f in snapshot['features']))
                if diag['outliers_evaluated']:
                    self.assertEqual(diag['post_outlier_before_slide']['phase'],'post_outlier_before_slide')
                    self.assertEqual(diag['post_solve_pre_outlier']['total_features']-diag['post_outlier_before_slide']['total_features'],diag['removed_ids_total'])
                    self.assertEqual(diag['evaluated_feature_count'],diag['post_solve_pre_outlier']['eligible_features'])
                    self.assertTrue(all(r['error_terms']>0 for r in diag['reprojections']))
                    self.assertTrue(all(abs(r['average_normalized_error']*460-r['average_virtual_px_error'])<1e-8 for r in diag['reprojections']))
            print('Actual diagnostic solve evidence:',evidence)
            print('actual solves:',len(solved),'source window:',solved[0]['stamp'],solved[-1]['stamp'])

    def test_generator_opt_in_changes_only_diagnostic_flag(self):
        spec=importlib.util.spec_from_file_location('visual_assets',ROOT/'vio/generate_assets.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory(prefix='vins_visual_config_') as tmp:
            module.ROOT=Path(tmp)/'vio'
            env={'UAV_VIO_LOG_DIR':str(Path(tmp)/'native'),
                 'PX4_AGENT_VINS_BIAS_DIAGNOSTICS_DIR':''}
            with patch.dict(os.environ,env):
                with patch.dict(os.environ,{'PX4_AGENT_VINS_VISUAL_DIAGNOSTICS':'0'}): module.generate()
                normal=(module.ROOT/'config/stereo_imu.yaml').read_text()
                with patch.dict(os.environ,{'PX4_AGENT_VINS_VISUAL_DIAGNOSTICS':'1'}): module.generate()
                enabled=(module.ROOT/'config/stereo_imu.yaml').read_text()
            self.assertIn('visual_lifecycle_diagnostics: 0',normal)
            self.assertEqual(normal.replace('visual_lifecycle_diagnostics: 0','visual_lifecycle_diagnostics: 1'),enabled)
        parameters=(SOURCE/'vins_estimator/src/estimator/parameters.cpp').read_text()
        self.assertIn('VISUAL_LIFECYCLE_DIAGNOSTICS = 0;',parameters)
        self.assertEqual((ROOT/'vio/patches/visual_lifecycle_diagnostics.h').read_bytes(),
                         (SOURCE/'vins_estimator/src/factor/visual_lifecycle_diagnostics.h').read_bytes())
        # Config physically identical to the preserved V9 apart from the new
        # default-disabled flag, comments and owned output paths.
        archived=ROOT/'logs/ev/native_v9_visual_diagnostics_design_01/original/vio/config/stereo_imu.yaml'
        def physical(text):
            return '\n'.join(line for line in text.splitlines() if line.strip() and not line.lstrip().startswith('#')
                             and not line.startswith(('output_path:','pose_graph_save_path:','visual_lifecycle_diagnostics:')))
        if archived.exists(): self.assertEqual(physical(archived.read_text()),physical((ROOT/'vio/config/stereo_imu.yaml').read_text()))

if __name__=='__main__': unittest.main()
