"""Actual native Estimator startup regression; no ROS master/flight/GT input."""
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'vio_ws/src/VINS-Fusion'
LIB = ROOT / 'vio_ws/devel/lib'

class StationaryInitializationTests(unittest.TestCase):
    def test_actual_delayed_stereo_imu_initialization(self):
        if not shutil.which('g++') or not (LIB/'libvins_lib.so').exists():
            self.skipTest('actual built native VINS library required')
        with tempfile.TemporaryDirectory(prefix='vins_init_compile_') as tmp:
            executable=Path(tmp)/'initialization'
            includes=['-I'+str(SOURCE/'vins_estimator/src/estimator'),
                      '-I'+str(SOURCE/'vins_estimator/src/factor'),
                      '-I'+str(SOURCE/'camera_models/include'),'-I/usr/include/eigen3',
                      '-I/usr/include/opencv4','-I/opt/ros/noetic/include']
            links=['-L'+str(LIB),'-Wl,-rpath,'+str(LIB),'-L/opt/ros/noetic/lib',
                   '-Wl,-rpath,/opt/ros/noetic/lib','-lvins_lib','-lceres',
                   '-lglog','-lgflags','-lrosconsole','-lroscpp','-lrostime',
                   '-lopencv_core','-lspqr','-lcholmod','-lccolamd','-lcamd',
                   '-lcolamd','-lamd','-llapack','-lblas','-lsuitesparseconfig',
                   '-lrt','-lmetis','-lcxsparse','-lgomp','-ltbb','-ltbbmalloc','-lpthread']
            compiled=subprocess.run(['g++','-std=c++11','-O0','-g0']+includes+
                [str(ROOT/'tests/vins_stationary_initialization_test.cpp')]+links+['-o',str(executable)],
                capture_output=True,text=True,timeout=120)
            self.assertEqual(compiled.returncode,0,compiled.stdout+compiled.stderr)
            result=subprocess.run([str(executable),str(ROOT/'vio/config/camera.yaml')],
                                  capture_output=True,text=True,timeout=90)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('PASS: actual Estimator delayed INIT',result.stdout)
            match=re.search(r'evidence=([^\s]+)',result.stdout)
            self.assertIsNotNone(match,result.stdout)
            evidence=Path(match.group(1))
            data=json.loads((evidence/'summary.json').read_text())
            self.assertTrue(data['passed']);self.assertEqual(data['seed_count'],1)
            self.assertEqual(data['initial_prior_insertions'],1)
            self.assertGreaterEqual(data['moving_negative_duration_sim_s'],6)
            self.assertTrue(data['first_pose_roll_pitch_free'])
            self.assertFalse(data['actual_flight']);self.assertFalse(data['ground_truth_feedback'])
            rows=[json.loads(line) for line in (evidence/'actual_estimator_health.jsonl').read_text().splitlines()]
            first_ready=next(i for i,row in enumerate(rows) if row['tracking_ok'])
            self.assertTrue(all(not row['tracking_ok'] and row['initialization']['seed_count']==0
                                for row in rows[:first_ready]))
            initialization=rows[first_ready]['initialization']
            self.assertTrue(all(initialization['stationary_frames']))
            self.assertTrue(all(initialization['stationary_intervals'][1:]))
            self.assertTrue(initialization['current_source_gate'])
            self.assertTrue(initialization['current_stationary_confirmed'])
            self.assertEqual(initialization['seed_count'],1)
            self.assertFalse(initialization['seed_prior_was_present'])
            self.assertTrue(initialization['existing_valid_marginal_prior'])
            self.assertGreaterEqual(initialization['seed_imu_samples'],150)
            # The production opt-in writer reuses the existing JSON; it must be
            # complete from partial initialization, not only after ready.
            trace=evidence/'native_health.jsonl'
            self.assertTrue(trace.exists())
            trace_rows=[json.loads(line) for line in trace.read_text().splitlines()]
            self.assertEqual(len(trace_rows),len(rows)+1)
            self.assertFalse(trace_rows[0]['tracking_ok'])
            seeds=[(i,row) for i,row in enumerate(trace_rows)
                   if row['initialization']['phase']=='seeded_before_first_solve']
            self.assertEqual(len(seeds),1)
            seed_index,seed_row=seeds[0]
            self.assertFalse(seed_row['tracking_ok'])
            self.assertEqual(seed_row['current_state_snapshot_phase'],'seed_before_first_solve_and_slide')
            self.assertFalse(seed_row['initialization']['existing_valid_marginal_prior'])
            self.assertEqual(seed_row['initialization']['initial_bias_prior_insertion_count'],0)
            completed=trace_rows[seed_index+1]
            self.assertEqual(seed_row['stamp'],completed['stamp'])
            self.assertEqual(completed['initialization']['phase'],'first_solve_complete')
            self.assertTrue(completed['tracking_ok'])
            self.assertTrue(completed['initialization']['existing_valid_marginal_prior'])
            self.assertEqual(completed['initialization']['initial_bias_prior_insertion_count'],1)
            print('Actual Estimator evidence:',evidence)
            print(json.dumps(data,sort_keys=True))

    def test_actual_mapping_and_motion_history_rejections(self):
        if not shutil.which('g++'): self.skipTest('g++ required')
        with tempfile.TemporaryDirectory(prefix='vins_init_mapping_') as tmp:
            executable=Path(tmp)/'mapping'
            compiled=subprocess.run(['g++','-std=c++11','-I'+str(ROOT/'vio/patches'),
                str(ROOT/'tests/vins_stationary_initialization_mapping_test.cpp'),'-o',str(executable)],
                capture_output=True,text=True,timeout=30)
            self.assertEqual(compiled.returncode,0,compiled.stdout+compiled.stderr)
            result=subprocess.run([str(executable)],capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            print(result.stdout,end='')

    def test_default_disabled_and_reproducible_helper(self):
        parameters=(SOURCE/'vins_estimator/src/estimator/parameters.cpp').read_text()
        self.assertIn('STATIONARY_INITIALIZATION = 0;',parameters)
        for path in [ROOT/'vio/config/stereo_imu.yaml',ROOT/'vio/generate_assets.py']:
            self.assertIn('stationary_initialization: 1',path.read_text())
        self.assertEqual((ROOT/'vio/patches/stationary_initialization.h').read_bytes(),
                         (SOURCE/'vins_estimator/src/factor/stationary_initialization.h').read_bytes())

if __name__ == '__main__': unittest.main()
