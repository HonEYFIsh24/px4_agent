"""Exercise native sensor gates and residuals with the actual Ceres library."""
import importlib.util
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'vio_ws/src/VINS-Fusion'


class NativeStationaryTests(unittest.TestCase):
    def test_real_cpp_factors_and_sensor_gates(self):
        if not shutil.which('cmake') or not shutil.which('g++'):
            self.skipTest('C++/CMake toolchain unavailable')
        with tempfile.TemporaryDirectory(prefix='vins_stationary_test_') as temp:
            directory = Path(temp)
            cmake = '''cmake_minimum_required(VERSION 3.5)
project(vins_stationary_test CXX)
set(CMAKE_CXX_STANDARD 11)
find_package(Ceres REQUIRED)
add_executable(native_stationary "{test}")
target_include_directories(native_stationary PRIVATE "{factor}" ${{CERES_INCLUDE_DIRS}})
target_link_libraries(native_stationary ${{CERES_LIBRARIES}})
add_executable(native_imu_chain "{chain_test}" "{marginalization}")
target_include_directories(native_imu_chain PRIVATE "{factor}" /opt/ros/noetic/include ${{CERES_INCLUDE_DIRS}})
find_library(ROS_CONSOLE_LIBRARY rosconsole PATHS /opt/ros/noetic/lib)
find_library(ROS_CPP_LIBRARY roscpp PATHS /opt/ros/noetic/lib)
target_link_libraries(native_imu_chain ${{CERES_LIBRARIES}} ${{ROS_CONSOLE_LIBRARY}} ${{ROS_CPP_LIBRARY}} pthread)
'''.format(test=ROOT/'tests/vins_stationary_factors_test.cpp',
           factor=SOURCE/'vins_estimator/src/factor', chain_test=ROOT/'tests/vins_imu_chain_test.cpp',
           marginalization=SOURCE/'vins_estimator/src/factor/marginalization_factor.cpp')
            (directory/'CMakeLists.txt').write_text(cmake)
            for command in (['cmake', '-S', str(directory), '-B', str(directory/'build')],
                            ['cmake', '--build', str(directory/'build'), '-j2']):
                result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, timeout=120)
                self.assertEqual(result.returncode, 0, result.stdout)
            result = subprocess.run([str(directory/'build/native_stationary')], capture_output=True,
                                    text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn('PASS:', result.stdout)
            result = subprocess.run([str(directory/'build/native_imu_chain')], capture_output=True,
                                    text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn('actual Schur bias prior', result.stdout)

    def test_reproducible_patch_and_idempotence(self):
        spec = importlib.util.spec_from_file_location('vins_patcher', ROOT/'vio/patch_upstream.py')
        patcher = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(patcher)
        relative_files = [Path('vins_estimator/src/estimator') / name for name in
                          ('estimator.cpp', 'estimator.h', 'parameters.cpp', 'parameters.h')]
        relative_files += [Path('vins_estimator/src/factor')/name for name in
                           ('marginalization_factor.cpp','marginalization_factor.h')]
        with tempfile.TemporaryDirectory(prefix='vins_stationary_patch_') as temp:
            directory = Path(temp)
            originals = {}
            for relative in relative_files:
                target = directory / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                originals[relative] = (SOURCE/relative).read_bytes()
                target.write_bytes(originals[relative])
            (directory/'vins_estimator/src/factor').mkdir(parents=True, exist_ok=True)
            reverse = subprocess.run(['patch', '--batch', '--fuzz=0', '-R', '-p1',
                                      '--directory', str(directory), '--input',
                                      str(ROOT/'vio/patches/stationary_health.patch')],
                                     capture_output=True, text=True)
            self.assertEqual(reverse.returncode, 0, reverse.stdout + reverse.stderr)
            patcher.patch_stationary_health(directory)
            for relative, content in originals.items():
                self.assertEqual((directory/relative).read_bytes(), content)
            before = {p: p.read_bytes() for p in directory.rglob('*') if p.is_file()}
            patcher.patch_stationary_health(directory)
            self.assertEqual(before, {p: p.read_bytes() for p in directory.rglob('*') if p.is_file()})
            # Reverse the exact migration chain to each real old source version,
            # then require the installer to reconstruct identical final sources.
            migrations = ('stationary_health_v9_visual_diagnostics.patch',
                          'stationary_health_v8_to_v9.patch',
                          'stationary_health_v7_to_v8.patch',
                          'stationary_health_v6_to_v7.patch',
                          'stationary_health_v5_to_v6.patch',
                          'stationary_health_v4_to_v5.patch',
                          'stationary_health_v3_to_v4.patch',
                          'stationary_health_v2_to_v3.patch',
                          'stationary_health_v1_to_v2.patch')
            for count in range(1, len(migrations)+1):
                for name in migrations[:count]:
                    result = subprocess.run(['patch','--batch','--fuzz=0','-R','-p1',
                        '--directory',str(directory),'--input',str(ROOT/'vio/patches'/name)],
                        capture_output=True,text=True)
                    self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                patcher.patch_stationary_health(directory)
                for relative,content in originals.items():
                    self.assertEqual((directory/relative).read_bytes(),content)
            # Partial opt-in diagnostic installation must also fail before edits.
            one = directory/relative_files[0]
            intact = one.read_text()
            one.write_text(intact.replace('PX4_AGENT_VISUAL_LIFECYCLE_DIAGNOSTICS_V1','REMOVED_DIAGNOSTIC_MARKER'))
            with self.assertRaisesRegex(RuntimeError, 'Partial'):
                patcher.patch_stationary_health(directory)
            one.write_text(intact)
            # A partly applied source tree must fail, rather than silently skip.
            one = directory/relative_files[0]
            one.write_text(one.read_text().replace('PX4_AGENT_STATIONARY_HEALTH_V9', 'REMOVED_MARKER'))
            with self.assertRaisesRegex(RuntimeError, 'Partial'):
                patcher.patch_stationary_health(directory)
        self.assertEqual((ROOT/'vio/patches/stationary_factors.h').read_bytes(),
                         (SOURCE/'vins_estimator/src/factor/stationary_factors.h').read_bytes())
        self.assertEqual((ROOT/'vio/patches/imu_chain_health.h').read_bytes(),
                         (SOURCE/'vins_estimator/src/factor/imu_chain_health.h').read_bytes())


if __name__ == '__main__':
    unittest.main()
