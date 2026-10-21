"""Execute owned boot hooks with isolated parameter I/O; no ROS or flight."""
import copy
import importlib.util
import json
import os
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("px4_agent_prepare_ev_boot", ROOT / "scripts/prepare_ev_boot.py")
BOOT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BOOT)


class OwnedBootHookContracts(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.run = self.directory / "run space"
        self.run.mkdir()
        (self.run / "controller.json").write_text('{"pid":1}')
        self.px4 = self.directory / "px4 space"
        self.official = self.px4 / "ROMFS/px4fmu_common/init.d-posix/px4-rc.params"
        self.official.parent.mkdir(parents=True)
        self.official.write_text("#!/bin/sh\n# official per-instance parameters\n")
        self.original = dict(EKF2_HGT_REF=1, EKF2_BARO_CTRL=1, EKF2_GPS_CTRL=7,
                EKF2_EV_CTRL=15, EKF2_EV_QMIN=0, EKF2_EV_NOISE_MD=0, EKF2_EV_DELAY=0,
                EKF2_EVP_NOISE=.1, EKF2_EVV_NOISE=.1, EKF2_EVA_NOISE=.1,
                EKF2_EV_POS_X=0, EKF2_EV_POS_Y=0, EKF2_EV_POS_Z=0,
                COM_OBL_RC_ACT=0, COM_OF_LOSS_T=2., SDLOG_MODE=0, SDLOG_PROFILE=3)
        self.parameters = self.directory / "fake_parameters.json"
        self.parameters.write_text(json.dumps(self.original))
        self.bin = self.directory / "fake-bin"
        self.bin.mkdir()
        param = self.bin / "param"
        param.write_text('''#!/usr/bin/python3
import json,os,sys
from pathlib import Path
path=Path(os.environ['EV_LAUNCH_TEST_PARAMETERS'])
data=json.loads(path.read_text())
args=sys.argv[1:]
if os.environ.get('EV_LAUNCH_TEST_FAIL') == ' '.join(args):
    sys.exit(1)
if args[0]=='show':
    if '-a' in args:
        for name,value in data.items(): print(name+' : '+str(value))
    else:
        name=args[-1]
        value=data[name]
        print(value if '-q' in args else name+' : '+str(value))
elif args[0]=='set':
    name=args[1]
    value=float(args[2])
    data[name]=int(value) if value.is_integer() else value
    with path.open('w') as stream: json.dump(data,stream)
    with Path(os.environ['EV_LAUNCH_TEST_SET_LOG']).open('a') as stream:
        stream.write(name+'='+str(value)+'\\n')
else: sys.exit(1)
''')
        param.chmod(0o700)
        self.environment = {**os.environ, "PATH": str(self.bin) + os.pathsep + os.environ.get("PATH", ""),
                "PX4_AGENT_EV_PROFILE": "no_gps", "PX4_AGENT_EV_BOOT_RUN_DIR": str(self.run),
                "PX4_SIM_MODEL": "gazebo-classic_iris", "EV_LAUNCH_TEST_PARAMETERS": str(self.parameters),
                "EV_LAUNCH_TEST_SET_LOG": str(self.directory / "set.log")}

    def execute(self, environment=None):
        return subprocess.run(["/bin/sh", str(self.run / "runtime-bin/px4-rc.params")],
                env=environment or self.environment, capture_output=True, text=True, timeout=10)

    def test_prepare_has_no_live_parameter_io_or_upstream_mutation(self):
        original_hook = self.official.read_bytes()
        parameters = self.parameters.read_bytes()
        with patch.object(subprocess, "run", side_effect=AssertionError("no process during preparation")):
            metadata = BOOT.prepare(self.run, self.px4)
        self.assertEqual(self.official.read_bytes(), original_hook)
        self.assertEqual(self.parameters.read_bytes(), parameters)
        self.assertFalse(metadata["gps_disabled_before_ekf_start"])
        self.assertFalse(metadata["executed"])

    def test_actual_private_hook_snapshots_imported_params_and_applies_no_gps_in_correct_order(self):
        BOOT.prepare(self.run, self.px4)
        result = self.execute()
        self.assertEqual(result.returncode, 0, result.stderr)
        metadata = BOOT.capture(self.run)
        current = json.loads(self.parameters.read_text())
        self.assertEqual(current, BOOT.target_profile("no_gps", self.original))
        self.assertEqual(json.loads((self.run / "startup-original.json").read_text())["parameters"], self.original)
        lines = (self.directory / "set.log").read_text().splitlines()
        self.assertEqual(lines[0], "EKF2_HGT_REF=3.0")
        self.assertLess(lines.index("EKF2_GPS_CTRL=0.0"), lines.index("EKF2_EV_CTRL=7.0"))
        self.assertEqual(current["SDLOG_PROFILE"], 131)
        self.assertTrue(metadata["gps_disabled_before_ekf_start"])
        self.assertTrue(metadata["executed"])

    def test_private_hook_cannot_mutate_another_profile_or_run_or_vehicle(self):
        BOOT.prepare(self.run, self.px4)
        for overrides in ({"PX4_AGENT_EV_PROFILE": "shadow"},
                          {"PX4_AGENT_EV_BOOT_RUN_DIR": str(self.directory / "foreign")},
                          {"PX4_SIM_MODEL": "real_hardware"}):
            with self.subTest(overrides=overrides):
                result = self.execute({**self.environment, **overrides})
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(json.loads(self.parameters.read_text()), self.original)
                self.assertFalse((self.run / "startup-original.txt").exists())

    def test_no_hook_execution_is_rejected_instead_of_claiming_gps_denied_boot(self):
        BOOT.prepare(self.run, self.px4)
        with self.assertRaises((RuntimeError, OSError)):
            BOOT.capture(self.run)
        self.assertFalse(json.loads((self.run / "ev-boot.json").read_text())["executed"])

    def test_all_original_reads_must_succeed_before_any_mutation(self):
        BOOT.prepare(self.run, self.px4)
        result = self.execute({**self.environment, "EV_LAUNCH_TEST_FAIL": "show -a"})
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(json.loads(self.parameters.read_text()), self.original)
        self.assertFalse((self.run / "startup-hook-completed").exists())

    def test_partial_mutation_keeps_original_restoration_record_without_false_boot_proof(self):
        BOOT.prepare(self.run, self.px4)
        result = self.execute({**self.environment, "EV_LAUNCH_TEST_FAIL": "set EKF2_EV_CTRL 7"})
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(BOOT.capture_original(self.run), self.original)
        self.assertEqual(json.loads((self.run / "startup-original.json").read_text())["parameters"], self.original)
        with self.assertRaises(RuntimeError):
            BOOT.capture(self.run)
        self.assertFalse(json.loads((self.run / "ev-boot.json").read_text())["executed"])

    def test_wrong_pre_ekf_readback_preserves_original_but_rejects_boot_proof(self):
        BOOT.prepare(self.run, self.px4)
        self.assertEqual(self.execute().returncode, 0)
        path = self.run / "startup-target.txt"
        path.write_text(path.read_text().replace("EKF2_GPS_CTRL : 0", "EKF2_GPS_CTRL : 7"))
        with self.assertRaises(RuntimeError):
            BOOT.capture(self.run)
        self.assertEqual(json.loads((self.run / "startup-original.json").read_text())["parameters"], self.original)

    def test_modified_hook_or_restore_record_is_rejected(self):
        BOOT.prepare(self.run, self.px4)
        self.assertEqual(self.execute().returncode, 0)
        BOOT.capture(self.run)
        path = self.run / "startup-original.json"
        record = json.loads(path.read_text())
        record["parameters"]["EKF2_GPS_CTRL"] = 0
        path.write_text(json.dumps(record))
        with self.assertRaises(RuntimeError):
            BOOT.capture_original(self.run)
        hook = self.run / "runtime-bin/px4-rc.params"
        hook.write_text(hook.read_text() + "# changed\n")
        with self.assertRaises(RuntimeError):
            BOOT.capture_original(self.run)

    def test_upstream_hook_is_sourced_without_modifying_it(self):
        self.official.write_text("#!/bin/sh\nparam set SDLOG_PROFILE 7\n")
        before = self.official.read_bytes()
        BOOT.prepare(self.run, self.px4)
        self.assertEqual(self.execute().returncode, 0)
        BOOT.capture(self.run)
        self.assertEqual(self.official.read_bytes(), before)
        self.assertEqual(json.loads((self.run / "startup-original.json").read_text())["parameters"]["SDLOG_PROFILE"], 7)
        self.assertEqual(json.loads(self.parameters.read_text())["SDLOG_PROFILE"], 135)


class LaunchEntryContracts(unittest.TestCase):
    def test_roslaunch_gui_and_shadow_defaults_and_explicit_profile_fault_forwarding(self):
        root = ET.parse(ROOT / "launch/uav_system.launch").getroot()
        args = {arg.get("name"): arg.get("default") for arg in root.findall("arg")}
        self.assertEqual(args["gui"], "true")
        self.assertEqual(args["ev_profile"], "shadow")
        self.assertEqual(args["ev_test_faults"], "false")
        node = root.find("node")
        env = {item.get("name"): item.get("value") for item in node.findall("env")}
        self.assertEqual(env["PX4_AGENT_EV_PROFILE"], "$(arg ev_profile)")
        self.assertEqual(env["PX4_AGENT_EV_TEST_FAULTS"], "$(arg ev_test_faults)")

    def test_bad_profile_is_rejected_before_any_service_inspection_or_mutation(self):
        candidate = ROOT / "scripts/start_system.sh.next"
        if not candidate.exists():
            candidate = ROOT / "scripts/start_system.sh"
        result = subprocess.run(["bash", str(candidate), "--ev-profile", "real_aircraft"],
                capture_output=True, text=True, timeout=3)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("ev-profile", result.stderr)
        self.assertNotIn("检测到已有服务", result.stderr)

    def test_manager_cli_help_lists_modes_without_process_or_ros_actions(self):
        candidate = ROOT / "scripts/start_system.sh.next"
        if not candidate.exists():
            candidate = ROOT / "scripts/start_system.sh"
        result = subprocess.run(["bash", str(candidate), "--help"], capture_output=True, text=True, timeout=3)
        self.assertEqual(result.returncode, 0)
        self.assertIn("--ev-profile", result.stdout)
        self.assertIn("--ev-test-faults", result.stdout)

    def test_px4_official_hook_point_precedes_sensors_commander_and_vehicle_estimator_setup(self):
        source = Path("/home/dut/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS")
        if not source.exists():
            self.skipTest("this source contract requires the selected PX4 checkout")
        text = source.read_text()
        index = text.index(". px4-rc.params")
        for later in ("sensors start", "commander start", "rc.vehicle_setup"):
            self.assertLess(index, text.index(later))


if __name__ == "__main__":
    unittest.main()
