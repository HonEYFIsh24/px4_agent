#!/usr/bin/env python3
"""Prepare/capture a per-manager PX4 pre-EKF no-GPS parameter hook.

This writes only the selected run directory. It never contacts PX4/ROS, changes
the upstream checkout or applies a live parameter. PX4's normal rcS sources the
generated px4-rc.params via the manager's private PATH before estimator startup.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR.parent) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR.parent))
from scripts.configure_ev_profile import PARAMETERS, parse_value, target_profile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(run_directory, px4_directory):
    run = Path(run_directory).resolve()
    original_hook = Path(px4_directory).resolve() / "ROMFS/px4fmu_common/init.d-posix/px4-rc.params"
    if not (run / "controller.json").is_file() or not original_hook.is_file():
        raise RuntimeError("owned run record or official parameter hook is missing")
    directory = run / "runtime-bin"
    directory.mkdir(exist_ok=False)
    raw = run / "startup-original.txt"
    observed = run / "startup-target.txt"
    marker = run / "startup-hook-completed"
    hook = directory / "px4-rc.params"
    quote = shlex.quote
    lines = ["#!/bin/sh", "# Per-run pre-EKF hook. Original PX4 files are untouched.",
             "[ \"$PX4_AGENT_EV_PROFILE\" = no_gps ] || exit 1",
             "[ \"$PX4_AGENT_EV_BOOT_RUN_DIR\" = " + quote(str(run)) + " ] || exit 1",
             "[ \"$PX4_SIM_MODEL\" = gazebo-classic_iris ] || exit 1",
             ". " + quote(str(original_hook)),
             "# Record imported/autostart parameters before the first mutation.",
             "{" ]
    lines += ["  param show -a || exit 1"]
    lines += ["} > " + quote(str(raw)),
              "original_sdlog_profile=$(param show -q SDLOG_PROFILE)",
              "case \"$original_sdlog_profile\" in ''|*[!0-9]*) exit 1 ;; esac"]
    # Profile constant fields and ordering share the ground configurator.
    target = target_profile("no_gps", {"SDLOG_PROFILE": 0})
    for name in PARAMETERS:
        value = ('"$((original_sdlog_profile | 128))"' if name == "SDLOG_PROFILE" else str(target[name]))
        lines.append("param set " + name + " " + value + " || exit 1")
    lines += ["{" , "  param show -a || exit 1"]
    lines += ["} > " + quote(str(observed)),
              "printf '%s\\n' 'no_gps pre-EKF parameter hook completed' > " + quote(str(marker)), ""]
    hook.write_text("\n".join(lines))
    hook.chmod(0o600)
    metadata = {"profile": "no_gps", "prepared": True, "executed": False,
                "gps_disabled_before_ekf_start": False,
                "hook": str(hook), "hook_sha256": digest(hook),
                "official_hook": str(original_hook), "official_hook_sha256": digest(original_hook),
                "startup_original_raw": str(raw), "startup_target_raw": str(observed),
                "integration": "private_PATH_px4_rc_params_before_simulator_sensors_and_ekf",
                "upstream_modified": False, "restore_source": str(run / "startup-original.json")}
    (run / "ev-boot.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return metadata


def capture_original(run_directory):
    run = Path(run_directory).resolve()
    metadata = json.loads((run / "ev-boot.json").read_text())
    if digest(Path(metadata["hook"])) != metadata["hook_sha256"]:
        raise RuntimeError("the private pre-EKF hook changed after preparation")
    raw_path = run / "startup-original.txt"
    original_text = raw_path.read_text()
    original = {name: parse_value(original_text, name) for name in PARAMETERS}
    original_path = run / "startup-original.json"
    if original_path.exists():
        if json.loads(original_path.read_text()).get("parameters") != original:
            raise RuntimeError("startup restore record does not match its immutable raw parameters")
    else:
        with original_path.open("x") as stream:
            json.dump({"parameters": original, "source": "pre_ekf_owned_px4_param_hook",
                       "raw_sha256": digest(raw_path), "hook_sha256": metadata["hook_sha256"]}, stream, indent=2)
            stream.write("\n")
    return original


def capture(run_directory):
    run = Path(run_directory).resolve()
    original = capture_original(run)
    metadata = json.loads((run / "ev-boot.json").read_text())
    if not (run / "startup-hook-completed").is_file():
        raise RuntimeError("the private pre-EKF hook did not complete; no GPS-bootstrap fallback")
    raw_path, target_path = run / "startup-original.txt", run / "startup-target.txt"
    target_text = target_path.read_text()
    target = {name: parse_value(target_text, name) for name in PARAMETERS}
    expected = target_profile("no_gps", original)
    if target != expected:
        raise RuntimeError("pre-EKF hook parameter readback does not match the no-GPS profile")
    metadata.update(executed=True, gps_disabled_before_ekf_start=True,
                    startup_original_raw_sha256=digest(raw_path),
                    startup_target_raw_sha256=digest(target_path), original_parameters=original,
                    startup_target_parameters=target)
    temporary = run / "ev-boot.json.next"
    temporary.write_text(json.dumps(metadata, indent=2) + "\n")
    os.replace(str(temporary), str(run / "ev-boot.json"))
    return metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-directory", type=Path, required=True)
    parser.add_argument("--px4-directory", type=Path, default=Path(os.getenv("PX4_DIR", "/home/dut/PX4-Autopilot")))
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--capture-original-only", action="store_true")
    args = parser.parse_args(argv)
    if args.capture and args.capture_original_only:
        parser.error("choose --capture or --capture-original-only")
    result = (capture_original(args.run_directory) if args.capture_original_only else
              capture(args.run_directory) if args.capture else prepare(args.run_directory, args.px4_directory))
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
