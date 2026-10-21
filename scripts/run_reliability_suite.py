#!/usr/bin/env python3
"""Repeatable real local-SITL reliability suite with recorded manager ownership.

--dry-run reads local assets only: no requests, processes, directories or flights.
Failed cases are retained and never automatically retried. A distinct next case
requires freshly observed landed/disarmed state and no active operator command.
"""
import argparse
import copy
import fcntl
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import time
import uuid
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
REGISTRY = ROOT / "scene/reliability/index.json"
CANONICAL_WORLD = ROOT / "scene/worlds/obstacle_yard.world"
CANONICAL_LAYOUT = ROOT / "scene/obstacle_layout.json"
START_SCRIPT = ROOT / "scripts/start_system.sh"
STOP_SCRIPT = ROOT / "scripts/stop_system.sh"
WEB_URL = "http://127.0.0.1:8080"
BRIDGE_URL = "http://127.0.0.1:8765"
PYTHON = "/usr/bin/python3"
DEFAULT_PX4_DIR = Path("/home/dut/PX4-Autopilot")
CASE_MAP = {
    "qwen_baseline": ("baseline", None, "complete_round_trip"),
    "seed_17": ("reliability_seed_17", "success", "complete_round_trip"),
    "seed_73": ("reliability_seed_73", "success", "complete_round_trip"),
    "seed_211": ("reliability_seed_211", "success", "complete_round_trip"),
    "blocked_route": ("reliability_blocked_17", "blocked_route", "safe_failure"),
    "lidar_drop": ("baseline", "lidar_drop", "safe_failure"),
    "lidar_delay": ("baseline", "lidar_delay", "safe_failure"),
    "lidar_invalid": ("baseline", "lidar_invalid", "safe_failure"),
    "temporary_obstacle": ("baseline", "temporary_obstacle", "wait_then_complete"),
    "persistent_obstacle": ("baseline", "persistent_obstacle", "wait_then_safe_failure"),
    "vio_reset": ("baseline", "vio_reset", "monitor_fault_and_complete"),
    "vio_tracking_loss": ("baseline", "vio_tracking_loss", "monitor_fault_and_complete"),
}
DEFAULT_CASES = [key for key in CASE_MAP if key != "vio_tracking_loss"]


class SafetyError(RuntimeError):
    """State or process ownership cannot safely authorize the next mutation."""


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def registry_assets(path=REGISTRY):
    registry = read_json(path)
    if type(registry.get("schema_version")) is not int or registry["schema_version"] != 1:
        raise ValueError("Unsupported reliability registry version")
    scenes = registry.get("scenes")
    if not isinstance(scenes, list):
        raise ValueError("Registry scenes must be a list")
    assets = {}
    directory = Path(path).resolve().parent
    for item in scenes:
        identifier = item.get("id") if isinstance(item, dict) else None
        if not isinstance(identifier, str) or identifier in assets:
            raise ValueError("Scene registrations require unique identifiers")
        registered = {}
        for name in ("world", "manifest", "layout", "tasks"):
            value = item.get(name)
            if not isinstance(value, str) or Path(value).name != value or value in ("", ".", ".."):
                raise ValueError("Registered assets must be direct relative child paths")
            asset = (directory / value).resolve()
            if asset.parent != directory or (item.get("asset_sha256") or {}).get(name) != sha256(asset):
                raise ValueError("Registered asset path/hash mismatch: " + identifier + "/" + name)
            registered[name] = str(asset)
        digest = sha256(registered["world"])
        if item.get("world_sha256") != digest or read_json(registered["manifest"]).get("world_sha256") != digest:
            raise ValueError("Registered world identity mismatch: " + identifier)
        if read_json(registered["layout"]).get("world_sha256") != digest:
            raise ValueError("Evaluation layout/world mismatch: " + identifier)
        registered["world_sha256"] = digest
        assets[identifier] = registered
    return assets


def build_plan(output, selected=None, repeat=1, only_navigation=False, timeout=800., registry=REGISTRY):
    output = Path(output).resolve()
    if output.exists():
        raise ValueError("Refusing to overwrite existing suite evidence: " + str(output))
    if type(repeat) is not int or not 1 <= repeat <= 10:
        raise ValueError("--repeat must be an explicitly requested integer in 1..10")
    if not 1 <= timeout <= 1200:
        raise ValueError("--timeout must be in 1..1200 wall seconds")
    chosen = list(selected or DEFAULT_CASES)
    if len(set(chosen)) != len(chosen) or any(key not in CASE_MAP for key in chosen):
        raise ValueError("Cases must be unique known case IDs; use --repeat for explicit repetitions")
    if only_navigation:
        chosen = [key for key in chosen if key != "qwen_baseline"]
    if not chosen:
        raise ValueError("No selected cases")
    assets = registry_assets(registry)
    baseline_hash = sha256(CANONICAL_WORLD)
    if read_json(CANONICAL_LAYOUT).get("world_sha256") != baseline_hash:
        raise ValueError("Canonical evaluation layout/world mismatch")
    assets["baseline"] = {"world": str(CANONICAL_WORLD), "layout": str(CANONICAL_LAYOUT),
                          "world_sha256": baseline_hash}
    cases = []
    for key in chosen:
        scene_id, navigation_case, contract = CASE_MAP[key]
        asset = assets[scene_id]
        for iteration in range(1, repeat + 1):
            identifier = "{}_{:02d}".format(key, iteration)
            case_dir = output / identifier
            runner_output = case_dir / "runner"
            if navigation_case is None:
                runner = ROOT / "scripts/test_avoidance_scene.py"
                args = [PYTHON, str(runner), "--task", "autonomous_goal_round_trip",
                        "--layout", asset["layout"], "--base-url", WEB_URL,
                        "--output", str(runner_output), "--timeout", str(timeout)]
                report_path = runner_output / "autonomous_goal_round_trip/task_validation.json"
            else:
                runner = ROOT / "scripts/test_navigation_reliability.py"
                args = [PYTHON, str(runner), "--case", navigation_case, "--layout", asset["layout"],
                        "--output", str(runner_output), "--timeout", str(timeout)]
                report_path = runner_output / "task_validation.json"
            cases.append({"id": identifier, "case": key, "iteration": iteration,
                          "world": asset["world"], "world_sha256": asset["world_sha256"],
                          "evaluation_layout": asset["layout"], "runner": str(runner), "args": args,
                          "expected_contract": contract, "actual_model_call": navigation_case is None,
                          "report_path": str(report_path), "case_directory": str(case_dir),
                          "stdout_path": str(case_dir / "runner.log")})
    return {"schema_version": 1, "source": "actual_local_sitl_reliability_suite",
            "output": str(output), "requested_case_ids": chosen, "explicit_repeat": repeat,
            "automatic_failure_retries": 0, "case_timeout_wall_s": timeout,
            "readiness_timeout_wall_s": 180., "registry_sha256": sha256(registry),
            "px4_preflight": {"command": [str(Path(os.environ.get("PX4_DIR") or DEFAULT_PX4_DIR).resolve()
                                                / "build/px4_sitl_default/bin/px4-commander"), "check"],
                              "required_stdout": "Preflight check: OK", "subprocess_timeout_wall_s": 5.,
                              "ownership": "unique PX4 daemon descendant of the validated suite manager",
                              "arming_requested": False, "force_or_parameter_changes": False},
            "manager_environment": {"GUI": "true", "PX4_SIM_SPEED_FACTOR": "0.35", "UAV_RELIABILITY_TESTING": "true"},
            "restore": {"world": str(CANONICAL_WORLD), "world_sha256": baseline_hash,
                        "GUI": "true", "PX4_SIM_SPEED_FACTOR": "0.35", "UAV_RELIABILITY_TESTING": "false"},
            "cases": cases,
            "limits": ["No automatic retry of a failed case; repetitions exist only when explicitly requested.",
                       "An unsafe aircraft or active operator command stops all further cases and prevents manager teardown.",
                       "The authorized test window must remain exclusive; each manager transition rechecks operator state."]}


def safe_ground(bridge, web):
    if web.get("active_command_id"):
        raise SafetyError("An operator/Agent command is active; no manager will be stopped")
    for name, state in (("bridge", bridge), ("web", web)):
        if state.get("simulation_only") is not True or state.get("armed") is not False or state.get("landed_state") != 1:
            raise SafetyError(name + " has not confirmed local simulation, landed and disarmed")
        if (state.get("mission") or {}).get("status") == "running":
            raise SafetyError(name + " still reports a running flight mission")
    if bridge.get("scene", {}).get("world_sha256") != web.get("scene", {}).get("world_sha256"):
        raise SafetyError("Bridge and web world identities disagree")
    return {"armed": False, "landed_state": 1, "active_command_id": None,
            "world_sha256": bridge.get("scene", {}).get("world_sha256")}


def ready_state(bridge, web, world_sha256, testing):
    if web.get("active_command_id") or any((s.get("mission") or {}).get("status") == "running" for s in (bridge, web)):
        raise SafetyError("An operator command/flight arrived during startup")
    if any(s.get("armed") is True for s in (bridge, web)):
        raise SafetyError("Aircraft became armed during startup; no test will be submitted")
    if any(s.get("simulation_only") is not True or s.get("armed") is not False or s.get("landed_state") != 1
           for s in (bridge, web)):
        return False
    safe_ground(bridge, web)
    if not bridge.get("ready") or not web.get("ready") or not bridge.get("gazebo", {}).get("available"):
        return False
    position = bridge.get("position_enu_m")
    if (not isinstance(position, list) or len(position) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in position)
            or abs(position[2]) > .2):
        return False
    scene = bridge.get("scene") or {}
    if not scene.get("verified") or scene.get("world_sha256") != world_sha256:
        return False
    nav = bridge.get("navigation") or {}
    if not nav.get("enabled") or not nav.get("lidar_available") or nav.get("lidar_sample_count") != 360:
        return False
    if not bridge.get("vio", {}).get("ready") or not all(bridge.get("camera", {}).get(side, {}).get("available") for side in ("left", "right")):
        return False
    hook = bridge.get("reliability_test") or {}
    if bool(hook.get("enabled")) != testing or hook.get("active_fault") or hook.get("pending_fault"):
        return False
    return True


def process_identity(run_dir):
    run_dir = Path(run_dir).resolve()
    record = read_json(run_dir / "controller.json")
    if Path(record["script"]).resolve() != START_SCRIPT.resolve():
        raise SafetyError("Manager record belongs to another startup script")
    pid = int(record["pid"])
    process = Path("/proc") / str(pid)
    try:
        fields = (process / "stat").read_text().rsplit(")", 1)[1].split()
    except FileNotFoundError:
        return None
    if fields[0] == "Z":
        return None
    if fields[19] != str(record["start_time"]) or process.stat().st_uid != os.getuid():
        raise SafetyError("Manager PID identity/owner does not match recorded startup")
    cwd = (process / "cwd").resolve()
    args = [(Path(os.fsdecode(value)) if Path(os.fsdecode(value)).is_absolute() else cwd / os.fsdecode(value)).resolve()
            for value in (process / "cmdline").read_bytes().split(b"\0") if value]
    if START_SCRIPT.resolve() not in args:
        raise SafetyError("Manager process is not this project's start_system.sh")
    return {"pid": pid, "start_time": fields[19], "run_directory": str(run_dir),
            "script": str(START_SCRIPT.resolve())}


def owned_px4_process(manager, commander):
    """Require the command's PX4 daemon to be a descendant of this manager."""
    records, candidates = {}, []
    expected_binary = Path(commander).resolve()
    for directory in Path("/proc").iterdir():
        if not directory.name.isdigit():
            continue
        try:
            fields = (directory / "stat").read_text().rsplit(")", 1)[1].split()
            pid = int(directory.name)
            records[pid] = {"pid": pid, "parent_pid": int(fields[1]), "start_time": fields[19], "state": fields[0]}
            command = [os.fsdecode(v) for v in (directory / "cmdline").read_bytes().split(b"\0") if v]
            if (fields[0] != "Z" and directory.stat().st_uid == os.getuid()
                    and (directory / "exe").resolve() == expected_binary
                    and command and Path(command[0]).name == "px4" and "-s" in command):
                candidates.append(pid)
        except (OSError, ValueError, IndexError):
            continue
    owned = []
    for pid in candidates:
        ancestors, current = [], pid
        while current in records and current not in ancestors:
            ancestors.append(current)
            if current == manager["pid"]:
                owned.append({**records[pid], "ancestor_pids": ancestors, "binary": str(expected_binary)})
                break
            current = records[current]["parent_pid"]
    if len(candidates) != 1 or len(owned) != 1:
        raise SafetyError("Cannot uniquely verify this manager's local PX4 process for preflight checks")
    return owned[0]


def preflight_status(stdout, stderr, return_code):
    """Commander check can return zero while reporting FAILED; require its OK."""
    strip_ansi = lambda text: re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", text)
    statuses = re.findall(r"Preflight check:\s*(OK|FAILED)\b", strip_ansi(stdout))
    failed_stderr = "Preflight check: FAILED" in strip_ansi(stderr)
    return {"passed": return_code == 0 and statuses == ["OK"] and not failed_stderr,
            "parsed_statuses": statuses}


class SystemOperations:
    """The only transport/process layer; tests replace this boundary, not orchestration."""
    def __init__(self):
        self.started = None
        self.px4_directory = DEFAULT_PX4_DIR
        self.preflight_evidence_path = None
        self.preflight_attempt = 0

    @staticmethod
    def read_state(url):
        with urlopen(url, timeout=2.) as response:
            value = json.load(response)
        if not isinstance(value, dict):
            raise SafetyError("State response must be an object")
        return value

    def states(self):
        return self.read_state(BRIDGE_URL + "/state"), self.read_state(WEB_URL + "/api/state")

    def current_manager(self):
        latest = ROOT / "logs/system/latest"
        return process_identity(latest) if (latest / "controller.json").is_file() else None

    def initial_manager(self):
        identity = self.current_manager()
        try:
            bridge, web = self.states()
        except (OSError, URLError, ValueError):
            if identity:
                raise SafetyError("Existing manager state cannot be verified; refusing to stop it") from None
            # Both endpoints must be absent: an unrelated live bridge or web
            # server must not be reused or interrupted.
            for url in (BRIDGE_URL + "/state", WEB_URL + "/api/state"):
                try:
                    self.read_state(url)
                except (OSError, URLError):
                    continue
                raise SafetyError("Unowned live service detected; refusing manager mutation")
            return None
        safe_ground(bridge, web)
        if identity is None:
            raise SafetyError("Live services have no validated project manager owner")
        return identity

    def stop_manager(self, identity, stdout_path):
        if identity is None:
            return
        bridge, web = self.states()
        safe_ground(bridge, web)
        current = self.current_manager()
        if current != identity:
            raise SafetyError("Manager identity changed; refusing to stop a different invocation")
        if process_identity(identity["run_directory"]) != identity:
            raise SafetyError("Pinned manager identity changed before shutdown")
        with Path(stdout_path).open("x") as log:
            result = subprocess.run(["bash", str(STOP_SCRIPT), identity["run_directory"]],
                                    cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=200., check=False)
        if result.returncode:
            raise RuntimeError("Whole-manager shutdown failed with exit code " + str(result.returncode))

    def start_manager(self, world, testing, stdout_path):
        environment = os.environ.copy()
        self.px4_directory = Path(environment.get("PX4_DIR") or DEFAULT_PX4_DIR).resolve()
        environment.update(GUI="true", PX4_SIM_SPEED_FACTOR="0.35", UAV_RELIABILITY_TESTING="true" if testing else "false",
                           PX4_DIR=str(self.px4_directory))
        log_path = Path(stdout_path)
        self.preflight_evidence_path = log_path.with_name(log_path.stem + "_preflight_checks.jsonl")
        self.preflight_attempt = 0
        if self.preflight_evidence_path.exists():
            raise ValueError("Refusing to append preflight checks to an existing manager evidence file")
        with Path(stdout_path).open("x") as log:
            self.started = subprocess.Popen(["bash", str(START_SCRIPT), "--world-file", str(world)],
                cwd=ROOT, env=environment, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
        return self.started

    def preflight_check(self, manager, timeout):
        if timeout <= 0:
            return {"passed": False, "error": "Readiness deadline expired before preflight check"}
        if self.current_manager() != manager or process_identity(manager["run_directory"]) != manager:
            raise SafetyError("Manager identity changed before PX4 preflight check")
        # Preserve the symlink's argv[0]: resolving px4-commander to px4 for
        # execution would start PX4 rather than dispatch its read-only command.
        commander = self.px4_directory / "build/px4_sitl_default/bin/px4-commander"
        px4 = owned_px4_process(manager, commander)
        self.preflight_attempt += 1
        started, wall_started = time.monotonic(), time.time()
        evidence = {"attempt": self.preflight_attempt, "command": [str(commander), "check"],
                    "manager": copy.deepcopy(manager), "px4_process": px4,
                    "started_at_wall_s": wall_started, "timeout_wall_s": min(5., timeout),
                    "passed": False, "return_code": None, "stdout": "", "stderr": ""}
        def decoded(value):
            return value.decode(errors="replace") if isinstance(value, bytes) else value or ""
        try:
            result = subprocess.run(evidence["command"], cwd=ROOT, stdin=subprocess.DEVNULL,
                                    capture_output=True, text=True, timeout=evidence["timeout_wall_s"], check=False)
            evidence.update(return_code=result.returncode, stdout=result.stdout[-16000:], stderr=result.stderr[-16000:])
            evidence.update(preflight_status(evidence["stdout"], evidence["stderr"], result.returncode))
        except (OSError, subprocess.TimeoutExpired) as exc:
            evidence.update(error_type=type(exc).__name__, error=str(exc),
                            stdout=decoded(getattr(exc, "stdout", ""))[-16000:],
                            stderr=decoded(getattr(exc, "stderr", ""))[-16000:])
        evidence.update(completed_at_wall_s=time.time(), duration_wall_s=round(time.monotonic()-started, 4))
        if self.preflight_evidence_path is not None:
            with self.preflight_evidence_path.open("a") as stream:
                stream.write(json.dumps(evidence, ensure_ascii=False, allow_nan=False) + "\n")
        return evidence

    def wait_ready(self, world_sha256, testing, timeout):
        deadline = time.monotonic() + timeout
        last_error = "Startup not observed"
        identity = None
        while time.monotonic() < deadline:
            if self.started and self.started.poll() is not None:
                raise RuntimeError("Started manager exited with code " + str(self.started.returncode))
            try:
                identity = self.current_manager()
                if identity and self.started and identity["pid"] != self.started.pid:
                    raise SafetyError("Another manager replaced this suite's startup")
                bridge, web = self.states()
                # Operators retain priority even during startup. Ground sensor
                # readiness may lag; an armed aircraft is never treated as ready.
                if web.get("active_command_id"):
                    raise SafetyError("Operator command arrived during startup")
                if identity and ready_state(bridge, web, world_sha256, testing):
                    remaining = deadline-time.monotonic()
                    preflight = self.preflight_check(identity, min(5., remaining))
                    if preflight["passed"]:
                        result = safe_ground(bridge, web)
                        result["px4_preflight"] = {"passed": True, "attempt": preflight["attempt"],
                                                  "checked_at_wall_s": preflight["completed_at_wall_s"],
                                                  "evidence_path": str(self.preflight_evidence_path)}
                        return identity, result
                    last_error = preflight.get("error") or "PX4 preflight check has not reported OK"
            except SafetyError:
                raise
            except (OSError, URLError, ValueError) as exc:
                last_error = str(exc)
            time.sleep(min(1., max(0., deadline-time.monotonic())))
        raise TimeoutError("Manager readiness exceeded {:.0f}s: {}".format(timeout, last_error))

    def run_runner(self, args, stdout_path, timeout):
        with Path(stdout_path).open("x") as log:
            process = subprocess.Popen(args, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                return {"exit_code": process.wait(timeout=timeout + 220.), "timed_out": False}
            except subprocess.TimeoutExpired:
                # Only the runner's own process is stopped. The simulation stays
                # running until its observed state can authorize any teardown.
                process.terminate()
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5.)
                return {"exit_code": process.returncode, "timed_out": True}


def case_outcome(case, execution):
    result = {"id": case["id"], "case": case["case"], "iteration": case["iteration"],
              "expected_contract": case["expected_contract"], "world_sha256": case["world_sha256"],
              "report_path": case["report_path"], "stdout_path": case["stdout_path"],
              "runner_exit_code": execution["exit_code"], "runner_timed_out": execution["timed_out"],
              "requested_mission_completed": False, "expected_safety_contract_pass": False,
              "safety_confirmed": False}
    try:
        report = read_json(case["report_path"])
        result["requested_mission_completed"] = (
            report.get("command", {}).get("status") == "succeeded" if case["actual_model_call"]
            else report.get("validation", {}).get("original_mission_status") == "succeeded")
        result["expected_safety_contract_pass"] = (report.get("validation_passed") is True
            and execution["exit_code"] == 0 and not execution["timed_out"])
        result["actual_flight"] = report.get("actual_flight")
        result["actual_model_call"] = case["actual_model_call"]
        result["error"] = report.get("error") or report.get("validation", {}).get("original_mission_error")
    except (OSError, ValueError, AttributeError) as exc:
        result["report_error"] = str(exc)
    return result


def write_json(path, value, exclusive=False):
    path = Path(path)
    text = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if exclusive:
        with path.open("x") as stream:
            stream.write(text)
    else:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(text)
        temporary.replace(path)


@contextmanager
def suite_lock():
    path = ROOT / "logs/reliability/suite.lock"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+") as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SafetyError("Another reliability suite holds the manager test window") from None
        try:
            yield
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def run_suite(plan, operations=None):
    operations = operations or SystemOperations()
    output = Path(plan["output"])
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "plan.json", plan, exclusive=True)
    summary = {"schema_version": 1, "source": plan["source"], "started_at_utc": datetime.now(timezone.utc).isoformat(),
               "status": "running", "planned_case_count": len(plan["cases"]), "cases": [],
               "automatic_failure_retries": 0, "restore": {"status": "pending"}}
    def save():
        write_json(output / "summary.json", summary)
    def phase(text):
        print("[suite] " + text, flush=True)
        with (output / "events.jsonl").open("a") as stream:
            stream.write(json.dumps({"wall_time": time.time(), "phase": text}, ensure_ascii=False) + "\n")
        save()
    manager, changed = None, False
    try:
        manager = operations.initial_manager()
        summary["initial_manager"] = copy.deepcopy(manager)
        for case in plan["cases"]:
            case_dir = Path(case["case_directory"])
            case_dir.mkdir(exist_ok=False)
            phase("{}: checking landed/disarmed state before manager transition".format(case["id"]))
            operations.stop_manager(manager, case_dir / "stop_previous_manager.log")
            manager = None
            changed = True
            operations.start_manager(case["world"], True, case_dir / "manager.log")
            manager, initial_state = operations.wait_ready(case["world_sha256"], True, plan["readiness_timeout_wall_s"])
            phase("{}: ready; starting {}".format(case["id"], "real Qwen goal-only task" if case["actual_model_call"] else "real executor reliability case"))
            # No operator command may enter between readiness and test startup.
            bridge, web = operations.states()
            safe_ground(bridge, web)
            execution = operations.run_runner(case["args"], case["stdout_path"], plan["case_timeout_wall_s"])
            outcome = case_outcome(case, execution)
            outcome["manager"] = copy.deepcopy(manager)
            outcome["initial_ground_state"] = initial_state
            summary["cases"].append(outcome)
            try:
                bridge, web = operations.states()
                outcome["final_ground_state"] = safe_ground(bridge, web)
                outcome["safety_confirmed"] = True
            except Exception as exc:
                outcome["safety_error"] = str(exc)
                write_json(case_dir / "suite_case_summary.json", outcome, exclusive=True)
                phase(case["id"] + ": stopping suite; landing/command ownership unconfirmed")
                raise SafetyError(outcome["safety_error"]) from None
            write_json(case_dir / "suite_case_summary.json", outcome, exclusive=True)
            phase("{}: exit={}, mission_completed={}, expected_contract_pass={}, landed=True".format(
                case["id"], outcome["runner_exit_code"], outcome["requested_mission_completed"], outcome["expected_safety_contract_pass"]))
        summary["status"] = "passed" if all(c["expected_safety_contract_pass"] for c in summary["cases"]) else "completed_with_failures"
    except (Exception, KeyboardInterrupt) as exc:
        summary.update(status="aborted", error=type(exc).__name__ + ": " + str(exc))
        phase("suite aborted: " + summary["error"])
    finally:
        if changed:
            try:
                # Reconfirm ground/command authority before touching any manager.
                # A failed safety check leaves the current system alive for recovery.
                current = operations.initial_manager()
                if current is not None:
                    if manager is not None and current != manager:
                        raise SafetyError("Manager changed before restore; preserving other invocation")
                    if manager is None and operations.started and current["pid"] != operations.started.pid:
                        raise SafetyError("Unowned manager found after failed startup; refusing restore")
                phase("restoring canonical obstacle_yard with test hooks disabled")
                operations.stop_manager(current, output / "restore_stop_manager.log")
                operations.start_manager(plan["restore"]["world"], False, output / "restore_manager.log")
                identity, state = operations.wait_ready(plan["restore"]["world_sha256"], False, plan["readiness_timeout_wall_s"])
                summary["restore"] = {"status": "ready", "manager": identity, "ground_state": state,
                                      "world_sha256": plan["restore"]["world_sha256"], "testing_enabled": False}
            except Exception as exc:
                summary["restore"] = {"status": "blocked", "error": type(exc).__name__ + ": " + str(exc),
                                      "current_manager_left_running": True}
                summary["status"] = "aborted"
        else:
            summary["restore"] = {"status": "not_needed", "initial_manager_preserved": True}
        summary.update(finished_at_utc=datetime.now(timezone.utc).isoformat(),
                       executed_case_count=len(summary["cases"]),
                       requested_mission_completed_count=sum(c["requested_mission_completed"] for c in summary["cases"]),
                       expected_safety_contract_pass_count=sum(c["expected_safety_contract_pass"] for c in summary["cases"]))
        phase("finished: {} ({}/{})".format(summary["status"], summary["expected_safety_contract_pass_count"], summary["planned_case_count"]))
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--case", action="append", choices=CASE_MAP)
    parser.add_argument("--only-navigation", action="store_true", help="Omit the real Qwen baseline")
    parser.add_argument("--repeat", type=int, default=1, help="Explicit planned repetitions; no automatic retry")
    parser.add_argument("--timeout", type=float, default=800.)
    parser.add_argument("--output", type=Path, default=None, help="New suite directory; existing paths are refused")
    args = parser.parse_args(argv)
    output = args.output or ROOT / "logs/reliability" / ("suite_" + datetime.now().strftime("%Y%m%d-%H%M%S") + "_" + uuid.uuid4().hex[:8])
    try:
        plan = build_plan(output, args.case, args.repeat, args.only_navigation, args.timeout)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    if args.dry_run:
        print(json.dumps({**plan, "dry_run": True, "actual_requests": False, "actual_processes": False}, ensure_ascii=False, indent=2))
        return 0
    try:
        with suite_lock():
            summary = run_suite(plan)
    except (OSError, SafetyError) as exc:
        print("[suite] " + str(exc), file=sys.stderr, flush=True)
        return 1
    return 0 if summary["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
