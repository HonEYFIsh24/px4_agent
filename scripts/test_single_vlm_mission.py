"""Real single-VLM takeoff -> camera observation -> landing integration check."""
import json
import os
import time
from pathlib import Path
from urllib.request import Request, urlopen

BASE = os.environ.get("UAV_WEB_URL", "http://127.0.0.1:8080")
OUTPUT = Path(__file__).resolve().parents[1] / "logs/single_vlm"
OUTPUT.mkdir(parents=True, exist_ok=True)


def request(path, payload=None):
    data = json.dumps(payload).encode() if payload is not None else None
    with urlopen(Request(BASE + path, data=data, headers={"Content-Type": "application/json"}), timeout=8) as response:
        return json.load(response)


report = {"started_at": time.time(), "actual_model_call": True, "actual_flight": True, "samples": []}
try:
    initial = request("/api/state")
    report["initial_state"] = initial
    assert initial["ready"] and not initial["armed"] and initial["landed_state"] == 1
    assert initial["agent_architecture"] == "single_vlm"
    assert initial["model"]["name"] == initial["vision_model"]["name"]
    prompt = "起飞到本地高度2米并保持当前位置，寻找当前相机视野中清晰可见的绿色地面图案，报告大概位置及周围有什么，然后降落。无需移动到目标；看不清或没有发现请如实报告。"
    accepted = request("/api/commands", {"prompt": prompt, "mode": "task"})
    report["command_id"] = accepted["id"]
    print("Submitted real single-VLM mixed mission:", accepted["id"], flush=True)
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline:
        state = request("/api/state")
        report["samples"].append({"wall_time": time.time(), "ready": state.get("ready"),
            "armed": state.get("armed"), "landed_state": state.get("landed_state"),
            "position_enu_m": state.get("position_enu_m"), "gazebo": state.get("gazebo"),
            "mission": state.get("mission"), "vio_ready": state.get("vio", {}).get("ready"),
            "camera_ready": {side: item.get("available") for side, item in state.get("camera", {}).items()}})
        command = request("/api/commands/" + accepted["id"])
        if command["status"] != "running":
            break
        time.sleep(.5)
    else:
        raise TimeoutError("Mixed flight/observation command did not finish within 180 seconds")
    report["command"] = command
    assert command["status"] == "succeeded", command
    perception = command["perception"]
    assert perception["status"] == "ok" and perception["architecture"] == "single_vlm"
    calls = command["model_calls"]
    assert calls and all(item["model"] == initial["model"]["name"] for item in calls)
    assert any("image" in item["input_modalities"] and perception["capture_id"] in item["capture_ids"] for item in calls)
    assert "image_base64" not in json.dumps(command)
    final = request("/api/state")
    report["final_state"] = final
    assert final["ready"] and not final["armed"] and final["landed_state"] == 1
    missions = {}
    for sample in report["samples"]:
        mission = sample.get("mission") or {}
        if mission.get("id"):
            missions[mission["id"]] = mission
    new_missions = [value for key, value in missions.items() if key != (initial.get("mission") or {}).get("id")]
    assert any(value["status"] == "succeeded" and value["steps"][0]["action"] == "takeoff" for value in new_missions)
    assert any(value["status"] == "succeeded" and [step["action"] for step in value["steps"]] == ["land"] for value in new_missions)
    assert all(step["action"] in ("takeoff", "hold", "land") for mission in new_missions for step in mission["steps"])
    stamp = perception["captured_at_sim_s"]
    matching = min(report["samples"], key=lambda sample: abs((sample.get("gazebo") or {}).get("stamp_sim_s", -1000)-stamp))
    report["sample_nearest_capture"] = matching
    assert matching["armed"] and abs(matching["position_enu_m"][2]-2) < .4, matching
    report["missions"] = new_missions
    report["validation_passed"] = True
    print("PASS: same VLM took off, inspected its actual airborne image, reported, and landed/disarmed", flush=True)
except Exception as exc:
    report["validation_passed"] = False
    report["error"] = str(exc)
    # A failed test must leave an airborne simulator under the direct landing executor.
    try:
        report["safety_landing"] = request("/api/land", {})
    except Exception as landing_exc:
        report["safety_landing_error"] = str(landing_exc)
    raise
finally:
    report["finished_at"] = time.time()
    (OUTPUT / "mixed_mission_validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
