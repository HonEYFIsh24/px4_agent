"""Actual Gazebo flight test without a model API call. Start SITL and bridge first."""
import json
import math
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from client import BridgeClient


def control_test(client):
    accepted = client.request("/mission", {"steps": [{"action": "takeoff", "altitude": 2},
                                                     {"action": "hold", "duration": 60}],
                                            "description": "Cancellation and immediate landing test"})
    mission_id = accepted["mission"]["id"]
    deadline = time.monotonic() + 50
    while time.monotonic() < deadline:
        state = client.state()
        assert state["mission"]["id"] == mission_id
        assert state["mission"]["status"] == "running", state
        if state["mission"].get("action") == "hold":
            break
        time.sleep(0.2)
    else:
        raise AssertionError("takeoff did not finish")
    try:
        client.request("/mission", {"steps": [{"action": "land"}]})
    except RuntimeError:
        pass
    else:
        raise AssertionError("concurrent mission was accepted")
    stopped = client.request("/stop", {})
    assert stopped["holding"], stopped
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        cancelled = client.state()
        if cancelled["mission"]["status"] == "cancelled":
            break
        time.sleep(0.05)
    else:
        raise AssertionError("mission did not cancel promptly")
    time.sleep(1)
    held = client.state()
    assert held["mode"] == "OFFBOARD" and held["armed"]
    assert math.dist(held["position_enu_m"], stopped["state"]["position_enu_m"]) < 0.6
    result = client.wait_for(client.request("/land", {}))
    assert result["mission"]["status"] == "succeeded", result
    assert not result["armed"] and result["landed_state"] == 1
    output = Path(__file__).resolve().parent.parent / "logs" / "control_result.json"
    output.write_text(json.dumps({"cancelled_mission": cancelled["mission"], "landing": result},
                                ensure_ascii=False, indent=2), encoding="utf-8")
    print("PASS: concurrent mission rejected, prompt cancellation, OFFBOARD hold, immediate landing and disarming")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--control", action="store_true", help="Test cancellation, hold and immediate landing")
    args = parser.parse_args()
    client = BridgeClient()
    before = client.state()
    assert before["ready"] and not before["armed"], "start with a ready, disarmed simulator"
    # Reject the whole mission before the takeoff step can start.
    try:
        client.request("/mission", {"steps": [{"action": "takeoff", "altitude": 2},
                                              {"action": "goto", "x": 31, "y": 0, "altitude": 2}]})
    except RuntimeError:
        pass
    else:
        raise AssertionError("out-of-bounds mission was accepted")
    rejected = client.state()
    assert not rejected["armed"] and rejected["mission"] == before["mission"]
    if args.control:
        try:
            control_test(client)
        finally:
            if client.state()["armed"]:
                client.wait_for(client.request("/land", {}))
        return
    steps = [{"action": "takeoff", "altitude": 2},
             {"action": "goto", "x": 2, "y": 0, "altitude": 2},
             {"action": "goto", "x": 2, "y": 2, "altitude": 2},
             {"action": "hold", "duration": 1},
             {"action": "return_home"}, {"action": "land"}]
    result = client.execute(steps, "Deterministic flight smoke test",
                            lambda m: print("step", m["step"], m.get("action"), flush=True))
    mission = result["mission"]
    assert mission["status"] == "succeeded", mission
    assert not result["armed"] and result["landed_state"] == 1
    assert [e["action"] for e in mission["events"]] == [s["action"] for s in steps]
    for step, event in zip(steps, mission["events"]):
        if step["action"] == "goto":
            expected = [step["x"], step["y"], step["altitude"]]
            assert math.dist(expected, event["position_enu_m"]) <= 0.3
    output = Path(__file__).resolve().parent.parent / "logs" / "smoke_result.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print("PASS: preflight rejection, takeoff, waypoints, hold, return home, landing and disarming")


if __name__ == "__main__":
    main()
