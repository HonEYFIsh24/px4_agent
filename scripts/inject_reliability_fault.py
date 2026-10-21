#!/usr/bin/python3
"""Set one explicitly enabled, expiring Synthetic local-SITL fault.

Run with the ROS Noetic environment sourced. This is an operator/test script,
never a language-model tool. It does not issue flight or landing commands.
"""
import argparse
import json
import sys
import time
import uuid
from pathlib import Path
from urllib.request import urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from reliability_faults import FAULT_KINDS, FAULT_PARAMETER, fault_request, validate_request


def state(base_url):
    with urlopen(base_url.rstrip("/") + "/state", timeout=3) as response:
        return json.load(response)


def require_test_simulation(snapshot, rospy):
    if snapshot.get("simulation_only") is not True:
        raise RuntimeError("refusing fault injection outside the local simulation")
    if (snapshot.get("reliability_test") or {}).get("enabled") is not True:
        raise RuntimeError("restart simulation with UAV_RELIABILITY_TESTING=true to enable test-only faults")
    if not rospy.get_param("/use_sim_time", False):
        raise RuntimeError("/use_sim_time must be true")
    if rospy.get_param("/mavros/fcu_url", "") != "udp://:14540@localhost:14557":
        raise RuntimeError("MAVROS must use the provided local SITL endpoint")
    from gazebo_msgs.srv import GetWorldProperties
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=3)
    world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
    if not world.success or "iris" not in world.model_names:
        raise RuntimeError("Gazebo must contain the local Iris model")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument("--kind", choices=FAULT_KINDS)
    operation.add_argument("--clear", action="store_true")
    parser.add_argument("--duration", type=float, default=2., help="wall seconds, .1..30, automatic recovery")
    parser.add_argument("--id", default=None)
    parser.add_argument("--range", dest="range_m", type=float, default=.7)
    parser.add_argument("--bearing", dest="bearing_rad", type=float, default=0., help="body-relative radians at activation")
    parser.add_argument("--delay", type=float, default=2., help="Synthetic measurement age in simulation seconds")
    parser.add_argument("--bridge-url", default="http://127.0.0.1:8765")
    args = parser.parse_args()
    # Read-only fail before ROS mutation if the bridge exposes no enabled hook.
    snapshot = state(args.bridge_url)
    import rospy
    rospy.init_node("uav_reliability_injector", anonymous=True, disable_signals=True)
    require_test_simulation(snapshot, rospy)
    request = {} if args.clear else fault_request(
        args.kind, args.duration, args.id or uuid.uuid4().hex,
        range_m=args.range_m, bearing_rad=args.bearing_rad, delay_s=args.delay)
    if request:
        validate_request(request, time.time())
    rospy.set_param(FAULT_PARAMETER, request)
    restarted = False
    if args.kind == "vio_reset":
        # Actual upstream VINS restart, rather than fabricating a good pose or
        # claiming a synthetic odometry offset is an estimator reset.
        from std_msgs.msg import Bool
        publisher = rospy.Publisher("/vins_restart", Bool, queue_size=1, latch=False)
        deadline = time.monotonic()+3.
        while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(.05)
        if publisher.get_num_connections() == 0:
            rospy.set_param(FAULT_PARAMETER, {})
            raise RuntimeError("VINS restart subscriber is unavailable; injection cleared")
        publisher.publish(Bool(data=True))
        time.sleep(.15)
        restarted = True
    deadline = time.monotonic()+2.
    current = None
    while time.monotonic() < deadline:
        current = state(args.bridge_url).get("reliability_test") or {}
        if args.clear and not current.get("active_fault") and not current.get("pending_fault"):
            break
        if request and (current.get("fault_id") == request["id"] or current.get("pending_fault") == request["kind"]):
            break
        time.sleep(.05)
    print(json.dumps({"synthetic": True, "simulation_only": True,
                      "parameter": FAULT_PARAMETER, "request": request,
                      "estimator_restart_published": restarted,
                      "current": current}, ensure_ascii=False, allow_nan=False))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(json.dumps({"status": "error", "error": str(exc), "synthetic": True}), file=sys.stderr)
        sys.exit(1)
