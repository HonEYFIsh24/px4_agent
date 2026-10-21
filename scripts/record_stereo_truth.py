#!/usr/bin/env python3
"""Independent, read-only stereo/vehicle truth evidence; default is a plan.

Only --record constructs ROS subscribers. No capture, model or flight requests
are made. Gazebo ModelStates has no source header: its receipt-clock brackets
are approximate associations, never exact image-time ground truth.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import threading
import time
from urllib.parse import urlsplit

# Direct-script imports need the project root, without leaving sys.path changed.
_original_path = list(sys.path)
try:
    if __package__ in (None, ""):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from scripts.test_stereo_search import LocalSimulationOwner, require, write_once
    from scripts.test_stereo_positive import paired_archive_contract
    from scripts.evaluate_scene_boxes import jpeg_dimensions
    from scripts.test_ev_shadow import GcRecordingSession
    from stereo_localization import finite, vector, matvec, quaternion_rotation, iris_left_optical_to_body
finally:
    sys.path[:] = _original_path

MAX_RECORDS = 200000
MAX_BRACKET_SIM_S = .05
TIME_LIMITATION = ("ModelStates has no source header. ROS-clock receipt samples bracket the actual "
                   "image source stamp; interpolation is approximate and does not establish "
                   "simultaneous Gazebo publication or bound transport/scheduling latency.")


def local_ros_master(uri):
    endpoint = urlsplit(uri)
    require(endpoint.scheme == "http" and endpoint.hostname in ("127.0.0.1", "localhost", "::1")
            and endpoint.port == 11311 and endpoint.username is None and endpoint.password is None
            and endpoint.path in ("", "/") and not endpoint.query and not endpoint.fragment,
            "Independent recording requires the owned local ROS master on port 11311")
    return "http://%s:11311" % ("[%s]" % endpoint.hostname if endpoint.hostname == "::1" else endpoint.hostname)


def _pose(pose):
    p, q = pose.position, pose.orientation
    value = {"position_m": [p.x, p.y, p.z], "orientation_xyzw": [q.x, q.y, q.z, q.w]}
    vector(value["position_m"], 3, "body position")
    quaternion_rotation(value["orientation_xyzw"])
    return value


def model_states_body(message, receipt_sim_s, model_name="iris"):
    """Never invent a source stamp from ModelStates receipt time."""
    if model_name not in message.name:
        return None
    index = message.name.index(model_name)
    require(message.name.count(model_name) == 1 and index < len(message.pose), "Ambiguous Gazebo body model")
    return {"kind": "gazebo_body_truth", "topic": "/gazebo/model_states", "model_name": model_name,
            "frame_id": "gazebo_world", "body_reference": "base_link",
            "body_reference_assumption": "current Iris SDF model origin coincides with base_link",
            "source_stamp_sim_s": None, "source_stamp_ns": None,
            "receipt_ros_clock_sim_s": finite(receipt_sim_s, "truth receipt clock"),
            "timestamp_semantics": "receipt_ros_clock_no_source_header", **_pose(message.pose[index])}


def local_pose_row(message, receipt_sim_s):
    stamp = message.header.stamp
    ns = stamp.to_nsec() if hasattr(stamp, "to_nsec") else None
    require(ns is None or (type(ns) is int and ns >= 0), "Invalid actual pose nanosecond stamp")
    source = finite(stamp.to_sec(), "actual pose source stamp")
    require(source >= 0 and (ns is None or abs(source-ns/1e9) <= 1e-6), "Actual pose header stamps disagree")
    return {"kind": "local_body_pose", "topic": "/mavros/local_position/pose",
            "source_stamp_sim_s": source, "source_stamp_ns": ns,
            "receipt_ros_clock_sim_s": finite(receipt_sim_s, "pose receipt clock"),
            "timestamp_semantics": "actual_PoseStamped_header", "frame_id": message.header.frame_id,
            "body_reference": "base_link", **_pose(message.pose)}


def receipt_truth_bracket(rows, image_source_sim_s, maximum_gap_s=MAX_BRACKET_SIM_S):
    """Strict receipt-time bracketing; no extrapolation, sorting or clock repair."""
    source = finite(image_source_sim_s, "actual image source stamp")
    gap_limit = finite(maximum_gap_s, "truth bracket limit")
    require(0 < gap_limit <= MAX_BRACKET_SIM_S, "Truth bracket cannot exceed .05 simulation seconds")
    samples = [r for r in rows if r.get("kind") == "gazebo_body_truth"]
    previous = None
    for sample in samples:
        require(sample.get("source_stamp_sim_s") is None and sample.get("source_stamp_ns") is None
                and sample.get("timestamp_semantics") == "receipt_ros_clock_no_source_header",
                "ModelStates receipt must not be represented as an actual source stamp")
        stamp = finite(sample["receipt_ros_clock_sim_s"], "truth receipt clock")
        require(previous is None or stamp >= previous, "Truth receipt clock is nonmonotonic")
        previous = stamp
        vector(sample["position_m"], 3, "truth position")
        quaternion_rotation(sample["orientation_xyzw"])
    before = next((s for s in reversed(samples) if s["receipt_ros_clock_sim_s"] < source), None)
    after = next((s for s in samples if s["receipt_ros_clock_sim_s"] > source), None)
    require(before is not None and after is not None, "No strict truth receipt samples on both sides of image source")
    require(all(sum(s["receipt_ros_clock_sim_s"] == endpoint["receipt_ros_clock_sim_s"] for s in samples) == 1
                for endpoint in (before, after)), "Ambiguous duplicate truth receipt bracket endpoint")
    gap = after["receipt_ros_clock_sim_s"] - before["receipt_ros_clock_sim_s"]
    require(gap <= gap_limit + 1e-12, "Truth receipt bracket exceeds .05 simulation seconds")
    fraction = (source - before["receipt_ros_clock_sim_s"]) / gap
    p = [a + fraction*(b-a) for a, b in zip(before["position_m"], after["position_m"])]
    qa, qb = list(before["orientation_xyzw"]), list(after["orientation_xyzw"])
    # Preserve raw endpoints above, normalize each separately so tiny valid
    # quaternion norm differences cannot change the interpolation weight.
    qa = [x/math.sqrt(sum(y*y for y in qa)) for x in qa]
    qb = [x/math.sqrt(sum(y*y for y in qb)) for x in qb]
    if sum(a*b for a, b in zip(qa, qb)) < 0:
        qb = [-q for q in qb]
    q = [a + fraction*(b-a) for a, b in zip(qa, qb)]
    norm = math.sqrt(sum(x*x for x in q)); require(norm > 0, "Invalid interpolated truth quaternion")
    q = [x/norm for x in q]
    return {"status": "approximate_receipt_bracket", "image_source_stamp_sim_s": source,
            "before": copy.deepcopy(before), "after": copy.deepcopy(after), "gap_sim_s": gap,
            "fraction": fraction, "maximum_bracket_gap_sim_s": gap_limit,
            "body_pose_world_approximate": {"position_m": p, "orientation_xyzw": q,
                                            "frame_id": "gazebo_world", "body_reference": "base_link"},
            "interpolation": "linear_position_and_shortest_arc_normalized_linear_quaternion",
            "exact_source_time_ground_truth": False, "limitation": TIME_LIMITATION}


def actual_enu_neighbors(rows, source):
    """No sorting, reset repair or interpolation; preserve actual header samples."""
    nearby = []
    for sample in rows:
        if sample.get("kind") != "local_body_pose":
            continue
        stamp = finite(sample.get("source_stamp_sim_s"), "actual ENU source stamp")
        require(sample.get("timestamp_semantics") == "actual_PoseStamped_header"
                and sample.get("frame_id") in ("map", "local_enu") and sample.get("body_reference") == "base_link",
                "Actual ENU sample frame/header evidence is invalid")
        if abs(stamp-source) <= .05:
            nearby.append(copy.deepcopy(sample))
    return nearby


def optical_surface_to_world(optical_xyz, truth_body_pose):
    """Body-GT transform only; never estimate an ENU/world gauge from first yaw."""
    require(truth_body_pose.get("frame_id") == "gazebo_world"
            and truth_body_pose.get("body_reference") == "base_link", "Optical/world transform requires body GT")
    mounting = iris_left_optical_to_body()
    body_point = [a+b for a, b in zip(matvec(mounting["rotation"], vector(optical_xyz, 3, "optical surface")),
                                    mounting["translation_m"])]
    return [a+b for a, b in zip(matvec(quaternion_rotation(truth_body_pose["orientation_xyzw"]), body_point),
                               vector(truth_body_pose["position_m"], 3, "world body position"))]


class RosTruthObservations:
    """Light queue-1 subscriptions; JPEG data is read from captured pairs only."""
    def __init__(self):
        import rospy
        from gazebo_msgs.msg import ModelStates
        from geometry_msgs.msg import PoseStamped
        from rosgraph_msgs.msg import Clock
        self.rospy = rospy
        self.lock = threading.Lock()
        self.events, self.pending = [], []
        self.error = None
        self.clock_messages, self.last_clock_sim_s = 0, None
        self.recording_session = GcRecordingSession()
        rospy.init_node("stereo_independent_truth", anonymous=True, disable_signals=True)

        def callback(convert, message):
            try:
                row = convert(message, rospy.Time.now().to_sec())
                if row is not None:
                    self.add(row)
            except (ValueError, TypeError, KeyError, IndexError, AttributeError) as exc:
                self.error = "Invalid actual truth/pose callback: " + type(exc).__name__ + ": " + str(exc)

        def clock(message):
            try:
                source = finite(message.clock.to_sec(), "actual clock")
                with self.lock:
                    self.clock_messages += 1
                    self.last_clock_sim_s = source
            except (TypeError, ValueError) as exc:
                self.error = "Invalid actual clock callback: " + str(exc)

        subscribe = lambda *a, **kw: self.recording_session.subscribe(rospy.Subscriber, *a, **kw)
        try:
            subscribe("/clock", Clock, clock, queue_size=1, buff_size=4096, tcp_nodelay=True)
            subscribe("/gazebo/model_states", ModelStates, lambda m: callback(model_states_body, m),
                      queue_size=1, tcp_nodelay=True)
            subscribe("/mavros/local_position/pose", PoseStamped, lambda m: callback(local_pose_row, m),
                      queue_size=1, tcp_nodelay=True)
            self.recording_session.start()
        except BaseException:
            self.recording_session.close()
            raise

    def add(self, row):
        row = {**row, "receipt_wall_s": time.time(), "receipt_monotonic_s": time.monotonic()}
        # Validate serialization now, rather than allowing a late flush failure.
        json.dumps(row, allow_nan=False)
        with self.lock:
            if len(self.events) >= MAX_RECORDS:
                self.error = "Independent callback record limit reached; no silent sample dropping"
                return
            self.events.append(row); self.pending.append(row)

    def flush(self, stream):
        with self.lock:
            pending, self.pending = self.pending, []
        for row in pending:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
        stream.flush()

    def snapshot(self):
        with self.lock:
            return list(self.events)

    def evidence(self):
        with self.lock:
            return {"record_count": len(self.events), "clock_messages": self.clock_messages,
                    "last_clock_sim_s": self.last_clock_sim_s,
                    "gc_recording_session": self.recording_session.evidence()}

    def close(self):
        self.recording_session.close()


def _read_bounded(path, limit):
    require(not path.is_symlink() and path.is_file(), "Archive evidence must be a regular owned capture file")
    with path.open("rb") as handle:
        data = handle.read(limit+1)
    require(0 < len(data) <= limit, "Capture archive exceeded its bounded file size")
    return data


def capture_identifiers(directory):
    if not directory.exists():
        return set()
    require(directory.is_dir() and not directory.is_symlink(), "Unsafe owned capture directory")
    return {p.stem for p in directory.glob("*.json") if re.fullmatch(r"[a-f0-9]{32}", p.stem)}


def archive_pair(identifier, owner, destination):
    """Read existing exact-source bytes once; this function never requests a frame."""
    require(isinstance(identifier, str) and re.fullmatch(r"[a-f0-9]{32}", identifier), "Unsafe capture ID")
    owner.check()
    source = Path(owner.manager["run_directory"]) / "captures"
    raw_manifest = _read_bounded(source/(identifier+".json"), 1024*1024)
    metadata = json.loads(raw_manifest)
    require(metadata.get("capture_id") == identifier and b"image_base64" not in raw_manifest,
            "Unexpected capture manifest or embedded image data")
    if not metadata.get("stereo_evidence"):
        owner.check()
        return {"kind": "captured_pair_unavailable", "capture_id": identifier,
                "reason": "Existing capture has no paired right-JPEG provenance; no fabricated source evidence"}
    pair = paired_archive_contract(metadata, identifier)
    files = {".json": raw_manifest}
    hashes = {}
    for side, suffix in (("left", ".jpg"), ("right", ".right.jpg")):
        raw = _read_bounded(source/(identifier+suffix), 2*1024*1024)
        require(jpeg_dimensions(raw) == (640, 480), "Actual archived JPEG dimension mismatch")
        hashes[side] = hashlib.sha256(raw).hexdigest()
        files[suffix] = raw
    require(hashes == pair["image_sha256"] and metadata["camera"]["image_sha256"] == hashes["left"],
            "Actual captured-pair bytes differ from advertised source hashes")
    owner.check()
    for suffix, raw in files.items():
        with (destination/(identifier+suffix)).open("xb") as handle:
            handle.write(raw)
    return {"kind": "captured_stereo_pair", "capture_id": identifier, "pair_id": pair["pair_id"],
            "source_stamp_sim_s": pair["stamp_sim_s"], "source_stamp_ns": None,
            "source_stamp_precision": "actual_Image_header_to_sec_archived; nanoseconds not archived",
            "timestamp_semantics": "actual_ROS_Image_header_same_left_right_and_CameraInfo",
            "verified_image_sha256": hashes, "manifest_sha256": hashlib.sha256(raw_manifest).hexdigest(),
            "stereo_evidence": copy.deepcopy(pair), "archive_subdirectory": "captures",
            "source": "pinned_manager_existing_capture_archive_no_new_capture_request"}


def record(output, duration=120., poll=.5, max_captures=16, base="http://127.0.0.1:8080", px4_directory=None,
           ownership_factory=None, collector_factory=None, monotonic=None, sleep=None):
    """Bounded read-only session; failed ownership closes subscribers, never lands."""
    duration, poll = finite(duration, "recording duration"), finite(poll, "poll interval")
    require(0 < duration <= 1800 and .1 <= poll <= 1., "Recording duration/poll exceed bounded limits")
    require(type(max_captures) is int and 1 <= max_captures <= 32, "Capture budget must be 1..32")
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    clock, sleeper = monotonic or time.monotonic, sleep or time.sleep
    owner, collector, captures = None, None, []
    result = {"schema_version": 1, "recording_completed": False, "read_only": True,
              "actual_model_requests": False, "actual_control_requests": False, "new_capture_requests": False,
              "ground_truth_exposed_to_agent": False, "enu_to_world_gauge_estimated": False,
              "metric_accuracy_ground_truth_verified": False, "time_limitation": TIME_LIMITATION,
              "subscriptions": {"topics": ["/gazebo/model_states", "/mavros/local_position/pose", "/clock"],
                                "queue_size": 1, "tcp_nodelay": True, "image_topics": [],
                                "complete_callback_delivery_proven": False},
              "maximum_truth_bracket_gap_sim_s": MAX_BRACKET_SIM_S,
              "requested_duration_wall_s": duration, "max_captures": max_captures}
    started = clock()
    try:
        result["ros_master"] = local_ros_master(os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311"))
        owner = (ownership_factory or LocalSimulationOwner)(base, px4_directory)
        owner.check()
        source = Path(owner.manager["run_directory"])/"captures"
        seen = capture_identifiers(source)
        result["preexisting_capture_count_excluded"] = len(seen)
        (output/"captures").mkdir(exist_ok=False)
        owner.check()  # Check again immediately before creating subscribers.
        collector = (collector_factory or RosTruthObservations)()
        with (output/"observations.jsonl").open("x") as stream, (output/"captured_pairs.jsonl").open("x") as pairs_stream:
            def capture_new_manifests():
                owner.check()
                identifiers = capture_identifiers(source)-seen
                require(len(captures)+len(identifiers) <= max_captures, "Capture evidence budget exceeded")
                for identifier in sorted(identifiers):
                    owner.check()
                    row = archive_pair(identifier, owner, output/"captures")
                    captures.append(row); seen.add(identifier)
                    pairs_stream.write(json.dumps(row, allow_nan=False)+"\n"); pairs_stream.flush()

            while clock()-started < duration:
                owner.check()
                require(not collector.error, collector.error or "Invalid recorder callback")
                collector.flush(stream)
                capture_new_manifests()
                sleeper(min(poll, max(0., duration-(clock()-started))))
            owner.check()
            capture_new_manifests()  # Retain a manifest committed during the final polling interval.
            collector.close()
            collector.flush(stream)
            require(not collector.error, collector.error or "Invalid recorder callback")
            require(not collector.evidence().get("gc_recording_session", {}).get("cleanup_errors"),
                    "Independent subscriptions could not all be unregistered")
            result["recording_completed"] = True
    except (Exception, KeyboardInterrupt) as exc:
        result["error"] = {"type": type(exc).__name__, "message": str(exc)}
    finally:
        if collector is not None:
            collector.close()
            # Keep the last callbacks even when ownership or another gate fails.
            with (output/"observations.jsonl").open("a") as stream:
                collector.flush(stream)
            result["collector"] = collector.evidence()
        if owner is not None:
            result["ownership"] = owner.evidence()
        result["elapsed_wall_s"] = max(0., clock()-started)
    rows = collector.snapshot() if collector is not None else []
    associations = []
    for capture in captures:
        if capture["kind"] != "captured_stereo_pair":
            continue
        association = {"capture_id": capture["capture_id"], "image_sha256": capture["verified_image_sha256"],
                       "image_source_stamp_sim_s": capture["source_stamp_sim_s"],
                       "body_pose_enu_from_exact_capture": capture["stereo_evidence"]["body_pose_enu"],
                       "coordinate_epoch": capture["stereo_evidence"]["coordinate_epoch"]}
        try:
            association["truth"] = receipt_truth_bracket(rows, capture["source_stamp_sim_s"])
        except (ValueError, KeyError, TypeError) as exc:
            association["truth"] = {"status": "unavailable", "reason": str(exc),
                                     "exact_source_time_ground_truth": False}
        # Preserve raw ENU samples for independent analysis; do not interpolate
        # across unknown EKF resets or derive any ENU/world coordinate transform.
        try:
            association["nearby_actual_enu_source_samples"] = actual_enu_neighbors(rows, capture["source_stamp_sim_s"])
        except (ValueError, KeyError, TypeError) as exc:
            association["nearby_actual_enu_source_samples"] = []
            association["enu_evidence_error"] = str(exc)
        associations.append(association)
    write_once(output/"associations.json", {"captures": associations, "limitation": TIME_LIMITATION,
                                         "enu_world_transform_estimated": False})
    result["captured_pairs"] = len(associations)
    result["capture_without_pair_count"] = sum(c["kind"] != "captured_stereo_pair" for c in captures)
    result["approximate_truth_brackets_available"] = sum(a["truth"]["status"] == "approximate_receipt_bracket"
                                                        for a in associations)
    result["all_captured_pairs_have_receipt_brackets"] = bool(associations) and all(
        a["truth"]["status"] == "approximate_receipt_bracket" for a in associations)
    write_once(output/"validation.json", result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--duration", type=float, default=120.)
    parser.add_argument("--poll", type=float, default=.5)
    parser.add_argument("--max-captures", type=int, default=16)
    parser.add_argument("--base-url", default="http://127.0.0.1:8080")
    parser.add_argument("--px4-directory")
    args = parser.parse_args(argv)
    if not args.record:
        print(json.dumps({"actual_requests": False, "ros_subscriptions_started": False, "default_plan": True,
                          "topics": ["/gazebo/model_states", "/mavros/local_position/pose", "/clock"],
                          "image_topic_subscriptions": [], "pair_source": "existing owned manager capture archives",
                          "duration_wall_s": args.duration, "max_captures": args.max_captures,
                          "maximum_truth_bracket_gap_sim_s": MAX_BRACKET_SIM_S,
                          "time_limitation": TIME_LIMITATION, "no_control_or_model_calls": True,
                          "ground_truth_exposed_to_agent": False, "enu_to_world_gauge_estimated": False}, indent=2))
        return 0
    if args.output is None:
        parser.error("--record requires a new exclusive --output directory")
    result = record(args.output, args.duration, args.poll, args.max_captures, args.base_url, args.px4_directory)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result["recording_completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
