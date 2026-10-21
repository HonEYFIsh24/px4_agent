#!/usr/bin/env python3
"""Read-only, bounded ROS1 bag analysis; no ROS imports until --bag is given.

Image headers retain integer nanoseconds. LinkStates/ModelStates have NO source
header: bag recorder times are receipt associations, never render-time poses.
"""
import argparse
from collections import OrderedDict
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import struct
import zlib
import xml.etree.ElementTree as ET

TOPICS = {
    "left": "/uav/camera/left/image_raw", "right": "/uav/camera/right/image_raw",
    "imu": "/uav/imu/data_raw", "links": "/gazebo/link_states",
    "models": "/gazebo/model_states", "clock": "/clock",
    "native": "/vins_estimator/native_health",
}
ROOT = Path(__file__).resolve().parents[1]
MAX_RECORDS = 100000
MAX_IMAGE_BYTES = 16 * 1024 * 1024
MAX_PENDING_BYTES = 16 * 1024 * 1024
BRACKET_NS = 50000000
ACTUAL_QUATERNION_NORM_TOLERANCE = .025
LIMITATION = ("LinkStates/ModelStates have no source header. Recorded rosbag time is a recorder "
              "receipt association, not sensor/render/source pose time. A <=.05 sim-second "
              "receipt bracket does not bound transport latency or prove exact render-time pose.")


def require(value, message):
    if not value:
        raise ValueError(message)


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), "Nonfinite or nonnumeric value")
    return float(value)


def xyz(values):
    require(len(values) == 3, "Position needs three components")
    return [number(v) for v in values]


def stamp_ns(stamp):
    """Never round to_sec() back to nanoseconds; require actual ROS integer fields."""
    sec, ns = getattr(stamp, "secs", None), getattr(stamp, "nsecs", None)
    require(type(sec) is int and type(ns) is int and sec >= 0 and 0 <= ns < 1000000000,
            "Actual ROS stamp must have nonnegative integer secs/nsecs")
    return sec * 1000000000 + ns


def seconds_ns(value):
    scaled = Decimal(str(value)) * 1000000000
    require(scaled.is_finite() and scaled >= 0 and scaled == scaled.to_integral_value(),
            "Window must use nonnegative seconds with at most nanosecond precision")
    return int(scaled)


def qnorm(q):
    require(len(q) == 4, "Quaternion needs four components")
    q = [number(v) for v in q]
    norm = math.hypot(*q)
    require(math.isfinite(norm) and norm > 1e-12, "Invalid quaternion norm")
    return [v / norm for v in q]


def qmul(a, b):
    x, y, z, w = a; X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def qinv(q):
    return [-q[0], -q[1], -q[2], q[3]]


def rotate(q, v):
    return qmul(qmul(q, list(v)+[0.]), qinv(q))[:3]


def rpy_quaternion(rpy):
    r, p, y = [number(v)/2 for v in rpy]
    return qnorm([math.sin(r)*math.cos(p)*math.cos(y)-math.cos(r)*math.sin(p)*math.sin(y),
                  math.cos(r)*math.sin(p)*math.cos(y)+math.sin(r)*math.cos(p)*math.sin(y),
                  math.cos(r)*math.cos(p)*math.sin(y)-math.sin(r)*math.sin(p)*math.cos(y),
                  math.cos(r)*math.cos(p)*math.cos(y)+math.sin(r)*math.sin(p)*math.sin(y)])


def pose_value(message):
    p, q = message.position, message.orientation
    raw = [number(v) for v in (q.x, q.y, q.z, q.w)]
    norm = math.hypot(*raw)
    require(math.isfinite(norm) and abs(norm-1.) <= ACTUAL_QUATERNION_NORM_TOLERANCE,
            "Actual Gazebo quaternion norm is not near unit")
    return {"position_m": [number(p.x), number(p.y), number(p.z)],
            "orientation_xyzw": qnorm(raw), "actual_orientation_raw_xyzw": raw,
            "actual_orientation_raw_norm": norm}


def relative_pose(base, child):
    qb, qc = qnorm(base["orientation_xyzw"]), qnorm(child["orientation_xyzw"])
    delta = [c-b for b, c in zip(xyz(base["position_m"]), xyz(child["position_m"]))]
    return {"position_m": rotate(qinv(qb), delta), "orientation_xyzw": qnorm(qmul(qinv(qb), qc))}


def pose_error(actual, expected):
    position = math.dist(xyz(actual["position_m"]), xyz(expected["position_m"]))
    dq = qnorm(qmul(qinv(qnorm(expected["orientation_xyzw"])), qnorm(actual["orientation_xyzw"])))
    angle = 2*math.atan2(math.hypot(*dq[:3]), abs(dq[3]))
    return {"translation_error_m": position, "rotation_error_rad": angle}


def camera_center(camera, offset):
    shifted = rotate(qnorm(camera["orientation_xyzw"]), xyz(offset))
    return {"position_m": [a+b for a, b in zip(camera["position_m"], shifted)],
            "orientation_xyzw": list(camera["orientation_xyzw"])}


def compose_pose(parent, relative):
    shifted=rotate(qnorm(parent["orientation_xyzw"]),xyz(relative["position_m"]))
    return {"position_m":[a+b for a,b in zip(xyz(parent["position_m"]),shifted)],
            "orientation_xyzw":qnorm(qmul(qnorm(parent["orientation_xyzw"]),qnorm(relative["orientation_xyzw"])))}


def configured_pose(element):
    pose=element.find("pose")
    require(pose is None or not pose.get("relative_to"),"Unsupported explicit SDF pose frame")
    values=[number(float(v)) for v in (pose.text if pose is not None else "0 0 0 0 0 0").split()]
    require(len(values)==6,"Configured pose needs xyz/rpy")
    return {"position_m":values[:3],"orientation_xyzw":rpy_quaternion(values[3:])}


def sdf_configuration(path):
    raw = Path(path).read_bytes()
    root = ET.fromstring(raw); models = root.findall("model")
    require(len(models) == 1, "SDF must identify one actual vehicle model")
    model = models[0]
    def configured_link(name):
        links = [l for l in model.findall("link") if l.get("name", "").lstrip("/") == name]
        require(len(links) == 1, "Missing/ambiguous configured " + name)
        return links[0],configured_pose(links[0])
    base_link, base = configured_link("base_link")
    _, imu = configured_link("imu_link")
    mounts=[(link,sensor) for link in model.findall("link") for sensor in link.findall("sensor")
            if sensor.get("type")=="multicamera"]
    require(len(mounts)==1,"Missing/ambiguous actual multicamera mount")
    link,sensor=mounts[0];parent=link.get("name","").lstrip("/")
    require(parent in ("vio_camera_link","base_link"),"Unsupported multicamera parent")
    if parent=="vio_camera_link":
        joints=[j for j in model.findall("joint") if j.findtext("child","").lstrip("/")==parent]
        require(len(joints)==1 and joints[0].get("type")=="fixed"
                and joints[0].findtext("parent","").lstrip("/")=="base_link","Camera physics link requires actual fixed base joint")
        sensor_mount=configured_pose(sensor)
        require(pose_error(sensor_mount,{"position_m":[0.,0.,0.],"orientation_xyzw":[0.,0.,0.,1.]})["translation_error_m"]<1e-12
                and pose_error(sensor_mount,{"position_m":[0.,0.,0.],"orientation_xyzw":[0.,0.,0.,1.]})["rotation_error_rad"]<1e-12,
                "Legacy measured camera link requires sensor at actual link origin")
        camera=relative_pose(base,compose_pose(configured_pose(link),configured_pose(sensor)))
        topology="physics_camera_link_fixed_joint"
    else:
        require(not any(l.get("name","").lstrip("/")=="vio_camera_link" for l in model.findall("link")),
                "Direct sensor configuration still contains obsolete camera physics link")
        require(not any(j.findtext("child","").lstrip("/")=="vio_camera_link" for j in model.findall("joint")),
                "Direct sensor configuration still contains obsolete camera physics joint")
        camera=configured_pose(sensor);topology="sensor_direct_parent_base_link"
    camera_members=sensor.findall("camera")
    cameras = {c.get("name"): c for c in camera_members}
    require(len(camera_members)==2 and set(cameras) == {"left", "right"}, "Expected unambiguous actual left/right multicamera configuration")
    def sensor_pose(name):
        values = [float(v) for v in cameras[name].findtext("pose", "0 0 0 0 0 0").split()]
        require(len(values) == 6 and all(math.isfinite(v) for v in values), "Invalid sensor pose")
        return values
    left, right = sensor_pose("left"), sensor_pose("right")
    require(all(abs(v) < 1e-12 for v in left) and all(abs(v) < 1e-12 for v in right[3:]),
            "Rig-center definition requires left at link origin and parallel right sensor")
    offset = [v/2 for v in right[:3]]
    imu = relative_pose(base, imu)
    inertial=base_link.find("inertial")
    require(inertial is not None,"Missing actual base inertial configuration")
    mass=number(float(inertial.findtext("mass")));require(mass>0,"Invalid actual base mass")
    inertia={key:number(float(inertial.findtext("inertia/"+key))) for key in ("ixx","iyy","izz","ixy","ixz","iyz")}
    return {"model_sdf": str(Path(path).resolve()), "model_sdf_sha256": hashlib.sha256(raw).hexdigest(),
            "model_name": model.get("name"), "camera_mount_topology":topology,"camera_parent_link":parent,
            "camera_sensor_name":sensor.get("name"),"camera_sensor_expected":camera,
            "camera_link_expected":camera if parent=="vio_camera_link" else None,"imu_link_expected": imu,
            "rig_center_offset_camera_link_m": offset, "rig_center_expected": camera_center(camera, offset),
            "base_inertial_configuration":{"mass_kg":mass,"pose":configured_pose(inertial),"inertia_kg_m2":inertia},
            "expected_mount_semantics": "actual_SDF_geometry_not_measured_or_fitted"}


def resolve_link(message, model, suffix):
    names = getattr(message, "name", [])
    candidates = [i for i, n in enumerate(names) if n.startswith(model+"::")
                  and n[len(model)+2:].lstrip("/") == suffix]
    require(len(candidates) == 1, "Missing/ambiguous actual link " + suffix)
    require(len(message.pose) == len(names), "LinkStates name/pose arrays differ")
    i = candidates[0]
    return names[i], pose_value(message.pose[i])


def link_row(message, receipt_ns, config, model="iris"):
    require(type(receipt_ns) is int and receipt_ns >= 0, "Invalid recorder receipt stamp")
    name, base = resolve_link(message, model, "base_link")
    row={"status": "ok", "recorder_receipt_ns": receipt_ns, "source_stamp_ns": None,
            "timestamp_semantics": "rosbag_recorder_receipt_headerless_not_render_time",
            "base_link":name,"base_pose_world":base,"camera_relative_body":None,"camera_error":None,
            "rig_center_relative_body":None,"rig_center_error":None,"camera_link":None,
            "imu_relative_body":None,"imu_error":None,"imu_link":None,
            "exact_render_time_pose_proven":False}
    try:
        im,imu=resolve_link(message,model,"imu_link");imu=relative_pose(base,imu)
        row.update(imu_link=im,imu_relative_body=imu,imu_error=pose_error(imu,config["imu_link_expected"]),
                   imu_measurement={"status":"measured_same_LinkStates_message"})
    except ValueError as exc:
        row.update(status="partial",imu_measurement={"status":"unavailable","reason":str(exc)})
    direct=config.get("camera_mount_topology")=="sensor_direct_parent_base_link"
    if direct:
        require(not any(n.startswith(model+"::") and n[len(model)+2:].lstrip("/")=="vio_camera_link"
                        for n in message.name),"Actual LinkStates contradict direct-parent SDF topology")
        nominal=compose_pose(base,config["camera_sensor_expected"])
        row.update(camera_measurement={"status":"unavailable_by_direct_parent",
                   "reason":"Sensor has no independent physics camera link; configuration cannot measure camera rigidity"},
                   nominal_camera_world={"status":"configured_kinematics_only","pose_world":nominal,
                                         "measured_sensor_pose":False,"exact_render_time_pose_proven":False})
    else:
        try:
            cn,camera=resolve_link(message,model,"vio_camera_link");camera=relative_pose(base,camera)
            center=camera_center(camera,config["rig_center_offset_camera_link_m"])
            row.update(camera_link=cn,camera_relative_body=camera,rig_center_relative_body=center,
                       camera_error=pose_error(camera,config["camera_link_expected"]),
                       rig_center_error=pose_error(center,config["rig_center_expected"]),
                       camera_measurement={"status":"measured_same_LinkStates_message"})
        except ValueError as exc:
            row.update(status="partial",camera_measurement={"status":"unavailable","reason":str(exc)})
    return row


def receipt_bracket(rows, source_ns):
    """Preserve order; never label bag receipts as LinkStates source stamps."""
    require(type(source_ns) is int and source_ns >= 0, "Image source ns must be an integer")
    previous = None
    for row in rows:
        t = row["recorder_receipt_ns"]
        require(type(t) is int and t >= 0 and (previous is None or t >= previous), "Nonmonotonic bag receipts")
        require(row.get("source_stamp_ns") is None, "Headerless pose must not have an invented source stamp")
        previous = t
    before = next((r for r in reversed(rows) if r["recorder_receipt_ns"] < source_ns), None)
    after = next((r for r in rows if r["recorder_receipt_ns"] > source_ns), None)
    require(before is not None and after is not None, "Missing strict receipt bracket")
    require(all(sum(r["recorder_receipt_ns"] == e["recorder_receipt_ns"] for r in rows) == 1
                for e in (before, after)), "Duplicate receipt bracket endpoint")
    gap = after["recorder_receipt_ns"]-before["recorder_receipt_ns"]
    require(gap <= BRACKET_NS, "Receipt bracket exceeds .05 sim seconds / incomparable clock domains")
    return {"status": "approximate_receipt_bracket", "image_source_stamp_ns": source_ns,
            "before": before, "after": after, "gap_ns": gap,
            "exact_render_time_pose_proven": False, "limitation": LIMITATION}


def image_value(message, receipt_ns):
    h = message.header; ns = stamp_ns(h.stamp)
    width, height, step = message.width, message.height, message.step
    require(all(type(v) is int and v > 0 for v in (width, height, step))
            and width <= 4096 and height <= 4096, "Invalid/beyond-bounds image dimensions")
    enc = message.encoding
    channels = {"rgb8": 3, "bgr8": 3, "mono8": 1, "8UC1": 1}.get(enc)
    require(channels is not None, "Unsupported image encoding; no lossy conversion")
    data = bytes(message.data)
    require(step >= width*channels and len(data) == height*step and len(data) <= MAX_IMAGE_BYTES,
            "Invalid/beyond-bounds actual image layout")
    require(type(h.seq) is int and h.seq >= 0 and isinstance(h.frame_id, str), "Invalid actual image header")
    return {"source_stamp_ns": ns, "header": {"secs": h.stamp.secs, "nsecs": h.stamp.nsecs,
            "seq": h.seq, "frame_id": h.frame_id}, "recorder_receipt_ns": receipt_ns,
            "width": width, "height": height, "step": step, "encoding": enc,
            "is_bigendian": int(message.is_bigendian), "raw_data_sha256": hashlib.sha256(data).hexdigest()}, data


def png_bytes(metadata, data):
    """Lossless original pixel bytes; raw .bin also retains exact row padding."""
    width, height, step, enc = (metadata[k] for k in ("width", "height", "step", "encoding"))
    channels = 1 if enc in ("mono8", "8UC1") else 3
    rows = []
    for i in range(height):
        row = data[i*step:i*step+width*channels]
        if enc == "bgr8":
            row = bytes(v for j in range(0, len(row), 3) for v in (row[j+2], row[j+1], row[j]))
        rows.append(b"\x00"+row)
    def chunk(kind, payload):
        return struct.pack(">I",len(payload))+kind+payload+struct.pack(">I",zlib.crc32(kind+payload)&0xffffffff)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR",struct.pack(">IIBBBBB",width,height,8,0 if channels==1 else 2,0,0,0))
            + chunk(b"IDAT",zlib.compress(b"".join(rows))) + chunk(b"IEND",b""))


def save_json(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)


def jsonline(stream, value):
    stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False)+"\n")


def analysis(messages, output, config, start_ns, end_ns, model="iris", sample_every=5, max_pairs=32):
    """Single streaming pass. Image queues <=16MiB; metadata count/window bounded."""
    require(type(start_ns) is int and type(end_ns) is int and 0 <= start_ns < end_ns
            and end_ns-start_ns <= 30000000000, "Analysis window must be positive and <=30sim seconds")
    require(type(sample_every) is int and sample_every >= 1 and type(max_pairs) is int and 1 <= max_pairs <= 64,
            "Invalid bounded image sample budget")
    output = Path(output); output.mkdir(parents=True, exist_ok=False); (output/"images").mkdir()
    result = {"schema_version": 2, "read_only": True, "actual_model_requests": False,
              "actual_control_requests": False, "ground_truth_exposed_to_agent": False,
              "window_source_ns": [start_ns,end_ns], "configuration": config, "limitation": LIMITATION,
              "sample_every_exact_pair": sample_every, "maximum_saved_pairs": max_pairs,
              "topics_seen": {}, "errors": [], "unpaired_images": [], "pairs": 0, "saved_pairs": 0}
    links, models, pairs, pending, pending_bytes = [], [], [], OrderedDict(), 0
    initial={}
    image_stamps, clocks = {}, []
    streams = {name:(output/(name+".jsonl")).open("x") for name in ("images","mount_timeline","model_timeline","imu","clock","native")}
    maxima = {name:None for name in ("camera_translation_m","camera_rotation_rad","imu_translation_m","imu_rotation_rad")}
    count = 0
    try:
        for topic, message, receipt in messages:
            if topic not in TOPICS.values(): continue
            t = stamp_ns(receipt)
            image_side = next((side for side in ("left","right") if TOPICS[side] == topic), None)
            source_ns = stamp_ns(message.header.stamp) if image_side or topic == TOPICS["imu"] else t
            if not start_ns-100000000 <= source_ns <= end_ns+100000000: continue
            count += 1; require(count <= MAX_RECORDS, "Bounded metadata record limit exceeded")
            result["topics_seen"][topic] = result["topics_seen"].get(topic,0)+1
            if image_side:
                if not start_ns <= source_ns <= end_ns: continue
                require(source_ns > image_stamps.get(image_side,-1), "Duplicate/nonmonotonic actual image source stamp")
                image_stamps[image_side] = source_ns
                metadata, data = image_value(message,t); metadata["topic"] = topic
                jsonline(streams["images"],metadata)
                key = (source_ns,image_side)
                require(key not in pending, "Duplicate pending image source/side")
                other = (source_ns,"right" if image_side=="left" else "left")
                if other in pending:
                    previous, other_data = pending.pop(other); pending_bytes -= len(other_data)
                    members = {image_side:(metadata,data),other[1]:(previous,other_data)}
                    left, right = members["left"][0], members["right"][0]
                    require((left["width"],left["height"],left["encoding"]) == (right["width"],right["height"],right["encoding"]), "Pair layouts differ")
                    pair = {"source_stamp_ns":source_ns,"left":left,"right":right,"same_exact_header_stamp":True}
                    pair_index = result["pairs"];result["pairs"]+=1
                    if pair_index % sample_every == 0 and result["saved_pairs"] < max_pairs:
                        for side,(meta,raw) in members.items():
                            stem = "%019d_%s"%(source_ns,side); png = png_bytes(meta,raw)
                            for suffix,content in ((".bin",raw),(".png",png)):
                                with (output/"images"/(stem+suffix)).open("xb") as f:f.write(content)
                            meta.update(raw_filename=stem+".bin",png_filename=stem+".png",png_sha256=hashlib.sha256(png).hexdigest(),png_lossless=True)
                        result["saved_pairs"]+=1
                    pairs.append(pair)
                else:
                    pending[key] = (metadata,data);pending_bytes += len(data)
                    while pending_bytes > MAX_PENDING_BYTES or len(pending)>32:
                        old,(meta,raw)=pending.popitem(last=False);pending_bytes-=len(raw)
                        result["unpaired_images"].append({"source_stamp_ns":old[0],"side":old[1],"reason":"bounded_pending_queue_exhausted"})
            elif topic == TOPICS["links"]:
                try:
                    row = link_row(message,t,config,model)
                    for component in ("camera","imu"):
                        actual=row[component+"_relative_body"]
                        if actual is not None:
                            if component not in initial:initial[component]=actual
                            row[component+"_change_from_first"]=pose_error(actual,initial[component])
                        else:row[component+"_change_from_first"]=None
                    for name,part,field in (("camera_translation_m","camera_error","translation_error_m"),("camera_rotation_rad","camera_error","rotation_error_rad"),
                                            ("imu_translation_m","imu_error","translation_error_m"),("imu_rotation_rad","imu_error","rotation_error_rad")):
                        if row[part] is not None and (maxima[name] is None or row[part][field]>maxima[name]["value"]):
                            maxima[name]={"value":row[part][field],"recorder_receipt_ns":t}
                    links.append(row)
                except ValueError as exc: row={"status":"unavailable","recorder_receipt_ns":t,"source_stamp_ns":None,"reason":str(exc)}
                jsonline(streams["mount_timeline"],row)
            elif topic == TOPICS["models"]:
                indexes=[i for i,n in enumerate(message.name) if n==model]
                if len(indexes)==1 and len(message.pose)==len(message.name):
                    row={"status":"ok","recorder_receipt_ns":t,"source_stamp_ns":None,"model_name":model,"body_pose_world":pose_value(message.pose[indexes[0]])}
                    models.append(row)
                else:row={"status":"unavailable","recorder_receipt_ns":t,"source_stamp_ns":None,"reason":"Missing/ambiguous ModelStates body"}
                jsonline(streams["model_timeline"],row)
            elif topic == TOPICS["clock"]:
                row={"recorder_receipt_ns":t,"actual_clock_ns":stamp_ns(message.clock)}
                clocks.append(row);jsonline(streams["clock"],row)
            elif topic == TOPICS["imu"]:
                v=lambda a:[number(a.x),number(a.y),number(a.z)]
                jsonline(streams["imu"],{"source_stamp_ns":source_ns,"recorder_receipt_ns":t,"frame_id":message.header.frame_id,
                                         "linear_acceleration_m_s2":v(message.linear_acceleration),"angular_velocity_rad_s":v(message.angular_velocity)})
            elif topic == TOPICS["native"]:
                require(len(message.data)<=2*1024*1024,"Native message exceeds bounded size")
                jsonline(streams["native"],{"recorder_receipt_ns":t,"native":json.loads(message.data)})
    except Exception as exc:
        result["errors"].append({"type":type(exc).__name__,"message":str(exc)})
    finally:
        for stream in streams.values():stream.close()
    for (ns,side),_ in pending.items():result["unpaired_images"].append({"source_stamp_ns":ns,"side":side,"reason":"no_exact_source_partner"})
    with (output/"pairs.jsonl").open("x") as stream:
        for pair in pairs:
            nearby_clock = next((c for c in clocks if abs(c["actual_clock_ns"]-pair["source_stamp_ns"]) <= BRACKET_NS
                                 and abs(c["recorder_receipt_ns"]-pair["source_stamp_ns"]) <= BRACKET_NS), None)
            pair["nearby_actual_clock_evidence"] = nearby_clock
            for label,rows in (("link_receipt_bracket",links),("body_receipt_bracket",models)):
                try:
                    require(nearby_clock is not None,"Missing/incomparable nearby /clock evidence; bag receipt domain unverified")
                    pair[label]=receipt_bracket(rows,pair["source_stamp_ns"])
                except ValueError as exc:pair[label]={"status":"unavailable","reason":str(exc),"exact_render_time_pose_proven":False}
            jsonline(stream,pair)
    result["maximum_mount_errors"]=maxima
    result["availability"]={label:{"status":"available" if result["topics_seen"].get(topic) else "unavailable","topic":topic} for label,topic in TOPICS.items()}
    camera_available=any(r.get("camera_relative_body") is not None for r in links)
    imu_available=any(r.get("imu_relative_body") is not None for r in links)
    result["camera_relative_measurement"]={"status":"available" if camera_available else
        "unavailable_by_direct_parent" if config.get("camera_mount_topology")=="sensor_direct_parent_base_link" else "unavailable"}
    result["imu_relative_measurement"]={"status":"available" if imu_available else "unavailable"}
    result["relative_mount_pose_available"]=camera_available and imu_available
    result["imu_relative_pose_available"]=imu_available
    result["exact_pairs_available"]=bool(pairs)
    result["analysis_completed"]=not result["errors"]
    result["mount_rigidity_or_render_time_accuracy_passed"]=None
    save_json(output/"summary.json",result)
    return result


def failure_window(messages):
    for topic,message,_ in messages:
        if topic != TOPICS["native"]:continue
        try:r=json.loads(message.data)
        except (ValueError,TypeError):continue
        if r.get("tracking_ok") is False and r.get("reason")=="native_limits_or_visual_tracking_lost":
            source=number(r.get("stamp"));require(source>=0,"Invalid native failure stamp")
            return max(0,seconds_ns(str(source))-4000000000),seconds_ns(str(source))+1000000000
    raise ValueError("No actual native failure marker; supply explicit --start/--end")


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag",type=Path);parser.add_argument("--output",type=Path)
    parser.add_argument("--start",default="122");parser.add_argument("--end",default="126")
    parser.add_argument("--failure-window",action="store_true",help="Use first actual native lost marker minus4/plus1 seconds")
    parser.add_argument("--model-sdf",type=Path,default=ROOT/"vio/models/iris_vio/iris_vio.sdf")
    parser.add_argument("--model",default="iris");parser.add_argument("--sample-every",type=int,default=5)
    parser.add_argument("--max-pairs",type=int,default=32)
    args=parser.parse_args(argv)
    if args.bag is None:
        print(json.dumps({"usage":"--bag CLOSED.bag --output NEW_DIRECTORY [--failure-window | --start 122 --end 126]",
                          "read_only":True,"default_no_bag_read":True,"no_ROS_runtime_or_model_calls":True,"limitation":LIMITATION}));return 0
    if args.output is None:parser.error("--bag requires a new --output directory")
    require(args.bag.is_file() and not args.bag.is_symlink() and args.bag.suffix==".bag","Input must be a closed regular .bag file")
    before=args.bag.stat();config=sdf_configuration(args.model_sdf)
    import rosbag
    with rosbag.Bag(str(args.bag),"r") as bag:
        start,end=failure_window(bag.read_messages(topics=[TOPICS["native"]])) if args.failure_window else (seconds_ns(args.start),seconds_ns(args.end))
        result=analysis(bag.read_messages(topics=list(TOPICS.values())),args.output,config,start,end,args.model,args.sample_every,args.max_pairs)
    after=args.bag.stat()
    provenance={"bag":str(args.bag.resolve()),"size_bytes":before.st_size,"mtime_ns":before.st_mtime_ns,
                "input_stat_stable":(before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns),
                "window_selection":"first_actual_native_failure_minus4_plus1" if args.failure_window else "explicit_source_window",
                "original_bag_remains_source_evidence":True}
    save_json(args.output/"bag_provenance.json",provenance)
    print(json.dumps({"analysis_completed":result["analysis_completed"],"input_stat_stable":provenance["input_stat_stable"],
                      "exact_pairs":result["pairs"],"relative_mount_pose_available":result["relative_mount_pose_available"],"output":str(args.output)},allow_nan=False))
    return 0 if result["analysis_completed"] and provenance["input_stat_stable"] else 1


if __name__=="__main__":raise SystemExit(main())
