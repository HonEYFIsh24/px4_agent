#!/usr/bin/env python3
"""Offline projected-SDF-AABB comparison against one public VLM bounding box.

This evaluator is never an Agent tool. It reads saved evidence and the private
scene layout, makes no network/ROS/model requests, and does not modify runtime
configuration. Its rectangle IoU is not pixel segmentation or detection accuracy:
the SDF AABB includes visual/collision bounds, and occlusion is not modeled.
"""
import argparse
import hashlib
import itertools
import json
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NEAR_M = 0.01


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("Expected a finite numeric value")
    return float(value)


def vector(values, length):
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError("Expected a vector with {} components".format(length))
    return [finite(value) for value in values]


def normalized_box(values):
    box = vector(values, 4)
    if not (0 <= box[0] < box[2] <= 1 and 0 <= box[1] < box[3] <= 1):
        raise ValueError("Public predicted bbox must be normalized [left,top,right,bottom]")
    return box


def inverse_rotate(quaternion, point):
    x, y, z, w = vector(quaternion, 4)
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if abs(norm-1) > 0.025:
        raise ValueError("Camera optical/world quaternion is not normalized")
    x, y, z, w = -x/norm, -y/norm, -z/norm, w/norm
    vx, vy, vz = point
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return [vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx]


def convex_hull(points):
    points = sorted(set(tuple(point) for point in points))
    if len(points) <= 1:
        return points

    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])

    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1]+upper[:-1]


def clip_polygon(points, width, height):
    """Clip the projected convex AABB silhouette to the image rectangle."""
    result = list(points)
    for axis, boundary, keep_above in ((0, 0.0, True), (0, width, False),
                                        (1, 0.0, True), (1, height, False)):
        clipped = []
        if not result:
            return []
        previous = result[-1]
        previous_inside = previous[axis] >= boundary if keep_above else previous[axis] <= boundary
        for current in result:
            current_inside = current[axis] >= boundary if keep_above else current[axis] <= boundary
            if current_inside != previous_inside:
                scale = (boundary-previous[axis])/(current[axis]-previous[axis])
                clipped.append(tuple(previous[index]+scale*(current[index]-previous[index]) for index in (0, 1)))
            if current_inside:
                clipped.append(tuple(current))
            previous, previous_inside = current, current_inside
        result = clipped
    return result


def polygon_box(points):
    if len(points) < 3:
        return None
    box = [min(point[0] for point in points), min(point[1] for point in points),
           max(point[0] for point in points), max(point[1] for point in points)]
    return box if box[0] < box[2] and box[1] < box[3] else None


def project_aabb(aabb, snapshot):
    if snapshot.get("location_frame") != "gazebo_world":
        raise ValueError("Saved camera pose is not in Gazebo world coordinates")
    pose = snapshot["camera_pose_world"]
    if pose.get("optical_axes") != "x_right_y_down_z_forward":
        raise ValueError("Saved camera pose is not a documented optical frame")
    if not 0 <= finite(pose["time_alignment_error_s"]) <= .05:
        raise ValueError("Saved image and camera pose do not satisfy the alignment contract")
    intrinsics = snapshot["camera_intrinsics"]
    if any(abs(finite(value)) > 1e-9 for value in intrinsics.get("distortion_coefficients", intrinsics.get("D", []))):
        raise ValueError("Nonzero lens distortion is not supported by the offline pinhole evaluator")
    width, height = finite(snapshot["width"]), finite(snapshot["height"])
    fx, fy, cx, cy = [finite(intrinsics[key]) for key in ("fx", "fy", "cx", "cy")]
    if min(width, height, fx, fy) <= 0:
        raise ValueError("Saved camera dimensions/focal lengths are invalid")
    origin = vector(pose["position_m"], 3)
    bounds = [vector(aabb[axis], 2) for axis in ("x", "y", "z")]
    if any(low >= high for low, high in bounds):
        raise ValueError("SDF AABB has invalid bounds")
    corners = [inverse_rotate(pose["orientation_xyzw"], [value-origin[index] for index, value in enumerate(point)])
               for point in itertools.product(*bounds)]
    # Intersections of the twelve box edges with the positive-depth near plane.
    positive = [point for point in corners if point[2] >= NEAR_M]
    for first in range(8):
        for bit in (1, 2, 4):
            second = first ^ bit
            if second <= first:
                continue
            a, b = corners[first], corners[second]
            if (a[2] >= NEAR_M) != (b[2] >= NEAR_M):
                fraction = (NEAR_M-a[2])/(b[2]-a[2])
                positive.append([a[index]+fraction*(b[index]-a[index]) for index in range(3)])
    hull = convex_hull([(fx*point[0]/point[2]+cx, fy*point[1]/point[2]+cy) for point in positive])
    raw = polygon_box(hull)
    clipped = polygon_box(clip_polygon(hull, width, height))
    return {"status": "projected" if clipped else "outside_current_field_of_view",
            "bbox_pixels": clipped, "bbox_normalized": [clipped[index]/(width if index % 2 == 0 else height)
                                                           for index in range(4)] if clipped else None,
            "unclipped_bbox_pixels": raw,
            "image_boundary_clipped": bool(raw and clipped and any(abs(a-b) > 1e-7 for a, b in zip(raw, clipped))),
            "near_plane_clipped": any(point[2] < NEAR_M for point in corners),
            "corner_optical_depth_range_m": [min(point[2] for point in corners), max(point[2] for point in corners)],
            "near_plane_m": NEAR_M}


def rectangle_iou(actual, predicted):
    if actual is None:
        return 0.0
    intersection = max(0.0, min(actual[2], predicted[2])-max(actual[0], predicted[0])) * max(
        0.0, min(actual[3], predicted[3])-max(actual[1], predicted[1]))
    actual_area = (actual[2]-actual[0])*(actual[3]-actual[1])
    predicted_area = (predicted[2]-predicted[0])*(predicted[3]-predicted[1])
    return intersection/(actual_area+predicted_area-intersection)


def jpeg_dimensions(data):
    """Read standard JPEG SOF dimensions without an image or numeric dependency."""
    if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
        raise ValueError("Capture evidence is not a complete JPEG")
    index = 2
    sof_markers = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}
    while index < len(data):
        if data[index] != 0xFF:
            raise ValueError("Unexpected data before JPEG scan")
        while index < len(data) and data[index] == 0xFF:
            index += 1
        if index >= len(data):
            break
        marker = data[index]
        index += 1
        if marker in (0xD8, 0xD9, 0x01) or 0xD0 <= marker <= 0xD7:
            continue
        length = int.from_bytes(data[index:index+2], "big")
        if length < 2 or index+length > len(data):
            raise ValueError("JPEG segment length is invalid")
        if marker in sof_markers:
            payload = data[index+2:index+length]
            if len(payload) < 6:
                raise ValueError("JPEG frame header is too short")
            return int.from_bytes(payload[3:5], "big"), int.from_bytes(payload[1:3], "big")
        if marker == 0xDA:
            break
        index += length
    raise ValueError("No JPEG frame dimensions were found")


def read_reports(path):
    if path.suffix == ".jsonl":
        lines = [line for line in path.read_text().splitlines() if line.strip()]
        if not lines:
            raise ValueError("Public command log is empty")
        value = json.loads(lines[-1])
    else:
        value = json.loads(path.read_text())
    if isinstance(value, list):
        candidates = value
    elif isinstance(value, dict):
        if isinstance(value.get("command"), dict):
            value = value["command"]
        candidates = value.get("perceptions") or ([value["perception"]] if value.get("perception") else [value])
    else:
        raise ValueError("Expected a public report, command, task validation, or perceptions array")
    reports, seen = [], set()
    for report in candidates:
        identifier = report.get("capture_id") if isinstance(report, dict) else None
        if identifier and identifier not in seen and report.get("status") == "ok":
            reports.append(report)
            seen.add(identifier)
    if not reports:
        raise ValueError("No successful public observation report with a capture_id exists")
    return reports


def evaluate(layout, report, metadata, jpeg, object_id, target_index):
    objects = [item for item in layout["objects"] if item.get("id") == object_id]
    if len(objects) != 1:
        raise ValueError("Object ID does not identify exactly one private-layout object")
    if report["capture_id"] != metadata.get("capture_id"):
        raise ValueError("Public observation and saved capture IDs differ")
    snapshot = metadata["snapshot"]
    digest = hashlib.sha256(jpeg).hexdigest()
    for camera in (metadata.get("camera") or {}, report.get("camera") or {}):
        if camera.get("image_sha256") != digest:
            raise ValueError("JPEG bytes do not match the saved/public image hash")
    width, height = jpeg_dimensions(jpeg)
    if (width, height) != (snapshot["width"], snapshot["height"]):
        raise ValueError("JPEG dimensions differ from snapshot calibration")
    if finite(report["captured_at_sim_s"]) != finite(snapshot["stamp_sim_s"]):
        raise ValueError("Public report and capture snapshot timestamps differ")
    targets = report.get("targets") or []
    if not 0 <= target_index < len(targets):
        raise ValueError("Target index is outside this public report's targets array")
    selected = targets[target_index]
    predicted = normalized_box(selected["bbox"])
    actual = project_aabb(objects[0]["aabb_world_m"], snapshot)
    limitations = ["这是实际 SDF visual/collision 联合世界 AABB 的相机投影，不是目标像素分割真值。",
                   "投影不判断前景遮挡、透明材质或实际可见表面；视野裁剪和 AABB 空白区域会影响矩形 IoU。",
                   "单帧单目标矩形 IoU 不能代表检测召回率、定位误差或总体模型准确率。",
                   "相机位姿采用 ModelStates 接收时刻近似对齐，最多允许 0.05 仿真秒偏差。",
                   "离线 layout 必须对应生成该次世界的版本；本脚本不向 Agent 提供任何对象真值。"]
    return {"evaluation": "offline_sdf_aabb_projection_vs_public_vlm_bbox", "capture_id": report["capture_id"],
            "image_sha256": digest, "captured_at_sim_s": snapshot["stamp_sim_s"],
            "image_size_pixels": [width, height], "scene_capture": snapshot.get("scene"),
            "private_layout_scene_id": layout.get("scene_id"), "object_id": object_id,
            "object_label_ground_truth": objects[0].get("label"),
            "aabb_bounds_source": objects[0].get("bounds_source", "layout_aabb"),
            "aabb_world_m": objects[0]["aabb_world_m"], "actual_projected_aabb": actual,
            "predicted": {"target_index": target_index, "label": selected.get("label"),
                          "bbox_normalized": predicted,
                          "bbox_pixels": [predicted[index]*(width if index % 2 == 0 else height) for index in range(4)]},
            "rectangle_iou": rectangle_iou(actual["bbox_normalized"], predicted),
            "camera_pose_alignment_error_sim_s": snapshot["camera_pose_world"]["time_alignment_error_s"],
            "limitations": limitations}


def self_test():
    photo = {"location_frame": "gazebo_world", "width": 640, "height": 480,
             "camera_intrinsics": {"fx": 320, "fy": 320, "cx": 320, "cy": 240,
                                   "distortion_coefficients": [0]*5},
             "camera_pose_world": {"position_m": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1],
                                   "optical_axes": "x_right_y_down_z_forward", "time_alignment_error_s": 0}}
    box = project_aabb({"x": [-.5, .5], "y": [-.5, .5], "z": [2, 3]}, photo)
    expected = [.375, 1/3, .625, 2/3]
    assert all(abs(a-b) < 1e-10 for a, b in zip(box["bbox_normalized"], expected))
    assert abs(rectangle_iou(expected, expected)-1) < 1e-12
    assert rectangle_iou(expected, [0, 0, .1, .1]) == 0
    assert abs(rectangle_iou([0, 0, .5, .5], [.25, .25, .75, .75])-1/7) < 1e-12
    assert project_aabb({"x": [-.5, .5], "y": [-.5, .5], "z": [-3, -2]}, photo)["bbox_normalized"] is None
    clipped = project_aabb({"x": [1, 3], "y": [-.5, .5], "z": [2, 3]}, photo)
    assert clipped["image_boundary_clipped"] and clipped["bbox_normalized"][2] == 1
    near = project_aabb({"x": [-1, 1], "y": [-1, 1], "z": [-1, 1]}, photo)
    assert near["near_plane_clipped"] and near["bbox_normalized"] == [0, 0, 1, 1]
    photo["camera_pose_world"]["orientation_xyzw"] = [0, math.sqrt(.5), 0, math.sqrt(.5)]
    rotated = project_aabb({"x": [2, 3], "y": [-.5, .5], "z": [-.5, .5]}, photo)
    assert all(abs(a-b) < 1e-10 for a, b in zip(rotated["bbox_normalized"], expected))
    return {"self_test": "passed", "checks": ["pinhole", "inverse camera quaternion", "rectangle IoU",
             "behind camera", "viewport polygon clipping", "near-plane edge clipping"]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", type=Path, default=ROOT / "scene/layout.json")
    parser.add_argument("--capture-dir", type=Path, default=ROOT / "logs/system/latest/captures")
    parser.add_argument("--report", type=Path, help="Public perceptions JSON, task validation, command JSON, or command_updates.jsonl")
    parser.add_argument("--object-id", "--object", dest="object_id", help="Private layout object ID, e.g. red_shipping_container")
    parser.add_argument("--capture-id", help="Select a specific captured view instead of --view-index")
    parser.add_argument("--view-index", type=int, default=0, help="Successful distinct observation index (default 0)")
    parser.add_argument("--target-index", type=int, default=0, help="Public targets array index (default 0)")
    parser.add_argument("--output", type=Path, help="Optional offline evaluation JSON destination")
    parser.add_argument("--self-test", action="store_true", help="Verify geometry locally without reading experiment evidence")
    args = parser.parse_args(argv)
    if args.self_test:
        result = self_test()
    else:
        if args.report is None or args.object_id is None:
            parser.error("--report and --object-id are required unless --self-test is used")
        reports = read_reports(args.report)
        if args.capture_id:
            matches = [item for item in reports if item["capture_id"] == args.capture_id]
            if len(matches) != 1:
                raise ValueError("Requested capture_id is not a successful report in this file")
            report = matches[0]
        else:
            if not 0 <= args.view_index < len(reports):
                raise ValueError("View index is outside the successful observations")
            report = reports[args.view_index]
        identifier = report["capture_id"]
        if not isinstance(identifier, str) or len(identifier) != 32 or any(char not in "0123456789abcdef" for char in identifier):
            raise ValueError("Capture ID must be the archived 32-character hexadecimal identifier")
        metadata_path = args.capture_dir / (identifier + ".json")
        jpeg_path = args.capture_dir / (identifier + ".jpg")
        result = evaluate(json.loads(args.layout.read_text()), report,
                          json.loads(metadata_path.read_text()), jpeg_path.read_bytes(),
                          args.object_id, args.target_index)
        result["source_files"] = {"private_layout": str(args.layout.resolve()), "public_report": str(args.report.resolve()),
                                  "capture_metadata": str(metadata_path.resolve()), "capture_jpeg": str(jpeg_path.resolve())}
    encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded)
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, KeyError, OSError) as exc:
        print("Offline evaluation failed: {}".format(exc), file=sys.stderr)
        sys.exit(1)
