#!/usr/bin/env python3
"""Offline matplotlib artifacts from recorded actual flight poses and collision AABBs.

No HTTP, model, ROS, simulation control, or flight request is issued. Hidden
feasibility witnesses and navigation planner paths are never read or plotted.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import textwrap

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle


ROOT = Path(__file__).resolve().parents[1]
COLORS = {"outbound": "#2468a8", "return": "#d77426", "land": "#7a8791", "other": "#a7b2bb"}


def finite_point(point, length=3):
    return (isinstance(point, (list, tuple)) and len(point) >= length
            and all(not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
                    for value in point[:length]))


def read_json(path):
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        # A report can be in the process of being written for an active run.
        return None


def load_samples(path):
    rows = []
    lines = path.read_text().splitlines()
    for index, line in enumerate(lines):
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            if index != len(lines) - 1:
                raise ValueError("invalid recorded sample at line {}".format(index + 1))
    if len(rows) < 2:
        raise ValueError("at least two recorded samples are required")
    return rows


def load_collision_boxes(path):
    layout = json.loads(path.read_text())
    if layout.get("ground_truth_only") is not True:
        raise ValueError("layout must be explicitly evaluation-only")
    boxes = []
    for item in layout.get("objects", []):
        # This is the only geometry field used. Visual bounds and witness
        # routes play no part in this offline picture or clearance calculation.
        box = item.get("collision_aabb_world_m")
        if not isinstance(box, dict):
            raise ValueError("missing pure collision AABB for {}".format(item.get("id")))
        if any(not finite_point(box.get(axis), 2) or box[axis][0] > box[axis][1] for axis in ("x", "y", "z")):
            raise ValueError("invalid collision AABB")
        boxes.append({"id": item.get("id", "collision"), "box": box})
    if not boxes:
        raise ValueError("no collision AABBs are available")
    return boxes


def segment_box_distance(start, end, box):
    """Exact XY distance of a sampled connecting segment to a rectangle."""
    delta = [end[index] - start[index] for index in (0, 1)]
    intervals = [0., 1.]
    for axis, key in enumerate(("x", "y")):
        if abs(delta[axis]) > 1e-12:
            for boundary in box[key]:
                fraction = (boundary - start[axis]) / delta[axis]
                if 0 < fraction < 1:
                    intervals.append(fraction)
    intervals = sorted(set(intervals))
    best = (float("inf"), 0.)
    for low, high in zip(intervals, intervals[1:]):
        middle, coefficients = (low + high) / 2, []
        for axis, key in enumerate(("x", "y")):
            value = start[axis] + delta[axis] * middle
            boundary = box[key][0] if value < box[key][0] else box[key][1] if value > box[key][1] else None
            coefficients.append((delta[axis], start[axis] - boundary) if boundary is not None else (0., 0.))
        candidates = [low, high]
        denominator = sum(first * first for first, _ in coefficients)
        if denominator:
            stationary = -sum(first * second for first, second in coefficients) / denominator
            candidates.append(max(low, min(high, stationary)))
        for fraction in candidates:
            squared = sum((first * fraction + second) ** 2 for first, second in coefficients)
            if squared < best[0]:
                best = (squared, fraction)
    return (math.sqrt(max(0., best[0])),
            [start[index] + delta[index] * best[1] for index in (0, 1)])


def records_from_samples(samples):
    records, phase = [], "other"
    for index, sample in enumerate(samples):
        state = sample.get("state", sample)
        gazebo = state.get("gazebo") or {}
        world, enu = gazebo.get("position_m"), state.get("position_enu_m")
        action = (state.get("mission") or {}).get("action")
        if action == "goto":
            phase = "outbound"
        elif action == "return_home":
            phase = "return"
        elif action == "land" or state.get("mode") == "AUTO.LAND":
            phase = "land"
        elif action == "takeoff":
            phase = "other"
        if not gazebo.get("available") or not finite_point(world) or not math.isfinite(gazebo.get("stamp_sim_s", float("nan"))):
            continue
        orientation = gazebo.get("orientation_xyzw")
        lidar_z = float("nan")
        if finite_point(orientation, 4):
            qx, qy, qz, qw = orientation
            norm_squared = qx*qx + qy*qy + qz*qz + qw*qw
            if norm_squared > 1e-12:
                # Mount translation [0,0,.35] in the body frame, transformed
                # with the observed world orientation; this is not body Z.
                lidar_z = world[2] + .35 * (1 - 2 * (qx*qx + qy*qy) / norm_squared)
        records.append({"index": index, "world": world, "enu_z": enu[2] if finite_point(enu) else float("nan"),
                        "sim_time": gazebo["stamp_sim_s"], "wall_time": sample.get("wall_time"),
                        "lidar_z": lidar_z, "phase": phase})
    if len(records) < 2:
        raise ValueError("insufficient valid fresh Gazebo body poses")
    if any(second["sim_time"] < first["sim_time"] for first, second in zip(records, records[1:])):
        raise ValueError("simulation clock reset; refusing to combine different clock epochs")
    return records


def actual_clearance(records, boxes):
    best = {"distance_m": float("inf")}
    for first, second in zip(records, records[1:]):
        if second["index"] != first["index"] + 1:
            continue
        for item in boxes:
            distance, point = segment_box_distance(first["world"], second["world"], item["box"])
            if distance < best["distance_m"]:
                best = {"distance_m": distance, "position_xy_m": point, "object_id": item["id"],
                        "box": item["box"], "start_sample": first["index"], "end_sample": second["index"]}
    if not math.isfinite(best["distance_m"]):
        raise ValueError("no consecutive valid world pose pair is available")
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path, help="task directory containing recorded state_samples.jsonl")
    parser.add_argument("--output", type=Path, default=ROOT / "logs/avoidance/round_trip_trajectory.png")
    parser.add_argument("--layout", type=Path, default=ROOT / "scene/obstacle_layout.json")
    args = parser.parse_args()
    sample_path, evaluation_path, task_path = (args.run / name for name in
        ("state_samples.jsonl", "trajectory_evaluation.json", "task_validation.json"))
    samples = load_samples(sample_path)
    evaluation, task = read_json(evaluation_path), read_json(task_path)
    records, boxes = records_from_samples(samples), load_collision_boxes(args.layout)
    closest = actual_clearance(records, boxes)
    if evaluation and evaluation.get("body_center_coordinate_frame") != "gazebo_world":
        raise ValueError("trajectory evaluator frame is not Gazebo world")
    if evaluation and not math.isclose(closest["distance_m"], evaluation["minimum_interpolated_segment_clearance_m"],
                                       abs_tol=1e-6, rel_tol=1e-6):
        raise ValueError("recorded trajectory and saved clearance evaluation do not match")
    outcome = "PASSED" if task and task.get("validation_passed") is True else (
        "FAILED" if task and task.get("validation_passed") is False and task.get("finished_at") else "PENDING")
    outcome_color = {"PASSED": "#237a54", "FAILED": "#a83939", "PENDING": "#97751d"}[outcome]
    initial = (task or {}).get("initial_state") or samples[0].get("state", samples[0])
    region = (initial.get("scene") or {}).get("flight_region") or {}
    low, high = region.get("bounds_min_m"), region.get("bounds_max_m")
    bounds = [low[0], low[1], high[0], high[1]] if finite_point(low) and finite_point(high) else [-3., -8., 16., 8.]
    baseline = (evaluation or {}).get("direct_segment_baseline") or {}
    home = baseline.get("home_xy_m") if finite_point(baseline.get("home_xy_m"), 2) else records[0]["world"][:2]
    observed_goal = baseline.get("goal_xy_m")
    goal = observed_goal if finite_point(observed_goal, 2) else None
    goal_source = "saved observed goal evaluation" if goal else None
    # Direct executor reliability reports retain arrival events instead of the
    # older baseline's goal field. Select an actual fresh world sample near
    # that arrival, after checking the requested goal in its ENU frame.
    if goal is None and task:
        original_id = task.get("original_mission_id")
        for mission in task.get("missions", []):
            if original_id and mission.get("id") != original_id:
                continue
            for event in mission.get("events", []):
                if event.get("action") != "goto":
                    continue
                step_index = event.get("step", 0)-1
                steps = mission.get("steps", [])
                if not 0 <= step_index < len(steps) or steps[step_index].get("action") != "goto":
                    continue
                step = steps[step_index]
                requested = [step.get("x"), step.get("y"), step.get("altitude")]
                candidates = []
                for record in records:
                    row = samples[record["index"]]
                    state = row.get("state", row)
                    enu = state.get("position_enu_m")
                    difference = abs(row.get("wall_time", -1e20)-event.get("finished_at", 1e20))
                    if (state.get("armed") and difference <= 1.5 and finite_point(requested)
                            and finite_point(enu) and math.dist(enu, requested) <= .65):
                        candidates.append((difference, record))
                if candidates:
                    record = min(candidates, key=lambda value: value[0])[1]
                    goal = record["world"][:2]
                    goal_source = {"mission_id": mission.get("id"), "event_finished_at": event["finished_at"],
                                   "recorded_sample_index": record["index"], "wall_time": record["wall_time"],
                                   "requested_goal_enu_m": requested}
                    break
            if goal:
                break
    minimum_requirement = (evaluation or {}).get("expected_clearance_m", 1.)
    sim_gaps = [second["sim_time"] - first["sim_time"] for first, second in zip(records, records[1:])]
    max_gap = max(sim_gaps)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "svg.fonttype": "none",
                         "axes.spines.top": False, "axes.spines.right": False})
    figure, axes = plt.subplots(2, 1, figsize=(12, 11), gridspec_kw={"height_ratios": [3.1, 1.15]})
    xy, height = axes
    figure.suptitle("Autonomous obstacle navigation | {} | {}".format(outcome, args.run.parent.name),
                   fontsize=16, fontweight="bold", color=outcome_color, y=.98)
    figure.text(.5, .952, "Actual sampled-polyline clearance: {:.3f} m / {:.1f} m requirement   |   {} poses   |   max sim gap {:.3f} s".format(
        closest["distance_m"], minimum_requirement, len(records), max_gap), ha="center", color="#45515f", fontsize=10)
    xy.set_facecolor("#f6f8fa")
    for item in boxes:
        box, identifier = item["box"], item["id"]
        main_obstacle = identifier.startswith("obstacle_wall") or identifier.endswith("boundary_cargo_stack") or identifier.startswith("utility_pillar")
        rectangle = Rectangle((box["x"][0], box["y"][0]), box["x"][1] - box["x"][0], box["y"][1] - box["y"][0],
                              facecolor="#5b6875" if main_obstacle else "#ced5da", edgecolor="#43505d",
                              linewidth=.8, alpha=.9 if main_obstacle else .55, zorder=2)
        xy.add_patch(rectangle)
        if identifier in ("obstacle_wall_1", "obstacle_wall_2"):
            xy.text(sum(box["x"]) / 2, sum(box["y"]) / 2, "Wall " + identifier[-1],
                    rotation=90, ha="center", va="center", color="white", fontsize=10, zorder=3)
    xy.add_patch(Rectangle((bounds[0], bounds[1]), bounds[2] - bounds[0], bounds[3] - bounds[1],
                          fill=False, edgecolor="#84929e", linestyle="--", linewidth=1.1, zorder=1))
    segments, phase_colors = [], []
    for first, second in zip(records, records[1:]):
        if second["index"] == first["index"] + 1:
            segments.append([first["world"][:2], second["world"][:2]])
            phase_colors.append(COLORS[second["phase"]])
    xy.add_collection(LineCollection(segments, colors=phase_colors, linewidths=2.6, zorder=4))
    xy.scatter([home[0]], [home[1]], s=80, facecolor="white", edgecolor="#237a54", linewidth=2, zorder=6)
    xy.annotate("Start / takeoff", home, xytext=(-15, -20), textcoords="offset points", fontsize=9, color="#237a54")
    if goal:
        xy.scatter([goal[0]], [goal[1]], s=140, marker="*", color="#1e6e43", zorder=6)
        xy.annotate("Goal (observed arrival)", goal, xytext=(-80, 13), textcoords="offset points", fontsize=9, color="#237a54")
    else:
        xy.text(.99, .98, "Goal world arrival not recorded", transform=xy.transAxes, ha="right", va="top",
                color="#6a7580", fontsize=9, bbox={"facecolor": "white", "edgecolor": "#e2e6ea", "alpha": .9})
    final = records[-1]["world"][:2]
    xy.scatter([final[0]], [final[1]], marker="x", s=65, color=outcome_color, linewidth=1.7, zorder=6)
    clearance_point = closest["position_xy_m"]
    box = closest["box"]
    contact = [max(box[key][0], min(box[key][1], clearance_point[index])) for index, key in enumerate(("x", "y"))]
    xy.plot([clearance_point[0], contact[0]], [clearance_point[1], contact[1]], color="#a24473", linestyle=":", linewidth=1.5, zorder=5)
    xy.scatter([clearance_point[0]], [clearance_point[1]], s=40, color="#a24473", zorder=6)
    xy.annotate("Minimum {:.3f} m".format(closest["distance_m"]), clearance_point,
                xytext=(-100, 22), textcoords="offset points", color="#a24473", fontsize=9,
                arrowprops={"arrowstyle": "-", "color": "#a24473"})
    xy.set_xlim(bounds[0] - .6, bounds[2] + .6)
    xy.set_ylim(bounds[1] - .6, bounds[3] + .6)
    xy.set_aspect("equal", adjustable="box")
    xy.set_xlabel("Gazebo world X (m)")
    xy.set_ylabel("Gazebo world Y (m)")
    xy.grid(True, color="#dce3e8", linewidth=.6, zorder=0)
    handles = [Patch(facecolor="#5b6875", label="Collision AABB (offline evaluation)")]
    for phase, label in (("outbound", "Actual outbound / goto"), ("return", "Actual return_home"), ("land", "Actual landing")):
        if phase in [record["phase"] for record in records]:
            handles.append(Line2D([0], [0], color=COLORS[phase], lw=2.6, label=label))
    xy.legend(handles=handles, loc="lower left", fontsize=8.5, framealpha=.95)
    elapsed = [record["sim_time"] - records[0]["sim_time"] for record in records]
    # The band applies to the mounted LiDAR origin in world coordinates, not
    # to body/ENU height or the takeoff/landing phases while mapping is off.
    height.axhspan(2.5, 4.3, facecolor="#e5f0e8", alpha=.7, label="LiDAR world-height gate (mapping only)")
    height.plot(elapsed, [record["world"][2] for record in records], color="#2468a8", lw=1.6, label="Body Z / Gazebo world")
    height.plot(elapsed, [record["enu_z"] for record in records], color="#d77426", lw=1.4, label="Body up / PX4 local ENU")
    height.plot(elapsed, [record["lidar_z"] for record in records], color="#3c7760", lw=1.3, linestyle=":", label="LiDAR origin Z / Gazebo world")
    height.set_xlabel("Elapsed Gazebo simulation time (s)")
    height.set_ylabel("Height (m); frames in legend")
    height.set_title("Recorded heights; world and local ENU are distinct coordinate frames", fontsize=10, loc="left")
    height.grid(True, color="#e1e6ea", linewidth=.6)
    height.legend(loc="lower center", bbox_to_anchor=(.5, 1.06), ncol=2, fontsize=8, framealpha=.95)
    footer = "Collision geometry is evaluation-only. Clearance measures recorded body-center connecting segments, not continuous-time contact."
    if outcome == "FAILED" and (task or {}).get("error"):
        footer += "\nFailure: " + str(task["error"])
    figure.text(.08, .018, textwrap.fill(footer.split("\n")[0], 135) + ("\n" + textwrap.fill(footer.split("\n", 1)[1], 135) if "\n" in footer else ""),
                 ha="left", va="bottom", fontsize=8.2, color="#63717d")
    figure.subplots_adjust(top=.91, bottom=.09, left=.08, right=.96, hspace=.55)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    png_path, svg_path = args.output.with_suffix(".png"), args.output.with_suffix(".svg")
    figure.savefig(png_path, dpi=180, facecolor="white")
    figure.savefig(svg_path, facecolor="white")
    plt.close(figure)
    sources = {}
    for name, path in (("samples", sample_path), ("evaluation", evaluation_path), ("task", task_path), ("collision_layout", args.layout)):
        sources[name] = {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()} if path.exists() else None
    metadata = {"offline_only": True, "outcome_from_task_validation": outcome, "run": str(args.run),
                "coordinate_frame_xy": "gazebo_world", "planner_paths_plotted": False, "witness_paths_used": False,
                "valid_world_poses": len(records), "recorded_samples": len(samples), "collision_objects": len(boxes),
                "actual_sampled_polyline_minimum_clearance_m": closest["distance_m"], "closest_obstacle_id": closest["object_id"],
                "maximum_sim_sample_gap_s": max_gap, "observed_goal_world_xy_m": goal,
                "observed_goal_source": goal_source,
                "lidar_body_translation_m": [0, 0, .35], "lidar_world_height_gate_m": [2.5, 4.3],
                "data_sources": sources, "outputs": [str(png_path), str(svg_path)]}
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2))
    print(json.dumps({"outcome": outcome, "minimum_sampled_polyline_clearance_m": closest["distance_m"],
                      "poses": len(records), "outputs": metadata["outputs"]}))


if __name__ == "__main__":
    main()
