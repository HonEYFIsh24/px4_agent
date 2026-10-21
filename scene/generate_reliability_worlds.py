#!/usr/bin/env python3
"""Generate reproducible lidar-navigation trials without rewriting the baseline.

Worlds are deep copies of the already validated obstacle yard, including its
local materials, lighting, ground and PX4 physics. Only tall walls and pillars
are changed. Runtime manifests contain operating permissions; collision truth,
random parameters and offline certificates stay in evaluation-only layouts.
This module is stdlib-only and does not start ROS, flight or model requests.
"""
import argparse
import copy
import hashlib
import json
import math
import random
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent
BASE_WORLD = ROOT / "worlds/obstacle_yard.world"
BASE_MANIFEST = ROOT / "obstacle_manifest.json"
BASE_LAYOUT = ROOT / "obstacle_layout.json"
BASE_TASKS = ROOT / "avoidance_tasks.json"
OUTPUT = ROOT / "reliability"
SEEDS = (17, 73, 211)
BOUNDS_XY = (-3.0, -8.0, 16.0, 8.0)
GOAL = [13.0, 0.0, 3.5]
SAMPLE_STEP_M = .02
CERTIFICATE_CLEARANCE_M = 1.6
CHANGED_NAMES = frozenset(("obstacle_wall_1", "obstacle_wall_2",
    "south_boundary_cargo_stack", "north_boundary_cargo_stack",
    "utility_pillar_southwest", "utility_pillar_northeast", "utility_pillar_southeast"))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                    allow_nan=False) + "\n", encoding="utf-8")


def values(numbers):
    return " ".join("{:.9g}".format(number) for number in numbers)


def parse_pose(element):
    return [float(value) for value in element.findtext("pose", "0 0 0 0 0 0").split()]


def rotation(pose):
    # Same SDF Rz(yaw) Ry(pitch) Rx(roll) convention as generate_world.py,
    # expressed without its texture-generation NumPy/OpenCV dependencies.
    roll, pitch, yaw = pose[3:]
    cr, cp, cy = math.cos(roll), math.cos(pitch), math.cos(yaw)
    sr, sp, sy = math.sin(roll), math.sin(pitch), math.sin(yaw)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))


def multiply(first, second):
    return tuple(tuple(sum(first[row][k]*second[k][col] for k in range(3))
                       for col in range(3)) for row in range(3))


def apply(matrix, vector):
    return [sum(row[k]*vector[k] for k in range(3)) for row in matrix]


def geometry_bounds(model, collisions_only=False):
    """AABB of actual SDF geometry, preserving independent collision extents."""
    model_pose = parse_pose(model)
    rm = rotation(model_pose)
    minimum, maximum = [math.inf]*3, [-math.inf]*3
    for link in model.findall("link"):
        link_pose = parse_pose(link)
        rl = rotation(link_pose)
        elements = list(link.findall("collision"))
        if not collisions_only:
            elements += list(link.findall("visual"))
        for element in elements:
            pose = parse_pose(element)
            r = multiply(multiply(rm, rl), rotation(pose))
            inner = apply(rl, pose[:3])
            offset = apply(rm, [link_pose[k]+inner[k] for k in range(3)])
            center = [model_pose[k]+offset[k] for k in range(3)]
            geometry = element.find("geometry")
            if geometry.find("box") is not None:
                half = [float(value)/2 for value in geometry.findtext("box/size").split()]
                extent = [sum(abs(r[row][k])*half[k] for k in range(3)) for row in range(3)]
            elif geometry.find("cylinder") is not None:
                radius = float(geometry.findtext("cylinder/radius"))
                half = float(geometry.findtext("cylinder/length"))/2
                extent = [half*abs(r[row][2])+radius*math.sqrt(max(0., 1-r[row][2]**2))
                          for row in range(3)]
            elif geometry.find("sphere") is not None:
                extent = [float(geometry.findtext("sphere/radius"))]*3
            else:
                raise ValueError("Unexpected obstacle geometry: " + model.attrib["name"])
            for axis in range(3):
                minimum[axis] = min(minimum[axis], center[axis]-extent[axis])
                maximum[axis] = max(maximum[axis], center[axis]+extent[axis])
    if not all(math.isfinite(value) for value in minimum+maximum):
        raise ValueError("No physical geometry for " + model.attrib["name"])
    return {axis: [round(minimum[index], 6), round(maximum[index], 6)]
            for index, axis in enumerate(("x", "y", "z"))}


def reposition(model, x, y):
    pose = parse_pose(model)
    pose[:2] = [x, y]
    model.find("pose").text = values(pose)


def resize_wall(model, x, lower, upper):
    """Resize copied wall geometry, retaining its concrete and hazard details."""
    collision = model.find("./link/collision/geometry/box/size")
    old_length = float(collision.text.split()[1])
    ratio = (upper-lower)/old_length
    reposition(model, x, (upper+lower)/2)
    for element in model.findall("./link/visual")+model.findall("./link/collision"):
        pose = parse_pose(element)
        pose[1] *= ratio
        element.find("pose").text = values(pose)
        size = element.find("geometry/box/size")
        if size is None:
            raise ValueError("Expected only copied box geometry in a wall")
        dimensions = [float(value) for value in size.text.split()]
        dimensions[1] *= ratio
        size.text = values(dimensions)


def distance_to_rectangle(point, box):
    return math.hypot(max(box["x"][0]-point[0], point[0]-box["x"][1], 0),
                      max(box["y"][0]-point[1], point[1]-box["y"][1], 0))


def active_in_xy(box):
    return (box["x"][1] > BOUNDS_XY[0] and box["x"][0] < BOUNDS_XY[2]
            and box["y"][1] > BOUNDS_XY[1] and box["y"][0] < BOUNDS_XY[3])


def certify(objects, witness, blocked):
    active = [obj for obj in objects if obj["inside_navigation_xy"]]
    for obj in active:
        if obj["collision_aabb_world_m"]["z"][1] < 5.4:
            raise ValueError("An active obstacle does not cover the lidar plane")
    if any(distance_to_rectangle([0, 0], obj["collision_aabb_world_m"]) < 1.5 for obj in objects):
        raise ValueError("Home vertical corridor is obstructed")
    result = {"xml_parse": True, "home_radius_1_5m_clear": True,
              "all_active_obstacles_cover_lidar_plane": True,
              "active_obstacles": [obj["id"] for obj in active],
              "witness_hidden_from_agent": True}
    if blocked:
        barrier = next(obj for obj in objects if obj["id"] == "obstacle_wall_1")
        box = barrier["collision_aabb_world_m"]
        if not (0 < box["x"][0] < box["x"][1] < GOAL[0]
                and box["y"][0] <= BOUNDS_XY[1] and box["y"][1] >= BOUNDS_XY[3]):
            raise ValueError("The negative trial must physically seal the complete permitted width")
        result.update(feasible_path_exists=False,
            blocked_path_certificate={"type": "continuous_full_width_collision_barrier",
                "object_id": barrier["id"], "collision_aabb_world_m": copy.deepcopy(box),
                "home_and_goal_on_opposite_x_sides": True,
                "fixed_altitude_only": True,
                "known_boundary_bypass_available": False})
        return result
    minimum = math.inf
    for first, second in zip(witness, witness[1:]):
        length = math.hypot(second[0]-first[0], second[1]-first[1])
        count = max(1, int(math.ceil(length/SAMPLE_STEP_M)))
        for index in range(count+1):
            point = [first[axis]+(second[axis]-first[axis])*index/count for axis in (0, 1)]
            if min(point[0]-BOUNDS_XY[0], BOUNDS_XY[2]-point[0],
                   point[1]-BOUNDS_XY[1], BOUNDS_XY[3]-point[1]) < 2.0:
                raise ValueError("Witness must retain boundary margin for actual grid navigation")
            minimum = min(minimum, *(distance_to_rectangle(point, obj["collision_aabb_world_m"])
                                     for obj in objects))
    lower_bound = minimum-SAMPLE_STEP_M
    if lower_bound < CERTIFICATE_CLEARANCE_M:
        raise ValueError("Insufficient independently evaluable physical clearance")
    result.update(feasible_path_exists=True, witness_sample_step_m=SAMPLE_STEP_M,
        certificate_method="sampled_2d_distance_minus_full_step_lipschitz_bound",
        certified_horizontal_clearance_lower_bound_m=round(lower_bound, 6),
        required_certificate_clearance_m=CERTIFICATE_CLEARANCE_M,
        planner_minimum_clearance_m=1.0)
    return result


def indent(element, level=0):
    """Stable serialization on Python 3.8 and 3.11 (ET.indent arrived in 3.9)."""
    prefix = "\n"+"  "*level
    if len(element):
        if not element.text or not element.text.strip():
            element.text = prefix+"  "
        for child in element:
            indent(child, level+1)
        child.tail = prefix
    if level and (not element.tail or not element.tail.strip()):
        element.tail = prefix


def generate_variant(output, seed, blocked=False):
    identifier = "reliability_{}_{}".format("blocked" if blocked else "seed", seed)
    rng = random.Random(seed)
    # Quantized input parameters make the recorded fixture easy to reproduce.
    # >= 4.7 m physical separation keeps both the hard inflation and its soft
    # clearance preference feasible through the inter-wall turning corridor.
    x1, x2 = round(rng.uniform(4.0, 4.15), 3), round(rng.uniform(9.85, 10.0), 3)
    upper, lower = round(rng.uniform(1.55, 2.45), 3), round(rng.uniform(-.45, .45), 3)
    pillars = {"utility_pillar_southwest": [round(rng.uniform(.6, 1.4), 3), round(rng.uniform(-6.6, -5.4), 3)],
        "utility_pillar_northeast": [round(rng.uniform(12.6, 13.7), 3), round(rng.uniform(5.6, 6.4), 3)],
        "utility_pillar_southeast": [round(rng.uniform(13.1, 14.3), 3), round(rng.uniform(-6.5, -5.5), 3)]}
    sdf = copy.deepcopy(ET.parse(str(BASE_WORLD)).getroot())
    world = sdf.find("world")
    models = {model.attrib["name"]: model for model in world.findall("model")}
    resize_wall(models["obstacle_wall_1"], x1, -8 if blocked else -6, 8 if blocked else upper)
    resize_wall(models["obstacle_wall_2"], x2, lower, 6)
    resize_wall(models["north_boundary_cargo_stack"], x2, 6, 8)
    if blocked:
        world.remove(models.pop("south_boundary_cargo_stack"))
    else:
        resize_wall(models["south_boundary_cargo_stack"], x1, -8, -6)
    for name, point in pillars.items():
        reposition(models[name], *point)
    objects = []
    for source in json.loads(BASE_LAYOUT.read_text())["objects"]:
        if source["id"] not in models:
            continue
        record = copy.deepcopy(source)
        model = models[record["id"]]
        record["position_world_m"] = parse_pose(model)[:3]
        record["aabb_world_m"] = geometry_bounds(model)
        record["collision_aabb_world_m"] = geometry_bounds(model, collisions_only=True)
        record["inside_navigation_xy"] = active_in_xy(record["collision_aabb_world_m"])
        if record["id"] in CHANGED_NAMES:
            record["dimensions_model_m"] = [round(record["aabb_world_m"][axis][1]-record["aabb_world_m"][axis][0], 6)
                                             for axis in ("x", "y", "z")]
        objects.append(record)
    midpoint = round((x1+x2)/2, 4)
    witness = [] if blocked else [[0., 0., 3.5], [1.7, upper+2., 3.5],
        [midpoint, upper+2., 3.5], [midpoint, lower-2., 3.5],
        [12., lower-2., 3.5], list(GOAL)]
    validation = certify(objects, witness, blocked)
    output = Path(output)
    world_path = output/(identifier+".world")
    indent(sdf)
    ET.ElementTree(sdf).write(str(world_path), encoding="utf-8", xml_declaration=True)
    ET.parse(str(world_path))
    manifest = copy.deepcopy(json.loads(BASE_MANIFEST.read_text()))
    manifest.update(id=identifier, world_sha256=sha256(world_path))
    layout = {"scene_id": identifier, "name": "可重复随机绕障可靠性试验" if not blocked else "完全封路安全停止试验",
        "ground_truth_only": True, "world_sha256": manifest["world_sha256"],
        "ground": copy.deepcopy(manifest["ground_plane"]),
        "navigation_constraints": copy.deepcopy(manifest["navigation"]),
        "objects": objects, "validation": validation,
        "feasibility_witness_world_m": witness, "recommended_goal_enu_m": list(GOAL),
        "randomization": {"seed": seed, "first_wall_x_m": x1, "first_wall_upper_y_m": upper,
                          "second_wall_x_m": x2, "second_wall_lower_y_m": lower,
                          "pillar_positions_world_xy_m": pillars,
                          "fully_blocked": blocked},
        "coordinate_note": "Align live ENU and own-aircraft Gazebo pose when evaluating. Layout and certificate are never runtime inputs."}
    tasks = copy.deepcopy(json.loads(BASE_TASKS.read_text()))
    tasks.update(name=identifier+"_goal_only", expected_outcome="no_route" if blocked else "success")
    # First priority tests flight reliability. Image inspection remains covered
    # by the immutable baseline task configuration, avoiding unnecessary API cost.
    tasks["tasks"] = [tasks["tasks"][0]]
    tasks["tasks"][0]["expected_outcome"] = tasks["expected_outcome"]
    filenames = {"world": world_path.name, "manifest": identifier+"_manifest.json",
                 "layout": identifier+"_layout.json", "tasks": identifier+"_tasks.json"}
    write_json(output/filenames["manifest"], manifest)
    write_json(output/filenames["layout"], layout)
    write_json(output/filenames["tasks"], tasks)
    return {"id": identifier, "seed": seed, "expected_outcome": tasks["expected_outcome"],
            **filenames, "world_sha256": manifest["world_sha256"],
            "asset_sha256": {kind: sha256(output/filename) for kind, filename in filenames.items()}}


def generate(output=OUTPUT):
    baseline_paths = (BASE_WORLD, BASE_MANIFEST, BASE_LAYOUT, BASE_TASKS,
                      ROOT/"worlds/logistics_yard.world", ROOT/"runtime_manifest.json")
    before = {str(path): sha256(path) for path in baseline_paths}
    output = Path(output).resolve()
    if output == ROOT or output == ROOT/"worlds":
        raise ValueError("Output must be a separate reliability fixture directory")
    output.mkdir(parents=True, exist_ok=True)
    variants = [generate_variant(output, seed) for seed in SEEDS]
    variants.append(generate_variant(output, SEEDS[0], blocked=True))
    index = {"schema_version": 1, "path_base": "index_directory",
        "ground_truth_policy": "Only world, public manifest, prompt and min_observations belong in runtime. Layout, seed certificate and witness are evaluator-only.",
        "baseline": {"world": "scene/worlds/obstacle_yard.world", "world_sha256": before[str(BASE_WORLD)]},
        "scenes": variants}
    write_json(output/"index.json", index)
    if before != {str(path): sha256(path) for path in baseline_paths}:
        raise RuntimeError("A baseline asset was modified")
    return index


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT)
    args = parser.parse_args()
    result = generate(args.output_dir)
    for variant in result["scenes"]:
        print("{}: {} ({})".format(variant["id"], variant["expected_outcome"], variant["world_sha256"]))
    print("Baseline worlds and manifests unchanged; layouts and certificates are evaluator-only.")
