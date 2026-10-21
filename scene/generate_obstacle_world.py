#!/usr/bin/env python3
"""Build a separate, lidar-visible S-shaped obstacle yard.

Existing logistics_yard.world and its manifest are never rewritten. The runtime
manifest provides operating constraints, while obstacle_layout.json holds
independent evaluation truth and a feasibility witness hidden from the Agent.
"""
import copy
import hashlib
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import generate_world as yard


ROOT = Path(__file__).resolve().parent
BOUNDS_XY = [-3.0, -8.0, 16.0, 8.0]
HEIGHT = 5.5
# Validation-only witness; never placed in public navigation metadata.
WITNESS = [(0, 0, 3.5), (2, 3.8, 3.5), (7, 3.8, 3.5),
           (7, -2, 3.5), (12, -2, 3.5), (14, 0, 3.5)]


def reposition(world, name, x, y):
    mod = next(mod for mod in world.findall("model") if mod.attrib["name"] == name)
    pose = [float(value) for value in mod.findtext("pose").split()]
    pose[:2] = [x, y]
    mod.find("pose").text = yard.values(pose)
    record = next(item for item in yard.OBJECTS if item["id"] == name)
    record["position_world_m"][:2] = [x, y]


def wall(world, name, x, y, length, label):
    link = yard.model(world, name, (x, y, yard.GROUND_Z), size=(1.08, length + .08, HEIGHT), label=label)
    yard.part(link, "collision_wall", "box", (1, length, HEIGHT),
              (0, 0, HEIGHT / 2, 0, 0, 0), texture="concrete", collision=True)
    # Panel seams, thick coping, and industrial safety paint give this wall
    # recognizable surface details rather than a decorative feature barrier.
    yard.part(link, "wall_coping", "box", (1.06, length + .06, .13),
              (0, 0, HEIGHT - .065, 0, 0, 0), color=(.24, .27, .28, 1))
    for index in range(max(2, int(length))):
        offset = -length / 2 + .5 + index
        if offset >= length / 2:
            break
        for side in (-1, 1):
            yard.part(link, "panel_joint_{}_{}".format(side, index), "box", (.028, .034, HEIGHT - .2),
                      (side * .513, offset, HEIGHT / 2, 0, 0, 0), color=(.26, .29, .29, 1))
    for index in range(max(2, int(length * 2))):
        offset = -length / 2 + .25 + index * .5
        if offset > length / 2 - .1:
            break
        color = (.78, .64, .10, 1) if index % 2 else (.075, .085, .085, 1)
        yard.part(link, "safety_band_" + str(index), "box", (.025, .34, .28),
                  (-.517, offset, .65, 0, 0, 0), color=color)


def pillar(world, name, x, y):
    link = yard.model(world, name, (x, y, yard.GROUND_Z), size=(.8, .8, HEIGHT),
                      label="tall concrete utility pillar")
    yard.part(link, "tall_pillar", "cylinder", (.34, HEIGHT),
              (0, 0, HEIGHT / 2, 0, 0, 0), color=(.43, .46, .46, 1), collision=True)
    yard.part(link, "top_cap", "box", (.8, .8, .10),
              (0, 0, HEIGHT - .05, 0, 0, 0), color=(.25, .28, .28, 1))
    for index, z in enumerate((.3, .6, .9, 1.2)):
        yard.part(link, "hazard_ring_" + str(index), "cylinder", (.345, .14),
                  (0, 0, z, 0, 0, 0), color=(.80, .66, .14, 1))


def in_navigation_region(bounds):
    return (bounds["x"][1] > BOUNDS_XY[0] and bounds["x"][0] < BOUNDS_XY[2]
            and bounds["y"][1] > BOUNDS_XY[1] and bounds["y"][0] < BOUNDS_XY[3])


def distance_to_box(point, bounds):
    dx = max(bounds["x"][0] - point[0], point[0] - bounds["x"][1], 0)
    dy = max(bounds["y"][0] - point[1], point[1] - bounds["y"][1], 0)
    return math.hypot(dx, dy)


def add_collision_bounds(sdf):
    """Keep physical collision extents separate from decorative details."""
    original = {item["id"]: copy.deepcopy(item["aabb_world_m"]) for item in yard.OBJECTS}
    physical = copy.deepcopy(sdf)
    for link in physical.findall("./world/model/link"):
        for visual in list(link.findall("visual")):
            link.remove(visual)
    yard.actual_geometry_bounds(physical)
    for item in yard.OBJECTS:
        item["collision_aabb_world_m"] = item["aabb_world_m"]
        item["aabb_world_m"] = original[item["id"]]


def validate_collision_pairs(sdf):
    components = []
    saved_objects = yard.OBJECTS[:]
    try:
        for source in sdf.findall("./world/model"):
            if source.attrib["name"] == "yard_ground":
                continue
            for selected in source.findall("./link/collision"):
                isolated = ET.Element("sdf")
                world = ET.SubElement(isolated, "world")
                mod = copy.deepcopy(source)
                world.append(mod)
                for link in mod.findall("link"):
                    for visual in list(link.findall("visual")):
                        link.remove(visual)
                    for collision in list(link.findall("collision")):
                        if collision.attrib["name"] != selected.attrib["name"]:
                            link.remove(collision)
                yard.OBJECTS[:] = [{"id": source.attrib["name"]}]
                yard.actual_geometry_bounds(isolated)
                components.append((source.attrib["name"], selected.attrib["name"],
                                   yard.OBJECTS[0]["aabb_world_m"]))
    finally:
        yard.OBJECTS[:] = saved_objects
    fence_joins = 0
    for first, second in itertools.combinations(components, 2):
        if first[0] == second[0]:
            continue
        overlap = all(min(first[2][axis][1], second[2][axis][1])
                      - max(first[2][axis][0], second[2][axis][0]) > 1e-5
                      for axis in ("x", "y", "z"))
        if overlap:
            if first[0].endswith("_fence") and second[0].endswith("_fence"):
                fence_joins += 1
            else:
                raise ValueError("Cross-model collision component overlap: {} / {}".format(first[:2], second[:2]))
    return {"collision_components_checked": len(components),
            "unintended_cross_model_collision_aabb_intersections": 0,
            "intentional_perimeter_fence_corner_joins": fence_joins}


def validate():
    interior = []
    for item in yard.OBJECTS:
        bounds = item["collision_aabb_world_m"]
        active = in_navigation_region(bounds)
        item["inside_navigation_xy"] = active
        if active:
            if bounds["z"][1] < 5.4:
                raise ValueError("Low obstacle in the lidar flight region: " + item["id"])
            interior.append(item)
        # Pad cylinder remains open from the ground to the flight ceiling.
        if distance_to_box((0, 0), bounds) < 1.5 and bounds["z"][0] < 6:
            raise ValueError("Obstacle intrudes into the home takeoff clearance")
    # Sample each segment at <= 2 cm. Distance to a closed box is 1-Lipschitz,
    # so subtracting one full sample step certifies a conservative lower bound.
    minimum = math.inf
    for first, second in zip(WITNESS, WITNESS[1:]):
        length = math.hypot(second[0] - first[0], second[1] - first[1])
        count = max(1, int(math.ceil(length / .02)))
        for index in range(count + 1):
            t = index / count
            point = (first[0] + t * (second[0] - first[0]), first[1] + t * (second[1] - first[1]))
            if not (BOUNDS_XY[0] <= point[0] <= BOUNDS_XY[2] and BOUNDS_XY[1] <= point[1] <= BOUNDS_XY[3]):
                raise ValueError("Feasibility witness leaves navigation region")
            minimum = min(minimum, min(distance_to_box(point, item["aabb_world_m"]) for item in yard.OBJECTS))
    lower_bound = minimum - .02
    if lower_bound < 1.5:
        raise ValueError("Insufficient clearance including 1.2 m inflation and 0.3 m grid buffer")
    return {"xml_parse": True, "all_active_obstacles_cover_lidar_plane": True,
            "home_radius_1_5m_clear": True, "feasible_path_exists": True,
            "witness_sample_step_m": .02, "certified_horizontal_clearance_lower_bound_m": round(lower_bound, 4),
            "planner_inflation_test_m": 1.2, "extra_grid_buffer_lower_bound_m": round(lower_bound - 1.2, 4),
            "witness_hidden_from_agent": True, "active_obstacles": [item["id"] for item in interior]}


def generate():
    old_world = ROOT / "worlds/logistics_yard.world"
    old_manifest = ROOT / "runtime_manifest.json"
    before = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in (old_world, old_manifest)}
    # Reuse the local lighting, ground material definitions and PX4-compatible
    # physics while building a separate model population in memory.
    tree = ET.parse(str(old_world))
    sdf = tree.getroot()
    world = sdf.find("world")
    for mod in list(world.findall("model")):
        world.remove(mod)
    world.find("gui/camera/pose").text = "-16 -24 23 0 .57 .76"
    yard.OBJECTS.clear()
    yard.ground(world)
    wall(world, "obstacle_wall_1", 4.5, -2, 8, "first tall industrial wall")
    wall(world, "obstacle_wall_2", 9.5, 3, 6, "second tall industrial wall")
    # These separately named end stacks seal narrow bypasses at the permitted
    # map boundaries. The two principal wall dimensions remain unchanged.
    wall(world, "south_boundary_cargo_stack", 4.5, -7, 2, "tall southern cargo stack")
    wall(world, "north_boundary_cargo_stack", 9.5, 7, 2, "tall northern cargo stack")
    pillar(world, "utility_pillar_southwest", 1, -6)
    pillar(world, "utility_pillar_northeast", 13, 6.2)
    pillar(world, "utility_pillar_southeast", 14, -6)
    yard.container(world)
    reposition(world, "red_shipping_container", 18.5, 0)
    yard.van(world)
    reposition(world, "white_delivery_van", 14.2, -9.2)
    yard.drum(world, "blue_drum", 16.7, -1.8)
    yard.drum(world, "blue_drum_rear", 16.7, 1.8)
    yard.cone(world, "traffic_cone_1", 16.5, -4.8)
    yard.cone(world, "traffic_cone_2", 17.3, -5.7)
    yard.warehouse(world)
    reposition(world, "brick_warehouse", 15, -13.9)
    yard.office(world)
    reposition(world, "park_office", 16, 12)
    yard.pallet(world, "cartons_pallet_1", 10.95, -9.4)
    yard.pallet(world, "cartons_pallet_2", 17.8, 6.2)
    for index, (x, y, radius) in enumerate([(-13, -9, 1.7), (-13, 7, 1.6), (-8, 14, 1.7),
                                           (5, 13, 1.6), (4, -13, 1.65), (17, 17, 1.25), (7, -15, 1.3)]):
        yard.tree(world, "yard_tree_" + str(index + 1), x, y, radius)
    yard.lamp(world, "street_lamp_south", 17, -7)
    yard.lamp(world, "street_lamp_north", 17, 7)
    yard.fence(world)
    # The new eastern facilities are wider than the old empty foreground;
    # keep the fence at the ground boundary so it does not cut through them.
    reposition(world, "east_fence", 19.94, 0)
    yard.actual_geometry_bounds(sdf)
    add_collision_bounds(sdf)
    validation = validate()
    validation.update(validate_collision_pairs(sdf))
    output = ROOT / "worlds/obstacle_yard.world"
    if hasattr(ET, "indent"):
        ET.indent(sdf, space="  ")
    ET.ElementTree(sdf).write(str(output), encoding="utf-8", xml_declaration=True)
    ET.parse(str(output))
    manifest = {"id": "obstacle_yard", "world_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
                "ground_plane": {"known": True, "z_m": yard.GROUND_Z, "bounds_xy_m": [-20, -20, 20, 20]},
                "flight_region": {"frame": "gazebo_world", "bounds_min_m": [-3, -8, -.2],
                                  "bounds_max_m": [16, 8, 6], "margin_m": .6},
                "takeoff_pad": {"center_world_m": [0, 0, yard.GROUND_Z], "radius_m": 1.0,
                                "vertical_clearance_m": 6},
                "navigation": {"enabled": True, "method": "lidar_grid", "bounds_xy_m": BOUNDS_XY,
                               "flight_altitude_m": 3.5, "minimum_clearance_m": 1.0}}
    (ROOT / "obstacle_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    layout = {"scene_id": "obstacle_yard", "name": "物流园 S 形自主绕障测试场",
              "ground_truth_only": True, "world_sha256": manifest["world_sha256"],
              "ground": manifest["ground_plane"], "navigation_constraints": manifest["navigation"],
              "objects": yard.OBJECTS, "validation": validation,
              "feasibility_witness_world_m": [list(point) for point in WITNESS],
              "recommended_goal_enu_m": [14, 0, 3.5],
              "recommended_instruction": "起飞到3.5米，自主绕开障碍到ENU坐标(14,0,3.5)，到达后悬停观察前方并报告可见物体，再自主返航并降落。",
              "coordinate_note": "ENU and Gazebo world horizontal frames must be aligned from live state; use actual vertical offset when evaluating aircraft altitude."}
    (ROOT / "obstacle_layout.json").write_text(json.dumps(layout, ensure_ascii=False, indent=2) + "\n")
    after = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in (old_world, old_manifest)}
    if before != after:
        raise RuntimeError("Existing world or manifest was modified")
    print("Generated", output)
    print("Validated lidar-visible obstacles and an S-shaped feasible route; certified clearance >=",
          validation["certified_horizontal_clearance_lower_bound_m"], "m.")
    print("Original logistics world and manifest hashes unchanged.")


if __name__ == "__main__":
    generate()
