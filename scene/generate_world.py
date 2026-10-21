#!/usr/bin/env python3
"""Generate a self-contained Gazebo Classic logistics yard, without downloads.

The layout file is validation ground truth. Never feed it to the vision model.
Only the scene identifier and ground projection bounds belong in perception.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
GROUND_Z = 0.003
FREE_REGION = {"x": [-3.0, 8.0], "y": [-4.0, 4.0], "z": [0.0, 6.0]}
OBJECTS = []


def node(parent, tag, text=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if text is not None:
        child.text = str(text)
    return child


def values(items):
    return " ".join("{:.7g}".format(item) for item in items)


def material(visual, name=None, color=None):
    mat = node(visual, "material")
    if name:
        script = node(mat, "script")
        node(script, "uri", "file://" + str(ROOT / "materials/scripts"))
        node(script, "uri", "file://" + str(ROOT / "materials/textures"))
        node(script, "name", "Yard/" + name)
    if color is not None:
        node(mat, "ambient", values(color))
        node(mat, "diffuse", values(color))
        node(mat, "specular", "0.04 0.04 0.04 1")
    return mat


def geometry(parent, shape, dimensions):
    geo = node(parent, "geometry")
    item = node(geo, shape)
    if shape == "box":
        node(item, "size", values(dimensions))
    elif shape == "cylinder":
        node(item, "radius", dimensions[0])
        node(item, "length", dimensions[1])
    elif shape == "sphere":
        node(item, "radius", dimensions[0])
    elif shape == "plane":
        node(item, "normal", "0 0 1")
        node(item, "size", values(dimensions))
    else:
        raise ValueError(shape)


def part(link, name, shape, dimensions, pose=(0, 0, 0, 0, 0, 0),
         texture=None, color=None, collision=False, shadows=True):
    visual = node(link, "visual", name=name)
    node(visual, "pose", values(pose))
    geometry(visual, shape, dimensions)
    node(visual, "cast_shadows", str(shadows).lower())
    material(visual, texture, color)
    if collision:
        coll = node(link, "collision", name=name + "_collision")
        node(coll, "pose", values(pose))
        geometry(coll, shape, dimensions)
        surf = node(coll, "surface")
        friction = node(surf, "friction")
        ode = node(friction, "ode")
        node(ode, "mu", 1.0)
        node(ode, "mu2", 1.0)
    return visual


def model(world, name, position, yaw=0, size=None, label=None, ground_contact=True):
    mod = node(world, "model", name=name)
    node(mod, "static", "true")
    node(mod, "pose", values(list(position) + [0, 0, yaw]))
    link = node(mod, "link", name="structure")
    if size is not None:
        # Conservative, axis-aligned footprint, including visible details.
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        extent_x, extent_y = size[0] * c + size[1] * s, size[0] * s + size[1] * c
        OBJECTS.append({"id": name, "label": label or name,
                        "position_world_m": list(position), "yaw_rad": yaw,
                        "dimensions_model_m": list(size),
                        "aabb_world_m": {"x": [position[0] - extent_x / 2, position[0] + extent_x / 2],
                                         "y": [position[1] - extent_y / 2, position[1] + extent_y / 2],
                                         "z": [position[2], position[2] + size[2]]},
                        "ground_contact": ground_contact})
    return link


def write_texture(name, image):
    if not cv2.imwrite(str(ROOT / "materials/textures" / (name + ".png")), image):
        raise RuntimeError("Cannot write texture " + name)


def textures():
    rng = np.random.RandomState(20261004)
    n = 2048
    noise = rng.normal(0, 12, (n, n, 1))
    asphalt = np.clip(np.array([72, 75, 77])[None, None, :] + noise, 20, 130).astype(np.uint8)
    # Worn gravel and cracks are natural visual features for VIO.
    for _ in range(12000):
        x, y = rng.randint(0, n, 2)
        value = int(rng.randint(35, 145))
        cv2.circle(asphalt, (int(x), int(y)), int(rng.randint(1, 4)), (value, value, value), -1)
    for _ in range(34):
        x, y = rng.randint(0, n, 2)
        points = [(int(x), int(y))]
        for _ in range(int(rng.randint(6, 18))):
            x += rng.randint(-90, 90)
            y += rng.randint(-90, 90)
            points.append((int(x), int(y)))
        cv2.polylines(asphalt, [np.array(points, dtype=np.int32)], False, (35, 37, 39), 2, cv2.LINE_AA)
    write_texture("asphalt", asphalt)

    n = 1024
    grass = np.clip(np.array([48, 88, 63])[None, None, :] + rng.normal(0, 16, (n, n, 1)), 20, 135).astype(np.uint8)
    for _ in range(20000):
        x, y = rng.randint(0, n, 2)
        cv2.line(grass, (int(x), int(y)), (int(x + rng.randint(-2, 3)), int(y + rng.randint(2, 7))),
                 (int(rng.randint(28, 62)), int(rng.randint(62, 118)), int(rng.randint(43, 80))), 1)
    write_texture("grass", grass)

    brick = np.full((1024, 1024, 3), (122, 132, 138), np.uint8)
    for row in range(16):
        for col in range(-1, 8):
            x = col * 128 + (64 if row % 2 else 0)
            y = row * 64
            bgr = np.clip(np.array([74, 91, 126]) + rng.randint(-18, 19, 3), 30, 180)
            cv2.rectangle(brick, (x + 3, y + 3), (x + 125, y + 61), tuple(int(v) for v in bgr), -1)
    brick = np.clip(brick.astype(float) + rng.normal(0, 3, (1024, 1024, 1)), 0, 255).astype(np.uint8)
    write_texture("brick", brick)

    concrete = np.clip(np.array([166, 171, 172])[None, None, :] + rng.normal(0, 8, (1024, 1024, 1)), 110, 210).astype(np.uint8)
    for row in range(0, 1024, 256):
        cv2.line(concrete, (0, row), (1023, row), (124, 126, 127), 3)
    for col in range(0, 1024, 256):
        cv2.line(concrete, (col, 0), (col, 1023), (124, 126, 127), 3)
    write_texture("concrete", concrete)

    wood = np.full((512, 512, 3), (79, 108, 140), np.uint8)
    for _ in range(320):
        y = int(rng.randint(0, 512))
        cv2.line(wood, (0, y), (511, y + int(rng.randint(-5, 6))),
                 (int(rng.randint(55, 100)), int(rng.randint(77, 120)), int(rng.randint(100, 155))), 1)
    write_texture("wood", wood)

    foliage = np.clip(np.array([35, 80, 46])[None, None, :] + rng.normal(0, 19, (512, 512, 1)), 12, 140).astype(np.uint8)
    write_texture("foliage", foliage)

    signs = [("warehouse_sign", "LOGISTICS", "WAREHOUSE  A", (225, 230, 225), (35, 55, 48)),
             ("office_sign", "PARK OFFICE", "RECEPTION", (233, 234, 225), (50, 69, 78))]
    for name, first, second, background, foreground in signs:
        sign = np.full((256, 1024, 3), background, np.uint8)
        cv2.rectangle(sign, (12, 12), (1011, 243), foreground, 6)
        cv2.putText(sign, first, (65, 122), cv2.FONT_HERSHEY_SIMPLEX, 2.35, foreground, 6, cv2.LINE_AA)
        cv2.putText(sign, second, (190, 203), cv2.FONT_HERSHEY_SIMPLEX, 1.5, foreground, 3, cv2.LINE_AA)
        write_texture(name, sign)

    materials = []
    # Asphalt repeats at a plausible 10 m aggregate/crack scale.
    for name, scale in [("asphalt", 0.25), ("grass", 0.2), ("brick", 0.5),
                        ("concrete", 0.4), ("wood", 1), ("foliage", 1),
                        ("warehouse_sign", 1), ("office_sign", 1)]:
        materials.append("""material Yard/%s
{
  technique
  {
    pass
    {
      ambient 0.72 0.72 0.72 1
      diffuse 0.9 0.9 0.9 1
      specular 0.02 0.02 0.02 1
      texture_unit
      {
        texture %s.png
        filtering anisotropic
        max_anisotropy 4
        scale %s %s
      }
    }
  }
}
""" % (name, name, scale, scale))
    (ROOT / "materials/scripts/yard.material").write_text("\n".join(materials))


def ground(world):
    link = model(world, "yard_ground", (0, 0, 0))
    part(link, "solid_ground", "box", (40, 40, .1), (0, 0, GROUND_Z - .05, 0, 0, 0),
         texture="asphalt", collision=True)
    # Lawns and concrete pedestrian surfaces are coplanar; no curb lies in the flight corridor.
    for name, x, y, sx, sy, tex in [("west_lawn", -12.5, 0, 15, 38, "grass"),
                                   ("north_lawn", 8, 16, 20, 6, "grass"),
                                   ("south_lawn", 5, -16, 20, 6, "grass"),
                                   ("warehouse_apron", 14, -8.5, 11, 5, "concrete"),
                                   ("office_walkway", 14, 7.5, 11, 2, "concrete")]:
        part(link, name, "plane", (sx, sy), (x, y, GROUND_Z + .0002, 0, 0, 0),
             texture=tex, shadows=False)
    white = (.85, .85, .79, 1)
    yellow = (.82, .7, .18, 1)
    # Road edges and worn lane dashes are actual road paint, without physical collisions.
    for index, y in enumerate((-6, 6)):
        part(link, "road_edge_" + str(index), "box", (28, .10, .0001),
             (4, y, GROUND_Z + .0003, 0, 0, 0), color=white, shadows=False)
    for index, x in enumerate(np.arange(-9, 18, 4)):
        part(link, "lane_dash_" + str(index), "box", (1.8, .09, .0001),
             (float(x), -5.25, GROUND_Z + .0003, 0, 0, 0), color=white, shadows=False)
    for index, y in enumerate((2.25, 4.95)):
        part(link, "parking_line_" + str(index), "box", (6.4, .07, .0001),
             (13, y, GROUND_Z + .0003, 0, 0, 0), color=white, shadows=False)
    part(link, "parking_back", "box", (.07, 2.77, .0001), (16.2, 3.6, GROUND_Z + .0003, 0, 0, 0),
         color=white, shadows=False)
    # H landing marking at the origin, rather than the old decorative target field.
    for name, px, py, sx, sy in [("h_left", -.65, 0, .18, 1.7), ("h_right", .65, 0, .18, 1.7),
                                ("h_middle", 0, 0, 1.4, .18)]:
        part(link, name, "box", (sx, sy, .0001), (px, py, GROUND_Z + .0003, 0, 0, 0),
             color=white, shadows=False)
    for index, (x, y, sx, sy) in enumerate([(0, -1.6, 3.2, .08), (0, 1.6, 3.2, .08),
                                          (-1.6, 0, .08, 3.2), (1.6, 0, .08, 3.2)]):
        part(link, "pad_border_" + str(index), "box", (sx, sy, .0001),
             (x, y, GROUND_Z + .0003, 0, 0, 0), color=yellow, shadows=False)


def container(world):
    link = model(world, "red_shipping_container", (13, -2, GROUND_Z), math.pi / 2,
                 (6.12, 2.5, 2.66), "red corrugated shipping container")
    red = (.56, .045, .028, 1)
    dark_red = (.39, .025, .018, 1)
    steel = (.34, .35, .35, 1)
    part(link, "body", "box", (6.058, 2.438, 2.591), (0, 0, 1.2955, 0, 0, 0),
         color=red, collision=True)
    # Corrugation on both long sides and roof, modeled as shallow metal ribs.
    for index, x in enumerate(np.linspace(-2.8, 2.8, 22)):
        for side in (-1, 1):
            part(link, "wall_rib_{}_{}".format(index, side), "box", (.055, .038, 2.39),
                 (float(x), side * 1.237, 1.2955, 0, 0, 0), color=dark_red)
        part(link, "roof_rib_" + str(index), "box", (.06, 2.35, .03),
             (float(x), 0, 2.606, 0, 0, 0), color=dark_red)
    for x in (-3.055, 3.055):
        for y in (-1.245, 1.245):
            part(link, "corner_{}_{}".format(x, y), "box", (.09, .09, 2.65),
                 (x, y, 1.325, 0, 0, 0), color=dark_red)
    # Standard double end doors and locking bars.
    for side in (-1, 1):
        part(link, "end_door_" + str(side), "box", (.025, 1.14, 2.40),
             (-3.043, side * .593, 1.29, 0, 0, 0), color=red)
        part(link, "lock_bar_" + str(side), "cylinder", (.018, 2.32),
             (-3.070, side * .52, 1.29, 0, 0, 0), color=steel)
        part(link, "lock_handle_" + str(side), "box", (.045, .21, .025),
             (-3.095, side * .52, 1.10, 0, 0, 0), color=steel)


def van(world):
    link = model(world, "white_delivery_van", (13, 3.5, GROUND_Z), 0,
                 (5.30, 2.22, 2.82), "white delivery van with black wheels")
    white = (.83, .84, .82, 1)
    trim = (.13, .15, .15, 1)
    glass = (.10, .20, .25, 1)
    part(link, "cargo_box", "box", (3.40, 1.96, 2.15), (.65, 0, 1.71, 0, 0, 0),
         color=white, collision=True)
    part(link, "cabin", "box", (1.45, 1.87, 1.64), (-1.725, 0, 1.31, 0, 0, 0),
         color=white, collision=True)
    part(link, "bonnet", "box", (.65, 1.85, .37), (-2.20, 0, .99, 0, .08, 0), color=white)
    part(link, "front_windshield", "box", (.018, 1.63, .70),
         (-2.456, 0, 1.75, 0, -.17, 0), color=glass)
    part(link, "front_bumper", "box", (.16, 1.95, .20), (-2.54, 0, .60, 0, 0, 0), color=trim)
    part(link, "rear_bumper", "box", (.16, 2.0, .18), (2.42, 0, .56, 0, 0, 0), color=trim)
    part(link, "front_grille", "box", (.022, .97, .24), (-2.539, 0, .89, 0, 0, 0), color=trim)
    for side in (-1, 1):
        part(link, "cabin_window_" + str(side), "box", (.97, .021, .69),
             (-1.63, side * .944, 1.75, 0, 0, 0), color=glass)
        part(link, "door_handle_" + str(side), "box", (.18, .035, .045),
             (-1.19, side * .97, 1.26, 0, 0, 0), color=trim)
        part(link, "side_rubstrip_" + str(side), "box", (3.7, .025, .06),
             (.33, side * .996, .94, 0, 0, 0), color=trim)
        part(link, "headlamp_" + str(side), "box", (.031, .30, .18),
             (-2.54, side * .68, 1.02, 0, 0, 0), color=(.93, .93, .76, 1))
        part(link, "rear_lamp_" + str(side), "box", (.025, .12, .26),
             (2.365, side * .85, .98, 0, 0, 0), color=(.66, .028, .015, 1))
        for axle, x in enumerate((-1.65, 1.70)):
            part(link, "wheel_{}_{}".format(side, axle), "cylinder", (.37, .22),
                 (x, side * 1.0, .37, math.pi / 2, 0, 0),
                 color=(.035, .036, .038, 1), collision=True)
            part(link, "wheel_hub_{}_{}".format(side, axle), "cylinder", (.17, .225),
                 (x, side * 1.015, .37, math.pi / 2, 0, 0), color=(.47, .48, .48, 1))
    part(link, "cargo_rear_door_split", "box", (.026, .025, 1.95),
         (2.365, 0, 1.72, 0, 0, 0), color=trim)


def drum(world, name, x, y):
    link = model(world, name, (x, y, GROUND_Z), size=(.69, .69, 1.02), label="blue metal drum")
    part(link, "drum_body", "cylinder", (.32, .94), (0, 0, .48, 0, 0, 0),
         color=(.03, .15, .53, 1), collision=True)
    for index, z in enumerate((.04, .27, .72, .94)):
        part(link, "drum_ring_" + str(index), "cylinder", (.342, .026), (0, 0, z, 0, 0, 0),
             color=(.10, .19, .40, 1))
    part(link, "drum_lid", "cylinder", (.32, .035), (0, 0, .971, 0, 0, 0), color=(.07, .18, .46, 1))
    part(link, "lid_bung", "cylinder", (.043, .018), (.17, 0, .997, 0, 0, 0), color=(.39, .40, .40, 1))


def cone(world, name, x, y):
    link = model(world, name, (x, y, GROUND_Z), size=(.54, .54, .77), label="orange and white traffic cone")
    part(link, "rubber_base", "box", (.54, .54, .065), (0, 0, .0325, 0, 0, 0),
         color=(.07, .07, .065, 1), collision=True)
    # SDF 1.5 has no cone primitive; narrow cylinder slices form its taper.
    for index in range(12):
        z = .09 + index * .053
        radius = .225 - index * .016
        white_band = index in (4, 5, 8, 9)
        color = (.88, .87, .79, 1) if white_band else (.92, .25, .025, 1)
        part(link, "cone_slice_" + str(index), "cylinder", (radius, .056),
             (0, 0, z, 0, 0, 0), color=color)
    # Collision envelope preserves the base footprint and full height.
    coll = node(link, "collision", name="cone_body_collision")
    node(coll, "pose", "0 0 .39 0 0 0")
    geometry(coll, "cylinder", (.23, .73))


def warehouse(world):
    link = model(world, "brick_warehouse", (15, -11, GROUND_Z), size=(8.3, 7.3, 5.45),
                 label="brick warehouse with loading door")
    part(link, "main_building", "box", (8, 7, 5), (0, 0, 2.5, 0, 0, 0),
         texture="brick", collision=True)
    part(link, "flat_roof", "box", (8.25, 7.25, .22), (0, 0, 5.1, 0, 0, 0),
         color=(.20, .21, .21, 1), collision=True)
    part(link, "loading_door", "box", (.032, 3.4, 3.5), (-4.022, 0, 1.75, 0, 0, 0),
         color=(.45, .48, .48, 1))
    for index, z in enumerate(np.linspace(.18, 3.35, 17)):
        part(link, "door_slat_" + str(index), "box", (.036, 3.36, .025),
             (-4.048, 0, float(z), 0, 0, 0), color=(.27, .30, .30, 1))
    for side in (-1, 1):
        part(link, "front_window_frame_" + str(side), "box", (.044, 1.28, 1.15),
             (-4.033, side * 2.6, 3.17, 0, 0, 0), color=(.20, .22, .22, 1))
        part(link, "front_window_" + str(side), "box", (.048, 1.12, .99),
             (-4.064, side * 2.6, 3.17, 0, 0, 0), color=(.12, .25, .30, 1))
    part(link, "warehouse_name", "box", (.028, 4.3, 1.075), (-4.06, 0, 4.35, 0, 0, 0),
         texture="warehouse_sign")
    for index, y in enumerate((-2.1, 2.1)):
        part(link, "dock_bollard_" + str(index), "cylinder", (.10, 1),
             (-4.65, y, .5, 0, 0, 0), color=(.82, .70, .18, 1), collision=True)
    # A separate object is required for its enlarged apron bollards' validation extent.
    OBJECTS[-1]["aabb_world_m"]["x"][0] = 10.24


def office(world):
    link = model(world, "park_office", (16, 11, GROUND_Z), size=(6.25, 6.25, 6.4),
                 label="two-storey office building with windows and entrance")
    part(link, "office_shell", "box", (6, 6, 6.1), (0, 0, 3.05, 0, 0, 0),
         texture="concrete", collision=True)
    part(link, "office_roof", "box", (6.2, 6.2, .18), (0, 0, 6.19, 0, 0, 0),
         color=(.24, .25, .25, 1), collision=True)
    glass = (.10, .24, .30, 1)
    frame = (.18, .20, .21, 1)
    for row, z in enumerate((1.65, 4.45)):
        for col, y in enumerate((-2.1, 0, 2.1)):
            if row == 0 and col == 1:
                continue
            part(link, "front_frame_{}_{}".format(row, col), "box", (.045, 1.62, 1.72),
                 (-3.025, y, z, 0, 0, 0), color=frame)
            part(link, "front_glass_{}_{}".format(row, col), "box", (.05, 1.44, 1.54),
                 (-3.056, y, z, 0, 0, 0), color=glass)
            part(link, "window_mullion_{}_{}".format(row, col), "box", (.055, .06, 1.58),
                 (-3.086, y, z, 0, 0, 0), color=frame)
    part(link, "entrance_frame", "box", (.06, 1.6, 2.45), (-3.036, 0, 1.225, 0, 0, 0), color=frame)
    part(link, "glass_entrance", "box", (.065, 1.4, 2.25), (-3.076, 0, 1.225, 0, 0, 0), color=glass)
    part(link, "door_handle", "box", (.07, .028, .40), (-3.13, -.32, 1.15, 0, 0, 0), color=(.63, .65, .65, 1))
    part(link, "office_name", "box", (.04, 3.9, .98), (-3.07, 0, 3.10, 0, 0, 0), texture="office_sign")
    for side in (-1, 1):
        for col, x in enumerate((-1.8, .4, 2.1)):
            part(link, "side_window_{}_{}".format(side, col), "box", (1.35, .045, 1.60),
                 (x, side * 3.029, 4.4, 0, 0, 0), color=glass)


def pallet(world, name, x, y):
    link = model(world, name, (x, y, GROUND_Z), size=(1.25, 1.05, 1.05),
                 label="wooden pallet with stacked brown cartons")
    part(link, "pallet_collision_base", "box", (1.2, 1, .15), (0, 0, .075, 0, 0, 0),
         texture="wood", collision=True)
    for index, yoff in enumerate(np.linspace(-.43, .43, 5)):
        part(link, "pallet_slat_" + str(index), "box", (1.2, .13, .035),
             (0, float(yoff), .168, 0, 0, 0), texture="wood")
    cardboard = (.56, .40, .23, 1)
    for index, (xoff, yoff, z) in enumerate([(-.29, -.24, .43), (.29, -.24, .43),
                                          (-.29, .24, .43), (.29, .24, .43), (0, 0, .87)]):
        part(link, "carton_" + str(index), "box", (.54, .44, .40),
             (xoff, yoff, z, 0, 0, 0), color=cardboard, collision=True)
        part(link, "packing_tape_" + str(index), "box", (.075, .442, .008),
             (xoff, yoff, z + .205, 0, 0, 0), color=(.72, .58, .36, 1))


def tree(world, name, x, y, radius=1.55):
    link = model(world, name, (x, y, GROUND_Z), size=(radius * 2.1, radius * 2.1, 5.65),
                 label="leafy tree")
    part(link, "trunk", "cylinder", (.16, 3.2), (0, 0, 1.6, 0, 0, 0), texture="wood", collision=True)
    part(link, "crown", "sphere", (radius,), (0, 0, 3.95, 0, 0, 0), texture="foliage", collision=True)
    part(link, "crown_top", "sphere", (radius * .75,), (.25, .06, 4.60, 0, 0, 0), texture="foliage")


def lamp(world, name, x, y):
    link = model(world, name, (x, y, GROUND_Z), size=(1.1, .60, 6.35), label="yard street lamp")
    metal = (.26, .29, .28, 1)
    part(link, "lamp_pole", "cylinder", (.075, 6.15), (0, 0, 3.075, 0, 0, 0), color=metal, collision=True)
    part(link, "lamp_head", "box", (.82, .36, .13), (-.26, 0, 6.17, 0, 0, 0), color=metal)
    part(link, "lamp_glass", "box", (.67, .28, .015), (-.26, 0, 6.098, 0, 0, 0),
         color=(.82, .82, .66, 1), shadows=False)


def fence(world):
    for name, center, dims in [("north_fence", (0, 19, GROUND_Z), (38, .12, 1.8)),
                              ("south_fence", (0, -19, GROUND_Z), (38, .12, 1.8)),
                              ("east_fence", (19, 0, GROUND_Z), (.12, 38, 1.8)),
                              ("west_fence", (-19, 0, GROUND_Z), (.12, 38, 1.8))]:
        link = model(world, name, center, size=dims, label="perimeter metal fence")
        horizontal = dims[0] > dims[1]
        # Simple rail fence is visible and collision-solid, with efficient geometry.
        collision = node(link, "collision", name="fence_collision")
        node(collision, "pose", "0 0 .9 0 0 0")
        geometry(collision, "box", dims)
        for i in range(20):
            v = -19 + 2 * i
            pose = (v, 0, .9, 0, 0, 0) if horizontal else (0, v, .9, 0, 0, 0)
            part(link, "fence_post_" + str(i), "box", (.08, .08, 1.8), pose,
                 color=(.31, .34, .33, 1))
        for i, z in enumerate((.48, 1.35)):
            size = (38, .04, .055) if horizontal else (.04, 38, .055)
            part(link, "fence_rail_" + str(i), "box", size, (0, 0, z, 0, 0, 0),
                 color=(.35, .38, .37, 1))


def pose_rotation(pose):
    roll, pitch, yaw = pose[3:]
    cr, cp, cy = math.cos(roll), math.cos(pitch), math.cos(yaw)
    sr, sp, sy = math.sin(roll), math.sin(pitch), math.sin(yaw)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ \
        np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ \
        np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def actual_geometry_bounds(sdf):
    """Use actual serialized visual + collision extents for evaluation truth."""
    records = {item["id"]: item for item in OBJECTS}
    for mod in sdf.findall("./world/model"):
        if mod.attrib["name"] not in records:
            continue
        p = np.array([float(value) for value in mod.findtext("pose", "0 0 0 0 0 0").split()])
        rm = pose_rotation(p)
        minimum = np.full(3, np.inf)
        maximum = np.full(3, -np.inf)
        for link in mod.findall("link"):
            lp = np.array([float(value) for value in link.findtext("pose", "0 0 0 0 0 0").split()])
            rl = pose_rotation(lp)
            for element in link.findall("collision") + link.findall("visual"):
                q = np.array([float(value) for value in element.findtext("pose", "0 0 0 0 0 0").split()])
                rotation = rm @ rl @ pose_rotation(q)
                center = p[:3] + rm @ (lp[:3] + rl @ q[:3])
                geo = element.find("geometry")
                if geo.find("box") is not None:
                    half = .5 * np.array([float(value) for value in geo.findtext("box/size").split()])
                    extent = np.abs(rotation) @ half
                elif geo.find("sphere") is not None:
                    extent = np.full(3, float(geo.findtext("sphere/radius")))
                elif geo.find("cylinder") is not None:
                    radius = float(geo.findtext("cylinder/radius"))
                    half = float(geo.findtext("cylinder/length")) / 2
                    axis = rotation[:, 2]
                    extent = half * np.abs(axis) + radius * np.sqrt(np.maximum(0, 1 - axis * axis))
                else:
                    raise ValueError("Unexpected obstacle geometry")
                minimum = np.minimum(minimum, center - extent)
                maximum = np.maximum(maximum, center + extent)
        records[mod.attrib["name"]]["aabb_world_m"] = {
            axis: [round(float(minimum[index]), 6), round(float(maximum[index]), 6)]
            for index, axis in enumerate(("x", "y", "z"))}
        records[mod.attrib["name"]]["bounds_source"] = "actual_sdf_visual_and_collision_union"


def validate_layout():
    for obj in OBJECTS:
        bounds = obj["aabb_world_m"]
        overlap = all(bounds[axis][1] > FREE_REGION[axis][0] and bounds[axis][0] < FREE_REGION[axis][1]
                      for axis in ("x", "y", "z"))
        if overlap:
            raise ValueError("Object intersects free flight region: " + obj["id"])
        # All physical obstacles are at least 2 m away horizontally from this corridor.
        dx = max(FREE_REGION["x"][0] - bounds["x"][1], bounds["x"][0] - FREE_REGION["x"][1], 0)
        dy = max(FREE_REGION["y"][0] - bounds["y"][1], bounds["y"][0] - FREE_REGION["y"][1], 0)
        if math.hypot(dx, dy) < 2:
            raise ValueError("Obstacle clearance under 2 m: " + obj["id"])


def generate():
    OBJECTS.clear()
    for directory in (ROOT / "materials/scripts", ROOT / "materials/textures", ROOT / "worlds"):
        directory.mkdir(parents=True, exist_ok=True)
    textures()
    sdf = ET.Element("sdf", {"version": "1.5"})
    world = node(sdf, "world", name="default")
    include = node(world, "include")
    node(include, "uri", "model://sun")
    scene = node(world, "scene")
    node(scene, "ambient", ".54 .56 .58 1")
    node(scene, "background", ".66 .76 .85 1")
    node(scene, "shadows", "true")
    fog = node(scene, "fog")
    node(fog, "type", "linear")
    node(fog, "color", ".72 .78 .83 1")
    node(fog, "density", ".001")
    node(fog, "start", "70")
    node(fog, "end", "150")
    physics = node(world, "physics", name="default_physics", default="0", type="ode")
    node(physics, "gravity", "0 0 -9.8066")
    ode = node(physics, "ode")
    solver = node(ode, "solver")
    for name, value in [("type", "quick"), ("iters", 10), ("sor", 1.3), ("use_dynamic_moi_rescaling", 0)]:
        node(solver, name, value)
    constraints = node(ode, "constraints")
    for name, value in [("cfm", 0), ("erp", .2), ("contact_max_correcting_vel", 100), ("contact_surface_layer", .001)]:
        node(constraints, name, value)
    for name, value in [("max_step_size", .004), ("real_time_factor", 1), ("real_time_update_rate", 250),
                        ("magnetic_field", "6.0e-6 2.3e-5 -4.2e-5")]:
        node(physics, name, value)
    gui = node(world, "gui", fullscreen="0")
    camera = node(gui, "camera", name="yard_overview")
    node(camera, "pose", "-17 -22 19 0 .48 .78")
    node(camera, "view_controller", "orbit")
    ground(world)
    container(world)
    van(world)
    drum(world, "blue_drum", 10.8, .7)
    drum(world, "blue_drum_rear", 10.8, 1.6)
    cone(world, "traffic_cone_1", 10.5, -4.8)
    cone(world, "traffic_cone_2", 12, -5.7)
    warehouse(world)
    office(world)
    pallet(world, "cartons_pallet_1", 10.95, -6.9)
    pallet(world, "cartons_pallet_2", 13.25, -6.9)
    for i, (x, y, r) in enumerate([(-13, -9, 1.7), (-13, 7, 1.6), (-8, 14, 1.7),
                                   (5, 13, 1.6), (4, -13, 1.65), (17, 17, 1.25), (17, -17, 1.3)]):
        tree(world, "yard_tree_" + str(i + 1), x, y, r)
    lamp(world, "street_lamp_south", 12, -6.4)
    lamp(world, "street_lamp_north", 12, 6.4)
    fence(world)
    actual_geometry_bounds(sdf)
    validate_layout()
    if hasattr(ET, "indent"):
        ET.indent(sdf, space="  ")
    output = ROOT / "worlds/logistics_yard.world"
    ET.ElementTree(sdf).write(str(output), encoding="utf-8", xml_declaration=True)
    ET.parse(str(output))
    runtime = {"id": "logistics_yard", "world_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
               "ground_plane": {"known": True, "z_m": GROUND_Z,
                                "bounds_xy_m": [-20, -20, 20, 20]},
               "flight_region": {"frame": "gazebo_world", "bounds_min_m": [-3, -4, -.2],
                                 "bounds_max_m": [8, 4, 6], "margin_m": .6}}
    (ROOT / "runtime_manifest.json").write_text(json.dumps(runtime, indent=2) + "\n")
    tasks = [
        {"id": "container_inspection", "waypoints_enu_m": [[5, -2, 3.5]],
         "natural_language": "起飞到 3.5 米，飞到 ENU 坐标 (5,-2,3.5)，保持位置观察前方，寻找红色货运集装箱，报告它的位置方向和周围物体，然后返航并降落。"},
        {"id": "delivery_vehicle_inspection", "waypoints_enu_m": [[5, 3, 3.5], [7, 3, 3.5]],
         "natural_language": "起飞到 3.5 米，先去 ENU (5,3,3.5) 观察白色货车及停车区，再沿原高度到 (7,3,3.5) 重新观察并比较两次结果，返航降落。"},
        {"id": "two_station_patrol", "waypoints_enu_m": [[5, -2, 3.5], [7, 0, 3.0]],
         "natural_language": "起飞到 3.5 米，按 ENU (5,-2,3.5) 和 (7,0,3) 两个站点巡检，每站悬停拍摄。第一站找红色集装箱和锥桶，第二站找蓝色圆桶并描述附近的车辆、建筑；全部观察结束后返航降落。"},
    ]
    layout = {"scene_id": "logistics_yard_v1", "name": "小型物流园区",
              "world_file": str(output), "offline_assets": True,
              "ground_truth_only": True,
              "ground": {"frame": "gazebo_world", "z_m": GROUND_Z,
                         "bounds_m": {"x": [-20, 20], "y": [-20, 20]},
                         "surface": "coplanar asphalt, grass and concrete"},
              "free_region_world_m": FREE_REGION,
              "minimum_obstacle_clearance_m": 2,
              "home_world_m": [0, 0, GROUND_Z],
              "coordinate_note": "Candidate ENU routes assume simulator spawn at (0,0,0), zero heading; confirm live MAVROS home alignment before flight.",
              "camera": {"heading_body": "+X", "pitch_down_rad": .35, "horizontal_fov_rad": math.pi / 2},
              "objects": OBJECTS, "task_candidates": tasks,
              "validation": {"xml_parse": True, "obstacle_free_region": True, "horizontal_clearance_at_least_2m": True}}
    (ROOT / "layout.json").write_text(json.dumps(layout, ensure_ascii=False, indent=2) + "\n")
    print("Generated", output)
    print("Validated", len(OBJECTS), "static physical objects; flight corridor is clear with >=2 m obstacle clearance.")


if __name__ == "__main__":
    generate()
