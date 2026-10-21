"""Verified scene geometry; no target identities or target poses reach the Agent."""
import hashlib
import json
import math
from pathlib import Path

from mission import MissionError, number

ROOT = Path(__file__).resolve().parent
BUILTIN_WORLD = ROOT / "scene/worlds/logistics_yard.world"
DEFAULT_MANIFEST = ROOT / "scene/runtime_manifest.json"
OBSTACLE_WORLD = ROOT / "scene/worlds/obstacle_yard.world"
OBSTACLE_MANIFEST = ROOT / "scene/obstacle_manifest.json"
RELIABILITY_SCENE_ROOT = ROOT / "scene/reliability"


def reliability_manifest(world, digest):
    """Recognize only explicitly registered, hash-matched local test assets.

    The registry contains paths and hashes, never evaluation geometry. External
    world files still cannot acquire these navigation assumptions by supplying
    an arbitrary manifest.
    """
    directory = RELIABILITY_SCENE_ROOT.resolve()
    if world.parent != directory:
        return None
    index = directory / "index.json"
    if not index.is_file():
        return None
    registry = json.loads(index.read_text())
    if (type(registry.get("schema_version")) is not int or registry["schema_version"] != 1
            or not isinstance(registry.get("scenes"), list)):
        raise MissionError("invalid reliability scene registry")
    matches = []
    for entry in registry["scenes"]:
        if not isinstance(entry, dict):
            raise MissionError("invalid reliability scene entry")
        paths = {}
        for name in ("world", "manifest"):
            value = entry.get(name)
            if (not isinstance(value, str) or value in ("", ".", "..")
                    or Path(value).name != value or Path(value).is_absolute()):
                raise MissionError("reliability scene paths must be local relative paths")
            paths[name] = (directory / value).resolve()
            if paths[name].parent != directory:
                raise MissionError("reliability scene path escapes registered directory")
        if paths["world"] == world:
            if entry.get("world_sha256") != digest:
                return None
            assets = entry.get("asset_sha256") or {}
            if (assets.get("world") != digest or not paths["manifest"].is_file()
                    or assets.get("manifest") != hashlib.sha256(paths["manifest"].read_bytes()).hexdigest()):
                return None
            matches.append(paths["manifest"])
    if len(matches) > 1:
        raise MissionError("ambiguous reliability scene registration")
    return matches[0] if matches else None


def load_scene(world_path=None, manifest_path=None):
    if not world_path:
        return {"id": "unconfigured", "verified": False, "ground_plane": {"known": False}}
    world = Path(world_path).resolve()
    digest = hashlib.sha256(world.read_bytes()).hexdigest()
    unknown = {"id": world.stem, "world_sha256": digest, "verified": False,
               "ground_plane": {"known": False}, "flight_region": None}
    # An arbitrary external manifest must not confer the built-in map assumptions.
    trusted = {Path(BUILTIN_WORLD).resolve(): Path(DEFAULT_MANIFEST).resolve(),
               Path(OBSTACLE_WORLD).resolve(): Path(OBSTACLE_MANIFEST).resolve()}
    expected_manifest = trusted.get(world)
    reliability = False
    if expected_manifest is None:
        expected_manifest = reliability_manifest(world, digest)
        reliability = expected_manifest is not None
    if expected_manifest is None or Path(manifest_path or expected_manifest).resolve() != expected_manifest:
        return unknown
    manifest = json.loads(expected_manifest.read_text())
    if manifest.get("world_sha256") != digest:
        return unknown
    ground, region = manifest["ground_plane"], manifest["flight_region"]
    z = number(ground["z_m"], "ground z")
    bounds = [number(value, "ground bounds") for value in ground["bounds_xy_m"]]
    minimum = [number(value, "flight minimum") for value in region["bounds_min_m"]]
    maximum = [number(value, "flight maximum") for value in region["bounds_max_m"]]
    margin = number(region["margin_m"], "flight clearance", .4, 2)
    if (ground.get("known") is not True or len(bounds) != 4 or not bounds[0] < bounds[2]
            or not bounds[1] < bounds[3] or len(minimum) != 3 or len(maximum) != 3
            or any(a >= b for a, b in zip(minimum, maximum)) or region.get("frame") != "gazebo_world"):
        raise MissionError("invalid built-in scene geometry")
    result = {"id": manifest["id"], "world_sha256": digest, "verified": True,
            "ground_plane": {"known": True, "z_m": z, "bounds_xy_m": bounds},
            "flight_region": {"frame": "gazebo_world", "bounds_min_m": minimum,
                              "bounds_max_m": maximum, "margin_m": margin}}
    if world == Path(OBSTACLE_WORLD).resolve() or reliability:
        navigation = manifest["navigation"]
        nav_bounds = [number(v, "navigation bounds") for v in navigation["bounds_xy_m"]]
        altitude = number(navigation["flight_altitude_m"], "navigation altitude", 1, 5)
        clearance = number(navigation["minimum_clearance_m"], "navigation clearance", 1, 2)
        if (navigation.get("enabled") is not True or navigation.get("method") != "lidar_grid"
                or len(nav_bounds) != 4 or nav_bounds != [minimum[0], minimum[1], maximum[0], maximum[1]]):
            raise MissionError("invalid autonomous navigation scene contract")
        pad = manifest["takeoff_pad"]
        center = [number(v, "takeoff pad") for v in pad["center_world_m"]]
        if len(center) != 3:
            raise MissionError("invalid takeoff pad")
        result["navigation"] = {"enabled": True, "method": "lidar_grid", "bounds_xy_m": nav_bounds,
                                "flight_altitude_m": altitude, "minimum_clearance_m": clearance,
                                "scope": "static horizontal obstacles at fixed altitude; no 3D or dynamic guarantee"}
        result["takeoff_pad"] = {"center_world_m": center,
                                 "radius_m": number(pad["radius_m"], "takeoff pad radius", .5, 2),
                                 "vertical_clearance_m": number(pad["vertical_clearance_m"], "pad clearance", 5, 10)}
    return result


def quaternion_yaw(values):
    if len(values) != 4:
        raise MissionError("scene frame alignment has invalid orientation")
    x, y, z, w = [number(v, "orientation") for v in values]
    if abs(math.sqrt(x*x+y*y+z*z+w*w)-1) > .025:
        raise MissionError("scene frame alignment has invalid quaternion")
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def validate_scene_trajectory(scene, steps, start_enu, home_enu, world_pose, local_orientation):
    """Check spatial permission; navigation scenes separately require sensed clearance.

    Legacy clear-corridor scenes also guarantee their straight segments are clear.
    The autonomous scene contains obstacles: this function provides NO obstacle map.
    """
    region = scene.get("flight_region")
    if not scene.get("verified") or not region:
        return
    yaw = quaternion_yaw(world_pose["orientation_xyzw"]) - quaternion_yaw(local_orientation)
    c, s = math.cos(yaw), math.sin(yaw)
    start = [number(v, "start position") for v in start_enu]
    world_start = [number(v, "world position") for v in world_pose["position_m"]]
    if len(start) != 3 or len(world_start) != 3:
        raise MissionError("scene frame alignment has invalid position")
    offset = [world_start[0]-c*start[0]+s*start[1], world_start[1]-s*start[0]-c*start[1], world_start[2]-start[2]]
    minimum, maximum, margin = region["bounds_min_m"], region["bounds_max_m"], region["margin_m"]

    def check(position):
        point = [c*position[0]-s*position[1]+offset[0], s*position[0]+c*position[1]+offset[1], position[2]+offset[2]]
        if (any(not minimum[i]+margin <= point[i] <= maximum[i]-margin for i in (0, 1))
                or not minimum[2] <= point[2] <= maximum[2]-.2):
            raise MissionError("flight path leaves this scene's permitted flight region")
        return point

    point, home = list(start), list(home_enu) if home_enu is not None else None
    check(point)
    for step in steps:
        action = step["action"]
        if action == "takeoff":
            if scene.get("navigation", {}).get("enabled"):
                pad = scene["takeoff_pad"]
                world_point = check(point)
                if math.hypot(world_point[0]-pad["center_world_m"][0], world_point[1]-pad["center_world_m"][1]) > pad["radius_m"]-.2:
                    raise MissionError("autonomous navigation takeoff requires the verified clear pad")
            home = list(point)
            point[2] = step["altitude"]
        elif action == "goto":
            point = [step["x"], step["y"], step["altitude"]]
        elif action == "return_home":
            if home is None:
                raise MissionError("scene cannot verify return path without recorded home")
            point = [home[0], home[1], point[2]]
        elif action == "land":
            # AUTO.LAND preserves the verified horizontal position; ground contact
            # lies in the legacy clear corridor. Autonomous-scene experiments
            # return to the verified pad; emergency descent elsewhere depends
            # on this scene's full-height obstacle columns, not a general 3-D map.
            # Contact and disarming remain observed by the existing executor.
            check(point)
            continue
        check(point)
