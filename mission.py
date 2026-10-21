"""Shared mission contract, compatible with both Python 3.8 and 3.11."""
import math

MIN_ALTITUDE = 1.0
MAX_ALTITUDE = 10.0
MAX_RADIUS = 30.0
MAX_STEPS = 24
MAX_SPEED = 1.0


class MissionError(ValueError):
    pass


def number(value, name, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionError("{} must be a number".format(name))
    value = float(value)
    if not math.isfinite(value):
        raise MissionError("{} must be finite".format(name))
    if (low is not None and value < low) or (high is not None and value > high):
        raise MissionError("{} must be between {} and {}".format(name, low, high))
    return value


def validate_mission(steps, airborne=False, home_known=False):
    """Validate the entire sortie before accepting any flight command."""
    if not isinstance(steps, list) or not 1 <= len(steps) <= MAX_STEPS:
        raise MissionError("steps must contain 1..{} actions".format(MAX_STEPS))
    allowed = {
        "takeoff": {"altitude"},
        "goto": {"x", "y", "altitude"},
        "hold": {"duration"},
        "return_home": set(),
        "land": set(),
    }
    result = []
    has_takeoff = False
    for index, raw in enumerate(steps):
        if not isinstance(raw, dict):
            raise MissionError("each step must be an object")
        step = {k: v for k, v in raw.items() if v is not None}
        action = step.get("action")
        if not isinstance(action, str) or action not in allowed:
            raise MissionError("unknown action: {}".format(action))
        if set(step) != allowed[action] | {"action"}:
            raise MissionError("{} requires exactly these fields: {}".format(
                action, sorted(allowed[action] | {"action"})))
        if action in ("takeoff", "goto"):
            step["altitude"] = number(step["altitude"], "altitude", MIN_ALTITUDE, MAX_ALTITUDE)
        if action == "takeoff":
            if airborne or has_takeoff:
                raise MissionError("takeoff requires a landed vehicle; one takeoff per mission")
            airborne, home_known, has_takeoff = True, True, True
        elif action in ("goto", "hold", "return_home"):
            if not airborne:
                raise MissionError("{} requires takeoff first or an airborne vehicle".format(action))
            if action == "goto":
                step["x"] = number(step["x"], "x")
                step["y"] = number(step["y"], "y")
                if math.hypot(step["x"], step["y"]) > MAX_RADIUS:
                    raise MissionError("waypoint exceeds {} m radius".format(MAX_RADIUS))
            if action == "hold":
                step["duration"] = number(step["duration"], "duration", 0.1, 60.0)
            if action == "return_home" and not home_known:
                raise MissionError("takeoff position is unknown; cannot return home")
        elif action == "land":
            if index != len(steps) - 1:
                raise MissionError("land must be the last action")
            airborne = False
        result.append(step)
    return result
