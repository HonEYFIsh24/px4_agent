"""Deterministic camera-relative translation; no LLM or ROS dependencies."""
import math

from mission import MissionError, MAX_ALTITUDE
from search_tools import coordinate_epoch
from stereo_localization import iris_left_optical_to_body, matvec, quaternion_rotation, vector


def validate_photo_rejection(result, request, state):
    """Validate the executor's no-submission contract against the real request/state."""
    if (result.get('status') != 'not_executed' or result.get('mission_submitted') is not False
            or 'mission' in result or result.get('reason_code') != 'photo_path_unreachable'
            or request.get('operation') != 'adjust' or result.get('operation') != 'adjust'
            or result.get('expected_mission_id') != request.get('expected_mission_id')
            or result.get('expected_epoch') != request.get('expected_epoch')
            or not isinstance(result.get('reason'), str) or not result['reason']):
        raise MissionError('invalid photo candidate rejection evidence')
    require_photo_navigation(state, request['expected_mission_id'], request['expected_epoch'])


def camera_directions(quaternion):
    rotation = quaternion_rotation(quaternion)
    optical = matvec(rotation, matvec(iris_left_optical_to_body()['rotation'], [0., 0., 1.]))
    norm = math.hypot(optical[0], optical[1])
    if norm < .2:
        raise MissionError('camera horizontal direction is undefined near nadir')
    forward = [optical[0]/norm, optical[1]/norm]
    return {'forward_xy_enu': forward, 'right_xy_enu': [forward[1], -forward[0]],
            'optical_axis_enu': optical, 'reference': 'current_ENU_body_attitude_and_fixed_camera_mount'}


def relative_photo_goal(position, quaternion, direction, distance_m, altitude):
    position = vector(position, 3, 'current ENU position')
    if direction not in ('forward', 'backward', 'left', 'right', 'vertical'):
        raise MissionError('unknown relative photo direction')
    if type(distance_m) not in (int, float) or not math.isfinite(distance_m) or not 0 <= distance_m <= 10:
        raise MissionError('relative photo distance must be finite and within 0..10m per adjustment')
    if type(altitude) not in (int, float) or not math.isfinite(altitude) or not 1 <= altitude <= MAX_ALTITUDE:
        raise MissionError('photo altitude outside current mission limits')
    if (direction == 'vertical') != (distance_m == 0):
        raise MissionError('vertical requires zero horizontal distance; other directions require positive distance')
    axes = camera_directions(quaternion)
    axis = axes['forward_xy_enu'] if direction in ('forward', 'backward') else axes['right_xy_enu']
    sign = -1 if direction in ('backward', 'left') else 1
    goal = [position[i] + sign*distance_m*axis[i] for i in range(2)] + [altitude]
    return goal, axes


def relative_photo_yaw(quaternion, direction, angle_deg):
    """Resolve a bounded horizontal turn from current ENU body orientation.

    ENU positive yaw turns left. The fixed camera rotates with the body; this
    changes its horizontal bearing, not its downward pitch or mount offset.
    """
    if direction not in ('left', 'right'):
        raise MissionError('photo turn direction must be left or right')
    if type(angle_deg) not in (int, float) or not math.isfinite(angle_deg) or not 5 <= angle_deg <= 90:
        raise MissionError('photo turn angle must be finite and within 5..90 degrees')
    rotation = quaternion_rotation(quaternion)
    axes = camera_directions(quaternion)
    if math.hypot(rotation[0][0], rotation[1][0]) < .2:
        raise MissionError('body horizontal heading is undefined')
    yaw = math.atan2(rotation[1][0], rotation[0][0])
    delta = math.radians(angle_deg) * (1 if direction == 'left' else -1)
    target = math.atan2(math.sin(yaw+delta), math.cos(yaw+delta))
    return target, dict(source_yaw_enu_rad=yaw, yaw_delta_rad=delta, **axes)


def require_photo_navigation(state, expected_mission_id, expected_epoch):
    """No relaxation for framing failures: invalid pose/reference cannot return home."""
    mission = state.get('mission') or {}
    ev = state.get('external_vision') or {}
    nav = state.get('navigation') or {}
    mapping = nav.get('map', nav)
    epoch = coordinate_epoch(state)
    if (not isinstance(expected_mission_id, str) or not expected_mission_id
            or mission.get('id') != expected_mission_id or mission.get('status') != 'succeeded'):
        raise MissionError('photo navigation requires the same completed owned mission')
    if (state.get('simulation_only') is not True or state.get('ready') is not True
            or state.get('connected') is not True or state.get('armed') is not True
            or state.get('landed_state') != 2 or state.get('mode') != 'OFFBOARD'):
        raise MissionError('photo navigation requires healthy airborne OFFBOARD state')
    if (not isinstance(epoch, dict) or epoch.get('vio_epoch') is None or epoch.get('ekf_counters') is None
            or expected_epoch != epoch or ev.get('healthy') is not True
            or ev.get('used_for_flight_control') is not True or ev.get('frame_reset_latched')):
        raise MissionError('photo navigation coordinate identity or VIO is invalid')
    if (nav.get('enabled') is not True or nav.get('active') is not True or nav.get('method') != 'static_3d'
            or mapping.get('map_registered') is not True
            or mapping.get('source') != 'verified_static_collision_map_3d'):
        raise MissionError('photo navigation requires registered static 3D navigation')
    vector(state.get('home_enu_m'), 3, 'actual takeoff home ENU')
    vector(state.get('position_enu_m'), 3, 'current ENU position')
