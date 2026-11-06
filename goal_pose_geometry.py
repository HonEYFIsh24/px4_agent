"""Versioned terminal-pose adapter; Python 3.8, no model or learned dependencies.

Reuses StaticViewpointEvaluator and its A*, volume and optical occlusion checks.
This explicit interface permits a program-generated continuous yaw; the old NBV
interface retains its original discrete ±45-degree contract.
"""
import copy
import math
import re

from mission import MissionError
from stereo_localization import finite, vector


def validate_goal_context(context, active):
    keys = {'schema_version', 'stage_id', 'stage_revision', 'plan_revision',
            'target_id', 'target_revision', 'relation', 'terminal_action'}
    if not isinstance(context, dict) or set(context) != keys or context['schema_version'] != 1:
        raise MissionError('invalid versioned goal context')
    if (not active or active.get('revoked') or context['stage_id'] != active.get('stage_id')
            or context['stage_revision'] != active.get('stage_revision')
            or any(type(context[k]) is not int or context[k] < 1 for k in
                   ('stage_revision', 'plan_revision', 'target_revision'))
            or not isinstance(context['target_id'], str)
            or not re.fullmatch(r'(target_\d{2}|bearing_\d{3})', context['target_id'])
            or context['relation'] not in ('object_front', 'object_behind', 'object_near', 'object_beside',
                                           'body_left', 'body_right', 'facing_target', 'route_forward')
            or context['terminal_action'] not in ('approach', 'relative_hover', 'relative_land', 'relative_turn', 'face_target', 'pass_landmark')):
        raise MissionError('goal stage, target revision or relation is not bound to this owner')
    pairs = {'relative_turn': ('body_left', 'body_right'), 'face_target': ('facing_target',),
             'pass_landmark': ('route_forward',), 'approach': ('object_front', 'object_behind', 'object_near', 'object_beside'),
             'relative_hover': ('object_front', 'object_behind', 'object_near', 'object_beside'),
             'relative_land': ('object_front', 'object_behind', 'object_near', 'object_beside')}
    if context['relation'] not in pairs[context['terminal_action']]:
        raise MissionError('goal relation does not match its authorized action type')
    return copy.deepcopy(context)


def query_goal_poses(evaluator, operation, origin, identity, candidates, goal_context=None):
    if (operation not in ('context', 'paths') or not isinstance(candidates, list)
            or len(candidates) > 8 or operation == 'context' and candidates):
        raise MissionError('invalid goal-pose query batch')
    response = evaluator.query('context', origin, identity, [])
    response.update(source='verified_static_goal_pose_v1', schema_version=1)
    seen = set()
    for c in candidates:
        if not isinstance(c, dict) or set(c) != {'id', 'position_enu_m', 'yaw_enu_rad'}:
            raise MissionError('goal candidate needs only id, program position and yaw')
        if not re.fullmatch(r'goal_\d{3}', str(c['id'])) or c['id'] in seen:
            raise MissionError('invalid or duplicate goal candidate ID')
        seen.add(c['id'])
        point = vector(c['position_enu_m'], 3, 'goal pose')
        yaw = finite(c['yaw_enu_rad'], 'goal yaw')
        if not -math.pi <= yaw <= math.pi:
            raise MissionError('goal yaw must be wrapped ENU radians')
        temporary = {'id': 'view_001', 'position_enu_m': point}
        result = evaluator.query('paths', origin, identity, [temporary])['results'][0]
        result.update(id=c['id'], target_yaw_enu_rad=yaw, target_visible=False)
        view_required = not goal_context or goal_context['terminal_action'] not in ('relative_turn', 'face_target', 'pass_landmark')
        result['target_visibility_required'] = view_required
        if result['reachable'] and view_required:
            predicted = copy.copy(evaluator)
            predicted.yaw = yaw
            visibility = predicted.query('visibility', origin, identity, [temporary])['results'][0]
            probes = visibility.get('probe_visibility') or []
            result['target_visible'] = bool(probes) and all(
                p['inside_fov_and_range'] and p['unoccluded'] for p in probes)
            result['probe_visibility'] = probes
            if not result['target_visible']:
                result.update(reachable=False, reason='goal target is not geometrically visible at the actual camera yaw')
        elif result['reachable']:
            result.update(target_visible=None, visibility_basis='not_used_for_checked_heading_or_route_plane_action')
        response['results'].append(result)
    return response
