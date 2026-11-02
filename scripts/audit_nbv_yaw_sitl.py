#!/usr/bin/env python3
"""Audit saved actual camera/mission evidence for one yaw NBV executor test."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from yaw_control import error


def heading(q):
    if not isinstance(q, list) or len(q) != 4 or any(type(x) not in (int, float) or not math.isfinite(x) for x in q):
        raise ValueError('invalid actual image quaternion')
    if not .99 <= math.sqrt(sum(x*x for x in q)) <= 1.01:
        raise ValueError('actual image quaternion is not normalized')
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def audit(result_path, output):
    record = json.loads(result_path.read_text())
    before, after = record['images']['before'], record['images']['after']
    candidate, rotation = record['candidate'], record['rotation']
    initial, terminal, landing = record['accepted_mission'], record['viewpoint_terminal'], record['landing_terminal']
    steps, route = initial['steps'], candidate['path']
    points = [route['start_enu_m']] + route['waypoints']
    origin, radius = route['search_origin_xy_enu_m'], route['search_radius_m']
    ids = record['owned_mission_ids']
    events = [e for e in terminal.get('events', []) if e.get('action') == 'rotate']
    landing_events = [e for e in landing.get('events', []) if e.get('action') == 'land']
    evidence = (landing_events[-1].get('landing_fcu_evidence') or {}) if landing_events else {}
    checks = {
        'all_runtime_checks': record['status'] == 'passed' and bool(record['checks']) and all(v is True for v in record['checks'].values()),
        'three_owned_missions': len(ids) == 3 and len(set(ids)) == 3 and terminal['id'] == initial['id'] == ids[1] and landing['id'] == ids[2],
        'ticket_bound_steps': initial.get('viewpoint_candidate_id') == candidate['id'] and len(steps) == 2
            and steps[0] == {'action': 'goto', 'x': candidate['position_enu_m'][0], 'y': candidate['position_enu_m'][1],
                             'altitude': candidate['position_enu_m'][2]}
            and steps[1] == {'action': 'rotate', 'yaw_rad': candidate['target_yaw_enu_rad']},
        'server_discrete_heading': candidate.get('heading_step') == 1
            and abs(error(candidate['target_yaw_enu_rad'], record['source_yaw_enu_rad']+math.pi/4)) < 1e-9,
        'verified_bounded_route': route.get('source') == 'verified_static_collision_map_3d'
            and route.get('route_complete') is True and route.get('full_route_within_search_volume') is True
            and radius == 12. and all(1.-1e-9 <= p[2] <= 10.+1e-9 and math.dist(p[:2], origin) <= radius+1e-9 for p in points)
            and math.dist(points[-1], candidate['position_enu_m']) < 1e-8,
        'actual_rotation_arrival': terminal.get('status') == 'succeeded' and len(events) == 1
            and events[0].get('rotation') == rotation and abs(error(candidate['target_yaw_enu_rad'], rotation['actual_rad'])) <= .1,
        'fresh_actual_camera_pose': before['coordinate_epoch'] == after['coordinate_epoch']
            and before['stamp_sim_s'] < rotation['stamp_s'] < after['stamp_sim_s']
            and abs(error(candidate['target_yaw_enu_rad'], heading(after['body_pose_enu']['orientation_xyzw']))) <= .12
            and abs(error(heading(after['body_pose_enu']['orientation_xyzw']), heading(before['body_pose_enu']['orientation_xyzw']))) >= math.radians(30.),
        'landed_disarmed': landing.get('status') == 'succeeded' and record['checks'].get('landed_disarmed') is True
            and bool(evidence),
    }
    for name, image in (('before', before), ('after', after)):
        checks[name+'_image_sha256'] = hashlib.sha256((result_path.parent/(name+'.jpg')).read_bytes()).hexdigest() == image['sha256']
    result = {'passed': all(checks.values()), 'checks': checks,
              'limitation': 'One hand-selected candidate; not an autonomous NBV benchmark, VLM recognition test or proof of continuous trajectory.'}
    output.mkdir(parents=True, exist_ok=False)
    (output/'nbv-yaw-audit.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); result = audit(args.result, args.output)
    print(json.dumps(result, indent=2)); sys.exit(0 if result['passed'] else 2)
