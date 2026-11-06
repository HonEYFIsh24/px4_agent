#!/usr/bin/env python3
"""Offline NBV execution audit; no flight, provider calls or semantic ground truth."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'scripts'))
from stereo_localization import vector, finite
from mission import MAX_ALTITUDE
from search_limits import validate_grid


def snapshot_revision(snapshot):
    return hashlib.sha256(json.dumps(snapshot, ensure_ascii=False, sort_keys=True,
                                     allow_nan=False).encode()).hexdigest()


def audit_record(record, load_snapshot, segment_safe, require_semantic_guidance=False, require_vlm_selection=False,
                 require_diverse_pool=False, require_auto_prepare=False, load_version=None):
    """Check archived execution, not model prose. Callbacks supply immutable evidence."""
    command = record['command']
    predictions = command.get('nbv_events') or []
    accepted = command.get('flight_missions') or []
    completed = command.get('flight_mission_results') or []
    report = {'actual_flight': False, 'new_model_calls': 0,
              'scope': 'archived NBV plans, endpoint observations and ownership; not continuous measured trajectory',
              'semantic_labels_independently_verified': False, 'checks': {}, 'moves': [], 'problems': []}
    checks = report['checks']
    checks['task_succeeded'] = command.get('status') == 'succeeded'
    checks['finished_grounded_disarmed'] = (record.get('flight', {}).get('armed') is False
                                          and record.get('flight', {}).get('landed_state') == 1)
    checks['has_nbv_predictions'] = bool(predictions)
    checks['public_predictions_hide_tickets'] = 'execution_ticket' not in json.dumps(
        [predictions, command.get('nbv_candidate_events', [])])
    reports = [r for r in command.get('perceptions', []) if r.get('status') == 'ok']
    capture_ids = [r['capture_id'] for r in reports]
    checks['unique_saved_captures'] = bool(capture_ids) and len(capture_ids) == len(set(capture_ids))
    checked_ids, batches = set(), set()
    for prediction in predictions:
        if prediction.get('selected_id') is None:
            continue
        try:
            owner, batch, revision = prediction['owner'], prediction['batch_id'], prediction['map_revision']
            if (not isinstance(owner, list) or len(owner) != 2 or owner[1] != command['id']
                    or not all(isinstance(v, str) and v for v in owner) or batch in batches):
                raise ValueError('prediction task ownership or batch is ambiguous')
            batches.add(batch)
            source_id = prediction['source_capture_id']
            index = capture_ids.index(source_id)
            source = load_snapshot(source_id)
            if snapshot_revision(source) != revision and load_version is not None:
                source = load_version(revision)
            validate_grid(source['grid'])
            radius = source['grid']['radius_m']
            identity = prediction['identity']
            if (set(identity) != {'world_sha256', 'sortie_id', 'coordinate_epoch', 'registration'}
                    or source['owner'] != owner or snapshot_revision(source) != revision
                    or any(source[k] != identity[k] for k in ('world_sha256', 'sortie_id', 'coordinate_epoch', 'registration'))
                    or prediction['config']['mode'] not in ('geometry', 'semantic')
                    or prediction['search_mode'] != 'nbv_'+prediction['config']['mode']):
                raise ValueError('source map revision, identity or mode differs from prediction')
            candidates = [c for c in prediction['ranked_candidates'] if c['id'] == prediction['selected_id']]
            if len(candidates) != 1 or candidates[0].get('path_verified') is not True:
                raise ValueError('selected ID is not uniquely path-verified')
            selector = prediction.get('selector', 'deterministic')
            if selector not in ('deterministic', 'vlm'):
                raise ValueError('unknown candidate selector')
            from viewpoint_planner import ViewpointConfig, ViewpointPlanner, viewpoint_group
            pool_config = ViewpointConfig(**prediction['config'])
            expected_pool = ViewpointPlanner(pool_config).candidate_pool(
                prediction['ranked_candidates'], prediction.get('reference', {}), fill=True)[:8]
            if 'candidate_pool_ids' in prediction and prediction['candidate_pool_ids'] != expected_pool:
                raise ValueError('declared candidate pool differs from verified representatives')
            if pool_config.candidate_pool_mode == 'diverse' and (
                    type(prediction.get('path_evaluation_count')) is not int
                    or not 0 <= prediction['path_evaluation_count'] <= pool_config.max_path_evaluations):
                raise ValueError('diverse pool exceeded its shared path budget')
            if selector == 'vlm':
                offers = [e for e in command.get('nbv_candidate_events', []) if e.get('batch_id') == batch]
                if (len(offers) != 1 or offers[0].get('selection_status') != 'awaiting_vlm'
                        or offers[0].get('selected_id') is not None
                        or any(offers[0].get(k) != prediction.get(k) for k in
                               ('owner', 'identity', 'map_revision', 'source_capture_id', 'ranked_candidates',
                                'offered_candidate_ids', 'numeric_best_id', 'config', 'candidate_pool_ids'))
                        or not 1 <= len(prediction['offered_candidate_ids']) <= 8
                        or prediction['offered_candidate_ids'] != expected_pool
                        or prediction['selected_id'] not in prediction['offered_candidate_ids']
                        or prediction.get('selection_status') != 'selected_by_vlm'
                        or prediction.get('numeric_best_id') != prediction['ranked_candidates'][0]['id']):
                    raise ValueError('VLM decision changed the offered candidate batch')
                choices = [e['result'] for e in command.get('search_events', [])
                           if e.get('tool') == 'select_target_search_view'
                           and e.get('result', {}).get('batch_id') == batch]
                if (len(choices) != 1 or choices[0].get('status') != 'ok'
                        or choices[0].get('candidate_id') != prediction['selected_id']
                        or choices[0].get('selection_reason') != prediction.get('selection_reason')
                        or not prediction.get('selection_reason')
                        or not any(t.get('status') == 'ok' and t.get('image_purpose') == 'nbv_candidate_selection'
                            and 'image' in t.get('input_modalities', []) and source_id in t.get('capture_ids', [])
                            and any(c.get('name') == 'select_target_search_view' for c in t.get('called_tools', []))
                            for t in command.get('model_calls', []))):
                    raise ValueError('VLM decision lacks its selection result or owned source-image delivery')
            selected = candidates[0]
            goal = vector(selected['position_enu_m'], 3, 'selected goal')
            owned = [item for item in accepted if
                     (item.get('mission', {}).get('search_constraint') or {}).get('batch_id') == batch
                     and item['mission'].get('viewpoint_candidate_id') == selected['id']]
            if len(owned) != 1 or owned[0].get('accepted_via') != '/navigation/execute-viewpoint':
                raise ValueError('selected candidate lacks a unique ticket-accepted mission')
            mission_id = owned[0]['mission']['id']
            results = [item['mission'] for item in completed if item.get('mission', {}).get('id') == mission_id]
            if len(results) != 1 or results[0].get('status') != 'succeeded' or mission_id in checked_ids:
                raise ValueError('ticket mission lacks unique successful completion evidence')
            checked_ids.add(mission_id)
            mission = results[0]
            scope = mission['search_constraint']
            origin = vector(source['grid']['origin_xy_enu_m'], 2, 'search origin')
            expected_steps = [{'action': 'goto', 'x': goal[0], 'y': goal[1], 'altitude': goal[2]}]
            heading_step = selected.get('heading_step')
            if heading_step is not None:
                from yaw_control import error
                target_yaw = finite(selected['target_yaw_enu_rad'], 'target heading')
                source_yaw = finite(prediction['reference']['fixed_yaw_enu_rad'], 'source heading')
                if (pool_config.heading_mode != 'discrete' or type(heading_step) is not int or heading_step not in (-1, 1)
                        or abs(error(target_yaw, source_yaw+heading_step*math.pi/4)) > 1e-9):
                    raise ValueError('selected heading is not a valid server-derived discrete candidate')
                expected_steps.append({'action': 'rotate', 'yaw_rad': target_yaw})
            elif 'target_yaw_enu_rad' in selected:
                raise ValueError('selected candidate has an unbound heading')
            if (scope != owned[0]['mission']['search_constraint'] or scope.get('revoked') is not False
                    or scope['owner'] != owner or scope['identity'] != identity
                    or scope['map_revision'] != revision or scope['batch_id'] != batch
                    or scope['origin_xy_enu_m'] != origin or scope['search_radius_m'] != radius
                    or mission.get('viewpoint_candidate_id') != selected['id']
                    or mission['steps'] != expected_steps or owned[0]['mission']['steps'] != expected_steps):
                raise ValueError('actual mission changed candidate, identity or search permission')
            plans = mission.get('navigation_plans') or []
            if not plans:
                raise ValueError('completed mission has no actual navigation plan')
            length = 0.
            for plan_index, entry in enumerate(plans):
                plan = entry['plan']
                points = [vector(plan['start_enu_m'], 3, 'actual start')]
                points += [vector(p, 3, 'actual waypoint') for p in plan['waypoints']]
                if (len(points) < 2 or plan.get('source') != 'verified_static_collision_map_3d'
                        or plan.get('route_complete') is not True
                        or plan.get('full_route_within_search_volume') is not True
                        or plan.get('search_origin_xy_enu_m') != origin or plan.get('search_radius_m') != radius
                        or (plan_index == 0 and math.dist(points[0],
                            vector(prediction['reference']['start_enu_m'], 3, 'predicted source')) > .35)
                        or math.dist(vector(entry['goal_enu_m'], 3, 'actual goal'), goal) > 1e-8
                        or math.dist(vector(plan['goal_enu_m'], 3, 'plan goal'), goal) > 1e-8
                        or math.dist(points[-1], goal) > 1e-8
                        or any(not 1.-1e-9 <= p[2] <= MAX_ALTITUDE+1e-9
                               or math.dist(p[:2], origin) > radius+1e-9 for p in points)
                        or any(not segment_safe(identity, a, b) for a, b in zip(points, points[1:]))):
                    raise ValueError('actual plan violates candidate, volume or registered collision geometry')
                current_length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
                if not math.isclose(current_length, finite(plan['path_length_m'], 'actual path length'), rel_tol=1e-9, abs_tol=1e-9):
                    raise ValueError('actual plan length differs from its vertices')
                length += current_length
            arrivals = [e for e in mission.get('events', []) if e.get('action') == 'goto']
            if len(arrivals) != 1 or math.dist(vector(arrivals[0]['position_enu_m'], 3, 'arrival'), goal) > .4:
                raise ValueError('completed candidate has no matching actual arrival')
            after = load_snapshot(capture_ids[index+1])
            observation = after['observations'][-1]
            if heading_step is not None:
                from stereo_localization import quaternion_rotation
                rotations = [e.get('rotation') for e in mission.get('events', []) if e.get('action') == 'rotate']
                if len(rotations) != 1 or not isinstance(rotations[0], dict):
                    raise ValueError('heading candidate lacks unique actual rotation arrival')
                arrival = rotations[0]
                actual = finite(arrival['actual_rad'], 'actual arrival heading')
                stamp = finite(arrival['stamp_s'], 'arrival timestamp')
                attitude = quaternion_rotation(observation['orientation_xyzw'])
                image_yaw = math.atan2(attitude[1][0], attitude[0][0])
                if (arrival.get('status') != 'reached' or abs(error(target_yaw, finite(arrival['target_rad'], 'arrival target'))) > 1e-9
                        or abs(error(target_yaw, actual)) > .1 or abs(error(target_yaw, image_yaw)) > .12
                        or finite(observation['stamp_sim_s'], 'post-arrival capture stamp') <= stamp):
                    raise ValueError('heading arrival or post-rotation image is inconsistent')
            if (after['owner'] != owner or observation['capture_id'] != capture_ids[index+1]
                    or any(after[k] != identity[k] for k in identity)
                    or math.dist(vector(observation['position_enu_m'], 3, 'next observer'), goal) > .4
                    or [o['capture_id'] for o in after['observations']] !=
                       [o['capture_id'] for o in source['observations']]+[capture_ids[index+1]]):
                raise ValueError('next actual observation does not extend the same map at the executed candidate')
            if after['grid'] != source['grid'] or after['eligible_cell_ids'] != source['eligible_cell_ids']:
                raise ValueError('map grid or eligible region changed after the candidate')
            before_cells = dict(enumerate(source['cells']))
            after_cells = dict(enumerate(after['cells']))
            if before_cells.keys() != after_cells.keys() or any(after_cells[k]['coverage'] < c['coverage'] for k, c in before_cells.items()):
                raise ValueError('geometric coverage history shrank or changed grid')
            area = finite(source['grid']['resolution_m'], 'grid resolution')**2
            gain = sum(after_cells[k]['coverage']-c['coverage'] for k, c in before_cells.items())*area
            report['moves'].append({'batch_id': batch, 'candidate_id': selected['id'], 'mission_id': mission_id,
                'source_capture_id': source_id, 'next_capture_id': capture_ids[index+1],
                'selector': selector, 'numeric_best_id': prediction.get('numeric_best_id'),
                'prepared_by': prediction.get('prepared_by', 'next_target_search_view'),
                'candidate_pool_mode': pool_config.candidate_pool_mode,
                'offered_groups': sorted({viewpoint_group(p, prediction['reference']) for p in prediction['ranked_candidates']
                                         if p['id'] in expected_pool}) if
                    {'start_enu_m', 'fixed_yaw_enu_rad'} <= prediction.get('reference', {}).keys() else [],
                'predicted_coverage_gain_m2': selected['coverage_gain_m2'],
                'heading_step': heading_step,
                'realized_geometric_gain_m2': gain, 'actual_planned_length_m': length,
                'nonzero_semantic_guidance': (prediction['config']['mode'] == 'semantic'
                    and source.get('semantic_field_active') is True
                    and finite(selected.get('semantic_gain_m2', 0.), 'semantic gain') > 0.
                    and finite(selected.get('score_terms', {}).get('semantic', 0.), 'semantic score term') > 0.)})
        except (ValueError, KeyError, IndexError, TypeError, OSError) as exc:
            report['problems'].append(str(exc))
    ticket_ids = {item['mission']['id'] for item in accepted if item.get('accepted_via') == '/navigation/execute-viewpoint'}
    checks['has_executed_nbv_move'] = bool(checked_ids) and bool(report['moves'])
    checks['all_ticket_missions_accounted_for'] = bool(ticket_ids) and ticket_ids == checked_ids
    checks['all_selected_moves_verified'] = not report['problems']
    checks['no_unscoped_search_goto'] = all(
        not any(s.get('action') == 'goto' for s in item['mission'].get('steps', []))
        or item.get('accepted_via') == '/navigation/execute-viewpoint' for item in accepted)
    report['semantic_guided_move_count'] = sum(m['nonzero_semantic_guidance'] for m in report['moves'])
    if require_semantic_guidance:
        checks['nonzero_semantic_guidance_executed'] = report['semantic_guided_move_count'] > 0
    report['vlm_selected_move_count'] = sum(m['selector'] == 'vlm' for m in report['moves'])
    if require_vlm_selection:
        checks['vlm_selection_executed'] = (report['vlm_selected_move_count'] > 0
                                          and report['vlm_selected_move_count'] == len(report['moves']))
    if require_diverse_pool:
        checks['diverse_pool_executed'] = bool(report['moves']) and all(m['candidate_pool_mode'] == 'diverse' for m in report['moves'])
        checks['rising_and_multiple_groups_offered'] = any(len(m['offered_groups']) >= 3
            and any(g.startswith('rise:') for g in m['offered_groups']) for m in report['moves'])
    if require_auto_prepare:
        checks['report_preparation_executed'] = bool(report['moves']) and all(
            m['prepared_by'] in ('report_scene', 'review_target_candidate') for m in report['moves'])
        checks['no_redundant_next_view_model_calls'] = not any(
            t.get('name') == 'next_target_search_view' for call in command.get('model_calls', [])
            for t in call.get('called_tools', []))
    report['passed'] = all(checks.values())
    return report


def audit(result_file, output, world, require_semantic_guidance=False, require_vlm_selection=False,
          require_diverse_pool=False, require_auto_prepare=False):
    from audit_semantic_map_evidence import audit as audit_semantic
    from scene_config import load_scene
    from static_navigation import StaticNavigator
    record = json.loads(result_file.read_text())
    directory = Path(record.get('capture_directory') or result_file.parent/'captures')
    output.mkdir(parents=True, exist_ok=False)
    nav = StaticNavigator.from_scene(world, load_scene(world, navigation_mode='static_3d'))

    def load(identifier):
        if Path(identifier).name != identifier or identifier in ('.', '..'):
            raise ValueError('invalid capture basename')
        return json.loads((directory/(identifier+'.semantic-map.json')).read_text())

    def safe(identity, a, b):
        if nav.world_sha256 != identity['world_sha256']:
            return False
        registration = identity['registration']
        nav.origin = vector(registration['world_from_enu_origin_m'], 3, 'registered origin')
        nav.yaw = finite(registration['world_from_enu_yaw_rad'], 'registered yaw')
        return nav.segment_safe(a, b)

    def load_version(revision):
        if not isinstance(revision, str) or len(revision) != 64 or any(c not in '0123456789abcdef' for c in revision):
            raise ValueError('Invalid map revision digest')
        value = json.loads((directory/('map-'+revision+'.snapshot.json')).read_text())
        if snapshot_revision(value) != revision:
            raise ValueError('Planning map snapshot does not match its SHA256 revision')
        return value

    result = audit_record(record, load, safe, require_semantic_guidance, require_vlm_selection,
                          require_diverse_pool, require_auto_prepare, load_version=load_version)
    semantic = audit_semantic(result_file, output/'semantic')
    required = ['task_succeeded', 'finished_grounded_disarmed', 'map_events_all_valid',
                'map_events_cover_successful_reports', 'replay_has_all_successful_reports',
                'online_replay_equal', 'duplicate_rejected_without_mutation']
    result['checks']['image_report_map_replay_verified'] = all(semantic['checks'].get(k) is True for k in required)
    result['passed'] = all(result['checks'].values())
    result['source'] = str(result_file.resolve())
    (output/'nbv-evidence-audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--require-semantic-guidance', action='store_true')
    parser.add_argument('--require-vlm-selection', action='store_true')
    parser.add_argument('--require-diverse-pool', action='store_true')
    parser.add_argument('--require-auto-prepare', action='store_true')
    args = parser.parse_args()
    try:
        result = audit(args.result, args.output, args.world, args.require_semantic_guidance, args.require_vlm_selection,
                       args.require_diverse_pool, args.require_auto_prepare)
    except (ValueError, KeyError, OSError) as exc:
        print('NBV evidence audit rejected:', str(exc), file=sys.stderr)
        raise SystemExit(1)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result['passed'] else 2)
