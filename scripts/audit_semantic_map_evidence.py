#!/usr/bin/env python3
"""Audit archived real observations without flying, calling a model or reading target truth."""
import argparse
import contextlib
import copy
import io
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
from replay_semantic_map import replay
from semantic_map import SemanticMapError


def compare_cells(online, replayed):
    """Only derived semantic exp scores allow libm/Python rounding differences."""
    if not isinstance(online, list) or len(online) != len(replayed):
        return False, None
    maximum = 0.
    valid = True
    for a, b in zip(online, replayed):
        if not isinstance(a, dict) or a.keys() != b.keys():
            return False, None
        for key in b:
            if key != 'semantic_score':
                valid = valid and a[key] == b[key]
                continue
            x, y = a[key], b[key]
            if (type(x) not in (int, float) or type(y) not in (int, float)
                    or not math.isfinite(x) or not math.isfinite(y)):
                return False, None
            maximum = max(maximum, abs(x-y))
            valid = valid and math.isclose(x, y, rel_tol=1e-12, abs_tol=1e-15)
    return valid, maximum


def audit(result_file, output):
    record = json.loads(result_file.read_text())
    command = record['command']
    with contextlib.redirect_stdout(io.StringIO()):
        semantic_map = replay(result_file, output, audit_only=True)
    report = {'source': str(result_file.resolve()), 'actual_flight': False,
              'new_model_calls': 0, 'semantic_labels_independently_verified': False,
              'command_status': command['status'], 'checks': {}, 'evidence': {}}
    checks = report['checks']
    checks['task_succeeded'] = command['status'] == 'succeeded'
    checks['finished_grounded_disarmed'] = (record.get('flight', {}).get('armed') is False
        and record.get('flight', {}).get('landed_state') == 1)
    events = command.get('semantic_map_events') or []
    checks['map_events_all_valid'] = bool(events) and all(e.get('status') == 'valid' for e in events)
    saved_reports = [r for r in command.get('perceptions', []) if r.get('status') == 'ok']
    checks['map_events_cover_successful_reports'] = bool(saved_reports) and len(events) == len(saved_reports) and sorted(
        e.get('capture_id', '') for e in events) == sorted(r.get('capture_id', '') for r in saved_reports)
    checks['replay_has_all_successful_reports'] = bool(semantic_map) and len(semantic_map.observations) == sum(
        r.get('status') == 'ok' for r in command.get('perceptions', []))
    if semantic_map:
        snapshot = semantic_map.snapshot()
        directory = Path(record.get('capture_directory') or result_file.parent / 'captures')
        latest = snapshot['observations'][-1]['capture_id']
        online_file = directory / (latest + '.semantic-map.json')
        online = json.loads(online_file.read_text()) if online_file.exists() else None
        compared = ('cells', 'observations', 'landmarks', 'registration', 'coordinate_epoch',
                    'geometric_coverage_fraction', 'world_sha256', 'sortie_id', 'grid', 'query')
        report['evidence']['online_replay_comparison'] = {
            key: online is not None and online.get(key) == snapshot[key] for key in compared}
        report['evidence']['online_replay_exact_comparison'] = dict(report['evidence']['online_replay_comparison'])
        cells_equal, max_error = compare_cells(online.get('cells') if online else None, snapshot['cells'])
        report['evidence']['online_replay_comparison']['cells'] = cells_equal
        report['evidence']['semantic_score_max_abs_error'] = max_error
        report['evidence']['semantic_score_tolerance'] = {'relative': 1e-12, 'absolute': 1e-15,
            'scope': 'derived cell semantic_score only; IDs, evidence, coverage and all other fields remain exact'}
        checks['online_replay_equal'] = all(report['evidence']['online_replay_comparison'].values())
        first_id = snapshot['observations'][0]['capture_id']
        archive = json.loads((directory / (first_id + '.json')).read_text())
        geometry = json.loads((directory / (first_id + '.geometry.json')).read_text())
        original = next(r for r in command['perceptions'] if r.get('capture_id') == first_id)
        frame = {'owner': semantic_map.owner, 'delivered': True, 'capture_id': first_id,
                 'snapshot': archive['snapshot']}
        before = copy.deepcopy(semantic_map.snapshot())
        try:
            semantic_map.update_observation(frame, original, geometry)
        except SemanticMapError as exc:
            checks['duplicate_rejected_without_mutation'] = ('Duplicate capture' in str(exc)
                and semantic_map.snapshot() == before)
        else:
            checks['duplicate_rejected_without_mutation'] = False
        landmarks = snapshot['landmarks']
        multiview = [l for l in landmarks if l.get('position_enu_m') is not None
                     and l.get('attributes', {}).get('distinct_viewpoint_count', 0) >= 2]
        bearing = [l for l in landmarks if l['status'] == 'unlocalized']
        localized = [l for l in landmarks if l.get('position_enu_m') is not None]
        checks['real_multiview_geometric_evidence'] = bool(multiview)
        checks['explicit_semantic_field_active'] = snapshot['semantic_field_active']
        checks['unlocalized_clues_have_no_xyz_and_have_bearing'] = bool(bearing) and all(
            l['position_enu_m'] is None and l['covariance_enu_m2'] is None
            and all(len(o['bearing_direction_enu']) == 3 for o in l['observations']) for l in bearing)
        report['evidence'].update(context=semantic_map.get_context(), localized=[{
            'id': l['id'], 'label': l['label'], 'status': l['status'],
            'position_enu_m': l['position_enu_m'], 'covariance_enu_m2': l['covariance_enu_m2'],
            'distinct_viewpoints': l['attributes']['distinct_viewpoint_count'],
            'capture_ids': [o['capture_id'] for o in l['observations']]} for l in localized],
            unlocalized_clue_count=len(bearing),
            insufficiently_observed_cell_count=len(semantic_map.insufficiently_observed_cells()),
            unobserved_semantic_regions=semantic_map.semantic_regions(),
            remaining_coverage_gain_m2=semantic_map.coverage_gain(sorted(semantic_map.eligible)),
            remaining_semantic_gain=semantic_map.semantic_gain(sorted(semantic_map.eligible)))
    output.mkdir(parents=True, exist_ok=True)
    (output / 'evidence-audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-multiview', action='store_true')
    parser.add_argument('--require-semantic-field', action='store_true')
    parser.add_argument('--require-bearing', action='store_true')
    args = parser.parse_args()
    try:
        result = audit(args.result, args.output)
    except (ValueError, KeyError, OSError) as exc:
        print('Evidence audit rejected:', str(exc), file=sys.stderr)
        raise SystemExit(1)
    print(json.dumps({'checks': result['checks'], 'output': str(args.output)}, ensure_ascii=False, indent=2))
    required = ['task_succeeded', 'finished_grounded_disarmed', 'map_events_all_valid',
                'map_events_cover_successful_reports', 'replay_has_all_successful_reports',
                'online_replay_equal', 'duplicate_rejected_without_mutation']
    if args.require_multiview: required.append('real_multiview_geometric_evidence')
    if args.require_semantic_field: required.append('explicit_semantic_field_active')
    if args.require_bearing: required.append('unlocalized_clues_have_no_xyz_and_have_bearing')
    raise SystemExit(0 if all(result['checks'].get(k) is True for k in required) else 2)
