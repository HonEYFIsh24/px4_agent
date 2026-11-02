#!/usr/bin/env python3
"""Audit paired-image review provenance and geometry; does not establish class truth."""
import argparse
import base64
import hashlib
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from semantic_map import point_in_box
from target_confirmation import observation_for, observations_for, pair_geometry, review_crop


def audit(result_file, output, require_confirmed=False, require_rejected=False):
    record = json.loads(result_file.read_text())
    command = record['command']
    directory = Path(record.get('capture_directory') or result_file.parent/'captures')
    captures = [p['capture_id'] for p in command.get('perceptions', []) if p.get('status') == 'ok']
    reviews, problems = [], []
    for event in command.get('target_confirmation_events', []):
        if event.get('outcome') not in ('confirmed', 'rejected', 'unconfirmed'):
            continue
        try:
            active = event['candidate']
            source, current = active['source_capture_id'], active['current_capture_id']
            for identifier in (source, current):
                if Path(identifier).name != identifier or identifier in ('.', '..'):
                    raise ValueError('Invalid review capture basename')
            if captures.index(source) >= captures.index(current):
                raise ValueError('Review images are not ordered actual captures')
            snapshot = json.loads((directory/(current+'.semantic-map.json')).read_text())
            if event['owner'] != snapshot['owner'] or event['owner'][1] != command['id']:
                raise ValueError('Review owner differs from map/task')
            semantic_map = SimpleNamespace(landmarks={l['id']: SimpleNamespace(**l) for l in snapshot['landmarks']})
            source_landmark, original = observation_for(semantic_map, active['target_id'], source)
            actual_views = {o['capture_id']: o for o in snapshot['observations']}
            if (actual_views[current]['stamp_sim_s'] <= actual_views[source]['stamp_sim_s']
                    or math.dist(actual_views[current]['position_enu_m'], actual_views[source]['position_enu_m']) < .5):
                raise ValueError('Review lacks a newer actual view with at least 0.5m baseline')
            candidates = observations_for(semantic_map, event['query'], current)
            if active['current_target_ids'] != [l.id for l, _ in candidates]:
                raise ValueError('Current candidates differ from saved visual evidence')
            policy = event.get('geometry_policy', 'center_v1')
            matches = [(l, pair_geometry(source_landmark, original, l, o, policy)) for l, o in candidates]
            matches = [(l, g) for l, g in matches if g['compatible']]
            if policy in ('clipped_regions_v2', 'clipped_regions_v3') and len(matches) == 1:
                landmark, _ = matches[0]
                current_observation = next(o for l, o in candidates if l.id == landmark.id)
                competitors = [l.id for l, o in observations_for(semantic_map, event['query'], source)
                               if pair_geometry(l, o, landmark, current_observation, policy)['compatible']]
                if competitors != [source_landmark.id]:
                    matches = []
            if original['position_enu_m'] is not None and sum(
                    point_in_box(original['position_enu_m'], o) for _, o in candidates) > 1:
                matches = []
            compatible = (len(matches) == 1 and matches[0][0].id == event['current_target_id'])
            flags = [event[k] for k in ('source_supported', 'current_supported', 'same_object')]
            uncertain = [event.get(k, False) for k in ('source_uncertain', 'current_uncertain')]
            if (any(type(v) is not bool for v in flags+uncertain)
                    or (flags[0] and uncertain[0]) or (flags[1] and uncertain[1])):
                raise ValueError('Review flags are not booleans')
            strong = compatible and not (policy == 'clipped_regions_v3' and matches[0][1].get('association_hypothesis_only', False))
            outcome = 'rejected' if not flags[0] and not uncertain[0] else 'confirmed' if all(flags) and strong else 'unconfirmed'
            if outcome != event['outcome']:
                raise ValueError('Review outcome differs from visual flags and recomputed geometry')
            calls = [c for c in command.get('model_calls', []) if c.get('status') == 'ok'
                and c.get('image_purpose') == 'target_confirmation' and c.get('capture_ids') == [source, current]
                and 'image' in c.get('input_modalities', [])
                and any(t.get('name') == 'review_target_candidate' for t in c.get('called_tools', []))]
            if not calls:
                raise ValueError('Review lacks actual paired model image delivery')
            archive = json.loads((directory/(source+'.json')).read_text())
            raw = (directory/(source+'.jpg')).read_bytes()
            digest = hashlib.sha256(raw).hexdigest()
            if digest != archive['snapshot']['stereo_image_sha256']['left']:
                raise ValueError('Review source JPEG hash differs from capture')
            frame = {'capture_id': source, 'camera': {'image_sha256': digest},
                     'image_base64': base64.b64encode(raw).decode()}
            crop = review_crop(frame, original['bbox'])
            for call in calls:
                if (call['completed_at'] < next(p['finished_at'] for p in command['perceptions']
                        if p['capture_id'] == current)):
                    raise ValueError('Paired review preceded the saved current report')
                metadata = call['derived_roi']
                if any(metadata.get(k) != v for k, v in crop.items() if k != 'image_base64'):
                    raise ValueError('Delivered ROI provenance differs from recomputed source crop')
                expected_filename = source+'.'+current+'.confirmation-roi.jpg'
                if metadata.get('archive_filename') != expected_filename or hashlib.sha256(
                        (directory/expected_filename).read_bytes()).hexdigest() != crop['crop_sha256']:
                    raise ValueError('Archived ROI differs from actual delivered crop')
                if policy in ('clipped_regions_v2', 'clipped_regions_v3'):
                    current_archive = json.loads((directory/(current+'.json')).read_text())
                    current_raw = (directory/(current+'.jpg')).read_bytes()
                    current_hash = hashlib.sha256(current_raw).hexdigest()
                    if current_hash != current_archive['snapshot']['stereo_image_sha256']['left']:
                        raise ValueError('Current review JPEG hash differs from capture')
                    rois = call.get('current_rois', [])
                    if [r['target_id'] for r in rois] != [l.id for l, _ in candidates]:
                        raise ValueError('Current candidates were not all shown as exact ROI inputs')
                    for index, (item, (_, observation)) in enumerate(zip(rois, candidates)):
                        derived = review_crop({'capture_id': current, 'camera': {'image_sha256': current_hash},
                            'image_base64': base64.b64encode(current_raw).decode()}, observation['bbox'])
                        if item.get('bbox') != observation['bbox'] or any(
                                item.get(k) != v for k, v in derived.items() if k != 'image_base64'):
                            raise ValueError('Current ROI differs from saved candidate pixels')
                        filename = source+'.'+current+'.current-'+str(index)+'.confirmation-roi.jpg'
                        if item.get('archive_filename') != filename or hashlib.sha256(
                                (directory/filename).read_bytes()).hexdigest() != derived['crop_sha256']:
                            raise ValueError('Archived current ROI differs from actual model input')
            moves = [e for e in command.get('nbv_events', []) if e.get('planning_mode') == 'confirmation'
                and e.get('source_capture_id') == source]
            if not moves:
                raise ValueError('No actual confirmation-view planning record')
            reviews.append({'source_capture_id': source, 'current_capture_id': current,
                            'outcome': outcome, 'geometry_compatible': compatible, 'paired_calls': len(calls)})
        except (ValueError, KeyError, OSError) as exc:
            problems.append(str(exc))
    checks = {'multiview_mode_recorded': record.get('target_confirmation_mode') == 'multiview',
              'has_paired_review': bool(reviews), 'all_reviews_verified': not problems,
              'landed_disarmed': record.get('flight', {}).get('armed') is False
                                  and record.get('flight', {}).get('landed_state') == 1}
    found = any((e.get('result') or {}).get('search_status') == 'target_found'
                for e in command.get('search_events', []))
    checks['found_requires_confirmed_review'] = not found or any(r['outcome'] == 'confirmed' for r in reviews)
    if any(e.get('geometry_policy') in ('clipped_regions_v2', 'clipped_regions_v3') for e in command.get('target_confirmation_events', [])):
        outcome = command.get('search_outcome') or {}
        checks['structured_final_outcome_consistent'] = (
            type(outcome.get('target_confirmed')) is bool
            and outcome['target_confirmed'] == (found and any(r['outcome'] == 'confirmed' for r in reviews))
            and outcome.get('review_count') == len(reviews)
            and outcome.get('observation_count') == len(captures)
            and outcome.get('basis') == 'structured_search_and_confirmation_tools')
    if require_confirmed:
        checks['confirmed_target_required'] = any(r['outcome'] == 'confirmed' for r in reviews) and found
    if require_rejected:
        checks['rejected_candidate_required'] = any(r['outcome'] == 'rejected' for r in reviews)
    result = {'passed': all(checks.values()), 'checks': checks, 'reviews': reviews, 'problems': problems,
              'limitation': 'Checks evidence lineage and geometry, not independent visual category truth.'}
    output.mkdir(parents=True, exist_ok=False)
    (output/'confirmation-audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-confirmed-target', action='store_true')
    parser.add_argument('--require-rejected-candidate', action='store_true')
    args = parser.parse_args()
    result = audit(args.result, args.output, args.require_confirmed_target, args.require_rejected_candidate)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    sys.exit(0 if result['passed'] else 2)
