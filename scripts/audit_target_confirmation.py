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
from target_confirmation import (CONFIRMATION_GEOMETRY_POLICY, association_allows_confirmation,
                                 observation_for, observations_for, pair_geometry, review_crop)


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
            if policy in ('clipped_regions_v2', 'clipped_regions_v3', 'bearing_hypothesis_v4', CONFIRMATION_GEOMETRY_POLICY) and len(matches) == 1:
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
            if policy == CONFIRMATION_GEOMETRY_POLICY:
                computed = matches[0][1] if compatible else {'compatible': False, 'association_basis': 'none'}
                recorded = event.get('geometry') or {}
                if (recorded.get('compatible') is not computed['compatible']
                        or recorded.get('association_basis') != computed['association_basis']
                        or (compatible and recorded.get('basis') != computed.get('basis'))):
                    raise ValueError('Recorded association basis differs from recomputed geometry')
                strong = compatible and association_allows_confirmation(computed)
            else:
                strong = compatible and not (policy in ('clipped_regions_v3', 'bearing_hypothesis_v4') and matches[0][1].get('association_hypothesis_only', False))
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
                if policy in ('clipped_regions_v2', 'clipped_regions_v3', 'bearing_hypothesis_v4', CONFIRMATION_GEOMETRY_POLICY):
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
    progress = [e['result'] for e in command.get('search_events', [])
                if isinstance(e.get('result'), dict) and e['result'].get('target_confirmation_mode') == 'multiview']
    last = progress[-1] if progress else {}
    visual_confirmed = last.get('target_visual_confirmed', found)
    confirmed_evidence = any(r['outcome'] == 'confirmed' for r in reviews)
    checks['visual_confirmation_has_review'] = not visual_confirmed or confirmed_evidence
    if any(e.get('geometry_policy') in ('clipped_regions_v2', 'clipped_regions_v3', 'bearing_hypothesis_v4', CONFIRMATION_GEOMETRY_POLICY) for e in command.get('target_confirmation_events', [])):
        outcome = command.get('search_outcome') or {}
        checks['structured_final_outcome_consistent'] = (
            type(outcome.get('target_confirmed')) is bool
            and outcome['target_confirmed'] == (visual_confirmed and confirmed_evidence)
            and outcome.get('review_count') == len(reviews)
            and outcome.get('observation_count') == len(captures)
            and outcome.get('basis') == 'structured_search_and_confirmation_tools')
    if (command.get('task_contract') or {}).get('target_observation_required'):
        close = last.get('target_observation') or {}
        completed = close.get('completed') is True
        checks['found_requires_close_observation'] = not found or completed
        checks['close_observation_evidence'] = True
        if completed:
            try:
                from target_observation import observation_quality, RANGE_POLICY, LEGACY_RANGE_POLICY
                identifier = close['capture_id']
                if (not isinstance(identifier, str) or Path(identifier).name != identifier
                        or identifier in ('.', '..') or identifier not in captures or not any(
                        e.get('outcome') == 'confirmed' and e.get('current_target_id') == close.get('target_id')
                        and (e.get('candidate') or {}).get('current_capture_id') == identifier
                        for e in command.get('target_confirmation_events', []))):
                    raise ValueError('Close observation has no confirmed current-frame association')
                snapshot = json.loads((directory/(identifier+'.semantic-map.json')).read_text())
                if snapshot['owner'][1] != command['id']:
                    raise ValueError('Close observation belongs to another task')
                semantic_map = SimpleNamespace(landmarks={l['id']: SimpleNamespace(**l) for l in snapshot['landmarks']})
                landmark, observation = observation_for(semantic_map, close['target_id'], identifier)
                policy = close.get('range_policy', LEGACY_RANGE_POLICY)
                if policy == LEGACY_RANGE_POLICY and 'relative_stereo' in observation:
                    raise ValueError('New relative-stereo evidence cannot downgrade to the legacy range policy')
                if policy == RANGE_POLICY:
                    archive = json.loads((directory/(identifier+'.json')).read_text())['snapshot']
                    if any(observation[k] != archive[v] for k, v in (
                            ('image_sha256', 'stereo_image_sha256'), ('coordinate_epoch', 'coordinate_epoch'),
                            ('stereo_pair_id', 'stereo_pair_id'), ('body_pose_enu', 'body_pose_enu'),
                            ('camera_info', 'camera_intrinsics'), ('stamp_sim_s', 'stamp_sim_s'))):
                        raise ValueError('Relative range observation differs from the original capture')
                    for side, suffix in (('left', '.jpg'), ('right', '.right.jpg')):
                        if hashlib.sha256((directory/(identifier+suffix)).read_bytes()).hexdigest() != observation['image_sha256'][side]:
                            raise ValueError('Relative range stereo JPEG hash differs from capture')
                    report = next(p for p in command['perceptions'] if p.get('status') == 'ok' and p['capture_id'] == identifier)
                    metrics = [t.get('stereo_location') for t in report['targets']
                               if t['label'] == landmark.label and t['bbox'] == observation['bbox']]
                    if len(metrics) != 1 or metrics[0] != observation['relative_stereo']['measurement']:
                        raise ValueError('Relative range measurement differs from the actual saved report')
                if close != observation_quality(landmark, observation, policy=policy) or not close['completed']:
                    raise ValueError('Close observation differs from saved depth/framing evidence')
            except Exception as exc:
                checks['close_observation_evidence'] = False
                problems.append({'stage': 'target_observation', 'reason': str(exc)})
    if require_confirmed:
        checks['confirmed_target_required'] = visual_confirmed and confirmed_evidence
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
