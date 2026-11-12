#!/usr/bin/env python3
"""Replay archived front measurements and explicit offline annotations; no live IO."""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stereo_localization import (calibration_from_camera_info, compute_disparities, decode_frame,
    estimate_from_disparities, validate_pair, StereoLocalizationError)
from task_runtime.object_frame import (estimate_front_face_frame, estimate_front_surface_frame,
    front_relation_quality, front_surface_regions)


def diagnose(result_file, output, annotations=None):
    record = json.loads(result_file.read_text())
    command = record['command'];directory = Path(record['capture_directory'])
    manual = json.loads(annotations.read_text()) if annotations else {'regions': {}}
    if annotations and (manual.get('diagnostic_only') is not True or manual.get('not_autonomous_detections') is not True):
        raise ValueError('Manual annotations must be explicitly diagnostic, never autonomous detections')
    attempts = {}
    for p in sorted(directory.glob('nbv-diagnostics-*.jsonl')):
        for line in p.read_text().splitlines():
            event = json.loads(line)
            if event.get('event') == 'task_object_frame_evidence':
                attempts[event['capture_id']] = event
    output.mkdir(parents=True, exist_ok=True)
    rows, hashes = [], {}
    reports = [r for r in command['perceptions'] if r.get('status') == 'ok']
    for index, report in enumerate(reports, 1):
        cid = report['capture_id']
        if Path(cid).name != cid or len(cid) != 32:
            raise ValueError('Invalid archived capture ID')
        path = directory/(cid+'.json');metadata = json.loads(path.read_text())
        pair = metadata['stereo_evidence'];gray = {}
        hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        for side in ('left', 'right'):
            filename = pair[side]['jpeg_filename']
            if Path(filename).name != filename:
                raise ValueError('Archived JPEG must be in the capture directory')
            image = directory/filename;pixels, digest = decode_frame({'jpeg_bytes': image.read_bytes()})
            if digest != pair['image_sha256'][side]:
                raise ValueError('Archived stereo JPEG hash mismatch')
            hashes[str(image)] = digest;gray[side] = pixels
        # Derive the recorded acquisition now from its recorded age. This is
        # historical replay; no stamp/age is replaced by the current wall time.
        stamp, limits = validate_pair(pair, pair['left']['stamp_sim_s']+pair['left']['measurement_age_seconds'])
        calibration = calibration_from_camera_info(pair['left']['camera_intrinsics'], pair['right']['camera_intrinsics'], .12)
        dl, dr = compute_disparities(gray['left'], gray['right'])
        semantic = json.loads((directory/(cid+'.semantic-map.json')).read_text())
        found = [(l, o) for l in semantic['landmarks'] for o in l['observations'] if o['capture_id'] == cid]
        attempt = attempts.get(cid)
        match = next(((l, o) for l, o in found if attempt and l['id'] == attempt['target_id']), None)
        if match is None:
            match = next(((l, o) for l, o in found if '货车' in l['label']), None)
        frame = {'capture_id': cid, 'delivered': True, 'snapshot': metadata['snapshot']}

        def measure(boxes):
            measured = []
            for box in boxes:
                try:
                    m = estimate_from_disparities(gray['left'], dl, dr, box, calibration, pair['body_pose_enu'], stamp, limits)
                    m.update(image_sha256=pair['image_sha256'], coordinate_epoch=pair['coordinate_epoch'],
                             capture_pair_id=pair['pair_id'])
                except StereoLocalizationError as exc:
                    m = {'status': 'unavailable', 'reason': str(exc)}
                measured.append(m)
            return measured

        row = {'index': index, 'capture_id': cid, 'stamp_sim_s': stamp,
               'body_position_enu_m': pair['body_pose_enu']['position_m'], 'hashes_verified': True}
        if attempt and match and attempt['profile'] == 'front_face_lateral':
            metrics = measure(attempt['boxes'])
            evidence = estimate_front_face_frame(frame, match[1], attempt['boxes'], metrics, attempt['descriptions'][0])
            row['original'] = {'boxes': attempt['boxes'], 'measurements': metrics,
                'quality': front_relation_quality(evidence, cid, attempt['boxes'][0]),
                'matches_recorded_reason': evidence['reason_code'] == attempt['reason_code']}
        elif attempt and match and attempt['profile'] == 'opaque_front_surface':
            face, regions = attempt['boxes'][0], attempt['boxes'][1:]
            metrics = measure(regions)
            evidence = estimate_front_surface_frame(frame, match[1], face, regions, metrics, attempt['descriptions'][0])
            row['autonomous_surface_replay'] = {'boxes': attempt['boxes'], 'measurements': metrics,
                'quality': front_relation_quality(evidence, cid, face), 'evidence': evidence,
                'matches_recorded_reason': evidence['reason_code'] == attempt['reason_code'],
                'manually_annotated': False}
        annotation = manual['regions'].get(cid)
        if annotation and match:
            observed = copy.deepcopy(match[1]);face = annotation['opaque_face_box']
            # A manual correction is a separately labelled counterfactual; the
            # original report/map remain byte-for-byte unchanged.
            if annotation.get('target_bbox_corrected_for_diagnosis'):
                observed['bbox'] = list(face)
            regions = front_surface_regions(face);metrics = measure(regions)
            evidence = estimate_front_surface_frame(frame, observed, face, regions, metrics, annotation['description'])
            row['manual_surface_diagnosis'] = {'autonomous': False, 'annotation': annotation,
                'quality': front_relation_quality(evidence, cid, face), 'evidence': evidence,
                'regions': regions, 'measurements': metrics}
        rows.append(row)
    result = {'source_result': str(result_file), 'source_result_sha256': hashlib.sha256(result_file.read_bytes()).hexdigest(),
        'offline_only': True, 'manual_regions_never_sent_to_runtime': True,
        'source_sha256': hashes, 'source_pairs': len(rows), 'rows': rows,
        'limits': limits, 'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'limitations': ['manual region diagnostics do not establish autonomous detection or flight success',
            'a passing selected region is not a statistical robustness result; acquisition/uncertainty gates are unchanged']}
    (output/'diagnosis.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    # Prove replay did not alter any input, including original failure records.
    for path, digest in hashes.items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise ValueError('A source archive changed during read-only replay')
    for row in rows:
        summary = {k: row[k] for k in ('index', 'capture_id', 'hashes_verified')}
        summary.update({key: row[key]['quality'] for key in
                        ('original', 'autonomous_surface_replay', 'manual_surface_diagnosis') if key in row})
        print(json.dumps(summary, ensure_ascii=False))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--annotations', type=Path)
    args = parser.parse_args()
    diagnose(args.result, args.output, args.annotations)
