#!/usr/bin/env python3
"""Run a frozen image-disjoint blind crop evaluation; no flight or map writes."""
import argparse
import base64
import hashlib
import json
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from review_blind_crop import prompt_for, run, validate_review


def validate_manifest(manifest):
    prompt, _ = prompt_for(manifest['prompt_variant'])
    if hashlib.sha256(prompt.encode()).hexdigest() != manifest['prompt_sha256']:
        raise ValueError('Frozen prompt changed after evaluation manifest creation')
    cases = manifest['cases']; seen_ids, seen_hashes = set(), set()
    if not isinstance(cases, list) or not cases:
        raise ValueError('Evaluation needs predeclared cases')
    for case in cases:
        identifier, digest = case['id'], case['source_image_sha256']
        capture_id = case['capture_id']
        if (not isinstance(identifier, str) or not identifier or Path(identifier).name != identifier
                or identifier in seen_ids or digest in seen_hashes or digest in manifest['excluded_development_image_sha256']
                or not isinstance(capture_id, str) or not capture_id or Path(capture_id).name != capture_id
                or capture_id in ('.', '..')
                or type(case['visually_present']) is not bool
                or case['target_category'] not in ('vehicle', 'barrel', 'traffic_cone')
                or ('manual_bbox' in case) == ('target_index' in case)):
            raise ValueError('Invalid, duplicate or development-overlapping case')
        if 'target_index' in case and (type(case['target_index']) is not int or case['target_index'] < 0):
            raise ValueError('Invalid archived target index')
        if 'manual_bbox' in case:
            from semantic_map import bbox
            bbox(case['manual_bbox'])
        source = (ROOT/case['source_result']).resolve()
        try:
            source.relative_to(ROOT.resolve())
        except ValueError:
            raise ValueError('Evaluation source must be inside the project')
        record = json.loads(source.read_text())
        capture_path = (Path(record['capture_directory'])/(capture_id+'.jpg')).resolve()
        try:
            capture_path.relative_to(ROOT.resolve())
        except ValueError:
            raise ValueError('Evaluation image must be inside the project')
        raw = capture_path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != digest:
            raise ValueError('Frozen image no longer matches its source hash')
        seen_ids.add(identifier); seen_hashes.add(digest)
    return cases


def summarize(cases, reviews, manifest):
    rows = []
    for case in cases:
        review = reviews[case['id']]
        if (review.get('status') != 'ok' or review.get('source_capture_id') != case['capture_id']
                or review.get('source_image_sha256') != case['source_image_sha256']
                or review.get('prompt_variant') != manifest['prompt_variant']
                or review.get('prompt_sha256') != manifest['prompt_sha256']):
            raise ValueError('Review provenance or status does not match the frozen case')
        result = validate_review(review['result'], manifest['prompt_variant'])
        categories = result['categories']
        support = case['target_category'] in categories
        rows.append({'id': case['id'], 'target_category': case['target_category'],
                     'visually_present': case['visually_present'], 'model_categories': categories,
                     'target_supported': support, 'returned_empty': not categories,
                     'duration_seconds': review['duration_seconds'], 'source_image_sha256': case['source_image_sha256']})
    positive = [x for x in rows if x['visually_present']]
    negative = [x for x in rows if not x['visually_present']]
    return {'cases': rows, 'counts': {'positive': len(positive), 'negative': len(negative),
            'positive_supported': sum(x['target_supported'] for x in positive),
            'positive_not_supported': sum(not x['target_supported'] for x in positive),
            'negative_falsely_supported': sum(x['target_supported'] for x in negative),
            'negative_not_supported': sum(not x['target_supported'] for x in negative),
            'empty_outputs': sum(x['returned_empty'] for x in rows)},
            'model_duration_seconds_median': statistics.median(x['duration_seconds'] for x in rows),
            'model_duration_seconds_sum': sum(x['duration_seconds'] for x in rows),
            'limitations': manifest['selection_limitations'],
            'ground_truth_scope': manifest['annotation_source'],
            'semantics': 'not-supported is not an explicit absence decision; labels need independent human review',
            'actual_flight': False, 'production_gate': False}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true'); p.add_argument('--resume', action='store_true')
    p.add_argument('--manifest', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); manifest = json.loads(args.manifest.read_text()); cases = validate_manifest(manifest)
    if not args.execute:
        print('Dry run: {} frozen cases, no model requests or flight.'.format(len(cases))); return 0
    if args.output.exists() and not args.resume:
        raise ValueError('Refusing to overwrite evaluation output; use --resume for this same manifest')
    args.output.mkdir(parents=True, exist_ok=True)
    frozen_hash = hashlib.sha256(args.manifest.read_bytes()).hexdigest(); lock = args.output/'manifest-sha256.txt'
    if lock.exists() and lock.read_text().strip() != frozen_hash:
        raise ValueError('Cannot resume with a different frozen manifest')
    if not lock.exists(): lock.write_text(frozen_hash+'\n')
    reviews = {}
    for case in cases:
        dest = args.output/case['id']; existing = dest/'blind-review.json'
        if existing.exists():
            review = json.loads(existing.read_text())
        else:
            if dest.exists(): raise ValueError('Incomplete case directory; preserve and inspect before resuming')
            review = run(ROOT/case['source_result'], case['capture_id'], case.get('target_index', 0), dest,
                         manifest['prompt_variant'], case.get('manual_bbox'))
        original = json.loads((ROOT/case['source_result']).read_text())
        report = next(x for x in original['command']['perceptions'] if x['capture_id'] == case['capture_id'])
        expected_box = case['manual_bbox'] if 'manual_bbox' in case else report['targets'][case['target_index']]['bbox']
        expected_source = 'manual_offline_evaluation_only' if 'manual_bbox' in case else 'archived_report'
        from target_confirmation import review_crop
        source_bytes = (dest/'source.jpg').read_bytes()
        derived = review_crop({'capture_id': case['capture_id'], 'camera': {'image_sha256': case['source_image_sha256']},
                               'image_base64': base64.b64encode(source_bytes).decode()}, expected_box, padding=0.)
        if (review.get('original_bbox') != expected_box or review.get('bbox_source') != expected_source
                or hashlib.sha256(source_bytes).hexdigest() != case['source_image_sha256']
                or review.get('crop_sha256') != derived['crop_sha256']
                or review.get('crop_pixel_bounds') != derived['crop_pixel_bounds']
                or hashlib.sha256((dest/'roi.jpg').read_bytes()).hexdigest() != derived['crop_sha256']):
            raise ValueError('Archived input images or selected box differ from the frozen case')
        reviews[case['id']] = review
        if review.get('status') != 'ok':
            raise RuntimeError('Model request failed for '+case['id']+'; preserve the evidence')
        print(json.dumps({'case': case['id'], 'categories': review['result']['categories'],
                          'duration_seconds': review['duration_seconds']}, ensure_ascii=False), flush=True)
    report = summarize(cases, reviews, manifest)
    report['manifest_sha256'] = frozen_hash
    (args.output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report['counts'], ensure_ascii=False)); return 0


if __name__ == '__main__': sys.exit(main())
