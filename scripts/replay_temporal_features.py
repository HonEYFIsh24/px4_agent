#!/usr/bin/env python3
"""Offline temporal feature experiment from archived image pairs; does not fly or change maps."""
import argparse
import copy
import hashlib
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from stereo_localization import iris_left_optical_to_body, matvec, quaternion_rotation
from target_confirmation import observation_for
from semantic_map import bbox


def ray(observation, pixel):
    k = observation['camera_info']['K']
    optical = [(pixel[0]-k[2])/k[0], (pixel[1]-k[5])/k[4], 1.]
    direction = matvec(quaternion_rotation(observation['body_pose_enu']['orientation_xyzw']),
                       matvec(iris_left_optical_to_body()['rotation'], optical))
    length = math.sqrt(sum(x*x for x in direction))
    return [x/length for x in direction]


def geometry(a, b, pa, pb):
    u, v = ray(a, pa), ray(b, pb)
    origins = [a['bearing_origin_enu_m'], b['bearing_origin_enu_m']]
    w = [x-y for x, y in zip(*origins)]
    dot = lambda x, y: sum(i*j for i, j in zip(x, y))
    cosine = max(-1., min(1., dot(u, v)))
    denom = 1.-cosine*cosine
    if denom < math.sin(math.radians(3.))**2:
        return None
    du, dv = dot(u, w), dot(v, w)
    t, s = (cosine*dv-du)/denom, (dv-cosine*du)/denom
    if not (.5 <= t <= 20. and .5 <= s <= 20.):
        return None
    p = [o+d*t for o, d in zip(origins[0], u)]
    q = [o+d*s for o, d in zip(origins[1], v)]
    angular_gap = math.degrees(math.atan2(math.dist(p, q), min(t, s)))
    if angular_gap > .5:
        return None
    return {'parallax_deg': round(math.degrees(math.acos(cosine)), 4), 'ray_gap_deg': round(angular_gap, 4)}


def spatially_distinct(matches, separation_px=8.):
    """Keep lower-residual matches only when separated in both actual images."""
    accepted = []
    for item in sorted((x for x in matches if x['geometry'] is not None), key=lambda x: x['geometry']['ray_gap_deg']):
        if all(math.dist(item[side], other[side]) >= separation_px
               for other in accepted for side in ('source_pixel', 'current_pixel')):
            accepted.append(item)
    return accepted


def replay(result_path, event_index, output, controlled_boxes=None):
    record = json.loads(result_path.read_text()); command = record['command']
    events = [e for e in command['target_confirmation_events'] if e['outcome'] in ('confirmed', 'unconfirmed', 'rejected')]
    event = events[event_index]; candidate = event['candidate']
    directory = Path(record.get('capture_directory') or result_path.parent/'captures')
    ids = [candidate['source_capture_id'], candidate['current_capture_id']]
    if any(Path(i).name != i or i in ('.', '..') for i in ids):
        raise ValueError('Invalid capture basename')
    snapshot = json.loads((directory/(ids[1]+'.semantic-map.json')).read_text())
    if event['owner'] != snapshot['owner'] or event['owner'][1] != command['id']:
        raise ValueError('Pair ownership does not match task and map')
    m = SimpleNamespace(landmarks={l['id']: SimpleNamespace(**l) for l in snapshot['landmarks']})
    observations = [observation_for(m, key, identifier)[1] for key, identifier in
                    ((candidate['target_id'], ids[0]), (event['current_target_id'], ids[1]))]
    observations = copy.deepcopy(observations)
    if controlled_boxes is not None:
        if len(controlled_boxes) != 2:
            raise ValueError('Controlled comparison requires exactly two boxes')
        for obs, box in zip(observations, controlled_boxes):
            obs['bbox'] = bbox(box)
    if (observations[1]['stamp_sim_s'] <= observations[0]['stamp_sim_s'] or
            math.dist(observations[0]['body_pose_enu']['position_m'], observations[1]['body_pose_enu']['position_m']) < .5):
        raise ValueError('Pair lacks newer view or observer baseline')
    detector = cv2.ORB_create(nfeatures=2500, edgeThreshold=8, fastThreshold=8)
    features, hashes, identities = [], [], []
    for identifier, observation in zip(ids, observations):
        raw = (directory/(identifier+'.jpg')).read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        archive = json.loads((directory/(identifier+'.json')).read_text())
        acquisition = json.loads((directory/(identifier+'.geometry.json')).read_text())
        identity = {k: acquisition[k] for k in ('world_sha256', 'sortie_id', 'coordinate_epoch', 'registration')}
        if (archive['snapshot']['coordinate_epoch'] != identity['coordinate_epoch'] or
                acquisition['body_pose_enu'] != observation['body_pose_enu'] or
                acquisition['camera_intrinsics'] != observation['camera_info']):
            raise ValueError('Archived pose, camera or epoch differs from saved observation')
        identities.append(identity)
        if digest != archive['snapshot']['stereo_image_sha256']['left']:
            raise ValueError('Archived image hash differs from capture')
        if not any(c.get('status') == 'ok' and identifier in c.get('capture_ids', [])
                   and 'image' in c.get('input_modalities', []) for c in command['model_calls']):
            raise ValueError('Image was not delivered to the model')
        frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
        if frame is None or frame.shape != (480, 640):
            raise ValueError('Unsupported camera image')
        box = observation['bbox']; mask = np.zeros(frame.shape, np.uint8)
        mask[int(box[1]*480):int(box[3]*480), int(box[0]*640):int(box[2]*640)] = 255
        features.append(detector.detectAndCompute(frame, mask)); hashes.append(digest)
    if identities[0] != identities[1]:
        raise ValueError('Images belong to different world, sortie, epoch or registration')
    (ka, da), (kb, db) = features
    matches = []
    if da is not None and db is not None:
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        def ratios(pairs):
            return {x.queryIdx: x.trainIdx for pair in pairs if len(pair) == 2 for x, y in [pair] if x.distance < .75*y.distance}
        forward, reverse = ratios(matcher.knnMatch(da, db, k=2)), ratios(matcher.knnMatch(db, da, k=2))
        for i, j in forward.items():
            if reverse.get(j) != i:
                continue
            pa, pb = ka[i].pt, kb[j].pt
            check = geometry(*observations, pa, pb)
            matches.append({'source_pixel': list(pa), 'current_pixel': list(pb), 'geometry': check})
    inliers = [x for x in matches if x['geometry'] is not None]
    distinct = spatially_distinct(matches)
    spans = []
    for side, obs in zip(('source_pixel', 'current_pixel'), observations):
        box = obs['bbox']; width, height = (box[2]-box[0])*640, (box[3]-box[1])*480
        spans.append([round((max(p[side][axis] for p in distinct)-min(p[side][axis] for p in distinct))/scale, 4)
                      for axis, scale in enumerate((width, height))] if distinct else [0., 0.])
    result = {'source_result': str(result_path.resolve()), 'event_index': event_index, 'capture_ids': ids,
              'image_sha256': hashes, 'feature_count': [len(ka), len(kb)], 'mutual_matches': len(matches),
              'geometric_inliers': len(inliers), 'spatially_distinct_inliers': len(distinct),
              'distinct_bbox_span_fraction_xy': spans, 'matches': matches,
              'box_source': 'controlled_analyst_regions_not_model_detections' if controlled_boxes else 'saved_model_detections',
              'evaluated_boxes': [o['bbox'] for o in observations],
              'limitation': 'Box features may belong to background; geometric inliers do not establish class or identity.',
              'experimental_only': True, 'map_or_flight_updated': False}
    output.mkdir(parents=True, exist_ok=False)
    (output/'temporal-features.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--event-index', type=int, default=0)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-box', type=float, nargs=4)
    parser.add_argument('--current-box', type=float, nargs=4)
    args = parser.parse_args()
    if (args.source_box is None) != (args.current_box is None):
        parser.error('Both controlled boxes must be supplied together')
    boxes = [args.source_box, args.current_box] if args.source_box is not None else None
    print(json.dumps(replay(args.result, args.event_index, args.output, boxes), ensure_ascii=False, indent=2))
