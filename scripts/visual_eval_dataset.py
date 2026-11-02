#!/usr/bin/env python3
"""Validate a manually reviewed evaluation manifest and score frozen predictions.

The tool never labels images or invokes a model. Counts describe the provided sample,
not population-wide performance. Paths are relative to an explicitly supplied root.
"""
import argparse
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

STATES = {'identifiable', 'insufficient_evidence', 'absent'}
ANSWERS = {'present', 'absent', 'uncertain'}
PAIR_LABELS = {'same', 'different', 'insufficient_evidence'}
PAIR_ANSWERS = {'same', 'different', 'uncertain'}
TASK_LABELS = {'correct', 'incorrect', 'insufficient_evidence'}
TASK_ANSWERS = {'confirmed', 'not_confirmed', 'error'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def secure_file(root, name):
    require(isinstance(name, str) and name and not Path(name).is_absolute(), 'paths must be nonempty and relative')
    path = (root / name).resolve()
    try:
        path.relative_to(root)
    except ValueError:
        raise ValueError('path escapes dataset root')
    require(path.is_file(), 'missing file: ' + name)
    return path


def box(value):
    require(isinstance(value, list) and len(value) == 4 and all(type(x) in (int, float) and math.isfinite(x) for x in value), 'invalid bbox')
    require(0 <= value[0] < value[2] <= 1 and 0 <= value[1] < value[3] <= 1, 'bbox must be normalized with positive area')


def review(record):
    a, b = record.get('annotator'), record.get('reviewer')
    require(isinstance(a, str) and a.strip() and isinstance(b, str) and b.strip() and a != b,
            'independent annotator and reviewer identifiers required')
    require(record.get('annotation_source') in ('human_visual_review', 'assistant_visual_review'), 'visual annotation source required')
    require(record.get('reviewer_kind') == 'human' and record.get('review_status') == 'approved', 'independent human review must be approved')


def validate(data, root, minimum=True):
    root = Path(root).resolve()
    require(data.get('schema_version') == 1, 'unsupported manifest version')
    images, pairs, tasks = data.get('images'), data.get('pairs'), data.get('tasks')
    require(all(isinstance(v, list) for v in (images, pairs, tasks)), 'images, pairs and tasks must be lists')
    if minimum:
        require(80 <= len(images) <= 120 and 20 <= len(pairs) <= 30, 'requires 80-120 images and 20-30 pairs')
    ids, hashes, capture_ids, scene_split, layout_split, world_split, worlds, by_id = set(), set(), set(), {}, {}, {}, set(), {}
    for x in images:
        identifier = x.get('id'); require(isinstance(identifier, str) and identifier and identifier not in ids, 'duplicate or invalid image ID')
        ids.add(identifier); by_id[identifier] = x
        digest = x.get('image_sha256'); capture = x.get('capture_id')
        require(isinstance(digest, str) and len(digest) == 64 and digest not in hashes, 'duplicate or invalid image hash')
        require(isinstance(capture, str) and capture and capture not in capture_ids, 'duplicate or invalid capture ID')
        hashes.add(digest); capture_ids.add(capture)
        require(hashlib.sha256(secure_file(root, x.get('image_path')).read_bytes()).hexdigest() == digest, 'image hash mismatch')
        g = json.loads(secure_file(root, x.get('geometry_path')).read_text())
        require(g.get('image_sha256', {}).get('left') == digest and g.get('world_sha256') == x.get('world_sha256'), 'geometry identity mismatch')
        require(g.get('body_pose_enu') == x.get('body_pose_enu') and g.get('stamp_sim_s') == x.get('stamp_sim_s'), 'capture pose/time mismatch')
        require(isinstance(g.get('camera_origin_enu_m'), list) and len(g['camera_origin_enu_m']) == 3
                and g['camera_origin_enu_m'] == x.get('camera_origin_enu_m'), 'camera origin missing or mismatch')
        require(x.get('camera_intrinsics') == g.get('camera_intrinsics') and bool(x.get('camera_intrinsics')), 'camera calibration mismatch')
        require(isinstance(x.get('sortie_id'), str) and x['sortie_id'] == g.get('sortie_id'), 'sortie identity mismatch')
        seed = x.get('seed_evidence')
        require(isinstance(seed, dict) and type(seed.get('seed')) is int and seed.get('source') == 'actual_gzserver_proc_cmdline', 'verified simulator seed required')
        source = json.loads(secure_file(root, x.get('source_result')).read_text())
        require(source.get('gazebo_seed_evidence') == seed and source.get('run') == x.get('run'), 'seed or run evidence mismatch')
        require(any(p.get('capture_id') == capture and p.get('status') == 'ok' for p in source.get('command', {}).get('perceptions', [])), 'capture not in successful reports')
        scene, layout, world, split = (x.get(k) for k in ('scene_id', 'layout_group', 'world_sha256', 'split'))
        require(all(isinstance(v, str) and v for v in (scene, layout, world)) and len(world) == 64 and split in ('development', 'holdout'), 'invalid scene or split')
        require(scene_split.setdefault(scene, split) == split and layout_split.setdefault(layout, split) == split
                and world_split.setdefault(world, split) == split, 'scene, layout or world leaks across splits')
        world_path=secure_file(root, x.get('world_path'))
        require(hashlib.sha256(world_path.read_bytes()).hexdigest() == world, 'world hash mismatch')
        worlds.add(world)
        label = x.get('label'); require(isinstance(label, dict), 'missing label'); review(label)
        require(label.get('state') in STATES and isinstance(label.get('category'), str) and label['category'].strip(), 'invalid visual label')
        box(label.get('observation_region'))
        if label.get('target_bbox') is not None: box(label['target_bbox'])
        if label['state'] == 'identifiable':
            require(label.get('target_bbox') is not None and isinstance(label.get('instance_id'), str) and label['instance_id'], 'identifiable label needs box and instance')
        if label['state'] == 'absent':
            require(label.get('target_bbox') is None and label.get('instance_id') is None, 'absent label cannot contain target')
        require(label.get('occlusion') in ('none', 'partial', 'heavy', 'unknown'), 'occlusion must be explicit')
        require(label.get('distance_band') in ('near', 'medium', 'far', 'unknown'), 'distance band must be explicit')
    if minimum:
        require(len(scene_split) >= 3 and len(worlds) >= 3 and len(layout_split) >= 3, 'requires at least three layouts and worlds')
        require(set(scene_split.values()) == {'development', 'holdout'}, 'development and holdout required')
        require({x['label']['state'] for x in images} == STATES, 'all three visual states required')
        require(any(x['label']['state'] == 'identifiable' and x['label']['category'] == 'barrel' for x in images), 'identifiable barrel positive required')
    pair_ids, seen_pairs = set(), set()
    for x in pairs:
        identifier=x.get('id'); require(isinstance(identifier, str) and identifier and identifier not in pair_ids, 'duplicate or invalid pair ID'); pair_ids.add(identifier)
        a,b=x.get('image_a'),x.get('image_b'); require(a in by_id and b in by_id and a != b, 'pair must reference distinct images')
        pair_key=frozenset((a,b)); require(pair_key not in seen_pairs, 'duplicate image pair'); seen_pairs.add(pair_key)
        require(by_id[a]['split'] == by_id[b]['split'] and by_id[a]['scene_id'] == by_id[b]['scene_id'], 'pair crosses scene or split')
        poses=[by_id[i]['body_pose_enu'] for i in (a,b)]
        positions=[p.get('position_m') for p in poses]
        require(all(isinstance(p,list) and len(p)==3 and all(type(v) in (int,float) and math.isfinite(v) for v in p) for p in positions), 'invalid pair poses')
        distance=math.sqrt(sum((u-v)**2 for u,v in zip(*positions)))
        rotations=[p.get('orientation_xyzw') for p in poses]
        angle=0.
        if all(isinstance(q,list) and len(q)==4 and all(type(v) in (int,float) and math.isfinite(v) for v in q) for q in rotations):
            norms=[math.sqrt(sum(v*v for v in q)) for q in rotations]
            if all(n > 1e-6 for n in norms):
                dot=abs(sum(u*v for u,v in zip(*rotations))/(norms[0]*norms[1]))
                angle=2*math.acos(min(1.,max(0.,dot)))
        require(distance >= .5 or angle >= math.radians(10), 'pair lacks a distinct measured viewpoint')
        require(x.get('identity') in PAIR_LABELS, 'invalid pair identity'); review(x)
        if x['identity'] in ('same', 'different'):
            left, right = by_id[a]['label'], by_id[b]['label']
            require(left['state'] == right['state'] == 'identifiable' and left['category'] == right['category'], 'pair needs identifiable same-category targets')
            require((left['instance_id'] == right['instance_id']) == (x['identity'] == 'same'), 'pair identity contradicts instance labels')
    if minimum:
        require({'same', 'different'} <= {x['identity'] for x in pairs}, 'both positive and negative identity pairs required')
    task_ids, source_task_ids=set(), set()
    for x in tasks:
        identifier=x.get('id'); require(isinstance(identifier,str) and identifier and identifier not in task_ids, 'duplicate or invalid task ID'); task_ids.add(identifier)
        require(x.get('identity_label') in TASK_LABELS and x.get('split') in ('development','holdout'), 'invalid task label or split'); review(x)
        linked=x.get('image_ids'); require(isinstance(linked,list) and linked and all(i in by_id and by_id[i]['split']==x['split'] for i in linked), 'task image provenance or split invalid')
        require(isinstance(x.get('task_id'),str) and x['task_id'], 'task ID required')
        require(x['task_id'] not in source_task_ids, 'duplicate archived task'); source_task_ids.add(x['task_id'])
        task_record=json.loads(secure_file(root, x.get('source_result')).read_text()).get('command', {})
        require(task_record.get('id') == x['task_id'], 'task ID differs from archived command')
        actual_ids={p.get('capture_id') for p in task_record.get('perceptions', []) if p.get('status') == 'ok'}
        require(all(by_id[i]['capture_id'] in actual_ids for i in linked), 'task images are not from archived task')
        found=(task_record.get('search_outcome') or {}).get('target_confirmed')
        if task_record.get('status') == 'failed': actual='error'
        else:
            require(type(found) is bool, 'task lacks structured confirmation outcome')
            actual='confirmed' if found else 'not_confirmed'
        require(x.get('actual_outcome') == actual, 'task outcome differs from archived structured result')
    if minimum:
        require(bool(tasks), 'archived, independently reviewed tasks required for task-level metric')
    return {'images':len(images),'pairs':len(pairs),'tasks':len(tasks),'scenes':len(scene_split),'layouts':len(layout_split),'worlds':len(worlds), 'states':dict(Counter(x['label']['state'] for x in images)), 'ready_for_task_metric':bool(tasks)}


def ratio(n,d): return {'numerator':n, 'denominator':d, 'rate':n/d if d else None}


def score(data, predictions):
    output={}
    for kind, allowed in (('images', ANSWERS), ('pairs', PAIR_ANSWERS), ('tasks', TASK_ANSWERS)):
        rows=data[kind]; given=predictions.get(kind)
        require(isinstance(given,list), 'missing predictions: '+kind)
        mapping={}
        for item in given:
            require(item.get('id') not in mapping and item.get('answer') in allowed, 'duplicate or invalid prediction')
            mapping[item.get('id')]=item['answer']
        require(set(mapping)=={x['id'] for x in rows}, 'prediction IDs do not match manifest')
        if kind == 'tasks':
            require(all(mapping[x['id']] == x['actual_outcome'] for x in rows), 'task predictions differ from archived outcomes')
        output[kind]=mapping
    result={}
    for split in ('development','holdout'):
        img=[x for x in data['images'] if x['split']==split]; answers=output['images']
        absent=[x for x in img if x['label']['state']=='absent']; pos=[x for x in img if x['label']['state']=='identifiable']
        tasks=[x for x in data['tasks'] if x['split']==split]; ta=output['tasks']
        confirmed=[x for x in tasks if ta[x['id']]=='confirmed']; adjudicated=[x for x in confirmed if x['identity_label']!='insufficient_evidence']
        pairs=[x for x in data['pairs'] if next(i for i in data['images'] if i['id']==x['image_a'])['split']==split]; pa=output['pairs']
        negative=[x for x in pairs if x['identity']=='different']; positive=[x for x in pairs if x['identity']=='same']
        result[split]={
            'single_frame_false_positive':ratio(sum(answers[x['id']]=='present' for x in absent),len(absent)),
            'identifiable_false_negative_explicit':ratio(sum(answers[x['id']]=='absent' for x in pos),len(pos)),
            'single_frame_undecided':ratio(sum(answers[x['id']]=='uncertain' for x in img),len(img)),
            'insufficient_evidence_count':sum(x['label']['state']=='insufficient_evidence' for x in img),
            'insufficient_evidence_predicted_present':ratio(sum(answers[x['id']]=='present' for x in img if x['label']['state']=='insufficient_evidence'), sum(x['label']['state']=='insufficient_evidence' for x in img)),
            'pair_false_same':ratio(sum(pa[x['id']]=='same' for x in negative),len(negative)),
            'pair_false_different':ratio(sum(pa[x['id']]=='different' for x in positive),len(positive)),
            'pair_undecided':ratio(sum(pa[x['id']]=='uncertain' for x in pairs),len(pairs)),
            'task_incorrect_confirmation_among_adjudicated_confirmed':ratio(sum(x['identity_label']=='incorrect' for x in adjudicated),len(adjudicated)),
            'task_incorrect_confirmation_among_all_tasks':ratio(sum(x['identity_label']=='incorrect' for x in confirmed),len(tasks)),
            'task_confirmed_unadjudicated':sum(x['identity_label']=='insufficient_evidence' for x in confirmed),
            'task_error_count':sum(ta[x['id']]=='error' for x in tasks),
        }
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True);p.add_argument('--root',type=Path,required=True);p.add_argument('--predictions',type=Path);p.add_argument('--allow-small',action='store_true',help='validation fixture only; never report as full dataset')
    args=p.parse_args();raw=args.manifest.read_bytes();data=json.loads(raw);summary=validate(data,args.root,minimum=not args.allow_small)
    report={'manifest_sha256':hashlib.sha256(raw).hexdigest(),'manifest_counts':summary,'full_size_gate_applied':not args.allow_small}
    if args.predictions:
        predictions=json.loads(args.predictions.read_text());require(predictions.get('manifest_sha256')==report['manifest_sha256'], 'prediction manifest hash mismatch');require(isinstance(predictions.get('run_id'),str) and predictions['run_id'], 'prediction run ID required');report['metrics']=score(data,predictions)
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__': main()
