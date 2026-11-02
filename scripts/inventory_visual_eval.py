#!/usr/bin/env python3
"""Inventory archived actual capture provenance. Outputs an unlabelled draft only."""
import argparse
import hashlib
import json
from pathlib import Path


def inventory(root):
    root=Path(root).resolve(); rows=[]; seen=set(); rejected={}
    def reject(reason): rejected[reason]=rejected.get(reason,0)+1
    for path in (root/'logs/setup').rglob('result.json'):
        try:
            record=json.loads(path.read_text()); seed=record.get('gazebo_seed_evidence'); world=Path(record.get('world_file') or '')
            directory=Path(record.get('capture_directory') or path.parent/'captures')
            if not (isinstance(seed,dict) and type(seed.get('seed')) is int and seed.get('source')=='actual_gzserver_proc_cmdline' and world.is_file()):
                reject('missing_seed_or_world_evidence'); continue
            world=world.resolve(); directory=directory.resolve()
            world.relative_to(root); directory.relative_to(root)
            world_hash=hashlib.sha256(world.read_bytes()).hexdigest()
            for report in record.get('command',{}).get('perceptions',[]):
                if report.get('status')!='ok': continue
                capture=report['capture_id']
                if Path(capture).name!=capture: reject('invalid_capture_id');continue
                image=directory/(capture+'.jpg'); geometry=directory/(capture+'.geometry.json')
                if not image.is_file() or not geometry.is_file(): reject('missing_image_or_geometry');continue
                g=json.loads(geometry.read_text()); digest=hashlib.sha256(image.read_bytes()).hexdigest()
                if digest!=g.get('image_sha256',{}).get('left') or world_hash!=g.get('world_sha256'):
                    reject('geometry_or_world_hash_mismatch');continue
                if digest in seen: reject('duplicate_image');continue
                seen.add(digest)
                rows.append({'id':capture,'capture_id':capture,'image_path':str(image.relative_to(root)),
                    'geometry_path':str(geometry.relative_to(root)),'source_result':str(path.relative_to(root)),
                    'world_path':str(world.relative_to(root)),'image_sha256':digest,'world_sha256':world_hash,
                    'scene_id':world.stem,'layout_group':None,'split':None,'run':record.get('run'),
                    'seed_evidence':seed,'sortie_id':g.get('sortie_id'),'stamp_sim_s':g.get('stamp_sim_s'),
                    'body_pose_enu':g.get('body_pose_enu'),'camera_origin_enu_m':g.get('camera_origin_enu_m'),
                    'camera_intrinsics':g.get('camera_intrinsics'),'label':None})
        except (OSError,ValueError,KeyError,TypeError) as exc:
            reject(type(exc).__name__)
    return {'status':'unlabelled_inventory_not_evaluation_set','schema_version':1,
            'description':'Automatic provenance inventory only. Never use model reports or world object names as labels.',
            'images':rows,'pairs':[],'tasks':[],'rejected':rejected}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    data=inventory(args.root);args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as f: json.dump(data,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps({'status':data['status'],'images':len(data['images']),'worlds':len({r['world_sha256'] for r in data['images']}),'rejected':data['rejected']},ensure_ascii=False))
