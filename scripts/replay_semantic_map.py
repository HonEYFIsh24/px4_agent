#!/usr/bin/env python3
"""Replay actual saved reports with archived geometry, or run an explicit controlled fixture."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from semantic_map import SemanticMap


def replay(result_file, output, audit_only=False):
    record=json.loads(result_file.read_text());command=record['command']
    directory=Path(record.get('capture_directory') or result_file.parent/'captures')
    owner=('offline-replay',command['id']);model_calls=command.get('model_calls',[])
    m=None;skipped=[]
    for report in command.get('perceptions',[]):
        if report.get('status')!='ok' or not report.get('capture_id'):continue
        identifier=report['capture_id']
        if Path(identifier).name!=identifier or identifier in ('.','..'):raise ValueError('Invalid archived capture basename')
        path=directory/(identifier+'.geometry.json')
        if not path.exists():
            skipped.append({'capture_id':identifier,'reason':'missing acquisition-time registered geometry; not guessed'})
            continue
        archive=json.loads((directory/(identifier+'.json')).read_text());geometry=json.loads(path.read_text())
        snapshot=archive['snapshot']
        for side,suffix in [('left','.jpg'),('right','.right.jpg')]:
            digest=hashlib.sha256((directory/(identifier+suffix)).read_bytes()).hexdigest()
            if digest!=snapshot['stereo_image_sha256'][side]:raise ValueError('Archived image hash mismatch')
        delivered=any(c.get('status')=='ok' and 'image' in c.get('input_modalities',[]) and identifier in c.get('capture_ids',[])
                      for c in model_calls)
        frame={'owner':owner,'delivered':delivered,'capture_id':identifier,'snapshot':snapshot}
        if m is None:m=SemanticMap(owner,geometry,query=archive['query'],instruction=command['prompt'])
        m.update_observation(frame,report,geometry)
    output.mkdir(parents=True,exist_ok=True)
    audit={'source':'actual_saved_images_reports_and_registered_geometry','skipped':skipped,
           'replayed_observations':len(m.observations) if m else 0}
    if m:
        snapshot=m.snapshot();snapshot['evidence_mode']='actual archived camera/report replay'
        ids={e['mission']['id'] for e in command.get('flight_missions',[]) if e.get('mission')}
        paths=[]
        log=ROOT/'logs/flight.jsonl'
        if log.exists():
            for line in log.read_text().splitlines():
                event=json.loads(line)
                if event.get('event')=='navigation_planned' and event.get('mission_id') in ids:
                    plan=event['plan'];paths.append({'points_enu_m':[plan['start_enu_m'],*plan['waypoints']]})
        snapshot['recorded_navigation_paths']=paths
        (output/'semantic-map.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2))
        audit['map_context']=m.get_context()
    (output/'replay-audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2))
    print(json.dumps(audit,ensure_ascii=False,indent=2))
    if not m and not audit_only:raise RuntimeError('No replayable registered observations; old ENU/world transform must not be guessed')
    return m


def demo(output):
    # Deliberately synthetic data to test geometry and multi-view math offline.
    # No Gazebo identities or target poses are used as visual answers.
    from static_navigation import StaticNavigator
    info={'width':640,'height':480,'K':[320.,0.,320.,0.,320.,240.,0.,0.,1.]}
    epoch={'vio_epoch':0,'ekf_counters':{'xy':1,'z':1,'heading':1}}
    boxes=[([2.9,-1.,0.],[3.6,1.,4.])]
    nav=StaticNavigator(boxes,[[-13.,-13.,.8],[13.,13.,10.4]])
    nav.world_sha256='a'*64
    nav.bind([0.,0.,0.],[0.,0.,0.,1.],{'position_m':[0.,0.,0.],'orientation_xyzw':[0.,0.,0.,1.]})
    owner=('controlled-fixture','demo');m=None
    for index,position in enumerate(([0.,0.,3.],[0.,3.,3.],[1.,3.,3.]),1):
        identifier='controlled-'+str(index)
        pose={'frame_id':'map','body_reference':'base_link','position_m':position,'orientation_xyzw':[0.,0.,0.,1.],
              'stamp_sim_s':float(index),'age_seconds':.1}
        hashes={s:hashlib.sha256((identifier+s).encode()).hexdigest() for s in ('left','right')}
        snap={'stamp_sim_s':float(index),'age_seconds':.1,'measurement_age_seconds':.01,'body_pose_enu':pose,
              'coordinate_epoch':epoch,'image_quality':{'laplacian_variance':100.,'mean_luma':120.,'saturated_fraction':.1},
              'camera_intrinsics':info,'stereo_image_sha256':hashes,'stereo_pair_id':identifier+'-pair'}
        geometry=nav.observation_visibility(pose,info,[0.,0.])
        geometry.update(capture_pair_id=snap['stereo_pair_id'],coordinate_epoch=epoch,stamp_sim_s=float(index),image_sha256=hashes,
                        body_pose_enu=pose,camera_intrinsics=info)
        if m is None:m=SemanticMap(owner,geometry,query='蓝色油桶',instruction='寻找红色集装箱附近的蓝色油桶')
        targets=[]
        if index>1:
            point=[5.+.1*(index-2),2.,1.3]
            targets=[{'label':'红色集装箱','bbox':[.02,.02,.98,.98], 'stereo_location':{
                'status':'ok','method':'stereo_sgbm_robust_surface','frame':'local_enu','position_enu_m':point,
                'covariance_enu_m2':[[.09,0.,0.],[0.,.09,0.],[0.,0.,.16]],'image_stamp_sim_s':float(index),
                'image_sha256':hashes,'coordinate_epoch':epoch}}]
        report={'status':'ok','capture_id':identifier,'targets':targets}
        m.update_observation({'owner':owner,'delivered':True,'capture_id':identifier,'snapshot':snap},report,geometry)
    result=m.snapshot();result.update(evidence_mode='CONTROLLED SYNTHETIC FIXTURE — not actual camera measurements',debug_raw_boxes=boxes)
    output.mkdir(parents=True,exist_ok=True)
    (output/'semantic-map.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(m.get_context(),ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--audit-only',action='store_true');p.add_argument('--demo',action='store_true')
    a=p.parse_args()
    if a.demo:demo(a.output)
    elif a.result:replay(a.result,a.output,a.audit_only)
    else:p.error('--result or --demo is required')
