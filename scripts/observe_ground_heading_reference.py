#!/usr/bin/env python3
"""Read-only ground heading experiment. No publishers, services or flight commands."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from client import BridgeClient
from vio.heading_reference_probe import GroundHeadingProbe


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-directory',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration-wall-s',type=float,default=12.)
    args=parser.parse_args()
    if not 5 <= args.duration_wall_s <= 30:
        parser.error('duration must be between 5 and 30 seconds')
    from run_reliability_suite import process_identity, owned_px4_process
    from record_ev_fusion import stable_px4_identity
    px4_binary=Path(os.getenv('PX4_DIR',str(Path.home()/'PX4-Autopilot')))/'build/px4_sitl_default/bin/px4-listener'
    manager=process_identity(args.run_directory)
    if not manager:
        raise RuntimeError('no live owned SITL manager')
    px4=stable_px4_identity(owned_px4_process(manager,px4_binary))
    import rospy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    rospy.init_node('ground_heading_reference_probe',anonymous=True,disable_signals=True,disable_rosout=True)
    lock=threading.Lock();visual=deque(maxlen=256);pose=deque(maxlen=256)
    def qvals(q):return [q.x,q.y,q.z,q.w]
    def on_visual(m):
        with lock:visual.append({'stamp':m.header.stamp.to_sec(),'frame':m.header.frame_id,
            'child':m.child_frame_id,'q':qvals(m.pose.pose.orientation),'variance':m.pose.covariance[35]})
    def on_pose(m):
        with lock:pose.append({'stamp':m.header.stamp.to_sec(),'frame':m.header.frame_id,'q':qvals(m.pose.orientation)})
    subscriptions=[rospy.Subscriber('/uav/vio/ev_shadow',Odometry,on_visual,queue_size=20),
                   rospy.Subscriber('/mavros/local_position/pose',PoseStamped,on_pose,queue_size=20)]
    args.output.mkdir(parents=True,exist_ok=False)
    result={'status':'running','scope':'read-only; no ground truth, publishers, services, or flight actions',
            'flight_authorized':False,'started_wall_s':time.time()}
    probe=GroundHeadingProbe();client=BridgeClient();last_stamp=None;state_history=deque(maxlen=30)
    unpaired_since=None;skipped_unpaired=0
    started=time.monotonic();deadline=started+args.duration_wall_s;startup=started+5
    try:
        with (args.output/'samples.jsonl').open('x',encoding='utf-8') as stream:
            while time.monotonic()<deadline:
                if process_identity(args.run_directory)!=manager or stable_px4_identity(owned_px4_process(manager,px4_binary))!=px4:
                    raise RuntimeError('owned SITL identity changed')
                state=client.state()
                try:
                    current_identity=probe.check_state(state)
                except ValueError as exc:
                    rejected={'event':'current_state_rejected','reason':str(exc),
                              'flight_state':{k:state.get(k) for k in
                                  ('simulation_only','ready','connected','armed','landed_state','mission')},
                              'external_vision':state.get('external_vision'),'scene':state.get('scene'),
                              'recorded_wall_s':time.time()}
                    stream.write(json.dumps(rejected,ensure_ascii=False,allow_nan=False)+'\n');stream.flush()
                    raise
                state_history.append((state,current_identity))
                with lock:
                    v=visual[-1].copy() if visual else None
                    p=next((item.copy() for item in reversed(pose) if v and item['stamp']<=v['stamp']),None)
                if not v or not p or v['stamp']==last_stamp:
                    if time.monotonic()>startup and not probe.rows:
                        raise RuntimeError('no fresh causally matched ROS poses')
                    time.sleep(.1);continue
                sample={'clock_sim_s':rospy.Time.now().to_sec(),'visual_stamp_sim_s':v['stamp'],
                        'pose_stamp_sim_s':p['stamp'],'visual_frame':v['frame'],'visual_child_frame':v['child'],
                        'pose_frame':p['frame'],'visual_q_xyzw':v['q'],'pose_q_xyzw':p['q'],
                        'visual_yaw_variance':v['variance']}
                matched=None
                for prior,identity in reversed(state_history):
                    ekf=(prior.get('external_vision') or {}).get('ekf') or {}
                    mag=ekf.get('mag_reference') or {}
                    local_time=ekf.get('local_position_source_stamp_sim_s')
                    mag_time=mag.get('source_stamp_sim_s')
                    observed=ekf.get('observed_sim_s')
                    if (identity==current_identity and all(type(x) in (int,float) for x in (local_time,mag_time,observed))
                            and 0 <= v['stamp']-local_time <= 1. and 0 <= v['stamp']-mag_time <= 1.6
                            and -.05 <= sample['clock_sim_s']-observed <= 1.):
                        matched=prior;break
                if matched is None:
                    skipped_unpaired+=1
                    if unpaired_since is None:unpaired_since=time.monotonic()
                    if time.monotonic()-unpaired_since>1.:
                        raise RuntimeError('no recent causally earlier EKF state')
                    time.sleep(.1);continue
                unpaired_since=None
                row={'sample':sample,'flight_state':{k:state.get(k) for k in
                        ('simulation_only','ready','connected','armed','landed_state','mission')},
                     'external_vision':matched.get('external_vision'),
                     'current_external_vision':state.get('external_vision'),'scene':state.get('scene'),
                     'recorded_wall_s':time.time()}
                try:
                    summary=probe.add(sample,matched)
                except ValueError as exc:
                    row['status']={'status':'rejected','reason':str(exc)}
                    stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');stream.flush()
                    raise
                last_stamp=v['stamp'];row['status']=summary
                stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');stream.flush()
                time.sleep(.1)
        result.update(status='completed',summary=probe.summary())
    except Exception as exc:
        result.update(status='failed',error=str(exc),summary=probe.summary())
    finally:
        for sub in subscriptions:sub.unregister()
        result['finished_wall_s']=time.time()
        result['skipped_unpaired_samples']=skipped_unpaired
        (args.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['status']=='completed' and result['summary'].get('status')=='candidate_only' else 2


if __name__=='__main__':
    sys.exit(main())
