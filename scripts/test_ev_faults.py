#!/usr/bin/env python3
"""Actual EV-chain fault tests. Default is a plan, with no ROS/flight requests.

--execute is an explicit short local-SITL fault sortie. Drop/delay operate the
EV adapter's test transport, not the older bridge VIO health test. Reset sends
the real upstream VINS restart message. Original worker failure/cancellation
and the separately completed safety landing are reported independently.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import sys
import time
from urllib.parse import urlsplit
import uuid
import xmlrpc.client

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0,str(ROOT))
from client import BridgeClient

PARAMETER='/uav_ev_bridge/test_fault'
DESCRIPTION='Actual external-vision fault safety test'
STEPS=[{'action':'takeoff','altitude':3.5},{'action':'hold','duration':20.}]


def fault_request(kind,duration,identifier,issued):
    if kind not in ('drop','delay'):
        raise ValueError('Only EV transport drop/delay use the test parameter')
    if not math.isfinite(duration) or not .5<=duration<=4 or not math.isfinite(issued) or issued<0:
        raise ValueError('Fault duration must be 0.5..4 wall seconds with a finite issued time')
    if not isinstance(identifier,str) or not identifier or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in identifier):
        raise ValueError('Fault identifier must be a safe nonempty string')
    return {'id':identifier,'kind':kind,'duration_s':duration,'issued_wall_s':issued,
            'expires_wall_s':issued+duration+.5,'synthetic':True,'simulation_only':True}


class BoundedXmlrpc(xmlrpc.client.Transport):
    def make_connection(self,host):
        connection=super().make_connection(host);connection.timeout=.5
        return connection


class Parameters:
    def __init__(self):
        uri=os.environ.get('ROS_MASTER_URI','http://127.0.0.1:11311')
        parsed=urlsplit(uri)
        if parsed.scheme!='http' or parsed.hostname not in ('localhost','127.0.0.1','::1'):
            raise RuntimeError('Fault testing requires the local ROS master')
        self.master=xmlrpc.client.ServerProxy(uri,transport=BoundedXmlrpc())

    def get(self,name):
        code,message,value=self.master.getParam('/ev_fault_evaluator',name)
        if code==-1:
            return None
        if code!=1:
            raise RuntimeError('Cannot read parameter: '+str(message))
        return value

    def install(self,value):
        if self.get(PARAMETER) not in (None,{}):
            raise RuntimeError('Another EV fault request is present; do not overwrite it')
        code,message,_=self.master.setParam('/ev_fault_evaluator',PARAMETER,value)
        if code!=1:
            raise RuntimeError('EV fault parameter refused: '+str(message))
        current=self.get(PARAMETER)
        if current!=value:
            raise RuntimeError('EV fault parameter readback differs from the request')
        return current

    def clear_owned(self,value):
        current=self.get(PARAMETER)
        if not value or current is None or current=={}:
            return {'cleared':False,'reason':'already absent or no owned parameter injected'}
        if current!=value:
            return {'cleared':False,'reason':'parameter has been replaced; do not clear another request'}
        code,message,_=self.master.deleteParam('/ev_fault_evaluator',PARAMETER)
        if code!=1:
            raise RuntimeError('Cannot clear owned EV fault request: '+str(message))
        return {'cleared':True,'confirmed_absent':self.get(PARAMETER) is None}


def require_ev_health(state,armed=False):
    ev=state.get('external_vision') or {}
    adapter,ekf=ev.get('adapter') or {},ev.get('ekf') or {}
    if (state.get('simulation_only') is not True or not state.get('ready')
            or ev.get('required') is not True or ev.get('healthy') is not True
            or ev.get('used_for_flight_control') is not True
            or adapter.get('mode')!='fuse' or adapter.get('ready') is not True
            or adapter.get('forwarding') is not True or adapter.get('sitl_verified') is not True
            or ekf.get('source')!='actual_owned_px4_uorb' or ekf.get('ready') is not True
            or ekf.get('fusion_active') is not True or ekf.get('local_position_valid') is not True):
        raise RuntimeError('Real EV transport/selected EKF fusion is not healthy: '+str(ev.get('reason')))
    if armed and (state.get('armed') is not True or state.get('mode')!='OFFBOARD'
            or abs(state['position_enu_m'][2]-3.5)>.12 or not state.get('navigation',{}).get('fresh')):
        raise RuntimeError('Fault trigger requires actual armed OFFBOARD fixed-altitude cruise with fresh navigation')
    return ev


def require_ground_preparation_state(state):
    if (not isinstance(state,dict) or state.get('simulation_only') is not True
            or state.get('armed') is not False or state.get('landed_state')!=1
            or (state.get('mission') or {}).get('status')=='running'):
        raise RuntimeError('Ground OFFBOARD preparation requires disarmed local simulation and no active worker')


def prepare_ground_preflight(client,operations,manager,report):
    """Prepare only a stationary stream/mode; never arm or submit a mission."""
    state,web=operations.states()
    require_ground_preparation_state(state)
    if web.get('active_command_id'):
        raise RuntimeError('Operator command is active; do not prepare OFFBOARD')
    require_ev_health(state)
    prepared=client.request('/prepare-offboard',{},timeout=15)
    # Save the original response before checking it, including any malformed
    # acknowledgment. A failed prepare/prearm must remain a preflight failure.
    report['ground_offboard_preparation']=prepared
    if not isinstance(prepared,dict) or prepared.get('status')!='ok' or prepared.get('armed') is not False:
        raise RuntimeError('Ground OFFBOARD preparation did not confirm armed=False')
    prepared_state=prepared.get('state')
    require_ground_preparation_state(prepared_state)
    if prepared_state.get('mode')!='OFFBOARD':
        raise RuntimeError('Ground OFFBOARD preparation did not confirm actual OFFBOARD mode')
    require_ev_health(prepared_state)
    preflight=operations.preflight_check(manager,2.)
    report['preflight']=preflight
    if not preflight.get('passed'):
        raise RuntimeError('PX4 preflight did not explicitly report OK')
    return preflight


def pinned_process_valid(identity):
    directory=Path('/proc')/str(identity['pid'])
    fields=(directory/'stat').read_text().rsplit(')',1)[1].split()
    if fields[0]=='Z' or fields[19]!=identity['start_time'] or directory.stat().st_uid!=os.getuid() or str((directory/'exe').resolve())!=identity['binary']:
        raise RuntimeError('Pinned PX4 process identity changed')


def verify_vins_restart_target(parameters,manager):
    """Verify the sole live project VINS node and its real restart subscription."""
    code,message,state=parameters.master.getSystemState('/ev_fault_evaluator')
    if code!=1:
        raise RuntimeError('Cannot read ROS restart subscribers: '+str(message))
    subscribers=dict(state[1]).get('/vins_restart',[])
    vins=[node for node in subscribers if node.rsplit('/',1)[-1]=='vins_estimator']
    if len(vins)!=1 or '/uav_ev_bridge' not in subscribers:
        raise RuntimeError('Require exactly one VINS estimator and the active EV adapter restart subscriber')
    code,message,uri=parameters.master.lookupNode('/ev_fault_evaluator',vins[0])
    if code!=1:
        raise RuntimeError('Cannot resolve the VINS node: '+str(message))
    endpoint=urlsplit(uri)
    local_hosts = tuple(host.casefold() for host in ('localhost','127.0.0.1','::1',os.uname().nodename))
    if (endpoint.hostname or '').casefold() not in local_hosts:
        raise RuntimeError('VINS node endpoint is not local')
    node=xmlrpc.client.ServerProxy(uri,transport=BoundedXmlrpc())
    code,message,pid=node.getPid('/ev_fault_evaluator')
    if code!=1:
        raise RuntimeError('VINS PID lookup failed')
    expected=(ROOT/'vio_ws/devel/lib/vins/vins_node').resolve()
    candidates=[]
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():continue
        try:
            fields=(directory/'stat').read_text().rsplit(')',1)[1].split()
            if fields[0]!='Z' and directory.stat().st_uid==os.getuid() and (directory/'exe').resolve()==expected:
                candidates.append(int(directory.name))
        except (OSError,ValueError,IndexError):
            continue
    if candidates!=[pid]:
        raise RuntimeError('Cannot uniquely establish the owned VINS estimator process')
    ancestor=int(pid);visited=[]
    while ancestor not in visited and ancestor>0:
        visited.append(ancestor)
        if ancestor==manager['pid']:break
        fields=(Path('/proc')/str(ancestor)/'stat').read_text().rsplit(')',1)[1].split()
        ancestor=int(fields[1])
    else:
        raise RuntimeError('VINS node is not a descendant of the owned system manager')
    return {'node':vins[0],'pid':pid,'binary':str(expected),'ancestor_pids':visited,'restart_subscribers':subscribers}


def poll_px4(binary,cycle):
    from concurrent.futures import ThreadPoolExecutor
    from record_ev_fusion import bounded_command,parse_listener
    topics=('vehicle_visual_odometry','vehicle_local_position','estimator_status_flags',
            'estimator_aid_src_ev_pos','estimator_aid_src_ev_vel',
            'vehicle_status','failsafe_flags')
    def poll(topic):
        result=bounded_command([str(binary),topic,'-i','0','-n','1'],.7)
        result.update(kind='topic',topic=topic,cycle=cycle,**parse_listener(result['stdout']))
        return result
    with ThreadPoolExecutor(max_workers=4) as workers:
        return list(workers.map(poll,topics))


class FlightJournal:
    def __init__(self,path):
        self.path=Path(path);self.offset=self.path.stat().st_size if self.path.exists() else 0
        self.events=[];self.missions={}

    def read(self):
        if not self.path.exists():return
        with self.path.open('rb') as stream:
            stream.seek(self.offset);raw=stream.read()
        boundary=raw.rfind(b'\n')
        if boundary<0:return
        self.offset+=boundary+1
        for line in raw[:boundary+1].splitlines():
            if not line.strip():continue
            event=json.loads(line);self.events.append(event)
            if event.get('event')=='mission_started':
                mission=event['mission'];self.missions[mission['id']]=copy.deepcopy(mission)
            identifier=event.get('mission_id')
            if identifier in self.missions and event.get('event')=='mission_finished':
                self.missions[identifier].update(status=event['status'],error=event.get('error'),finished_at=event.get('time'))


def owned_emergency_landing(mission,original_id):
    """Use the Bridge's atomic parent ID; description/timing alone is not ownership."""
    return (isinstance(original_id,str) and bool(original_id) and isinstance(mission,dict)
            and isinstance(mission.get('id'),str) and bool(mission['id'])
            and mission['id']!=original_id and mission.get('parent_mission_id')==original_id
            and mission.get('steps')==[{'action':'land'}])


def original_fault_mission(mission,original_id):
    return (isinstance(original_id,str) and bool(original_id) and isinstance(mission,dict)
            and mission.get('id')==original_id and mission.get('description')==DESCRIPTION
            and mission.get('steps')==STEPS)


def recover_owned_fault_flight(client,operations,owner,original_id,report,on_state=None,
                               timeout=120.,clock=time.monotonic,sleep=time.sleep,
                               operator_replacement=False):
    """Send one guarded landing or wait for its owned child; never cancel a replacement.

    Every observation verifies the pinned manager/PX4 before and after HTTP.
    A lost/refused landing acknowledgment is reconciled using actual parent-ID
    state, without resubmitting. This recovery never upgrades the original error
    or the fault-contract result into a successful flight.
    """
    result={'requested':False,'confirmed_ground_disarmed':False,'landing_id':None}
    if not report.get('flight_submission_attempted') or operator_replacement:
        result['skipped']='no submitted owned flight or operator replacement already observed'
        return result
    if not math.isfinite(timeout) or timeout<=0:
        raise ValueError('Owned recovery requires a positive finite timeout')
    def observation():
        owner()
        state,web=operations.states()
        owner()
        if on_state is not None:
            on_state(state)
        if web.get('active_command_id'):
            raise RuntimeError('Operator command arrived; do not replace its flight')
        mission=state.get('mission') or {}
        if owned_emergency_landing(mission,original_id):
            return state,'landing'
        if original_fault_mission(mission,original_id):
            return state,'original'
        raise RuntimeError('Current mission has no proven owned lineage; no replacement landing requested')
    state,kind=observation()
    mission=state['mission']
    report['recovery_ownership']={'original_mission_id':original_id,'current_mission_id':mission['id'],
        'kind':kind,'parent_mission_id':mission.get('parent_mission_id')}
    if kind=='landing':
        report['recovery']='waiting for the already active owned landing'
    elif state.get('armed') is True or mission.get('status')=='running':
        # The Bridge serializes this comparison with cancellation/submission.
        # Any replacement between the state read and this POST is rejected.
        owner()
        result['requested']=True
        report['fallback_landing_submission_attempted']=True
        try:
            response=client.request('/land',{'expected_mission_id':mission['id']})
            report['fallback_landing_response']=response
            report['fallback_landing_acknowledged']=(isinstance(response,dict)
                and owned_emergency_landing(response.get('mission'),original_id))
        except Exception as exc:
            report['fallback_landing_request_error']=str(exc)
        owner()
        state,kind=observation()
    deadline=clock()+timeout
    while True:
        mission=state['mission']
        if kind=='landing':
            if result['landing_id'] is not None and result['landing_id']!=mission['id']:
                raise RuntimeError('Owned landing was replaced; do not claim another worker completion')
            result['landing_id']=mission['id']
        elif result['landing_id'] is not None:
            raise RuntimeError('Owned landing reverted to another mission; stop recovery observation')
        stopped=mission.get('status') in ('succeeded','failed','cancelled')
        if (state.get('armed') is False and type(state.get('landed_state')) is int
                and state['landed_state']==1 and stopped):
            result.update(confirmed_ground_disarmed=True,final_state=state)
            return result
        if clock()>=deadline:
            raise TimeoutError('Owned recovery did not reach grounded, disarmed and stopped worker state')
        sleep(.25)
        state,kind=observation()


def legitimate_safety_landing(mission,events,injection,original_id):
    if not injection or not owned_emergency_landing(mission,original_id):
        return False
    if mission.get('description')!='Operator requested landing' or mission.get('steps')!=[{'action':'land'}]:
        return False
    starts=[e for e in events if e.get('event')=='mission_started' and e.get('mission',{}).get('id')==mission['id']]
    stops=[e for e in events if e.get('event')=='mission_finished' and e.get('mission_id')==original_id
           and e.get('status') in ('failed','cancelled')]
    if not starts or not stops:return False
    start=starts[0]['time']
    return any(e.get('event')=='external_vision_safety_landing' and e.get('reason')
               and injection['issued_wall_s']<=e['time']<=start and start-e['time']<=4
               and any(stop['time']<=start for stop in stops) for e in events)


def autopilot_landing_evidence(records, issued):
    """Identify PX4's real state, including the no-XY DESCEND fallback.

    A heartbeat AUTO.LAND label is insufficient: it encodes states 12 and 18.
    Source freshness and altitude/attitude/gyro checks use the same poll cycle.
    This records a mode observation, not landing success or horizontal control.
    """
    if type(issued) not in (int,float) or not math.isfinite(issued):
        return []
    valid = [r for r in records if r.get('return_code') == 0 and r.get('published') is True
             and type(r.get('cycle')) is int and type(r.get('started_at_wall_s')) in (int,float)
             and math.isfinite(r['started_at_wall_s']) and r['started_at_wall_s'] >= issued]
    result = []
    for record in valid:
        if record.get('topic') != 'vehicle_status':
            continue
        state = record['fields']; nav = state.get('nav_state')
        stamp, activated = state.get('timestamp'), state.get('nav_state_timestamp')
        if (type(nav) is not int or nav not in (12,18) or state.get('arming_state') != 2
                or type(state.get('arming_state')) is not int
                or any(type(v) is not int or v <= 0 for v in (stamp,activated)) or activated > stamp):
            continue
        cycle = [r for r in valid if r['cycle'] == record['cycle']]
        local = next((r['fields'] for r in cycle if r.get('topic') == 'vehicle_local_position'), {})
        flags = next((r['fields'] for r in cycle if r.get('topic') == 'failsafe_flags'), {})
        clock, flag_stamp = local.get('timestamp'), flags.get('timestamp')
        if (any(type(v) is not int or v <= 0 for v in (clock, flag_stamp))
                or not -.05 <= (clock-stamp)/1e6 <= 1.5
                or not -.05 <= (clock-flag_stamp)/1e6 <= 1.5
                or any(flags.get(k) is not False for k in (
                    'local_altitude_invalid','attitude_invalid','angular_velocity_invalid'))
                or nav == 12 and state.get('failsafe') is not True):
            continue
        result.append({'navigation_state': nav, 'kind': 'position_landing' if nav == 18
                       else 'descend_without_horizontal_control',
                       'cycle': record['cycle'], 'status_publish_stamp_sim_s': stamp/1e6,
                       'navigation_state_activated_sim_s': activated/1e6,
                       'failsafe_publish_stamp_sim_s': flag_stamp/1e6,
                       'observed_local_clock_sim_s': clock/1e6,
                       'failsafe': state.get('failsafe'),
                       'local_position_invalid': flags.get('local_position_invalid'),
                       'altitude_attitude_gyro_requirements_observed_valid': True})
    return result


def evaluate(kind,states,sensor_events,px4_records,flight_events,missions,original_id,injection,trajectory):
    checks=[]
    def check(name,value,observed):checks.append({'name':name,'passed':bool(value),'observed':observed})
    original=missions.get(original_id) or {}
    final=states[-1]['state'] if states else {}
    issued=injection.get('issued_wall_s') if injection else None
    after=[e for e in sensor_events if issued is not None and e.get('receipt_wall_s',0)>=issued]
    before=[e for e in sensor_events if issued is not None and e.get('receipt_wall_s',0)<issued and e.get('kind')=='status']
    baseline=before[-1]['value'] if before else (injection or {}).get('adapter_before',{})
    statuses=[e for e in after if e.get('kind')=='status']
    check('actual_fault_trigger_armed_cruise',bool(injection and injection.get('trigger',{}).get('armed') is True
          and injection.get('trigger',{}).get('mode')=='OFFBOARD'),bool(injection))
    applied=[]
    if kind in ('drop','delay'):
        applied=[e for e in statuses if e['value'].get('test_fault',{}).get('id')==(injection or {}).get('id')
                 and e['value']['test_fault'].get('kind')==kind and e['value']['test_fault'].get('applied_count',0)>0
                 and e['value']['test_fault'].get('synthetic') is True and e['value']['test_fault'].get('simulation_only') is True]
        check('actual_adapter_fault_applied',bool(applied),max((e['value']['test_fault']['applied_count'] for e in applied),default=0))
        incoming=[e for e in after if e.get('kind')=='propagated' and e.get('receipt_wall_s',0)<=issued+injection['duration_s']]
        check('real_vins_source_kept_advancing_during_transport_fault',
              len({e.get('stamp_sim_s') for e in incoming if isinstance(e.get('stamp_sim_s'),(int,float))})>=2,len(incoming))
        if kind=='delay':
            rejected=[e for e in applied if e['value'].get('rejected_count',0)>baseline.get('rejected_count',0)
                      and (e['value'].get('last_rejection') or e['value'].get('reasons'))]
            check('historical_real_vio_state_rejected',bool(rejected),
                  [e['value'].get('last_rejection') for e in rejected][:5])
    else:
        check('actual_upstream_restart_published',bool(injection and injection.get('estimator_restart_published')
              and injection.get('vins_target',{}).get('pid')),bool(injection and injection.get('estimator_restart_published')))
        check('actual_restart_message_observed',any(e.get('kind')=='restart' and e.get('value') is True for e in after),len(after))
        check('adapter_epoch_reset_latched',any(e['value'].get('epoch',0)>baseline.get('epoch',0)
              and e['value'].get('reset_latched') is True for e in statuses),[e['value'].get('epoch') for e in statuses[:6]])
        check('actual_vins_tracking_reset_observed',any(e.get('kind')=='tracking' and e.get('value') is False for e in after),
              sum(e.get('kind')=='tracking' and e.get('value') is False for e in after))
    # Independently measured PX4 EV uORB must stop advancing while its own
    # local position/clock keeps updating. Adapter status alone is insufficient.
    freeze=[]
    window_end=(issued+min(injection.get('duration_s',3.),4.)) if issued is not None else -1
    for record in px4_records:
        if (issued is not None and issued+.3<=record.get('started_at_wall_s',0)<=window_end
                and record.get('topic')=='vehicle_visual_odometry' and record.get('published') and record.get('return_code')==0):
            freeze.append(record)
    unchanged=any(b['started_at_wall_s']-a['started_at_wall_s']>=.4
                  and all(type(a['fields'].get(k)) is int and a['fields'][k]>0 for k in ('timestamp','timestamp_sample'))
                  and a['fields'].get('timestamp_sample')==b['fields'].get('timestamp_sample')
                  and a['fields'].get('timestamp')==b['fields'].get('timestamp')
                  for a in freeze for b in freeze if b['started_at_wall_s']>a['started_at_wall_s'])
    check('actual_px4_ev_input_stopped',unchanged,{'poll_count':len(freeze),'timestamps':[r['fields'].get('timestamp_sample') for r in freeze]})
    local=[r for r in px4_records if r.get('topic')=='vehicle_local_position' and r.get('published')
           and r.get('return_code')==0 and issued is not None and issued<=r.get('started_at_wall_s',0)<=window_end]
    check('px4_clock_continued_during_fault',len({r['fields'].get('timestamp') for r in local})>=2,len(local))
    forward=[e['value'].get('forwarded_count') for e in (applied if kind!='reset' else statuses)
             if issued is not None and e['receipt_wall_s']<=window_end and type(e['value'].get('forwarded_count')) is int]
    check('actual_adapter_forward_count_stopped',len(forward)>=2 and len(set(forward))==1,forward[:10])
    safety=[e for e in flight_events if e.get('event')=='external_vision_safety_landing'
            and issued is not None and e['time']>=issued]
    landings=[m for m in missions.values() if legitimate_safety_landing(m,flight_events,injection,original_id)]
    check('automatic_ev_supervisor_landing_event',bool(safety),[e.get('reason') for e in safety])
    check('original_worker_preserved_failed_or_cancelled',original.get('status') in ('failed','cancelled'),original.get('status'))
    completed_landings=[m for m in landings if m.get('status')=='succeeded' and any(
        e.get('event')=='step_completed' and e.get('mission_id')==m['id'] and e.get('action')=='land' for e in flight_events)]
    check('independent_supervisor_landing_completed',bool(completed_landings),
          [{'id':m.get('id'),'status':m.get('status')} for m in landings])
    check('actual_autoland_mode_observed',any(row['state'].get('mode')=='AUTO.LAND' and row['wall_time']>=(issued or float('inf')) for row in states),
          sorted({row['state'].get('mode','') for row in states}))
    autopilot = autopilot_landing_evidence(px4_records, issued)
    check('actual_px4_landing_or_descend_state_observed', bool(autopilot), autopilot)
    final_mission=final.get('mission') or {}
    check('final_ground_disarmed_no_running_worker',final.get('armed') is False
          and type(final.get('landed_state')) is int and final['landed_state']==1
          and final_mission.get('status') in ('succeeded','failed','cancelled')
          and owned_emergency_landing(final_mission,original_id),
          {'armed':final.get('armed'),'landed_state':final.get('landed_state'),
           'mission_id':final_mission.get('id'),'parent_mission_id':final_mission.get('parent_mission_id')})
    clearance=(trajectory or {}).get('minimum_interpolated_segment_clearance_m')
    check('recorded_obstacle_clearance_at_least_1m',clearance is not None and clearance>=1.,clearance)
    return {'passed':all(c['passed'] for c in checks),'checks':checks,'original_mission_status':original.get('status'),
            'quality_evaluation_version': 'ev_fault_raw_autopilot_state_v2',
            'autopilot_landing_states': autopilot,
            'heartbeat_auto_land_label_does_not_distinguish_descend': True,
            'requested_mission_completed':original.get('status')=='succeeded',
            'expected_safety_contract_pass':all(c['passed'] for c in checks),'scope':'actual EV chain, original mission cancellation/failure and independently observed automatic landing',
            'fault_kind':kind,'synthetic':kind in ('drop','delay'),'real_estimator_restart':kind=='reset'}


def wait_for_restart_transports(node_uri,target_node,evidence):
    """Check actual outgoing TCPROS peer identities before one restart publish."""
    endpoint=urlsplit(node_uri)
    local_hosts=tuple(host.casefold() for host in ('localhost','127.0.0.1','::1',os.uname().nodename))
    if (endpoint.scheme!='http' or (endpoint.hostname or '').casefold() not in local_hosts
            or endpoint.username is not None or endpoint.password is not None
            or endpoint.path not in ('','/') or endpoint.query or endpoint.fragment):
        raise RuntimeError('Restart publisher node endpoint is not a local ROS API')
    required={target_node,'/uav_ev_bridge'}
    if not isinstance(target_node,str) or not target_node.startswith('/') or len(required)!=2:
        raise RuntimeError('Restart requires distinct verified VINS and EV targets')
    node=xmlrpc.client.ServerProxy(node_uri,transport=BoundedXmlrpc())
    deadline=time.monotonic()+2.
    evidence.update(required_nodes=sorted(required),connection_publication_deadline_wall_s=2.,
                    per_RPC_timeout_wall_s=.5,
                    ready_before_single_publish=False,connected_snapshot_is_callback_ack=False)
    while True:
        if time.monotonic()>=deadline:
            raise RuntimeError('Verified VINS and EV restart transports did not connect before deadline')
        code,message,rows=node.getBusInfo('/ev_fault_evaluator')
        evidence.update(sampled_at_wall_s=time.time(),get_bus_info_code=code,
                        get_bus_info_message=message,actual_bus_rows=rows)
        if code!=1 or not isinstance(rows,(list,tuple)):
            raise RuntimeError('Cannot obtain actual restart publisher transports')
        peers={row[1]for row in rows if isinstance(row,(list,tuple))and len(row)==7
               and type(row[0])is int and row[0]>=0 and isinstance(row[1],str)
               and row[2]=='o'and row[3]=='TCPROS'and row[4]=='/vins_restart'and row[5]is True}
        evidence['observed_connected_outgoing_nodes']=sorted(peers)
        if time.monotonic()>=deadline:
            raise RuntimeError('Restart transport confirmation arrived after publication deadline')
        if required<=peers:
            evidence['ready_before_single_publish']=True
            return
        if time.monotonic()>=deadline:
            raise RuntimeError('Verified VINS and EV restart transports did not connect')
        time.sleep(.02)


def run(args):
    from run_reliability_suite import SystemOperations,safe_ground,owned_px4_process,process_identity
    from record_ev_fusion import stable_px4_identity,bounded_command,parse_parameters
    from test_ev_shadow import RosObservations,require_shadow_health
    from test_avoidance_scene import load_ground_truth,trajectory_evaluation,segment_rectangle_distance
    operations=SystemOperations();operations.px4_directory=args.px4_directory.resolve()
    manager=operations.initial_manager()
    if manager is None:raise RuntimeError('An owned complete simulation must already be running')
    binary=operations.px4_directory/'build/px4_sitl_default/bin/px4-listener'
    identity=stable_px4_identity(owned_px4_process(manager,binary))
    initial,web=operations.states();safe_ground(initial,web);require_ev_health(initial)
    layout,objects=load_ground_truth(args.layout)
    if initial.get('scene',{}).get('world_sha256')!=layout['world_sha256'] or not initial.get('scene',{}).get('verified'):
        raise RuntimeError('Evaluator collision layout does not match the verified active scene')
    home=initial['gazebo']['position_m'][:2]
    if min(segment_rectangle_distance(home,home,o['collision_aabb_world_m'])['distance_m'] for o in objects)<1.:
        raise RuntimeError('Independent takeoff-pad clearance is less than one metre')
    parameters=Parameters()
    if parameters.get('/use_sim_time') is not True or parameters.get('/mavros/fcu_url')!='udp://:14540@localhost:14557':
        raise RuntimeError('Fault injection requires the actual local simulated ROS/FCU configuration')
    if args.kind!='reset' and (initial['external_vision']['adapter'].get('test_fault') or {}).get('enabled') is not True:
        raise RuntimeError('EV adapter transport fault hooks must have been explicitly enabled')
    target=verify_vins_restart_target(parameters,manager) if args.kind=='reset' else None
    args.output.mkdir(parents=True,exist_ok=False)
    report={'schema_version':1,'source':'actual_ev_chain_fault_sortie','kind':args.kind,'started_at_wall_s':time.time(),
        'actual_model_call':False,'flight_submission_attempted':False,'physical_flight_observed':False,
        'requested_mission_completed':False,'expected_safety_contract_pass':False,'automatic_flight_retries':0,
        'manager':manager,'px4_process':identity,'initial_state':initial,'steps':STEPS,
        'fault_parameter':PARAMETER if args.kind!='reset' else None,'vins_target':target}
    collector=None;stream=None;states=[];records=[];injection=None;owned_parameter=None;original_id=None
    journal=FlightJournal(ROOT/'logs/flight.jsonl');fallback_requested=False;operator_replacement=False
    client=BridgeClient()
    def owner():
        if process_identity(manager['run_directory'])!=manager:raise RuntimeError('Owned manager changed')
        pinned_process_valid(identity)
    try:
        import rospy
        from std_msgs.msg import String,Bool
        collector=RosObservations();stream=(args.output/'sensor_samples.jsonl').open('x')
        def ekf_status(message):
            try:collector.add('ekf',{'value':json.loads(message.data)})
            except ValueError as exc:collector.error=str(exc)
        collector.subscribers.append(rospy.Subscriber('/uav/vio/ekf_status',String,ekf_status,queue_size=10,tcp_nodelay=True))
        deadline=time.monotonic()+args.timeout
        with (args.output/'state_samples.jsonl').open('x') as state_stream, (args.output/'px4_samples.jsonl').open('x') as px4_stream:
            ready_deadline=time.monotonic()+30
            while True:
                collector.flush(stream)
                try:require_shadow_health(collector.snapshot(),'fuse',collector.clock(),time.monotonic());break
                except RuntimeError:
                    if time.monotonic()>ready_deadline:raise
                    time.sleep(.1)
            owner();prepare_ground_preflight(client,operations,manager,report)
            parameter_record=bounded_command([str(binary.with_name('px4-param')),'show','EKF2_*'],1.5)
            parameter_record['values']=parse_parameters(parameter_record['stdout']);report['parameters_observed']=parameter_record
            current,current_web=operations.states();safe_ground(current,current_web);require_ev_health(current)
            report['flight_submission_attempted']=True
            accepted=client.request('/mission',{'steps':STEPS,'description':DESCRIPTION});report['accepted_response']=accepted
            original_id=accepted['mission']['id'];report['original_mission_id']=original_id
            print('Started actual EV '+args.kind+' fault mission '+original_id,flush=True)
            cycle=0;next_poll=0.;landing_id=None
            while time.monotonic()<deadline:
                owner();collector.flush(stream);journal.read()
                if collector.error:raise RuntimeError(collector.error)
                state,current_web=operations.states();owner();mission=state.get('mission') or {}
                row={'wall_time':time.time(),'state':state};states.append(row)
                state_stream.write(json.dumps(row,allow_nan=False)+'\n');state_stream.flush()
                if current_web.get('active_command_id'):
                    operator_replacement=True;raise RuntimeError('Operator command arrived; preserve operator priority')
                if mission.get('id'):
                    journal.missions[mission['id']]=copy.deepcopy(mission)
                if state.get('armed'):report['physical_flight_observed']=True
                if time.monotonic()>=next_poll:
                    batch=poll_px4(binary,cycle);records.extend(batch);cycle+=1
                    for record in batch:px4_stream.write(json.dumps(record,allow_nan=False)+'\n')
                    px4_stream.flush();next_poll=time.monotonic()+.4
                if mission.get('id')!=original_id:
                    # The Bridge writes this parent ID under its submission
                    # lock before the landing is visible. Journal events may
                    # arrive later; the final safety evaluation still requires
                    # the watchdog reason and actual completed landing events.
                    if not owned_emergency_landing(mission,original_id):
                        operator_replacement=True;raise RuntimeError('Unproven replacement mission; do not compete with it')
                    if landing_id is not None and landing_id!=mission['id']:
                        operator_replacement=True;raise RuntimeError('Owned landing was replaced; preserve the replacement worker')
                    landing_id=mission['id']
                elif not original_fault_mission(mission,original_id):
                    operator_replacement=True;raise RuntimeError('Original task identity no longer matches its accepted description and steps')
                if injection is None and mission.get('id')==original_id and mission.get('action')=='hold' and mission.get('status')=='running':
                    require_ev_health(state,armed=True);require_shadow_health(collector.snapshot(),'fuse',collector.clock(),time.monotonic())
                    issued=time.time();identifier=args.kind+'_'+uuid.uuid4().hex[:12]
                    injection={'id':identifier,'kind':args.kind,'duration_s':args.duration,'issued_wall_s':issued,
                               'trigger':state,'adapter_before':copy.deepcopy(state['external_vision']['adapter']),
                               'estimator_restart_published':False,'synthetic':args.kind!='reset','simulation_only':True}
                    if args.kind=='reset':
                        injection['vins_target']=verify_vins_restart_target(parameters,manager)
                        publisher=rospy.Publisher('/vins_restart',Bool,queue_size=1,latch=False)
                        injection['restart_transport_evidence_before_publish']={}
                        wait_for_restart_transports(rospy.get_node_uri(),injection['vins_target']['node'],
                                                    injection['restart_transport_evidence_before_publish'])
                        owner()
                        current_trigger,current_trigger_web=operations.states()
                        owner()
                        current_mission=current_trigger.get('mission') or {}
                        if (current_trigger_web.get('active_command_id')
                                or not original_fault_mission(current_mission,original_id)
                                or current_mission.get('status')!='running' or current_mission.get('action')!='hold'):
                            operator_replacement=True
                            raise RuntimeError('Task ownership changed while waiting for restart transport; preserve operator priority')
                        require_ev_health(current_trigger,armed=True)
                        require_shadow_health(collector.snapshot(),'fuse',collector.clock(),time.monotonic())
                        injection['requested_wall_s']=injection['issued_wall_s']
                        injection['issued_wall_s']=time.time()
                        owner()
                        publisher.publish(Bool(data=True));injection['estimator_restart_published']=True
                        injection['published_wall_s']=time.time();time.sleep(.1)
                    else:
                        owned_parameter=fault_request(args.kind,args.duration,identifier,issued)
                        injection['parameter_request']=owned_parameter;injection['parameter_readback']=parameters.install(owned_parameter)
                    (args.output/'injection.json').write_text(json.dumps(injection,indent=2,allow_nan=False)+'\n')
                    print('Injected actual EV '+args.kind+' chain fault '+identifier,flush=True)
                if landing_id and mission.get('status')!='running':
                    if mission.get('status')!='succeeded':raise RuntimeError('Supervisor landing failed: '+str(mission.get('error')))
                    if state.get('armed') is False and state.get('landed_state')==1:break
                if mission.get('id')==original_id and mission.get('status')=='succeeded':
                    raise RuntimeError('Original hold task completed instead of supervisor interrupting it')
                if mission.get('id')==original_id and mission.get('status') in ('failed','cancelled') and injection is None:
                    raise RuntimeError('Original flight failed before the EV fault could be applied')
                time.sleep(.1)
            else:raise TimeoutError('EV fault sortie exceeded wall-time limit')
            report['final_state']=state
    except (Exception,KeyboardInterrupt) as exc:
        report.update(error=str(exc),error_type=type(exc).__name__)
        try:
            def record_recovery_state(state):
                journal.read()
                if collector and stream:collector.flush(stream)
                row={'wall_time':time.time(),'state':state};states.append(row)
                with (args.output/'state_samples.jsonl').open('a') as recovery_stream:
                    recovery_stream.write(json.dumps(row,allow_nan=False)+'\n')
                mission=state.get('mission') or {}
                if mission.get('id'):journal.missions[mission['id']]=copy.deepcopy(mission)
            recovery=recover_owned_fault_flight(client,operations,owner,original_id,report,
                on_state=record_recovery_state,operator_replacement=operator_replacement)
            report['owned_recovery']=recovery
            report['fallback_ground_disarmed_confirmed']=recovery['confirmed_ground_disarmed']
            if 'final_state' in recovery:report['final_state']=recovery['final_state']
        except Exception as landing_error:report['recovery_error']=str(landing_error)
        finally:
            # Preserve a guarded request attempt even if its acknowledgment,
            # ownership checks or the bounded ground wait subsequently fail.
            fallback_requested=report.get('fallback_landing_submission_attempted') is True
    finally:
        try:
            owner();report['fault_clear']=parameters.clear_owned(owned_parameter)
        except Exception as clear_error:report['fault_clear_error']=str(clear_error)
        if collector:
            collector.close();collector.flush(stream)
        if stream:stream.close()
        try:journal.read()
        except Exception as journal_error:report['flight_journal_error']=str(journal_error)
        report.update(injection=injection,missions=list(journal.missions.values()),flight_events=journal.events,
            independent_runner_fallback_requested=fallback_requested,finished_at_wall_s=time.time(),state_sample_count=len(states))
        trajectory=None
        if len(states)>1:
            try:trajectory=trajectory_evaluation(states,objects,home,home,initial['scene']['navigation'])
            except Exception as geometry_error:report['geometry_evaluation_error']=str(geometry_error)
        report['trajectory']=trajectory
        report['validation']=evaluate(args.kind,states,collector.events if collector else [],records,
            journal.events,journal.missions,original_id,injection,trajectory)
        report['expected_safety_contract_pass']=not report.get('error') and report['validation']['passed'] and not fallback_requested
        report['requested_mission_completed']=report['validation']['requested_mission_completed']
        report['duration_wall_s']=report['finished_at_wall_s']-report['started_at_wall_s']
        (args.output/'flight_events.json').write_text(json.dumps(journal.events,indent=2,allow_nan=False)+'\n')
        (args.output/'task_validation.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind',choices=('drop','delay','reset'),required=True)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--duration',type=float,default=3.)
    parser.add_argument('--timeout',type=float,default=300.)
    parser.add_argument('--layout',type=Path,default=ROOT/'scene/obstacle_layout.json')
    parser.add_argument('--px4-directory',type=Path,default=Path(os.getenv('PX4_DIR','/home/dut/PX4-Autopilot')))
    args=parser.parse_args(argv)
    if not math.isfinite(args.duration) or not .5<=args.duration<=4 or not math.isfinite(args.timeout) or not 30<=args.timeout<=600:
        parser.error('duration 0.5..4 wall seconds; sortie timeout 30..600 wall seconds')
    if not args.execute:
        print(json.dumps({'execute':False,'actual_requests':False,'kind':args.kind,'flight_steps_if_requested':STEPS,
            'ev_parameter':PARAMETER if args.kind!='reset' else None,'real_restart_topic':'/vins_restart' if args.kind=='reset' else None,
            'original_failed_or_cancelled_expected':True,'landing_worker_must_be_independent':True,
            'requires_real_selected_ekf_fusion':True,'automatic_flight_retries':0},indent=2))
        return 0
    if args.output is None:parser.error('--execute requires --output NEW_DIRECTORY')
    result=run(args)
    print(json.dumps({'report':str(args.output/'task_validation.json'),'requested_mission_completed':result['requested_mission_completed'],
        'expected_safety_contract_pass':result['expected_safety_contract_pass'],'error':result.get('error')}))
    return 0 if result['expected_safety_contract_pass'] else 1


if __name__=='__main__':raise SystemExit(main())
