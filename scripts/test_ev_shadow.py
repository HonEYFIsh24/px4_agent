#!/usr/bin/env python3
"""Independent real-sensor EV quality evaluation; default prints a plan only.

Use --observe-only for a stationary observation, or --fly for the explicit
short production-executor sortie. Gazebo truth is only an evaluator input.
Each VIO source is aligned once by initial yaw and translation, without scale
or trajectory fitting. ModelStates has no measurement stamp; its ROS simulation
receipt time is recorded, so truth matching has an explicit timing limitation.
"""
import argparse
import bisect
import copy
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from client import BridgeClient

STEPS = [{'action': 'takeoff', 'altitude': 3.5},
         {'action': 'goto', 'x': 1., 'y': 0., 'altitude': 3.5},
         {'action': 'goto', 'x': 1., 'y': 1., 'altitude': 3.5},
         {'action': 'return_home'}, {'action': 'land'}]
TOPICS = {'shadow': '/uav/vio/ev_shadow', 'status': '/uav/vio/ev_status',
          'optimized': '/vins_estimator/odometry', 'propagated': '/vins_estimator/imu_propagate',
          'tracking': '/vins_estimator/tracking_ok', 'truth': '/gazebo/model_states',
          'lidar': '/uav/lidar/scan', 'restart': '/vins_restart',
          'native_health': '/vins_estimator/native_health'}
TOPICS['timing'] = '/uav/vio/ev_timing'
LIMITS = {'position_rmse_m': .25, 'maximum_position_error_m': .6,
          'ground_drift_m': .15, 'ground_truth_displacement_m': .15,
          'shadow_rate_sim_hz': 30., 'shadow_max_age_sim_s': .10,
          'shadow_max_future_sim_s': .02, 'truth_bracket_max_gap_sim_s': .05,
          'minimum_clearance_m': 1.}
# New independent experiment criterion. Existing position/time limits and old
# immutable evidence retain their previous meaning; this is not FCU evidence.
GRAVITY_CRITERION = {'tag': 'shadow_gravity_axis_6deg_v1', 'maximum_error_deg': 6.,
                    'maximum_error_rad': math.radians(6.), 'source': 'shadow',
                    'alignment': 'existing_once_yaw_and_translation_no_roll_pitch_fit',
                    'truth': 'headerless_ModelStates_receipt_bracket_interpolation',
                    'scope': 'independent_simulation_evaluation_not_FCU_control_proof',
                    'ground_truth_feedback': False}


def norm(values):
    return math.sqrt(sum(v*v for v in values))


def sub(a, b):
    return [x-y for x, y in zip(a, b)]


def add(a, b):
    return [x+y for x, y in zip(a, b)]


def finite_vector(value,size):
    return isinstance(value,(list,tuple)) and len(value)==size and all(
        isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in value)


def finite_number(value):
    return type(value) in (int,float) and math.isfinite(value)


def valid_quaternion(value):
    return finite_vector(value,4) and abs(norm(value)-1)<=.025


def normalized_quaternion(value):
    if not valid_quaternion(value):
        raise ValueError('invalid measurement/truth quaternion')
    length=norm(value)
    return [v/length for v in value]


def valid_source_sample(value):
    return (finite_vector(value.get('p'),3) and valid_quaternion(value.get('q'))
        and finite_vector(value.get('v'),3) and finite_number(value.get('stamp_sim_s'))
        and finite_number(value.get('receipt_sim_s')))


def sanitize(value):
    if isinstance(value,float) and not math.isfinite(value):
        return None
    if isinstance(value,dict):
        return {k:sanitize(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):
        return [sanitize(v) for v in value]
    return value


def rotate_yaw(vector, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [c*vector[0]-s*vector[1], s*vector[0]+c*vector[1], vector[2]]


def quaternion_yaw(q):
    x, y, z, w = normalized_quaternion(q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def rotate_quaternion(vector, quaternion):
    x, y, z, w = normalized_quaternion(quaternion)
    # Rodrigues equivalent for quaternion vector rotation, independent of adapter.
    cross = [y*vector[2]-z*vector[1], z*vector[0]-x*vector[2], x*vector[1]-y*vector[0]]
    cross2 = [y*cross[2]-z*cross[1], z*cross[0]-x*cross[2], x*cross[1]-y*cross[0]]
    return [vector[i]+2*w*cross[i]+2*cross2[i] for i in range(3)]


def interpolate_truth(truth, stamps, stamp, max_gap=.05):
    """Bracket by recorded truth receipt time; never extrapolate old truth."""
    index = bisect.bisect_left(stamps, stamp)
    if index < len(stamps) and abs(stamps[index]-stamp) < 1e-8:
        return truth[index] if valid_quaternion(truth[index].get('q')) else None
    if index == 0 or index == len(stamps):
        return None
    first, second = truth[index-1], truth[index]
    gap = second['stamp_sim_s']-first['stamp_sim_s']
    if gap <= 0 or gap > max_gap:
        return None
    fraction = (stamp-first['stamp_sim_s'])/gap
    if not valid_quaternion(first.get('q')) or not valid_quaternion(second.get('q')):
        return None
    q1, q2 = normalized_quaternion(first['q']), normalized_quaternion(second['q'])
    if sum(x*y for x, y in zip(q1, q2)) < 0:
        q2 = [-x for x in q2]
    quaternion = [a+fraction*(b-a) for a, b in zip(q1, q2)]
    length = norm(quaternion)
    if not math.isfinite(length) or length <= 0:
        return None
    result = {'stamp_sim_s': stamp, 'q': [x/length for x in quaternion], 'truth_bracket_gap_sim_s': gap}
    for key in ('p', 'v', 'w'):
        result[key] = [a+fraction*(b-a) for a, b in zip(first[key], second[key])]
    return result


def sample_statistics(samples):
    stamps = [s['stamp_sim_s'] for s in samples]
    deltas = [b-a for a, b in zip(stamps, stamps[1:])]
    span = stamps[-1]-stamps[0] if len(stamps) > 1 else 0
    unique = sorted(set(stamps))
    gaps = [b-a for a, b in zip(unique, unique[1:])]
    ages = [s['receipt_sim_s']-s['stamp_sim_s'] for s in samples]
    return {'sample_count': len(samples), 'distinct_stamp_count': len(unique), 'span_sim_s': span,
            'average_distinct_rate_sim_hz': (len(unique)-1)/(unique[-1]-unique[0]) if len(unique)>1 else None,
            'nonincreasing_source_stamp_count': sum(d<=0 for d in deltas),
            'maximum_source_gap_sim_s': max(gaps) if gaps else None,
            'maximum_receipt_minus_source_age_sim_s': max(ages) if ages else None,
            'minimum_receipt_minus_source_age_sim_s': min(ages) if ages else None}


def evaluate_source(samples, truth, source, ground_window, lever_arm=(0., 0., .02)):
    all_samples = [s for s in samples if s.get('kind') == source]
    samples = [s for s in all_samples if valid_source_sample(s)]
    result = dict(sample_statistics(samples), source=source, matched_count=0,
                  invalid_sample_count=len(all_samples)-len(samples),
                  alignment_method='one_initial_yaw_and_translation_no_scale_no_trajectory_refit')
    stamps = [s['stamp_sim_s'] for s in truth]
    pairs, alignment = [], None
    for measurement in samples:
        reference = interpolate_truth(truth, stamps, measurement['stamp_sim_s'])
        if reference is None:
            continue
        truth_position = reference['p']
        if source in ('optimized', 'propagated'):
            truth_position = add(truth_position, rotate_quaternion(lever_arm, reference['q']))
        if alignment is None:
            yaw = quaternion_yaw(reference['q'])-quaternion_yaw(measurement['q'])
            translation = sub(truth_position, rotate_yaw(measurement['p'], yaw))
            alignment = {'stamp_sim_s': measurement['stamp_sim_s'], 'yaw_rad': yaw,
                         'translation_m': translation, 'scale': 1., 'reference_point': 'body' if source=='shadow' else 'original_imu'}
        aligned_position = add(rotate_yaw(measurement['p'], alignment['yaw_rad']), alignment['translation_m'])
        error = sub(aligned_position, truth_position)
        velocity = measurement['v']
        reference_velocity = reference['v']
        if source == 'shadow':
            velocity = rotate_quaternion(velocity, measurement['q'])
        else:
            arm_world = rotate_quaternion(lever_arm, reference['q'])
            w = reference['w']
            reference_velocity = add(reference_velocity, [w[1]*arm_world[2]-w[2]*arm_world[1],
                w[2]*arm_world[0]-w[0]*arm_world[2], w[0]*arm_world[1]-w[1]*arm_world[0]])
        velocity_error = sub(rotate_yaw(velocity, alignment['yaw_rad']), reference_velocity)
        yaw_error = (quaternion_yaw(measurement['q'])+alignment['yaw_rad']-quaternion_yaw(reference['q'])+math.pi)%(2*math.pi)-math.pi
        x,y,z,w=measurement['q']
        c,s=math.cos(alignment['yaw_rad']/2),math.sin(alignment['yaw_rad']/2)
        aligned_q=[c*x-s*y,c*y+s*x,c*z+s*w,c*w-s*z]
        dot=abs(sum(a*b for a,b in zip(aligned_q,reference['q']))/norm(aligned_q)/norm(reference['q']))
        orientation_error=2*math.acos(min(1.,dot))
        local_z=rotate_quaternion([0.,0.,1.],aligned_q)
        true_z=rotate_quaternion([0.,0.,1.],reference['q'])
        gravity_axis_error=math.acos(max(-1.,min(1.,sum(a*b for a,b in zip(local_z,true_z)))))
        pairs.append({'stamp_sim_s': measurement['stamp_sim_s'], 'position_error_m': error,
                      'velocity_error_m_s': velocity_error, 'yaw_error_rad': yaw_error,
                      'orientation_error_rad':orientation_error,'gravity_axis_error_rad':gravity_axis_error,
                      'physical_body_position_m': reference['p']})
    result.update(alignment=alignment, matched_count=len(pairs), unmatched_count=len(samples)-len(pairs))
    if not pairs:
        result.update(position_rmse_m=None, maximum_position_error_m=None, ground_drift=None,
                      maximum_gravity_axis_error_rad=None, gravity_axis_rmse_rad=None)
        return result
    errors = [norm(p['position_error_m']) for p in pairs]
    result.update(position_rmse_m=math.sqrt(sum(e*e for e in errors)/len(errors)),
                  maximum_position_error_m=max(errors),
                  velocity_rmse_m_s=math.sqrt(sum(norm(p['velocity_error_m_s'])**2 for p in pairs)/len(pairs)),
                  yaw_rmse_rad=math.sqrt(sum(p['yaw_error_rad']**2 for p in pairs)/len(pairs)),
                  orientation_geodesic_rmse_rad=math.sqrt(sum(p['orientation_error_rad']**2 for p in pairs)/len(pairs)),
                  maximum_orientation_geodesic_error_rad=max(p['orientation_error_rad'] for p in pairs),
                  gravity_axis_rmse_rad=math.sqrt(sum(p['gravity_axis_error_rad']**2 for p in pairs)/len(pairs)),
                  maximum_gravity_axis_error_rad=max(p['gravity_axis_error_rad'] for p in pairs),
                  first_matched_stamp_sim_s=pairs[0]['stamp_sim_s'], last_matched_stamp_sim_s=pairs[-1]['stamp_sim_s'])
    ground = [p for p in pairs if ground_window and ground_window[0]<=p['stamp_sim_s']<=ground_window[1]]
    if len(ground)>1:
        result['ground_drift'] = {'span_sim_s': ground[-1]['stamp_sim_s']-ground[0]['stamp_sim_s'],
            'endpoint_error_change_m': norm(sub(ground[-1]['position_error_m'], ground[0]['position_error_m'])),
            'maximum_error_change_m': max(norm(sub(p['position_error_m'],ground[0]['position_error_m'])) for p in ground),
            'maximum_physical_displacement_m': max(norm(sub(p['physical_body_position_m'],ground[0]['physical_body_position_m'])) for p in ground),
            'source_stamp_interval_sim_s': ground_window}
    else:
        result['ground_drift'] = None
    return result


def timestamp_assessment(events):
    """Keep raw callback ages, distinguish proved /clock delivery reordering.

    The producer clock is observed at publication, never inferred from a sensor
    stamp. A negative consumer age needs both a matching fresh producer record
    and an actual subsequent /clock message within 50 ms wall time. All source
    stamps and the existing simulation-age limits remain unchanged.
    """
    samples=[r for r in events if r.get('kind')=='shadow']
    timings=[r.get('value') for r in events if r.get('kind')=='timing']
    timings=[r for r in timings if isinstance(r,dict)]
    clocks=[r for r in events if r.get('kind')=='clock']
    raw=sample_statistics(samples)
    result={'basis':'consumer_receipt_clock','raw_consumer':raw,'passed':False}
    if not samples:
        return result
    maximum_age=raw['maximum_receipt_minus_source_age_sim_s']
    minimum_age=raw['minimum_receipt_minus_source_age_sim_s']
    if not timings:
        result['passed']=(maximum_age<=LIMITS['shadow_max_age_sim_s']
                          and minimum_age>=-LIMITS['shadow_max_future_sim_s'])
        return result
    by_stamp={}; invalid=0
    for timing in timings:
        ns=timing.get('stamp_ns'); stamp=timing.get('stamp_sim_s')
        if (type(ns) is not int or ns<0 or type(stamp) not in (int,float)
                or not math.isfinite(stamp) or abs(ns/1e9-stamp)>5e-10 or ns in by_stamp):
            invalid+=1
        else:
            by_stamp[ns]=timing
    ages=[]; missing=0; reordered=[]; publication_intervals=[]
    for sample in samples:
        stamp=sample['stamp_sim_s']; ns=sample.get('stamp_ns')
        timing=by_stamp.get(ns) if type(ns) is int else None
        if timing is None:
            missing+=1
            if sample['receipt_sim_s']-stamp < -LIMITS['shadow_max_future_sim_s']:
                reordered.append({'stamp_sim_s':stamp,'raw_age_sim_s':sample['receipt_sim_s']-stamp,
                                  'producer_age_sim_s':None,'clock_delivery_delay_wall_s':None,'proved':False})
            continue
        clock=timing.get('actual_publish_clock_sim_s'); age=timing.get('measurement_age_at_publish_sim_s')
        if (type(clock) not in (int,float) or type(age) not in (int,float)
                or not math.isfinite(clock) or not math.isfinite(age)
                or abs(clock-stamp-age)>1e-7 or timing.get('source_timestamp_unchanged') is not True
                or timing.get('source')!='producer_ros_clock_at_publish'):
            invalid+=1
            continue
        ages.append(age)
        # Same-host monotonic timestamps bound enqueue-call to callback delay.
        # They describe this recorder, not the FCU, and never subtract pauses
        # from a measured consumer age or relax its existing upper limit.
        before=timing.get('shadow_publish_monotonic_before_s')
        after=timing.get('shadow_publish_monotonic_after_s')
        if any(key in timing for key in ('shadow_publish_monotonic_before_s',
                'shadow_publish_monotonic_after_s','monotonic_clock_scope','publish_semantics')):
            receipt=sample.get('receipt_monotonic_s')
            if (any(type(v) not in (int,float) or not math.isfinite(v) for v in (before,after,receipt))
                    or after<before or receipt<before or timing.get('monotonic_clock_scope')!='same_simulation_host_not_cross_machine'
                    or timing.get('publish_semantics')!='rospy_publish_call_not_socket_delivery'):
                invalid+=1
            else:
                publication_intervals.append({'stamp_ns':ns,
                    'callback_minus_publish_before_wall_s':receipt-before,
                    'callback_minus_publish_after_wall_s':receipt-after,
                    'publish_call_duration_wall_s':after-before})
        if sample['receipt_sim_s']-stamp < -LIMITS['shadow_max_future_sim_s']:
            received=sample.get('receipt_monotonic_s')
            follow=next((c for c in clocks if received is not None
                and 0<=c.get('receipt_monotonic_s',-1)-received<=.05
                and c.get('stamp_sim_s',-1)>=stamp-1e-8),None)
            reordered.append({'stamp_sim_s':stamp,'raw_age_sim_s':sample['receipt_sim_s']-stamp,
                'producer_age_sim_s':age,'clock_delivery_delay_wall_s':
                    follow['receipt_monotonic_s']-received if follow else None,'proved':follow is not None})
    result.update(basis='matched_producer_publish_clock_and_actual_clock_delivery',
        timing_match_count=len(ages),missing_count=missing,invalid_count=invalid,
        producer_minimum_age_sim_s=min(ages) if ages else None,
        producer_maximum_age_sim_s=max(ages) if ages else None,
        consumer_clock_reordering=reordered,
        producer_to_recorder_delivery={'same_host_monotonic_interval_count':len(publication_intervals),
            'maximum_callback_minus_publish_before_wall_s':max((p['callback_minus_publish_before_wall_s'] for p in publication_intervals),default=None),
            'maximum_callback_minus_publish_after_wall_s':max((p['callback_minus_publish_after_wall_s'] for p in publication_intervals),default=None),
            'maximum_rospy_publish_call_duration_wall_s':max((p['publish_call_duration_wall_s'] for p in publication_intervals),default=None),
            'semantics':'recorder_callback_after_deserialization_not_socket_or_fcu_receipt',
            'consumer_age_not_corrected':True})
    result['passed']=(bool(ages) and len(ages)/len(samples)>=.98 and invalid==0
        and min(ages)>=-LIMITS['shadow_max_future_sim_s'] and max(ages)<=LIMITS['shadow_max_age_sim_s']
        and maximum_age<=LIMITS['shadow_max_age_sim_s'] and all(r['proved'] for r in reordered))
    return result


def evaluate(events, ground_window, require_full_ground=True):
    raw_truth = [s for s in events if s.get('kind')=='truth']
    truth = [s for s in raw_truth if all(finite_vector(s.get(k),3) for k in ('p','v','w'))
        and valid_quaternion(s.get('q')) and finite_number(s.get('stamp_sim_s'))]
    # Do not conceal a simulation clock reset by sorting it into a continuous trajectory.
    truth_monotonic = all(b['stamp_sim_s']>=a['stamp_sim_s'] for a,b in zip(truth,truth[1:]))
    deduplicated=[]
    for row in truth:
        if deduplicated and row['stamp_sim_s']==deduplicated[-1]['stamp_sim_s']:
            deduplicated[-1]=row
        else:
            deduplicated.append(row)
    truth_duplicate_count=len(truth)-len(deduplicated)
    truth=deduplicated
    sources = {key: evaluate_source(events, truth if truth_monotonic else [], key, ground_window)
               for key in ('shadow','optimized','propagated')}
    statuses = [s for s in events if s.get('kind')=='status']
    tracking = [s for s in events if s.get('kind')=='tracking']
    shadow = sources['shadow']
    drift = shadow.get('ground_drift') or {}
    checks = []
    def check(name, passed, observed):
        checks.append({'name':name,'passed':bool(passed),'observed':observed})
    check('truth_receipt_clock_continuous',truth_monotonic and len(truth)>1,len(truth))
    check('truth_values_valid',len(raw_truth)==len(truth)+truth_duplicate_count,
          len(raw_truth)-len(truth)-truth_duplicate_count)
    check('shadow_values_valid',shadow['invalid_sample_count']==0,shadow['invalid_sample_count'])
    check('shadow_stamp_strictly_monotonic',shadow['sample_count']>1 and shadow['nonincreasing_source_stamp_count']==0,
          shadow['nonincreasing_source_stamp_count'])
    rate = shadow.get('average_distinct_rate_sim_hz')
    check('shadow_real_stamp_rate_at_least_30_sim_hz',rate is not None and rate>=LIMITS['shadow_rate_sim_hz'],rate)
    # Invalid source fields already fail shadow_values_valid. Do not let them
    # raise during age arithmetic or mask the explicit orientation criterion.
    timestamps=timestamp_assessment([s for s in events if s.get('kind')!='shadow' or valid_source_sample(s)])
    check('shadow_source_timestamp_fresh',timestamps['passed'],timestamps)
    check('truth_match_coverage',shadow['sample_count']>1 and shadow['matched_count']/shadow['sample_count']>=.98,
          {'matched':shadow['matched_count'],'total':shadow['sample_count']})
    check('shadow_position_rmse',shadow.get('position_rmse_m') is not None and shadow['position_rmse_m']<=LIMITS['position_rmse_m'],shadow.get('position_rmse_m'))
    check('shadow_maximum_position_error',shadow.get('maximum_position_error_m') is not None and shadow['maximum_position_error_m']<=LIMITS['maximum_position_error_m'],shadow.get('maximum_position_error_m'))
    maximum_gravity=shadow.get('maximum_gravity_axis_error_rad')
    gravity_valid=(finite_number(maximum_gravity) and maximum_gravity>=0
        and shadow['matched_count']>1 and shadow['invalid_sample_count']==0
        and len(raw_truth)==len(truth)+truth_duplicate_count)
    check('shadow_gravity_axis_within_6deg',gravity_valid and maximum_gravity<=GRAVITY_CRITERION['maximum_error_rad'],
          {'maximum_error_rad':maximum_gravity if finite_number(maximum_gravity) else None,
           'maximum_error_deg':math.degrees(maximum_gravity) if finite_number(maximum_gravity) else None,
           'orientation_evidence_valid':gravity_valid,'criterion':GRAVITY_CRITERION['tag']})
    if require_full_ground:
        check('ground_30_sim_seconds_observed',drift.get('span_sim_s',0)>=29.9,drift.get('span_sim_s'))
        check('ground_physical_stationarity',drift.get('maximum_physical_displacement_m',float('inf'))<=LIMITS['ground_truth_displacement_m'],drift.get('maximum_physical_displacement_m'))
        check('ground_30_sim_seconds_drift',drift.get('maximum_error_change_m',float('inf'))<=LIMITS['ground_drift_m'],drift.get('maximum_error_change_m'))
    epochs = sorted({s['value'].get('epoch') for s in statuses if type(s['value'].get('epoch')) is int})
    check('no_adapter_reset_or_latch',len(epochs)==1 and not any(s['value'].get('reset_latched') for s in statuses),epochs)
    check('shadow_output_observed',bool(statuses) and max((s['value'].get('shadow_count',0) for s in statuses),default=0)>0,len(statuses))
    return {'source_metrics':sources,'checks':checks,'passed':all(c['passed'] for c in checks),
            'quality_evaluation_version':'shadow_quality_gravity_axis_v2',
            'additional_independent_criterion':copy.deepcopy(GRAVITY_CRITERION),
            'truth_duplicate_receipt_stamp_count':truth_duplicate_count,
            'full_ground_30_sim_seconds_assessment':require_full_ground,
            'adapter_epochs':epochs,'tracking_sample_count':len(tracking),
            'tracking_false_count':sum(s.get('value') is False for s in tracking),
            'status_not_ready_count':sum(s['value'].get('ready') is not True for s in statuses),
            'status_reasons':sorted({str(reason) for s in statuses for reason in s['value'].get('reasons',[])}),
            'restart_message_count':sum(s.get('kind')=='restart' and s.get('value') is True for s in events),
            'limits':LIMITS,'truth_time_limitation':'Gazebo ModelStates has no header: ROS simulation receipt time; interpolation brackets <=0.05 sim s. No ground-truth feedback, no scale fit, no trajectory re-alignment.'}


class GcRecordingSession:
    """Suspend cyclic GC only while retaining bounded, acyclic sensor records.

    Reference counting remains active. Unregister all subscriptions before
    restoring the original GC state, including partial construction failures.
    This changes recorder scheduling, never source stamps or measured ages.
    """
    def __init__(self):
        self.subscriptions=[]; self.started=False; self.closed=False
        self.previous_enabled=None; self.started_monotonic_s=None
        self.finished_monotonic_s=None; self.cleanup_errors=[]

    def subscribe(self, factory, *args, **kwargs):
        if self.closed or self.started:
            raise RuntimeError('Subscriptions must be constructed before recording starts')
        subscription=factory(*args, **kwargs)
        self.subscriptions.append(subscription)
        return subscription

    def start(self):
        if self.closed or self.started:
            raise RuntimeError('Recording session cannot be started twice or after closing')
        self.previous_enabled=gc.isenabled()
        gc.disable()
        self.started_monotonic_s=time.monotonic()
        self.started=True

    def close(self):
        if self.closed:
            return
        self.closed=True
        try:
            for subscription in self.subscriptions:
                try:
                    subscription.unregister()
                except Exception as exc:
                    self.cleanup_errors.append(str(exc))
        finally:
            self.finished_monotonic_s=time.monotonic()
            if self.started and self.previous_enabled:
                gc.enable()
            elif self.started:
                gc.disable()

    def evidence(self):
        return {'policy':'cyclic_gc_suspended_during_bounded_acyclic_sensor_recording',
                'record_limit':200000,'reference_counting_active':True,
                'started':self.started,'closed':self.closed,
                'previous_gc_enabled':self.previous_enabled,
                'started_monotonic_s':self.started_monotonic_s,
                'finished_monotonic_s':self.finished_monotonic_s,
                'cleanup_errors':list(self.cleanup_errors),
                'source_stamps_and_age_limits_unchanged':True}


class RosObservations:
    """Record callbacks independently; filesystem writes occur in the runner loop."""
    def __init__(self):
        import rospy
        from gazebo_msgs.msg import ModelStates
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Bool,String
        from rosgraph_msgs.msg import Clock
        self.rospy=rospy
        rospy.init_node('ev_independent_evaluator',anonymous=True,disable_signals=True)
        # /clock and odometry use independent TCP streams. Disable Nagle and
        # bound the shared clock transport queue, preserving the raw receipt
        # clock and the unchanged source stamp/quality thresholds.
        self.lock=threading.Lock(); self.events=[]; self.pending=[]; self.latest={}; self.error=None
        self.recording_session=GcRecordingSession()
        def odometry(kind,message):
            p,q,v=message.pose.pose.position,message.pose.pose.orientation,message.twist.twist.linear
            self.add(kind,{'stamp_sim_s':message.header.stamp.to_sec(),'p':[p.x,p.y,p.z],
                'stamp_ns':message.header.stamp.to_nsec(),
                'q':[q.x,q.y,q.z,q.w],'v':[v.x,v.y,v.z],
                'parent_frame':message.header.frame_id,'child_frame':message.child_frame_id})
        def truth(message):
            if 'iris' not in message.name:
                return
            index=message.name.index('iris'); p,q=message.pose[index].position,message.pose[index].orientation
            v,w=message.twist[index].linear,message.twist[index].angular
            self.add('truth',{'stamp_sim_s':rospy.Time.now().to_sec(),'p':[p.x,p.y,p.z],
                'q':[q.x,q.y,q.z,q.w],'v':[v.x,v.y,v.z],'w':[w.x,w.y,w.z],
                'timestamp_semantics':'receipt_sim_time_no_source_header'})
        def status(message):
            try:
                value=json.loads(message.data)
                if not isinstance(value,dict):
                    raise ValueError('EV status must be an object')
                self.add('status',{'value':value})
            except (ValueError,TypeError) as exc:
                self.error=str(exc)
        def json_evidence(kind,message):
            try:
                value=json.loads(message.data)
                if not isinstance(value,dict):
                    raise ValueError(kind+' must be an object')
                self.add(kind,{'value':value})
            except (ValueError,TypeError) as exc:
                self.error=str(exc)
        def lidar(message):
            ranges=[float(r) if math.isfinite(r) else None for r in message.ranges]
            self.add('lidar',{'stamp_sim_s':message.header.stamp.to_sec(),'sample_count':len(ranges),
                'minimum_valid_range_m':min((r for r in ranges if r is not None and r>=message.range_min),default=None),
                'negative_infinite_count':sum(r==float('-inf') for r in message.ranges),
                'nan_range_count':sum(math.isnan(r) for r in message.ranges),
                'range_min':message.range_min,'range_max':message.range_max})
        subscribe=lambda *a,**kw:self.recording_session.subscribe(rospy.Subscriber,*a,**kw)
        try:
            self.clock_subscriber=subscribe('/clock',Clock,
                lambda m:self.add('clock',{'stamp_sim_s':m.clock.to_sec()}),queue_size=1,
                buff_size=4096,tcp_nodelay=True)
            subscribe(TOPICS['timing'],String,lambda m:json_evidence('timing',m),queue_size=100,tcp_nodelay=True)
            # Construction does not prove either TCP stream has delivered data.
            # Establish both evidence streams before subscribing to measurements;
            # retain every startup callback and the unchanged evaluator gates.
            self.wait_for_timestamp_sources()
            for k in ('shadow','optimized','propagated'):
                subscribe(TOPICS[k],Odometry,lambda m,k=k:odometry(k,m),queue_size=100,tcp_nodelay=True)
            subscribe(TOPICS['truth'],ModelStates,truth,queue_size=100)
            subscribe(TOPICS['status'],String,status,queue_size=10)
            subscribe(TOPICS['native_health'],String,lambda m:json_evidence('native_health',m),queue_size=30,tcp_nodelay=True)
            subscribe(TOPICS['tracking'],Bool,lambda m:self.add('tracking',{'value':bool(m.data)}),queue_size=10)
            subscribe(TOPICS['restart'],Bool,lambda m:self.add('restart',{'value':bool(m.data)}),queue_size=10)
            subscribe(TOPICS['lidar'],LaserScan,lidar,queue_size=20)
            self.recording_session.start()
        except BaseException:
            self.recording_session.close()
            raise
        self.subscribers=self.recording_session.subscriptions

    def wait_for_timestamp_sources(self, timeout_wall_s=2.):
        started=time.monotonic(); deadline=started+timeout_wall_s
        while True:
            now=time.monotonic()
            if now>=deadline:
                raise TimeoutError('Actual /clock and producer timing startup receipts did not become ready')
            if self.error:
                raise RuntimeError(self.error)
            with self.lock:
                clock=self.latest.get('clock'); timing=self.latest.get('timing')
            current=self.rospy.Time.now().to_sec()
            value=timing.get('value') if timing else None
            valid_timing=False
            if isinstance(value,dict):
                ns=value.get('stamp_ns'); stamp=value.get('stamp_sim_s')
                publish_clock=value.get('actual_publish_clock_sim_s')
                age=value.get('measurement_age_at_publish_sim_s')
                valid_timing=(type(ns) is int and ns>0 and finite_number(stamp) and stamp>0
                    and abs(ns/1e9-stamp)<=5e-10 and finite_number(publish_clock)
                    and publish_clock>0 and finite_number(age) and abs(publish_clock-stamp-age)<=1e-7
                    and value.get('source')=='producer_ros_clock_at_publish'
                    and value.get('source_timestamp_unchanged') is True)
            if (clock and finite_number(clock.get('stamp_sim_s')) and clock['stamp_sim_s']>0
                    and finite_number(current) and current>0 and valid_timing):
                if time.monotonic()>=deadline:
                    raise TimeoutError('Timestamp startup receipts arrived after their wall-time deadline')
                self.add('recorder_startup_ready',{'source':'actual_clock_and_producer_timing_callbacks',
                    'started_monotonic_s':started,'deadline_monotonic_s':deadline,
                    'clock_stamp_sim_s':clock['stamp_sim_s'],
                    'clock_receipt_monotonic_s':clock['receipt_monotonic_s'],
                    'timing_stamp_ns':value['stamp_ns'],
                    'timing_receipt_monotonic_s':timing['receipt_monotonic_s'],
                    'measurements_not_yet_subscribed':True,'all_startup_events_retained':True})
                return
            time.sleep(min(.01,deadline-now))

    def add(self,kind,value):
        row={'kind':kind,'receipt_sim_s':self.rospy.Time.now().to_sec(),
             'receipt_wall_s':time.time(),'receipt_monotonic_s':time.monotonic(),**value}
        if kind in ('shadow','optimized','propagated','truth') and (
                not all(finite_vector(value.get(k),size) for k,size in (('p',3),('q',4),('v',3)))):
            self.error='Nonfinite or invalid '+kind+' observation; raw numeric slots stored as null'
            row=sanitize(row)
        with self.lock:
            if len(self.events)>=200000:
                self.error='Evaluation callback record limit reached; no silent sample dropping'
                return
            self.events.append(row); self.pending.append(row); self.latest[kind]=row

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.latest)

    def flush(self,stream):
        with self.lock:
            pending,self.pending=self.pending,[]
        for row in pending:
            stream.write(json.dumps(row,allow_nan=False)+'\n')
        stream.flush()

    def clock(self):
        return self.rospy.Time.now().to_sec()

    def close(self):
        self.recording_session.close()


def require_shadow_health(latest, mode, clock, monotonic):
    status=latest.get('status')
    shadow=latest.get('shadow')
    lidar=latest.get('lidar')
    if not status or not shadow or not lidar:
        raise RuntimeError('EV status/shadow or actual LiDAR has not arrived')
    payload=status['value']
    if (payload.get('mode')!=mode or payload.get('ready') is not True or payload.get('quality')!=1
            or payload.get('reset_latched') or monotonic-status['receipt_monotonic_s']>1.):
        raise RuntimeError('EV adapter not ready in expected mode: '+str(payload.get('reasons')))
    if not -.02<=clock-shadow['stamp_sim_s']<=.10 or monotonic-shadow['receipt_monotonic_s']>1.:
        raise RuntimeError('EV shadow source/receipt time is stale')
    if (lidar['sample_count']!=360 or not -.02<=clock-lidar['stamp_sim_s']<=.5
            or monotonic-lidar['receipt_monotonic_s']>1 or lidar['negative_infinite_count']
            or lidar.get('nan_range_count',0)):
        raise RuntimeError('Real 360-ray LiDAR source/receipt time is stale or invalid')


def independent_land(client, original_id, submission_attempted, collector, stream, timeout=120.,owner_check=None):
    """Cancel/land only this submitted task; preserve its original failure."""
    evidence={'requested':False,'confirmed_ground_disarmed':False}
    if not submission_attempted:
        return evidence
    if owner_check:
        owner_check()
    state=client.state(); mission=state.get('mission') or {}
    # A timeout/malformed acknowledgment may still have started the worker. The
    # pre-submit ground check established exclusive ownership; record that case.
    child_landing=(bool(original_id) and mission.get('parent_mission_id')==original_id
        and mission.get('steps')==[{'action':'land'}])
    if original_id and mission.get('id')!=original_id and not child_landing:
        raise RuntimeError('Another mission replaced this task; do not cancel operator flight')
    if (original_id is None and (state.get('armed') or mission.get('status')=='running') and
            (mission.get('description')!='Independent EV quality short route' or mission.get('steps')!=STEPS)):
        raise RuntimeError('Submission acknowledgment is missing and active worker ownership cannot be established')
    if state.get('armed') or mission.get('status')=='running':
        if child_landing:
            evidence.update(existing_owned_landing=True,landing_mission_id=mission['id'])
        else:
            if owner_check:
                owner_check()
            evidence.update(requested=True,accepted=client.request('/land',{
                'expected_mission_id':mission.get('id')}))
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if owner_check:
                owner_check()
            state=client.state()
            collector.flush(stream)
            if state.get('armed') is False and state.get('landed_state')==1 and (state.get('mission') or {}).get('status')!='running':
                break
            time.sleep(.25)
        evidence['landing_mission']=copy.deepcopy(state.get('mission'))
    evidence.update(confirmed_ground_disarmed=state.get('armed') is False and state.get('landed_state')==1
                    and (state.get('mission') or {}).get('status')!='running',final_state=state)
    return evidence


def run(args):
    from run_reliability_suite import SystemOperations,safe_ground,process_identity,owned_px4_process
    from record_ev_fusion import stable_px4_identity
    from test_avoidance_scene import load_ground_truth,segment_rectangle_distance,trajectory_evaluation
    operations=SystemOperations()
    operations.px4_directory=args.px4_directory.resolve()
    manager=operations.initial_manager()
    if manager is None:
        raise RuntimeError('Start the owned full simulation before running this test')
    commander=operations.px4_directory/'build/px4_sitl_default/bin/px4-commander'
    process=stable_px4_identity(owned_px4_process(manager,commander))
    def owner():
        if (operations.current_manager()!=manager or process_identity(manager['run_directory'])!=manager
                or stable_px4_identity(owned_px4_process(manager,commander))!=process):
            raise RuntimeError('Manager/PX4 identity changed; do not affect another simulation')
    layout,objects=load_ground_truth(args.layout)
    initial,web=operations.states(); safe_ground(initial,web)
    if (not initial.get('ready') or not initial.get('scene',{}).get('verified')
            or initial['scene']['world_sha256']!=layout['world_sha256']
            or abs(initial['position_enu_m'][2])>.2 or not initial.get('gazebo',{}).get('available')):
        raise RuntimeError('Ground/local scene readiness does not match evaluator layout')
    if initial.get('scene',{}).get('navigation',{}).get('minimum_clearance_m',0)<1.:
        raise RuntimeError('Scene must retain >=1 metre configured clearance')
    home=initial['gazebo']['position_m'][:2]
    points=[home,[home[0]+1.,home[1]],[home[0]+1.,home[1]+1.],home]
    planned_clearance=min(segment_rectangle_distance(a,b,o['collision_aabb_world_m'])['distance_m']
        for a,b in zip(points,points[1:]) for o in objects)
    if planned_clearance<1.:
        raise RuntimeError('Independent evaluator rejects short route with <1m obstacle clearance')
    args.output.mkdir(parents=True,exist_ok=False)
    report={'schema_version':1,'source':'actual_ev_independent_evaluation','started_at_wall_s':time.time(),
        'actual_model_call':False,'flight_submission_attempted':False,'actual_worker_acknowledged':False,
        'physical_flight_observed':False,'requested_mission_completed':False,'quality_passed':False,
        'validation_passed':False,'manager':manager,'px4_process':process,'steps':STEPS if args.fly else [],
        'mode':args.expected_mode,'evaluation_truth_not_supplied_to_control':True,'layout_path':str(args.layout),
        'layout_sha256':hashlib.sha256(args.layout.read_bytes()).hexdigest(),
        'world_sha256':layout['world_sha256'],'initial_state':initial,
        'planned_horizontal_segment_clearance_m':planned_clearance,'automatic_flight_retries':0}
    collector=None; original_id=None; missions={}; states=[]; ground_window=None; stream=None
    try:
        stream=(args.output/'sensor_samples.jsonl').open('x')
        collector=RosObservations()
        deadline=time.monotonic()+args.timeout
        ready_deadline=time.monotonic()+90
        while True:
            collector.flush(stream)
            if collector.error:
                raise RuntimeError(collector.error)
            try:
                require_shadow_health(collector.snapshot(),args.expected_mode,collector.clock(),time.monotonic())
                break
            except RuntimeError:
                if time.monotonic()>ready_deadline:
                    raise
                time.sleep(.1)
        ground_start=collector.clock()
        duration=2. if args.fly else args.observe_sim
        ground_window=[ground_start,ground_start+duration]
        with (args.output/'state_samples.jsonl').open('x') as state_stream:
            next_state=0.
            while collector.clock()<ground_window[1]:
                if time.monotonic()>deadline:
                    raise TimeoutError('Simulation clock observation exceeded wall-time limit')
                collector.flush(stream)
                if collector.error:
                    raise RuntimeError(collector.error)
                if time.monotonic()>=next_state:
                    current,current_web=operations.states(); safe_ground(current,current_web)
                    row={'wall_time':time.time(),'state':current}; states.append(row)
                    state_stream.write(json.dumps(row,allow_nan=False)+'\n'); state_stream.flush()
                    next_state=time.monotonic()+.5
                time.sleep(.1)
            if args.fly:
                current,current_web=operations.states(); safe_ground(current,current_web)
                owner()
                require_shadow_health(collector.snapshot(),args.expected_mode,collector.clock(),time.monotonic())
                report['preliminary_quality']=evaluate(collector.events,ground_window,require_full_ground=False)
                if not report['preliminary_quality']['passed']:
                    failures=[check['name'] for check in report['preliminary_quality']['checks'] if not check['passed']]
                    raise RuntimeError('Independent preflight source evaluation failed: '+', '.join(failures))
                report['ground_offboard_preparation']=BridgeClient().request('/prepare-offboard',{},timeout=15)
                prepared=report['ground_offboard_preparation']
                if prepared.get('status') != 'ok' or prepared.get('armed') is not False:
                    raise RuntimeError('Ground OFFBOARD preparation was not explicitly disarmed and successful')
                preflight=operations.preflight_check(manager,2.)
                report['preflight']=preflight
                if not preflight['passed']:
                    raise RuntimeError('PX4 commander check did not explicitly report Preflight check: OK')
                report['flight_submission_attempted']=True
                accepted=BridgeClient().request('/mission',{'steps':STEPS,'description':'Independent EV quality short route'})
                report['accepted_response']=accepted
                original_id=accepted['mission']['id']; report['actual_worker_acknowledged']=True
                report['original_mission_id']=original_id
                print('Started actual short EV evaluation mission '+original_id,flush=True)
                while time.monotonic()<deadline:
                    owner()
                    collector.flush(stream)
                    if collector.error:
                        raise RuntimeError(collector.error)
                    state=BridgeClient().state(); mission=state.get('mission') or {}
                    row={'wall_time':time.time(),'state':state}; states.append(row)
                    state_stream.write(json.dumps(row,allow_nan=False)+'\n'); state_stream.flush()
                    if mission.get('id')!=original_id:
                        raise RuntimeError('Original mission was replaced; retained evidence and stop evaluation')
                    missions[original_id]=copy.deepcopy(mission)
                    if state.get('armed'):
                        report['physical_flight_observed']=True
                    if mission.get('status')!='running':
                        if mission.get('status')!='succeeded':
                            raise RuntimeError('Original flight mission failed: '+str(mission.get('error')))
                        if state.get('armed') is not False or state.get('landed_state')!=1:
                            raise RuntimeError('Mission returned success without landing/disarming evidence')
                        if (mission.get('step')!=len(STEPS) or
                                [event.get('action') for event in mission.get('events',[])]!=[step['action'] for step in STEPS]):
                            raise RuntimeError('Mission success lacks all five completed step events')
                        report['requested_mission_completed']=True
                        break
                    time.sleep(.25)
                else:
                    raise TimeoutError('Actual EV flight exceeded wall-time limit')
                report['initial_ground_window_sim_s']=ground_window
                ground_start=collector.clock()
                ground_window=[ground_start,ground_start+args.ground_duration_sim]
                report['ground_assessment_phase']='after_completed_landing_and_disarming'
                while collector.clock()<ground_window[1]:
                    owner()
                    if time.monotonic()>deadline:
                        raise TimeoutError('Post-landing stationary observation exceeded wall-time limit')
                    collector.flush(stream)
                    if collector.error:
                        raise RuntimeError(collector.error)
                    state,current_web=operations.states(); safe_ground(state,current_web)
                    if (state.get('mission') or {}).get('id')!=original_id:
                        raise RuntimeError('Mission changed during post-landing observation')
                    row={'wall_time':time.time(),'state':state}; states.append(row)
                    state_stream.write(json.dumps(row,allow_nan=False)+'\n'); state_stream.flush()
                    time.sleep(.25)
            # A final truth bracket permits matching the last real VIO samples.
            time.sleep(.2); collector.flush(stream)
            report['final_state']=BridgeClient().state()
    except (Exception,KeyboardInterrupt) as exc:
        report.update(error=str(exc),error_type=type(exc).__name__)
        if collector is not None and stream is not None:
            try:
                report['independent_landing']=independent_land(BridgeClient(),original_id,
                    report['flight_submission_attempted'],collector,stream,owner_check=owner)
            except Exception as recovery_error:
                report['independent_landing_error']=str(recovery_error)
    finally:
        if collector is not None:
            collector.close(); collector.flush(stream)
            report['recorder_gc_policy']=collector.recording_session.evidence()
            if collector.recording_session.cleanup_errors:
                report.setdefault('error','Recorder subscription cleanup failed')
            try:
                report['quality']=evaluate(collector.events,ground_window,
                    require_full_ground=(args.ground_duration_sim>=30 if args.fly else args.observe_sim>=30))
                report['quality_passed']=report['quality']['passed']
            except Exception as evaluation_error:
                report['quality_evaluation_error']=str(evaluation_error)
            report['sensor_event_count']=len(collector.events)
        if stream is not None:
            stream.close()
        report.update(ground_window_sim_s=ground_window,missions=list(missions.values()),
                      finished_at_wall_s=time.time(),state_sample_count=len(states))
        report['duration_wall_s']=report['finished_at_wall_s']-report['started_at_wall_s']
        if len(states)>1:
            config=initial['scene']['navigation']
            try:
                report['trajectory']=trajectory_evaluation(states,objects,home,points[2],config)
            except Exception as geometry_error:
                report['trajectory_evaluation_error']=str(geometry_error)
        clearance=report.get('trajectory',{}).get('minimum_interpolated_segment_clearance_m')
        report['recorded_clearance_passed']=clearance is not None and clearance>=1.
        report['validation_passed']=not report.get('error') and report['quality_passed'] and (
            report['requested_mission_completed'] if args.fly else True) and report['recorded_clearance_passed']
        (args.output/'task_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--fly',action='store_true')
    mode.add_argument('--observe-only',action='store_true')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--ground-duration-sim',type=float,default=30.)
    parser.add_argument('--observe-sim',type=float,default=30.)
    parser.add_argument('--timeout',type=float,default=600.)
    parser.add_argument('--expected-mode',choices=('shadow','fuse'),default='shadow')
    parser.add_argument('--layout',type=Path,default=ROOT/'scene/obstacle_layout.json')
    parser.add_argument('--px4-directory',type=Path,default=Path(os.environ.get('PX4_DIR','/home/dut/PX4-Autopilot')))
    args=parser.parse_args(argv)
    if (not all(math.isfinite(v) and v>0 for v in (args.ground_duration_sim,args.observe_sim,args.timeout))
            or args.ground_duration_sim>300 or args.observe_sim>300 or not 1<=args.timeout<=1200):
        parser.error('ground/observe simulation seconds in (0,300], wall timeout 1..1200')
    if args.fly and args.ground_duration_sim<30:
        parser.error('Flight quality assessment requires at least 30 simulation seconds of stationary ground recording')
    if not args.fly and not args.observe_only:
        print(json.dumps({'default_dry_run':True,'actual_requests':False,'flight_commands_sent':False,
            'topics':TOPICS,'flight_steps_if_requested':STEPS,'ground_duration_sim_s':args.ground_duration_sim,
            'independent_evaluation_limits':LIMITS,'additional_independent_criterion':GRAVITY_CRITERION,
            'truth_feedback_to_adapter_or_control':False},ensure_ascii=False,indent=2))
        return 0
    if args.output is None:
        parser.error('Actual observation/flight requires --output NEW_DIRECTORY')
    result=run(args)
    print(json.dumps({'report':str(args.output/'task_validation.json'),'validation_passed':result['validation_passed'],
        'requested_mission_completed':result['requested_mission_completed'],'quality_passed':result['quality_passed'],
        'error':result.get('error')},ensure_ascii=False))
    return 0 if result['validation_passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
