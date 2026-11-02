#!/usr/bin/env python3
"""Validate a server-issued heading candidate in local SITL; no VLM calls.

This exercises executor authorization with a deliberately selected in-place
candidate. It is not an autonomous planner benchmark or recognition test.
"""
import argparse
import base64
import hashlib
import json
import math
import sys
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from client import BridgeClient
from yaw_control import error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        print('Dry run; no flight. --execute requires explicitly enabled local SITL.')
        return 0
    client = BridgeClient(); state = client.state(); ev = state.get('external_vision') or {}
    if not (state.get('simulation_only') is True and state.get('ready') is True
            and state.get('armed') is False and state.get('landed_state') == 1
            and state.get('navigation', {}).get('method') == 'static_3d'
            and state.get('navigation', {}).get('viewpoint_yaw_supported') is True
            and ev.get('healthy') is True and ev.get('used_for_flight_control') is True):
        raise RuntimeError('Requires grounded local SITL, fused VIO and explicitly enabled yaw NBV')
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'run': (ROOT/'logs/system/latest').resolve().name,
              'owned_mission_ids': [], 'checks': {}, 'images': {},
              'scope': 'one manually selected executor-issued yaw candidate; no model or planner benchmark'}
    def save():
        tmp = args.output/'result.tmp'
        tmp.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        tmp.replace(args.output/'result.json')
    def submit(steps):
        accepted = client.request('/mission', {'steps': steps, 'description': 'owned yaw NBV validation'})
        report['owned_mission_ids'].append(accepted['mission']['id']); save()
        return accepted
    def photo(name, after=None):
        deadline = time.monotonic()+12
        while True:
            try:
                pair = client.capture_stereo()
                if after is not None and pair['stamp_sim_s'] <= after:
                    raise RuntimeError('capture predates rotation arrival')
                break
            except RuntimeError:
                if time.monotonic() >= deadline: raise
                time.sleep(.2)
        image = base64.b64decode(pair['left']['image_base64'], validate=True)
        digest = hashlib.sha256(image).hexdigest()
        if digest != pair['image_sha256']['left']: raise RuntimeError('image SHA mismatch')
        (args.output/(name+'.jpg')).write_bytes(image)
        report['images'][name] = {'sha256': digest, 'stamp_sim_s': pair['stamp_sim_s'],
                                  'body_pose_enu': pair['body_pose_enu'], 'coordinate_epoch': pair['coordinate_epoch']}
        save()
        return pair
    save()
    try:
        takeoff = client.wait_for(submit([{'action': 'takeoff', 'altitude': 3.}]))
        if takeoff['mission']['status'] != 'succeeded': raise RuntimeError('takeoff failed')
        pair = photo('before')
        position = client.state()['position_enu_m']; origin = position[:2]
        geometry = client.request('/navigation/observation-geometry', {'pair_id': pair['pair_id'], 'origin_xy_enu_m': origin}, timeout=30)
        identity = {k: geometry[k] for k in ('world_sha256', 'sortie_id', 'coordinate_epoch', 'registration')}
        # Revision is a test batch identifier derived from actual image hashes,
        # not a claim that a report or a production semantic map exists here.
        revision = hashlib.sha256(json.dumps(pair['image_sha256'], sort_keys=True).encode()).hexdigest()
        context = {'owner': ['yaw-nbv-validation', uuid.uuid4().hex],
                   'map_revision': revision, 'batch_id': uuid.uuid4().hex}
        query = {'pair_id': pair['pair_id'], 'origin_xy_enu_m': origin, 'expected_identity': identity,
                 'search_context': context}
        client.request('/navigation/viewpoints', {**query, 'operation': 'context', 'candidates': []}, timeout=30)
        point = client.state()['position_enu_m']
        candidate = {'id': 'view_001', 'position_enu_m': point, 'heading_step': 1}
        prediction = client.request('/navigation/viewpoints', {**query, 'operation': 'paths', 'candidates': [candidate]}, timeout=30)
        result = prediction['results'][0]
        if result.get('reachable') is not True or not result.get('execution_ticket'):
            raise RuntimeError('candidate did not receive a verified path ticket')
        report['candidate'] = {k: v for k, v in result.items() if k != 'execution_ticket'}
        report['source_yaw_enu_rad'] = prediction['fixed_yaw_enu_rad']; report['identity'] = identity
        report['checks']['heading_server_derived'] = abs(error(result['target_yaw_enu_rad'],
            prediction['fixed_yaw_enu_rad']+math.pi/4)) < 1e-9
        payload = {'owner': context['owner'], 'batch_id': context['batch_id'], 'map_revision': revision,
                   'candidate_id': candidate['id'], 'ticket': result['execution_ticket'], 'description': 'owned yaw NBV validation'}
        accepted = client.request('/navigation/execute-viewpoint', payload)
        report['owned_mission_ids'].append(accepted['mission']['id']); report['accepted_mission'] = accepted['mission']; save()
        try:
            client.capture_stereo(); report['checks']['capture_blocked_while_executing'] = False
        except RuntimeError as exc:
            report['checks']['capture_blocked_while_executing'] = 'rotation' in str(exc)
        flight = client.wait_for(accepted); report['viewpoint_terminal'] = flight['mission']
        if flight['mission']['status'] != 'succeeded': raise RuntimeError('candidate mission failed: '+str(flight['mission'].get('error')))
        events = [e for e in flight['mission']['events'] if e.get('action') == 'rotate']
        if len(events) != 1: raise RuntimeError('missing actual rotation arrival event')
        rotation = events[0]['rotation']; report['rotation'] = rotation
        report['checks']['actual_arrival'] = abs(error(result['target_yaw_enu_rad'], rotation['actual_rad'])) <= .1
        after = photo('after', rotation['stamp_s'])
        report['checks']['post_arrival_capture'] = after['stamp_sim_s'] > rotation['stamp_s']
        report['checks']['same_epoch'] = pair['coordinate_epoch'] == after['coordinate_epoch']
        try:
            client.request('/navigation/execute-viewpoint', payload); report['checks']['ticket_reuse_rejected'] = False
        except RuntimeError:
            report['checks']['ticket_reuse_rejected'] = True
        landed = client.wait_for(submit([{'action': 'return_home'}, {'action': 'land'}]))
        report['landing_terminal'] = landed['mission']
        report['checks']['landed_disarmed'] = (landed['mission']['status'] == 'succeeded'
                                               and landed.get('armed') is False and landed.get('landed_state') == 1)
        report['status'] = 'passed' if all(report['checks'].values()) else 'failed'
    except Exception as exc:
        report.update(status='failed', error=str(exc))
        try:
            current = client.state(); identifier = (current.get('mission') or {}).get('id')
            if current.get('armed') is True and identifier in report['owned_mission_ids']:
                accepted = client.request('/land', {'expected_mission_id': identifier})
                landed = client.wait_for(accepted)
                report['recovery'] = {'status': landed['mission']['status'], 'armed': landed.get('armed'),
                                      'landed_state': landed.get('landed_state')}
        except Exception as recovery_error:
            report['recovery_error'] = str(recovery_error)
    save(); print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
