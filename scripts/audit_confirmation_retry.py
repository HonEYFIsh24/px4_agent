#!/usr/bin/env python3
"""Audit whether a real archived task exercised retained-source re-observation."""
import argparse
import json
import math
from pathlib import Path


def audit_record(record, nbv_audit, confirmation_audit, load_map):
    command = record['command']; events = command.get('target_confirmation_events') or []
    checks = {'nbv_audit_passed': nbv_audit.get('passed') is True,
              'confirmation_audit_passed': confirmation_audit.get('passed') is True}
    witnesses = []
    for index, event in enumerate(events):
        if (event.get('outcome') != 'unconfirmed' or event.get('current_supported') is not False
                or not (event.get('source_supported') is True or event.get('source_uncertain') is True)):
            continue
        first = event.get('candidate') or {}; source = first.get('source_capture_id'); current = first.get('current_capture_id')
        later = events[index+1:]
        retained = next((x for x in later if x.get('outcome') == 'candidate'
                         and (x.get('candidate') or {}).get('source_capture_id') == source
                         and (x.get('candidate') or {}).get('review_attempts') == 1), None)
        second = next((x for x in later if x.get('outcome') in ('unconfirmed', 'rejected', 'confirmed')
                       and (x.get('candidate') or {}).get('source_capture_id') == source
                       and (x.get('candidate') or {}).get('review_attempts') == 1), None)
        if retained is None or second is None or later.index(retained) >= later.index(second):
            continue
        follow = (second.get('candidate') or {}).get('current_capture_id')
        if not all(isinstance(x, str) and x and Path(x).name == x for x in (source, current, follow)) or len({source, current, follow}) != 3:
            continue
        matching = [m for m in nbv_audit.get('moves', []) if m.get('source_capture_id') == current and m.get('next_capture_id') == follow]
        predictions = [p for p in command.get('nbv_events', []) if p.get('source_capture_id') == current
                       and (p.get('confirmation_goal') or {}).get('source_capture_id') == source
                       and p.get('planning_mode') == 'confirmation']
        if len(matching) != 1 or len(predictions) != 1 or matching[0].get('batch_id') != predictions[0].get('batch_id'):
            continue
        snapshot = load_map(follow); views = {x['capture_id']: x for x in snapshot['observations']}
        try:
            a, b, c = (views[x] for x in (source, current, follow))
            distances = [math.dist(c['position_enu_m'], x['position_enu_m']) for x in (a, b)]
            timestamps = [x['stamp_sim_s'] for x in (a, b, c)]
        except (KeyError, TypeError, ValueError):
            continue
        if not (timestamps[0] < timestamps[1] < timestamps[2] and all(x >= .75 for x in distances)):
            continue
        witnesses.append({'source_capture_id': source, 'first_recheck_capture_id': current,
                          'second_recheck_capture_id': follow, 'distances_m': distances,
                          'review_attempts': 1, 'first_source_uncertain': event.get('source_uncertain', False),
                          'final_review_outcome': second['outcome']})
    checks['retained_source_reobserved'] = bool(witnesses)
    return {'passed': all(checks.values()), 'checks': checks, 'witnesses': witnesses,
            'limitation': 'Proves archived control/evidence lineage for this task, not object identity, category accuracy or continuous trajectory.'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); record = json.loads(args.result.read_text()); directory = args.result.parent
    nbv = json.loads((directory/'nbv-audit'/'nbv-evidence-audit.json').read_text())
    confirmation = json.loads((directory/'confirmation-audit'/'confirmation-audit.json').read_text())
    captures = Path(record['capture_directory'])
    result = audit_record(record, nbv, confirmation,
                          lambda identifier: json.loads((captures/(identifier+'.semantic-map.json')).read_text()))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'confirmation-retry-audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 2


if __name__ == '__main__': raise SystemExit(main())
