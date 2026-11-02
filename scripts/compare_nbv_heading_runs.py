#!/usr/bin/env python3
"""Compare one audited seeded fixed/discrete pair; no model or flight calls."""
import argparse
import json
import math
from pathlib import Path


def load_run(directory):
    record = json.loads((directory/'result.json').read_text())
    audit = json.loads((directory/'nbv-audit'/'nbv-evidence-audit.json').read_text())
    if audit.get('passed') is not True:
        raise ValueError('comparison requires a passing independent NBV audit')
    captures = record['command']['perceptions']
    if not captures:
        raise ValueError('comparison requires actual observations')
    maps = [json.loads((directory/'captures'/(item['capture_id']+'.semantic-map.json')).read_text()) for item in captures]
    return record, audit, maps


def compare(fixed, discrete):
    f, fa, fm = fixed; d, da, dm = discrete
    keys = ('search_mode', 'target_confirmation_mode', 'nbv_selector', 'candidate_pool_mode', 'nbv_auto_prepare', 'world_file')
    if any(f.get(k) != d.get(k) for k in keys) or f['command']['prompt'] != d['command']['prompt']:
        raise ValueError('task, selector or environment configuration differs')
    if f.get('heading_mode') != 'fixed' or d.get('heading_mode') != 'discrete':
        raise ValueError('expected one fixed and one discrete heading run')
    if not any(move.get('heading_step') in (-1, 1) for move in da.get('moves', [])):
        raise ValueError('discrete run did not actually execute a heading candidate')
    fs, ds = f.get('gazebo_seed_evidence') or {}, d.get('gazebo_seed_evidence') or {}
    if (fs.get('source') != 'actual_gzserver_proc_cmdline' or ds.get('source') != fs.get('source')
            or fs.get('seed') != ds.get('seed') or fs.get('lockstep') is not True or ds.get('lockstep') is not True
            or fs.get('world_file') != ds.get('world_file') or fs.get('pid') == ds.get('pid')):
        raise ValueError('missing matched seed/world/lockstep evidence from independent simulator processes')
    if fm[0]['world_sha256'] != dm[0]['world_sha256']:
        raise ValueError('world hashes differ')
    cfg1 = dict(f['command']['nbv_events'][0]['config']); cfg2 = dict(d['command']['nbv_events'][0]['config'])
    cfg1.pop('heading_mode', None); cfg2.pop('heading_mode', None)
    if cfg1 != cfg2:
        raise ValueError('candidate path budgets or score settings differ')
    models1 = sorted({c.get('model') for c in f['command']['model_calls']})
    models2 = sorted({c.get('model') for c in d['command']['model_calls']})
    if models1 != models2:
        raise ValueError('model names differ')
    first1, first2 = fm[0]['observations'][0], dm[0]['observations'][0]
    def world_position(observation, snapshot):
        x, y, z = observation['position_enu_m']
        registration = snapshot['registration']; ox, oy, oz = registration['world_from_enu_origin_m']
        angle = registration['world_from_enu_yaw_rad']; c, s = math.cos(angle), math.sin(angle)
        return [ox+c*x-s*y, oy+s*x+c*y, oz+z]
    distance = math.dist(world_position(first1, fm[0]), world_position(first2, dm[0]))
    if distance > .5:
        raise ValueError('first observation positions differ by more than 0.5 m')
    def yaw(observation, snapshot):
        x, y, z, w = observation['orientation_xyzw']
        return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)) + snapshot['registration']['world_from_enu_yaw_rad']
    delta = yaw(first1, fm[0])-yaw(first2, dm[0])
    heading_difference = abs(math.atan2(math.sin(delta), math.cos(delta)))
    if heading_difference > .2:
        raise ValueError('first observation headings differ by more than 0.2 rad')
    def metrics(record, audited, maps):
        moves = audited['moves']; final = maps[-1]
        eligible = final['eligible_cell_ids']
        return {'status': record['command']['status'], 'elapsed_wall_s': record['elapsed_seconds'],
                'model_calls': len(record['command']['model_calls']), 'reports': len(record['command']['perceptions']),
                'ticket_moves': len(moves), 'rotating_moves': sum(m.get('heading_step') in (-1, 1) for m in moves),
                'planned_outbound_path_m': sum(m['actual_planned_length_m'] for m in moves),
                'geometric_coverage_fraction': sum(final['cells'][i]['coverage'] for i in eligible)/len(eligible),
                'model_target_found': any((e.get('result') or {}).get('search_status') == 'target_found'
                                          for e in record['command'].get('search_events', [])),
                'independent_target_class_verification': False}
    return {'seed': fs['seed'], 'world_sha256': fm[0]['world_sha256'], 'models': models1,
            'first_observer_distance_m': distance, 'first_heading_difference_rad': heading_difference,
            'fixed': metrics(f, fa, fm), 'discrete': metrics(d, da, dm),
            'scope': 'one seeded pair; VLM calls and timing remain nondeterministic; planned paths are not measured trajectories',
            'statistics_or_causal_advantage_proven': False}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fixed', type=Path, required=True); p.add_argument('--discrete', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); result = compare(load_run(args.fixed), load_run(args.discrete))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'comparison.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
