#!/usr/bin/env python3
"""Replay exact archived NBV query responses, without ROS, network or flight tools."""
import argparse
import copy
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from semantic_map import SemanticMap
from viewpoint_planner import ViewpointConfig, ViewpointPlanner
from search_diagnostics import exception_detail, sanitized


class FrozenMap(SemanticMap):
    def __init__(self, snapshot):
        self._snapshot = copy.deepcopy(snapshot)
        for name in ('status', 'query', 'world_sha256', 'sortie_id', 'coordinate_epoch',
                     'registration', 'grid', 'cells', 'observations'):
            setattr(self, name, copy.deepcopy(snapshot[name]))
        self.eligible = set(snapshot['eligible_cell_ids'])

    def snapshot(self):
        return copy.deepcopy(self._snapshot)


class RecordedQueries:
    def __init__(self, events, goal):
        self.calls = [e for e in events if e['stage'] == 'remote_viewpoints' and e['event'] == 'begin']
        self.ends = {e['id']: e for e in events if e['stage'] == 'remote_viewpoints' and e['event'] in ('end','error')}
        self.index, self.goal = 0, goal

    def query(self, operation, origin, identity, candidates):
        if self.index == len(self.calls):
            raise ValueError('Replay exhausted: new query has no archived response')
        call = self.calls[self.index]; expected = call['inputs']
        actual = {'operation': operation, 'origin_xy_enu_m': origin,
                  'expected_identity': identity, 'candidates': candidates}
        if self.goal is not None:
            actual['probe_points_enu_m'] = self.goal['probe_points_enu_m']
        if (any(expected.get(k) != v for k, v in actual.items())
                or expected.get('probe_points_enu_m') != actual.get('probe_points_enu_m')):
            raise ValueError('Replay query differs from actual captured inputs at query '+str(self.index))
        self.index += 1
        end = self.ends.get(call['id'])
        if end is None:
            raise ValueError('Replay has no recorded result for query '+str(self.index-1))
        if end['event'] == 'error':
            raise RuntimeError('Recorded remote failure: '+end['message'])
        return copy.deepcopy(end['result'])


def replay(path, index=-1):
    events = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    starts = [i for i, e in enumerate(events) if e['stage'] == 'planner_evaluate' and e['event'] == 'begin']
    start = starts[index]; initial = events[start]
    finish = next((i for i in range(start+1, len(events)) if events[i].get('id') == initial['id']
                   and events[i]['event'] in ('end','error')), None)
    if finish is None:
        raise ValueError('Incomplete planner trace; cannot claim replay completion')
    data = initial['inputs']; m = FrozenMap(data['semantic_map'])
    queries = RecordedQueries(events[start+1:finish], data['confirmation_goal'])
    planner = ViewpointPlanner(ViewpointConfig(**data['config']))
    result = {'actual_flight': False, 'network_calls': 0, 'source': str(path),
              'batch_id': data['context']['batch_id'], 'original_outcome': events[finish]['event']}
    try:
        prediction = planner.evaluate(m, queries, data['confirmation_goal'])
        result.update(replay_outcome='completed', selected_id=prediction['selected_id'],
                      status=prediction['status'], all_recorded_queries_used=queries.index == len(queries.calls),
                      matches_recorded_prediction=sanitized(prediction) == events[finish].get('result'))
    except Exception as exc:
        result.update(replay_outcome='error', **exception_detail(exc))
    result.update(queries_consumed=queries.index, recorded_query_count=len(queries.calls))
    return result


def replay_executor(path, world, index=-1):
    from scene_config import load_scene
    from static_navigation import StaticNavigator
    from viewpoint_geometry import StaticViewpointEvaluator
    events = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    starts = [e for e in events if e['stage'] == 'executor_geometry' and e['event'] == 'begin']
    initial = starts[index]; data = initial['inputs']
    end = next((e for e in events if e.get('id') == initial['id'] and e['event'] in ('end','error')), None)
    if end is None:
        raise ValueError('Incomplete executor query; preserve the live log and wait')
    nav = StaticNavigator.from_scene(world, load_scene(world, navigation_mode='static_3d'))
    identity = data['identity']; registration = identity['registration']
    if nav.world_sha256 != identity['world_sha256'] or nav.ground_world_z != registration['ground_world_z_m']:
        raise ValueError('Replay world does not match captured registration')
    nav.origin = registration['world_from_enu_origin_m']; nav.yaw = registration['world_from_enu_yaw_rad']
    nav.registration_id = identity['sortie_id']  # isolated offline geometry; never an execution authority
    backend = StaticViewpointEvaluator(nav, data['pose'], data['camera_info'], identity['coordinate_epoch'],
        fixed_yaw=data['fixed_yaw'], reference_kind='live_enu_pose', probe_points_enu_m=data['probes'], allow_yaw=True)
    result = {'actual_flight':False, 'network_calls':0, 'source':str(path), 'operation':data['operation'],
              'original_outcome':end['event'], 'query_id':initial['id']}
    try:
        prediction = backend.query(data['operation'],data['origin_xy'],identity,data['candidates'])
        result.update(replay_outcome='completed', all_recorded_queries_used=True,
                      matches_recorded_prediction=sanitized(prediction) == end.get('result'))
    except Exception as exc:
        result.update(replay_outcome='error', **exception_detail(exc))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--index', type=int, default=-1)
    parser.add_argument('--world', type=Path, help='Replay an executor_geometry query against its verified world')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output file; preserve earlier evidence')
    result = replay_executor(args.log, args.world, args.index) if args.world else replay(args.log, args.index)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if (result.get('replay_outcome') == 'completed' and result.get('all_recorded_queries_used')
                 and result.get('matches_recorded_prediction')) else 2


if __name__ == '__main__':
    raise SystemExit(main())
