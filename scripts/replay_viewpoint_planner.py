#!/usr/bin/env python3
"""Replay real map evidence and evaluate NBV offline; no model/network/flight calls."""
import argparse
import contextlib
import copy
import hashlib
import io
import json
import platform
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
from replay_semantic_map import replay
from scene_config import load_scene
from static_navigation import StaticNavigator
from stereo_localization import vector, finite
from viewpoint_geometry import StaticViewpointEvaluator, registered_identity
from viewpoint_planner import ViewpointPlanner, ViewpointConfig, map_identity


def run(result_file, world, output, modes=('geometry', 'semantic'), candidate_pool_mode='ranked', heading_mode='fixed'):
    output.mkdir(parents=True, exist_ok=False)
    with contextlib.redirect_stdout(io.StringIO()):
        semantic_map = replay(result_file, output / 'source-map')
    record = json.loads(result_file.read_text())
    directory = Path(record.get('capture_directory') or result_file.parent / 'captures')
    last = semantic_map.observations[-1]['capture_id']
    archive = json.loads((directory / (last + '.json')).read_text())
    snapshot = archive['snapshot']
    scene = load_scene(world, navigation_mode='static_3d')
    navigator = StaticNavigator.from_scene(world, scene)
    if navigator.world_sha256 != semantic_map.world_sha256:
        raise ValueError('offline NBV world does not match the actual archived capture')
    # Restore the recorded transform for offline analysis, never a guessed transform
    # or an authority to register a live sortie. No Gazebo target labels are read.
    registration = semantic_map.registration
    navigator.origin = vector(registration['world_from_enu_origin_m'], 3, 'archived registration origin')
    navigator.yaw = finite(registration['world_from_enu_yaw_rad'], 'archived registration yaw')
    navigator.registration_id = semantic_map.sortie_id
    if registered_identity(navigator, semantic_map.coordinate_epoch) != map_identity(semantic_map):
        raise ValueError('offline geometric registration does not match the saved evidence')
    evaluator = StaticViewpointEvaluator(navigator, snapshot['body_pose_enu'], snapshot['camera_intrinsics'],
                                        semantic_map.coordinate_epoch, allow_yaw=heading_mode == 'discrete',
                                        search_radius_m=semantic_map.grid['radius_m'],
                                        grid_resolution_m=semantic_map.grid['resolution_m'])
    before, last_plan = copy.deepcopy(semantic_map.snapshot()), copy.deepcopy(navigator.last_plan)
    report = {'source_result': str(result_file.resolve()), 'source_capture_id': last,
              'world_sha256': navigator.world_sha256, 'python_version': platform.python_version(),
              'code_sha256': {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in
                  ('viewpoint_planner.py', 'viewpoint_geometry.py', 'static_navigation.py',
                   'bridge.py', 'scripts/replay_viewpoint_planner.py')},
              'scope': 'offline deterministic NBV on actual archived observations; all viewpoints are predictions',
              'actual_flight': False, 'model_calls': 0, 'candidate_pool_mode': candidate_pool_mode,
              'heading_mode': heading_mode, 'modes': {}}
    for mode in modes:
        planner = ViewpointPlanner(ViewpointConfig(mode=mode, candidate_pool_mode=candidate_pool_mode,
                                                  heading_mode=heading_mode))
        started = time.monotonic()
        result = planner.evaluate(semantic_map, evaluator)
        elapsed = time.monotonic()-started
        selected = planner.select(semantic_map)
        if semantic_map.snapshot() != before or navigator.last_plan != last_plan:
            raise AssertionError('candidate evaluation changed observation history or executing last_plan')
        report['modes'][mode] = {'elapsed_wall_s': elapsed, 'status': result['status'],
            'generated_count': result['generated_count'], 'path_evaluation_count': result['path_evaluation_count'],
            'selected_id': result['selected_id'], 'selected_position_enu_m': selected['position_enu_m'] if selected else None,
            'selected_heading_step': selected.get('heading_step') if selected else None,
            'selected_score_terms': selected['score_terms'] if selected else None,
            'candidate_pool_ids': result['candidate_pool_ids'],
            'candidate_pool_groups': result['candidate_pool_groups'],
            'diversity_path_check_ids': result['diversity_path_check_ids'],
            'unvalidated_count': len(result['unvalidated_candidate_ids']), 'history_unchanged': True}
        mode_dir = output / mode
        mode_dir.mkdir()
        (mode_dir / 'candidates.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        (mode_dir / 'selected.json').write_text(json.dumps(selected, ensure_ascii=False, indent=2, allow_nan=False))
        display = semantic_map.snapshot()
        display.update(evidence_mode='actual archived map + OFFLINE NBV PREDICTIONS (not flown)',
                       viewpoint_prediction=result)
        (mode_dir / 'map-with-predictions.json').write_text(json.dumps(display, ensure_ascii=False, indent=2, allow_nan=False))
    (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('geometry', 'semantic', 'both'), default='both')
    parser.add_argument('--candidate-pool', choices=('ranked', 'diverse'), default='ranked')
    parser.add_argument('--heading', choices=('fixed', 'discrete'), default='fixed')
    args = parser.parse_args()
    run(args.result, args.world, args.output,
        ('geometry', 'semantic') if args.mode == 'both' else (args.mode,), args.candidate_pool, args.heading)
