#!/usr/bin/env python3
"""Explicit opt-in end-to-end test using configured VLM and Gazebo camera/flight."""
import argparse
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def gazebo_seed_evidence(seed, world, proc=Path('/proc')):
    """Verify the running Gazebo server argv, not just a requested launch option."""
    servers = []
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = (entry/'cmdline').read_bytes().split(b'\0')
            argv = [x.decode('utf-8') for x in argv if x]
        except (OSError, UnicodeError):
            continue
        if argv and Path(argv[0]).name == 'gzserver':
            servers.append((int(entry.name), argv))
    if len(servers) != 1:
        raise RuntimeError('Expected exactly one inspectable local gzserver for seeded comparison')
    pid, argv = servers[0]
    seed_args = [argv[i+1] for i, value in enumerate(argv[:-1]) if value == '--seed']
    if seed_args != [str(seed)] or str(world.resolve()) not in argv or '--lockstep' not in argv:
        raise RuntimeError('Gazebo actual process arguments do not match the requested seed, world and lockstep')
    return {'pid': pid, 'seed': seed, 'world_file': str(world.resolve()), 'lockstep': True,
            'source': 'actual_gzserver_proc_cmdline'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Actually send simulator images to the configured model and fly SITL')
    parser.add_argument('--prompt', default='找到前方的油桶，拍照分析并报告周围的情况。')
    parser.add_argument('--output', type=Path, default=ROOT/'logs/setup/autonomous-target-search')
    parser.add_argument('--memory-history-turns', type=int, default=0, choices=range(0, 21),
                        help='Seed explicit synthetic historical turns before the real current flight test')
    parser.add_argument('--memory-followup', action='store_true', help='After landing, ask about the previous task with camera/flight transport disabled')
    parser.add_argument('--require-nbv', action='store_true',
                        help='Require archived ticket execution, full planned-route checks and real observation/map replay')
    parser.add_argument('--world', type=Path, help='Registered world file for independent NBV collision audit')
    parser.add_argument('--gazebo-seed', type=int, help='Require this actual gzserver seed; verify its process arguments')
    parser.add_argument('--require-semantic-guidance', action='store_true',
                        help='Require an actual selected move with a nonzero semantic score before capture')
    parser.add_argument('--require-vlm-selection', action='store_true',
                        help='Require actual VLM candidate-ID selection with owned source-image input')
    parser.add_argument('--require-diverse-pool', action='store_true',
                        help='Require budgeted diverse pools with rising and multiple groups offered in actual execution')
    parser.add_argument('--require-auto-prepare', action='store_true',
                        help='Require report-side preparation without an extra next-view model tool call')
    parser.add_argument('--require-heading-execution', action='store_true',
                        help='Require an audited executed discrete-heading candidate')
    parser.add_argument('--require-target-confirmation', action='store_true',
                        help='Require an actual owned two-view review with archived ROI provenance')
    parser.add_argument('--require-confirmed-target', action='store_true', help='Require a verified confirmed outcome, not only a review')
    parser.add_argument('--require-rejected-candidate', action='store_true', help='Require an actual rejected visual hypothesis')
    args = parser.parse_args()
    if args.gazebo_seed is not None and (args.world is None or not 0 <= args.gazebo_seed <= 2147483647):
        parser.error('--gazebo-seed requires --world and an integer from 0 to 2147483647')
    if args.require_nbv and (args.world is None or os.getenv('UAV_SEARCH_MODE', 'fixed') not in ('nbv_geometry', 'nbv_semantic')):
        parser.error('--require-nbv needs --world and UAV_SEARCH_MODE=nbv_geometry or nbv_semantic')
    if args.require_semantic_guidance and (not args.require_nbv or os.getenv('UAV_SEARCH_MODE') != 'nbv_semantic'):
        parser.error('--require-semantic-guidance needs --require-nbv and UAV_SEARCH_MODE=nbv_semantic')
    if args.require_vlm_selection and (not args.require_nbv or os.getenv('UAV_NBV_SELECTOR') != 'vlm'):
        parser.error('--require-vlm-selection needs --require-nbv and UAV_NBV_SELECTOR=vlm')
    if args.require_diverse_pool and (not args.require_nbv or os.getenv('UAV_NBV_CANDIDATE_POOL') != 'diverse'):
        parser.error('--require-diverse-pool needs --require-nbv and UAV_NBV_CANDIDATE_POOL=diverse')
    if args.require_auto_prepare and (not args.require_nbv or os.getenv('UAV_NBV_AUTO_PREPARE') != '1'):
        parser.error('--require-auto-prepare needs --require-nbv and UAV_NBV_AUTO_PREPARE=1')
    if args.require_heading_execution and (not args.require_nbv or os.getenv('UAV_NBV_HEADING_MODE') != 'discrete'):
        parser.error('--require-heading-execution needs --require-nbv and UAV_NBV_HEADING_MODE=discrete')
    if args.require_target_confirmation and (not args.require_nbv or os.getenv('UAV_TARGET_CONFIRMATION') != 'multiview'):
        parser.error('--require-target-confirmation needs --require-nbv and UAV_TARGET_CONFIRMATION=multiview')
    if (args.require_confirmed_target or args.require_rejected_candidate) and not args.require_target_confirmation:
        parser.error('Outcome requirements need --require-target-confirmation')
    if not args.execute:
        print('Dry run. Add --execute after starting the verified static_3d simulator. Task:', args.prompt)
        return 0
    from web_server import build_application
    from client import BridgeClient
    bridge = BridgeClient()
    state = bridge.state()
    if not (state.get('simulation_only') is True and state.get('ready') is True
            and state.get('armed') is False and state.get('landed_state') == 1
            and state.get('navigation', {}).get('method') == 'static_3d'
            and state.get('external_vision', {}).get('used_for_flight_control') is True):
        raise RuntimeError('Validation requires ready, grounded, disarmed static_3d SITL with actual VIO fusion')
    if args.require_nbv:
        from scene_config import load_scene
        if load_scene(args.world, navigation_mode='static_3d')['world_sha256'] != state.get('scene', {}).get('world_sha256'):
            raise RuntimeError('NBV audit world differs from the actual simulator world')
    seed_evidence = gazebo_seed_evidence(args.gazebo_seed, args.world) if args.gazebo_seed is not None else None
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['UAV_CAPTURE_EVIDENCE_DIR'] = str(args.output.resolve()/'captures')
    app = build_application(ROOT/'.env')
    if args.memory_history_turns:
        from validate_task_memory import fixture_history
        app.agent.update_state({'configurable': {'thread_id': app.agent_thread_id}},
                               {'messages': fixture_history(args.memory_history_turns)})
    identifier = app.submit(args.prompt)['id']
    run = (ROOT/'logs/system/latest').resolve().name
    print('COMMAND', identifier, 'RUN', run, flush=True)
    started, previous = time.monotonic(), None
    while time.monotonic()-started < 1200:
        command = app.command(identifier)
        connection_error = None
        try:
            state = bridge.state()
        except Exception as exc:
            connection_error = app.clean_error(exc)
        progress = (command.get('search_events') or [{}])[-1].get('result', {})
        mission = state.get('mission') or {}
        marker = (len(command.get('model_calls', [])), len(command.get('perceptions', [])),
                  progress.get('search_status'), mission.get('id'), mission.get('step'), command['status'])
        if marker != previous:
            print(json.dumps(dict(elapsed=round(time.monotonic()-started, 1), status=command['status'],
                calls=marker[0], reports=marker[1], search=marker[2], mission=mission.get('id'),
                action=mission.get('action'), position=state.get('position_enu_m')), ensure_ascii=False), flush=True)
            previous = marker
        with app.lock:
            failures = [(name, app.clean_perception(value)) for name, value in app.commands[identifier]['tool_results']
                        if isinstance(value, dict) and (value.get('status') == 'error'
                            or (value.get('mission') or {}).get('status') == 'failed')]
        record = dict(run=run, command=command, tool_failures=failures, connection_error=connection_error,
            search_mode=os.getenv('UAV_SEARCH_MODE', 'fixed'),
            target_confirmation_mode=os.getenv('UAV_TARGET_CONFIRMATION', 'single_frame'),
            nbv_selector=os.getenv('UAV_NBV_SELECTOR', 'deterministic'),
            candidate_pool_mode=os.getenv('UAV_NBV_CANDIDATE_POOL', 'ranked'),
            heading_mode=os.getenv('UAV_NBV_HEADING_MODE', 'fixed'),
            gazebo_seed_evidence=seed_evidence,
            nbv_auto_prepare=os.getenv('UAV_NBV_AUTO_PREPARE', '0') == '1',
            world_file=str(args.world.resolve()) if args.world else None,
            synthetic_history_turns=args.memory_history_turns,
            capture_directory=os.environ['UAV_CAPTURE_EVIDENCE_DIR'],
            elapsed_seconds=round(time.monotonic()-started, 1),
            flight={key: state.get(key) for key in ('ready', 'armed', 'landed_state', 'position_enu_m', 'external_vision', 'navigation')})
        temp = args.output/'result.tmp'
        temp.write_text(json.dumps(record, ensure_ascii=False, indent=2))
        temp.replace(args.output/'result.json')
        if connection_error:
            print('VALIDATION_ABORTED', connection_error, flush=True)
            return 2
        if command['status'] != 'running':
            lines = ['# 自主搜索实测报告', '', '指令：'+command['prompt'], '', '状态：'+command['status'], '',
                     command.get('reply') or command.get('error') or '没有最终说明', '']
            for index, report in enumerate(command.get('perceptions', []), 1):
                lines.extend(['## 观察 '+str(index), '', report.get('summary', ''), ''])
                capture = report.get('capture_id', '')
                if (args.output/'captures'/(capture+'.jpg')).exists():
                    lines.extend(['![实际相机照片](captures/'+capture+'.jpg)', ''])
            (args.output/'report.md').write_text('\n'.join(lines))
            print('RESULT', command['status'], 'REPLY', command.get('reply'), 'ERROR', command.get('error'), flush=True)
            if args.require_nbv:
                from audit_nbv_evidence import audit
                try:
                    nbv_audit = audit(args.output/'result.json', args.output/'nbv-audit', args.world,
                                      args.require_semantic_guidance, args.require_vlm_selection,
                                      args.require_diverse_pool, args.require_auto_prepare)
                except (ValueError, KeyError, OSError) as exc:
                    (args.output/'nbv-audit-error.json').write_text(json.dumps(
                        {'passed': False, 'error': str(exc)}, ensure_ascii=False, indent=2))
                    print('NBV_AUDIT_REJECTED', str(exc), flush=True)
                    return 2
                print('NBV_AUDIT', json.dumps(nbv_audit['checks'], ensure_ascii=False), flush=True)
                if not nbv_audit['passed']:
                    return 2
                if args.require_heading_execution and not any(move.get('heading_step') in (-1, 1) for move in nbv_audit['moves']):
                    print('NBV_AUDIT_REJECTED no executed heading candidate', flush=True)
                    return 2
            if args.require_target_confirmation:
                from audit_target_confirmation import audit as audit_confirmation
                confirmation = audit_confirmation(args.output/'result.json', args.output/'confirmation-audit',
                                                  args.require_confirmed_target, args.require_rejected_candidate)
                print('CONFIRMATION_AUDIT', json.dumps(confirmation, ensure_ascii=False), flush=True)
                if not confirmation['passed']:
                    return 2
            if args.memory_followup and command['status'] == 'succeeded':
                original = app.client
                class HistoryOnlyClient:
                    supports_stereo_localization = False
                    def state(self): return original.state()
                    def request(self, path, payload=None):
                        if payload is not None: raise RuntimeError('历史追问验证禁止任何飞行动作')
                        return original.request(path)
                    def capture_camera(self, *args): raise RuntimeError('历史追问验证禁止重新采图')
                    def capture_stereo(self): raise RuntimeError('历史追问验证禁止重新采图')
                app.client = HistoryOnlyClient()
                followup = app.submit('上一轮实际任务找到什么目标，周围有哪些物体？只根据历史报告回答，明确这是历史观察，不重新起飞或采图。')['id']
                end = time.monotonic()+90
                while time.monotonic()<end:
                    answer = app.command(followup)
                    if answer['status'] != 'running': break
                    time.sleep(.5)
                (args.output/'followup.json').write_text(json.dumps(answer, ensure_ascii=False, indent=2))
                assert answer['status'] == 'succeeded', answer.get('error')
                assert not answer['flight_missions'] and not answer['perceptions']
                assert all(c.get('input_modalities') == ['text'] for c in answer['model_calls'])
                assert '油桶' in answer.get('reply', '')
                print('MEMORY_FOLLOWUP_PASSED', answer['reply'], flush=True)
            return 0 if command['status'] == 'succeeded' else 1
        time.sleep(2)
    with app.lock:
        accepted = {v.get('mission', {}).get('id') for v in app.commands[identifier].get('flight_missions', [])}
    if (bridge.state().get('mission') or {}).get('id') in accepted:
        app.interrupt('/land')
    raise RuntimeError('Validation deadline exceeded; requested landing only for owned flight')


if __name__ == '__main__':
    sys.exit(main())
