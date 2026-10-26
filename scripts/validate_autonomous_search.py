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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Actually send simulator images to the configured model and fly SITL')
    parser.add_argument('--prompt', default='找到前方的油桶，拍照分析并报告周围的情况。')
    parser.add_argument('--output', type=Path, default=ROOT/'logs/setup/autonomous-target-search')
    parser.add_argument('--memory-history-turns', type=int, default=0, choices=range(0, 21),
                        help='Seed explicit synthetic historical turns before the real current flight test')
    parser.add_argument('--memory-followup', action='store_true', help='After landing, ask about the previous task with camera/flight transport disabled')
    args = parser.parse_args()
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
    args.output.mkdir(parents=True, exist_ok=True)
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
