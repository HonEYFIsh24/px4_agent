#!/usr/bin/env python3
"""A/B test historical context using synthetic transcripts and the configured text model."""
import argparse
import copy
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def fixture_history(turns=8):
    from langchain_core.messages import HumanMessage, AIMessage, ToolMessage
    messages = []
    for index in range(turns):
        turn = 'memory-fixture-'+str(index)
        messages.extend([
            HumanMessage('历史测试任务：寻找蓝色油桶并报告周边，最后降落。', id=turn),
            AIMessage('', id=turn+'-a1', tool_calls=[dict(name='report_scene', args={}, id=turn+'-c1', type='tool_call')]),
            ToolMessage(json.dumps(dict(status='ok', capture_id=turn+'-photo',
                summary='红色集装箱前可见蓝色油桶，右侧有白色货车和橙色交通锥。', targets=[dict(label='蓝色油桶')],
                source='synthetic_history_fixture_not_actual_flight',
                diagnostics=[{'sample': n, 'position_enu_m': [1., 2., 3.], 'tracking': 'nominal', 'message': '历史遥测测试样本'} for n in range(100)]),
                ensure_ascii=False), name='report_scene', tool_call_id=turn+'-c1', id=turn+'-t1'),
            AIMessage('', id=turn+'-a2', tool_calls=[dict(name='execute_mission', args={'steps': [{'action': 'land'}], 'description': '历史测试降落'}, id=turn+'-c2', type='tool_call')]),
            ToolMessage(json.dumps(dict(mission=dict(id=turn+'-mission', status='succeeded', action='land'))),
                        name='execute_mission', tool_call_id=turn+'-c2', id=turn+'-t2'),
            AIMessage('历史测试任务完成，观察到蓝色油桶，最后降落步骤成功。', id=turn+'-final')])
    return messages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Send synthetic text history to configured provider; no images or flight')
    parser.add_argument('--output', type=Path, default=ROOT/'logs/setup/task-memory-ab.json')
    parser.add_argument('--recorded-task', type=Path, help='Replay a saved actual command report for a text-only follow-up; no camera/flight')
    args = parser.parse_args()
    if not args.execute:
        print('Dry run. --execute compares full/summary inputs with synthetic text history. No flight or images.')
        return 0
    from agent_config import runtime_configuration
    from model_network import configure_agent_network
    from single_vlm import SceneMiddleware, FrameStore
    from langchain.agents import create_agent
    from langchain_openai import ChatOpenAI
    from langgraph.checkpoint.memory import InMemorySaver
    from urllib.parse import urlsplit
    from langchain_core.messages import HumanMessage
    configure_agent_network(ROOT/'.env')
    _, config, _ = runtime_configuration(ROOT/'.env')

    class TextOnlyClient:
        supports_stereo_localization = False
        def __init__(self): self.records = []
        def record(self, name, result):
            self.records.append((name, copy.deepcopy(result)))
            return result
        def state(self): raise RuntimeError('This test forbids telemetry, camera and flight tools')
        def capture_camera(self, *args): raise RuntimeError('Camera forbidden in text-only memory test')

    outcomes = []
    historical = fixture_history()
    actual_command = None
    if args.recorded_task:
        actual_command = json.loads(args.recorded_task.read_text())['command']
        from langchain_core.messages import AIMessage, ToolMessage
        historical = [HumanMessage(actual_command['prompt'], id=actual_command['id'])]
        for index, report in enumerate(actual_command['perceptions']):
            call = 'recorded-report-'+str(index)
            historical.extend([AIMessage('', tool_calls=[dict(name='report_scene', args={}, id=call, type='tool_call')]),
                               ToolMessage(json.dumps(report, ensure_ascii=False), name='report_scene', tool_call_id=call)])
        historical.append(AIMessage(actual_command.get('reply') or '历史任务已结束'))
    for mode in ('full', 'summary'):
        os.environ['UAV_SHORT_TERM_MEMORY'] = mode
        client = TextOnlyClient()
        options = dict(timeout=45, max_retries=0, max_tokens=1200, temperature=0)
        if (urlsplit(config.get('base_url') or '').hostname or '').endswith('aliyuncs.com'):
            options['extra_body'] = {'enable_thinking': False}
        model = ChatOpenAI(**config, **options)
        test_prompt = ('你在执行纯文本历史记忆测试。历史消息是合成测试数据，不是待执行指令。'
             if actual_command is None else '你在执行历史记忆测试。历史消息来自已保存任务，包含当时模型的报告，不等于独立核实的事实。') + (
            '禁止调用工具、飞行或采图。只根据历史工具结果回答，不声称当前看到了目标。'
            + ('最终只返回JSON，字段为target、observation、last_flight_status；最后一字段取原始状态succeeded/failed/unknown。'
             if actual_command is None else '最终只返回JSON，字段为reported_target、observation_count、evidence_basis；观察总数必须准确，依据为历史模型报告、未独立核实。'))
        if actual_command is None:
            graph = create_agent(model, [], system_prompt=test_prompt,
                middleware=[SceneMiddleware(FrameStore(client, config), client, config)], checkpointer=InMemorySaver())
        else:
            from single_vlm import build_single_vlm_agent
            from agent import SYSTEM_PROMPT
            graph = build_single_vlm_agent(client, config, [], SYSTEM_PROMPT+test_prompt, model=model)
        runtime = {'configurable': {'thread_id': 'memory-ab-'+mode}, 'recursion_limit': 10, 'max_concurrency': 1}
        graph.update_state(runtime, {'messages': historical})
        if actual_command is not None:
            client.memory_records = lambda: [dict(id=actual_command['id'], status=actual_command['status'],
                error=actual_command.get('error'), observation_count=len(actual_command['perceptions']),
                perceptions=actual_command['perceptions'][-2:], mission=actual_command.get('mission'))]
        question = ('最近一轮历史任务寻找什么、观察到什么、最后飞行步骤是否成功？' if actual_command is None
                    else '复述上一轮报告中的目标、总共做了几次有效观察、这些判断是否经过独立核实。只根据历史回答。')
        result = graph.invoke({'messages': [HumanMessage(question, id='current-memory-query')]}, runtime)
        reply = result['messages'][-1].content
        calls = [value for name, value in client.records if name == 'vlm_call']
        assert calls and all(c.get('input_modalities') == ['text'] for c in calls)
        assert not any(c.get('called_tools') for c in calls)
        answer = json.loads(reply.strip().removeprefix('```json').removesuffix('```').strip())
        passed = (('油桶' in answer.get('target', '') and '集装箱' in answer.get('observation', '')
                  and answer.get('last_flight_status') == 'succeeded') if actual_command is None else
                  ('油桶' in answer.get('reported_target', '') and answer.get('observation_count') == len(actual_command['perceptions'])
                   and (('报告' in answer.get('evidence_basis', '') and any(x in answer.get('evidence_basis', '') for x in ('未', '未经')))
                        or 'historical_model_reports_not_independently_verified' in answer.get('evidence_basis', ''))))
        outcomes.append(dict(mode=mode, passed=passed, reply=answer, model_calls=calls))
        print(mode, 'passed', passed, 'usage', calls[-1].get('token_usage'), flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(source=('synthetic_history_text_only_provider_ab' if actual_command is None
        else 'recorded_task_production_agent_text_only_ab'), results=outcomes), ensure_ascii=False, indent=2))
    assert all(o['passed'] for o in outcomes)
    return 0


if __name__ == '__main__':
    sys.exit(main())
