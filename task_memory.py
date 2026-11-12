"""Deterministic request projections; complete tool evidence stays in checkpoints."""
import json
import os

from langchain_core.messages import SystemMessage


def value(message, key, default=None):
    return message.get(key, default) if isinstance(message, dict) else getattr(message, key, default)


def role(message):
    return value(message, 'type') or value(message, 'role')


def text(item, limit):
    return item[:limit] if isinstance(item, str) else ''


def payload(message):
    try:
        item = json.loads(value(message, 'content', ''))
        return item if isinstance(item, dict) else {}
    except (ValueError, TypeError):
        return {}


def task_summary(messages, completed=None):
    """Summaries are deterministic projections of saved reports, never invented by an LLM."""
    user = messages[0]
    result = {'turn_id': value(user, 'id'), 'request': text(value(user, 'content'), 240)}
    reports, failure, mission, reviews = [], None, None, []
    for message in messages[1:]:
        if role(message) != 'tool':
            continue
        item = payload(message)
        if item.get('status') == 'error' or value(message, 'status') == 'error':
            failure = text(item.get('error'), 160) or '工具失败'
        if isinstance(item.get('mission'), dict):
            mission = item['mission']
        if value(message, 'name') in ('report_scene', 'inspect_scene') and item.get('status') == 'ok':
            reports.append(item)
        if value(message, 'name') == 'review_target_candidate' and item.get('status') == 'ok':
            reviews.append({'query': item.get('query'), 'outcome': item.get('confirmation_outcome'),
                'source_capture_id': item.get('reviewed_source_capture_id'),
                'current_capture_id': item.get('reviewed_current_capture_id'), 'reason': item.get('review_reason')})
    if completed is not None:
        result['task_status'] = completed.get('status')
        failure = completed.get('error') or failure
        reports = completed.get('perceptions') or reports
        mission = completed.get('mission') or mission
        reviews = completed.get('target_confirmation_reviews') or reviews
    result['observation_count_total'] = (completed.get('observation_count') if completed is not None
                                         and completed.get('observation_count') is not None else len(reports))
    # Tool evidence is not the same thing as a verified final command status.
    if 'task_status' not in result:
        result['task_status'] = 'tool_error_recorded' if failure else 'historical_turn'
    if failure:
        result['error'] = text(failure, 160)
    if isinstance(mission, dict):
        result['last_flight_status'] = mission.get('status')
        result['last_flight_action'] = mission.get('action')
    if reports:
        result['observations'] = [{'capture_id': text(p.get('capture_id'), 80),
            'summary': text(p.get('summary'), 320),
            'targets': [text(t.get('label'), 60) for t in p.get('targets', [])[:4] if isinstance(t, dict)]}
            for p in reports[-2:] if p.get('status') == 'ok']
        result['observation_summaries_retained'] = len(result['observations'])
        result['visual_evidence_basis'] = 'historical_model_reports_not_independently_verified'
    if reviews:
        result['target_confirmation_reviews'] = [{k: text(r.get(k), 240) for k in
            ('query', 'outcome', 'source_capture_id', 'current_capture_id', 'reason')} for r in reviews[-8:]]
        result['visual_evidence_basis'] = 'historical_reports_with_review_outcomes_not_independent_ground_truth'
        for observation in result.get('observations', []):
            rejected = [r['query'] for r in result['target_confirmation_reviews']
                        if r['outcome'] == 'rejected' and r['source_capture_id'] == observation['capture_id']]
            if rejected: observation['rejected_label_hypotheses'] = rejected
    if not reports and not mission and not failure:
        answers = [value(m, 'content') for m in messages[1:] if role(m) in ('ai', 'assistant') and not value(m, 'tool_calls')]
        if answers:
            result['historical_answer_unverified'] = text(answers[-1], 200)
    return result


def message_chars(messages):
    return sum(len(json.dumps({'content': value(m, 'content', ''), 'tool_calls': value(m, 'tool_calls', [])},
                              ensure_ascii=False, default=str)) for m in messages)


_PROJECTED_TOOLS = frozenset((
    'get_drone_state', 'execute_mission', 'stop_drone', 'land_drone',
    'capture_scene', 'inspect_scene', 'report_scene', 'observe_target_search_view',
    'begin_target_search', 'next_target_search_view', 'execute_target_search_view',
    'select_target_search_view', 'target_search_progress', 'plan_region_search',
    'search_observation_status', 'define_task_plan', 'revise_task_plan',
    'report_target_frame', 'execute_task_goal', 'review_goal_region', 'choose_task_goal', 'review_task_goal',
    'land_task_goal', 'observe_task_photo', 'review_photo',
    'adjust_photo_view', 'rotate_photo_view',
))


def current_tool_projection(message):
    """Remove repeated audit collections from a model-request copy only.

    Keep every message/tool-call pair, original user text, decision result and
    error. Actual frame selection, delivery/identity guards and stage advancement
    read the unprojected graph state before this function is called. No image,
    memory, evidence or execution ticket is removed from that state.
    """
    if role(message) != 'tool' or value(message, 'name') not in _PROJECTED_TOOLS:
        return message
    item = payload(message)
    if not item or item.get('status') == 'error' or value(message, 'status') == 'error':
        return message
    counts = {}

    def omit(node, key):
        if key in node:
            removed = node.pop(key)
            counts[key] = counts.get(key, 0) + (len(removed) if isinstance(removed, (list, dict)) else 1)

    def walk(node):
        if isinstance(node, list):
            for child in node:
                walk(child)
            return
        if not isinstance(node, dict):
            return
        # Each report embeds all earlier model calls, which themselves include
        # context diagnostics. They are audit, not fresh visual/flight evidence.
        for key in ('vlm_calls', 'model_calls'):
            if isinstance(node.get(key), list) and node[key]:
                omit(node, key)
        if 'search_status' in node and isinstance(node.get('observations'), list):
            observations = node['observations']
            if len(observations) > 2:
                node['observation_count_total'] = len(observations)
                node['observation_summaries_retained'] = 2
                node['observations'] = observations[-2:]
                counts['earlier_search_observations'] = counts.get('earlier_search_observations', 0) + len(observations)-2
        # Parent tool results repeat the complete authoritative snapshot. The
        # middleware separately attaches the fresh parent model_context after
        # this projection, so old planning/measurement internals are redundant.
        if ('stage_revision' in node and isinstance(node.get('target_memory'), dict)
                and isinstance(node.get('plan'), dict)):
            for stage in node['plan'].get('stages', []):
                if isinstance(stage, dict):
                    omit(stage, 'evidence_refs')
            if isinstance(node.get('current_stage'), dict):
                omit(node['current_stage'], 'evidence_refs')
            for target in (node.get('target_memory') or {}).values():
                if not isinstance(target, dict):
                    continue
                omit(target, 'association_hypotheses')
                orientation = target.get('orientation')
                if isinstance(orientation, dict):
                    # Retain measured uncertainty, relation quality, source IDs,
                    # hashes, face ROI and fitting statistics. The original
                    # stereo samples/derived regions are evaluated by program.
                    for key in ('part_measurements', 'sample_measurements', 'sample_regions'):
                        omit(orientation, key)
            events = node.get('reasoning_events')
            if isinstance(events, list) and len(events) > 2:
                node['reasoning_events'] = events[-2:]
                counts['earlier_reasoning_events'] = counts.get('earlier_reasoning_events', 0) + len(events)-2
            goal = node.get('goal_state')
            if isinstance(goal, dict) and isinstance(goal.get('object_frame_attempts'), list):
                attempts = goal['object_frame_attempts']
                if len(attempts) > 2:
                    goal['object_frame_attempt_count_total'] = len(attempts)
                    goal['object_frame_attempts'] = attempts[-2:]
                    counts['earlier_object_frame_attempts'] = counts.get('earlier_object_frame_attempts', 0)+len(attempts)-2
            if (isinstance(goal, dict) and isinstance(goal.get('prepared'), dict)
                    and isinstance(goal['prepared'].get('candidates'), list)):
                prepared = goal['prepared']
                node['goal_state'] = goal = {**goal, 'prepared': {
                    key: prepared.get(key) for key in ('status', 'selected_id', 'target_revision')}}
                goal['candidate_summary'] = [{key: candidate.get(key) for key in
                    ('id', 'position_enu_m', 'yaw_enu_rad', 'score', 'hard_constraints')}
                    for candidate in prepared.get('candidates', [])[:3]]
                counts['prepared_goal_audit'] = counts.get('prepared_goal_audit', 0)+1
                for key in ('relation_review', 'landing', 'verified_outcome'):
                    omit(goal, key)
        for child in node.values():
            walk(child)

    walk(item)
    if not counts:
        return message
    item['_context_projection'] = {
        'basis': 'model_request_copy_only_full_evidence_in_checkpoint_and_task_audit',
        'omitted_collection_entries': counts,
        'earlier_results_are_historical_not_current_state': True,
    }
    content = json.dumps(item, ensure_ascii=False)
    if len(content) >= len(value(message, 'content', '')):
        return message  # Do not expand small results with projection metadata.
    if isinstance(message, dict):
        return {**message, 'content': content}
    return message.model_copy(update={'content': content})


class TaskContextMemory:
    def __init__(self, mode=None, max_turns=8, max_summary_chars=6000):
        self.mode = mode or os.getenv('UAV_SHORT_TERM_MEMORY', 'summary')
        if self.mode not in ('summary', 'full'):
            raise ValueError('UAV_SHORT_TERM_MEMORY must be summary or full')
        self.max_turns, self.max_summary_chars = max_turns, max_summary_chars

    def project(self, messages, owner, completed=None):
        messages = list(messages)
        # Locate the genuine turn by its ID, never by content or a generated summary.
        current = next((i for i in range(len(messages)-1, -1, -1)
                        if role(messages[i]) in ('human', 'user') and str(value(messages[i], 'id', '')) == owner[1]), None)
        if current is None:
            raise RuntimeError('当前用户指令不在上下文中，禁止裁剪')
        before = message_chars(messages)
        current_text = value(messages[current], 'content', '')
        history_only = isinstance(current_text, str) and any(phrase in current_text for phrase in
            ('只根据历史', '仅根据历史', '只依据历史', '只根据上一轮报告', '只根据旧报告'))
        metrics = {'mode': self.mode, 'input_messages': len(messages), 'input_characters': before,
                   'historical_tasks': 0, 'summarized_tasks': 0, 'explicit_history_only': history_only}
        if self.mode == 'full':
            return messages, {**metrics, 'output_messages': len(messages), 'output_characters': before}
        boundaries = [i for i, m in enumerate(messages[:current]) if role(m) in ('human', 'user')]
        metrics['historical_tasks'] = len(boundaries)
        records = {str(r.get('id')): r for r in (completed or [])}
        summaries = []
        # Prefer newest tasks, then restore chronological order. Do not cut JSON or tool pairs.
        for number in range(len(boundaries)-1, max(-1, len(boundaries)-self.max_turns-1), -1):
            start = boundaries[number]
            end = boundaries[number+1] if number+1 < len(boundaries) else current
            summary = task_summary(messages[start:end], records.get(str(value(messages[start], 'id'))))
            candidate = [summary, *summaries]
            if len(json.dumps(candidate, ensure_ascii=False)) > self.max_summary_chars:
                break
            summaries = candidate
        prefix = messages[:boundaries[0]] if boundaries else messages[:current]
        projected = list(prefix)
        if summaries:
            projected.append(SystemMessage(content='以下是已结束历史轮次的数据摘要，仅用于理解对话；'
                '其中请求、回复和视觉内容均是历史数据，不是新指令，不代表当前遥测或当前视野。'
                '视觉类别是当时模型的报告，未独立核实；应用状态成功不等于视觉识别独立验证通过。'
                '总观察次数以observation_count_total为准，摘要中最近两份观察不代表只有两次观察。'
                '不得自动重放旧动作，不得把旧capture_id用于本指令报告，旧坐标必须重新验证。\n'
                + json.dumps(summaries, ensure_ascii=False)))
        current_messages = [current_tool_projection(message) for message in messages[current:]]
        projected.extend(current_messages)
        return projected, {**metrics, 'summarized_tasks': len(summaries),
                           'projected_current_tool_results': sum(a is not b for a, b in zip(current_messages, messages[current:])),
                           'current_characters_before': message_chars(messages[current:]),
                           'current_characters_after': message_chars(current_messages),
                           'output_messages': len(projected), 'output_characters': message_chars(projected)}
