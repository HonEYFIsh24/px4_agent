"""Bounded task capability checks; no model, navigation or flight side effects.

This is a guard for known unsupported target-relative endings, not a claim to
understand arbitrary natural language. Keep the original request for auditing.
"""
import re


def contract_from_messages(messages):
    for message in reversed(messages):
        role = message.get('role', message.get('type')) if isinstance(message, dict) else getattr(message, 'type', None)
        if role in ('user', 'human'):
            content = message.get('content') if isinstance(message, dict) else message.content
            return assess_task(content) if isinstance(content, str) else None
    return None


def assess_task(prompt):
    text = prompt.strip()
    clauses = re.split(r'[。；;\n]', text.lower())
    unsupported = []
    positive_clauses = []
    for clause in clauses:
        # Explicit prohibitions do not request a target-relative maneuver.
        clause = re.sub(r'(?:不要|不需要|无需|禁止|不用)[^，,]*', '', clause)
        clause = re.sub(r"(?:do not|don't|no need to)[^,]*", '', clause)
        positive_clauses.append(clause)
        ending = re.search(r'降落|着陆|着地|落地|停在|悬停|\bland\b|\bhover\b', clause)
        relation = re.search(r'前方|前面|车前|车头|后方|后面|旁边|附近|左侧|右侧|顶部|车顶|上面|上方|边上|旁|in front of|behind|beside|next to|near|on top of|above', clause)
        target = re.search(r'货车|卡车|车辆|汽车|帐篷|油桶|目标|物体|集装箱|它|truck|vehicle|van\b|tent|barrel|target|object|\bit\b', clause)
        # Named objects outside the small vocabulary can still be recognized as
        # a relative destination by explicit locative syntax.
        relative_destination = re.search(r'(?:降落|着陆|悬停)(?:到|在).*?(?:前方|前面|旁边|附近|后方|后面|上方|顶部)', clause)
        # Direction attached to the search noun alone is not a landing relation:
        # "寻找前方的货车，完成后返航降落" remains supported.
        if ending and relation and (target or relative_destination):
            ending_clause = re.split(r'[，,]', clause)
            relative_ending = any(
                re.search(r'(?:' + ending.re.pattern + r').*?(?:' + relation.re.pattern + ')', part)
                or re.search(r'(?<!找)(?:在|到|于).*?(?:' + relation.re.pattern + r').*?(?:' + ending.re.pattern + ')', part)
                or re.search(r'(?:货车|卡车|车辆|汽车|帐篷|油桶|目标|物体|集装箱|它|其).*?(?:'
                             + relation.re.pattern + r').*?(?:' + ending.re.pattern + ')', part)
                for part in ending_clause)
            if relative_ending:
                unsupported.append('target_relative_landing' if re.search(r'降落|着陆|着地|落地|\bland\b', clause)
                                   else 'target_relative_hover')
    positive = '；'.join(positive_clauses)
    if re.search(r'(?:没找到|未找到|失败|未确认)[^。；]{0,30}(?:悬停|不要返航|原地降落)', positive):
        unsupported.append('custom_unmet_search_end_policy')
    hold = bool(re.search(r'悬停|保持悬停|\bhover\b|\bhold\b', positive, re.I))
    home = bool(re.search(r'返航|返回(?:起点|原点|起飞点)|return home', positive, re.I))
    return {'version': 1, 'original_request': prompt, 'admission': 'rejected' if unsupported else 'accepted',
            'requires_target_search': bool(re.search(r'找到|寻找|搜寻|\bfind\b|\bsearch for\b', positive)),
            'unsupported_capabilities': sorted(set(unsupported)),
            'matching_scope': 'known_target_relative_endings_not_general_language_completeness',
            'search_success_end': 'hold' if hold and not home else 'return_home_land',
            'search_unmet_end': 'return_home_land_if_healthy_else_owned_safety_cleanup',
            'policy_source': 'program_task_contract',
            'reason': ('当前不支持请求中的目标相对悬停/降落或指定的搜索失败处置；完整任务未接纳，不会自动改成普通搜索。'
                       if unsupported else None)}


def search_result(command, flight_completion_verified=False):
    """Separate goal completion from flow termination and recovery flight."""
    progress = [e['result'] for e in command.get('search_events', [])
                if isinstance(e.get('result'), dict) and 'search_status' in e['result']]
    if not progress:
        # The original bounded grid search has a different audited schema.
        # Preserve its measured outcome without upgrading it to VLM confirmation.
        region = [e['result']['search'] for e in command.get('search_events', [])
                  if isinstance(e.get('result'), dict) and isinstance(e['result'].get('search'), dict)
                  and 'status' in e['result']['search']]
        if not region:
            return None
        status = region[-1]['status']
        found = status == 'target_found'
        return {'user_goal_completed': found and flight_completion_verified,
                'search_process_ended': status in ('target_found', 'sampling_complete', 'budget_exhausted'),
                'search_status': status, 'target_found': found, 'target_confirmed': False,
                'confirmation_level': 'legacy_stereo_geometry' if found else 'unconfirmed',
                'flight_status': 'grounded_disarmed' if flight_completion_verified else 'not_verified',
                'flight_source': 'legacy_audited_flight',
                'basis': 'legacy_grid_search_and_owned_landing_evidence'}
    last = progress[-1]
    found = last.get('search_status') == 'target_found'
    multiview = last.get('target_confirmation_mode') == 'multiview'
    confirmed = found and multiview and any(e.get('outcome') == 'confirmed'
                                            for e in command.get('target_confirmation_events', []))
    termination = last.get('termination') or {}
    contract = command.get('task_contract') or assess_task(command['prompt'])
    normal = termination.get('source') == 'normal_completion' and termination.get('purpose') != 'unmet_goal_recovery'
    ending_ok = normal and (termination.get('flight_status') == 'grounded_disarmed'
                           or contract['search_success_end'] == 'hold')
    ended = last.get('search_status') in ('target_found', 'budget_exhausted', 'coverage_exhausted', 'no_reachable_view')
    return {'user_goal_completed': bool(found and (confirmed or not multiview) and ending_ok),
            'search_process_ended': ended, 'search_status': last.get('search_status'),
            'target_found': found, 'target_confirmed': confirmed,
            'confirmation_level': 'multiview' if confirmed else 'single_frame' if found and not multiview else 'unconfirmed',
            'flight_status': termination.get('flight_status', 'not_verified'),
            'flight_source': termination.get('source'), 'flight_purpose': termination.get('purpose'),
            'basis': 'task_contract_and_structured_search_evidence'}
