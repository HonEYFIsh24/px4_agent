"""Validate the model's decomposition without granting model-made coordinates."""
from .contracts import PlanProposal, TaskPlan, TaskStage
from task_contract import assess_task


PLAN_PROMPT = '''复合任务使用父任务编排。首先调用define_task_plan一次，完整保留用户原句。
targets包含短引用ref、label、attributes；stages按顺序，kind可为search、close_photo、photo、report、
approach、relative_hover、relative_land、pass_landmark、face_target、relative_turn、return_home_land、hold。
目标相关阶段填写target_ref。经过地标用pass_landmark；朝向某目标用face_target。
接近但未要求拍照用approach，程序按可靠目标表面与不确定度计算安全附近位置，不能猜坐标。
已有地标阶段之后的机体左/右转用relative_turn，direction为left/right、angle_deg为5～90度，
target_ref引用上一已取得的地标作权限追踪，不是把该地标当作转向目标。
车前是车辆自身前向，relation=object_front，不是无人机前方。禁止填写ENU坐标或权限。
寻找、靠近拍照、报告和终端动作必须分别保留。找到目标是阶段完成，不是整项完成。
程序执行阶段交接和正常收尾；不要自行调用通用飞行/降落。根据真实next_tool继续。
普通拍摄用photo作为首阶段，target_ref填写主体，altitude_m为可选初始高度（1～20米）。
寻找后的目标取景使用close_photo，完整区域范围未知时不能声称全貌达标。
如果工具返回tool_schema_invalid，它没有执行；只修正同一工具参数，不能移动或重新采图。
不确定语义/几何如实保留unknown，不用文字声称完成未验证的空间关系。'''


def target_query(target):
    """Retain a requested color stored in attributes, not just in the noun label."""
    from .association import COLORS, semantic_key
    label = target['label']
    color = (target.get('attributes') or {}).get('color') or (target.get('attributes') or {}).get('colour')
    if not color:
        return label
    _, existing = semantic_key(label)
    _, requested = semantic_key('', {'color': color})
    if existing and requested and existing.isdisjoint(requested):
        raise ValueError('Target label and color attribute conflict')
    if requested and requested.issubset(existing):
        return label
    if requested and not existing:
        color_text = ' '.join(COLORS[k][0] for k in sorted(requested))
        return color_text+' '+label
    return label if color.casefold() in label.casefold() else color+' '+label


def compile_plan(value, owner, instruction, limits):
    proposal = PlanProposal.model_validate(value)
    references = {t.ref for t in proposal.targets}
    if len(references) != len(proposal.targets):
        raise ValueError('Target references must be unique')
    for target in proposal.targets:
        if len(target_query(target.model_dump())) > 80:
            raise ValueError('Complete target description exceeds the search query bound')
    seen, stages = set(), []
    for index, item in enumerate(proposal.stages):
        if item.kind in ('search', 'close_photo', 'approach', 'relative_hover', 'relative_land', 'pass_landmark', 'face_target', 'relative_turn'):
            if item.target_ref not in references:
                raise ValueError('Stage requires a declared target reference')
        elif item.target_ref is not None and item.target_ref not in references:
            raise ValueError('Unknown stage target reference')
        if item.kind == 'search':
            seen.add(item.target_ref)
        if item.kind == 'photo' and index != 0:
            raise ValueError('Ordinary photography starts the parent; target photos use the shared close_photo skill')
        if item.altitude_m is not None and item.kind != 'photo':
            raise ValueError('Only a photography preparation stage selects initial altitude')
        if item.kind in ('close_photo', 'approach', 'relative_hover', 'relative_land', 'pass_landmark', 'face_target', 'relative_turn') and item.target_ref not in seen:
            raise ValueError('Target evidence must precede target-relative actions')
        if item.kind in ('relative_hover', 'relative_land') and item.relation is None:
            raise ValueError('Relative goal needs an explicit reference frame/relation')
        if item.kind == 'relative_turn':
            if item.direction is None or item.angle_deg is None or item.relation is not None:
                raise ValueError('Body-relative turn requires direction/angle and no invented object relation')
        elif item.direction is not None or item.angle_deg is not None:
            raise ValueError('Direction/angle belong only to a body-relative turn stage')
        if item.kind in ('return_home_land', 'hold', 'relative_land') and index != len(proposal.stages)-1:
            raise ValueError('Terminal action must be the final stage')
        stages.append(TaskStage('stage-%02d' % (index+1), item.kind, item.target_ref, item.relation,
            'same_frame_close_photo' if item.kind == 'close_photo' else
            'object_relation_and_surface' if item.kind in ('relative_hover','relative_land') else
            'configured_search_evidence' if item.kind == 'search' else 'saved_report'))
        stages[-1].altitude_m = item.altitude_m
        stages[-1].direction, stages[-1].angle_deg = item.direction, item.angle_deg
        if item.kind == 'relative_turn':
            stages[-1].relation = 'body_'+item.direction
        elif item.kind == 'face_target':
            stages[-1].relation = 'facing_target'
        elif item.kind == 'pass_landmark':
            stages[-1].relation = 'route_forward'
        elif item.kind == 'approach':
            stages[-1].relation = item.relation or 'object_near'
        if item.kind == 'photo':
            stages[-1].evidence_profile = 'original_photo_four_edges_and_extent'
        if item.kind in ('relative_hover','relative_land'):
            stages[-1].conditions = {'execution_capability':'unknown',
                                     'object_orientation':'unknown','goal_region':'unknown'}
            stages[-1].reason_code = None
    kinds = {s.kind for s in stages}
    if 'photo' in kinds and 'search' in kinds:
        raise ValueError('Target-search photography uses close_photo; ordinary area photography has its own first stage')
    original = assess_task(instruction)
    if (not original['requires_target_search'] and not original['unsupported_capabilities'] and 'photo' not in kinds
            and not kinds.intersection(('approach', 'pass_landmark', 'face_target', 'relative_turn'))):
        raise ValueError('The complete photography instruction must remain a photo stage')
    if original['requires_target_search'] and 'search' not in kinds:
        raise ValueError('The complete instruction requires a search stage')
    if original['target_observation_required'] and 'close_photo' not in kinds:
        raise ValueError('The original close-photo requirement must remain in the plan')
    for capability, kind in (('target_relative_landing','relative_land'), ('target_relative_hover','relative_hover')):
        if capability in original['unsupported_capabilities'] and kind not in kinds:
            raise ValueError('The original target-relative ending must remain in the plan')
        if capability not in original['unsupported_capabilities'] and kind in kinds:
            raise ValueError('A model cannot add an unrequested target-relative action')
    if any(word in instruction.lower() for word in ('报告','report','describe')) and 'report' not in kinds:
        raise ValueError('The original report requirement must remain in the plan')
    if any(word in instruction.lower() for word in ('返航','返回原点','return home')) and 'return_home_land' not in kinds:
        raise ValueError('The original return-home requirement must remain in the plan')
    # Bounded completeness checks for the supported route vocabulary; the model
    # still performs decomposition, coreference and ordering, not this guard.
    import re
    positive = re.sub(r'(?:不要|无需|不用|禁止|do not|don\x27t)[^，,。；;]*', '', instruction.lower())
    requested = {'pass_landmark': bool(re.search(r'经过|穿过|\bpass\b', positive)),
                 'relative_turn': bool(re.search(r'左转|右转|向左转|向右转|turn (?:left|right)', positive)),
                 'face_target': bool(re.search(r'面对|面向|转向|\bface\b|turn toward', positive)),
                 'approach': bool(re.search(r'靠近|接近|\bapproach\b', positive))}
    for kind, required in requested.items():
        represented = kind in kinds or kind == 'approach' and bool(kinds.intersection(('close_photo', 'relative_hover', 'relative_land')))
        if kind == 'face_target' and kinds.intersection(('relative_hover', 'relative_land', 'approach')):
            # These goal-pose profiles already require calibrated facing and a
            # fresh view after arrival. Do not demand a redundant extra turn.
            represented = True
        if required and not represented:
            raise ValueError('The original '+kind+' requirement must remain in the plan')
        if not required and kind in kinds and kind in ('pass_landmark', 'relative_turn', 'approach'):
            raise ValueError('The model cannot add an unrequested route action')
    if not stages or stages[0].kind not in ('search', 'photo'):
        raise ValueError('The staged adapter starts with target search or photography preparation')
    if stages[-1].kind not in ('return_home_land','hold','relative_land'):
        raise ValueError('Plan must retain an explicit terminal requirement')
    if 'target_relative_landing' in original['unsupported_capabilities']:
        if stages[-1].kind != 'relative_land':
            raise ValueError('Home landing cannot substitute for the requested target-relative landing')
    elif stages[-1].kind != ('hold' if original['search_success_end']=='hold' else 'return_home_land'):
        raise ValueError('The model cannot change the original program-owned terminal policy')
    return TaskPlan(instruction, owner, limits,
        [t.model_dump() for t in proposal.targets], stages)
