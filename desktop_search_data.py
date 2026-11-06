"""Read-only desktop projection of task-owned search evidence.

Uses atomic saved snapshots and the exact archived model-input JPEG; it never
reads the live video as a historical observation or exposes execution tickets.
"""
import hashlib
import json
import math
from pathlib import Path


def landmark_uncertainty_ellipse(covariance):
    """Two-sigma XY uncertainty ellipse; this is not an object's footprint."""
    try:
        xx, yy = covariance[0][0], covariance[1][1]
        xy = (covariance[0][1] + covariance[1][0])/2
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (xx, yy, xy)):
            return None
        half = (xx+yy)/2
        spread = math.hypot((xx-yy)/2, xy)
        major, minor = half+spread, half-spread
        if not math.isfinite(major) or not math.isfinite(minor) or minor < -1e-9 or major < 0:
            return None
        return (2*math.sqrt(major), 2*math.sqrt(max(0., minor)), .5*math.atan2(2*xy, xx-yy))
    except (IndexError, TypeError):
        return None


def static_obstacle_outlines(snapshot, world_path):
    """Unlabelled raw collision AABB footprints, only from the hash-verified world.

    These are 3-D AABB XY projections, not semantic objects or 2-D flight exclusions.
    """
    if not snapshot or not world_path:
        return None
    try:
        from static_navigation import collision_boxes
        boxes = collision_boxes(world_path, snapshot['world_sha256'])
        registration = snapshot['registration']
        ox, oy = registration['world_from_enu_origin_m'][:2]
        yaw = registration['world_from_enu_yaw_rad']
        c, s = math.cos(yaw), math.sin(yaw)
        outlines = []
        for low, high in boxes:
            outlines.append([[c*(x-ox)+s*(y-oy), -s*(x-ox)+c*(y-oy)] for x, y in
                             ((low[0], low[1]), (high[0], low[1]),
                              (high[0], high[1]), (low[0], high[1]))])
        return outlines
    except Exception:
        # No geometry is preferable to inventing an outline for a mismatched world.
        # Preserve ordinary startup when a custom or unavailable world is used.
        return None


def _id(value):
    return isinstance(value, str) and 0 < len(value) <= 128 and Path(value).name == value and value not in ('.', '..')


def search_snapshot(command, evidence_dir):
    if not isinstance(command, dict) or not _id(command.get('id')) or not evidence_dir:
        return None
    reports = {r.get('capture_id'): r for r in command.get('perceptions', [])
               if isinstance(r, dict) and r.get('status') == 'ok' and _id(r.get('capture_id'))}
    for event in reversed(command.get('semantic_map_events', [])):
        if event.get('status') != 'valid':
            return None
        identifier = event.get('capture_id')
        if identifier not in reports:
            continue
        try:
            path = Path(evidence_dir)/(identifier+'.semantic-map.json')
            if path.stat().st_size > 4_000_000:
                continue
            snapshot = json.loads(path.read_text())
            owner = snapshot.get('owner') or []
            from search_limits import validate_grid
            cell_count = validate_grid(snapshot.get('grid') or {})
            if (len(owner) != 2 or owner[1] != command['id'] or snapshot.get('status') != 'valid'
                    or snapshot.get('world_sha256') != event.get('world_sha256')
                    or snapshot.get('sortie_id') != event.get('sortie_id')
                    or not snapshot.get('observations') or snapshot['observations'][-1].get('capture_id') != identifier
                    or not isinstance((reports[identifier].get('camera') or {}).get('image_sha256'), str)
                    or len((reports[identifier].get('camera') or {}).get('image_sha256')) != 64
                    or (snapshot['observations'][-1].get('image_sha256') or {}).get('left') != (reports[identifier].get('camera') or {}).get('image_sha256')
                    or len(snapshot.get('cells', [])) != cell_count):
                continue
            return snapshot
        except (OSError, ValueError, TypeError, KeyError):
            continue
    return None


def observation_image(command, capture_id, evidence_dir):
    if not _id(capture_id) or not evidence_dir or not isinstance(command, dict):
        return None
    report = next((r for r in command.get('perceptions', []) if r.get('capture_id') == capture_id
                   and r.get('status') == 'ok'), None)
    if not report:
        return None
    expected = (report.get('camera') or {}).get('image_sha256')
    if not isinstance(expected, str) or len(expected) != 64:
        return None
    try:
        path = Path(evidence_dir)/(capture_id+'.jpg')
        if not 0 < path.stat().st_size <= 2_000_000:
            return None
        data = path.read_bytes()
        return data if hashlib.sha256(data).hexdigest() == expected else None
    except OSError:
        return None


def current_prediction(command, snapshot):
    if not snapshot or not isinstance(command, dict):
        return None
    observations = snapshot.get('observations') or []
    if not observations:
        return None
    try:
        revision = hashlib.sha256(json.dumps(snapshot, ensure_ascii=False, sort_keys=True, allow_nan=False).encode()).hexdigest()
    except (TypeError, ValueError):
        return None
    events = list(reversed(command.get('nbv_events', []))) + list(reversed(command.get('nbv_candidate_events', [])))
    for event in events:
        identity = event.get('identity') or {}
        if (event.get('owner') == snapshot.get('owner') and event.get('map_revision') == revision
                and event.get('source_capture_id') == observations[-1].get('capture_id')
                and identity.get('world_sha256') == snapshot.get('world_sha256')
                and identity.get('sortie_id') == snapshot.get('sortie_id')
                and identity.get('coordinate_epoch') == snapshot.get('coordinate_epoch')):
            return event
    return None


def telemetry_matches(state, snapshot):
    from search_tools import coordinate_epoch
    return bool(isinstance(state, dict) and snapshot and state.get('ready') is True
                and (state.get('scene') or {}).get('world_sha256') == snapshot.get('world_sha256')
                and (state.get('navigation') or {}).get('registration_id') == snapshot.get('sortie_id')
                and coordinate_epoch(state) == snapshot.get('coordinate_epoch'))


def status_text(command, state):
    if not command:
        return '等待指令'
    parent = task_runtime_view(command)
    if parent:
        result = parent.get('result')
        if result:
            goal = {'satisfied': '全部要求已核验', 'partial': '部分完成，仍有未完成要求', 'unmet': '用户目标未完成'}.get(result.get('goal_status'), '目标状态未知')
            return '父任务：'+goal+(' · 指令失败，请查看详情' if command.get('status') == 'failed' else '')
        stage = parent.get('current_stage') or {}
        phase = (parent.get('goal_state') or {}).get('phase')
        names = {'search': '寻找目标', 'close_photo': '靠近拍照', 'report': '环境报告',
                 'photo': '拍摄与原图复核', 'relative_hover': '目标相对悬停', 'relative_land': '目标相对降落',
                 'return_home_land': '返航降落', 'hold': '保持悬停'}
        phases = {'need_object_frame': '补充车头方向证据', 'observing': '补充观察', 'ready': '终点路径已验证，等待执行',
                  'verification': '已到达，正在新图复核', 'need_relation_review': '正在核对实际前脸与车前关系',
                  'landing_ready': '支撑面与下降通道已验证', 'landing': '正在执行目标相对降落'}
        return '{} · {} · {}'.format(stage.get('stage_id', '任务解析'), names.get(stage.get('kind'), '任务分解'),
                                     phases.get(phase, parent.get('reasoning_mode', '等待阶段证据')))
    mission = state.get('mission') or {}
    if mission.get('status') == 'running' and mission.get('action') in ('return_home', 'land'):
        return '正在返航降落'
    if mission.get('status') == 'running' and mission.get('action') == 'takeoff':
        return '起飞与导航初始化'
    events = command.get('search_events') or []
    progress = events[-1].get('result', {}) if events else {}
    if command.get('status') == 'failed':
        return '任务失败，查看安全状态'
    phase = progress.get('search_phase'); status = progress.get('search_status')
    if status == 'budget_exhausted': return '搜索预算已用尽'
    if status == 'no_reachable_view': return '没有可验证的观察路径，正在收尾'
    if phase == 'awaiting_review': return '正在比较多视角图像'
    if phase == 'confirming' and progress.get('target_focus_state') == 'approaching':
        return '目标已确认，正在接近并调整拍摄位置'
    if phase == 'confirming' and progress.get('target_focus_state') == 'acquiring_depth':
        return '发现目标，正在小步接近以获取可靠深度'
    if phase == 'confirming' and progress.get('target_focus_state') == 'reacquiring':
        return '目标暂时不可见，正在围绕已有线索重新观察'
    if phase == 'confirming': return '发现疑似目标，正在规划复核视角'
    if status == 'target_found':
        confirmed = (progress.get('target_confirmation_mode') == 'multiview' and phase == 'confirmed'
                     and any(e.get('outcome') == 'confirmed' and isinstance(e.get('owner'), list)
                             and len(e['owner']) == 2 and e['owner'][1] == command.get('id')
                             for e in command.get('target_confirmation_events', [])))
        return '多视角确认完成' if confirmed else '发现目标（没有多视角确认记录）'
    if status == 'budget_exhausted': return '搜索预算已用尽'
    if mission.get('status') == 'running': return '正在前往观察点 / 执行飞行任务'
    if status == 'searching': return '正在自主探索'
    if command.get('status') == 'succeeded': return '指令完成，请查看搜索与飞行结果'
    return '任务处理中'


def task_runtime_view(command):
    """Reject stale/foreign parent data, including a previous command's result."""
    if not isinstance(command, dict):
        return None
    value = command.get('task_runtime')
    if not isinstance(value, dict) or value.get('schema_version') != 1:
        return None
    owner = value.get('owner')
    plan = value.get('plan')
    if plan is not None and not isinstance(plan, dict):
        return None
    if (not isinstance(owner, list) or len(owner) != 2 or owner[1] != command.get('id')
            or value.get('original_instruction') != command.get('prompt')
            or plan and (plan.get('owner') != owner or plan.get('original_instruction') != command.get('prompt'))):
        return None
    return value


def goal_prediction(command, snapshot):
    """Read-only projection of current program-verified terminal candidates."""
    parent = task_runtime_view(command)
    if (not parent or not snapshot or parent.get('result') or command.get('status') != 'running'):
        return None
    goal = parent.get('goal_state') or {}
    if goal.get('phase') not in ('ready', 'verification', 'route_verification', 'need_relation_review', 'authorize_landing', 'landing_ready'):
        return None
    prepared = goal.get('prepared') or {}
    ctx = prepared.get('goal_context') or {}
    reference = next((r for r in parent.get('target_memory', {}).values() if r.get('target_id') == ctx.get('target_id')), None)
    if (not reference or any(reference.get(k) != snapshot.get(k) for k in ('world_sha256', 'sortie_id', 'coordinate_epoch'))
            or reference.get('map_status') != 'valid'):
        return None
    latest = snapshot['observations'][-1] if snapshot.get('observations') else {}
    candidates = []
    for c in prepared.get('candidates', []):
        hard = c.get('hard_constraints') or {}
        evidence = c.get('evidence_refs') or []
        action = ctx.get('terminal_action')
        required = ('identity', 'permission_volume', 'path')
        required += (('static_rotation_clearance',) if action in ('relative_turn', 'face_target') else
                     ('object_relation',) if action == 'pass_landmark' else ('object_relation', 'target_visibility'))
        if (any(hard.get(k) is not True for k in required)
                or not c.get('path') or c.get('coordinate_epoch') != snapshot.get('coordinate_epoch')):
            continue
        # After arrival retain only the executed path as a read-only historical
        # plan. New target observations must never revive its alternatives.
        arrived = goal.get('phase') != 'ready'
        if arrived and c.get('id') != prepared.get('selected_id'):
            continue
        if not arrived and (c.get('target_revision') != reference.get('target_revision')
                or not any(e.get('capture_id') == latest.get('capture_id') and
                           e.get('image_sha256') == latest.get('image_sha256') for e in evidence)):
            continue
        label = {'relative_turn': '机体相对转向', 'face_target': '面对目标', 'pass_landmark': '经过地标参考面'}.get(action, '目标相对终点')
        candidates.append({k: v for k, v in {**c, 'direction': label, 'path_verified': True,
            'target_yaw_enu_rad': c.get('yaw_enu_rad')}.items() if k != 'execution_ticket'})
    if not candidates:
        return None
    return {'planning_mode': 'goal_pose', 'ranked_candidates': candidates, 'selected_id': prepared.get('selected_id'),
            'batch_id': (prepared.get('context') or {}).get('batch_id'), 'owner': parent['owner'],
            'selector': 'program', 'phase': goal['phase']}


def task_runtime_text(command):
    parent = task_runtime_view(command)
    if not parent:
        return '当前任务使用原搜索流程；暂无父任务阶段数据。'
    stage = parent.get('current_stage') or {}
    lines = ['原任务：'+parent['original_instruction'],
             '当前阶段：{} / {} · 修订 {}'.format(stage.get('stage_id', '结束'), stage.get('kind', '—'), parent.get('stage_revision')),
             '推理模式：'+str(parent.get('reasoning_mode', '—'))]
    need = parent.get('observation_need') or {}
    if need:lines.append('当前信息需求：{} · {}'.format(need.get('kind'), '、'.join(need.get('missing_information') or [])))
    for ref, target in parent.get('target_memory', {}).items():
        lines.append('目标 {}：{} / {} · 版本 {} · {} · 朝向 {}'.format(ref, target.get('label', ''), target.get('target_id'),
            target.get('target_revision'), target.get('visibility', target.get('map_status')), '有依据' if target.get('orientation') else '未知'))
    goal = parent.get('goal_state') or {}
    if goal:lines.append('终点规划阶段：'+str(goal.get('phase')))
    result = parent.get('result')
    if result:
        lines.extend(['流程结果：'+str(result.get('process_status')), '用户目标：'+str(result.get('goal_status')),
                      '飞行结果：'+str(result.get('flight_status'))])
        for v in result.get('unmet_requirements') or []:
            lines.append('未完成：{} · {}'.format(v.get('kind'), v.get('reason_code')))
    return '\n'.join(lines)
