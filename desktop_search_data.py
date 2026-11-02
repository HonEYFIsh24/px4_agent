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
            if (len(owner) != 2 or owner[1] != command['id'] or snapshot.get('status') != 'valid'
                    or snapshot.get('world_sha256') != event.get('world_sha256')
                    or snapshot.get('sortie_id') != event.get('sortie_id')
                    or not snapshot.get('observations') or snapshot['observations'][-1].get('capture_id') != identifier
                    or not isinstance((reports[identifier].get('camera') or {}).get('image_sha256'), str)
                    or len((reports[identifier].get('camera') or {}).get('image_sha256')) != 64
                    or (snapshot['observations'][-1].get('image_sha256') or {}).get('left') != (reports[identifier].get('camera') or {}).get('image_sha256')
                    or snapshot.get('grid', {}).get('rows') != 48 or snapshot.get('grid', {}).get('cols') != 48
                    or len(snapshot.get('cells', [])) != 2304):
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
    if phase == 'awaiting_review': return '正在比较多视角图像'
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
