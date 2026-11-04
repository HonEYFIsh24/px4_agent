"""Bounded, turn-owned adaptive visual search. Geometry stays in the executor."""
from mission import MAX_ALTITUDE
import copy
import math
import os
import json
import time
import threading
import uuid
from pathlib import Path

from stereo_localization import quaternion_rotation, vector


class PrematureSearchCompletion(ValueError):
    """An unsubmitted return plan conflicts with an unfinished owned search."""


AUTONOMY_PROMPT = """
自主搜索任务：使用者说“找到前方的油桶，报告周围情况”等，已经授权完成目标所需的仿真起飞、
变高、横移、推进、绕障与采图，不需要逐步确认，不要求使用者提供坐标或路线。
“前方”默认任务开始时机头方向，先用begin_target_search(query=目标名称)建立任务。
每次按工具返回的next_tool推进。地面起始观察高度由begin_target_search(initial_altitude=3)保存，
调用observe_target_search_view即可自动起飞并采图；不需要重复填写起飞任务或向用户确认。
地面时自主起飞，static_3d初始建议3米（可根据场景在1～20米选择）；空中不重复起飞。
自主搜索优先observe_target_search_view后report_scene；前者一次完成已授权飞行及新鲜采图，减少模型往返。
纯观察使用capture_scene。未发现是正常搜索结果，不是视觉接口故障。
未发现时调用next_target_search_view(strategy='rise'|'left'|'right'|'forward'|'auto',reason=依据)，
工具会检查候选路径，返回可执行goto；随后调用observe_target_search_view直接执行已检查点并采图，
不需转抄坐标，再report_scene，持续循环，不把决策交回用户。
若工具返回search_mode=nbv_geometry或nbv_semantic，程序按地图自动评价三维候选，strategy不再指定固定动作；
只使用返回的候选ID和observe_target_search_view，不自行填写坐标或绕过候选票据。
若nbv_selector=vlm且next_tool=select_target_search_view，先比较工具提供的candidates，
结合真实照片、任务关系、遮挡和评分，调用select_target_search_view(batch_id,candidate_id,reason)。
只从本批提供的ID中选择，不请求用户决定；选定后按next_tool执行观察。
若nbv_auto_prepare=true，report_scene已自动准备下一候选，不再调用next_target_search_view，
直接按next_tool选择或执行；不重复准备同一批次。
候选均已通过路径预检查，但只是视场预测；不把预测当成已看见目标，也不把候选方向编成目标位置。
候选按ID排列，不代表优先级；结合任务与图像遮挡判断，不将地面覆盖增益当成识别概率。
若target_confirmation_mode=multiview，首次匹配目标仅是候选，按next_tool换位采图，
随后调用review_target_candidate复核实际两张图及源框裁图。不能用标签匹配或双目成功代替确认。
复核可否定旧候选或说明未确认，再继续搜索；未检出不能证明目标不存在，不向用户索取路线。
根据遮挡和视野选择策略，例如高墙挡住视线可升高或横移，不把像素/粗略投影直接当航点。
最多8份有效观察，15分钟，距离任务起点12米以内；不要重复观察同一位置或无限询问。
report_scene的目标label与query使用一致名称；看到目标即可报告视觉发现，不要求先获得精确双目位置。
目标位置无法测定时仍可用实际照片描述，明确位置不确定，不猜坐标。找到后报告周围可见情况，
完成或预算结束由程序执行返航降落，模型不调用通用收尾工具；使用者明确要求悬停/不降落时begin_target_search(end_behavior='hold')。
只有定位、图像接口或执行失败才终止新的搜索移动。无安全候选点时结束并安全降落，不反复询问路线。
纯“看一下当前画面/描述眼前情况”仍只观察，不启动自主搜索。
"""


class AdaptiveSearch:
    def __init__(self, client, clock=time.monotonic, search_mode=None, semantic_maps=None, nbv_selector=None):
        self.client, self.clock = client, clock
        self.sessions = {}
        self.binding_lock = threading.RLock()
        self.search_mode = search_mode or os.getenv('UAV_SEARCH_MODE', 'fixed')
        if self.search_mode not in ('fixed', 'nbv_geometry', 'nbv_semantic'):
            raise ValueError('UAV_SEARCH_MODE must be fixed, nbv_geometry or nbv_semantic')
        self.nbv_selector = nbv_selector or os.getenv('UAV_NBV_SELECTOR', 'deterministic')
        if self.nbv_selector not in ('deterministic', 'vlm') or (self.search_mode == 'fixed' and self.nbv_selector == 'vlm'):
            raise ValueError('UAV_NBV_SELECTOR must be deterministic, or vlm with an NBV search mode')
        self.candidate_pool_mode = os.getenv('UAV_NBV_CANDIDATE_POOL', 'ranked')
        if self.candidate_pool_mode not in ('ranked', 'diverse'):
            raise ValueError('UAV_NBV_CANDIDATE_POOL must be ranked or diverse')
        self.heading_mode = os.getenv('UAV_NBV_HEADING_MODE', 'fixed')
        if self.heading_mode not in ('fixed', 'discrete') or (self.heading_mode == 'discrete' and self.search_mode == 'fixed'):
            raise ValueError('UAV_NBV_HEADING_MODE must be fixed, or discrete with an NBV search mode')
        auto_prepare = os.getenv('UAV_NBV_AUTO_PREPARE', '0')
        if auto_prepare not in ('0', '1') or (auto_prepare == '1' and self.search_mode == 'fixed'):
            raise ValueError('UAV_NBV_AUTO_PREPARE requires 0, or 1 with an NBV search mode')
        self.auto_prepare = auto_prepare == '1'
        self.target_confirmation_mode = os.getenv('UAV_TARGET_CONFIRMATION', 'single_frame')
        if self.target_confirmation_mode not in ('single_frame', 'multiview') or (
                self.target_confirmation_mode == 'multiview' and self.search_mode == 'fixed'):
            raise ValueError('UAV_TARGET_CONFIRMATION must be single_frame, or multiview with an NBV search mode')
        self.semantic_maps = semantic_maps

    def begin(self, owner, query, end_behavior="land", initial_altitude=3.):
        with self.binding_lock:
            return self._begin(owner, query, end_behavior, initial_altitude)

    def _begin(self, owner, query, end_behavior="land", initial_altitude=3.):
        from search_tools import _owner, _require_coordinate_evidence
        owner = _owner(owner)
        if owner in self.sessions:
            raise ValueError("本指令已有自主任务，请继续现有任务")
        state = self.client.state()
        if (state.get("simulation_only") is not True or state.get("ready") is not True
                or state.get("scene", {}).get("verified") is not True
                or state.get("navigation", {}).get("method") != "static_3d"):
            raise ValueError("自主目标搜索需要就绪的已验证静态三维仿真场景")
        if self.search_mode != 'fixed' and state.get('navigation', {}).get('viewpoint_execution_supported') is not True:
            raise ValueError('NBV搜索需要支持候选票据执行的新版后端，请先重启系统')
        if self.heading_mode == 'discrete' and state.get('navigation', {}).get('viewpoint_yaw_supported') is not True:
            raise ValueError('航向NBV需要后端明确启用实验性航向和航向候选')
        query = query.strip()
        if not 1 <= len(query) <= 80 or end_behavior not in ("land", "hold"):
            raise ValueError("无效搜索目标或结束方式")
        if type(initial_altitude) not in (int, float) or not math.isfinite(initial_altitude) or not 1 <= initial_altitude <= MAX_ALTITUDE:
            raise ValueError("初始观察高度必须在1～20米")
        initial_epoch = _require_coordinate_evidence(state)
        pair = self.client.capture_stereo()
        if pair.get("status") != "ok":
            raise ValueError("无法读取搜索起始相机姿态")
        after_capture_state = self.client.state()
        if (initial_epoch is not None and 'external_vision' not in pair
                or 'external_vision' in pair and _require_coordinate_evidence(pair) != initial_epoch
                or _require_coordinate_evidence(after_capture_state) != initial_epoch
                or (after_capture_state.get('scene') or {}).get('world_sha256') != state['scene'].get('world_sha256')):
            raise ValueError('搜索初始化期间坐标系已改变')
        body = pair["body_pose_enu"]
        if body.get("frame_id") not in ("map", "local_enu") or body.get("body_reference") != "base_link":
            raise ValueError("搜索方向需要本地ENU机身姿态")
        r = quaternion_rotation(body["orientation_xyzw"])
        yaw = math.atan2(r[1][0], r[0][0])
        ground_position = vector(state["position_enu_m"], 3, "origin")
        grounded = state.get('armed') is False
        initial_binding = None
        if not grounded:
            from search_tools import _navigation_permission
            _navigation_permission(state)
            _navigation_permission(after_capture_state)
            registration = (state.get('navigation') or {}).get('registration_id')
            external = state.get('external_vision') or {}
            ekf = external.get('ekf') or {}
            mag = ekf.get('mag_alignment') or {}
            if (not isinstance(registration, str) or not registration
                    or pair.get('navigation_registration_id') != registration
                    or (after_capture_state.get('navigation') or {}).get('registration_id') != registration
                    or (state.get('mission') or {}).get('status') != 'succeeded'
                    or (after_capture_state.get('mission') or {}).get('id') != (state.get('mission') or {}).get('id')
                    or (after_capture_state.get('mission') or {}).get('status') != 'succeeded'
                    or type(after_capture_state.get('stamp_sim_s')) not in (int, float)
                    or not math.isfinite(after_capture_state['stamp_sim_s'])
                    or after_capture_state['stamp_sim_s'] < pair.get('stamp_sim_s', float('inf'))
                    or type(body.get('stamp_sim_s')) not in (int, float)
                    or type(pair.get('stamp_sim_s')) not in (int, float)
                    or not math.isfinite(body['stamp_sim_s']) or not math.isfinite(pair['stamp_sim_s'])
                    or type(body.get('age_seconds')) not in (int, float) or not 0 <= body['age_seconds'] <= 1
                    or (external.get('required') is True and (external.get('healthy') is not True
                        or external.get('heading_alignment_pending') is not False
                        or ekf.get('heading_good_for_control') is not True
                        or mag.get('aligned_in_flight') is not True or mag.get('yaw_aligned') is not True
                        or mag.get('in_air') is not True))):
                raise ValueError('空中搜索需要对齐完成和已配准地图的新鲜机体位姿')
            bind_position = vector(body.get('position_m'), 3, 'binding position')
            if math.dist(bind_position, ground_position) > .5:
                raise ValueError('搜索绑定位姿与实时位置不一致')
            initial_binding = {'mission_id': (state.get('mission') or {}).get('id'),
                'position_enu_m': bind_position, 'stamp_sim_s': after_capture_state['stamp_sim_s'],
                'registration_id': registration, 'ground_position_enu_m': copy.deepcopy(ground_position),
                'ground_offset_m': math.dist(bind_position[:2], ground_position[:2])}
        self.sessions[owner] = dict(query=query, origin=None if grounded else bind_position,
            boundary_origin=copy.deepcopy(ground_position), preflight_epoch=copy.deepcopy(initial_epoch),
            preflight_ekf=copy.deepcopy((state.get('external_vision') or {}).get('ekf') or {}),
            preflight_pose_stamp=body.get('stamp_sim_s'), binding=initial_binding,
            forward=[math.cos(yaw), math.sin(yaw)], started=self.clock(), status="searching",
            observations=[], pending=({'action': 'takeoff', 'altitude': initial_altitude} if grounded else None),
            arrived=False, epoch=None if grounded else copy.deepcopy(initial_epoch), epoch_bound=False, scene=state["scene"].get("world_sha256"),
            end_behavior=end_behavior, seen_captures=set())
        if not grounded:
            final_state = self.client.state()
            stamp = final_state.get('stamp_sim_s')
            final_ev = final_state.get('external_vision') or {}
            final_ekf = final_ev.get('ekf') or {}
            final_mag = final_ekf.get('mag_alignment') or {}
            if (_require_coordinate_evidence(final_state) != initial_epoch
                    or (final_state.get('navigation') or {}).get('registration_id') != registration
                    or (final_state.get('scene') or {}).get('world_sha256') != state['scene'].get('world_sha256')
                    or type(stamp) not in (int, float) or not math.isfinite(stamp)
                    or stamp < initial_binding['stamp_sim_s']
                    or (external.get('required') is True and (final_ev.get('healthy') is not True
                        or final_ev.get('heading_alignment_pending') is not False
                        or final_ekf.get('heading_good_for_control') is not True
                        or final_mag.get('aligned_in_flight') is not True
                        or final_mag.get('yaw_aligned') is not True or final_mag.get('in_air') is not True))):
                del self.sessions[owner]
                raise ValueError('空中搜索绑定期间坐标或地图身份已改变')
            initial_binding['stamp_sim_s'] = stamp
            self.sessions[owner]['epoch_bound'] = True
        self.sessions[owner]['search_mode'] = self.search_mode
        self.sessions[owner]['nbv_selector'] = self.nbv_selector
        self.sessions[owner]['candidate_pool_mode'] = self.candidate_pool_mode
        self.sessions[owner]['heading_mode'] = self.heading_mode
        self.sessions[owner]['nbv_auto_prepare'] = self.auto_prepare
        self.sessions[owner].update(target_confirmation_mode=self.target_confirmation_mode,
                                    search_phase='exploring', confirmation=None)
        # Old completed turns are not needed by the live search.
        if len(self.sessions) > 100:
            del self.sessions[next(iter(self.sessions))]
        return self.summary(owner)

    def summary(self, owner):
        s = self.sessions[owner]
        if s["status"] == "searching" and (self.clock()-s["started"] >= 900 or len(s["observations"]) >= 8):
            s["status"] = "budget_exhausted"
        memory = [dict(capture_id=o['capture_id'], position_enu_m=o['position_enu_m'], summary=o['summary'],
                       targets=[{'label': t.get('label'), 'surroundings': t.get('surroundings')} for t in o['targets']])
                  for o in s['observations']]
        pending = copy.deepcopy(s['pending'])
        if self.is_nbv(owner) and pending and pending['action'] == 'goto':
            pending = {'action': 'goto', 'candidate_id': s['nbv_selected']['id']}
        result = dict(status="ok", search_status=s["status"], query=s["query"], search_mode=s['search_mode'],
                    nbv_selector=s.get('nbv_selector', 'deterministic'),
                    candidate_pool_mode=s.get('candidate_pool_mode', 'ranked'),
                    heading_mode=s.get('heading_mode', 'fixed'),
                    nbv_auto_prepare=s.get('nbv_auto_prepare', False),
                    target_confirmation_mode=s.get('target_confirmation_mode', 'single_frame'),
                    search_phase=s.get('search_phase', 'exploring'), confirmation=copy.deepcopy(s.get('confirmation')),
                    observations=copy.deepcopy(memory), pending=pending,
                    elapsed_seconds=round(self.clock()-s["started"], 1), max_observations=8,
                    max_duration_seconds=900, radius_m=12, end_behavior=s["end_behavior"],
                    coordinate_binding=copy.deepcopy(s.get('binding')),
                    coordinate_epoch=copy.deepcopy(s.get('epoch')) if s['epoch_bound'] else None,
                    ground_reference_enu_m=copy.deepcopy(s['boundary_origin']),
                    failure_reason=s.get("failure_reason"), termination=copy.deepcopy(s.get("termination")),
                    next_tool=("land_drone" if s['status'] == 'failed' else
                               "review_target_candidate" if s.get('search_phase') == 'awaiting_review' else
                               "return_home_then_land" if s['status'] != 'searching' and s['end_behavior'] == 'land' else
                               "final_report" if s['status'] != 'searching' else
                               "observe_target_search_view" if s['arrived'] or s['pending'] or not s['observations'] else
                               "select_target_search_view" if s.get('nbv_awaiting_selection') else
                               "next_target_search_view"),
                    note="发现依据为实际图像报告；米制定位另按双目质量报告")
        if s.get('nbv_awaiting_selection') and s['status'] == 'searching':
            result.update(batch_id=s['nbv_batch_id'],
                          candidates=copy.deepcopy(s['nbv_choices']),
                          selection_expires_in_seconds=max(0., round(60.-(self.clock()-s['nbv_prepared_at']), 1)))
        if s.get('activity_closed'):
            result['next_tool'] = 'final_report'
        return result

    def validate_state(self, owner, state):
        from search_tools import _navigation_permission, coordinate_epoch
        s = self.sessions[owner]
        if (s.get('spatial_map_identity') or {}).get('status') == 'invalid':
            raise ValueError('空间语义地图坐标身份失效，禁止新的搜索移动')
        if s.get('activity_closed'):
            raise ValueError('搜索活动已结束，禁止继续观察或飞行')
        _navigation_permission(state)
        if state["scene"].get("world_sha256") != s["scene"]:
            raise ValueError("搜索场景已改变")
        if s["epoch_bound"] and coordinate_epoch(state) != s["epoch"]:
            raise ValueError("搜索坐标系已重置")
        if s.get('binding') and state.get('navigation', {}).get('registration_id') != s['binding']['registration_id']:
            raise ValueError('搜索导航地图配准已改变')

    def next_view(self, owner, strategy, reason, preparation_source='next_target_search_view'):
        s = self.sessions[owner]
        if s.get('search_phase') == 'awaiting_review':
            return self.summary(owner)
        if self.summary(owner)["search_status"] != "searching":
            return self.summary(owner)
        if s["pending"]:
            if self.is_nbv(owner):
                return self.summary(owner)
            return {**self.summary(owner), **({'goto': copy.deepcopy(s['pending'])} if s['pending']['action'] == 'goto' else {})}
        if not s["observations"]:
            return {**self.summary(owner), 'note': '先按next_tool完成当前观察，不需要用户确认'}
        state = self.client.state()
        self.validate_state(owner, state)
        if s.get('nbv_awaiting_selection'):
            self.validate_nbv_batch(owner, state)
            return self.summary(owner)
        if self.is_nbv(owner):
            return self.next_nbv(owner, reason, preparation_source)
        position = vector(state["position_enu_m"], 3, "position")
        f = s["forward"]
        right = [f[1], -f[0]]
        candidates = {
            "rise": [[*position[:2], min(MAX_ALTITUDE, position[2]+d)] for d in (2., 4.)],
            "left": [[position[0]-right[0]*d, position[1]-right[1]*d, position[2]] for d in (2., 4.)],
            "right": [[position[0]+right[0]*d, position[1]+right[1]*d, position[2]] for d in (2., 4.)],
            "forward": [[position[0]+f[0]*d, position[1]+f[1]*d, position[2]] for d in (2., 4.)]}
        if strategy not in (*candidates, "auto"):
            raise ValueError("未知观察策略")
        order = ([strategy] if strategy != "auto" else []) + [k for k in candidates if k != strategy]
        rejected = []
        for kind in order:
            for point in candidates[kind]:
                if math.dist(point[:2], s["boundary_origin"][:2]) > 12 or not 1 <= point[2] <= MAX_ALTITUDE:
                    continue
                if any(math.dist(point, o["position_enu_m"]) < .75 for o in s["observations"]):
                    continue
                step = dict(action="goto", x=point[0], y=point[1], altitude=point[2])
                preview = self.client.request("/navigation/preview", {"goal_enu_m": point})
                if preview.get("reachable") is not True:
                    rejected.append({"strategy": kind, "reason": preview.get("reason")})
                    continue
                s["pending"] = step
                s['arrived'] = False
                return {**self.summary(owner), "goto": copy.deepcopy(step), "strategy": kind,
                        "reason": reason[:300], "rejected_candidates": rejected,
                        "next": "observe_target_search_view，然后report_scene"}
        s["status"] = "no_reachable_view"
        return {**self.summary(owner), "rejected_candidates": rejected}

    def is_nbv(self, owner):
        return (self.sessions.get(owner) or {}).get('search_mode', 'fixed') != 'fixed'

    def next_nbv(self, owner, reason, preparation_source='next_target_search_view'):
        from dataclasses import asdict
        from search_diagnostics import stage
        from viewpoint_planner import ViewpointPlanner, ViewpointConfig, map_revision
        from viewpoint_geometry import RemoteViewpointEvaluator
        s = self.sessions[owner]
        if self.semantic_maps is None or owner not in self.semantic_maps.maps or not s.get('last_stereo_pair_id'):
            raise ValueError('NBV需要本指令已保存并成功进入空间地图的真实观察')
        semantic_map = self.semantic_maps.maps[owner]
        confirmation = None
        if s.get('search_phase') == 'confirming':
            from target_confirmation import confirmation_goal
            active = s['confirmation']
            with stage('confirmation_goal', {'owner': owner, 'candidate': active}) as diagnostic:
                confirmation = confirmation_goal(semantic_map, active['target_id'], active['source_capture_id'])
                confirmation['avoid_observer_positions'] = [list(o['position_enu_m']) for o in s['observations']]
                diagnostic['result'] = confirmation
        revision, batch = map_revision(semantic_map), uuid.uuid4().hex
        context = {'owner': list(owner), 'map_revision': revision, 'batch_id': batch}
        evaluator = RemoteViewpointEvaluator(self.client, s['last_stereo_pair_id'], context,
            confirmation['probe_points_enu_m'] if confirmation else None)
        planner = ViewpointPlanner(ViewpointConfig(mode=s['search_mode'][4:],
            candidate_pool_mode=s.get('candidate_pool_mode', 'ranked'), heading_mode=s.get('heading_mode', 'fixed')))
        s['nbv_bound'] = True
        with stage('planner_evaluate', {'context': context, 'config': asdict(planner.config),
                'semantic_map': semantic_map.snapshot(), 'confirmation_goal': confirmation}) as diagnostic:
            prediction = planner.evaluate(semantic_map, evaluator, confirmation)
            diagnostic['result'] = prediction
        with stage('planner_select', {'context': context}) as diagnostic:
            selected = planner.select(semantic_map)
            diagnostic['result'] = selected
        public = copy.deepcopy(prediction)
        public.update(owner=list(owner), batch_id=batch,
                      source_capture_id=s['observations'][-1]['capture_id'], search_mode=s['search_mode'],
                      selector=s.get('nbv_selector', 'deterministic'), numeric_best_id=prediction['selected_id'],
                      prepared_by=preparation_source)
        for candidate in public['ranked_candidates']:
            candidate.pop('execution_ticket', None)
        s['nbv_public'] = public
        if selected is None:
            self.publish_nbv(s, 'nbv_prediction')
            if confirmation is not None:
                self.confirmation_event(owner, 'view_unavailable', reason='no safe distinct confirmation view with predicted target visibility')
                s.update(search_phase='exploring', confirmation=None)
                return self.next_nbv(owner, reason, preparation_source)
            s['status'] = 'coverage_exhausted' if prediction['status'] == 'no_useful_view' else 'no_reachable_view'
            return {**self.summary(owner), 'nbv_status': prediction['status']}
        s['nbv_batch_id'], s['nbv_revision'] = batch, revision
        if s.get('nbv_selector') == 'vlm':
            by_id = {c['id']: c for c in prediction['ranked_candidates']}
            offered = [by_id[identifier] for identifier in prediction['candidate_pool_ids']]
            if any(not c.get('execution_ticket') for c in offered):
                raise ValueError('NBV后端未签发候选执行票据')
            s['nbv_planner'], s['nbv_prepared_at'] = planner, self.clock()
            s['nbv_choices'] = sorted([{**{k: copy.deepcopy(c[k]) for k in
                ('id', 'direction', 'coverage_gain_m2', 'semantic_gain_m2', 'motion_seconds',
                 'target_visibility_score', 'parallax_score')},
                'altitude_enu_m': c['position_enu_m'][2], 'path_length_m': c['path']['path_length_m']}
                for c in offered], key=lambda c: c['id'])
            public.update(selected_id=None, selection_status='awaiting_vlm',
                          offered_candidate_ids=[c['id'] for c in offered])
            s['nbv_awaiting_selection'] = True
            self.publish_nbv(s, 'nbv_candidates')
            return self.summary(owner)
        self.queue_nbv(s, selected)
        self.publish_nbv(s, 'nbv_prediction')
        return {**self.summary(owner), 'candidate_id': selected['id'], 'reason': reason[:300],
                'nbv_status': prediction['status'], 'generated_candidates': prediction['generated_count'],
                'path_evaluations': prediction['path_evaluation_count'],
                'selected_metrics': {k: selected[k] for k in ('coverage_gain_m2', 'semantic_gain_m2', 'motion_seconds', 'score', 'score_terms')},
                'next': 'observe_target_search_view，然后report_scene'}

    def publish_nbv(self, session, event):
        public = session['nbv_public']
        if hasattr(self.client, 'record'):
            self.client.record(event, public)
        directory = os.getenv('UAV_CAPTURE_EVIDENCE_DIR')
        if directory:
            identifier = public['source_capture_id']
            if Path(identifier).name != identifier or identifier in ('.', '..'):
                raise ValueError('invalid NBV evidence basename')
            target = Path(directory)/(identifier+'.'+session['search_mode']+'.json')
            temporary = target.with_suffix('.tmp')
            temporary.write_text(json.dumps(public, ensure_ascii=False, indent=2, allow_nan=False))
            temporary.replace(target)

    @staticmethod
    def queue_nbv(s, selected):
        if not selected.get('execution_ticket'):
            raise ValueError('NBV后端未签发候选执行票据')
        s['nbv_selected'] = selected
        point = selected['position_enu_m']
        s['pending'] = {'action': 'goto', 'x': point[0], 'y': point[1], 'altitude': point[2]}
        s['arrived'] = False

    def validate_nbv_batch(self, owner, state=None):
        from viewpoint_planner import map_revision
        s = self.sessions[owner]
        if self.clock()-s['nbv_prepared_at'] > 60.:
            raise ValueError('NBV候选选择已超时，禁止执行旧候选')
        if map_revision(self.semantic_maps.maps[owner]) != s['nbv_revision']:
            raise ValueError('NBV候选对应的地图已经更新，禁止选择旧候选')
        if state is not None:
            if (state.get('navigation', {}).get('registration_id') != s['nbv_public']['identity']['sortie_id']
                    or math.dist(vector(state['position_enu_m'], 3, 'selection source'),
                                 s['nbv_public']['reference']['start_enu_m']) > .35):
                raise ValueError('NBV候选源位置或架次配准已经改变')

    def select_nbv(self, owner, batch_id, candidate_id, reason):
        s = self.sessions[owner]
        if (s.get('nbv_selector') != 'vlm' or self.summary(owner)['search_status'] != 'searching'
                or not s.get('nbv_awaiting_selection') or s['pending'] is not None):
            raise ValueError('当前没有等待模型选择的NBV批次')
        state = self.client.state()
        self.validate_state(owner, state)
        self.validate_nbv_batch(owner, state)
        if batch_id != s['nbv_batch_id'] or candidate_id not in [c['id'] for c in s['nbv_choices']]:
            raise ValueError('候选ID不属于本任务当前已提供的批次')
        selected = s['nbv_planner'].select(self.semantic_maps.maps[owner], candidate_id)
        self.queue_nbv(s, selected)
        s['nbv_awaiting_selection'] = False
        s['nbv_public'].update(selected_id=candidate_id, selection_status='selected_by_vlm',
                               selection_reason=reason[:300])
        self.publish_nbv(s, 'nbv_prediction')
        return {**self.summary(owner), 'candidate_id': candidate_id,
                'batch_id': batch_id, 'selection_reason': reason[:300]}

    def execute_pending(self, owner, description='自主搜索：换位并采图'):
        s = self.sessions[owner]
        step = copy.deepcopy(s['pending'])
        if step is None or s.get('nbv_awaiting_selection'):
            raise ValueError('请先选择当前批次的候选观察点')
        self.check_mission(owner, [step])
        if self.is_nbv(owner) and step['action'] == 'goto':
            from viewpoint_planner import map_revision
            if map_revision(self.semantic_maps.maps[owner]) != s['nbv_revision']:
                raise ValueError('NBV候选对应的地图已经更新，请重新评估')
            selected = s['nbv_selected']
            result = self.client.execute_viewpoint(owner, s['nbv_batch_id'], s['nbv_revision'],
                selected['id'], selected['execution_ticket'], description)
        else:
            result = self.client.execute([step], description)
        if (result.get('mission') or {}).get('status') == 'succeeded':
            if step['action'] == 'takeoff' and not s['epoch_bound']:
                self.bind_after_takeoff(owner, result)
            else:
                self.validate_completed_flight(owner, result)
            s['arrived'] = True
        return result

    def bind_after_takeoff(self, owner, result):
        """Bind once after the executor's completed, registered takeoff."""
        try:
            with self.binding_lock:
                return self._bind_after_takeoff(owner, result)
        except Exception as exc:
            self.fail(owner, str(exc))
            if hasattr(self.client, 'record'):
                self.client.record('execute_mission', {'status': 'error', 'error': str(exc)})
            raise

    def _bind_after_takeoff(self, owner, result):
        from search_tools import _navigation_permission, _require_coordinate_evidence
        s = self.sessions[owner]
        if (s['status'] != 'searching' or s['epoch_bound'] or s.get('binding') or s['observations'] or s.get('spatial_map_identity')
                or s.get('nbv_bound') or (self.semantic_maps is not None and owner in self.semantic_maps.maps)):
            raise ValueError('搜索坐标身份已经绑定或已有观察、地图或票据')
        mission = result.get('mission') or {}
        if (mission.get('status') != 'succeeded' or not mission.get('id')
                or not s.get('pending') or s['pending'].get('action') != 'takeoff'
                or not mission.get('steps') or mission['steps'] != [s['pending']]):
            raise ValueError('搜索绑定需要本次已完成的独立起飞任务')
        state = self.client.state()
        _navigation_permission(state)
        if state.get('scene', {}).get('world_sha256') != s['scene']:
            raise ValueError('搜索场景已改变')
        epoch = _require_coordinate_evidence(state)
        if epoch != _require_coordinate_evidence(result):
            raise ValueError('起飞结果与实时坐标身份不一致')
        before = s['preflight_epoch']
        external = state.get('external_vision') or {}
        if external.get('required') is True:
            ekf = external.get('ekf') or {}
            mag = ekf.get('mag_alignment') or {}
            preflight_ekf = s['preflight_ekf']
            velocity = ekf.get('local_velocity_reset_counters')
            if (external.get('healthy') is not True or external.get('used_for_flight_control') is not True
                    or external.get('heading_alignment_pending') is not False
                    or ekf.get('heading_good_for_control') is not True
                    or mag.get('aligned_in_flight') is not True or mag.get('yaw_aligned') is not True
                    or mag.get('in_air') is not True or type(ekf.get('primary_instance')) is not int
                    or ekf.get('primary_instance') != preflight_ekf.get('primary_instance')
                    or not isinstance(velocity, dict) or set(velocity) != {'vxy', 'vz'}
                    or any(type(v) is not int or not 0 <= v <= 255 for v in velocity.values())
                    or velocity != preflight_ekf.get('local_velocity_reset_counters')):
                raise ValueError('起飞磁航向对齐尚未可靠完成')
            if before != epoch:
                event = external.get('heading_alignment_event') or {}
                old, new = (before or {}).get('ekf_counters') or {}, (epoch or {}).get('ekf_counters') or {}
                if (not before or before.get('vio_epoch') != epoch.get('vio_epoch')
                        or old.get('xy') != new.get('xy') or old.get('z') != new.get('z')
                        or event.get('old_heading_reset_counter') != old.get('heading')
                        or event.get('new_heading_reset_counter') != new.get('heading')
                        or event.get('adapter_epoch') != epoch.get('vio_epoch')
                        or event.get('xy_reset_counter') != new.get('xy')
                        or event.get('z_reset_counter') != new.get('z')
                        or event.get('primary_instance') != ekf.get('primary_instance')
                        or event.get('primary_instance') != preflight_ekf.get('primary_instance')
                        or event.get('velocity_reset_counters') != ekf.get('local_velocity_reset_counters')
                        or event.get('velocity_reset_counters') != preflight_ekf.get('local_velocity_reset_counters')
                        or event.get('permitted_context') != 'takeoff_before_horizontal_navigation'
                        or event.get('source') != 'actual_owned_px4_uorb'
                        or type(old.get('heading')) is not int or type(new.get('heading')) is not int
                        or (new['heading'] - old['heading']) % 256 != 1):
                    raise ValueError('搜索坐标系已重置，起飞坐标变化缺少有效磁对齐证据')
        registration = (state.get('navigation') or {}).get('registration_id')
        if not isinstance(registration, str) or not registration:
            raise ValueError('起飞后缺少已配准的导航地图身份')
        pair = self.client.capture_stereo()
        body = pair.get('body_pose_enu') or {}
        if (pair.get('status') != 'ok' or _require_coordinate_evidence(pair) != epoch
                or pair.get('navigation_registration_id') != registration
                or body.get('frame_id') not in ('map', 'local_enu') or body.get('body_reference') != 'base_link'
                or type(body.get('stamp_sim_s')) not in (int, float) or not math.isfinite(body['stamp_sim_s'])
                or type(pair.get('stamp_sim_s')) not in (int, float) or not math.isfinite(pair['stamp_sim_s'])
                or type(body.get('age_seconds')) not in (int, float) or not 0 <= body['age_seconds'] <= 1):
            raise ValueError('搜索绑定需要同一坐标和地图身份的新鲜机体位姿')
        position = vector(body['position_m'], 3, 'binding position')
        if before != epoch:
            event_time = (external.get('heading_alignment_event') or {}).get('reset_source_stamp_sim_s')
            if (type(event_time) not in (int, float) or not math.isfinite(event_time)
                    or event_time > body['stamp_sim_s'] or (type(s.get('preflight_pose_stamp')) in (int, float)
                    and event_time <= s['preflight_pose_stamp'])):
                raise ValueError('起飞磁对齐时间与绑定位姿不一致')
        if math.dist(position, vector(state['position_enu_m'], 3, 'current position')) > .5:
            raise ValueError('搜索绑定位姿与实时位置不一致')
        if math.dist(position[:2], s['boundary_origin'][:2]) > .4:
            raise ValueError('起飞位置偏离地面参考，不能保证原搜索范围')
        current = self.client.state()
        _navigation_permission(current)
        current_ev = current.get('external_vision') or {}
        current_ekf = current_ev.get('ekf') or {}
        current_mag = current_ekf.get('mag_alignment') or {}
        if (_require_coordinate_evidence(current) != epoch or (current.get('navigation') or {}).get('registration_id') != registration
                or (current.get('scene') or {}).get('world_sha256') != s['scene']
                or (current.get('mission') or {}).get('id') != mission['id']
                or (current.get('mission') or {}).get('status') != 'succeeded'
                or type(current.get('stamp_sim_s')) not in (int, float)
                or not math.isfinite(current['stamp_sim_s']) or current['stamp_sim_s'] < pair['stamp_sim_s']
                or current_ev.get('required') != external.get('required') or current_ev.get('healthy') is False
                or (external.get('required') is True and (current_ev.get('healthy') is not True
                    or current_ev.get('heading_alignment_pending') is not False
                    or current_ekf.get('heading_good_for_control') is not True
                    or current_ekf.get('primary_instance') != ekf.get('primary_instance')
                    or current_ekf.get('local_velocity_reset_counters') != velocity
                    or current_mag.get('aligned_in_flight') is not True or current_mag.get('yaw_aligned') is not True
                    or current_mag.get('in_air') is not True))):
            raise ValueError('搜索绑定期间坐标或地图身份已改变')
        rotation = quaternion_rotation(body['orientation_xyzw'])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
        s.update(origin=position, forward=[math.cos(yaw), math.sin(yaw)], epoch=copy.deepcopy(epoch), epoch_bound=False,
                 binding={'mission_id': mission['id'], 'position_enu_m': position, 'stamp_sim_s': current['stamp_sim_s'],
                          'registration_id': registration, 'ground_position_enu_m': copy.deepcopy(s['boundary_origin']),
                          'ground_offset_m': math.dist(position[:2], s['boundary_origin'][:2])})
        # The barrier is sampled after the provisional identity exists, while
        # epoch_bound remains false so no observation can be committed yet.
        confirmed = self.client.state()
        stamp = confirmed.get('stamp_sim_s')
        confirmed_ev = confirmed.get('external_vision') or {}
        confirmed_ekf = confirmed_ev.get('ekf') or {}
        confirmed_mag = confirmed_ekf.get('mag_alignment') or {}
        if (_require_coordinate_evidence(confirmed) != epoch
                or (confirmed.get('navigation') or {}).get('registration_id') != registration
                or (confirmed.get('scene') or {}).get('world_sha256') != s['scene']
                or (confirmed.get('mission') or {}).get('id') != mission['id']
                or (confirmed.get('mission') or {}).get('status') != 'succeeded'
                or confirmed_ev.get('required') != external.get('required')
                or type(stamp) not in (int, float) or not math.isfinite(stamp)
                or stamp < s['binding']['stamp_sim_s']
                or (external.get('required') is True and (confirmed_ev.get('healthy') is not True
                    or confirmed_ev.get('heading_alignment_pending') is not False
                    or confirmed_ekf.get('heading_good_for_control') is not True
                    or confirmed_ekf.get('primary_instance') != ekf.get('primary_instance')
                    or confirmed_ekf.get('local_velocity_reset_counters') != velocity
                    or confirmed_mag.get('aligned_in_flight') is not True
                    or confirmed_mag.get('yaw_aligned') is not True or confirmed_mag.get('in_air') is not True))):
            raise ValueError('搜索绑定完成时坐标或地图身份已改变')
        _navigation_permission(confirmed)
        s['binding']['stamp_sim_s'] = stamp
        s['epoch_bound'] = True

    def validate_completed_flight(self, owner, result):
        """Recheck identity before treating a completed movement as a search view."""
        from search_tools import _require_coordinate_evidence
        try:
            epoch = self.sessions[owner]['epoch']
            if (_require_coordinate_evidence(result) != epoch
                    or _require_coordinate_evidence(self.client.state()) != epoch):
                raise ValueError('搜索坐标系已重置')
        except Exception as exc:
            self.fail(owner, str(exc))
            if hasattr(self.client, 'record'):
                self.client.record('execute_mission', {'status': 'error', 'error': str(exc)})
            raise

    def check_mission(self, owner, steps):
        s = self.sessions.get(owner)
        if s is None:
            return
        if s.get('activity_closed'):
            raise ValueError('搜索活动已结束，禁止再次提交模型飞行')
        if not isinstance(steps, list) or not steps:
            raise ValueError("需要非空飞行步骤")
        actions = [v.get("action") for v in steps]
        status = self.summary(owner)["search_status"]
        if status != "searching":
            raise ValueError("搜索已结束，由程序收尾，禁止模型提交飞行")
        if not s["observations"] and actions == ["takeoff"]:
            from search_tools import _require_coordinate_evidence
            if _require_coordinate_evidence(self.client.state()) != s['preflight_epoch']:
                raise ValueError('搜索坐标系已重置')
            if s['pending'] and s['pending'].get('action') == 'takeoff' and abs(steps[0].get('altitude', 0)-s['pending']['altitude']) <= 1e-6:
                return
            raise ValueError('请执行已选择的初始起飞高度')
        state = self.client.state()
        self.validate_state(owner, state)
        if actions in (['land'], ['return_home'], ['return_home', 'land'], ['hold']):
            raise PrematureSearchCompletion('搜索尚未结束，模型收尾请求未提交')
        if (len(steps) != 1 or not s["pending"] or actions != ['goto']
                or set(steps[0]) != set(s['pending'])
                or any(type(steps[0][k]) not in (int, float) or not math.isfinite(steps[0][k])
                       or abs(steps[0][k]-s['pending'][k]) > 1e-6 for k in ('x', 'y', 'altitude'))):
            raise ValueError("请使用next_target_search_view返回的安全观察点，并逐点保存观察")

    def consume(self, owner, frame, report):
        from search_tools import coordinate_epoch
        s = self.sessions.get(owner)
        if s is None:
            return None
        state = self.client.state()
        self.validate_state(owner, state)
        snapshot = frame["snapshot"]
        if (not s['epoch_bound'] or frame.get("owner") != owner or frame.get("delivered") is not True
                or report.get("status") != "ok" or report.get("capture_id") != frame.get("capture_id")
                or snapshot.get("coordinate_epoch") != coordinate_epoch(state)
                or (s.get('binding') and (snapshot.get('navigation_registration_id') != s['binding']['registration_id']
                    or snapshot.get('stamp_sim_s', -1) <= s['binding']['stamp_sim_s']))):
            raise ValueError("搜索观察需要本指令的新鲜图像、已保存报告和一致的坐标系")
        if frame["capture_id"] in s["seen_captures"]:
            raise ValueError("同一张照片不能重复推进搜索")
        position = vector(snapshot["body_pose_enu"]["position_m"], 3, "capture position")
        mission = state.get("mission") or {}
        if mission.get("status") == "running" or math.dist(position, state["position_enu_m"]) > .5:
            raise ValueError("等待飞行完成后在当前观察点采图")
        pending = s["pending"]
        expected = ([*s['boundary_origin'][:2], pending['altitude']] if pending and pending['action'] == 'takeoff'
                    else [pending['x'], pending['y'], pending['altitude']] if pending else position)
        if pending and math.dist(position, expected) > .4:
            raise ValueError("照片不在本次计划的观察点")
        if s["observations"] and not pending:
            raise ValueError("请先选择新的观察点，不重复当前位置")
        self.summary(owner)
        if s["status"] != "searching":
            raise ValueError("搜索预算已结束")
        s["seen_captures"].add(frame["capture_id"])
        s["observations"].append(dict(capture_id=frame["capture_id"], position_enu_m=position,
                                      summary=report["summary"], targets=copy.deepcopy(report.get("targets", []))))
        s["pending"] = None
        s['arrived'] = False
        if (s.get('target_confirmation_mode', 'single_frame') == 'single_frame'
                and any(t.get("label") == s["query"] for t in report.get("targets", []))):
            s["status"] = "target_found"
        return self.summary(owner)

    def confirmation_event(self, owner, outcome, **fields):
        s = self.sessions[owner]
        event = {'owner': list(owner), 'outcome': outcome, 'query': s['query'],
                 'candidate': copy.deepcopy(s.get('confirmation')), **fields}
        if hasattr(self.client, 'record'):
            self.client.record('target_confirmation', event)
        return event

    def after_map_observation(self, owner, semantic_map, frame, report):
        """Lifecycle only; all boxes, rays and measurements stay in SemanticMap."""
        from target_confirmation import observations_for
        s = self.sessions[owner]
        if s.get('target_confirmation_mode') != 'multiview':
            return
        if semantic_map.owner != owner or semantic_map.status != 'valid':
            raise ValueError('Confirmation needs the valid owned semantic map')
        identifier = frame['capture_id']
        if s.get('search_phase') == 'confirming':
            active = s['confirmation']
            if identifier == active['source_capture_id']:
                raise ValueError('Confirmation requires a second actual capture')
            active.update(current_capture_id=identifier,
                          current_target_ids=[l.id for l, _ in observations_for(semantic_map, s['query'], identifier)])
            s['search_phase'] = 'awaiting_review'
            self.confirmation_event(owner, 'review_required')
        elif s.get('search_phase', 'exploring') == 'exploring':
            candidates = observations_for(semantic_map, s['query'], identifier)
            if candidates and s['status'] == 'searching':
                s['confirmation'] = {'target_id': candidates[0][0].id, 'source_capture_id': identifier}
                s['search_phase'] = 'confirming'
                self.confirmation_event(owner, 'candidate')

    def review_confirmation(self, owner, source_capture_id, current_capture_id, current_target_id,
                            source_supported, current_supported, same_object, reason,
                            source_uncertain=False, current_uncertain=False):
        from target_confirmation import (CONFIRMATION_GEOMETRY_POLICY, association_allows_confirmation,
                                         observation_for, observations_for, pair_geometry)
        from semantic_map import point_in_box
        s = self.sessions[owner]
        active = s.get('confirmation') or {}
        errors = []
        if s.get('search_phase') != 'awaiting_review' or s['status'] == 'failed':
            errors.append('review_phase_invalid')
        for field, value in (('source_capture_id', source_capture_id), ('current_capture_id', current_capture_id)):
            if value != active.get(field):
                errors.append(field + '_mismatch')
        for field, value in (('source_supported', source_supported), ('current_supported', current_supported),
                             ('same_object', same_object), ('source_uncertain', source_uncertain),
                             ('current_uncertain', current_uncertain)):
            if type(value) is not bool:
                errors.append(field + '_not_boolean')
        if source_supported is True and source_uncertain is True:
            errors.append('source_support_uncertainty_conflict')
        if current_supported is True and current_uncertain is True:
            errors.append('current_support_uncertainty_conflict')
        if not isinstance(reason, str) or not 0 < len(reason) <= 1000:
            errors.append('reason_invalid')
        if errors:
            raise ValueError('Confirmation review invalid: ' + ', '.join(errors))
        self.validate_state(owner, self.client.state())
        semantic_map = self.semantic_maps.maps[owner]
        if semantic_map.status != 'valid':
            raise ValueError('Cannot confirm with an invalid spatial map')
        source_landmark, source = observation_for(semantic_map, active['target_id'], source_capture_id)
        matched = []
        for landmark, observation in observations_for(semantic_map, s['query'], current_capture_id):
            geometry = pair_geometry(source_landmark, source, landmark, observation, CONFIRMATION_GEOMETRY_POLICY)
            if geometry['compatible']:
                matched.append((landmark, geometry))
        current_observations = observations_for(semantic_map, s['query'], current_capture_id)
        # Association must be unique in both views. Two similar source objects
        # cannot silently collapse onto one current detection.
        if len(matched) == 1:
            landmark, _ = matched[0]
            current = next(o for l, o in current_observations if l.id == landmark.id)
            competitors = [l.id for l, o in observations_for(semantic_map, s['query'], source_capture_id)
                           if pair_geometry(l, o, landmark, current, CONFIRMATION_GEOMETRY_POLICY)['compatible']]
            if competitors != [source_landmark.id]:
                matched = []
        if source['position_enu_m'] is not None and sum(
                point_in_box(source['position_enu_m'], observation) for _, observation in current_observations) > 1:
            matched = []
        if current_target_id and current_target_id not in active['current_target_ids']:
            raise ValueError('Review selected a target not reported in the current image')
        geometry = matched[0][1] if len(matched) == 1 and matched[0][0].id == current_target_id else {
            'compatible': False, 'association_basis': 'none', 'reason': 'no unique same-object geometric correspondence'}
        outcome = ('rejected' if not source_supported and not source_uncertain else
                   'confirmed' if source_supported and current_supported and same_object
                   and association_allows_confirmation(geometry) else 'unconfirmed')
        s['last_review_outcome'] = outcome
        self.confirmation_event(owner, outcome, current_target_id=current_target_id,
            geometry_policy=CONFIRMATION_GEOMETRY_POLICY,
            source_supported=source_supported, current_supported=current_supported, same_object=same_object,
            source_uncertain=source_uncertain, current_uncertain=current_uncertain,
            geometry=geometry, reason=reason)
        if outcome == 'confirmed':
            s.update(status='target_found', search_phase='confirmed')
        else:
            # Refutes/unconfirms this hypothesis, not the existence of all targets
            # in a geometrically observed region. Raw reports are retained.
            s.update(search_phase='exploring', confirmation=None)
            # A failed old hypothesis must not discard a stronger observation
            # just saved in the current frame. Start a new candidate from that
            # existing evidence, still requiring a later distinct view.
            self.summary(owner)  # Enforce observation/time budget before retaining any candidate.
            attempts = active.get('review_attempts', 0) + 1
            if (s['status'] == 'searching' and current_supported
                    and current_target_id in active['current_target_ids']
                    and (not source_supported and not source_uncertain or attempts < 2)):
                s['confirmation'] = {'target_id': current_target_id, 'source_capture_id': current_capture_id,
                                     'review_attempts': 0 if not source_supported and not source_uncertain else attempts}
                s['search_phase'] = 'confirming'
                self.confirmation_event(owner, 'candidate', reason='current supported observation becomes a new candidate; not confirmation')
            elif (s['status'] == 'searching' and (source_supported or source_uncertain)
                  and not current_supported and attempts < 2):
                s['confirmation'] = {'target_id': active['target_id'], 'source_capture_id': source_capture_id,
                                     'review_attempts': attempts}
                s['search_phase'] = 'confirming'
                self.confirmation_event(owner, 'candidate', reason='source supported or visually uncertain; one more distinct view before resuming exploration')
        return {**self.summary(owner), 'confirmation_outcome': outcome, 'geometry': geometry,
                'reviewed_source_capture_id': source_capture_id, 'reviewed_current_capture_id': current_capture_id,
                'review_reason': reason}

    def close_activity(self, owner, source, reason):
        """Internal lifecycle entry; source is supplied by trusted application code."""
        s = self.sessions.get(owner)
        if s is None:
            return
        if not s.get('termination') or s['termination']['source'] != source:
            s['termination'] = {'source': source, 'reason': str(reason)[:500], 'flight_status': 'pending'}
        s['activity_closed'] = True
        s['pending'] = None
        s['nbv_awaiting_selection'] = False
        if source != 'normal_completion':
            self.fail(owner, reason)
        if hasattr(self.client, 'record'):
            self.client.record('search_termination', {**self.summary(owner), 'termination': copy.deepcopy(s['termination'])})

    def finish_normal(self, owner):
        s = self.sessions[owner]
        if s.get('finish_result') is not None:
            return copy.deepcopy(s['finish_result'])
        progress = self.summary(owner)
        if (progress['search_status'] not in ('target_found', 'budget_exhausted', 'coverage_exhausted', 'no_reachable_view')
                or s.get('search_phase') == 'awaiting_review'):
            raise ValueError('搜索尚未满足确定性收尾条件')
        if s['end_behavior'] == 'hold':
            self.close_activity(owner, 'normal_completion', progress['search_status'])
            return None
        self.validate_state(owner, self.client.state())
        self.close_activity(owner, 'normal_completion', progress['search_status'])
        result = self.client.execute([{'action': 'return_home'}, {'action': 'land'}], 'Search state machine: '+progress['search_status'])
        if hasattr(self.client, 'record') and not hasattr(self.client, 'application'):
            self.client.record('execute_mission', result)
        mission = result.get('mission') or {}
        if mission.get('status') != 'succeeded' or result.get('armed') is not False or result.get('landed_state') != 1:
            raise ValueError('搜索收尾缺少成功落地上锁证据')
        s['termination']['flight_status'] = 'grounded_disarmed'
        s['finish_result'] = copy.deepcopy(result)
        if hasattr(self.client, 'record'):
            self.client.record('search_termination', {**self.summary(owner), 'termination': copy.deepcopy(s['termination'])})
        return result

    def guidance(self, owner):
        if owner not in self.sessions:
            return None
        summary = self.summary(owner)
        state = self.client.state()
        finished = summary["search_status"] != "searching" and self.sessions[owner].get('search_phase') != 'awaiting_review'
        complete = finished and (summary["end_behavior"] == "hold" or
                                 (state.get("armed") is False and state.get("landed_state") == 1))
        s = self.sessions[owner]
        if complete and self.is_nbv(owner) and s.get('nbv_bound') and not s.get('nbv_released'):
            self.client.request('/navigation/search-release', {'owner': list(owner)})
            s['nbv_released'] = True
        return summary, complete

    def fail(self, owner, reason):
        if owner in self.sessions:
            self.sessions[owner]['activity_closed'] = True
            self.sessions[owner]["status"] = "failed"
            self.sessions[owner]["pending"] = None
            self.sessions[owner]["failure_reason"] = str(reason)[:400]
            self.sessions[owner]['end_behavior'] = 'land'
            self.sessions[owner]['search_phase'] = 'failed'
