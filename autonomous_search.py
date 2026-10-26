"""Bounded, turn-owned adaptive visual search. Geometry stays in the executor."""
import copy
import math
import time

from stereo_localization import quaternion_rotation, vector


AUTONOMY_PROMPT = """
自主搜索任务：使用者说“找到前方的油桶，报告周围情况”等，已经授权完成目标所需的仿真起飞、
变高、横移、推进、绕障与采图，不需要逐步确认，不要求使用者提供坐标或路线。
“前方”默认任务开始时机头方向，先用begin_target_search(query=目标名称)建立任务。
每次按工具返回的next_tool推进。地面起始观察高度由begin_target_search(initial_altitude=3)保存，
调用observe_target_search_view即可自动起飞并采图；不需要重复填写起飞任务或向用户确认。
地面时自主起飞，static_3d初始建议3米（可根据场景在1～10米选择）；空中不重复起飞。
自主搜索优先observe_target_search_view后report_scene；前者一次完成已授权飞行及新鲜采图，减少模型往返。
纯观察使用capture_scene。未发现是正常搜索结果，不是视觉接口故障。
未发现时调用next_target_search_view(strategy='rise'|'left'|'right'|'forward'|'auto',reason=依据)，
工具会检查候选路径，返回可执行goto；随后调用observe_target_search_view直接执行已检查点并采图，
不需转抄坐标，再report_scene，持续循环，不把决策交回用户。
根据遮挡和视野选择策略，例如高墙挡住视线可升高或横移，不把像素/粗略投影直接当航点。
最多8份有效观察，15分钟，距离任务起点12米以内；不要重复观察同一位置或无限询问。
report_scene的目标label与query使用一致名称；看到目标即可报告视觉发现，不要求先获得精确双目位置。
目标位置无法测定时仍可用实际照片描述，明确位置不确定，不猜坐标。找到后报告周围可见情况，
完成或预算结束默认return_home再land；使用者明确要求悬停/不降落时begin_target_search(end_behavior='hold')。
只有定位、图像接口或执行失败才终止新的搜索移动。无安全候选点时结束并安全降落，不反复询问路线。
纯“看一下当前画面/描述眼前情况”仍只观察，不启动自主搜索。
"""


class AdaptiveSearch:
    def __init__(self, client, clock=time.monotonic):
        self.client, self.clock = client, clock
        self.sessions = {}

    def begin(self, owner, query, end_behavior="land", initial_altitude=3.):
        from search_tools import _owner
        owner = _owner(owner)
        if owner in self.sessions:
            raise ValueError("本指令已有自主任务，请继续现有任务")
        state = self.client.state()
        if (state.get("simulation_only") is not True or state.get("ready") is not True
                or state.get("scene", {}).get("verified") is not True
                or state.get("navigation", {}).get("method") != "static_3d"):
            raise ValueError("自主目标搜索需要就绪的已验证静态三维仿真场景")
        query = query.strip()
        if not 1 <= len(query) <= 80 or end_behavior not in ("land", "hold"):
            raise ValueError("无效搜索目标或结束方式")
        if type(initial_altitude) not in (int, float) or not math.isfinite(initial_altitude) or not 1 <= initial_altitude <= 10:
            raise ValueError("初始观察高度必须在1～10米")
        pair = self.client.capture_stereo()
        if pair.get("status") != "ok":
            raise ValueError("无法读取搜索起始相机姿态")
        body = pair["body_pose_enu"]
        if body.get("frame_id") not in ("map", "local_enu") or body.get("body_reference") != "base_link":
            raise ValueError("搜索方向需要本地ENU机身姿态")
        r = quaternion_rotation(body["orientation_xyzw"])
        yaw = math.atan2(r[1][0], r[0][0])
        self.sessions[owner] = dict(query=query, origin=vector(state["position_enu_m"], 3, "origin"),
            forward=[math.cos(yaw), math.sin(yaw)], started=self.clock(), status="searching",
            observations=[], pending=({'action': 'takeoff', 'altitude': initial_altitude} if state.get('armed') is False else None),
            arrived=False, epoch=None, epoch_bound=False, scene=state["scene"].get("world_sha256"),
            end_behavior=end_behavior, seen_captures=set())
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
        return dict(status="ok", search_status=s["status"], query=s["query"],
                    observations=copy.deepcopy(memory), pending=copy.deepcopy(s["pending"]),
                    elapsed_seconds=round(self.clock()-s["started"], 1), max_observations=8,
                    max_duration_seconds=900, radius_m=12, end_behavior=s["end_behavior"],
                    failure_reason=s.get("failure_reason"),
                    next_tool=("land_drone" if s['status'] == 'failed' else
                               "return_home_then_land" if s['status'] != 'searching' and s['end_behavior'] == 'land' else
                               "final_report" if s['status'] != 'searching' else
                               "observe_target_search_view" if s['arrived'] or s['pending'] or not s['observations'] else
                               "next_target_search_view"),
                    note="发现依据为实际图像报告；米制定位另按双目质量报告")

    def validate_state(self, owner, state):
        from search_tools import _navigation_permission, coordinate_epoch
        s = self.sessions[owner]
        _navigation_permission(state)
        if state["scene"].get("world_sha256") != s["scene"]:
            raise ValueError("搜索场景已改变")
        if s["epoch_bound"] and coordinate_epoch(state) != s["epoch"]:
            raise ValueError("搜索坐标系已重置")

    def next_view(self, owner, strategy, reason):
        s = self.sessions[owner]
        if self.summary(owner)["search_status"] != "searching":
            return self.summary(owner)
        if s["pending"]:
            return {**self.summary(owner), **({'goto': copy.deepcopy(s['pending'])} if s['pending']['action'] == 'goto' else {})}
        if not s["observations"]:
            return {**self.summary(owner), 'note': '先按next_tool完成当前观察，不需要用户确认'}
        state = self.client.state()
        self.validate_state(owner, state)
        position = vector(state["position_enu_m"], 3, "position")
        f = s["forward"]
        right = [f[1], -f[0]]
        candidates = {
            "rise": [[*position[:2], min(10., position[2]+d)] for d in (2., 4.)],
            "left": [[position[0]-right[0]*d, position[1]-right[1]*d, position[2]] for d in (2., 4.)],
            "right": [[position[0]+right[0]*d, position[1]+right[1]*d, position[2]] for d in (2., 4.)],
            "forward": [[position[0]+f[0]*d, position[1]+f[1]*d, position[2]] for d in (2., 4.)]}
        if strategy not in (*candidates, "auto"):
            raise ValueError("未知观察策略")
        order = ([strategy] if strategy != "auto" else []) + [k for k in candidates if k != strategy]
        rejected = []
        for kind in order:
            for point in candidates[kind]:
                if math.dist(point[:2], s["origin"][:2]) > 12 or not 1 <= point[2] <= 10:
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

    def check_mission(self, owner, steps):
        s = self.sessions.get(owner)
        if s is None:
            return
        if not isinstance(steps, list) or not steps:
            raise ValueError("需要非空飞行步骤")
        actions = [v.get("action") for v in steps]
        if actions == ["land"]:
            return
        status = self.summary(owner)["search_status"]
        if status != "searching":
            if status != "failed" and actions in (["return_home"], ["return_home", "land"]):
                return
            raise ValueError("搜索已结束，只允许结束飞行")
        if not s["observations"] and actions == ["takeoff"]:
            if s['pending'] and s['pending'].get('action') == 'takeoff' and abs(steps[0].get('altitude', 0)-s['pending']['altitude']) <= 1e-6:
                return
            raise ValueError('请执行已选择的初始起飞高度')
        state = self.client.state()
        self.validate_state(owner, state)
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
        if (frame.get("owner") != owner or frame.get("delivered") is not True
                or report.get("status") != "ok" or report.get("capture_id") != frame.get("capture_id")
                or snapshot.get("coordinate_epoch") != coordinate_epoch(state)):
            raise ValueError("搜索观察需要本指令的新鲜图像、已保存报告和一致的坐标系")
        if frame["capture_id"] in s["seen_captures"]:
            raise ValueError("同一张照片不能重复推进搜索")
        position = vector(snapshot["body_pose_enu"]["position_m"], 3, "capture position")
        mission = state.get("mission") or {}
        if mission.get("status") == "running" or math.dist(position, state["position_enu_m"]) > .5:
            raise ValueError("等待飞行完成后在当前观察点采图")
        pending = s["pending"]
        expected = ([*s['origin'][:2], pending['altitude']] if pending and pending['action'] == 'takeoff'
                    else [pending['x'], pending['y'], pending['altitude']] if pending else position)
        if pending and math.dist(position, expected) > .4:
            raise ValueError("照片不在本次计划的观察点")
        if s["observations"] and not pending:
            raise ValueError("请先选择新的观察点，不重复当前位置")
        self.summary(owner)
        if s["status"] != "searching":
            raise ValueError("搜索预算已结束")
        s["epoch"] = coordinate_epoch(state)
        s["epoch_bound"] = True
        s["seen_captures"].add(frame["capture_id"])
        s["observations"].append(dict(capture_id=frame["capture_id"], position_enu_m=position,
                                      summary=report["summary"], targets=copy.deepcopy(report.get("targets", []))))
        s["pending"] = None
        s['arrived'] = False
        if any(t.get("label") == s["query"] for t in report.get("targets", [])):
            s["status"] = "target_found"
        return self.summary(owner)

    def guidance(self, owner):
        if owner not in self.sessions:
            return None
        summary = self.summary(owner)
        state = self.client.state()
        finished = summary["search_status"] != "searching"
        complete = finished and (summary["end_behavior"] == "hold" or
                                 (state.get("armed") is False and state.get("landed_state") == 1))
        return summary, complete

    def fail(self, owner, reason):
        if owner in self.sessions:
            self.sessions[owner]["status"] = "failed"
            self.sessions[owner]["pending"] = None
            self.sessions[owner]["failure_reason"] = str(reason)[:400]
