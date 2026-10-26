"""Turn-owned read-only search tools and report bookkeeping; no flight dispatch.

The VLM moves using the existing supervised execute_mission tool. This module
only returns deterministic observation points and accepts real saved reports.
"""
import copy
import math
import threading
import time
import uuid

from coverage_search import CoverageSearch, CoverageSearchError, navigation_goal_permission, plan_search
from stereo_localization import calibration_from_camera_info, finite, quaternion_rotation, vector


SEARCH_PROMPT = """
以下规则仅针对给定区域的网格搜索；自然语言寻找目标使用begin_target_search，不要求用户提供区域。
使用者给定ENU区域搜索时调用plan_region_search；query填目标名称，与后续report_scene的目标label保持一致。
先get_drone_state。区域搜索需要已验证ENU导航地图；lidar_grid先起飞到固定3.5米，static_3d可在1～10米选高，等执行器完成后再规划。
plan_region_search只生成观察点，不执行飞行。不能把scene世界坐标bounds直接作为ENU边界。
bounds表示要寻找目标的区域，不是飞机必须停留的区域。前俯摄像头需要在区域后方观察；工具返回的车辆观察点可能在目标区域外，但必须在已验证的ENU飞行许可内，不能自行改回cell中心。
依次search_observation_status(view='next')取得当前点，用execute_mission的goto到返回的point，再capture_scene并report_scene。
每点必须有成功到达、真实新图像和已保存报告；保存报告后工具自动记录观察进度，然后再取得下一点。
不要一次飞过多个未观察点；不要自行跳过blocked点，也不要把目标双目坐标当goto目标。
最多8个观察点；只有query一致、两个不同位置的有效双目测量且均值及配置的不确定度包络能确认在请求区域内，工具才提前target_found。区域外或边界不确定目标仍可报告，不能据此提前结束。只汇报实际采样比例和测量不确定度，不声称完整无遮挡覆盖。
搜索blocked/observation_failed/budget_exhausted时停止新的移动，说明错误并完成使用者已授权的最后安全降落。
未发现仅说明本次采样视野未确认，不能说整个区域不存在目标。
"""


def runtime_owner(runtime):
    thread = (runtime.config.get("configurable") or {}).get("thread_id")
    for message in reversed(runtime.state.get("messages", [])):
        role = message.get("role") if isinstance(message, dict) else getattr(message, "type", None)
        if role in ("human", "user"):
            identifier = message.get("id") if isinstance(message, dict) else getattr(message, "id", None)
            if thread and identifier:
                return str(thread), str(identifier)
            break
    raise CoverageSearchError("无法确认本次搜索的指令所有者")


def _owner(owner):
    if not isinstance(owner, tuple) or len(owner) != 2 or any(not isinstance(v, str) or not v or len(v) > 200 for v in owner):
        raise CoverageSearchError("Invalid search turn owner")
    return owner


def _navigation_permission(state):
    scene = state.get("scene") or {}
    nav = state.get("navigation") or {}
    sensed_map = nav.get("map", nav)
    if nav.get("method") == "static_3d":
        if (state.get("simulation_only") is not True or state.get("ready") is not True
                or state.get("armed") is not True or scene.get("verified") is not True
                or nav.get("enabled") is not True or nav.get("active") is not True
                or sensed_map.get("source") != "verified_static_collision_map_3d"
                or sensed_map.get("map_registered") is not True):
            raise CoverageSearchError("搜索需要已起飞、定位有效且完成本地坐标配准的已知静态三维地图")
        bounds = vector(sensed_map.get("bounds_xy_enu_m"),4,"registered static-map ENU permission bounds")
        permission = navigation_goal_permission(bounds,sensed_map.get("obstacle_inflation_m"),
                                                sensed_map.get("visible_clearance_m"))
        altitude = vector(state.get("position_enu_m"),3,"current ENU position")[2]
        if not 1 <= altitude <= 10:
            raise CoverageSearchError("搜索观察高度必须位于本地 1～10 米范围内")
        _require_coordinate_evidence(state)
        return permission["usable_goal_bounds_enu_m"],altitude,permission
    if (state.get("simulation_only") is not True or state.get("ready") is not True or state.get("armed") is not True
            or scene.get("verified") is not True
            or nav.get("enabled") is not True or nav.get("active") is not True
            or sensed_map.get("source") != "horizontal_lidar_aligned_enu"
            or sensed_map.get("fresh") is not True or sensed_map.get("scan_count", 0) < 5):
        raise CoverageSearchError("搜索需要起飞后已建立的新鲜ENU激光地图；请先完成已授权的3.5米起飞，不使用场景世界坐标代替地图")
    bounds = vector(sensed_map.get("bounds_xy_enu_m"), 4, "sensed ENU permission bounds")
    permission = navigation_goal_permission(bounds, sensed_map.get("obstacle_inflation_m"),
                                            sensed_map.get("visible_clearance_m"))
    altitude = finite(scene.get("navigation", {}).get("flight_altitude_m"), "scene navigation altitude")
    if abs(altitude-3.5) > 1e-6:
        raise CoverageSearchError("Only the current fixed 3.5m navigation/search profile is supported")
    map_altitude = finite(sensed_map.get("map_altitude_enu_m"), "map altitude")
    if abs(map_altitude-altitude) > .35:
        raise CoverageSearchError("LiDAR map altitude does not match the search plane")
    _require_coordinate_evidence(state)
    return permission["usable_goal_bounds_enu_m"], altitude, permission


def coordinate_epoch(state):
    """Canonical identity from complete telemetry or its external_vision envelope.

    Missing or malformed observer fields stay absent; this never substitutes an
    assumed zero counter. A search must stop if this identity later changes.
    """
    if not isinstance(state, dict):
        return None
    external_vision = state.get("external_vision") if "external_vision" in state else state
    if not isinstance(external_vision, dict):
        return None
    adapter = external_vision.get("adapter") or {}
    ekf = external_vision.get("ekf") or {}
    if not isinstance(adapter, dict):
        adapter = {}
    if not isinstance(ekf, dict):
        ekf = {}
    counters = ekf.get("local_position_reset_counters")
    if (not isinstance(counters, dict) or set(counters) != {"xy", "z", "heading"}
            or any(type(value) is not int or not 0 <= value <= 255 for value in counters.values())):
        counters = None
    epoch = adapter.get("epoch")
    if type(epoch) is not int or epoch < 0:
        epoch = None
    if epoch is None and counters is None:
        # Missing observer evidence is explicit; appearing/disappearing evidence
        # changes this identity and aborts an existing search, even in GPS mode.
        return None
    return {"vio_epoch": copy.deepcopy(epoch),
            "ekf_counters": {key: counters[key] for key in ("xy", "z", "heading")} if counters is not None else None}


def _require_coordinate_evidence(state):
    external_vision = state.get("external_vision") or {}
    adapter = external_vision.get("adapter") or {}
    epoch = coordinate_epoch(state)
    if ((external_vision.get("required") is True or adapter.get("mode") == "fuse")
            and (epoch is None or epoch["vio_epoch"] is None or epoch["ekf_counters"] is None)):
        raise CoverageSearchError("实际EV融合搜索必须具有完整VIO epoch和PX4 xy/z/heading坐标重置计数证据")
    return epoch


class SearchManager:
    def __init__(self, client, clock=time.monotonic, max_turns=8):
        self.client, self.clock, self.max_turns = client, clock, max_turns
        self.lock = threading.RLock()
        self.sessions = {}
        from autonomous_search import AdaptiveSearch
        self.adaptive = AdaptiveSearch(client, clock)

    def _audit(self, name, result):
        # Never hold the search lock while calling the application audit lock.
        return self.client.record(name, result) if hasattr(self.client, "record") else result

    def plan(self, owner, query, bounds, spacing, max_observations):
        owner = _owner(owner)
        if not isinstance(query, str) or not 0 < len(query.strip()) <= 80:
            raise CoverageSearchError("query must be the requested target's short canonical label")
        if isinstance(max_observations, bool) or not isinstance(max_observations, int) or not 1 <= max_observations <= 8:
            raise CoverageSearchError("Agent search supports 1..8 observation points")
        with self.lock:
            if owner in self.sessions or owner in self.adaptive.sessions:
                raise CoverageSearchError("本指令已有搜索计划，不能自动重新规划或覆盖失败记录")
        state = self.client.state()
        allowed, altitude, permission = _navigation_permission(state)
        mission = state.get("mission") or {}
        if mission.get("status") == "running":
            raise CoverageSearchError("等待当前飞行步骤完成后再规划搜索")
        home = vector(state.get("home_enu_m"), 3, "recorded ground home ENU")
        pair = self.client.capture_stereo()
        if pair.get("status") != "ok":
            raise CoverageSearchError("Search planning requires actual stereo calibration")
        calibration_from_camera_info(pair["left"]["camera_intrinsics"], pair["right"]["camera_intrinsics"], .12)
        body = pair.get("body_pose_enu") or {}
        if body.get("frame_id") not in ("map", "local_enu") or body.get("body_reference") != "base_link":
            raise CoverageSearchError("Search heading requires the actual same-frame ENU base_link pose")
        rotation = quaternion_rotation(body["orientation_xyzw"])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
        generated = plan_search(bounds, spacing, altitude, pair["left"]["camera_intrinsics"],
                                max_observations=max_observations, allowed_bounds_enu_m=allowed,
                                ground_enu_z=home[2], max_duration_wall_s=900., planning_yaw_rad=yaw,
                                navigation_permission=permission)
        with self.lock:
            if owner in self.sessions:
                raise CoverageSearchError("Concurrent search planning is forbidden")
            if len(self.sessions) >= self.max_turns:
                expired = next((key for key, value in self.sessions.items()
                                if value["search"].summary()["status"] != "running"), None)
                if expired is None:
                    raise CoverageSearchError("Search session capacity is exhausted")
                del self.sessions[expired]
            identifier = uuid.uuid4().hex
            self.sessions[owner] = {"id": identifier, "search": CoverageSearch(generated, target_label=query.strip(), clock=self.clock),
                                    "coordinate_epoch": coordinate_epoch(state), "scene_sha256": scene_hash(state),
                                    "last_report": None}
            result = {"status": "ok", "search_id": identifier, "query": query.strip(), "plan": generated,
                      "actual_flight": False, "actual_visual_observations": 0,
                      "note": "bounds是目标区域；车辆点按实际相机朝向/杆臂后移并严格在ENU飞行许可内；逐点到达、实际目标cell在FOV内、采图并保存报告后计数"}
        return self._audit("plan_region_search", result)

    def status(self, owner, view):
        owner = _owner(owner)
        if view not in ("next", "summary"):
            raise CoverageSearchError("view must be next or summary")
        state = self.client.state()
        with self.lock:
            session = self.sessions.get(owner)
            if session is None:
                raise CoverageSearchError("本指令尚未建立搜索计划")
            search = session["search"]
            if search.status == "running":
                mission = state.get("mission") or {}
                if mission.get("status") in ("failed", "cancelled"):
                    self._fail_locked(session, "navigation_failed", "已观察到飞行失败/中断，不能跳过该点继续搜索")
                else:
                    try:
                        epoch = _require_coordinate_evidence(state)
                    except CoverageSearchError as exc:
                        self._fail_locked(session, "coordinate_evidence_missing", str(exc))
                    else:
                        if epoch != session["coordinate_epoch"] or scene_hash(state) != session["scene_sha256"]:
                            self._fail_locked(session, "coordinate_changed", "坐标epoch或场景改变，停止使用旧搜索和目标位置")
            summary = search.summary()
            result = {"status": "ok", "search_id": session["id"], "search": summary,
                      "next_view": search.next_view() if view == "next" else None,
                      "requires_stop_new_movement": summary["status"] in ("blocked", "observation_failed", "budget_exhausted", "cancelled"),
                      "safety_landing_required_if_owned_flight": summary["status"] in ("blocked", "observation_failed", "budget_exhausted")}
        return self._audit("search_observation_status", result)

    @staticmethod
    def _fail_locked(session, category, reason):
        search = session["search"]
        if search.status == "running":
            current = search.next_view()
            navigation_failure = category.startswith("navigation")
            search.status, search.reason = ("blocked" if navigation_failure else "observation_failed"), category + ": " + reason
            if current is not None:
                search.states[current["id"]] = "blocked" if navigation_failure else "observation_failed"

    def fail(self, owner, reason):
        self.adaptive.fail(owner, reason)
        with self.lock:
            session = self.sessions.get(_owner(owner))
            if session is None:
                return None
            self._fail_locked(session, "report_failed", str(reason)[:400])
            result = {"status": "error", "error": session["search"].reason,
                      "search_id": session["id"], "search": session["search"].summary()}
        return self._audit("search_observation", result)

    def check_mission(self, owner, steps):
        """Optional existing-flight-tool gate; validates but never submits a mission.

        Pass normalized action dictionaries before execute_mission. An active
        search permits only its current sample's goto (and a short hold) until
        the report is saved. A failed search permits direct landing only.
        """
        if owner in self.adaptive.sessions:
            return self.adaptive.check_mission(owner, steps)
        with self.lock:
            session = self.sessions.get(_owner(owner))
            if session is None:
                return True
            if not isinstance(steps, list) or not steps or any(not isinstance(step, dict) for step in steps):
                raise CoverageSearchError("Search mission must contain validated action objects")
            search = session["search"]
            summary = search.summary()
            actions = [step.get("action") for step in steps]
            if actions == ["land"]:
                return True
        state = self.client.state()
        with self.lock:
            session = self.sessions.get(owner)
            if session is None:
                raise CoverageSearchError("Search ownership expired before mission validation")
            try:
                epoch = _require_coordinate_evidence(state)
                if epoch != session["coordinate_epoch"] or scene_hash(state) != session["scene_sha256"]:
                    raise CoverageSearchError("坐标epoch或场景改变，禁止使用旧搜索观察点移动")
            except CoverageSearchError as exc:
                self._fail_locked(session, "coordinate_changed", str(exc))
                raise
            search = session["search"]
            summary = search.summary()
            if summary["status"] in ("blocked", "observation_failed", "budget_exhausted", "cancelled"):
                raise CoverageSearchError("搜索已停止，禁止新的移动；仅允许已有授权的直接安全降落")
            if summary["status"] in ("sampling_complete", "target_found"):
                if actions in (["return_home"], ["return_home", "land"]):
                    return True
                raise CoverageSearchError("搜索已结束，目标米制坐标仅用于报告；不得自动接近或增加搜索路线")
            current = search.next_view()
            if current is None or actions not in (["goto"], ["goto", "hold"]):
                raise CoverageSearchError("活动搜索每阶段只允许到当前观察点，保存图像报告后才能去下一点")
            requested = [finite(steps[0].get(key), "search goto " + key) for key in ("x", "y", "altitude")]
            if any(abs(a-b) > 1e-6 for a, b in zip(requested, current["position_enu_m"])):
                raise CoverageSearchError("搜索goto必须等于当前确定性观察点，不能改用目标或场景物体坐标")
            if len(steps) == 2 and not .1 <= finite(steps[1].get("duration"), "search observation hold") <= 10:
                raise CoverageSearchError("Search observation hold must be .1..10 seconds")
            return True

    def consume_report(self, owner, frame, report):
        """Invoke only after a genuinely delivered frame's report has been saved."""
        owner = _owner(owner)
        if owner in self.adaptive.sessions:
            return self._audit("target_search_progress", self.adaptive.consume(owner, frame, report))
        with self.lock:
            if owner not in self.sessions:
                return None
        state = self.client.state()
        with self.lock:
            session = self.sessions[owner]
            search = session["search"]
            current = search.next_view()
            if current is None:
                return {"status": "ok", "search_id": session["id"], "search": search.summary()}
            try:
                if (frame.get("owner") != owner or frame.get("delivered") is not True or report.get("status") != "ok"
                        or frame.get("capture_id") != report.get("capture_id")):
                    raise CoverageSearchError("搜索观察必须来自本turn成功传给模型并实际保存的报告")
                if _require_coordinate_evidence(state) != session["coordinate_epoch"] or scene_hash(state) != session["scene_sha256"]:
                    raise CoverageSearchError("搜索期间坐标epoch或场景已改变")
                snapshot = frame["snapshot"]
                if snapshot.get("coordinate_epoch") != session["coordinate_epoch"]:
                    raise CoverageSearchError("采图坐标epoch与搜索许可不一致，不能使用旧坐标图像推进搜索")
                capture = {"capture_id": frame["capture_id"], "stamp_sim_s": snapshot["stamp_sim_s"],
                           "age_seconds": snapshot["age_seconds"], "measurement_age_seconds": snapshot["measurement_age_seconds"],
                           "image_sha256": snapshot["stereo_image_sha256"], "image_quality": snapshot["image_quality"],
                           "body_pose_enu": snapshot["body_pose_enu"], "coordinate_epoch": snapshot.get("coordinate_epoch")}
                mission = state.get("mission") or {}
                if not search.record_navigation_result(current["id"], mission.get("status"), capture["body_pose_enu"]):
                    accepted = {"accepted": False, "reason": search.reason}
                else:
                    targets = [{"label": target["label"], "measurement": target["stereo_location"]}
                               for target in report.get("targets", [])
                               if (target.get("stereo_location") or {}).get("status") == "ok"]
                    if any(target["measurement"].get("coordinate_epoch") != session["coordinate_epoch"] for target in targets):
                        raise CoverageSearchError("目标双目测量来自其他坐标epoch，不能混合目标记忆")
                    accepted = search.record_observation(current["id"], capture, targets)
                session["last_report"] = frame["capture_id"]
            except (KeyError, TypeError, ValueError) as exc:
                self._fail_locked(session, "invalid_observation", str(exc))
                accepted = {"accepted": False, "reason": search.reason}
            result = {"status": "ok" if accepted["accepted"] else "error", "search_id": session["id"],
                      "bookkeeping": accepted, "search": search.summary()}
            if not accepted["accepted"]:
                result["error"] = accepted.get("reason")
        return self._audit("search_observation", result)


def scene_hash(state):
    scene = state.get("scene") or {}
    return scene.get("world_sha256") or scene.get("sha256")


def build_search_tools(client, manager=None):
    """Returns (two tools, manager); callback integration remains explicit."""
    from langchain.tools import ToolRuntime
    from langchain_core.tools import StructuredTool
    from pydantic import BaseModel, ConfigDict, Field
    from typing import Literal

    manager = manager if manager is not None else SearchManager(client)

    class SearchPlanArguments(BaseModel):
        model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)
        query: str = Field(min_length=1, max_length=80, description="Canonical user-requested target label, match report_scene label")
        bounds: list[float] = Field(min_length=4, max_length=4, description="Requested local ENU [xmin,ymin,xmax,ymax], inside verified live map bounds")
        spacing: float = Field(ge=.25, le=10, description="Observation grid spacing in metres; must fit verified camera footprint")
        max_observations: int = Field(ge=1, le=8, description="Hard budget for whole-region view sampling; do not truncate an oversized region")

    class SearchStatusArguments(BaseModel):
        model_config = ConfigDict(extra="forbid", strict=True)
        view: Literal["next", "summary"] = Field(description="next: unfinished observation point; summary: actual progress and targets")

    class TargetSearchArguments(BaseModel):
        model_config = ConfigDict(extra="forbid", strict=True)
        query: str = Field(min_length=1, max_length=80, description="Target label; use exactly this label in report_scene")
        end_behavior: Literal["land", "hold"] = "land"
        initial_altitude: float = Field(default=3., ge=1, le=10, description="Initial observation ENU altitude; default 3m; respect user-specified height")

    class TargetViewArguments(BaseModel):
        model_config = ConfigDict(extra="forbid", strict=True)
        strategy: Literal["auto", "rise", "left", "right", "forward"]
        reason: str = Field(min_length=1, max_length=300)

    class ExecuteTargetViewArguments(BaseModel):
        model_config = ConfigDict(extra="forbid", strict=True)

    class TurnSearchTool(StructuredTool):
        @property
        def tool_call_schema(self):
            return self.args_schema

        def _to_args_and_kwargs(self, tool_input, tool_call_id):
            if not isinstance(tool_input, dict):
                raise CoverageSearchError("Search tool arguments must be an object")
            runtime = tool_input.get("runtime")
            if not isinstance(runtime, ToolRuntime):
                raise CoverageSearchError("Search tools require authenticated ToolRuntime injection")
            parsed = self.args_schema.model_validate({k: v for k, v in tool_input.items() if k != "runtime"})
            return (), {**parsed.model_dump(), "runtime": runtime}

    def audited_error(name, exc):
        return manager._audit(name, {"status": "error", "error": str(exc)[:500], "actual_flight": False})

    def plan_region_search(query: str, bounds: list[float], spacing: float, max_observations: int, runtime: ToolRuntime) -> dict:
        """Plan up to eight deterministic observation points for an explicitly requested ENU area search.

        Requires fresh verified LiDAR ENU map after authorized takeoff to 3.5m.
        Reads metadata only and never flies; then request next view, execute its
        goto with the existing tool, capture_scene and report_scene at each point.
        """
        try:
            return manager.plan(runtime_owner(runtime), query, bounds, spacing, max_observations)
        except Exception as exc:
            return audited_error("plan_region_search", exc)

    def search_observation_status(view: Literal["next", "summary"], runtime: ToolRuntime) -> dict:
        """Read the current turn's next observation point or measured search progress.

        Blocked/failed/budget-ended searches have no next point. Do not skip them.
        Targets are reports with uncertainty, never flight destinations.
        """
        try:
            return manager.status(runtime_owner(runtime), view)
        except Exception as exc:
            return audited_error("search_observation_status", exc)

    def begin_target_search(query: str, runtime: ToolRuntime, end_behavior: Literal["land", "hold"] = "land", initial_altitude: float = 3.) -> dict:
        """Start autonomous simulation target search without user-specified coordinates.

        Forward means initial body heading. Authorizes takeoff and adaptive safe
        views within 12m, 1..10m altitude, eight observations and 900 seconds.
        Then take off if grounded, capture_scene, report_scene, and keep searching.
        """
        try:
            owner = runtime_owner(runtime)
            if owner in manager.sessions:
                raise CoverageSearchError("当前指令已有区域搜索")
            return manager._audit("begin_target_search", manager.adaptive.begin(owner, query, end_behavior, initial_altitude))
        except Exception as exc:
            return audited_error("begin_target_search", exc)

    def next_target_search_view(strategy: Literal["auto", "rise", "left", "right", "forward"],
                                reason: str, runtime: ToolRuntime) -> dict:
        """Select a new safe view after a saved report did not find the target.

        Chooses among rising/sideways/forward candidates using executor path
        preview. Returns goto to execute, then capture and report again. Never flies.
        """
        try:
            return manager._audit("next_target_search_view", manager.adaptive.next_view(runtime_owner(runtime), strategy, reason))
        except Exception as exc:
            return audited_error("next_target_search_view", exc)

    def execute_target_search_view(runtime: ToolRuntime) -> dict:
        """Fly to the current checked target-search observation point, without copying coordinates.

        Wait for actual flight completion, then capture_scene and report_scene.
        Operator interruption, VIO, collision and flight-failure gates still apply.
        """
        try:
            owner = runtime_owner(runtime)
            step = copy.deepcopy(manager.adaptive.sessions[owner]["pending"])
            if step is None or manager.adaptive.sessions[owner]['arrived']:
                return manager.adaptive.summary(owner)
            manager.check_mission(owner, [step])
            result = client.execute([step], "自主搜索：执行已验证的下一观察点")
            if (result.get('mission') or {}).get('status') == 'succeeded':
                manager.adaptive.sessions[owner]['arrived'] = True
            return manager._audit("execute_mission", result)
        except Exception as exc:
            manager.adaptive.fail(runtime_owner(runtime), str(exc))
            return manager._audit("execute_mission", {"status": "error", "error": str(exc)[:500]})

    return ([TurnSearchTool.from_function(plan_region_search, args_schema=SearchPlanArguments),
             TurnSearchTool.from_function(search_observation_status, args_schema=SearchStatusArguments),
             TurnSearchTool.from_function(begin_target_search, args_schema=TargetSearchArguments),
             TurnSearchTool.from_function(next_target_search_view, args_schema=TargetViewArguments),
             TurnSearchTool.from_function(execute_target_search_view, args_schema=ExecuteTargetViewArguments)], manager)
