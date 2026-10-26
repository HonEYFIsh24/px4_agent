"""Local operator UI: live telemetry and serialized, audited LangChain commands."""
import argparse
import copy
import json
import mimetypes
import os
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

from client import BridgeClient
from vision import model_metadata, observation_reply

PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_ENV_FILE = (PROJECT_DIR / ".env" if (PROJECT_DIR / ".env").is_file()
                    else Path("/home/dut/ros_agent/mcp-client-langchain/.env"))
MAX_BODY_BYTES = 16384


class CommandBusy(RuntimeError):
    pass


def message_value(message, field, default=None):
    return message.get(field, default) if isinstance(message, dict) else getattr(message, field, default)


def command_messages(result, prompt):
    """Exclude prior turns retained by the LangGraph checkpointer."""
    messages = result.get("messages", [])
    for index in range(len(messages) - 1, -1, -1):
        role = message_value(messages[index], "type") or message_value(messages[index], "role")
        if role in ("human", "user") and message_value(messages[index], "content") == prompt:
            return messages[index + 1:]
    return messages


def reply_text(message):
    content = message_value(message, "content", "")
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        return "\n".join(block.get("text", "") for block in content if isinstance(block, dict))
    return str(content)


def result_error(value, require_completed=False):
    if not isinstance(value, dict):
        return "工具没有返回有效的状态对象"
    if value.get("status") in ("error", "failed") or value.get("error"):
        return str(value.get("error") or "工具执行失败")
    if require_completed:
        mission = value.get("mission")
        if not isinstance(mission, dict):
            return "飞行工具没有返回可验证的任务结果"
        if mission.get("status") != "succeeded":
            return str(mission.get("error") or "任务状态为 {}".format(mission.get("status", "unknown")))
    return None


def completed_flight_evidence(tool_results):
    """Match this command's accepted flight IDs to their actual waited results.

    A final model exception may prevent graph ToolMessage parsing. The audit
    still proves execution, but an accepted request alone proves no completion.
    Require a completed landing and its observed ground/disarmed state; a later
    recovery landing must not turn a partially executed task into a full sortie.
    """
    pending, completed = {}, []
    for name, value in tool_results:
        if name == "/stop":
            return None
        if name not in ("/mission", "/land", "execute_mission", "land_drone"):
            continue
        if not isinstance(value, dict) or result_error(value):
            return None
        flight = value.get("mission")
        if (not isinstance(flight, dict) or not isinstance(flight.get("id"), str)
                or not flight["id"] or not isinstance(flight.get("steps"), list) or not flight["steps"]
                or not all(isinstance(step, dict) and isinstance(step.get("action"), str) for step in flight["steps"])):
            return None
        identifier = flight["id"]
        if name in ("/mission", "/land"):
            if identifier in pending or any(item["mission"]["id"] == identifier for item in completed):
                return None
            pending[identifier] = ("execute_mission" if name == "/mission" else "land_drone", flight["steps"])
        else:
            submitted = pending.get(identifier)
            if (submitted is None or submitted[0] != name or submitted[1] != flight["steps"]
                    or result_error(value, require_completed=True)
                    or type(flight.get("step")) is not int or flight["step"] != len(flight["steps"])):
                return None
            del pending[identifier]
            completed.append(value)
    if pending or not completed:
        return None
    final = completed[-1]
    if (final["mission"]["steps"][-1].get("action") != "land"
            or final.get("armed") is not False or final.get("landed_state") != 1):
        return None
    return copy.deepcopy(final["mission"])


class AuditedClient:
    """Use the real bridge while blocking further actions after an operator interrupt."""
    def __init__(self, application):
        self.application = application

    @property
    def agent_architecture(self):
        return self.application.agent_architecture

    @property
    def vision_config(self):
        return self.application.vision_config

    def capture_camera(self, side="left"):
        with self.application.lock:
            if self.application.current_command and self.application.current_command["interrupted"]:
                raise RuntimeError("操作员已中断此指令，取消本次视觉观察")
            if self.application.current_command and self.application.current_command["vision_failure"]:
                raise RuntimeError("此指令的视觉任务已失败，禁止自动重试；请发布新的观察指令")
        return self.application.client.capture_camera(side)

    @property
    def supports_stereo_localization(self):
        return getattr(self.application.client, "supports_stereo_localization", False) is True

    def capture_stereo(self):
        with self.application.lock:
            command = self.application.current_command
            if command and (command["interrupted"] or command["vision_failure"]):
                raise RuntimeError("此指令已中断或视觉分析失败，禁止继续采图")
        return self.application.client.capture_stereo()

    def localize_stereo(self, pair_id, bboxes):
        with self.application.lock:
            command = self.application.current_command
            if command and (command["interrupted"] or command["vision_failure"]):
                raise RuntimeError("此指令已中断或视觉分析失败，禁止继续定位")
        return self.application.client.localize_stereo(pair_id, bboxes)

    def record(self, tool_name, result):
        # LangChain may execute tools in worker threads. One serialized command owns all tools.
        with self.application.lock:
            command = self.application.current_command
            if command is not None:
                if tool_name == "inspect_scene":
                    result = self.application.clean_perception(result)
                    if command["interrupted"]:
                        result = {"status": "error", "error": "操作员已中断，已丢弃本次视觉分析结果",
                                  "targets": [], "limitations": []}
                    elif command["vision_failure"] and result.get("status") == "ok":
                        result = {"status": "error", "error": command["vision_failure"] + "；本指令后续报告不再采纳",
                                  "targets": [], "limitations": []}
                command["tool_results"].append((tool_name, copy.deepcopy(result)))
                if tool_name in ('/mission','/land') and isinstance(result,dict):
                    accepted_mission=result.get('mission')
                    if isinstance(accepted_mission,dict) and isinstance(accepted_mission.get('id'),str):
                        command.setdefault('flight_missions',[]).append({
                            'path':tool_name,'wall_s':time.time(),
                            'mission':self.application.clean_perception(accepted_mission)})
                if tool_name in ('plan_region_search', 'search_observation_status', 'search_progress',
                                 'begin_target_search', 'next_target_search_view', 'target_search_progress'):
                    event = {'tool': tool_name, 'result': self.application.clean_perception(result), 'wall_s': time.time()}
                    command.setdefault('search_events', []).append(event)
                    if tool_name == 'plan_region_search' and result.get('status') == 'ok':
                        command['search_plan'] = copy.deepcopy(event['result'])
                if tool_name == "vlm_call":
                    command["model_calls"].append(self.application.clean_perception(result))
                    if command.get("perception"):
                        command["perception"]["vlm_calls"] = copy.deepcopy(command["model_calls"])
                        self.application.latest_perception = copy.deepcopy(command["perception"])
                if tool_name == "inspect_scene":
                    command["perception"] = copy.deepcopy(result)
                    command["perceptions"].append(copy.deepcopy(result))
                    self.application.latest_perception = copy.deepcopy(result)
                    failure = result_error(result)
                    if failure and not command["vision_failure"]:
                        command["vision_failure"] = self.application.clean_error(failure)
                if tool_name in ("execute_mission", "land_drone"):
                    failure = result_error(result, require_completed=True)
                    if failure and not command["flight_failure"]:
                        command["flight_failure"] = self.application.clean_error(failure)
        return result

    def state(self):
        return self.application.client.state()

    def memory_records(self):
        """Only finalized command facts; never expose a running task as completed memory."""
        with self.application.lock:
            completed = [c for c in self.application.commands.values()
                         if c.get('status') != 'running' and c.get('finished_at') is not None][-8:]
            records = []
            for command in completed:
                record = {key: copy.deepcopy(command.get(key)) for key in ('id', 'status', 'error')}
                observations = [p for p in command.get('perceptions', []) if p.get('status') == 'ok'][-2:]
                record['observation_count'] = sum(p.get('status') == 'ok' for p in command.get('perceptions', []))
                record['perceptions'] = [{key: copy.deepcopy(p.get(key)) for key in ('status', 'capture_id', 'summary')}
                                         for p in observations]
                for p, raw in zip(record['perceptions'], observations):
                    p['targets'] = [{'label': t.get('label')} for t in raw.get('targets', [])]
                record['mission'] = {key: copy.deepcopy((command.get('mission') or {}).get(key))
                                     for key in ('status', 'action')}
                records.append(record)
            return records

    def request(self, path, payload=None):
        if payload is None:
            return self.application.client.request(path)
        try:
            with self.application.control_lock:
                with self.application.lock:
                    command = self.application.current_command
                    if command is not None and command.get("interrupted"):
                        raise RuntimeError("操作员已中断此指令，禁止继续提交飞行动作")
                    if path in ("/mission", "/land", "/stop") and command is not None and command["mode"] == "observe":
                        raise RuntimeError("本次指令仅允许视觉观察，禁止提交飞行动作")
                    if path == "/mission" and command is not None and command.get("vision_failure"):
                        raise RuntimeError("本次视觉分析已失败，禁止据此继续提交飞行任务")
                    if path == "/mission" and command is not None and command["flight_failure"]:
                        raise RuntimeError("此指令的飞行任务已失败，禁止自动重试；请查看状态后发布新指令")
                result = self.application.client.request(path, payload)
        except Exception as exc:
            if path in ("/mission", "/land"):
                self.record("execute_mission" if path == "/mission" else "land_drone",
                            {"status": "error", "error": str(exc)})
            raise
        return self.record(path, result)

    def execute(self, steps, description="", on_progress=None):
        accepted = self.request("/mission", {"steps": steps, "description": description})
        try:
            result = self.application.client.wait_for(accepted, on_progress)
        except Exception as exc:
            self.record("execute_mission", {"status": "error", "error": str(exc)})
            raise
        return self.record("execute_mission", result)

    def wait_for(self, accepted, on_progress=None):
        try:
            result = self.application.client.wait_for(accepted, on_progress)
        except Exception as exc:
            self.record("land_drone", {"status": "error", "error": str(exc)})
            raise
        return self.record("land_drone", result)


class WebApplication:
    def __init__(self, client, model_config=None, agent_factory=None, web_root=None, camera_path=None,
                 camera_paths=None, vision_config=None, agent_architecture="dual_model"):
        self.client = client
        self.model_config = model_config
        self.agent_architecture = agent_architecture
        self.vision_config = vision_config
        self.vision_model = model_metadata(vision_config)
        self.latest_perception = None
        self.model = {"name": model_config.get("model") if model_config else None,
                      "provider": urlsplit(model_config.get("base_url") or "https://api.openai.com").hostname
                      if model_config else None,
                      "available": bool(model_config)}
        self.web_root = Path(web_root or PROJECT_DIR / "web").resolve()
        self.camera_path = Path(camera_path or PROJECT_DIR / "logs/vio/camera_live.jpg")
        self.camera_paths = {"left": self.camera_path,
                             "right": self.camera_path.with_name("camera_right.jpg")}
        if camera_paths:
            self.camera_paths.update({side: Path(path) for side, path in camera_paths.items()
                                      if side in self.camera_paths})
        self.lock = threading.RLock()
        self.control_lock = threading.Lock()
        self.commands = {}
        self.active_id = None
        self.current_command = None
        self.audited_client = AuditedClient(self)
        self.agent = agent_factory(self.audited_client, model_config) if agent_factory and model_config else None
        self.agent_thread_id = "web-operator-" + uuid.uuid4().hex

    def clean_error(self, error):
        text = str(error)
        for config in (self.model_config, self.vision_config):
            if config and config.get("api_key"):
                text = text.replace(config["api_key"], "[已隐藏密钥]")
        return text[:1000]

    def clean_perception(self, value):
        if isinstance(value, dict):
            return {key: self.clean_perception(item) for key, item in value.items()
                    if key not in ("image_base64", "api_key")}
        if isinstance(value, list):
            return [self.clean_perception(item) for item in value]
        if isinstance(value, str):
            if "data:image/" in value:
                return "[已隐藏图像数据]"
            return self.clean_error(value)
        return copy.deepcopy(value)

    def state(self):
        try:
            state = self.client.state()
            if not isinstance(state, dict):
                raise RuntimeError("执行器返回了无效状态")
            state = copy.deepcopy(state)
        except Exception as exc:
            state = {"ready": False, "connected": False, "simulation_only": True,
                     "reason": "无法连接 ROS 执行器", "error": self.clean_error(exc),
                     "gazebo": {"available": False}, "vio": {"ready": False},
                     "camera": {side: {"available": False} for side in self.camera_paths}}
        state["model"] = self.model.copy()
        state["agent_architecture"] = self.agent_architecture
        with self.lock:
            state["vision_model"] = self.vision_model.copy()
            state["latest_perception"] = copy.deepcopy(self.latest_perception)
            state["active_command_id"] = self.active_id
        return state

    @staticmethod
    def public_command(command):
        return copy.deepcopy({key: value for key, value in command.items()
                              if key not in ("tool_results", "interrupted", "flight_failure", "vision_failure")})

    def submit(self, prompt, mode="task", min_observations=None):
        if not isinstance(prompt, str) or not 1 <= len(prompt.strip()) <= 2000:
            raise ValueError("指令必须为 1–2000 个字符")
        if mode not in ("task", "observe"):
            raise ValueError("指令模式必须为 task 或 observe")
        if min_observations is None:
            min_observations = 1 if mode == "observe" else 0
        if type(min_observations) is not int or not 0 <= min_observations <= 8:
            raise ValueError("min_observations 必须为 0–8 的整数")
        if self.agent is None:
            raise RuntimeError("模型尚未配置，请使用 --env-file 指定有效配置")
        with self.lock:
            if self.active_id:
                raise CommandBusy("已有自然语言指令正在执行，请等待结果或点击悬停/降落")
            command_id = uuid.uuid4().hex[:16]
            command = {"id": command_id, "status": "running", "prompt": prompt.strip(), "mode": mode,
                       "reply": "", "error": None, "created_at": time.time(), "finished_at": None,
                       "tool_results": [], "interrupted": False, "flight_failure": None, "perception": None,
                       "vision_failure": None, "model_calls": [], "safety_landing": None, "tool_errors": [],
                       "perceptions": [], "min_observations": min_observations, "flight_missions": []}
            if mode == "observe":
                self.latest_perception = None
            self.commands[command_id] = command
            self.active_id = command_id
            self.current_command = command
            while len(self.commands) > 100:
                self.commands.pop(next(iter(self.commands)))
            response = self.public_command(command)
        threading.Thread(target=self.run_command, args=(command,), daemon=True).start()
        return response

    def command(self, command_id):
        with self.lock:
            value = self.commands.get(command_id)
            return self.public_command(value) if value else None

    def run_command(self, command):
        status, error, reply, mission = "succeeded", None, "", None
        try:
            result = self.agent.invoke({"messages": [{"role": "user", "content": command["prompt"], "id": command['id']}]},
                                       {"configurable": {"thread_id": self.agent_thread_id},
                                        "recursion_limit": 120, "max_concurrency": 1})
            messages = command_messages(result, command["prompt"])
            if messages:
                reply = reply_text(messages[-1])
            if not reply:
                error = "模型没有返回说明"
            # Tool responses are independent evidence; model prose cannot turn a failed flight green.
            for message in messages:
                role = message_value(message, "type") or message_value(message, "role")
                if role != "tool":
                    continue
                tool_name = message_value(message, "name", "")
                try:
                    value = json.loads(message_value(message, "content", ""))
                except (TypeError, ValueError):
                    detail = self.clean_perception(str(message_value(message, "content", "")))[:500]
                    diagnostic = {"tool": tool_name, "error": detail or "工具返回结果无法解析"}
                    command["tool_errors"].append(diagnostic)
                    error = error or "{} 工具返回结果无法解析：{}".format(tool_name, diagnostic["error"])
                    continue
                failure = result_error(value)
                if failure:
                    command["tool_errors"].append({"tool": tool_name, "error": self.clean_error(failure)})
                error = error or result_error(value, tool_name in ("execute_mission", "land_drone"))
                if tool_name in ("execute_mission", "land_drone") and isinstance(value, dict):
                    if isinstance(value.get("mission"), dict):
                        mission = value["mission"]
            for tool_name, value in command["tool_results"]:
                if tool_name == "inspect_scene":
                    error = error or result_error(value)
                if tool_name in ("execute_mission", "land_drone"):
                    error = error or result_error(value, True)
                    if isinstance(value, dict) and isinstance(value.get("mission"), dict):
                        mission = value["mission"]
            if command["interrupted"]:
                error = error or "操作员已通过悬停/降落按钮中断此指令"
            if command["mode"] == "observe" and command["perception"] is None:
                error = error or "模型没有实际调用视觉工具，未分析当前图像"
            captures = {value.get("capture_id") for name, value in command["tool_results"]
                        if name == "capture_scene" and isinstance(value, dict)
                        and value.get("status") == "ok" and value.get("capture_id")}
            reports = {value.get("capture_id") for value in command["perceptions"]
                       if value.get("status") == "ok" and value.get("capture_id") in captures}
            if self.agent_architecture == "single_vlm":
                observed = len(reports)
            else:
                observed = sum(value.get("status") == "ok" for value in command["perceptions"])
            if captures - reports:
                error = error or "采图后未保存对应的有效观察报告"
            if observed < command["min_observations"]:
                error = error or "有效观察报告不足：需要{}份，实际{}份".format(command["min_observations"], observed)
            if error:
                status = "failed"
        except Exception as exc:
            status, error = "failed", self.clean_error(exc)
        finally:
            # A failed model cannot be relied on to invoke the remaining landing tool.
            # Cancel this command's running worker even before it arms. Otherwise
            # a failed command could leave a delayed takeoff executing afterwards.
            if status == "failed" and self.agent_architecture == "single_vlm" and command["mode"] == "task" and not command["interrupted"]:
                accepted_ids={event['mission']['id'] for event in command.get('flight_missions',[])
                    if event.get('path')=='/mission' and isinstance(event.get('mission'),dict)
                    and isinstance(event['mission'].get('id'),str)}
                # Keep support for authoritative completed-tool evidence while
                # checking the currently active mission, rather than any past flight.
                accepted_ids.update(value['mission']['id'] for name,value in command['tool_results']
                    if name=='execute_mission' and isinstance(value,dict) and isinstance(value.get('mission'),dict)
                    and isinstance(value['mission'].get('id'),str))
                owned_flight = bool(accepted_ids)
                if owned_flight:
                    try:
                        current_state=self.client.state()
                        current_mission=current_state.get('mission') or {}
                        current_id=current_mission.get('id')
                        if current_id in accepted_ids and (current_state.get('armed') is True
                                or current_mission.get('status') == 'running'):
                            steps=current_mission.get('steps')
                            landing_running=(current_mission.get('status') == 'running'
                                and isinstance(steps,list) and len(steps)==1
                                and isinstance(steps[0],dict) and steps[0].get('action')=='land')
                            accepted=({'mission':copy.deepcopy(current_mission)} if landing_running
                                else self.audited_client.request("/land", {'expected_mission_id':current_id}))
                            landing = self.audited_client.wait_for(accepted)
                            failure = result_error(landing, require_completed=True)
                            command["safety_landing"] = {"status": "error" if failure else "ok", "error": failure,
                                                         "mission": copy.deepcopy(landing.get("mission"))}
                            if landing.get("mission"):
                                mission = landing["mission"]
                    except Exception as exc:
                        command["safety_landing"] = {"status": "error", "error": self.clean_error(exc)}
            with self.lock:
                if command["interrupted"]:
                    status = "failed"
                    error = "该指令已由操作员中断"
                    reply = "该指令已中断，Agent 不会继续执行其中的飞行动作。请查看任务进度和当前飞行状态。"
                elif status == "succeeded" and command["mode"] == "observe" and command.get("perception"):
                    reply = observation_reply(command["perception"])
                elif status == "failed":
                    error = command["flight_failure"] or error or "无法确认指令执行结果"
                    saved_observation = any(value.get("status") == "ok" for value in command["perceptions"])
                    model_failure = any(value.get("status") == "error" for value in command["model_calls"])
                    completed_flight = completed_flight_evidence(command["tool_results"])
                    captures = {value.get("capture_id") for name, value in command["tool_results"]
                                if name == "capture_scene" and isinstance(value, dict)
                                and value.get("status") == "ok" and value.get("capture_id")}
                    reports = {value.get("capture_id") for value in command["perceptions"]
                               if value.get("status") == "ok" and value.get("capture_id") in captures}
                    final_model_failure = (bool(command["model_calls"])
                        and command["model_calls"][-1].get("status") == "error"
                        and command["model_calls"][-1].get("input_modalities") == ["text"]
                        and all(value.get("status") == "ok" for value in command["model_calls"][:-1]))
                    if (command["mode"] == "task" and completed_flight and final_model_failure
                            and not command["flight_failure"] and not command["vision_failure"]
                            and not command["tool_errors"] and not command["safety_landing"]
                            and not captures - reports and len(reports) >= command["min_observations"]):
                        mission = completed_flight
                        reply = "飞行执行已完成；模型最终回复生成失败：{}。".format(self.clean_error(error))
                        if saved_observation:
                            reply += "视觉观察报告已保存。"
                        reply += "Agent 不会自动重试或继续提交飞行动作。"
                    elif (command["mode"] == "task" and command["flight_failure"] and saved_observation
                            and not command["vision_failure"] and not model_failure):
                        reply = "视觉观察报告已保存；飞行任务失败：{}。Agent 不会自动重试飞行任务，请查看任务进度和当前飞行状态。".format(
                            self.clean_error(error))
                    elif command["mode"] == "observe" or command.get("perception"):
                        reply = "视觉观察未完成：{}。请检查视觉报告和模型配置后重新发起观察。".format(self.clean_error(error))
                        if command["perception"] is None:
                            command["perception"] = {"status": "error", "query": command["prompt"],
                                                     "model": self.vision_model["name"], "error": self.clean_error(error),
                                                     "targets": [], "limitations": []}
                            self.latest_perception = copy.deepcopy(command["perception"])
                    else:
                        reply = "指令执行失败：{}。Agent 不会自动重试飞行任务，请查看任务进度和当前飞行状态。".format(
                            self.clean_error(error))
                    if command.get("safety_landing"):
                        reply += "\n指令失败后，独立执行器已完成安全降落并上锁。" if command["safety_landing"]["status"] == "ok" else "\n安全降落未确认完成，请查看飞行状态并使用立即降落。"
                command.update(status=status, reply=reply, error=self.clean_error(error) if error else None,
                               finished_at=time.time())
                if mission:
                    command["mission"] = copy.deepcopy(mission)
                if self.active_id == command["id"]:
                    self.active_id = None
                    self.current_command = None

    def interrupt(self, path, payload=None):
        payload={} if payload is None else payload
        permitted={'expected_mission_id','expected_command_id'} if path=='/land' else set()
        if not isinstance(payload,dict) or set(payload)-permitted:
            raise ValueError('中断请求包含未知字段')
        for value in payload.values():
            if not isinstance(value,str) or not 1<=len(value)<=64:
                raise ValueError('回收请求的任务标识无效')
        with self.control_lock:
            with self.lock:
                expected_command=payload.get('expected_command_id')
                if expected_command is not None and (expected_command not in self.commands
                        or self.active_id not in (None,expected_command)):
                    raise ValueError('指令已改变，拒绝中断其他指令')
                expected_mission=payload.get('expected_mission_id')
                if expected_mission is not None:
                    state=self.client.state()
                    if (state.get('mission') or {}).get('id')!=expected_mission:
                        raise ValueError('飞行任务已改变，拒绝中断其他飞行')
                    if expected_command is not None:
                        accepted_ids={event['mission']['id'] for event in self.commands[expected_command].get('flight_missions',[])
                            if isinstance(event.get('mission'),dict) and isinstance(event['mission'].get('id'),str)}
                        if expected_mission not in accepted_ids:
                            raise ValueError('指定指令不拥有此飞行，拒绝回收其他任务')
                    if self.active_id and expected_command is None:
                        accepted_ids={event['mission']['id'] for event in self.commands[self.active_id].get('flight_missions',[])
                            if isinstance(event.get('mission'),dict) and isinstance(event['mission'].get('id'),str)}
                        if expected_mission not in accepted_ids:
                            raise ValueError('当前指令不拥有指定飞行，拒绝中断')
                if self.active_id:
                    self.commands[self.active_id]["interrupted"] = True
            bridge_payload={'expected_mission_id':payload['expected_mission_id']} if 'expected_mission_id' in payload else {}
            return self.client.request(path, bridge_payload)


def make_server(application, port=8080):
    class Handler(BaseHTTPRequestHandler):
        server_version = "UAVOperator/1.0"

        def log_message(self, *arguments):
            pass

        def send_bytes(self, status, content, content_type):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; "
                             "style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; "
                             "connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
            self.end_headers()
            try:
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def respond(self, status, value):
            self.send_bytes(status, json.dumps(value, ensure_ascii=False, allow_nan=False).encode(),
                            "application/json; charset=utf-8")

        def trusted_request(self, mutation=False):
            port = self.server.server_port
            host = self.headers.get("Host", "")
            permitted = {"127.0.0.1:{}".format(port), "localhost:{}".format(port)}
            if port == 80:
                permitted.update(("127.0.0.1", "localhost"))
            if host not in permitted:
                self.respond(403, {"error": "此页面只接受本机访问"})
                return False
            if mutation:
                origin = self.headers.get("Origin")
                fetch_site = self.headers.get("Sec-Fetch-Site")
                if (origin and origin != "http://" + host) or fetch_site in ("cross-site", "same-site"):
                    self.respond(403, {"error": "飞行操作必须从本机控制页面提交"})
                    return False
            return True

        def do_GET(self):
            if not self.trusted_request():
                return
            path = urlsplit(self.path).path
            if path == "/api/state":
                self.respond(200, application.state())
            elif path.startswith("/api/commands/"):
                command = application.command(path[len("/api/commands/"):])
                self.respond(200 if command else 404, command or {"error": "指令不存在"})
            elif path in ("/api/camera.jpg", "/api/camera/left.jpg", "/api/camera/right.jpg"):
                side = "right" if path == "/api/camera/right.jpg" else "left"
                camera_path = application.camera_paths[side]
                try:
                    if time.time() - camera_path.stat().st_mtime > 2:
                        raise FileNotFoundError()
                    self.send_bytes(200, camera_path.read_bytes(), "image/jpeg")
                except OSError:
                    self.respond(404, {"error": "当前没有新鲜的仿真相机图像"})
            elif path.startswith("/api/"):
                self.respond(404, {"error": "接口不存在"})
            else:
                try:
                    target = (application.web_root / unquote(path).lstrip("/")).resolve()
                    if path == "/":
                        target = (application.web_root / "index.html").resolve()
                    if application.web_root not in target.parents or not target.is_file():
                        raise FileNotFoundError()
                    content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
                    if content_type.startswith("text/") or content_type in ("application/javascript", "application/json"):
                        content_type += "; charset=utf-8"
                    self.send_bytes(200, target.read_bytes(), content_type)
                except (OSError, ValueError):
                    self.respond(404, {"error": "页面文件不存在"})

        def do_POST(self):
            if not self.trusted_request(mutation=True):
                return
            try:
                if self.headers.get("Transfer-Encoding"):
                    raise ValueError("不支持分块请求")
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= MAX_BODY_BYTES:
                    self.respond(413, {"error": "请求正文必须为 1–16384 字节"})
                    return
                if self.headers.get_content_type() != "application/json":
                    self.respond(415, {"error": "请使用 application/json 请求正文"})
                    return
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError("请求正文必须为 JSON 对象")
                path = urlsplit(self.path).path
                if path == "/api/commands":
                    self.respond(202, application.submit(payload.get("prompt"), payload.get("mode", "task"),
                                                         payload.get("min_observations")))
                elif path in ("/api/stop", "/api/land"):
                    self.respond(200, application.interrupt(path[len("/api"):],payload))
                else:
                    self.respond(404, {"error": "接口不存在"})
            except CommandBusy as exc:
                self.respond(409, {"error": str(exc)})
            except (ValueError, UnicodeDecodeError) as exc:
                self.respond(400, {"error": application.clean_error(exc)})
            except Exception as exc:
                self.respond(503, {"error": application.clean_error(exc)})

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    return server


def build_application(env_file=DEFAULT_ENV_FILE, bridge_url=None):
    """Shared operator session for browser and native desktop frontends."""
    from model_network import configure_agent_network
    configure_agent_network(env_file)
    from dotenv import dotenv_values
    from agent import build_agent
    from agent_config import runtime_configuration
    connection_config = {**dotenv_values(env_file), **os.environ}
    try:
        architecture, model_config, vision_config = runtime_configuration(env_file)
    except RuntimeError as exc:
        model_config, vision_config = None, None
        architecture = connection_config.get("AGENT_ARCHITECTURE", "single_vlm")
        print("模型配置不可用：{}。仍可查看状态和直接控制，任务需要配置模型。".format(exc), flush=True)
    return WebApplication(BridgeClient(bridge_url or connection_config.get("UAV_BRIDGE_URL")
                                       or "http://127.0.0.1:8765"), model_config, build_agent,
                          vision_config=vision_config, agent_architecture=architecture)


def main():
    parser = argparse.ArgumentParser(description="Local UAV agent operator page")
    parser.add_argument("--env-file", default=os.getenv("UAV_ENV_FILE", str(DEFAULT_ENV_FILE)))
    parser.add_argument("--bridge-url", default=None)
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    application = build_application(args.env_file, args.bridge_url)
    server = make_server(application, args.port)
    print("无人机控制页面：http://127.0.0.1:{}（模型：{}）".format(
        args.port, application.model["name"] or "未配置"), flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        with application.lock:
            active = application.active_id is not None
        if active:
            try:
                application.interrupt("/stop")
            except Exception:
                pass
        server.server_close()


if __name__ == "__main__":
    main()
