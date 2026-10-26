/* Local control console. Every flight value comes from the server's live telemetry. */
"use strict";

const $ = (id) => document.getElementById(id);
const ACTION_NAMES = { takeoff: "起飞", goto: "前往航点", hold: "悬停", return_home: "返航", land: "降落" };
const STATUS_NAMES = { running: "执行中", succeeded: "已完成", failed: "执行失败", cancelled: "已停止" };
const LANDED_NAMES = { 0: "未知", 1: "已着陆", 2: "飞行中", 3: "正在起飞", 4: "正在降落" };
const TELEMETRY_REASONS = {
  "MAVROS is disconnected or its state is stale": "飞控连接已中断，或状态数据已过期。",
  "local position is missing or stale": "尚未收到有效的本地位置，或位置数据已过期。",
  "local position is invalid": "本地定位数据无效，请等待定位恢复。",
  "Gazebo has not published Iris state": "仿真尚未提供无人机位置，请等待仿真启动。",
  "Gazebo state is stale or invalid": "仿真状态已过期或无效，请检查仿真是否暂停。",
  "VINS-Fusion has not published an optimized pose": "视觉惯性定位尚未输出有效结果。",
  "VINS is initializing, resetting, or its tracking status is stale": "视觉惯性定位正在初始化、重置，或跟踪状态已过期。",
  "VIO output is missing, delayed, or from an old simulation clock": "视觉惯性定位结果缺失或已过期。",
  "VIO output contains nonfinite values": "视觉惯性定位结果含无效数值。",
  "VIO orientation is not a unit quaternion": "视觉惯性定位的姿态数据无效。",
  "VIO speed exceeds the configured simulation validity limit": "视觉惯性定位的速度超出有效范围。",
  "VIO position exceeds the configured simulation validity limit": "视觉惯性定位的位置超出有效范围。",
};
let state = null;
let apiOnline = false;
let commandActive = false;
let commandId = null;
let pendingReply = null;
let controlBusy = false;
let missionSignature = "";
let mapLive = false;
let mapPosition = null;
let mapYaw = 0;
let lastSimulationStamp = null;
let trail = [];
let commandPollTimer = null;
let statePollTimer = null;
let visionCommandActive = false;
let activeCommandSource = "command";
let perceptionSignature = "";
const CAMERA_NAMES = ["left", "right"];
const cameraStreams = Object.fromEntries(CAMERA_NAMES.map((name) => [name, {
  metadata: null, timer: null, controller: null, url: null, generation: 0,
}]));

function finiteVector(value, length = 3) {
  return Array.isArray(value) && value.length >= length && value.slice(0, length).every(Number.isFinite);
}

function telemetryReason(reason, fallback = "等待有效状态数据。") {
  return TELEMETRY_REASONS[reason] || reason || fallback;
}

function number(value, digits = 2) {
  return Number.isFinite(value) ? value.toFixed(digits) : "—";
}

function vector(value, separator = " / ") {
  return finiteVector(value) ? value.slice(0, 3).map((n) => number(n)).join(separator) : "— / — / —";
}

function metric(id, value, unit, digits = 1) {
  const element = $(id);
  element.replaceChildren(document.createTextNode(number(value, digits)));
  const label = document.createElement("small");
  label.textContent = unit;
  element.append(label);
}

function badge(id, label, type = "muted", dot = false) {
  const element = $(id);
  element.className = `status-pill ${type}`;
  element.replaceChildren();
  if (dot) {
    const mark = document.createElement("span");
    mark.className = "status-dot";
    element.append(mark);
  }
  element.append(document.createTextNode(label));
}

async function request(path, options = {}) {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 12000);
  try {
    const response = await fetch(path, {
      cache: "no-store",
      ...options,
      headers: { "Content-Type": "application/json", ...(options.headers || {}) },
      signal: controller.signal,
    });
    let data;
    try { data = await response.json(); } catch { throw new Error("控制服务返回了无法读取的响应。"); }
    if (!response.ok) {
      const error = new Error(data.error || `请求失败（HTTP ${response.status}）。`);
      error.httpStatus = response.status;
      throw error;
    }
    return data;
  } catch (error) {
    if (error.name === "AbortError") throw new Error("控制服务响应超时，请检查服务连接。");
    if (error instanceof TypeError) throw new Error("无法连接本机控制服务。");
    throw error;
  } finally {
    clearTimeout(timeout);
  }
}

function updateButtons() {
  const modelReady = state?.model?.available !== false;
  $("submit-command").disabled = !apiOnline || commandActive || !modelReady;
  $("submit-label").textContent = commandActive ? "Agent 处理中" : "发送指令";
  $("prompt-input").disabled = commandActive;
  for (const button of document.querySelectorAll(".example-button")) button.disabled = commandActive;
  const visionReady = state?.vision_model?.configured === true;
  $("submit-vision").disabled = !apiOnline || commandActive || !modelReady || !visionReady;
  $("vision-query").disabled = commandActive;
  $("vision-submit-label").textContent = visionCommandActive ? "模型正在分析" : commandActive ? "等待当前指令完成" : "分析相机画面";
  $("stop-button").disabled = !apiOnline || !state?.connected || controlBusy;
  $("land-button").disabled = !apiOnline || !state?.connected || controlBusy;
}

function renderState(data) {
  state = data;
  const ready = data.ready === true;
  badge("connection-pill", ready ? "飞控已连接" : data.connected ? "飞控待就绪" : "飞控未连接", ready ? "good" : "warning", true);
  const banner = $("connection-banner");
  banner.className = `notice ${ready ? "good" : "bad"}`;
  banner.textContent = ready
    ? "飞行执行器已就绪。可发布自然语言任务；停止与降落按钮可直接控制无人机。"
    : `飞行执行器尚未就绪：${telemetryReason(data.reason || data.error, "等待飞控连接与有效定位。")}`;

  const model = data.model || {};
  $("model-name").textContent = model.name || "未配置模型";
  $("model-provider").textContent = model.available === false ? "模型接口未配置" : model.provider || "兼容 OpenAI 协议的接口";
  $("agent-architecture").textContent = {
    single_vlm: "单 VLM Agent", dual_model: "双模型 Agent",
  }[data.agent_architecture] || "Agent";
  const visionModel = data.vision_model || {};
  $("vision-model").textContent = visionModel.configured === true
    ? `${visionModel.name || "视觉模型"}${visionModel.provider ? ` · ${visionModel.provider}` : ""}${data.agent_architecture === "single_vlm" ? " · 同一 VLM 处理指令与图像" : ""}`
    : "视觉模型尚未配置，暂时无法发送图像进行分析。";
  if (!visionCommandActive) renderPerception(data.latest_perception || null);
  $("flight-mode").textContent = data.mode || "—";
  $("armed-state").textContent = data.connected ? (data.armed ? "已解锁" : "已上锁") : "—";
  $("landed-state").textContent = data.connected ? LANDED_NAMES[data.landed_state] || "未知" : "—";
  $("position-ready").textContent = ready ? "已就绪" : "待就绪";
  badge("vehicle-badge", ready ? (data.armed ? "飞行已解锁" : "准备就绪") : "待就绪", ready ? "good" : "warning");
  $("px4-position").textContent = ready ? vector(data.position_enu_m) : "— / — / —";
  $("px4-target").textContent = ready ? vector(data.target_enu_m) : "— / — / —";

  const vio = data.vio || {};
  $("vio-indicator").className = `vio-indicator${vio.ready ? " good" : ""}`;
  $("vio-title").textContent = `VINS-Fusion · ${vio.ready ? "定位可用" : "等待有效定位"}`;
  const controlNote = vio.used_for_flight_control
    ? "VIO 已用于飞行控制。"
    : "VIO 未参与飞行控制；当前使用 PX4 本地定位。";
  $("vio-detail").textContent = vio.ready && finiteVector(vio.position_vio_m)
    ? `VIO 位置 ${vector(vio.position_vio_m)} m（独立坐标系）。${controlNote}`
    : `${controlNote}${vio.reason ? ` ${telemetryReason(vio.reason)}` : ""}`;

  renderGazebo(data.gazebo || {});
  renderNavigation(data.navigation || {}, data.scene || {});
  renderCameras(data.camera || {});
  renderMission(data.mission);
  updateButtons();
}

function renderNavigation(navigation, scene) {
  const enabled = navigation.enabled === true;
  const active = navigation.active === true;
  const lidarAvailable = navigation.lidar_available === true;
  const fresh = navigation.fresh === true;
  const altitude = enabled && Number.isFinite(scene.navigation?.flight_altitude_m)
    ? scene.navigation.flight_altitude_m : null;
  let label = "当前场景未启用";
  let tone = "muted";
  let detail = "当前场景没有启用自主绕障。";
  if (enabled && !active) {
    label = lidarAvailable ? "待起飞建图" : "等待激光扫描";
    tone = lidarAvailable ? "muted" : "warning";
    detail = lidarAvailable
      ? "激光扫描已接收。无人机到达固定巡航高度后，开始建立绕障地图。"
      : "尚未收到新鲜激光扫描；起飞后建图仍需有效传感器数据。";
  } else if (enabled && active) {
    label = fresh ? "建图可用" : "等待有效建图";
    tone = fresh ? "good" : "warning";
    detail = fresh
      ? "只在已观测的空闲区域规划；遇到未知区域会先到可见位置继续观测。"
      : "地图数据缺失或已过期，当前路径不能继续移动。请查看任务状态。";
  }
  badge("navigation-status", label, tone);
  const samples = Number.isFinite(navigation.lidar_sample_count) ? `${number(navigation.lidar_sample_count, 0)} 束` : "等待扫描";
  const scans = Number.isFinite(navigation.scan_count) ? number(navigation.scan_count, 0) : "0";
  $("navigation-scans").textContent = `${samples} · ${scans} 次建图`;
  $("navigation-source").textContent = !active
    ? lidarAvailable ? navigation.scan_count > 0 ? "激光在线 · 等待重新建图" : "激光在线 · 尚未建图" : "尚无新鲜扫描"
    : fresh ? `有效 · ${number(navigation.source_age_s, 2)} 秒前` : "地图暂不可用";
  $("navigation-distance").textContent = Number.isFinite(navigation.nearest_obstacle_m)
    ? `${number(navigation.nearest_obstacle_m, 2)} m${fresh && active ? "" : "（上次）"}` : fresh && active ? "暂无障碍命中" : "—";
  const plan = navigation.last_plan;
  $("navigation-path").textContent = plan && Array.isArray(plan.waypoints)
    ? `${active ? "" : "上次 "}${plan.waypoints.length} 段 · ${plan.route_complete ? "到目标" : "待继续观测"}${Number.isFinite(plan.path_length_m) ? ` · ${number(plan.path_length_m, 1)} m` : ""}`
    : "尚未规划";
  $("navigation-map").textContent = Number.isFinite(navigation.observed_fraction)
    ? `${active ? "" : "上次"}已观测 ${number(navigation.observed_fraction * 100, 0)}% · 空闲 ${number(navigation.free_cells, 0)} 格 · 障碍 ${number(navigation.occupied_cells, 0)} 格`
    : "机载激光建图将在起飞后开始。";
  $("navigation-scope").textContent = enabled
    ? `固定${altitude === null ? "巡航高度" : `本地高度 ${number(altitude, 1)} m`} · 静态二维绕障，暂不支持立体绕障或动态目标。`
    : "此场景没有启用自主绕障。";
  $("navigation-detail").textContent = detail;
  if (enabled && navigation.method === "static_3d") {
    badge("navigation-status", active ? "三维路径规划" : "已知三维地图", "good");
    $("navigation-scans").textContent = `${samples} · 雷达仅作遥测`;
    $("navigation-source").textContent = "已校验的静态场景碰撞地图";
    $("navigation-distance").textContent = "使用三维碰撞检查";
    $("navigation-map").textContent = "执行器读取已知静态地图；不是机载雷达实时建图。";
    $("navigation-scope").textContent = "本地高度 1～10 m，可变高 · 已知静态三维避障 · 不保证动态障碍保护";
    $("navigation-detail").textContent = "定位来源以实际 EKF 融合状态为准，路径规划来源为已知地图。";
  }

  // Keep the existing example actions valid for a fixed-altitude scene.
  const exampleAltitude = String(altitude === null ? 2 : altitude);
  for (const button of document.querySelectorAll("[data-flight-example]")) {
    button.dataset.prompt = button.dataset.flightExample === "takeoff"
      ? `起飞到本地高度 ${exampleAltitude} 米，悬停 3 秒，然后降落。`
      : `起飞到本地高度 ${exampleAltitude} 米，飞到本地 ENU 坐标（2，0，${exampleAltitude}）米，然后返航降落。`;
  }
  $("prompt-input").placeholder = `例如：起飞到 ${exampleAltitude} 米，悬停 3 秒，然后降落。`;
}

function renderGazebo(gazebo) {
  const valid = gazebo.available === true && finiteVector(gazebo.position_m)
    && (!Number.isFinite(gazebo.age_wall_s) || gazebo.age_wall_s <= 2.5);
  mapLive = valid;
  document.querySelector(".map-frame").classList.toggle("is-live", valid);
  badge("gazebo-pill", valid ? "实时状态" : "暂无实时数据", valid ? "good" : "muted", true);
  $("map-overlay").hidden = valid;
  if (!valid) {
    $("map-overlay-title").textContent = trail.length ? "航迹数据已中断" : "等待航迹数据";
    $("map-overlay-copy").textContent = telemetryReason(gazebo.reason, "未收到仿真实时状态，请检查仿真是否已启动。");
    metric("altitude", null, "m");
    metric("speed", null, "m/s");
    metric("yaw", null, "°", 0);
    metric("simulation-time", null, "s");
    $("gazebo-position").textContent = "X —   Y —   Z —";
    $("update-age").textContent = "当前数据不可用";
    drawMap();
    return;
  }

  mapPosition = gazebo.position_m.slice(0, 3);
  const stamp = gazebo.stamp_sim_s;
  if (Number.isFinite(stamp) && Number.isFinite(lastSimulationStamp) && stamp < lastSimulationStamp) trail = [];
  if (!Number.isFinite(stamp) || stamp !== lastSimulationStamp) {
    const last = trail[trail.length - 1];
    if (last && Math.hypot(mapPosition[0] - last[0], mapPosition[1] - last[1]) > 10) trail = [];
    trail.push([...mapPosition]);
    if (trail.length > 1500) trail.shift();
    lastSimulationStamp = Number.isFinite(stamp) ? stamp : null;
  }
  const orientation = gazebo.orientation_xyzw;
  let yaw = null;
  if (finiteVector(orientation, 4)) {
    const [x, y, z, w] = orientation;
    const norm = Math.hypot(x, y, z, w);
    if (norm > 0.99 && norm < 1.01) yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  }
  mapYaw = yaw;
  metric("altitude", mapPosition[2], "m", 2);
  metric("speed", finiteVector(gazebo.linear_velocity_m_s) ? Math.hypot(...gazebo.linear_velocity_m_s.slice(0, 3)) : null, "m/s", 2);
  metric("yaw", yaw === null ? null : yaw * 180 / Math.PI, "°", 0);
  metric("simulation-time", stamp, "s", 1);
  $("gazebo-position").textContent = `X ${number(mapPosition[0])}   Y ${number(mapPosition[1])}   Z ${number(mapPosition[2])} m`;
  $("update-age").textContent = Number.isFinite(gazebo.age_wall_s) ? `数据 ${number(gazebo.age_wall_s, 1)} 秒前更新` : "实时数据";
  drawMap();
}

function clearCameraFrame(name, message, status = "图像不可用") {
  const stream = cameraStreams[name];
  const image = $(`camera-${name}-image`);
  image.hidden = true;
  image.removeAttribute("src");
  delete image.dataset.updatedAt;
  if (stream.url) { URL.revokeObjectURL(stream.url); stream.url = null; }
  $(`camera-${name}-overlay`).hidden = false;
  $(`camera-${name}-message`).textContent = message;
  badge(`camera-${name}-status`, status, "muted");
}

function pauseCamera(name, message, status) {
  const stream = cameraStreams[name];
  stream.generation += 1;
  clearTimeout(stream.timer);
  stream.timer = null;
  stream.controller?.abort();
  clearCameraFrame(name, message, status);
}

function cameraCanPoll(name) {
  return apiOnline && !document.hidden && cameraStreams[name].metadata?.available === true;
}

function scheduleCamera(name, delay = 250) {
  const stream = cameraStreams[name];
  if (!cameraCanPoll(name) || stream.controller || stream.timer !== null) return;
  stream.timer = setTimeout(() => pollCamera(name), delay);
}

function renderCameras(cameras) {
  for (const name of CAMERA_NAMES) {
    const stream = cameraStreams[name];
    const metadata = cameras[name] || {};
    stream.metadata = { ...metadata, available: metadata.available === true
      && (!Number.isFinite(metadata.age) || (metadata.age >= -0.05 && metadata.age <= 2.5)) };
    const dimensions = Number.isFinite(metadata.width) && Number.isFinite(metadata.height)
      ? `${metadata.width} × ${metadata.height}` : "等待图像尺寸";
    const rates = Number.isFinite(metadata.source_hz) && Number.isFinite(metadata.preview_hz)
      ? `采集 ${number(metadata.source_hz, 0)} Hz · 网页预览 ≤ ${number(metadata.preview_hz, 0)} Hz` : "等待采集状态";
    $(`camera-${name}-meta`).textContent = `${dimensions} · ${rates}`;
    if (document.hidden) pauseCamera(name, "页面在后台，预览已暂停。", "预览已暂停");
    else if (!stream.metadata.available) pauseCamera(name, "尚未收到有效相机图像，或图像已过期。", "等待相机");
    else scheduleCamera(name, 0);
  }
}

async function pollCamera(name) {
  const stream = cameraStreams[name];
  clearTimeout(stream.timer);
  stream.timer = null;
  if (!cameraCanPoll(name) || stream.controller) return;
  const controller = new AbortController();
  const generation = stream.generation;
  stream.controller = controller;
  const timeout = setTimeout(() => controller.abort(), 1500);
  let nextUrl = null;
  try {
    const response = await fetch(`/api/camera/${name}.jpg`, {
      cache: "no-store", headers: { Accept: "image/jpeg" }, signal: controller.signal,
    });
    if (!response.ok) throw new Error(response.status === 404 || response.status === 503
      ? "相机图像暂不可用，等待恢复。" : `相机图像请求失败（HTTP ${response.status}）。`);
    const blob = await response.blob();
    if (!blob.size || !blob.type.startsWith("image/jpeg")) throw new Error("相机图像格式无效，等待下一帧。");
    nextUrl = URL.createObjectURL(blob);
    const decoded = new Image();
    decoded.src = nextUrl;
    await decoded.decode();
    if (generation !== stream.generation || !cameraCanPoll(name) || controller.signal.aborted) return;
    const previousUrl = stream.url;
    const image = $(`camera-${name}-image`);
    image.src = nextUrl;
    stream.url = nextUrl;
    nextUrl = null;
    image.hidden = false;
    image.dataset.updatedAt = String(Date.now());
    $(`camera-${name}-overlay`).hidden = true;
    badge(`camera-${name}-status`, "实时画面", "good", true);
    if (previousUrl) URL.revokeObjectURL(previousUrl);
  } catch (error) {
    if (generation === stream.generation && !document.hidden) clearCameraFrame(name,
      error.name === "AbortError" ? "相机图像请求超时，正在重连。" : error.message || "无法读取相机图像，正在重连。");
  } finally {
    clearTimeout(timeout);
    if (nextUrl) URL.revokeObjectURL(nextUrl);
    if (stream.controller === controller) stream.controller = null;
    scheduleCamera(name);
  }
}

function renderMission(mission) {
  const signature = JSON.stringify(mission);
  if (signature === missionSignature) return;
  missionSignature = signature;
  const error = $("mission-error");
  const list = $("mission-steps");
  list.replaceChildren();
  error.hidden = true;
  if (!mission) {
    badge("mission-status", "暂无任务");
    $("mission-action").textContent = "等待发布第一条飞行指令";
    $("mission-counter").textContent = "—";
    $("mission-description").textContent = "任务执行后，各步骤的实际结果会显示在这里。";
    $("mission-progress").style.width = "0%";
    return;
  }
  const steps = Array.isArray(mission.steps) ? mission.steps : [];
  const events = Array.isArray(mission.events) ? mission.events : [];
  const finished = new Map(events.map((event) => [event.step, event]));
  const status = mission.status;
  badge("mission-status", STATUS_NAMES[status] || status || "等待状态", {
    running: "active", succeeded: "good", failed: "bad", cancelled: "warning",
  }[status] || "muted");
  $("mission-action").textContent = status === "running"
    ? `正在${ACTION_NAMES[mission.action] || "准备任务"}`
    : status === "succeeded" ? "全部飞行步骤已完成"
    : status === "cancelled" ? "任务已停止，请查看当前飞行状态"
    : "任务执行失败，请查看原因";
  $("mission-counter").textContent = `${finished.size} / ${steps.length} 步完成`;
  $("mission-description").textContent = mission.description || `任务 ${mission.id || ""}`;
  const progress = $("mission-progress");
  progress.style.width = `${steps.length ? Math.min(100, finished.size / steps.length * 100) : 0}%`;
  progress.className = status === "succeeded" ? "done" : status === "failed" ? "failed" : "";
  steps.forEach((step, index) => {
    const stepNumber = index + 1;
    const event = finished.get(stepNumber);
    const active = stepNumber === mission.step && !event;
    const item = document.createElement("li");
    item.className = event ? "done" : active ? status === "failed" ? "failed" : status === "running" ? "current" : "" : "";
    const marker = document.createElement("span");
    marker.className = "step-marker";
    marker.textContent = event ? "✓" : String(stepNumber);
    const copy = document.createElement("span");
    copy.className = "step-copy";
    copy.append(document.createTextNode(ACTION_NAMES[step.action] || step.action || "未知动作"));
    const detail = document.createElement("small");
    if (step.action === "takeoff") detail.textContent = `本地高度 ${number(step.altitude)} m`;
    if (step.action === "goto") detail.textContent = `ENU (${number(step.x)}, ${number(step.y)}, ${number(step.altitude)}) m`;
    if (step.action === "hold") detail.textContent = `持续 ${number(step.duration, 1)} 秒`;
    if (step.action === "return_home") detail.textContent = "回到记录的起飞点上方";
    if (step.action === "land") detail.textContent = "AUTO.LAND，等待落地与上锁";
    copy.append(detail);
    item.append(marker, copy);
    if (event && Number.isFinite(event.finished_at)) {
      const time = document.createElement("span");
      time.className = "step-time";
      time.textContent = new Date(event.finished_at * 1000).toLocaleTimeString("zh-CN", { hour12: false });
      item.append(time);
    }
    list.append(item);
  });
  if (mission.error) { error.textContent = mission.error; error.hidden = false; }
}

function readableValue(value) {
  if (typeof value === "string") return value;
  if (Number.isFinite(value)) return String(value);
  if (typeof value === "boolean") return value ? "是" : "否";
  if (Array.isArray(value)) return value.map(readableValue).filter(Boolean).join("；");
  if (!value || typeof value !== "object") return "";
  return Object.entries(value).map(([key, item]) => `${key}：${readableValue(item)}`).join("；");
}

function locationDescription(location) {
  if (!location || typeof location !== "object" || Array.isArray(location)) return readableValue(location);
  const details = [];
  if (location.frame === "gazebo_world") {
    if ([location.x_m, location.y_m, location.z_m].every(Number.isFinite)) {
      details.push(`Gazebo 世界坐标 X ${number(location.x_m)} / Y ${number(location.y_m)} / Z ${number(location.z_m)} m`);
    }
    if (location.method === "ground_plane_projection") details.push("按地面平面投影估算");
    if (Number.isFinite(location.projection_spread_m)) details.push(`投影范围 ${number(location.projection_spread_m)} m`);
  } else if (location.frame === "camera_optical") {
    details.push("相对相机的方向");
    if (Number.isFinite(location.bearing_right_deg)) details.push(`水平角 ${number(location.bearing_right_deg, 1)}°（向右为正）`);
    if (Number.isFinite(location.elevation_up_deg)) details.push(`俯仰角 ${number(location.elevation_up_deg, 1)}°（向上为正）`);
    if (location.method === "bearing_only") details.push("仅方向，未测量距离");
  } else {
    return readableValue(location);
  }
  if (location.reason) details.push(readableValue(location.reason));
  if (location.assumptions) details.push(`估算前提：${readableValue(location.assumptions)}`);
  return details.join("；");
}

function appendVisionDetail(item, label, value) {
  const text = readableValue(value);
  if (!text) return;
  const paragraph = document.createElement("p");
  const title = document.createElement("span");
  title.textContent = `${label}：`;
  paragraph.append(title, document.createTextNode(text));
  item.append(paragraph);
}

function renderPerception(report) {
  const signature = JSON.stringify(report);
  if (signature === perceptionSignature) return;
  perceptionSignature = signature;
  $("vision-targets").replaceChildren();
  $("vision-limitations").replaceChildren();
  $("vision-report-error").hidden = true;
  $("vision-limitations-block").hidden = true;
  $("vision-result").hidden = !report;
  $("vision-empty").hidden = Boolean(report);
  if (!report) {
    badge("vision-status", "尚未分析");
    $("vision-empty").textContent = "提交目标后，这里会显示模型对实际相机图像的分析结果。";
    return;
  }
  const failed = report.status === "error";
  badge("vision-status", failed ? "分析失败" : "分析已完成", failed ? "bad" : "good");
  const stamp = Number.isFinite(report.captured_at_sim_s) ? `采集时刻 ${number(report.captured_at_sim_s, 2)} s（仿真时间）` : "采集时刻未提供";
  const model = typeof report.model === "string" ? report.model : report.model?.name || state?.vision_model?.name || "视觉模型";
  $("vision-report-meta").textContent = `${model} · ${stamp} · ${report.architecture === "single_vlm" ? "单 VLM 图像分析快照" : "图像分析快照"}`;
  $("vision-report-query").textContent = report.query ? `分析需求：${report.query}` : "";
  $("vision-summary").textContent = report.summary || "";
  if (failed) {
    $("vision-report-error").textContent = `视觉模型调用或分析失败：${readableValue(report.error) || "接口未返回有效结果。"}`;
    $("vision-report-error").hidden = false;
  }
  for (const target of Array.isArray(report.targets) ? report.targets : []) {
    if (!target || typeof target !== "object") continue;
    const item = document.createElement("li");
    const heading = document.createElement("div");
    heading.className = "vision-target-heading";
    const label = document.createElement("strong");
    label.textContent = target.label || "未命名目标";
    heading.append(label);
    if (Number.isFinite(target.confidence) && target.confidence >= 0 && target.confidence <= 1) {
      const confidence = document.createElement("span");
      confidence.textContent = `模型置信度 ${number(target.confidence * 100, 0)}%`;
      heading.append(confidence);
    }
    item.append(heading);
    appendVisionDetail(item, "画面位置", target.image_position);
    if (finiteVector(target.bbox, 4) && target.bbox.slice(0, 4).every((value) => value >= 0 && value <= 1)) {
      appendVisionDetail(item, "图像框（归一化）", target.bbox.slice(0, 4).map((value) => number(value, 3)).join(", "));
    }
    appendVisionDetail(item, "大概位置", locationDescription(target.approximate_location));
    const stereo = target.stereo_location;
    if (stereo && stereo.status === "ok" && finiteVector(stereo.position_enu_m, 3)) {
      appendVisionDetail(item, "双目表面位置（ENU · m）", vector(stereo.position_enu_m));
      if (finiteVector(stereo.sigma_enu_m, 3)) appendVisionDetail(item, "位置标准差估计（m）", vector(stereo.sigma_enu_m));
      appendVisionDetail(item, "定位说明", "采图时刻的可见表面估计，不能直接用作航点");
    } else if (stereo) {
      appendVisionDetail(item, "双目定位", `暂不可用：${readableValue(stereo.reason || "深度质量不足")}`);
    }
    appendVisionDetail(item, "周围环境", target.surroundings);
    $("vision-targets").append(item);
  }
  const limitations = Array.isArray(report.limitations) ? report.limitations : report.limitations ? [report.limitations] : [];
  for (const limitation of limitations) {
    const text = readableValue(limitation);
    if (!text) continue;
    const item = document.createElement("li");
    item.textContent = text;
    $("vision-limitations").append(item);
  }
  $("vision-limitations-block").hidden = !$("vision-limitations").children.length;
}

function drawMap() {
  const canvas = $("flight-map");
  const bounds = canvas.getBoundingClientRect();
  const width = bounds.width;
  const height = bounds.height;
  if (!width || !height) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#f4f7fc";
  ctx.fillRect(0, 0, width, height);
  const maxCoordinate = Math.max(4, ...trail.flatMap((p) => [Math.abs(p[0]), Math.abs(p[1])]));
  const halfRange = Math.ceil((maxCoordinate + 1.5) / 2) * 2;
  const cell = halfRange <= 8 ? 1 : halfRange <= 20 ? 2 : 5;
  const scale = Math.min(width - 60, height - 75) / (halfRange * 2);
  const centerX = width / 2;
  const centerY = height / 2 + 1;
  const project = (p) => [centerX + p[0] * scale, centerY - p[1] * scale];
  $("map-scale").textContent = `每格 ${cell} m`;
  ctx.strokeStyle = "#e6ecf5";
  ctx.lineWidth = 1;
  for (let value = -halfRange; value <= halfRange; value += cell) {
    const x = centerX + value * scale;
    const y = centerY - value * scale;
    ctx.beginPath(); ctx.moveTo(x, 37); ctx.lineTo(x, height - 33); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(25, y); ctx.lineTo(width - 25, y); ctx.stroke();
  }
  ctx.setLineDash([3, 5]);
  ctx.strokeStyle = "#ccd7e8";
  ctx.beginPath(); ctx.moveTo(25, centerY); ctx.lineTo(width - 25, centerY); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(centerX, 37); ctx.lineTo(centerX, height - 33); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "#a0afc5";
  ctx.font = "9px system-ui, sans-serif";
  ctx.fillText("0", centerX + 6, centerY + 13);
  ctx.fillText("+X", width - 42, centerY - 7);
  ctx.fillText("+Y", centerX + 6, 48);
  ctx.fillStyle = "#a8b6cc";
  ctx.beginPath(); ctx.arc(centerX, centerY, 2.5, 0, Math.PI * 2); ctx.fill();
  if (trail.length > 1) {
    ctx.beginPath();
    trail.forEach((p, index) => { const [x, y] = project(p); index ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.strokeStyle = mapLive ? "#7898ef" : "#b5c1d6";
    ctx.lineWidth = 2;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.stroke();
    const [startX, startY] = project(trail[0]);
    ctx.fillStyle = "#fff";
    ctx.strokeStyle = mapLive ? "#91aaed" : "#b5c1d6";
    ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(startX, startY, 3.5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  }
  if (mapLive && mapPosition) {
    const [x, y] = project(mapPosition);
    ctx.fillStyle = "#315eef0b";
    ctx.beginPath(); ctx.arc(x, y, 31, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = "#315eef18";
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(x, y, 25, 0, Math.PI * 2); ctx.stroke();
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-(mapYaw ?? 0));
    ctx.strokeStyle = "#426bee";
    ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(-10, -10); ctx.lineTo(10, 10); ctx.moveTo(-10, 10); ctx.lineTo(10, -10); ctx.stroke();
    for (const [rotorX, rotorY] of [[-10, -10], [-10, 10], [10, -10], [10, 10]]) {
      ctx.fillStyle = "#f4f7fc";
      ctx.beginPath(); ctx.arc(rotorX, rotorY, 5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    }
    ctx.fillStyle = "#315eef";
    ctx.beginPath(); ctx.ellipse(0, 0, 7, 5, 0, 0, Math.PI * 2); ctx.fill();
    if (mapYaw !== null) {
      ctx.beginPath(); ctx.moveTo(19, -3); ctx.lineTo(24, 0); ctx.lineTo(19, 3); ctx.closePath(); ctx.fill();
    }
    ctx.restore();
    ctx.fillStyle = "#657eae";
    ctx.textAlign = "center";
    ctx.font = "9px system-ui, sans-serif";
    ctx.fillText(`Z ${number(mapPosition[2], 2)} m`, x, y + 39);
    ctx.textAlign = "left";
  }
}

async function pollState() {
  clearTimeout(statePollTimer);
  try {
    const data = await request("/api/state");
    apiOnline = true;
    renderState(data);
  } catch (error) {
    apiOnline = false;
    badge("connection-pill", "服务未连接", "bad", true);
    $("connection-banner").className = "notice bad";
    $("connection-banner").textContent = `${error.message} 页面会自动重连。`;
    badge("vehicle-badge", "状态不可用", "muted");
    for (const id of ["flight-mode", "armed-state", "landed-state", "position-ready", "px4-position", "px4-target"]) $(id).textContent = "—";
    $("vio-title").textContent = "VINS-Fusion · 状态不可用";
    $("vio-indicator").className = "vio-indicator";
    $("vio-detail").textContent = "等待服务重连。VIO 当前不参与飞行控制。";
    badge("navigation-status", "状态不可用", "muted");
    for (const id of ["navigation-scans", "navigation-source", "navigation-distance", "navigation-path", "navigation-map"]) $(id).textContent = "—";
    $("navigation-detail").textContent = "控制服务未连接，等待恢复后读取绕障状态。";
    renderGazebo({ available: false, reason: "本机控制服务未连接；图中历史轨迹不代表当前状态。" });
    for (const name of CAMERA_NAMES) pauseCamera(name, "本机控制服务未连接，等待恢复。", "服务未连接");
    updateButtons();
  } finally {
    statePollTimer = setTimeout(pollState, apiOnline ? 700 : 1800);
  }
}

function addMessage(role, text, extraClass = "") {
  $("conversation-empty")?.remove();
  const message = document.createElement("div");
  message.className = `message ${role} ${extraClass}`;
  const heading = document.createElement("div");
  heading.className = "message-role";
  const label = document.createElement("span");
  label.textContent = role === "user" ? "你" : "无人机 Agent";
  const time = document.createElement("span");
  time.textContent = new Date().toLocaleTimeString("zh-CN", { hour12: false, hour: "2-digit", minute: "2-digit" });
  heading.append(label, time);
  const body = document.createElement("div");
  body.className = "message-body";
  body.textContent = text;
  message.append(heading, body);
  $("conversation").append(message);
  $("conversation").scrollTop = $("conversation").scrollHeight;
  return message;
}

function finishCommand(data) {
  clearTimeout(commandPollTimer);
  const wasVision = visionCommandActive;
  commandActive = false;
  visionCommandActive = false;
  activeCommandSource = "command";
  commandId = null;
  try { sessionStorage.removeItem("uav-active-command"); } catch { /* Storage may be unavailable. */ }
  if (pendingReply) {
    pendingReply.className = `message assistant${data.status === "failed" ? " failed" : ""}`;
    pendingReply.querySelector(".message-body").textContent = data.reply || data.error || (data.status === "succeeded" ? "Agent 已完成此次处理，请结合任务进度查看飞行结果。" : "此次指令处理失败。");
    $("conversation").scrollTop = $("conversation").scrollHeight;
  }
  if (data.perception) {
    perceptionSignature = "";
    renderPerception(data.perception);
  } else if (wasVision) {
    perceptionSignature = "";
    renderPerception(null);
    badge("vision-status", data.status === "failed" ? "分析失败" : "未返回视觉报告", "bad");
    $("vision-empty").textContent = data.status === "failed"
      ? `此次视觉指令失败：${data.error || data.reply || "模型未返回有效结果。"}`
      : "此次指令未返回视觉分析报告，请查看 Agent 对话中的处理结果。";
  }
  pendingReply = null;
  updateButtons();
}

async function pollCommand() {
  if (!commandId) return;
  clearTimeout(commandPollTimer);
  try {
    const data = await request(`/api/commands/${encodeURIComponent(commandId)}`);
    if (data.perception) renderPerception(data.perception);
    if (data.status === "succeeded" || data.status === "failed") {
      finishCommand(data);
      return;
    }
    if (pendingReply) pendingReply.querySelector(".message-body").textContent = state?.mission?.status === "running"
      ? `飞行任务正在执行。${ACTION_NAMES[state.mission.action] || "准备任务"} · ${state.mission.step || 0}/${state.mission.steps?.length || 0} 步。`
      : visionCommandActive ? "Agent 正在采集相机图像并调用视觉模型进行分析…" : "Agent 正在读取状态、理解指令并处理任务…";
  } catch (error) {
    if (error.httpStatus === 404) { finishCommand({ status: "failed", error: "未找到此次指令，服务可能已重新启动。请检查飞行状态后再提交。" }); return; }
    if (pendingReply) pendingReply.querySelector(".message-body").textContent = `暂时无法读取执行结果：${error.message} 正在重连；可使用快捷控制。`;
  }
  if (commandId) commandPollTimer = setTimeout(pollCommand, 900);
}

async function submitCommand(prompt, source = "command") {
  if (!prompt || commandActive || !apiOnline || state?.model?.available === false) return;
  if (source === "vision" && state?.vision_model?.configured !== true) return;
  const errorElement = $(source === "vision" ? "vision-command-error" : "command-error");
  errorElement.hidden = true;
  commandActive = true;
  activeCommandSource = source;
  visionCommandActive = source === "vision";
  if (visionCommandActive) {
    badge("vision-status", "正在分析", "active");
    $("vision-result").hidden = true;
    $("vision-empty").hidden = false;
    $("vision-empty").textContent = "正在采集实际相机图像并调用视觉模型，请稍候。";
  }
  updateButtons();
  try {
    const result = await request("/api/commands", { method: "POST", body: JSON.stringify({ prompt, mode: source === "vision" ? "observe" : "task" }) });
    if (!result.id) throw new Error("控制服务未返回指令编号，请检查服务状态。");
    commandId = result.id;
    addMessage("user", prompt);
    pendingReply = addMessage("assistant", visionCommandActive ? "Agent 正在采集相机图像并调用视觉模型进行分析…" : "Agent 正在读取状态、理解指令并处理任务…", "pending");
    if (source === "command") $("prompt-input").value = "";
    try { sessionStorage.setItem("uav-active-command", JSON.stringify({ id: commandId, prompt, source })); } catch { /* Optional reload recovery. */ }
    if (result.status === "succeeded" || result.status === "failed") finishCommand(result);
    else pollCommand();
  } catch (error) {
    commandActive = false;
    visionCommandActive = false;
    activeCommandSource = "command";
    errorElement.textContent = error.message;
    errorElement.hidden = false;
    if (source === "vision") {
      badge("vision-status", "提交失败", "bad");
      $("vision-empty").textContent = `无法提交视觉分析：${error.message}`;
    }
    updateButtons();
  }
}

$("command-form").addEventListener("submit", (event) => {
  event.preventDefault();
  submitCommand($("prompt-input").value.trim());
});

$("vision-form").addEventListener("submit", (event) => {
  event.preventDefault();
  const query = $("vision-query").value.trim();
  if (!query) {
    $("vision-command-error").textContent = "请输入需要描述的场景或想寻找的物体。";
    $("vision-command-error").hidden = false;
    return;
  }
  submitCommand(`请根据当前机载相机图像进行视觉分析：${query} 仅报告可见证据；不要为此次观察起飞或移动无人机。`, "vision");
});

$("prompt-input").addEventListener("keydown", (event) => {
  if (event.key === "Enter" && (event.ctrlKey || event.metaKey) && !event.isComposing) {
    event.preventDefault();
    if (!$("submit-command").disabled) $("command-form").requestSubmit();
  }
});

for (const button of document.querySelectorAll(".example-button")) button.addEventListener("click", () => {
  if (button.classList.contains("vision-preset")) {
    $("vision-query").value = button.dataset.visionPrompt;
    $("vision-query").focus();
    return;
  }
  $("prompt-input").value = button.dataset.prompt;
  $("prompt-input").focus();
});

async function directControl(action) {
  if (controlBusy || !apiOnline || !state?.connected) return;
  controlBusy = true;
  updateButtons();
  const label = action === "stop" ? "停止并悬停" : "立即降落";
  $("control-message").className = "control-message";
  $("control-message").textContent = `正在发送${label}请求…`;
  try {
    const result = await request(`/api/${action}`, { method: "POST", body: "{}" });
    $("control-message").textContent = action === "stop"
      ? result.holding ? "停止请求已接受，执行器已设置当前悬停目标。" : "停止请求已接受；若已在降落，降落会继续。"
      : "降落请求已接受。请观察任务进度，等待落地与上锁。";
  } catch (error) {
    $("control-message").className = "control-message error";
    $("control-message").textContent = `${label}失败：${error.message}`;
  } finally {
    controlBusy = false;
    updateButtons();
  }
}

$("stop-button").addEventListener("click", () => directControl("stop"));
$("land-button").addEventListener("click", () => directControl("land"));
$("clear-trail").addEventListener("click", () => { trail = mapLive && mapPosition ? [[...mapPosition]] : []; drawMap(); });
new ResizeObserver(() => drawMap()).observe($("flight-map"));
document.addEventListener("visibilitychange", () => {
  if (!document.hidden) {
    clearTimeout(statePollTimer);
    statePollTimer = setTimeout(pollState, 0);
    for (const name of CAMERA_NAMES) scheduleCamera(name, 0);
  } else {
    for (const name of CAMERA_NAMES) pauseCamera(name, "页面在后台，预览已暂停。", "预览已暂停");
  }
});
window.addEventListener("pagehide", () => {
  for (const name of CAMERA_NAMES) pauseCamera(name, "预览已暂停。", "预览已暂停");
});

try {
  const previous = JSON.parse(sessionStorage.getItem("uav-active-command") || "null");
  if (previous?.id) {
    commandId = previous.id;
    commandActive = true;
    activeCommandSource = previous.source === "vision" ? "vision" : "command";
    visionCommandActive = activeCommandSource === "vision";
    addMessage("user", previous.prompt || "上一条指令");
    pendingReply = addMessage("assistant", "正在恢复上一条指令的执行结果…", "pending");
    pollCommand();
  }
} catch { /* A bad or unavailable stored value does not prevent the console from loading. */ }
drawMap();
pollState();
