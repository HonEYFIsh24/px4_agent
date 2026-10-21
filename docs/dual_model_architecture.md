# PX4 无人机 Agent：双模型方案架构与实现基线

文档日期：**2026-10-04，Asia/Shanghai**。项目目录：`/home/dut/px4_agent`。

本文记录单 VLM 迁移前已实现、已验收的双模型方案：**DeepSeek 负责自然语言规划和工具调用，Qwen-VL 负责读取真实机载图像；ROS 执行器和 PX4 负责实际飞行。** 本文是历史基线，后续代码可能改用单 VLM，不能把本文中的双模型调用链理解为迁移后运行状态。文末保留关键源码 SHA-256，便于区分版本。

## 1. 已实现的能力与范围

使用者通过中文网页或 CLI 发布需求。基础飞行支持起飞、绝对本地航点、悬停、多航点巡航、返航和降落。网页同时显示飞控状态、实际航迹、真实左右相机画面，以及独立 VINS-Fusion VIO 状态。

视觉任务支持**观察当前一帧左相机图像**：描述场景、寻找指定物体或图案、报告画面位置和可见周围环境。满足严格条件时，程序提供已知平面上的粗略 Gazebo 世界坐标投影。观察模式不授权飞行，且后台硬性阻止模型为这次观察起飞、移动、停止或降落；操作员的直接控制按钮仍有效。

当前没有自主覆盖整个环境的搜索、视觉驱动航线规划、避障、双目目标深度、跨帧目标地图，也没有把 VIO 输入 PX4 飞控闭环。“未发现”表示当前快照中未确认目标，不能推断整个环境中不存在该目标。

## 2. 总体架构

```mermaid
flowchart TB
    U[使用者：中文需求] --> W[网页控制台 :8080]
    W -->|异步指令与任务锁| AU[WebApplication / AuditedClient]
    AU --> A[LangChain Agent / Python 3.11]
    A <--> D[DeepSeek v3.2：规划、选工具、读取文字结果]
    A -->|execute_mission / stop / land| AU
    A -->|inspect_scene| V[VisionAnalyzer]
    V -->|固定左相机快照请求| AU
    AU <-->|本机 HTTP :8765| E[ROS 飞行执行器 / Python 3.8]
    V <--> Q[Qwen3-VL-Plus：文字需求 + JPEG 图像]
    Q --> J[JSON 校验 + 方向 / 地面投影后处理]
    J -->|不含图像编码的视觉报告| A
    J --> AU
    AU -->|经工具证据审计的结果| W
    E <-->|MAVROS 服务、20 Hz 设定点和遥测| M[MAVROS]
    M <--> P[PX4 SITL / 原有本地定位]
    P <--> G[Gazebo Classic：Iris + 场景]
    G -->|ModelStates 真值：仅展示 / 投影 / 离线评价| E
    G --> C[左右相机：640 × 480 / 20 Hz 仿真时钟]
    C -->|图像 + CameraInfo| E
    E -->|新鲜 JPEG，每路预览上限 4 Hz| W
    C --> F[VINS-Fusion：独立双目惯性估计]
    G --> I[Iris 原有 IMU 原始测量 / 250 Hz]
    I -->|原始加速度与角速度| F
    F -->|优化里程计 + tracking_ok| E
```

模型只提交受限工具请求，不直接生成 MAVLink 电机控制，也没有任意 shell、ROS 话题或 PX4 参数修改工具。即使模型生成错误动作，任务协议、遥测新鲜度、审计器和执行器仍独立进行检查。

### 2.1 模块和目录

| 文件 / 目录 | 职责 |
|---|---|
| [`agent.py`](../agent.py) | 模型配置、LangChain `create_agent`、系统提示、工具注册、CLI |
| [`vision.py`](../vision.py) | 独立视觉模型配置、JPEG 快照校验、图像模型调用、JSON 校验、位置后处理、错误脱敏 |
| [`web_server.py`](../web_server.py) | 本机 HTTP 服务、异步指令、并发锁、操作员中断、工具证据审计、页面和 JPEG 接口 |
| [`client.py`](../client.py) | 无 ROS 依赖的执行器 HTTP 客户端；读取状态、取图、提交和等待任务 |
| [`bridge.py`](../bridge.py) | ROS 执行器、仿真身份检查、持续设定点、MAVROS 模式 / 解锁、遥测、相机与姿态缓存 |
| [`mission.py`](../mission.py) | Python 3.8 / 3.11 共用的严格任务协议和边界校验 |
| [`web/`](../web/) | 原生 HTML / CSS / JavaScript 中文控制台，航迹、相机、视觉报告和任务进度 |
| [`launch/uav_system.launch`](../launch/uav_system.launch) | 整套系统的 ROS launch 入口，默认打开 Gazebo GUI |
| [`launch/sitl_vio.launch`](../launch/sitl_vio.launch) | 带视觉传感器的 SITL 入口，传入 Gazebo `--lockstep` |
| [`scripts/start_system.sh`](../scripts/start_system.sh) | 启动顺序、冲突检查、子进程归属、飞控就绪等待、退出前降落和有序关闭 |
| [`scripts/launch_system.sh`](../scripts/launch_system.sh) | 设置 ROS 搜索路径并执行统一 launch |
| [`scripts/stop_system.sh`](../scripts/stop_system.sh) | 核对 PID / 启动时间 / 命令后请求管理器停止 |
| [`vio/`](../vio/) | 模型与纹理生成、标定、输入检查、VIO 补丁、质量判定、验收程序 |
| `vio_ws/src/VINS-Fusion` | 独立 VINS-Fusion 源码工作区 |
| `vio_ws/src/uav_body_imu_bridge` | 同进程转发 Iris 原始 IMU 的 Gazebo 插件 |
| [`tests/`](../tests/) | 任务、执行器相机、网页、视觉格式 / 几何 / 权限、VIO 有效性测试 |
| `logs/system/<时间>-<标识>/` | 每次启动单独的仿真、执行器、网页、遥测和 ROS 日志 |
| `logs/flight.jsonl` | 执行器事件和真实任务过程 |
| `logs/vision/`、`logs/web/`、`logs/launch/`、`logs/vio/` | 各阶段验收记录、截图和定位评价 |

## 3. 运行环境、依赖和配置

### 3.1 环境分工

| 部分 | 基线环境 |
|---|---|
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS Noetic，`/opt/ros/noetic` |
| 仿真与飞控 | Gazebo Classic 11、PX4 v1.14 SITL、MAVROS |
| ROS 执行器 | 系统 `/usr/bin/python3`，Python 3.8，使用本机 `rospy` / `cv_bridge` / OpenCV |
| Agent 与网页后端 | Python 3.11；优先项目 `.venv`，否则使用已有 `/home/dut/ros_agent/mcp-client-langchain/.venv` |
| VIO | ROS Noetic、OpenCV 4.2、Ceres 1.14、本地独立 catkin 工作区 |
| 网页 | 原生前端，无需打包构建；Playwright 仅用于浏览器验收 |

Python 3.8 与 3.11 通过本机 HTTP 交换 JSON，避免让新版 LangChain 依赖与 ROS Noetic 的系统 Python 混装。`AGENT_PYTHON` 可显式指定 Agent 解释器。

基线 [`requirements.txt`](../requirements.txt) 固定为：

```text
langchain==1.1.3
langchain-openai==1.1.1
langgraph==1.0.4
pydantic==2.12.5
python-dotenv==1.2.1
```

### 3.2 双模型配置继承

基线真实验收使用 DashScope OpenAI 兼容接口，规划模型为 `deepseek-v3.2`，视觉模型为 `qwen3-vl-plus`。文件配置和环境变量合并时，**环境变量优先**。

网页默认先用项目 `.env`；若不存在，使用已有 `/home/dut/ros_agent/mcp-client-langchain/.env`。CLI 可用 `--env-file` 指定文件。文档不包含现有密钥，也不要求把密钥放入源码。

| 配置项 | 读取规则 |
|---|---|
| 规划模型 | `LLM_MODEL` → `MODEL` → 代码默认 `qwen-plus`；实际基线配置为 DeepSeek |
| 规划 URL | `LLM_BASE_URL` → `BASE_URL` → `OPENAI_BASE_URL` |
| 规划密钥 | `LLM_API_KEY` → `OPENAI_API_KEY` → `DASHSCOPE_API_KEY` |
| 视觉模型 | `VISION_MODEL`；未设置且 URL 主机名属于 `aliyuncs.com` 时，默认 `qwen3-vl-plus` |
| 视觉 URL | `VISION_BASE_URL` → 传入的规划配置 URL → 原有 URL 变量 |
| 视觉密钥 | `VISION_API_KEY` → 传入的规划配置密钥 → 原有密钥变量 |
| 执行器地址 | `--bridge-url` → `UAV_BRIDGE_URL` → `http://127.0.0.1:8765`；统一启动显式固定到本次执行器 |

**视觉配置只继承接口与凭证，不覆盖规划模型。** 将两项模型名称设成相同名称，并不自动把此架构变成共享图像上下文的单 VLM Agent。

示例只表示配置字段，不是现有凭证：

```dotenv
LLM_MODEL=deepseek-v3.2
LLM_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
LLM_API_KEY=YOUR_API_KEY
VISION_MODEL=qwen3-vl-plus
# 可选；省略时继承上述接口与密钥。
# VISION_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
# VISION_API_KEY=YOUR_VISION_API_KEY
```

缺少规划配置时，状态和相机页面仍能显示，模型指令不可提交。视觉模型未配置时，视觉表单不可用。部署到不同提供商或地域时，需明确指定该地域支持图像输入的模型。

## 4. 统一启动、图形界面与关闭

在 Ubuntu 图形桌面的终端运行：

```bash
cd /home/dut/px4_agent
source scripts/setup_ros.bash
roslaunch --sigint-timeout 180 px4_agent uav_system.launch
```

等价便捷命令：

```bash
bash /home/dut/px4_agent/scripts/launch_system.sh
```

控制页面：<http://127.0.0.1:8080>。这里的 **Gazebo GUI 是实际三维仿真窗口**；网页中的 **航迹监测**是 `/gazebo/model_states` 驱动的俯视航迹与遥测展示。

### 4.1 启动过程

1. 统一 launch 启动一个必需的管理器节点，管理器拥有本次所有子进程。
2. 检查已有 `px4`、`gzserver`、`vins_node` 和本次端口，防止启动另一套冲突系统。来自外层 launch 的 ROS master 只能是除 `/rosout` 外为空的环境。
3. 启动带双目与原机体 IMU 的 Iris、Gazebo GUI / server、PX4 SITL 和 MAVROS。
4. 核对 `/use_sim_time`、Gazebo 服务和本地 SITL MAVROS URL；启动 ROS 执行器。
5. 等待遥测有效、飞机落地并未解锁。默认就绪等待 150 秒。
6. 使用 PX4 本地模块，把本次 MAVROS 链路的 `HEARTBEAT` 设为 **仿真时间 5 Hz**，随后启动独立 VIO 与网页。不会通过重新发布旧遥测伪造新鲜度。
7. 网页接口就绪后打印访问地址；飞行就绪与 VIO 就绪分别判断，基础飞行无需等待 VIO 初始化。

基线图形模式默认 `gui:=true`、`sim_speed_factor:=0.35`；无图形模式默认 1 倍速。慢速设置为本机 Gazebo 渲染与 VIO 计算预留 CPU 时间，**不改变仿真时间内相机 20 Hz、IMU 250 Hz 和飞行速度参数**。Gazebo `--lockstep` 保证物理更新等待相机渲染，这与 PX4 MAVLink 插件的 `enable_lockstep` 是两项设置。

VINS 的 OpenCV 工作线程固定 2；OpenBLAS / OpenMP 各限制 1。低速仿真的真实心跳更密后，执行器仍保留 3 秒现实时间的状态新鲜度要求。显示依赖当前桌面的 `DISPLAY` / `XAUTHORITY`。

常用参数：

```bash
roslaunch --sigint-timeout 180 px4_agent uav_system.launch web_port:=8081
roslaunch --sigint-timeout 180 px4_agent uav_system.launch env_file:=/path/to/provider.env
roslaunch --sigint-timeout 180 px4_agent uav_system.launch gui:=false
roslaunch --sigint-timeout 180 px4_agent uav_system.launch sim_speed_factor:=1
```

### 4.2 退出行为

前台 Ctrl+C 请求管理器退出：先从网页请求降落，必要时回退到执行器；默认最多等待 90 秒落地并上锁，然后关闭本次网页、VIO、执行器与仿真。外层 ROS master 最后退出。`--sigint-timeout 180` 为这个过程预留时间，便捷脚本已经包含此参数。

后台启动和停止：

```bash
mkdir -p /home/dut/px4_agent/logs
setsid nohup bash /home/dut/px4_agent/scripts/launch_system.sh \
  > /home/dut/px4_agent/logs/system-launch.log 2>&1 < /dev/null &
bash /home/dut/px4_agent/scripts/stop_system.sh
```

停止脚本核对 PID、启动时间和实际命令，管理器只关闭其拥有的子进程。关闭浏览器本身不会关闭无人机系统。

## 5. 自然语言到真实飞行

### 5.1 LangChain 调用链

`agent.py` 使用 `ChatOpenAI` 访问兼容接口，通过 LangChain 1.x `create_agent` 注册工具，`InMemorySaver` 保存当前进程内的对话状态。规划调用配置：`temperature=0`、超时 30 秒、最多 1 次接口重试；Agent 调用设置 `recursion_limit=30`、`max_concurrency=1`。内存检查点不是持久任务日志，服务重启后不保留。

自然语言飞行链为：

```text
使用者需求 → DeepSeek 读取状态 / 规划 → execute_mission(结构化完整任务)
→ Web 审计客户端 → ROS 执行器校验 → MAVROS → PX4
→ 真实遥测与 mission/events → 工具结果 → 审计后的网页回复
```

工具清单：

| 工具 | 行为 |
|---|---|
| `get_drone_state()` | 返回飞控状态、本地位置、起飞点、任务、限制及独立 VIO / Gazebo / 相机状态 |
| `execute_mission(steps, description)` | 一次提交完整顺序任务，等待观测到的最终结果 |
| `stop_drone()` | 取消任务；OFFBOARD 飞行请求当前位置悬停；已进入 AUTO.LAND 时继续降落 |
| `land_drone()` | 取消任务、请求 AUTO.LAND，并等待落地 / 上锁 |
| `inspect_scene(query)` | 分析一次真实左相机快照，返回视觉报告；自身没有飞行行为 |

`MissionStep` 使用 Pydantic 严格数值字段及 `extra="forbid"`，执行器仍再次用共享 `validate_mission` 校验整条任务。Agent 提示要求实际调用状态和飞行工具，不能只用文字声称完成。

### 5.2 飞行协议与执行标准

| 项目 | 约束 / 语义 |
|---|---|
| 动作 | `takeoff`、`goto`、`hold`、`return_home`、`land` |
| 任务长度 | 1–24 步；字段必须与动作精确匹配 |
| 起飞和航点高度 | 本地 ENU 绝对 z，1–10 m |
| 航点 x / y | 绝对本地 ENU 米；距本地原点水平半径 ≤30 m |
| 悬停 | 每步 0.1–60 秒，按现实时间计时 |
| 起飞 | 已落地且未解锁；一条任务最多一次 |
| 返航 | 回到执行器记录的起飞点上方，保留当前高度；不隐含降落 |
| 降落 | 只能为最后一步；完成需要落地且 `armed=false` |
| 设定点 | 按现实时间 20 Hz 发布；目标平移最大 1 m/s，限速计算采用仿真时间 |
| 航点完成 | 实测距离 ≤0.25 m 并保持至少 1 秒 |
| 新鲜度 | MAVROS 连接 / 状态 ≤3 秒现实时间，本地位姿 ≤1 秒现实时间 |
| 空闲飞行 | 无任务悬停默认 120 秒后自动降落 |

任务失败终止后续步骤。PX4 的本地定位闭环完成稳定飞行，LLM 不运行实时姿态控制。`MAX_RADIUS` 等任务边界是执行器策略，不等于 PX4 原生围栏，也不代表具备碰撞检查。

### 5.3 网页审计、失败和操作员中断

`WebApplication` 每次只允许一个自然语言指令；新指令与飞行 / 观察共用锁，并发提交返回 HTTP 409。后台异步执行，前端查询指令状态，不让长模型请求阻塞状态页面。

`AuditedClient` 记录工具证据。任务是否成功以 `mission.status`、实际工具错误及操作员中断为准，模型写“完成”不能覆盖失败。一次指令发生飞行失败或提交 / 等待异常后，该指令不能再自动提交 `/mission`；用户核对状态后需发布新指令。降落途径保留。

停止 / 降落按钮绕过模型，调用原始执行器并把当前指令标为中断。模型之后即使迟到返回，也不能继续提交飞行动作；已中断观察的视觉结果被丢弃。最终失败说明由程序生成，避免模型把失败描述为成功。

## 6. 相机、原机体 IMU 与独立 VIO

### 6.1 传感器配置

| 项目 | 基线配置 |
|---|---|
| 左 / 右图像 | `/uav/camera/left/image_raw`、`/uav/camera/right/image_raw` |
| 左 / 右标定 | `/uav/camera/left/camera_info`、`/uav/camera/right/camera_info` |
| 相机 | 同一个 Gazebo MultiCamera sensor；每路 640×480、20 Hz 仿真时钟、水平视场 90°、零畸变 |
| 双目 | 同测量时间戳；基线 0.12 m；右侧 `P[3]=-38.4` |
| 内参 | `fx=fy=320`、`cx=320`、`cy=240` |
| 基座到相机 | 左 / 右 `(0.12, ±0.06, 0.18)` m，俯角 0.35 rad，约 20° |
| 光学坐标 | x 向右、y 向下、z 向前 |
| IMU | Iris 原有 IMU，约 250 Hz 仿真时钟，FLU：x 前、y 左、z 上 |
| 原始 IMU 话题 | `/uav/imu/data_raw` |
| VIO 输入 | 仅左右原始图像、原 IMU 加速度和角速度 |
| VIO 输出 | `/vins_estimator/odometry`，约 10 Hz；另有 `tracking_ok`、IMU 传播输出与偏置 |

IMU 转发插件在 Gazebo 同进程订阅原始 `~/iris/imu` 测量，不新增 IMU、不修改其噪声和偏置。仿真消息中的姿态被丢弃，以 `orientation_covariance[0]=-1` 标记未知。PX4 HIGHRES_IMU 会扣除 EKF 偏置，因此本基线不把 `/mavros/imu/data_raw` 当作独立原始 IMU。

VINS 不读取 GNSS、MAVROS 本地位置、飞控姿态或 Gazebo 真值；不启用 GPS 全局融合或回环。初始化六轴零均值偏置先验来自原 Iris turn-on bias 的标准差：加速度 0.196 m/s²、角速度 0.0087 rad/s，随后进入正常边缘化，不是实际偏置真值。双目入口拒绝重复 / 倒序帧，避免特征速度除以零。

`vio.ready` 要求 tracking 状态和接收时间新鲜、测量时间戳合理、数值有限、单位四元数、速度 ≤3 m/s、位置范数 ≤100 m。优化输出接收延迟 ≤1 秒现实时间，测量年龄在 [-0.05, 0.5] 秒仿真时间。这些健康门限只能判断运行有效性，不能单独证明绝对定位精度。

**VIO 当前仅供展示与 Agent 查询，`used_for_flight_control=false`。** PX4 仍采用原有本地定位；本系统验收不代表无 GPS 飞行已经完成。

### 6.2 网页实时图像

执行器订阅左右原始图像，各路编码上限为 4 Hz 现实时间，JPEG 质量 80，最大 2 MiB；以临时文件原子替换 `logs/vio/camera_live.jpg`、`camera_right.jpg`。两路编码锁独立，预览不改变 VINS 原始图像输入。

前端每路独立轮询 JPEG，使用 Blob URL 显示并释放旧 URL。页面切到后台暂停；图像过期、取图失败或执行器离线时隐藏旧画面，连接恢复后重取。报告中的目标框对应静态采集快照，不叠加到随后变化的直播画面。

### 6.3 模型快照与同帧标定

视觉取图不是抓屏，也不是前端生成的图片。`BridgeClient.capture_camera("left")` 读取执行器固定接口 `/camera/snapshot?side=left`，从内存获取不可变 JPEG 和同一帧元数据。该接口只允许 `left` / `right` 固定枚举，不能读取任意路径或请求任意 URL。

重要检查：

- 快照现实接收年龄在 0–2 秒；执行器取图要求测量年龄在 0–0.5 秒仿真时间，拒绝未来帧。
- `CameraInfo` 必须与图像的时间戳匹配到 `1e-6` 秒内，且尺寸相同；不使用最近的任意标定替代。
- 从最近 512 条 Gazebo 姿态中找与图像最接近的样本，仿真时间差 ≤0.05 秒，现实接收年龄 ≤2 秒。
- 可选 MAVROS 本地位置同样需要 ≤0.05 秒测量差与 ≤1 秒现实接收年龄；缺失时明确设为不可用，不妨碍图像观察。
- JPEG 必须存在、大小合理；视觉工具另检查 Base64、JPEG 头尾和输入文字长度。

快照字段：

| 字段 | 含义 |
|---|---|
| `status`、`side`、`mime_type` | 状态、固定相机侧、`image/jpeg` |
| `width`、`height` | 采集图像尺寸 |
| `stamp_sim_s` | 图像原始 ROS 测量时间 |
| `age_seconds`、`measurement_age_seconds` | 现实接收年龄、仿真测量年龄，时钟不能混用 |
| `image_source`、`image_frame_id` | 原始 ROS 来源和帧标识 |
| `image_base64`、`jpeg_size_bytes` | 真实 JPEG 的内部传输编码和大小；不公开到网页报告 |
| `camera_intrinsics` | 同帧 `fx/fy/cx/cy/K`、畸变、尺寸、来源和时间戳 |
| `camera_pose_world` | 相机世界位置、光学四元数、光轴约定、对齐误差和近似标记 |
| `drone_position_world_m` | 对齐姿态对应的 Gazebo 无人机位置 |
| `drone_position_enu_m`、`drone_position_enu_available` | 可选对齐本地 ENU 位置及是否可用 |
| `drone_position_enu_pose_offset_sim_s` | 本地位置相对图像的测量时间差 |
| `location_frame`、`simulation_only` | 明确为 `gazebo_world`、仿真用途 |
| `pose_alignment` | 方法、姿态时间、姿态减图像偏差、最大允许偏差、近似说明 |

`ModelStates` **没有源时间戳**，程序使用消息接收时的 ROS 仿真时间。因此 `alignment_method="nearest_gazebo_model_states_receipt_time"`、`approximate=true`。50 ms 是接收时间匹配门限，不是硬件同步或源测量误差保证；动态场景中的投影还会受通信延迟影响。

相机世界位姿由 Gazebo 机体姿态和实际 SDF 安装外参计算：

```text
t_world_camera = t_world_body + R_world_body · t_body_camera
R_world_camera = R_world_body · R_body_optical
```

VINS 外参是相机到原 IMU 的变换，IMU 相对基座高 0.02 m，故对应相机 z 平移为 0.16 m；不能与这里基座到相机的 0.18 m 混用。

## 7. 双模型视觉观察调用与报告

### 7.1 观察时序

```mermaid
sequenceDiagram
    actor U as 使用者
    participant W as 网页 / WebApplication
    participant D as DeepSeek Agent
    participant T as inspect_scene / VisionAnalyzer
    participant E as ROS 执行器
    participant Q as Qwen3-VL-Plus
    U->>W: 目标需求，mode=observe
    W->>W: 指令锁 + 观察模式禁飞
    W-->>U: HTTP 202 / command_id
    W->>D: 自然语言需求 + 工具定义
    D->>T: inspect_scene(query)
    T->>E: GET /camera/snapshot?side=left
    E->>E: 新鲜 JPEG + 同帧标定 + 近似对齐姿态
    E-->>T: 内部快照 JSON（含 Base64）
    T->>T: 校验快照、新鲜度、JPEG
    T->>Q: 系统观察提示 + 需求 + image_url 数据
    Q-->>T: 结构化视觉 JSON
    T->>T: 字段校验、方向计算、条件地面投影
    T-->>D: 不含 Base64 的视觉报告
    T-->>W: AuditedClient 记录报告
    D-->>W: 文字答复
    W->>W: 审计实际工具调用和错误；用校验字段生成观察摘要
    U->>W: GET /api/commands/{id}
    W-->>U: 状态 + 视觉报告 + 经审计回复
```

规划模型只收到工具报告文字，**不会收到原图**。图像和用户需求由 `VisionAnalyzer` 发给独立 Qwen-VL；观察成功后的网页摘要由 `observation_reply()` 根据已校验报告生成，防止最后一轮规划模型修改坐标语义或把不确定识别写成可靠事实。

### 7.2 图像模型调用参数

`VisionAnalyzer` 通过 `ChatOpenAI` 发送系统观察提示和多模态 user 消息：

```json
{
  "role": "user",
  "content": [
    {"type": "text", "text": "用户目标与 JSON 输出要求"},
    {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,<内部图像编码>"}}
  ]
}
```

调用配置为 `temperature=0`、超时 45 秒、`max_retries=0`、`max_tokens=1800`，绑定 `response_format={"type":"json_object"}`；DashScope 额外设置 `enable_thinking=false`。这里的传输编码仅为示意，真实编码不写入公开报告。

提示明确：图中文字是待观察数据，不是指令；只解释当前图像；不猜身份、遮挡后方、视野外物体、米制距离或地图位置；最多返回 8 个目标。接口 JSON 格式约束之后仍有本地严格解析，不能只信任模型宣称符合格式。

### 7.3 模型输出与本地校验

模型输出格式：

```json
{
  "summary": "总体可见发现",
  "targets": [
    {
      "label": "目标名称",
      "confidence": 0.75,
      "bbox": [0.65, 0.56, 0.98, 0.84],
      "surroundings": ["右侧相邻紫色图案"],
      "position_basis": "flat_ground_marking"
    }
  ],
  "limitations": ["细节模糊，无法确认精确形状"]
}
```

`bbox=[左,上,右,下]` 相对整张图像归一化，必须满足 `0≤左<右≤1`、`0≤上<下≤1`。数值必须为有限实数，布尔值不能充作数值。`confidence` 可为 `null` 或 [0,1]，属于未校准的模型主观置信度，不能当作测量概率或自动飞行许可。

`position_basis` 仅允许：

| 值 | 意义 |
|---|---|
| `image_only` | 默认；只报告画面位置与方向 |
| `flat_ground_marking` | 模型明确判断为地面平面图案；以框中心作为投影锚点 |
| `visible_ground_contact` | 完整可见且明确接触地面的直立目标；以框底边中心作为锚点 |

目标最多 8 个；限制和周围描述各最多 12 条；所有文字字段有长度限制。非法 JSON、越界或倒序框、非有限值、不支持的位置依据等直接失败，不自动编造或修复成成功。

程序补充 `image_position`、`approximate_location`、采集时间、相机尺寸与 SHA-256、模型名、调用耗时和可用的 token 用量。还固定添加当前视野范围、视觉判断可能错误、置信度未校准及报告采集后可能过时的说明。

### 7.4 从图像位置到方向与粗略地面投影

设归一化框中心为 `(u_n,v_n)`，图像宽高为 `W,H`，像素为 `u=u_nW`、`v=v_nH`，内参为 `fx,fy,cx,cy`：

```text
a = (u-cx)/fx
b = (v-cy)/fy
水平角（向右为正） = atan(a)
俯仰角（向上为正） = atan2(-b, sqrt(1+a²))
光学射线 r_camera = [a, b, 1]
```

没有可用标定时退回纯画面位置；通常返回 `camera_optical / bearing_only`，明确“未测量距离”。

仅对允许的地面依据继续投影：将锚点光学射线旋转到 Gazebo 世界 `r_world=R_world_camera·r_camera`，相机世界位置为 `C`，本场景地面纹理平面为 `z_g=0.003 m`：

```text
λ = (z_g-C_z)/r_world,z
P_world = C + λ · r_world
```

此处射线无须单位化；λ 是上述参数化射线的系数，程序另计算实际距离。投影还要求：目标未触及图像边缘裁切门限、位姿对齐误差 ≤0.05 秒、光学坐标约定明确、四元数有效、畸变为零、相机高于该平面、射线可靠朝下（z 分量 <-0.05）、估计距离 ≤30 m、投影 x / y 各在本场景 ±10 m 内。任一条件失败退回方向报告并说明原因。

结果四舍五入到厘米仅是显示精度，**不代表厘米级定位精度**。模型对地面接触点或图案类别的误判会影响结果，地面高度、姿态同步近似、边界框误差都会传播到投影。此方法没有实测深度，不是目标三维重建，也没有独立的目标位置精度验收。

### 7.5 三种坐标不能直接互换

| 坐标 | 来源与用途 | 约束 |
|---|---|---|
| `image` / `camera_optical` | 像素、画面位置、相对光轴方向 | 没有距离，不是飞行航点 |
| `gazebo_world` | 仿真机体 / 相机位姿与条件地面投影 | 用于展示与仿真粗估；与 PX4 本地原点不同 |
| MAVROS 本地 ENU | PX4 本地定位；x 东、y 北、z 上 | 当前唯一飞行任务坐标；z 为本地高度，不等于离地高度 |
| VIO `world` | VINS 初始位置与航向定义的独立坐标 | 可重初始化，无现成 ENU 对齐；当前只供查询 |

视觉工具不输出可直接执行的航点。把 `gazebo_world` 或 `position_vio_m` 直接填入 `goto` 会混淆原点和方向，当前系统明确禁止。

## 8. HTTP 接口、公开信息与权限边界

所有 HTTP 服务默认仅监听 `127.0.0.1`。

| 服务 | 方法 / 路径 | 请求 / 用途 |
|---|---|---|
| 执行器 :8765 | GET `/state`、`/health` | 实时执行器状态 |
| 执行器 :8765 | GET `/camera/snapshot?side=left\|right` | 内部固定相机快照，含 Base64 与标定 / 位姿 |
| 执行器 :8765 | POST `/mission` | `{"steps":[...],"description":"..."}` |
| 执行器 :8765 | POST `/stop`、`/land` | 请求取消悬停 / 立即降落 |
| 网页 :8080 | GET `/api/state` | 执行器状态 + 规划 / 视觉模型元信息 + 最近视觉报告 + 活动指令 |
| 网页 :8080 | POST `/api/commands` | `{"prompt":"...","mode":"task\|observe"}`；接受返回 202 |
| 网页 :8080 | GET `/api/commands/{id}` | 查询 `running/succeeded/failed`、回复、错误和视觉报告 |
| 网页 :8080 | POST `/api/stop`、`/api/land` | 操作员直接中断与控制；绕过模型 |
| 网页 :8080 | GET `/api/camera/left.jpg`、`right.jpg` | 新鲜单张 JPEG；超过 2 秒文件年龄返回 404 |
| 网页 :8080 | GET `/api/camera.jpg` | 兼容旧接口，等同左图 |

网页请求正文限 16 KiB，指令限 1–2000 字符；视觉工具 query 限 1500 字符。网页核对 Host、本机同源 Origin / Fetch-Site，静态资源限定到 web 根目录；响应禁止缓存，设置 CSP 和禁止 iframe 等头部。

**观察模式权限由 `AuditedClient.request()` 强制检查。** 在 `mode="observe"` 的指令内，`/mission`、`/land`、`/stop` 全部被拒绝，不能仅靠系统提示阻止动作。操作者直接按钮调用原始客户端，因此保留紧急控制能力。当前 CLI 的工具没有网页观察模式的这层硬性权限状态，CLI 视觉独立探测应使用 `--check-vision`，其本身不经过飞行工具。

`clean_perception()` 递归删除 `image_base64` / `api_key`，含 `data:image/` 的文字被替换。公开视觉报告包含图片 SHA-256 和尺寸 / 时刻，能够证明来源而不携带完整图像编码。模型元信息仅公开模型名、接口主机名和配置状态。

视觉错误保留 HTTP 状态、异常类型及已脱敏 request_id：400 表示图像支持或参数问题，401 认证，403 权限 / 额度，404 模型或地域不可用，429 限流 / 额度；超时、连接失败和非法响应也明确失败。代码隐藏已配置密钥及图像数据，不把异常当作识别成功。

## 9. 已有验收证据与评价限度

以下均来自迁移前已保存记录，**本文整理时没有重新运行飞行或调用模型**。

### 9.1 本次视觉验收：2026-10-04

来源：[`logs/vision/validation.json`](../logs/vision/validation.json)、[`live_validation.json`](../logs/vision/live_validation.json)。运行目录为 `logs/system/20261004-151510-BfeqhT`。

| 项目 | 已记录结果 |
|---|---|
| 是否真实调用 | 规划和视觉接口均真实调用；图像来自新鲜左 ROS 相机快照 |
| 规划 / 视觉模型 | `deepseek-v3.2` / `qwen3-vl-plus` |
| 观察命令 | `fe24cfceb0184c8c`，最终 `succeeded` |
| 指令端到端时间 | `finished_at-created_at=26.423652 s` |
| 视觉工具时间 | `analysis_duration_seconds=14.25 s`；从工具开始计时，包含取图、校验、模型调用和后处理，不是单独网络往返计时 |
| 图像 | 左图，640×480，采集仿真时间 16.152 s |
| 相机新鲜度 | 现实接收年龄约 0.108 s，仿真测量年龄约 0.064 s |
| 模型返回候选 | 4 个绿色地面图案，主观置信度 0.75 / 0.65 / 0.60 / 0.55 |
| 位置方式 | 3 个条件地面投影；1 个框触及左边缘，自动退回仅方向 |
| token 用量 | 视觉输入 703，输出 483，总计 1186；不是整个双模型任务的总用量 |
| 飞行 | 未执行；74 次采样均未解锁，最终 `landed_state=1`、`armed=false`，任务未改变 |
| 网页 | 桌面与移动端报告显示，无 JavaScript 错误 |
| 接口错误 | 该次视觉联调未遇到模型调用错误 |

4 个候选表示模型返回的观察，不是经过独立人工标注评估的 4 个正确检测。也没有从单次任务得出模型准确率、平均延迟或长期成本。端到端时间减去视觉工具时间的部分包含 Agent 调用、调度等，不能全部归因于某一个模型。

首次独立探测 [`model_probe.json`](../logs/vision/model_probe.json) 记录视觉工具 14.52 秒，图像为 [`model_input.jpg`](../logs/vision/model_input.jpg)。红色 / 粉红色区域主观置信度仅 0.30–0.40，报告明确画面模糊、形状难以确认，因此不能视为可靠具体物体识别。

### 9.2 单元测试与前端契约

基线 Python 3.11 环境 **65 项测试全部通过**。ROS Python 3.8 下 63 项通过，2 项依赖 dotenv 的配置测试跳过。覆盖任务边界、VIO 健康、同帧相机数据、新鲜度、视觉 JSON / 投影 / 错误脱敏、观察禁飞、操作员中断、失败禁止自动重试和网页 HTTP 行为。

[`logs/web/vision_contract_validation.json`](../logs/web/vision_contract_validation.json) 是使用测试数据的独立前端契约检查；它不调用真实模型、不执行真实飞行，不能替代上述真实视觉报告。

### 9.3 之前的图形飞行与 VIO 验收

2026-10-03 [`logs/launch/validation.json`](../logs/launch/validation.json) 记录真实 GUI、基础飞行、停止 / 降落和相机：GUI 仿真 0.35 倍速、OpenCV 2 线程、心跳 5 Hz；465 次健康采样中飞控、VIO 与双目预览有效，311 次为空中采样，VIO 测量最大延迟 0.38 仿真秒。这里当时的单测数量是 26，后来视觉模块加入后才增至 65，不能混淆验收阶段。

更早的独立 VIO 精度报告 [`logs/vio/validation.json`](../logs/vio/validation.json) 对应任务 `e88ad5dd4d42`：80.6 仿真秒、807 个优化位姿，初始平移和航向对齐后的三维 RMSE 0.1065 m、最大误差 0.1522 m、落地 30 秒漂移 0.0105 m、无跟踪重置。该报告属于当时构建版本；不能把其误差数值直接归到后续改动后的任意二进制，也不代表视觉目标定位精度。Gazebo 真值只用于离线评价，未输入 VINS。

## 10. 使用与排障

### 10.1 用户操作

网页基础飞行示例：

```text
查看无人机当前状态。
起飞到本地高度 2 米，飞到本地坐标 (2,0)，悬停 2 秒，返回起飞点后降落。
```

在“机载相机 → 视觉分析”输入：

```text
寻找当前视野中清晰可见的绿色地面图案，报告画面位置以及周围有什么；看不清时如实说明。
```

网页提交 `mode="observe"`，取当前视野，保留观察禁飞。若要换视角，应先发布明确的飞行任务，到达并稳定后再观察；本基线不会自动搜索未见区域。

CLI 检查：

```bash
cd /home/dut/px4_agent
bash scripts/run_agent.sh --status
bash scripts/run_agent.sh --env-file /home/dut/ros_agent/mcp-client-langchain/.env --check-model
bash scripts/run_agent.sh --env-file /home/dut/ros_agent/mcp-client-langchain/.env \
  --check-vision --vision-query '寻找绿色地面图案，报告画面位置及周围有什么。'
```

`--check-vision` 直接真实调用视觉模型，输出不含 Base64 的 JSON 报告；成功退出 0，失败退出 1。检查模型连接会产生真实接口请求。

### 10.2 复现验收

```bash
# 单测，无需真实模型或运行中的仿真。
/home/dut/ros_agent/mcp-client-langchain/.venv/bin/python -m unittest discover -s tests -v

# 前端契约测试，使用测试数据。
UAV_BROWSER_PATH=/usr/bin/microsoft-edge node scripts/test_vision_ui.mjs

# 真实网页视觉调用：要求仿真已运行、飞机事先落地且未解锁。
UAV_BROWSER_PATH=/usr/bin/microsoft-edge node scripts/test_vision_live.mjs

# 真实自然语言和飛行控制测试：会实际飞行。
UAV_BROWSER_PATH=/usr/bin/microsoft-edge npm run test:ui -- --flight
```

### 10.3 常见问题定位

| 现象 | 优先核对 |
|---|---|
| 启动提示冲突 | 停止已有系统；检查端口 8080 / 8765、已有 Gazebo / PX4 / VINS 和 ROS master |
| 没有 Gazebo 窗口 | 桌面终端、`gui:=true`、有效 DISPLAY / XAUTHORITY；查本次 `sitl.log` |
| 飞控不就绪 | `/use_sim_time`、本地 SITL URL、MAVROS 连接 / 位姿、`bridge.log` / `telemetry.log`；不要放宽新鲜度门限掩盖延迟 |
| 相机画面不可用 | 左右原始 ROS 话题、仿真时钟、JPEG 文件新鲜度；查看 `/api/state.camera` |
| 快照失败但直播存在 | 同帧 CameraInfo、≤50 ms 姿态对齐、快照测量年龄；直播与严格分析快照标准不同 |
| VIO 初始化或过期 | 原始 IMU / 双目频率、精确同步、纹理、VINS 运行与计算积压；查看 `logs/vio/inputs.json` / `vins.log` |
| 视觉配置未就绪 | `VISION_MODEL`、提供商 / 地域、继承 URL 和密钥的规则；网页只公开配置状态，不验证账户额度 |
| 401 / 403 / 404 / 429 | 按报告中的认证、权限、模型地域或限流原因检查；不要发送密钥给他人 |
| 模型超时或非法 JSON | 查看明确错误及脱敏 request_id；本次任务应失败，人工重新发起，不能把旧报告当作新结果 |
| 看到了目标但没有米制坐标 | 深度未知、地面依据不成立、边缘裁切、姿态 / 标定不合格或超出已知平面范围；方向结果是正常退化 |
| 停止后模型仍在推理 | 接口请求可能尚未结束，但动作被中断标记拦截；迟到视觉结果也会被丢弃 |

## 11. 下一阶段单 VLM 迁移应保留的模块

本文记录时，单 VLM 尚未实现。迁移目标是让同一高层 VLM 直接接收用户需求、真实图像、遥测和工具历史，承担视觉理解与任务决策，减少双模型之间的文字摘要信息损失。只更换模型名称或仍通过另一个独立视觉调用返回文字报告，不足以完成这个目标。

迁移应保留已有独立执行器和任务协议、同帧快照与标定、时钟 / 姿态对齐、视觉字段校验、几何后处理、坐标语义、观察模式权限、工具审计、操作员中断、失败停止、真实图像来源记录、统一 launch 和 VIO 独立链路。模型合并不自动增加深度、地图、避障或 VIO 闭环能力。

单 VLM 应重新验证工具调用和图像输入能否共同工作，观察模式实际禁飞，普通飞行是否仍正确执行，图像历史不会无界累积，公开接口不泄漏图像编码 / 密钥，以及错误是否明确呈现。随后才适合增加有界多视野搜索、跨帧目标关联和经校准的空间定位。

## 12. 基线源码指纹

以下 SHA-256 在整理本文时读取，供对照迁移前版本；不包含配置文件或凭证。链接指向工作区路径，迁移后文件内容可能已更新。

| 文件 | SHA-256 |
|---|---|
| `agent.py` | `a14cd23b4310aee59c215c92ce77fffe289b371aabde5177426fce8a5c64a529` |
| `vision.py` | `3891f0309f5433105c3bd8b4072bb71d9421c806897b50d32e2e8ebb4390eb06` |
| `web_server.py` | `cd831dffec4e782b95abf7dae98f7a50d52cbb62c90f714599f05ad7c3de9489` |
| `bridge.py` | `3b373c67d3ea9c64077f5193ef41a901a7560cf98a91ebcc5755cfb3768d2905` |
| `client.py` | `5b3665d0ba275eb6b43c0e549c4d7e176910cb1131dd888bd91b26e6b4de407a` |
| `mission.py` | `133cb1b95a56b3b437232405004e0f922ba2c4e4a49b1539650d6f65ebb8213c` |
| `requirements.txt` | `4f5994bcb9a94fdffa92acb6503ce3236fa72c8bc9f43c1ff57a053882655b98` |
| `launch/uav_system.launch` | `140864b031165dd0714024da9abf1bc667b4c318a2f40861adfb4d66f5f30a46` |

相关说明：[项目 README](../README.md)、[VIO 实现说明](../vio/README.md)、[Qwen-VL OpenAI 兼容图像接口](https://help.aliyun.com/zh/model-studio/qwen-vl-compatible-with-openai)、[Gazebo 传感器 lockstep](https://classic.gazebosim.org/tutorials?cat=sensors&tut=lockstep_physics_sensors)。
