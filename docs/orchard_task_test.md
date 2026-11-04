# 平地果园接入与任务验证（2026-11-04）

## 最新：相机相对调整、候选拒绝恢复与实际返航

实例 `20261104-134236-GIk3ie`，指令 `b0434cd3e4cb4765`，仍为平地果园/种子17/qwen3.7-plus/20 m 上限，原样任务 `请为我拍摄一张俯瞰整个果园的照片。`。世界 SHA256 `857a9639be2c2deb829c0acadca9023533afa7f0d9b95090c16a20fbe2632e4d`。启动前文本、工具、真实图像接口以及连续10次地面状态预检通过；没有人工航点、替换模型输出或录像。

本次取得了以下实际证据：

1. 起飞 `78208b64b253` 至8 m，并保存第一张照片/报告。
2. 模型请求后退5 m、升至12 m。实际相机水平方向接近 +X，计算目标约 `[-5.0134, 0.0367, 12]`；超出许可区域，执行器返回显式未提交结果。没有创建该移动任务，也没有因拒绝降落。
3. 模型改选垂直升至12 m，`0dcf4f6416a9` succeeded，取得第二张图。
4. 相机向右3 m并升至15 m，`c8edbf06623d` succeeded；目标约 `[0.0064,-2.9565,15]`，与实际相机右方向一致，取得第三张图。
5. 再垂直升至18 m，`7ec101f8021e` succeeded，取得第四张图。四次报告和构图复核均保存，20次模型调用；仅1次路径拒绝。没有目标搜索/NBV，不将普通相对取景记作票据执行。
6. 构图 limited 后，程序提交 `return_home + land`，任务 `0c58640680a3` succeeded。返航起始距真实 home 约3.003 m，静态规划路长约3.005 m；返航动作完成后再开始降落，最终实际 `armed=false / landed_state=1`，遥测距 home 约0.102 m。`photo_return.status=ok`、`safety_landing=null`，不是独立故障原地降落。

**任务目标仍未完成**：四张原图的近端/右侧仍有裁切，不能宣布“整个果园入镜”。应用 failed，原因 `拍摄构图目标未完成：limited`，没有用返航成功覆盖拍摄结果。复核仍将白色背景描述为裁切依据，这部分视觉理由不可靠；真实画面有其他裁切证据，但不能据此说模型判断已经完全正确。没有继续改提示词或扩大区域。

触地阶段 EV 融合健康短暂变差，完整状态序列保留；最终 healthy=true、reason=null、无最终坐标重置锁存。按用户当前要求不扩大触地异常专项，不宣称此异常已修复。实例落地后已按身份关闭，PX4 参数恢复 success=true 且 actual==target，VIO YAML 恢复启动前副本。

证据 `logs/setup/orchard-relative-return-20261104/`：`summary.json`、`flight-events.json`、`states.jsonl`、`final-state.json`、`image-evidence.json`、`run/result.json`。`photo-01.jpg`～`photo-04.jpg` 是与各报告 SHA256 一致的原图副本；现场桌面截图在 `run/`，无录像。先前失败实例 `20261104-133156-ImCjC2` 原始记录保留在 `logs/setup/orchard-relative-test-20261104/`，不能用本次结果改写。

相关联合回归 **352项通过，0失败/错误/跳过，72.971秒**；命令及代码机制见 [拍摄闭环](photo_tasks.md)，日志 `logs/setup/photo-candidate-return-20261104/regression.log`。方向转换、有限拒绝恢复、实际返航已验证；全园照片未完成，二者分别交付。

## 最新：拍摄闭环优化后的真实重飞

原样任务 `请为我拍摄一张俯瞰整个果园的照片。`，模型 qwen3.7-plus，平地果园、种子 17、20 m 上限；没有补发指令或指定航点。独立实例 `20261104-130816-F8E7Vm`，指令 `3729be9589da4750`。文本/工具/真实地面图像预检及连续地面状态检查通过。先前启动 `20261104-130723-CkfxMr` 在显示辅助进程校验 manager 身份时退出，未启动飞控或起飞；改用启动脚本绝对路径后通过，失败启动日志保留，没有修改身份校验。

本次真实完成 **1 次起飞、3 次普通 goto 换位、4 次采图/报告和 4 次原图构图复核**，21 次模型调用均为 qwen3.7-plus。未启动目标搜索或 NBV；不能把普通 goto 记为票据执行。模型自行选择的路线与实际拍摄高度如下：

| 次数 | 模型目标 ENU | 实际采图高度 | 复核 |
| --- | --- | --- | --- |
| 1 | 起飞至 10 m | 10.081 m | adjust |
| 2 | (0, -8, 15) | 15.048 m | adjust |
| 3 | (0, -15, 18) | 18.022 m | adjust |
| 4 | (0, -20, 20) | 20.026 m | limited |

所有起飞和换位任务 succeeded；没有飞行或视觉工具失败。照片仍裁切近端/部分侧边，未满足全园入镜，最终应用准确标为 failed，原因 `拍摄构图目标未完成：limited`。已证明优化后能够根据复核继续真实移动和采图，不能据此说已达到拍摄目标。

**本次定位到的具体问题**：模型把负 Y 的移动描述为“后退”，但按实际拍摄机体 ENU 四元数和已有相机安装标定计算，四帧机头方向均约 ENU +X（yaw 约 -0.008～-0.009 rad），相机光轴约 `[0.94, -0.005, -0.34]`。这三次位移主要是横向移动，并非沿相机水平方向后退。分析不使用场景语义真值，保存于 `camera-direction-analysis.json`。应优先补足模型可读的当前 ENU 朝向/相机方向和确定性的相对动作转换，而不是继续让模型默认负 Y 等于后退。

还有两项视觉/语言误解需保留：模型把“俯瞰”收紧成“必须垂直俯拍”，用户并未要求垂直；白色背景本身不等于果园边界被裁切，不能只凭白边判断。最后声称 y=-20 为“最远后退位置”也没有许可依据，实际通用水平半径仍为 30 m。本轮未改提示或规则强行通过，照片和错误理由均原样保留。

构图 limited 后由**应用独立失败收尾**执行降落 `393694d793d6`，不是模型调用 land_drone，也没有返航动作。实际 `armed=false`、`landed_state=1`，EV healthy=true、frame_reset_latched=null；本次没有最终速度重置锁存。任务未达标与飞行安全收尾分别记录。

证据：`logs/setup/orchard-photo-loop-20261104/`，含 `summary.json`、`run/result.json`、`preflight.json`、`states.jsonl`、`final-state.json`、`camera-direction-analysis.json`；`photo-01.jpg` 至 `photo-04.jpg` 均为核对 SHA256 后的原图副本。实际桌面截图有 `run/04-first-observation.png`、`07-second-observation.png`、`10-terminal-grounded.png`。没有录像、没有修改生产代码或重跑回归；前一轮 216 项通过是代码验证，不能冒充本轮新跑。实例使用已有管理器停止，检查参数恢复后恢复启动前 VIO 配置，保留模型配置。

## 最新：qwen3.7-plus 原句对照

按用户明确要求，将本机 `.env` 的 VLM_MODEL 与 VISION_MODEL 改为 `qwen3.7-plus`，保留原接口、密钥、single_vlm 架构和其他运行参数。新配置持续保留，不自动切回旧模型。官方[模型说明](https://www.alibabacloud.com/help/en/model-studio/qwen3-7-plus)列出图像输入；本机实际文本、typed_view 工具及地面真实图像三个独立预检均通过。

仍原样提交 `请为我拍摄一张俯瞰整个果园的照片。`，无附加步骤。实例 `20261104-112645-koDWi4` / 指令 `1a95675511e9448d`、果园和种子 17 不变。全部 5 次任务模型调用审计均为 qwen3.7-plus，没有静默回退。

新模型自行选择 10 m，起飞任务 `893dde7ccd3e` succeeded；拍摄并保存 1 份报告，capture `67b7c167bf144af789dc204159cd88b7`，实际 ENU 高度 10.120 m。它准确指出当前为前向相机而非垂直俯视，随后仍宣布任务完成，没有换位或继续升高。原图仍裁切近侧/右侧果园，**本次换模型没有解决全园构图的完成检查问题**。这是一轮个例，不据此宣称两模型总体性能相同或无提升。

模型明确停在空中悬停，没有自行降落。先保留其终态，再由验证流程单独降落，任务 `9d24f0e1cbfd` succeeded，实际 armed=false、landed_state=1。独立收尾中再现已知速度重置锁存，按用户要求记录、不开展额外诊断或放宽门禁。该收尾不是 Agent 自主规划。

证据 `logs/setup/orchard-qwen37-20261104/`：`model-change.json` 只保存模型名称变化，不保存凭据；`model-preflight/`、`image-preflight/` 为独立接口检查；`run/result.json`、`agent-completed-before-cleanup.json`、`states.jsonl`、`summary.json`、`test-cleanup.json` 为真实任务记录；便捷原图 `orchard-qwen37.jpg` 已核对哈希。实例关闭后恢复 PX4 参数和启动前 VIO 配置，保留新模型。无录像、无算法修改、没有重跑代码回归；`.env` 与日志/照片不提交仓库。

## 最新：只使用用户原句的测试

用户要求去掉此前附加的高度、原地拍摄和步骤约束。本轮通过原桌面入口**原样提交**：`请为我拍摄一张俯瞰整个果园的照片。`。没有另加飞行路线、目标高度、完整性提醒或降落要求，也未改系统提示词；现有 20 米许可与安全检查保持不变。

独立实例 `20261104-111752-AP7WFt` / 指令 `0c70ca542ffc44bc`，种子 17。Agent 自选 5 米，执行起飞任务 `4e0adc08ff6b` succeeded，然后 capture_scene、report_scene，5 次模型调用后宣布完成。实际保存照片 `4e026ed6cd9141409dce1f790fd96729`，拍摄 ENU 高度 5.084 m。原图仍裁切近侧与右侧果园，不能证明全园覆盖；Agent 没有继续升高/换位，也没有检验“整个果园”是否完整入镜。应用 succeeded 只表示工具链正常结束，**用户要求的全园构图目标没有得到验证或满足**。单次测试不能证明提示词对其他运行完全没有影响。

Agent 最终停留在空中，没有自行降落。本轮没有向其补发提示；先保存 `agent-completed-before-cleanup.json`，再由验证流程通过拥有者校验 `/land` 独立收尾，任务 `6a9b9a3ce2d2` succeeded，实际落地上锁。这次收尾必须与 Agent 原计划分开，不能写成模型自主完成返航降落。降落中曾短暂出现融合不健康，最后 EV healthy=true、无坐标锁存；按用户要求保留记录、不展开新的接触诊断。

证据 `logs/setup/orchard-simple-prompt-20261104/`：`run/start.json` 和 `run/result.json` 保存原句与调用，`orchard-simple-prompt.jpg` 是校验哈希后的原图副本，`summary.json`、`states.jsonl`、`test-cleanup.json` 保存真实结果与独立收尾来源。无录像，无生产代码改动，无新的回归运行。实例按身份关闭并恢复参数与 VIO 配置。

本次明确暴露的是普通拍摄任务缺少构图目标的完成检查：保存一张照片不等于已满足“整个果园入镜”。后续优化应基于这个真实失败样例，而不是继续向用户指令塞入预先指定的航点步骤。本轮没有立即修改算法或为改变结果而重复试飞。

最新维护更新：许可上限已由 10 米调整为 20 米，已执行下述真实 20 米拍摄测试。下文 3.5 米与 10 米记录保持原样。两次历史降落的速度创新、IMU 与真实运动对照及重置源码链，见 [降落速度诊断](landing_velocity_diagnosis.md)。用户现要求暂停追查接触异常、先验证拍摄任务；安全门禁仍保持启用。

## 最新：20 米语言拍摄测试，有照片但任务失败

独立实例 `20261104-111042-WEN7HP`，指令 `1c7d2e29b5dc4517`，世界与种子 17 不变；模型连通及十次地面 VIO/相机检查通过，实际执行器上限读回为 20 m。经原桌面自然语言入口明确要求原地垂直起飞到 20 m、拍一张概览、报告并降落。没有修改生产代码、飞控参数或安全门限，没有录像。

- Agent 提交 `takeoff(20) + hold(2)`，任务 `2d9fad975088`。实际上升到约 20 m，但最终 `flight operation timed out after 15 seconds`，起飞阶段 failed，未正式完成后续 hold 步骤。
- `bridge.takeoff()` 在位置到达后仍要求航向就绪，以及高度误差 ≤0.10 m 持续 1 秒，两项分别有 15 秒等待。遥测在约 20.1～20.3 m 附近偏移，支持定高收敛是本次阻断项；不能把异常简单说成“爬到20米需要超过15秒”。没有放宽条件或立即重飞。
- Agent 随后调用 hold、stop、capture、land、report 等工具，实际保存 1 张照片及有效报告，最终经 `land_drone` 独立降落任务 `f87e924a363f` 完成落地上锁。所有工具调用与失败原样保留，最终应用正确标记 **failed**，没有用后续降落成功覆盖起飞失败。
- 采图 `68f927729da243299b641109ae77b1fa`，图像时间 92.900 s，机体位姿时间 92.892 s，本地 ENU z=20.34425 m；这是 20 m 目标下的实际采图高度，不是精确等于 20 m。原图 SHA256 `5d4bb5fe895eb46dbf8c78eadc377ade8ce95c9c493cfb5ea81ec0a019299711` 已核验。
- 相比 10 m 图片，实际视野更广，但近侧和右侧仍有裁切，不能称为完整全园俯视。VLM 的“东南角”“不足10%”“15×12米”“白雾遮挡”等解释未经独立测量，不作为事实结论。
- 9 次模型调用、1 份报告、0 NBV。全程保存的 EV 健康样本均为 true；ULog 无 dropout，除地面初始化和起飞既有磁对齐外，没有新的位置或速度重置，**本次未复现此前触地速度重置**。仍不能据此宣布该历史异常已修复。

最终实际 armed=false、landed_state=1。实例已按身份退出，参数恢复 success=true 且 actual==target，VIO 配置恢复启动前副本。未重跑代码回归，因为本轮没有代码修改。

证据 `logs/setup/orchard-overview-20m-20261104/`：`orchard-overview-20m.jpg` 为原图副本，`run/result.json`、`states.jsonl`、`summary.json`、`final-state.json`、`timeout-current-state.json` 保存完整任务及实际状态，`run/` 有现场桌面截图，`px4.ulg` 和 `ulog-events.json` 保存重置复核。下一步若继续改进本任务，先核对 20 m 定高误差与现有双阶段到达条件，避免盲目延长等待时间。

## 后续：自然语言单点概览拍摄已完成

用户随后明确要求先进行简单的语言任务测试。按新指示启动独立实例 `20261104-104828-YNpiUH`，保持相同世界、种子 17、模型和飞控配置；没有清除旧实例锁存或继续使用旧坐标。新实例经 10 次地面健康/实际 VIO 融合/相机新鲜度检查后，通过现有桌面输入框提交：

> 本次是单点果园概览拍摄任务，不是目标搜索。请在当前许可高度内自主选择合适高度，原地垂直起飞，稳定后拍摄一张尽量能够俯瞰整个果园的概览照片，分析并报告画面实际可见的果树和通道情况，随后降落并确认已上锁。如果相机视场或高度限制导致无法拍到整个果园，请明确说明，不要把局部画面说成全景。

任务 `9ff4d8bf797941df` 实际执行 `get_drone_state → execute_mission(takeoff=10 m) → capture_scene → report_scene → execute_mission(land)`，6 次模型调用、1 张有效左相机照片/视觉报告、0 搜索会话、0 NBV 执行。起飞任务 `be56220ce15e` 和降落任务 `bfa4983b6236` 均 succeeded，最终 armed=false、landed_state=1。这是自然语言普通飞行/拍摄链路完成，不是多视角自主搜索通过。

真实照片 `36aa990648134b709066720edfdf42aa`，采图时间 103.852 s，匹配机体位姿时间 103.848 s、ENU 高度约 10.099 m，图像哈希已核对。实际画面可见成排果树、树冠和园边通道，右侧和近处被裁切，**不是整个果园的完整俯视图**。原相机固定下俯约 0.35 rad、水平视场约 90°，本轮未修改。VLM 报告中的“东南角”“15×15 米”“通道宽 1.5 米”和树木精确数量未经独立核验，不作为测量结果；其最终回复正确承认未覆盖整个果园，但这些细节仍应保留为模型报告的可靠性限制。

**降落异常再次复现**：ULog 在 92.284 s 记录起飞预期航向计数 1→2，delta_heading≈0.000171 rad；降落末段 137.740 s 的 vxy/vz 计数均 2→3，delta_vz≈−1.24839 m/s，原守卫锁存。没有继续移动或采图。应用任务状态为 succeeded 不能替代飞行估计连续性验收；本轮该项仍未通过，前一节的根因诊断仍待完成。ULog 无 dropout。

证据 `logs/setup/orchard-overview-20261104/`：`orchard-overview.jpg` 是哈希一致的原图副本；`run/result.json`、`summary.json`、`preflight.json`、`states.jsonl`、`final-state.json` 保存原始任务和状态；`px4.ulg` / `ulog-events.json` 保存实飞日志和重置时刻；`run/04-first-observation.png`、`04-live-camera.png`、`10-terminal-grounded.png` 来自实际桌面窗口。原照片和双目元数据保留在该实例 captures/。只有图片与日志，没有录屏。实例已按身份退出，参数恢复 success=true 且 actual==target，VIO 配置恢复启动前副本。

本轮未修改生产逻辑或参数，未重跑代码回归；下文 195 项是前一轮结果。一次前置取证脚本误用相机字段 age_seconds，改为实际接口的 age/measurement_age 后才提交任务，该错误未触发飞行。

## 结论与下一步

以下为首次接入的历史记录；上方是后续用户明确授权的单点拍摄结果。用户明确选择先用平地果园测试副本。已接入 Clearpath 原始果树、树冠、三脚架基站及纹理，完成真实地面验证和一次最小起飞。**起飞与悬停正常，但降落末段发生 EV 速度融合异常和 PX4 速度重置，基础飞行验收未通过。** 飞机实际落地上锁；未提交 Agent 搜索、未运行 NBV 或多视角复核，没有录像。

下一步先离线检查本次 ULog 的降落接触、IMU、垂直速度/高度创新与视觉测量时序，确定估计偏离和实际高度回升的先后关系。当前证据不能唯一归因于 VIO、地面接触或飞控融合。不得通过清除锁存、放宽创新门限或重启后直接搜索来忽略本次失败。原因明确且基础飞行复验通过后，再执行“寻找果园边缘的三脚架设备，调整角度多视角复核、报告果树和通道，返航降落”。保留此前林地成功基线。

## 场景与边界

- 上游 `clearpathrobotics/cpr_gazebo`，固定版本 `509afbeec942c6aad376a5a76a24330f1c56dc17`；下载/预览入口见 [现成 Gazebo 世界](gazebo_existing_worlds.md)。原始果园资产不改写；许可证保留在下载目录的 `cpr_orchard_gazebo/docs/LICENSE.txt`。
- 原地形高度范围约 −0.100～0.070804 m。副本仅将地面顶点压到 z=0，保留 UV/纹理；平面支撑碰撞顶面同为 z=0。不能据此宣称支持起伏地形导航。
- 使用本机 Gazebo MeshManager 读取实际变换后的几何，完整纳入非地面视觉和原碰撞三角形（312140 个）。以 2 m XY 分格包住每个三角形的完整 AABB，再加 5 mm 余量，共 294 个保守碰撞盒。叶片也作为障碍，可能高估占据和遮挡；飞行仍另用既有 1 m 安全膨胀。
- 整体刚性平移后绕 z 轴旋转 π，让固定初始机头朝向附近果树；无人机初始航向及 VIO 参考配置未改。果树与基站之间的布局不变。
- 飞行区域完全落在平面支撑内；搜索半径保持 12 m，观察预算和安全门禁未扩大。静态地图只提供几何，不向 Agent 提供模型标签或目标真值位置。
- 当前世界 `scene/test_environments/orchard_flat_test.world`，SHA256 `857a9639be2c2deb829c0acadca9023533afa7f0d9b95090c16a20fbe2632e4d`。manifest 锁定世界及外部网格/纹理哈希，资源缺失或变更则拒绝加载。

生成入口（依赖本机 Gazebo 开发库、C++ 编译器及虚拟环境 NumPy）：

```bash
python3 scripts/preview_external_world.py orchard --check
.venv/bin/python scene/import_orchard_world.py
```

资产未安装时先按现成世界文档下载固定版本。世界含本机资产绝对路径，换机器后必须重新生成；不要把生成世界直接当作免配置分发包。原始资源、照片、二进制和运行日志均不提交。

## 实际运行

两次实例均为 seed=17，PX4 原构建、no_gps EV profile、静态三维导航；启动配置与既有桌面工作台一致，只替换 world-file。实际 gzserver 参数及世界身份已记录。

### 第一实例：初始视角不满足 VIO 地面门禁

`20261104-102845-goW8gt`：相机朝向大片模糊地面和天空，55～75 仿真秒的 199 条原生健康记录只有 14～18 个近距离双目特征，stationary_confirmed/tracking_ok 均为 false。未获得优化位姿，启动器超时退出，**没有解锁或起飞**。没有降低 VIO 门槛；原世界和 manifest 保存到 `logs/setup/orchard-task-20261104/initial-view/`。

### 第二实例：地面通过，降落估计连续性未通过

`20261104-103833-0N8BER`：调整整个场景相对起飞点的朝向后，55～75 秒的 200 条健康记录有 84～89 个近距离特征，stationary_confirmed/tracking_ok 全部为 true。提交前另保存 10 次实际地面检查，确认 VIO 健康、已用于飞控、地面上锁。

最小飞行任务 `26de59d6f6fd`：takeoff 3.5 m → hold 3 s → return_home → land。没有 Agent、NBV 或 VLM 图像报告。

| 仿真时间 | 证据 |
| --- | --- |
| 80.280 s | 已受理起飞任务，尚未解锁 |
| 82.744 s | 首次轮询记录 armed=true |
| 87.084 s | ULog 航向计数 1→2，delta_heading≈−0.005138 rad；既有起飞初始化守卫依据实际证据接受，未改规则 |
| 91.892 s | 进入 hold，PX4 ENU 高度 3.519 m，EV 健康 |
| 95.776 s | 开始 land，EV 健康 |
| 102.120 s | 首次轮询发现速度融合不健康；PX4 高度 −0.386 m，Gazebo 世界高度 0.227 m，VIO 高度 0.140 m |
| 103.180 s | 实际高度回升至 0.664 m、VIO 0.615 m，但 PX4 估计 −0.533 m；融合曾短暂恢复 |
| 105.196 s | ULog vxy/vz 重置计数均 2→3，delta_vz≈−1.24607 m/s；航向计数仍为 2 |
| 105.640 s | 轮询记录 `PX4 local velocity reset during flight` 锁存 |
| 111.276 s | 执行器 succeeded，实际 armed=false、landed_state=1；锁存仍存在，搜索预检拒绝继续 |

表中轮询时刻不是事件精确发生时刻；标为 ULog 的才是飞控消息时间。Gazebo 世界高度、VIO 局部高度和 PX4 ENU 高度有各自起点，不应直接要求数值相等；降落时超过一米的偏离值得单独诊断。ULog 无 dropout，本窗口未出现新的视觉 reset_counter 变化。不能将此次速度重置误写成起飞航向门禁再次失效。

验证脚本最初只按执行器 succeeded 和落地上锁标记 passed，复查发现这个标准不完整。原记录保留为 `minimal-flight-executor-result.json`，最终 `minimal-flight-result.json` 明确标记 **failed_validation** 并记录 EV 连续性和坐标锁存检查。另有一次验证脚本对空 mission 的读取错误，发生在提交起飞前，原记录也保留；不是飞控故障或被删除的飞行失败。

两次实例都已关闭，参数恢复记录 success=true 且 actual==target；VIO 配置恢复启动前副本。第二实例 `cleanup.log` 再次确认实际落地上锁。

## 验证范围与本机证据

| 阶段 | 结果 |
| --- | --- |
| 资产、几何和受控回归 | 通过 |
| 原始初始视角地面验证 | 失败，未起飞 |
| 调整初始视角后的真实地面验证 | 通过 |
| 起飞及悬停动作 | 完成，EV 当时健康 |
| 包含降落的估计连续性验收 | 未通过：速度融合异常及坐标重置 |
| 旋转新鲜采图 / NBV / 多视角 / 完整搜索 | 未执行 |
| 实际落地上锁 / 参数恢复 | 已核实 |

证据根目录 `logs/setup/orchard-task-20261104/`：

- `ground-camera.jpg` 与 `viewpoint-retry/ground-camera.jpg`：两种初始视角的真实相机图像。
- `viewpoint-retry/ground-stable.json`、`minimal-flight-states.jsonl`、`flight-timeline.json`、`minimal-flight-result.json`：地面与飞行完整取样及最终判定。
- `viewpoint-retry/px4.ulg`、`ulog-events.json`：ULog 副本及计数变化；原始 `/home/fxxk/.ros/log/2026-11-04/02_38_38.ulg`，SHA256 `55a3e7560c96f3f401c87b1bc8c7646f8abce538927b1db5535f314cdec71b5e`。
- 两个实例的 `vio_output/native_health.jsonl`、`ekf-observer/`、`ev-adapter/`、`sitl.log`、`ev-profile-restore-1/result.json` 保留；搜索 `start.json` 未创建。
- `model-preflight/` 只有一次成功纯文本接口检查，不能算视觉报告；任务有效报告数 0、NBV 执行数 0、目标确认未进行。没有录屏或伪造搜索工作台关键帧。

最终回归 **195 项通过、失败 0、跳过 0**，38.738 s，完整日志 `regression-final.log`。此前 194 项及移动场景后的 73 项结果单独保留，不能相加当作独立测试总量：

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 PYTHONFAULTHANDLER=1 \
PYTHONPATH=.:tests:scripts .venv/bin/python -m unittest -v \
  test_orchard_environment test_scene_config test_static_navigation \
  test_bridge_static_navigation test_autonomous_search test_nbv_search_integration \
  test_semantic_map test_takeoff_heading_alignment test_ev_heading_guard \
  test_bridge_camera test_recovery_ownership test_web_server
```

新源码仅涉及果园导入、原生网格读取、场景外部资产核验及相关测试；未更改搜索算法、PX4/VIO 融合配置或既有安全保护。
