# 物流园场景与单 VLM 自然语言任务实验

更新日期：2026-10-04。本文记录将旧的彩色纹理 VIO 演示环境替换为有实际设施语义的小型物流园区，以及当前单 VLM Agent 的实验方案和验收证据。

**当前状态：新物流园已实际启动并打开 Gazebo GUI，三项真实自然语言飞行任务已完成验收。** 同一 `qwen3-vl-plus` 完成红集装箱巡检、白色车辆巡检，以及依据第一视点蓝桶判断选择下一明确航点的双视点条件巡检。三项均报告实际机载图像、返航并落地上锁。第一项原评估器存在返航动作名称导致的假阴性，原始记录与同轨迹重评均保留；后文同时记录模型较高置信度下仍存在的边界框误差。

## 1. 实验场景

场景是本地程序生成的 Gazebo Classic 世界，包括路面、停车区、起降区、草地、树、仓库、办公楼、货运集装箱、车辆、桶、交通锥、灯杆、栅栏及堆放纸箱的托盘。建筑有窗、门及材质纹理；车辆有驾驶室、黑色轮胎、轮毂、灯和保险杠；货运箱有波纹墙、角柱、门与锁杆。所有模型和纹理均在本地生成，可离线复现。

文件关系：

| 文件 | 实验用途 |
| --- | --- |
| [`scene/generate_world.py`](../scene/generate_world.py) | 固定随机种子生成几何、纹理与布局，并检查净空 |
| [`scene/worlds/logistics_yard.world`](../scene/worlds/logistics_yard.world) | Gazebo 实际加载的世界 |
| [`scene/runtime_manifest.json`](../scene/runtime_manifest.json) | 世界 SHA-256、已知地面和平面边界、飞行区域；不含目标位置真值 |
| [`scene/layout.json`](../scene/layout.json) | 独立评价使用的实体位置、尺寸与包围盒；不输入模型 |
| [`scene/testing_tasks.json`](../scene/testing_tasks.json) | 本次正式自然语言任务、允许航点及评价目标别名 |
| [`scripts/test_realistic_scene.py`](../scripts/test_realistic_scene.py) | 通过实际 HTTP Agent 执行任务、采样状态并保存验证结果 |

冻结的世界 SHA-256：

```text
b1791d061bac5c47554555d48279ab3ff3f12d7c12b62dc3a330e60f6931ddde
```

世界地面范围为 `x,y∈[-20,20] m`，已知视觉地面近似为 `z=0.003 m`。观察净空走廊为世界坐标 `x∈[-3,8]`、`y∈[-4,4]`、`z∈[0,6] m`；运行约束另保留 `0.6 m` 边界余量。场景生成器的静态记录显示，23 个设施实体均在净空走廊之外并满足至少 `2 m` 水平净空距离。实际飞行仍需核对 MAVROS 本地 ENU 与世界坐标的映射，不能只依赖静态布局。

下表仅用于独立核对模型图像判断，不会交给 Agent 作为答案。位置是模型原点；最终投影评价应使用 `layout.json` 中的实际视觉 / 碰撞包围盒。

| 目标 | Gazebo 世界模型位置，m | 几何特征 |
| --- | --- | --- |
| 红色货运集装箱 | `(13,-2,0.003)` | 约 `6.12×2.50×2.66 m`，沿世界 Y 放置 |
| 白色配送厢式车 | `(13,3.5,0.003)` | 约 `5.30×2.22×2.82 m`，车头朝西 |
| 蓝色金属桶 | `(10.8,0.7,0.003)`、`(10.8,1.6,0.003)` | 每个约 `0.69×0.69×1.02 m` |

本场景采用有纹理的程序几何建模。图像仍受仿真渲染、相机分辨率、遮挡和距离影响；模型准确识别这些实体的能力需由实际图像及报告验证。

## 2. Agent 与传感器链

使用当前单 VLM 架构：同一 `qwen3-vl-plus` 负责理解指令、直接阅读相机 JPEG 和选择受限工具。LangChain Agent 按飞行阶段、采图、报告、后续飞行阶段依次执行。`capture_scene` 取得新鲜快照，模型随后看到该帧，`report_scene` 保存结构化视觉证据；采图成功本身不构成识别成功。

PX4 SITL、MAVROS 与 ROS 执行器完成实时飞行。原机体 IMU 与双目摄像头送入 VINS-Fusion，VIO 目前用于独立定位状态展示，尚未输入 PX4 飞控。相机为 `640×480` 双目、约 `0.12 m` 基线、机头前向且俯角 `0.35 rad`。本次世界替换不自动增加深度估计、地图定位或避障能力。

实验中只执行使用者明确给定的航点。目标视觉位置不得被转成靠近目标的新航点。服务端继续独立检查任务序列、遥测新鲜度、飞行区域、中断权限与失败锁定；模型任务失败后记录原任务失败及独立安全降落结果。

## 3. 三个正式任务

所有航点为 MAVROS 本地 ENU 米，`x` 向东、`y` 向北，`z` 是本地坐标高度。正式完整中文指令在 `scene/testing_tasks.json`。每个任务必须以 `ready=true`、落地且上锁开始；有一次起飞，各观察点完成到点后才能取图，观察完成后返航、降落并确认 `landed_state=1` 且 `armed=false`。

| 任务 ID | 自然语言任务与明确路线 | 最少观察数 |
| --- | --- | --- |
| `red_container_inspection` | 起飞到 `3.5 m` → `(5,-2,3.5)` → 找红色货运集装箱，报告画面位置与实际可见周围关系 → 返航 → 降落 | 1 |
| `white_van_parking_inspection` | 起飞到 `3.5 m` → 经过 `(2,3,3.5)` → `(5,3,3.5)` → 找白色货车 / 厢式车，描述停车区可见关系 → 返航 → 降落 | 1 |
| `conditional_two_view_patrol` | 起飞 → `(4,0,3.5)`，采图并保存蓝桶报告 → 根据第一份视觉报告在两条明确路线间选择 → 新视点重新采图报告并比较 → 返航 → 降落 | 2 |

条件任务仅允许以下两条第二阶段路线：

- 第一份结构化报告明确识别蓝色桶，且 `confidence≥0.6`：到 `(5,3,3.5)` 观察白色车辆。
- 第一份报告未确认蓝色桶，包括类别不确定或置信度低于门限：到 `(7,0,3.5)` 再次观察蓝桶。

第二阶段需等第一份报告保存后再提交，不能提前把两条分支都串进任务。两次观察需有不同 `capture_id` 和各自的模型图像调用。该置信度是模型的未校准主观判断；门限用于展示受限条件决策，不代表检测概率已经校准。第一份摘要与结构化字段冲突或无法可靠确定分支时记 `branch_ambiguous`，不得将整项任务标为成功。

## 4. 评价原则

飞行执行、视觉识别和位置估计分别记录，避免把工具返回成功当成找到了目标。

| 层面 | 成功证据 | 不足或失败的记录 |
| --- | --- | --- |
| 执行与安全 | 实际完成的 `takeoff`、批准的 `goto`、返航动作或已验证的 home-goto、`land` 事件；取图时在指定观察点且已解锁；最终落地上锁 | 未完成事件、越出指定路线、超时、工具或模型错误、降落未确认 |
| 单 VLM 与图像来源 | 全部实际调用模型为同一 `qwen3-vl-plus`；报告 `capture_id` 对应带图像的调用；JPEG SHA-256、尺寸、采集时刻与新鲜度有效 | 无真实报告、跨视点复用快照、旧图、缺图、不同模型或失败调用 |
| 目标语义 | 结构化 target label 同时匹配类别及颜色，结合保存的原图进行独立人工核对 | `not_found`、错误类别、未确认、遮挡或图像不可判定 |
| 条件选择 | 第一份已保存证据与实际下一段航点一致，第二段飞行在首报告之后开始 | `branch_ambiguous`、依据不足、提前飞行、选错明确分支 |
| 图像定位 | 使用采集时刻相机内参与位姿，将独立实体包围盒投影到图像，与模型 bbox 核对 | bbox 不交叠、错目标或证据无法评价；包围盒投影不能替代像素级分割真值 |
| 粗略世界位置 | 只在接地证据与几何假设满足时输出地面射线交点，标明坐标系与适用假设 | 仅方向、未测距离；不将条件投影当作实测深度或 ENU 航点 |

语义自动评价按目标类别 / 颜色别名匹配结构化标签，写入独立 `semantic_result`。它不读取目标真值来生成答案，也不是独立视觉检测器；仍需查看对应原始模型输入 JPEG。模型把不存在的物体报告得很详细，或报告工具正确保存了错误判断，都不能因此获得识别正确的结论。

独立几何核对使用布局真值及快照外参，不会把真值传给 VLM。应同时注明遮挡、投影包围盒近似、同帧位姿约 `50 ms` 的接收时间对齐误差，以及模型 bbox 的主观误差。`Gazebo ModelStates` 无源时间戳，因此采集时刻姿态匹配属于近似。

## 5. 运行与取证

只校验计划、不发 HTTP 请求：

```bash
cd /home/dut/px4_agent
python3 scripts/test_realistic_scene.py --config scene/testing_tasks.json --dry-run
```

实际系统就绪并已核对世界版本后运行：

```bash
python3 scripts/test_realistic_scene.py \
  --config scene/testing_tasks.json \
  --output logs/realistic_scene/<run>
```

默认每项任务允许 `350 s` 现实时间，每 `0.5 s` 保存完整状态。可用 `--task <id>` 选择指定任务、`--timeout` 调整整项任务超时。runner 在实际 `POST /api/commands` 中携带 `mode="task"` 和 `min_observations`，前两项各为 1，条件任务为 2。测试脚本不会自动重试模型调用、识别或飞行；实际失败后请求独立降落，并停止后续任务。

证据按如下结构保存；本次实际任务目录分别为 `run1/red_container_inspection`、`run2/white_van_parking_inspection` 和 `run2/conditional_two_view_patrol`。

| 证据 | 保存位置 |
| --- | --- |
| 本次实际任务配置 | `logs/realistic_scene/<run>/executed_plan.json` |
| 三任务汇总 | `logs/realistic_scene/<run>/validation.json` |
| 每任务最终指令、飞行事件、所有观察及评价 | `<run>/<task_id>/task_validation.json` |
| 每任务完整状态采样 | `<run>/<task_id>/state_samples.jsonl` |
| 公开指令更新与全部模型调用证据 | `<run>/<task_id>/command_updates.jsonl` |
| 去重后的所有视点报告 | `<run>/<task_id>/perceptions.json` |
| 原始模型输入 JPEG 与同帧几何元数据 | 对应系统运行目录 `logs/system/<system_run>/captures/<capture_id>.jpg` 和 `.json` |

实际报告需要关联冻结世界 SHA、系统运行目录及任务目录，避免后续 `logs/system/latest` 更新后混淆实验版本。输入 JPEG 的 SHA-256 应与报告相同；网页实时预览或事后截图不能替代模型当时实际收到的那张输入图像。

## 6. 实测结果与当前验收状态

实际系统运行目录为 [`logs/system/20261004-183029-FeDrHJ`](../logs/system/20261004-183029-FeDrHJ)，旧环境已经停止。新默认世界与冻结 SHA 一致，`scene.verified=true`；Gazebo GUI 已打开，GUI 仿真速度因子为 `0.35`。原生 Gazebo 截图见 [园区概览](../logs/realistic_scene/gazebo_overview.png)。

![物流园原生 Gazebo 画面](../logs/realistic_scene/gazebo_overview.png)

静态与单元验证：任务脚本语法、配置 dry-run、确认 / 未确认 / 低置信 / 矛盾条件分支、错误目标类别、实际返航行为检查均已通过离线验证。Python 3.11 完整单元测试 **145 项通过**，见 [单测日志](../logs/realistic_unit_tests.log)。ROS Python 3.8 发现 115 项，其中 **102 项通过、13 项因依赖缺失跳过**，见 [ROS 日志](../logs/realistic_ros_unit_tests.log)；这些跳过不能作为完整 Agent SDK 已在 ROS Python 中通过验证的证据。

### 6.1 已完成的两项真实任务

| 项目 | 红集装箱巡检 | 白色车辆 / 停车区巡检 |
| --- | --- | --- |
| 实际 command ID | `edb5de3f928d4883` | `04228a2bce17491a` |
| 实际指令最终状态 | `succeeded` | `succeeded` |
| 记录用时，现实秒 | `127.210` | `135.157` |
| 同一 Qwen 实际模型调用 | 6 次 | 6 次 |
| 完整状态采样 | 245 帧 | 261 帧 |
| 实际目标标签 | 红色货运集装箱 | 白色厢式车 |
| 模型主观置信度 | `0.98` | `0.97` |
| 独立 AABB 图像投影矩形 IoU | `0.915818` | `0.675492` |
| 最终飞行状态 | 落地，上锁 | 落地，上锁 |

两条轨迹中全部 506 次完整采样的飞行就绪、Gazebo 位姿、双目预览新鲜度和独立 VIO 就绪均有效。该记录说明本次任务过程传感器链持续可用，没有给出 VIO 的位置精度结论。每个实际 `640×480` 输入均有 SHA-256、相机几何元数据及与报告对应的带图模型调用；两任务日志未记录模型调用错误。

红集装箱输入：[`dafbca7553454e4d95bcd97e00ca5de5.jpg`](../logs/system/20261004-183029-FeDrHJ/captures/dafbca7553454e4d95bcd97e00ca5de5.jpg)。模型报告左侧白色货车、前方两个蓝色桶、右侧砖墙 / 纸箱以及右下方交通锥。独立查看这张原始模型输入，可以核对上述物体确实可见；这里的周围关系属于该帧证据，没有推断被遮挡或视野外区域。红箱粗略位置最终为画面区域与视线方向，没有输出无依据的米制距离。

![红集装箱任务实际模型输入](../logs/system/20261004-183029-FeDrHJ/captures/dafbca7553454e4d95bcd97e00ca5de5.jpg)

首次 runner 校验将“返航”硬性限定为 `return_home` 动作名称，导致错误地把已经返航的任务判失败。实际 Agent 在保存报告后提交了 `goto(-0.0186,-0.0100,3.5) → land`，目标是当时记录的起飞点上方，真实返航事件与解锁轨迹均符合要求。原始 [任务报告](../logs/realistic_scene/run1/red_container_inspection/task_validation.json) 保留这一假阴性；新版验证检查报告后起始时序、提交的 home XY、空中高度、真实完成事件、附近解锁状态采样以及随后落地，得到 [同轨迹重评报告](../logs/realistic_scene/run1/red_container_inspection/reassessed_validation.json) `validation_passed=true`，`returned_via=goto_recorded_home`。此次重评没有新增模型调用，也没有重飞。独立红箱 [投影评价](../logs/realistic_scene/run1/red_container_inspection/bbox_evaluation.json) 另行保存，不更改原始任务历史。

白色车辆输入：[`12ea978119584fd38a88d7793645cdfc.jpg`](../logs/system/20261004-183029-FeDrHJ/captures/12ea978119584fd38a88d7793645cdfc.jpg)。模型识别白色厢式车，报告其位于两条白色停车线之间，左侧灰色多窗建筑、右侧红色集装箱和蓝桶，并描述开裂的深色沥青地面。独立查看原图可核对这些实际可见对象；“前方”等语言相对关系仍需结合图像理解，不能用来直接推断精确三维方位。该任务 [正式验收报告](../logs/realistic_scene/run2/white_van_parking_inspection/task_validation.json) 为 `validation_passed=true`，位置结果为仅方向、未测距离。[白车投影评价](../logs/realistic_scene/run2/white_van_parking_inspection/bbox_evaluation.json) 与报告独立保存。

![白色车辆任务实际模型输入](../logs/system/20261004-183029-FeDrHJ/captures/12ea978119584fd38a88d7793645cdfc.jpg)

上述 IoU 比较的是实际 SDF visual / collision 联合 AABB 的图像投影矩形与模型 bbox。它不处理遮挡或透明材质，也不是像素分割真值；两个单帧目标不能代表总体检测精度、召回率、距离误差或自主导航成功率。

### 6.2 条件双视点任务

实际指令 **`d82a7a3c51c840a9`** 最终 `succeeded`，[正式任务验收](../logs/realistic_scene/run2/conditional_two_view_patrol/task_validation.json) 为 `validation_passed=true`。记录用时 `180.997 s` 现实时间，9 次模型调用、346 次完整状态采样。实际选择的是“蓝桶已确认”分支；未确认分支已做离线选择逻辑验证，本次没有执行该分支的飞行。

第一观察点 `(4,0,3.5)` 的新快照为 **`dd85eb418acb4b66b3996fc0b4ccaa46`**，模型报告两个蓝色桶，主观置信度为 `0.98` 和 `0.97`，描述白色车辆、红色集装箱及地面停车线。第一报告于墙钟 `1791110347.572` 保存，后续到 `(5,3,3.5)` 的飞行阶段于 `1791110355.877` 才提交，满足报告后再选择路线的约束。

![条件巡检第一视点实际输入](../logs/system/20261004-183029-FeDrHJ/captures/dd85eb418acb4b66b3996fc0b4ccaa46.jpg)

第二观察点的新快照为 **`d4eb31ffe94742178d9e09218dcfff37`**，与第一帧的 `capture_id` 不同，存在该帧对应的真实带图模型调用。模型识别白色厢式车，主观置信度 `0.96`，并报告停车线、左侧建筑和右侧红箱 / 蓝桶。随后实际执行 `goto_recorded_home → land`，最终 `ready=true`、`armed=false`、`landed_state=1`，独立 VIO 就绪。实际输入帧中两视点目标均可独立核对，Agent 最终答复归纳了两个视点的目标与实际分支。

![条件巡检第二视点实际输入](../logs/system/20261004-183029-FeDrHJ/captures/d4eb31ffe94742178d9e09218dcfff37.jpg)

本任务全部独立框评价如下，保留较差结果：

| 视点 / 目标 | 主观置信度 | 实体 AABB 投影矩形 IoU | 独立评价文件 |
| --- | --- | --- | --- |
| 第一视点，画面左蓝桶 | `0.98` | `0.600224` | [左蓝桶评价](../logs/realistic_scene/run2/conditional_two_view_patrol/blue_left_bbox_evaluation.json) |
| 第一视点，画面右蓝桶 | `0.97` | `0.385632` | [右蓝桶评价](../logs/realistic_scene/run2/conditional_two_view_patrol/blue_right_bbox_evaluation.json) |
| 第二视点，白色车辆 | `0.96` | `0.595284` | [白车评价](../logs/realistic_scene/run2/conditional_two_view_patrol/white_van_bbox_evaluation.json) |

高置信度没有保证边界框准确，尤其右蓝桶的框误差明显；Agent 最终答复关于 bbox“准确覆盖”的自由文字不能取代独立评价。目标类别与主要周围设施正确，并不意味着所有视线关系、框或距离都准确，也不代表整个场景的识别召回率。

### 6.3 全部任务统计与结论

三条实际指令合计 **21 次同一 Qwen 模型调用、4 个新视点快照、852 次完整状态采样**。调用链未记录模型接口失败或工具失败；模型任务没有自动重试。852 个采样中，飞行就绪、Gazebo 位姿可用、左相机新鲜、右相机新鲜和独立 VIO 就绪分别均为 **852/852（100%）**，未在采样时刻观察到失鲜。采样间隔约 `0.5 s` 现实时间，这一结果不证明间隔内完全没有瞬时抖动，也不证明 VIO 位置精度。

所有 5 个结构化目标结果都采用 `image_only → bearing_only`，清楚标注未取得目标深度；没有输出虚构的米制位置，也没有根据视觉框生成靠近目标的飞行航点。全部测试完成后的真实状态为落地上锁、飞行执行器和独立 VIO 就绪，Gazebo GUI 保持打开。原生 GUI 截图使用未遮挡的实际 Gazebo 窗口取得。

| 实验项 | 当前结果 | 证据与界限 |
| --- | --- | --- |
| 物流园实际加载与 Gazebo 图形界面 | 已验证 | 活动世界 SHA / 场景状态、原生 Gazebo 截图 |
| 双目数据与独立 VIO | 全部 852 次采样有效 | 完整状态文件可复核；尚未评价 VIO 精度 |
| 红集装箱巡检 | 实际成功；原校验假阴性已同轨迹重评 | command、飞行事件、原图、报告、独立识别与投影评价齐全 |
| 白色车辆 / 停车区巡检 | 已验证成功 | command、经过点与观察点事件、原图与评价齐全 |
| 条件双视点巡检 | 已验证成功 | 两份新快照、实际已确认分支、报告后飞行和返航落地事件齐全 |
| 模型调用错误与安全降落 | 全部 21 次模型调用无接口 / 工具错误 | 无模型失败时未实测触发模型失败降落回退；单元测试另有覆盖 |
| 全部任务后的落地上锁与服务状态 | 已验证 | 第三任务最终真实状态 |

本次实验证明当前 Agent 能在有实际设施语义的新世界中执行明确路线、机载视觉观察、周围设施报告和依据视觉证据选择受限后续路线，随后真实返航落地。框误差说明视觉判断仍需独立校验；位置输出目前止于画面区域和方向。当前实现尚未验证全环境覆盖搜索、未知路线导航、自主避障、目标实测三维定位或 VIO 飞控闭环。

最终汇总见[三任务验收JSON](../logs/realistic_scene/validation.json)；[页面烟雾验证](../logs/realistic_scene/browser_smoke.json)确认左右640×480图像可见、航迹监测与机载相机标签正常、页面无JavaScript错误，见[页面截图](../logs/realistic_scene/console_desktop.png)。本次真实最终状态另保存为[final_state.json](../logs/realistic_scene/final_state.json)。
