# 面向完整任务的运行时重构

更新：2026-11-11。依据用户提供的 `px4_agent_goalfly_inspired_refactor.md`，在现有 PX4/VIO、静态三维导航、相机与桌面上增量迁移。当前相对位置任务已改为模型视觉选点与新图验收，受控stage实际成功，见 [视觉关系目标](visual_goal_runtime.md)。历史阶段 A、B、C 的数量不能当作最终版本重跑结果。

## 目标与当前实现

```text
完整原句 → 一次结构化 TaskPlan
→ TaskOrchestrator（唯一父任务、预算、阶段、正常收尾）
→ 局部执行 / 定向观察 / 有界全局重规划
→ 共享 SemanticMap / TargetMemory / FrameStore
→ 信息视点 NBV 或终点 GoalPosePlanner
→ 原 A* / 坐标与所有权门禁 / 一次性票据 / PX4
→ 实测到位 + 新鲜图片 + 条件复核 → 下一阶段或统一结果
```

A～E 的代码已经接入 Agent、执行器和桌面；代码测试涵盖搜索交接、目标附近/车前悬停、静态平地降落、路线和普通摄影。代码测试使用脚本模型及明确的相机/飞行替身；2026-11-11真实stage的图中选点、GoalPose、新图确认与正常降落成功单独记录，不与代码测试合并。

默认仍为 `UAV_TASK_RUNTIME=legacy`。新模式在进程启动前设置 `staged`；不在异常或空中静默切换。没有修改模型、PX4/VIO 参数、碰撞膨胀、预期起飞对齐、正式绑定和重置保护。

## 模块与迁移

| 模块 | 实际职责 |
| --- | --- |
| `contracts.py`、`planning.py` | 可序列化 schema v1 契约；验证原句必要阶段、目标引用、顺序、终端要求与固定许可；无模型坐标 |
| `orchestrator.py` | 唯一父生命周期、revision、串行动作、一次纠正、预算及 TaskResult；不是第二个搜索器 |
| `skills.py`、`autonomous_search.py` | standalone/orchestrated、明确 handoff、同 owner 多目标阶段和定向观察续接；一次起飞/正式绑定 |
| `association.py`、`memory.py` | 现有权威地标的只读任务视图、标签别名、真实 ROI 匹配与几何联合关联、关键视角和丢失状态 |
| `reasoning.py` | 根据真实新证据触发局部观察与有界全局重排，不按重复模型调用计进展 |
| `visual_goal.py` | 模型选择图中地面点或保留当前位置，转换航点，到位新图确认用户要求的关系 |
| `object_frame.py`、`route.py`、`goal_planner.py` | 可选对象度量诊断、经过/转向、终点位置和相机外参对应的航向；普通摄影构图配置 |
| `execution.py`、`goal_pose_geometry.py` | 适配原几何查询和版本化目标票据；提交前记账，到位后检查实际位置/航向和新帧 |
| `landing.py`、`goal_landing_authority.py`、`flat_landing.py` | 悬停后新图复核、无标签静态支持面与连续下降柱、一次降落许可及实际终态 |
| `photography.py` | 原 PhotoTask 作为父技能；真实动作→新图→四边独立复核，范围未知仍未知 |
| `single_vlm.py` | 模型/图片注入、严格工具及原图复核适配；状态推进交给父任务 |
| `web_server.py`、`task_contract.py` | 任务归属、实际执行记录、父结果；搜索结束、原目标完成和实际落地分开 |
| `desktop_search_data.py`、`desktop_workbench.py` | 现有地图和相机中增加父阶段、目标 ID、推理模式、信息缺口、目标终点和未完成项 |

执行器新增代码保持 Python 3.8，使用普通 JSON 契约，不导入 Agent 的 Pydantic、图像特征或模型依赖，不在控制线程做模型请求。

## 原句、阶段与证据

模型一次调用 `define_task_plan`。支持 `search`、`approach`、`close_photo`、`photo`、`report`、`pass_landmark`、`relative_turn`、`face_target`、`relative_hover`、`relative_land`、`return_home_land`、`hold`。

- 寻找→接近/拍照→报告→终端动作各自保留。车前是车辆自身前向，附近不是车辆中心，观察表面点不冒充完整占地。
- 目标动作之前须有实际搜索证据；路线左/右转的目标引用用于追踪前序地标，角度 5～90°，按新鲜机体 ENU 姿态计算。当前路线不支持在没有任何地标证据时以转向作为首阶段。
- 经过使用实际观察表面参考平面与位置不确定余量，不声称已越过未知物体完整包围盒。朝向旧米制线索只能用于回看；新图真正看到同一目标且居中后才完成 `face_target`。
- 计划语义验证保护已支持的必要动作，并验证完整引用/顺序；有限词汇保护不是通用自然语言完整性证明。
- 普通摄影以 `photo` 开始，沿用原图和四边一致性复核；寻找后的取景用 `close_photo`。普通区域摄影与后续搜索混合尚不支持，不将“拍全果园”改写成找一个物体。

初始搜索用途在计划建立时固定：后续需要关系动作时可交付 `candidate_acquired / visual_candidate`，供模型选择位置并在到位新图中确认。候选交付与关系验收分别记录；后续视觉确认使用 `visual_review`，不要求重复证明类别或满足旧对象轴门槛。

handoff 清除未执行候选、关闭该技能活动，不自动返航、不使用 `end_behavior=hold` 伪造交接、不换 owner 或清地图。下一目标只重算 query 的语义场并使旧票据失效；原观测、起点、epoch、父预算保留。

## 三层推理与共享记忆

`LOCAL_GROUNDING` 由程序显式生成当前合法 observe/next/execute 工具调用，并记录为 program，不伪造模型请求。`LOCAL_OBSERVATION` 通过 ObservationNeed 驱动缺深度、实例消歧、车头揭示、遮挡恢复和近照。连续两个真实新视角无进展或无可达候选才升级 `GLOBAL_REASONING`，最多两次，至少间隔两份新观察。模型只能选择保留目标的恢复/替代观察策略，并重排尚未执行且满足依赖的阶段；不能改完成前缀、当前阶段、原要求或许可。

记忆引用一个 SemanticMap/TargetMemory/FrameStore。保存真实原标签、规范类别/颜色、原图 hash、位姿、位置/方位、协方差、朝向证据、少量关键视角、最近可见与错失次数。预测回看不等于检测；长期不可见不许可按旧坐标接近。

默认实际外观后端是 **`cpu_harris_patch_affine_v1`**：对哈希核验的真实 JPEG/ROI提取局部归一化补丁、互相匹配和有界仿射一致性检查，缓存同图同框；不是 DINO，也不宣称等效复现视频。裁切、无纹理、缺图、重复图不产生强外观证据；纹理匹配失败不否定原有唯一双深度几何关联。同类同帧/多候选歧义不能只取最高分强合并。

bearing 候选可以在真实外观+几何证据后保留 ID 并获得米制观测。双图明确支持且唯一的单侧深度重投影复核，可把 bearing 与 metric 关联写入原地标，不补造第二份米制测量，不缩小协方差。supported必须支持当前query的类别与可见属性，不能仅表示有物体或两图同物体。明确否定的源图保留原报告和否定记录；在尚未执行目标动作、旧引用对应源图已经被严格复核明确否定时，新对象独立满足同query/当前epoch/阶段的强多视角交付，才允许有审计地接续。先前的确认等级不能令已被否定的引用永久有效；这不是关联两物体为同一实例。未决或未否定的引用、未经独立确认的新候选及已执行终点的实例均不允许替换。见 [本轮真实故障与修复](front_face_completion.md)。

原图只读回放：27 ROI、24组比较（1 matched、4 hypothesis、9 insufficient、10 unavailable），提取总86.45 ms，峰值RSS约64 MiB，无GPU或下载；原始文件 hash 不变。没有人工身份真值，不能作为准确率提升结论。记录：`logs/setup/task-runtime-stage-b-20261105/real-image-matcher-replay.json`。

## 终点位置、航向与执行

GoalSpec 与目标分开。相对位置任务由模型调用 choose_task_goal：当前位置已满足要求选here，移动时选择实际图中的地面点，不确定则补充观察。程序按已保存的相机标定、姿态和已注册平地计算航点，经通用路径检查与一次性票据执行；车体前向、前脸平面或固定距离误差界不再作为语义完成门槛。详见 [视觉关系目标](visual_goal_runtime.md)。

到位后取得实际新图，模型比较参考图与新图，通过review_task_goal确认目标和用户要求的关系。新图可以来自同一位置，不强制凑出独立基线或确认次数；不确定或不满足则继续观察、改选。当前模型结论与原始图像一起保存，不把几何预测冒充语义判断。

所有权、坐标、许可体积、可达路径仍由程序检查。终点票据绑定stage、plan_revision、target_revision；视觉位置配置使用版本2，明确要求新图复核。现有机体转向、地标路线和普通摄影的执行约束保留；度量前脸工具移出默认在线流程，保留作诊断。

普通摄影也调用 GoalPosePlanner 的局部构图配置计算实际 ENU 位置/yaw，沿原 photo-action 路径预检执行并实测核验。没有主体完整范围时，预测全入镜和新增覆盖均为 null；升高、保留旧框射线、四边未裁切均不能证明整个果园入镜。

所有副作用提交前记账。已经受理失败或超时不自动作为未执行重试；阶段切换只撤销旧批次，不能清除操作员永久撤销。取消、迟到 revision、重复回调、VIO/EKF失效继续阻断。

## 窄范围真实平地降落能力

范围仍为已验证静态仿真中的明确平坦地面，无动态对象、坡地或未知支持面。前方、后方、附近和旁边共用视觉关系流程，不以车辆类型区分。

1. 模型在实际图像中选择满足要求的位置，程序计算并检查航点。
2. 实际到位、新图保存、模型确认关系。
3. 执行器独立验证无标签几何支持面和完整下降通道，签发绑定owner/阶段/目标、实际悬停任务和新帧的短时一次性许可。
4. 正常下降完成并实际落地上锁后，任务成功。触地后的速度计数重置或诊断缺失单独记录，不推翻已完成的视觉语义任务；飞行中坐标或执行故障仍由原安全链处理。

路径或支持面在提交前不可用时，向模型返回具体原因并允许原预算内改选，不用固定几次几何尝试或未要求的距离门槛提前判定用户目标失败。

## 固定预算和统一结果

沿用当前工作区30米、20米、16观察、900秒；用户原文12米早于本机30米配置，本轮不继续扩大也不覆盖既有授权。预留关系/近照/终点验证份额，阶段不刷新中心、时钟或总次数。父任务最多60次实际模型请求、共享一次未提交协议纠正（含类型错误/提前文本），另有最多两次有冷却的全局重排。原 PhotoTask 最多4张且调整后必须有实际不同动作完成记录。

格式错误在真实工具schema校验后返回一次 `not_executed/tool_schema_invalid`，不重拍、不飞、不写地图。第二次错误及真实执行、图片归属、坐标故障保持原独立收尾。模型“安全/操作员要求”等理由不增加权限。

TaskResult 分开保存 process_status、goal_status、completed_stages、unmet_requirements、evidence_summary、flight_status、termination_source。视觉候选、模型新图确认和实际落地分别保存。任务目标由模型视觉验收及实际执行结果共同决定，完成就标记成功；运行诊断单独记录。

## 桌面

保留现有实时左相机、ENU覆盖地图、历史原图/检测框、多视角双图和独立安全按钮，增加“完整任务”页签与已验证终点路径/航向；状态区显示父任务阶段、信息缺口和分别的任务/飞行结果。预测终点不涂为已观察覆盖，终态/失效版本清除未执行候选。1220×900和960×720、13号字体有受控 Qt 布局检查；实际现场截图另附，离屏抓图不是实飞证据。

## 运行、回滚与验证

新实例启动前设置，禁止覆盖正在运行的任务：

```bash
# 在仓库根目录执行；保留绝对启动路径以便实例归属核验。
PROJECT_DIR="$(pwd -P)"
env UAV_TASK_RUNTIME=staged PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}" \
  UAV_NAVIGATION_MODE=static_3d UAV_ENABLE_YAW=1 UAV_ENABLE_NBV_YAW=1 \
  UAV_SEARCH_MODE=nbv_geometry UAV_NBV_SELECTOR=deterministic \
  UAV_NBV_HEADING_MODE=discrete UAV_NBV_AUTO_PREPARE=1 \
  UAV_TARGET_CONFIRMATION=multiview PX4_AGENT_GAZEBO_SEED=17 GUI=true \
  bash "$PROJECT_DIR/scripts/start_system.sh" --control-ui desktop \
  --env-file "$PROJECT_DIR/.env" --ev-profile no_gps \
  --world-file "$PROJECT_DIR/scene/test_environments/courtyard_spacious.world"
```

关闭使用 `bash scripts/stop_system.sh <实际运行目录>`，核实落地上锁与配置恢复。回滚入口是在已安全退出后新启动时使用 legacy，不删除工作区或空中接管。A前、B前、D前文件快照在忽略目录 `logs/setup/task-runtime-*/before/`，保留了前置未提交修改。本轮无自动commit/push。

### 参考思想到代码/证据

| 视频思想 / 本轮阶段 | 本项目接入 | 受控或实际证据 |
| --- | --- | --- |
| Task parsing / A | 完整 TaskPlan、handoff、唯一父结果 | `test_task_runtime`、`test_task_runtime_bridge`，真实Agent图无额外报告总结回合 |
| Adaptive reasoning / B | 新证据门槛、三层调度、两次有界全局重排 | `test_task_reasoning` 的无进展→重排→继续原任务、多地标同预算 |
| Cross-view memory / B | 实际 CPU ROI匹配 + 几何/时序/别名、bearing续接、源图否定 | `test_task_association`、真实旧照片只读回放；无DINO或性能复现声明 |
| View-aware goal / C | 图像选点、终点位置/yaw、新图关系确认、路线及取景配置 | `test_visual_goal`、`test_goal_pose`、`test_task_goal_graph`、`test_task_route` |
| Local planning/execution / C | 原 A*/权限/票据，阶段/目标版本，旋转后新图 | `test_goal_executor`、`test_viewpoint_authority`，Python3.8实际Controller方法 |
| 目标相对降落 / D | 到位新图复核、实际几何支持面/下降柱、one-use落地 | `test_visual_goal`、`test_flat_landing`、`test_goal_landing_executor`、`test_task_landing_graph`，触地后的诊断变化单独记录 |
| 摄影与展示 / D/E | PhotoTask父技能、独立四边复核、父阶段与终点图层 | `test_task_photography`、`test_task_desktop`、原桌面/摄影回归 |

### 验证记录

最新873项回归和真实stage正常降落成功见 [视觉关系目标](visual_goal_runtime.md)。以下为2026-11-06历史记录；当时的逐项结果、完整可执行回归命令、两个真实实例、截图和未通过项见 [任务运行时验收记录](task_runtime_validation.md)。

真实搜索/报告/返航已完成一次：实例 `20261106-003935-jdqTYi`，6份有效图像/地图、5次票据换位、13次模型请求，最终强复核为 single_depth_projection，实际返航落地上锁、VIO健康。自然出现一次 targets 字符串类型错误，经同帧的一次纠正恢复；没有重拍或凭空生成目标。真实旋转完成169.920/278.684秒，对应左右新帧170.048/278.748秒。原图/地图回放和确认审计通过。

另一次原句车前降落实例 `20261106-005649-A2ktCH` **失败**：3份有效观察、2次换位，未获得目标引用，模型请求需要目标引用的恢复策略，在进入车前阶段前异常中止。独立降落上锁、VIO健康；不是车前降落，也没有正常返航。程序缺陷已在飞行后修复，**未再实飞验证修复**。当前不会为此无限试飞或放宽视觉、坐标与落区条件。

修复后的GLOBAL_REASONING明确给出available_replan_strategies；没有实际目标引用时只允许continue_search。第一次不可用策略在工具执行前返回typed not_executed，保留原地图、预算、阶段和未执行额度，不调用执行器；共享同一个父纠正额度，第二次仍失败并走原独立安全收尾。后续原始重规划参数记录于本机诊断。协议异常在清理消息前按TaskProtocolError类型标记protocol_error，坐标/执行故障仍safety_fault；不根据异常字符串分类。目标颜色即使在attributes而不在label中，也进入实际搜索/摄影query，原标签保持；后续接近/路线/关系阶段继续预留观察份额。

规划前另保存不可覆盖的 `map-<sha256>.snapshot.json`，区分采图时地图和query/复核/方向更新后的规划版本，不改写原图对应的旧快照。旧回放器已适配实际CPU后端和真实JPEG；原首次审计失败记录保留，不将修复后的只读回放写成重新飞行。

适用边界：图像不足时模型仍可要求观察或返回不确定；场景/模型泛化与真实落点成功率需要后续正常使用积累，不以本次受控成功声称解决。整体果园范围未知支线保持停止。所有原图、运行日志、密钥、私人路径与请求记录只保留忽略目录，不提交公开仓库。
