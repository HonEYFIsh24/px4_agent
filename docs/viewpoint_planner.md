# 固定航向 NBV：确定性基线与受约束执行

本阶段基于已有空间语义地图生成和排序候选视点，并已实现候选票据与受约束执行接入。默认 `UAV_SEARCH_MODE=fixed` 仍沿用原方向策略；可选 `nbv_geometry` / `nbv_semantic` 已取得有限真实功能验证，原油桶任务保留单帧视觉误报。实验性离散航向候选及后端票据执行已经实现，单个人工选择的候选已实测；自动选点效果与公平对照尚待验证。最新进展、证据和下一步以 [项目交接](../PROJECT_HANDOFF.md) 为准。

离散航向实验需同时为执行器设置 `UAV_ENABLE_YAW=1`、`UAV_ENABLE_NBV_YAW=1`，并为 Agent 设置 `UAV_NBV_HEADING_MODE=discrete` 与 NBV 搜索模式。候选仅允许固定的左右 45° 偏移，目标航向由执行器生成并绑定一次性票据；移动和旋转完成前关闭采图，完成后只接受晚于实际到位时间的相机图像。关闭这些开关时保留固定航向。该模式尚未解决多视角目标类别误报。

可用 `gazebo_seed:=17` 启动独立仿真，再为 `scripts/validate_autonomous_search.py` 指定 `--world` 与 `--gazebo-seed 17`。验证器检查实际 `gzserver` 进程的种子、世界文件及 lockstep，而非只相信启动参数。对照时需重启独立实例并使用相同任务、地图、预算和模型；`scripts/compare_nbv_heading_runs.py` 会检查这些可记录条件及世界坐标系中的首个观察姿态。Gazebo 相同种子不能使远程模型和运行时间确定化，单对飞行不能证明统计收益。

## 固定航向多视角确认（2026-10-27，首轮真实证据）

2026-10-31 的 v2 正在真实闭环验证，以下补充取代本节中仅使用源 ROI 和固定框中心的旧实现说明：

- 配对输入包含源完整图、源框 ROI、当前完整图和各当前候选框 ROI。每个裁图记录照片 ID、框、源/派生 SHA256 与精确归档文件名；选中的当前 ID 必须实际展示过其 ROI。源和当前 ROI 篡改分别有拒绝测试。
- 未定位且有框贴边时使用 `clipped_regions_v2`：对源/当前框内部各取 5×5 校准射线，检查正深度 0.5～20 m、至少 3° 的**假设射线夹角**、0.5° 角残差以内的射线接近、框内投影，以及至少四对、双方各轴均分散的支持。它只说明两个观测区域在几何上相容，**不是已匹配同一图像特征、实际目标视差、类别证明或新定位测量**。不保存相交 XYZ。非裁切框继续用原中心射线；有效双目继续复用原 TargetMemory。
- 两个方向都要求唯一：多个当前候选可关联同一源对象，或多个源对象可关联选定当前对象，均拒绝确认。事件保存 `geometry_policy`，旧记录缺省 `center_v1`，历史未确认不能按新策略改写成成功。
- 确认评分按目标预计投影大小和画面边缘余量调节可见性，减少只保留边缘碎片的视点；探索评分不变。仍是启发式，不保证实际目标入镜。
- rejected/unconfirmed 后若当前真实图明确支持某个已报告 ID，保存该图为新候选并继续确认，不再丢掉新线索返回覆盖探索。必须还有原搜索预算，下一独立视角仍必需，不能因此单帧完成或延长预算。
- `--require-target-confirmation` 验证真实配对复核；进一步使用 `--require-confirmed-target` 或 `--require-rejected-candidate` 指定所需结果。v2 审计同时检查 `search_outcome` 与实际工具结论一致，应用 succeeded 本身不算发现目标。

`UAV_TARGET_CONFIRMATION=single_frame|multiview` 默认 `single_frame`，后者要求 NBV。`multiview` 下首个匹配报告只产生候选，状态为 exploring → confirming → awaiting_review；实际换位采图后，模型收到源完整图、注明来源的源框 ROI 和当前完整图，再调用 `review_target_candidate`。confirmed 才结束寻找；rejected/unconfirmed 保留原始报告并继续探索，不把未检出解释为不存在。

复用同一 `SemanticMap` / `TargetMemory`，生命周期只保存地标与照片 ID。确认规划允许零新增覆盖，评分为目标预计可见性（权重 1）＋视差（权重 0.5）－原有运动/重复代价；保留原路径预算、票据和整路径范围检查。视觉遮挡使用原始 AABB，只有精确表面终点接触作 0.1 mm 裁剪，没有目标障碍豁免。双目表面点的有界不确定度探针、无深度时的 2/4/6/8/10 m 方位距离样本均是**预测假设**，不写成对象 XYZ。

确认需要新照片、至少 0.5 m 实际观察基线、唯一几何对应，以及模型对两个实际视角的支持。两次双目测量复用原关联；单侧深度检查另一图投影；两侧均无深度检查至少 3° 视差、正距离和框内射线兼容性，不保存交会 XYZ。类别仍是未独立标定的模型判断，几何一致性不能证明类别。等待复核时发生 epoch/reset 也必须拒绝确认。无安全确认视点时记录 `view_unavailable` 并回到探索。

ROI 是原图派生数据，不算新照片/视角，记录源哈希、像素裁剪范围、派生哈希，并把模型实际输入 JPEG 单独归档。`scripts/audit_target_confirmation.py` 检查归属、实际观察顺序/基线、候选对应、配对图片投递和裁图字节；与原 NBV/地图审计共同使用。验证器新增 `--require-target-confirmation`，要求至少一次实际两图复核，目标未找到也可能是有效搜索结果，不能只看应用 succeeded。

候选展示同时改为中性 ID 顺序，只提供各项原始收益/代价与确认可见性/视差；不向选择模型展示数值最佳 ID 或加权总分。完整分数与数值基线仅在内部审计中保留。尚未以新版完成公平 VLM 收益比较。

首轮真实任务 `68b73812a3df4f8c`：398.9 秒、8 次观察、7 次票据移动、22 次模型调用，完成两次实际换位配对复核，均为 unconfirmed，预算用尽后返航降落上锁并正常关闭。执行/地图/配对来源审计通过，**目标确认没有通过**。第一确认移动覆盖增益为零；双目不可用，未写入对象 XYZ。裁切/遮挡使框中心方位不稳定，当前严格中心射线对应未能确认实际可见的车辆，不能用调低阈值强行验收。

同次测试还暴露最终模型结尾把 unconfirmed 说成已确认。`web_server.multiview_search_result` 现按结构化搜索/复核生成结论并保存 `search_outcome`，原模型文字另行留档。修复已有真实记录离线回放与受控 Agent 图验证，尚未以修复后版本重飞。原实测 `result.json` 不改写，详细证据在 `logs/setup/target-confirmation-20261027/validation.md`。下一步优先当前框 ROI、裁切框对应和同类歧义，再补真实 confirmed 正例与否定后续搜负例；航向另行实现。

## 职责与接口

- `viewpoint_planner.py`：候选生成、固定尺度评分、有限预算的路径评估、候选 ID 选择。
- `viewpoint_geometry.py`：`StaticViewpointEvaluator` 在注册静态地图上做只读查询；`RemoteViewpointEvaluator` 为内部 HTTP 适配器。都不读取 Gazebo 目标语义真值。
- `static_navigation.py`：新增 `preview_path`，保留实际执行的 `last_plan`，在 A*、起终点连接和路径简化的所有阶段约束整个路线。
- `bridge.py`：`/navigation/viewpoints` 提供 `context`、`visibility`、`paths` 三种查询。重计算在控制器锁之外，返回前再次核对地图身份、门禁和飞机状态。

典型内部调用：

```python
planner = ViewpointPlanner(ViewpointConfig(mode="semantic"))
prediction = planner.evaluate(semantic_map, evaluator)
selected = planner.select(semantic_map)  # 也可提供本批已验证的 candidate_id
```

`selected` 仍标记 `execution_authorized=False` 与 `requires_executor_revalidation=True`。这不是执行接口。返回值被调用方修改不会改写规划器私有的已验证候选批次；地图更新、身份失效或评估失败后，旧批次不能继续选择。

## 候选与视场

在当前 ENU 位置周围取水平 8 个方向、2/4 m 距离，以及当前/升 2 m/降 2 m 三层高度；另包含原地升降。高度裁剪为 1～10 m，去除重复、零位移和起点 12 m 圆外的位置。通常生成 50 个候选；在高度边界因重复消除可降至 33 个。

候选位置保持固定航向，不生成旋转动作。真实后端采用执行器锁定的 commanded yaw；离线回放采用最后真实照片的观测航向。预测假定机身水平，不能把其视场当成未来真实采图已验证的视场。

先用膨胀碰撞几何检查候选端点，再用未膨胀碰撞 AABB 和相机投影得到地面中心的预计可见单元。没有新增几何覆盖的候选本阶段不继续做 A*。预测不会调用地图的 `update_observation`，不增加覆盖、照片或目标证据。

## 固定尺度评分

```text
score = coverage_weight × 新增覆盖面积 / coverage_scale
      + semantic_weight × 新增语义加权面积 / semantic_scale
      - motion_weight × 路径估计秒数 / motion_scale
      - repeat_weight × 重复视角相似度
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| 覆盖权重 / 面积尺度 | 1 / 100 m² | 单元面积 0.25 m²，累加 `(1-coverage)` |
| 语义权重 / 面积尺度 | 1.5 / 25 m² | 语义分数来自实际地标与显式用户关系 |
| 运动权重 / 时间尺度 | 0.25 / 10 s | 实际规划路径长度 ÷ 名义 0.65 m/s |
| 重复权重 / 位置尺度 | 0.2 / 1.5 m | 对实际历史观察点取最大高斯位置相似度 |
| 完整路径预算 / 每批 | 12 / 4 | 与数十个快速候选分开 |
| 查询间源位置容差 | 0.35 m | 超过后拒绝整个批次；固定航向及标定必须一致 |

`mode="geometry"` 将语义权重置零，保留同样的候选、地图、路径预算与其它代价；`mode="semantic"` 使用上表权重。这两个确定性基线不调用模型做选择。

归一化尺度不会在每轮按候选最大/最小值改变。分项并非概率，也不保证都在 0～1 内。权重是首版工程配置，尚无训练、概率标定或性能调参结论；这不是严格熵下降或期望信息增益。

## 路径预算与“最优”的范围

用直线距离减去容许的源位置变化，给运动成本建立下界，由此得到候选分数上界。按上界从高到低分批调用真正三维 A*，使用完整路径长度重新评分。

当剩余候选的上界已低于当前最佳已验证分数时，可以停止评估；若达到预算后仍有可能更好的未验证候选，则明确返回 `path_budget_limited`。没有已验证可达点而预算尚未覆盖所有候选时，返回 `path_budget_exhausted`，不误称整个区域无路。

- `planned`：本次生成的离散候选中，已评估或利用上界排除其余候选。
- `path_budget_limited`：有可达选点，但不能证明它优于所有未验证候选。
- `no_useful_view`：没有预测到新地面几何覆盖。
- `no_reachable_view`：有用候选均已检查路径并不可达。

都不表示连续空间全局最优，也不表示目标可以识别或任务将成功。

## 完整路径约束与只读保证

`preview_path(start, goal, search_origin_xy, search_radius_m=12)` 把搜索圆与 1～10 m 高度限制纳入 A* 邻居、连接段和简化段。圆盘与高度区间是凸集，因此连续线段的两个端点都在其中即可保证整段在其中；碰撞则继续用连续 AABB 线段检查。

测试包含“终点在圆内、唯一绕障路必须出圆”的反例，该路径必须拒绝；不能仅检查最终目的地。既有非搜索任务默认不附加这个搜索圆限制。旧 `/navigation/preview` 也修正为不再覆盖实际 `last_plan`。

规划器还验证返回路径的起点、目标、全部顶点、连续路径长度与权限标识。只对坐标旋转/反变换的起终点比较采用 1e-8 m 数值容差，搜索圆和高度范围采用独立的 1e-9 m 舍入界，碰撞检测没有因此放宽。

world SHA256、架次随机标识、完整 VIO/EKF epoch 和世界/ENU 配准都必须匹配。地图修订摘要覆盖地图内容，不只看照片数量；评估中地图变动也会使本批失效。

**执行器已实现 NBV 搜索范围绑定，并取得有限真实闭环证据。** 活跃 NBV 搜索不接受未经票据授权的 generic goto；候选执行解析服务端缓存的目标，实际 A* 重规划继续使用相同起点搜索圆与高度限制。不可将离线选点直接转成普通 goto 来绕过该流程。

## 搜索生命周期与候选执行

`AdaptiveSearch` 保留状态机，空间地图、几何查询、候选评分仍分属独立模块。`single_vlm` 的搜索器与报告处理共享同一 `SemanticMapRuntime`；每次有效的已保存报告进入地图后，后续候选使用新的地图修订摘要。地图失败时停止 NBV，按故障路径降落，不静默回退固定动作。

### 可选 VLM 候选 ID 选择

`UAV_NBV_SELECTOR=deterministic` 为默认纯数值选择；`vlm` 必须配合 `nbv_geometry` 或 `nbv_semantic`。两种选择器复用相同的候选生成、评分、路径预算和执行器票据，不新增飞控接口。

VLM 模式的 `next_target_search_view` 先返回最多 8 个已经通过路径检查的候选，提供 ID、相对方向、高度、覆盖/语义收益、路径长度/耗时及评分，不向模型返回航点 XYZ 或票据。这里是有限路径预算内已经验证的候选池，通常只有 4 项；并不保证全部 50 个初始候选或每个方向都向模型开放，未验证候选不允许选择。

模型调用 `select_target_search_view(batch_id, candidate_id, reason)` 后才产生待执行观察点。等待选择时，按 owner 取回上一张已保存且投递成功的实际照片，重用于这次选择；不重新读取相机、不重新提交报告或增加地图覆盖。普通确定性模式仍不重发已报告图片。选择窗口为 60 秒，小于执行器票据的 90 秒；反复请求 next 不刷新批次。地图修订、epoch、架次配准、源位置或批次失效后拒绝，不静默回退数值第一。

`nbv_candidate_events` 记录提供的批次，`nbv_events` 记录最终选择，保留 `numeric_best_id` 用于对照。选择只接受本批提供的 ID；选定后不能重选或重放，实际飞行仍由候选票据、当前状态与完整路径重验授权。单帧目标终止规则没有因此变为多视角确认。

首轮真实验证：`2d11eb29d6664bae`，322.1 秒、5 图/4 次 VLM 选择，发现原图可见的白色车辆后返航上锁，VLM/执行/图像地图审计通过；系统 Python 3.8 也重审通过。4 次选择均等于数值第一，且多了 4 次源照片选择调用。首轮 16 个升高候选被数值上界剪枝提前排除，提供的 4 个候选均在后方，不能声称 VLM 能充分选择升高/前进。下一步需在有限预算内增加代表性，并用同样的候选池比较两种选择器。证据：`logs/setup/nbv-vlm-selection-20261027/validation.md`。

### 多样候选与报告后自动准备（可选）

`UAV_NBV_CANDIDATE_POOL=ranked|diverse` 默认 ranked。diverse 先保留原数值优化过程，再用剩余预算检查尚无代表的高度/相对方向组，优先原地升高、前方升高、前/左/右平移等。全部检查共用原 `max_path_evaluations=12` 上限、同一个整路径验证器；若初始数值检查耗尽预算，或代表点不可达，不保证所有方向均可提供。结果明确记录额外检查 ID 和没有已验证代表的有用组。

最终池最多 8 项，包含数值第一名和各组最佳已验证代表；不足时用其它已验证高分项填充。`candidate_pool_ids` 与按分数排序的 `ranked_candidates` 分开，VLM 只能选择池内 ID。确定性与 VLM 选择器共用此配置，不能让对照偷偷使用不同候选或路径预算。数学得分、碰撞检查和基于实际图像更新覆盖的规则均保留。

`UAV_NBV_AUTO_PREPARE=1` 让有效 `report_scene` 保存且进入地图后自动准备下一观察批次，返回新的 `search_progress.next_tool`。VLM 直接选 ID，确定性选择器直接等待观察执行，省去要求模型再调用 next 的一轮；准备本身没有飞行。发现目标、预算结束或地图失败时不准备新批次，旧 next 接口仍可读取当前同一批，不刷新票据或时间窗。默认 0 保留旧调用顺序。

相同真实首帧离线回放：ranked 检查 4 条路径、提供后方 2 类；diverse 检查 10 条路径、提供 8 个代表，含原地升高、前方升高与前/左/右平移，数值第一不变。地图和执行计划不变。152 项 Python 3.11 相关测试、系统 Python 3.8 的 25 项规划/审计测试通过。

真实补验 `b2d09be026a24c25`：296.2 秒、5 图/4 次选择，路径检查 10/8/10/10、所有新验收通过，最终返航上锁。模型没有额外 next 调用，总调用 18 次（上一轮显式 next 为 22 次）。全部选择仍等于数值第一，展示基线 ID/总分可能存在锚定；不证明独立决策价值。下一阶段需中性展示与主动确认，基线只在内部审计保留。证据 `logs/setup/nbv-diverse-pool-20261027/`。

```bash
UAV_SEARCH_MODE=nbv_geometry UAV_NBV_SELECTOR=vlm \
UAV_NBV_CANDIDATE_POOL=diverse UAV_NBV_AUTO_PREPARE=1 \
  .venv/bin/python scripts/validate_autonomous_search.py --execute \
  --prompt '找到前方的白色货车，拍照分析并报告周围可见环境，完成后返航降落。' \
  --require-nbv --require-vlm-selection --require-diverse-pool --require-auto-prepare \
  --world scene/test_environments/courtyard_delivery.world \
  --output logs/setup/nbv-diverse-pool-20261027/geometry-vlm-auto
```

新验收标志要求真实执行采用 diverse 池、实际提供过升高及多个方向组，且候选由报告准备、模型没有冗余 next 调用；仅设置环境变量不算通过。方向齐全不保证模型会选择升高，也不证明视觉目标正确或整体性能改善。

验证命令（会按已获得的授权调用现有模型和仿真飞行）：

```bash
UAV_SEARCH_MODE=nbv_geometry UAV_NBV_SELECTOR=vlm \
  .venv/bin/python scripts/validate_autonomous_search.py --execute \
  --prompt '找到前方的白色货车，拍照分析并报告周围可见环境，完成后返航降落。' \
  --require-nbv --require-vlm-selection \
  --world scene/test_environments/courtyard_delivery.world \
  --output logs/setup/nbv-vlm-selection-20261027/geometry-vlm
```

输出必须新目录。`--require-vlm-selection` 核对原始候选批次、实际源照片投递、模型选择工具结果与理由、最终候选和执行 mission；纯数值执行不能冒充 VLM 选择。它验证选择链，不证明模型优于数值策略或视觉识别正确。

`POST /navigation/viewpoints` 可附内部 `search_context`：`owner`（真实线程及任务 ID）、`map_revision`（64 位十六进制摘要）、`batch_id`（32 位十六进制）。`context` 绑定真实采图的注册身份和搜索原点；后续查询必须属于同一批次。只读调用不附该字段时不建立飞行授权。

已验证的可达候选由 `viewpoint_authority.py` 签发 90 秒有效的执行票据。票据绑定 owner、地图修订、候选批次/ID、世界与坐标身份、源位置、固定航向和缓存目标。对外模型摘要及 `nbv_events` 删除票据，只保留候选 ID 和评分。执行请求格式如下，不接受坐标：

```json
{
  "owner": ["实际线程 ID", "实际任务 ID"],
  "batch_id": "本批 ID",
  "map_revision": "本批地图修订",
  "candidate_id": "view_001",
  "ticket": "执行器签发的内部票据",
  "description": "NBV observation"
}
```

接口为 `POST /navigation/execute-viewpoint`，由内部客户端调用。接受前重新验证当前 VIO、身份、源位置变化不超过 0.35 m、固定航向和票据时效；一次接受会作废该批所有替代票据。实际 mission 附带 `search_constraint` 和候选 ID，检查点继续核验身份、归属及跟踪范围，实际重规划检查整条路线。规划目标严格限制在半径 12 m、ENU 高度 1～10 m 内；实际跟踪另有 0.35 m 偏差门限，不应混为规划许可。高度边界附近的实际姿态与规划源点兼容性尚待真实验证。

纯降落独立于地图/坐标身份门禁；成功降落且确认地面上锁后释放搜索。以 hold 收尾时仅在移动结束后释放本 owner，失败任务改为降落收尾。操作员停止会撤销票据，同任务不能重新绑定。`AuditedClient` 对新执行接口沿用纯观察、中断、视觉/飞行失败限制，以及 accepted/waited mission 审计；发送异常后禁止自动重试候选。

模式在 Agent 创建时读取，例如 `UAV_SEARCH_MODE=nbv_geometry`；修改后需要正常重启 Agent。闭环验证命令（会调用现有视觉服务并飞行，须先具备相应授权）：

```bash
UAV_SEARCH_MODE=nbv_geometry .venv/bin/python scripts/validate_autonomous_search.py \
  --execute --prompt '找到前方的白色货车，拍照分析并报告周围可见环境，完成后返航降落。' \
  --require-nbv --world scene/test_environments/courtyard_delivery.world \
  --output logs/setup/nbv-closed-loop-20261026/geometry
```

输出必须为新目录，失败也保留。另用独立架次验证 `nbv_semantic`；最终应检查真实照片、地图事件、`nbv_events`、实际 `navigation_plans`、完整路径范围及最终地面上锁状态。此命令记录不表示已经执行成功。

### 闭环证据审计

仅应用返回 `succeeded` 不足以验收 NBV；首帧发现目标、直接降落或原固定航线都可能没有执行过候选。`--require-nbv` 在发送任务前核对模式和世界指纹，终态后执行 `scripts/audit_nbv_evidence.py`；缺少必要证据则验证脚本返回非零。原结果目录不会被覆盖。失败任务仍保留 `result.json` 和审计失败原因。

新归档包含以下证据：

- `nbv_events` 及对应照片的 `.nbv_geometry.json` / `.nbv_semantic.json`：源照片、owner、批次、地图修订、候选评分；不包含票据。文件原子替换，避免读取半写入 JSON。
- `flight_missions`：被接受的 mission，`accepted_via` 区分候选执行与普通任务。
- `flight_mission_results`：已接受、已完成的 mission 与实际规划路径；下一任务替换 `/state` 前保存，重复工具审计不重复计数。

独立审计可在断网时运行：

```bash
/usr/bin/python3 scripts/audit_nbv_evidence.py \
  --result logs/setup/nbv-closed-loop-20261026/geometry/result.json \
  --world scene/test_environments/courtyard_delivery.world \
  --output /tmp/px4-nbv-execution-audit
```

审计核对候选源地图摘要、真实任务归属、唯一票据接受与完成记录、实际候选路径的完整搜索范围和长度、独立加载的已注册静态碰撞几何、到达位置，以及下一张照片是否在候选位置扩展同一地图。还调用已有语义地图审计，验证 JPEG 哈希、图像投递、报告、在线/回放一致性和去重。记录预测覆盖收益、实际新增几何覆盖、实际**规划**路径长度；不把规划长度当实飞里程，也不把这些证据当成连续真实轨迹或类别正确性的独立验证。

验证语义机制时再加 `--require-semantic-guidance`（验证器同时要求 `--require-nbv` 与 `UAV_SEARCH_MODE=nbv_semantic`）。只有源观察地图中语义场已经有效、该次执行候选的语义收益及加权语义评分均大于零，才计作一条实际语义引导移动。仅打开 semantic 模式或仅在最后终止帧生成语义场不满足此项；它仍不证明相对于几何基线改变了最优点或提高发现率。

### 2026-10-27 真实闭环进展

- 几何轮：任务 `de961d83a4b24f0c`，301.3 秒、5 帧、4 次票据移动，119→565/907 格，返航降落上锁；`--require-nbv` 全部通过，最后原图可见白色车辆。证据 `logs/setup/nbv-closed-loop-20261027/geometry/`。
- 第一语义轮：任务 `bb73559fec384c0a`，158.4 秒、2 帧、1 次移动，执行链与图像地图审计通过；但原图人工检查未见所报告的蓝色油桶，必须保留为视觉误报。唯一移动时语义场为零，新语义验收标记正确拒绝（退出 2）。证据 `logs/setup/nbv-closed-loop-20261027/semantic/`，包括 `manual-review.json` 和 `guidance-audit/`。
- 显式关系补验：任务 `9eec461726a243c2`，429.0 秒、8 帧/7 次移动，首帧红色地标定位后 7 次候选都有正语义项；覆盖 671/888（75.56%）。未检出紫色工具箱，预算耗尽后返航上锁，完整及非零语义引导审计通过。相同实际首帧的离线两模式选点不同（view_047 / view_043），生产首步选 view_043。证据 `logs/setup/nbv-closed-loop-20261027/semantic-guidance/`。
- 不能把不同指令/高度的测试当公平性能对照。单帧标签终止问题尚未修复，多视角主动确认仍是后续工作。

历史 `multiview-fresh` 虽然图像与地图审计成功，但没有 NBV 候选执行，已被此工具正确拒绝（退出码 2）。缺少新字段的旧日志不能补猜字段后宣称验收；合成归档通过仅证明工具链，仍须真实 NBV 架次。

## 只读后端接口

`POST /navigation/viewpoints` 的固定字段：

```json
{
  "operation": "visibility",
  "pair_id": "来自本次双目采图缓存",
  "origin_xy_enu_m": [0.0, 0.0],
  "expected_identity": {
    "world_sha256": "实际世界文件指纹",
    "sortie_id": "实际注册标识",
    "coordinate_epoch": {"vio_epoch": 0, "ekf_counters": {"xy": 2, "z": 2, "heading": 2}},
    "registration": {"world_from_enu_origin_m": [0.0, 0.0, 0.0], "world_from_enu_yaw_rad": 0.0, "ground_world_z_m": 0.003}
  },
  "candidates": [{"id": "view_001", "position_enu_m": [2.0, 0.0, 5.0]}]
}
```

数值只是结构示例，不能当真实身份。`context` 要求 candidates 为空；`visibility` 最多 64 项；`paths` 最多 8 项。候选只含 ID 和位置，不接受模型给出的任意姿态。实际相机标定、锁定航向、当前位置和健康 epoch 来自执行器；缓存采图 pair 必须属于当前注册与 epoch。

后端要求静态导航已激活、实际在空中、当前飞行步骤已结束、无人机没有在导航移动、VIO 门禁有效；不在飞行步骤执行过程中插入候选计算。计算期间没有持有控制发布锁，结束后再核对健康、源位置、航向和注册，失配则丢弃结果。预测可用性不等于当前桌面已加载新接口，旧进程需要正常重启后才可使用。

## 离线真实档案回放

在项目根目录运行，输出目录必须是新目录：

```bash
/usr/bin/python3 scripts/replay_viewpoint_planner.py \
  --result logs/setup/semantic-map-evidence-20261026/multiview-fresh/result.json \
  --world scene/test_environments/courtyard_delivery.world \
  --output /tmp/px4-nbv-replay
/usr/bin/python3 scripts/render_semantic_map.py \
  /tmp/px4-nbv-replay/semantic/map-with-predictions.json \
  --output /tmp/px4-nbv-replay/semantic/map.png \
  --world scene/test_environments/courtyard_delivery.world
```

脚本重新校验真实 JPEG、模型投递和地图证据，从实际归档恢复配准用于离线分析；不会把旧注册移交给正在运行的仿真。几何/语义两模式分别保存分项评分、已检查路径、未验证候选、选择结果和带预测的地图快照。紫色点/星/虚线为预测候选、选点、路径，未实际执行；不能与绿色历史规划路径混淆。

本机验证记录在 `logs/setup/nbv-offline-20261026/`，该目录被 Git 忽略。单个回放不足以证明成功率、航程或耗时改善。实际地图里保留的误框候选也仍然存在，本模块不能纠正视觉标签。

## 下一步验收

1. 用实际油桶误报回放推进候选确认状态与多视角复核；确认模式应允许零新增地面覆盖，按候选可见性/视差选点，复用共享记忆。
2. 移除模型展示中的数值第一标签/总分锚定，内部保留基线审计；先离线/模拟图、再实际闭环，对照共享候选池配置。
3. 扩展边界和故障复验，错误或坐标重置只允许安全收尾；已有正常架次不能代替故障与极限高度验收。
4. 同场景、种子、预算比较固定策略、几何 NBV、语义 NBV，保留原始失败和实际图像人工复核。
5. 后续阶段再做航向控制与主动目标确认，避免把单帧误识别直接当任务成功。
