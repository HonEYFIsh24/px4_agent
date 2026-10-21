# 第一优先级：可靠性测试与故障处理

本阶段先验证系统在障碍变化、输入故障和模型故障下是否保留真实结果、停止移动并正确处理降落。暂不接入 VINS 飞控闭环，也不扩大到三维导航。当前运行范围仍为固定本地 ENU 高度 **3.5 米**、静态二维激光导航，最小规划净空 **1.0 米**。

第一优先级的测试建设与本地实测已完成，结果以 [validation.json](../logs/reliability/validation.json) 为准。三个固定 seed 的随机障碍世界都完成绕障往返；封路、雷达中断/延迟/非法值、持续近障碍、临时近障碍恢复，以及 VIO 健康状态变化均留下实际证据。全部失败历史保持原样。

| 统计范围 | 完成记录 | 结果 |
|---|---:|---|
| 实际 Qwen API | 6 次 | 4 次正常、2 次异常 |
| 实际 Agent 端到端基线 | 2 次 | 0 次端到端通过；其中 1 次原飞行任务完整完成 |
| 真实执行器套件实例 | 13 次已提交 | 11 次预期安全合同通过，7 次原任务完成，12 次实际起飞 |
| 离线真实 LangChain 图 | 15 个故障场景 | 全部符合各自预期合同；模型与 bridge 为本地 fixture |
| 完整单元测试 | Agent Python 3.11：359；ROS Python 3.8：297 | 均 OK；ROS 跳过 16 个 Agent 依赖项 |

**安全停止或降落通过，并不代表原用户任务完成。** 13 个套件实例包括一次解析错误、未起飞的基础设施失败；两个套件安全合同失败也没有从分母中删除。首个中止套件余下 9 个计划项标为未执行，不冒充通过，也没有待运行项。这些是小规模、不同条件的实际记录，不能解释为系统总体成功率。上一阶段记录见 [自主绕障文档](autonomous_avoidance.md)。

## 1. 三层测试范围

| 层次 | 使用的真实组件 | 替代或注入的组件 | 结果可以证明什么 |
|---|---|---|---|
| 离线模块与评价回归 | 生产导航、故障适配器、bridge 回调方法、场景注册及独立轨迹评价代码 | ROS 消息、时间与状态由本地 fixture 提供 | 参数限制、故障锁存、等待逻辑及评价拒绝假阳性的合同正确；不能证明飞行通过 |
| 离线 Agent 故障流程 | 实际 LangChain 图、`single_vlm` 中间件、工具参数校验、WebApplication、AuditedClient 和恢复逻辑 | 模型为本地 scripted BaseChatModel；飞行 bridge 为有状态 fixture；网络发送被阻断 | 模型故障后没有多余移动或自动重试，报告、图像时效和操作员控制权正确；没有调用真实 Qwen、没有实际飞行 |
| 本地实际 SITL | PX4、Gazebo、MAVROS、实际传感器与执行器 | 可在明确开启的测试模式下修改进入执行器的测量副本；环境真值仅供评价 | 实际移动、暂停、停稳、恢复或降落符合要求。直接执行器测试不经过模型；自然语言基线另行经过真实 Qwen |

真实执行器脚本在报告中写入 `actual_model_call: false`。真实自然语言脚本记录实际模型调用与 mission 事件。离线 Agent 的报告明确标注 `offline_fault_injection_real_langchain_graph`，不能把其本地模型异常说成提供商故障。

相关入口：[故障适配器](../reliability_faults.py)、[离线 Agent 流程](../scripts/test_agent_reliability.py)、[实际执行器测试](../scripts/test_navigation_reliability.py)、[真实 Qwen 绕障基线](../scripts/test_avoidance_scene.py)。

## 2. 可重复随机世界与真值隔离

[生成器](../scene/generate_reliability_worlds.py) 深拷贝已验证的 `obstacle_yard.world`，保留原光照、地面、材质、物理参数和周边设施，只改变高墙及三根立柱的位置。原世界、manifest、layout 和任务文件的字节保持不变。注册入口是 [scene/reliability/index.json](../scene/reliability/index.json)。

| 场景 | 固定 seed | 离线预期 | 隐藏路线保守净空下界 |
|---|---:|---|---:|
| `reliability_seed_17` | 17 | 到目标后绕障返航降落 | 1.98 米 |
| `reliability_seed_73` | 73 | 到目标后绕障返航降落 | 1.98 米 |
| `reliability_seed_211` | 211 | 到目标后绕障返航降落 | 1.98 米 |
| `reliability_blocked_17` | 17 | 无路时停止，独立降落 | 不存在合法固定高度路径 |

三个可通行场景各有 30 个物理障碍记录，两墙间净宽超过 4.5 米，墙端与立柱位置不同。完全封路场景有 29 个障碍，第一道墙的实际碰撞几何连续覆盖 `y ∈ [-8,8]`、高度 5.5 米，且 home 与目标位于两侧；重叠的南端货堆被删除。

每场景保存四种资产：

- `.world`：Gazebo 加载的实际物理和视觉几何。
- `_manifest.json`：运行许可，只含场景 ID、世界哈希、地面、飞行区域、起飞区和导航约束。
- `_layout.json`：仅用于离线评价，包含实际碰撞 AABB、随机参数和可通行证书；封路场景没有伪造 witness。
- `_tasks.json`：任务配置，提交给 Agent 的文本始终只指定目标 `(13,0,3.5)`，不含绕行中间航点。

`index.json` 的 `schema_version=1`，资产路径相对其目录，保存每资产 SHA256。运行时 [scene_config.py](../scene_config.py) 只接受固定项目目录内注册的 world/manifest 配对，并校验 world 和 manifest 哈希。路径逃逸、外部复制文件、错误配对和篡改不能获得可信场景权限。运行时不读取 layout、witness 或障碍位置。

隐藏路线证书以不超过 2 厘米的步长检查所有实际碰撞 AABB，并减去完整采样步长，得到保守距离下界。独立测试又用精确的线段到矩形距离检查整个 witness，确认下界没有高估。证书证明离线几何可行性，实际飞行是否通过仍需逐次记录。

## 3. 故障注入边界

故障模块默认关闭，只有明确设置 `UAV_RELIABILITY_TESTING=true` 且确认本地仿真时才启用。故障请求不是模型工具，只由测试脚本设置本地 ROS 参数 `/uav_agent_executor/reliability_fault`。适配器不读取场景对象，不直接写栅格分数，不减小净空，也不控制飞机。

| 故障 | 注入方式 | 期望执行器行为 |
|---|---|---|
| 雷达中断 | 暂时丢弃进入执行器的实际测量副本 | 新鲜度门关闭、停止移动；有效测量恢复也不自动解除本次传感器故障锁存 |
| 雷达延迟 | 副本源时间戳向过去偏移 | 同步或新鲜度检查拒绝；保留传感器失败分类 |
| 非法雷达值 | 一个副本射线设为负无穷 | 拒绝整个无效扫描，停止并保留故障 |
| 临时近障碍 | 在有效、时间对齐姿态下增加固定 ENU 端点的近距离返回 | 正常射线更新地图，暂停停稳；起点净空恢复并稳定后再规划 |
| 持续近障碍 | 保持上述返回超过等待期限 | 等待 8 秒失败，保持原 mission 失败并独立降落，不擦图或逃出阻塞起点 |
| VIO 跟踪丢失 | 在测试输入层强制跟踪状态失效 | 可观测健康状态变化；当前飞控仍使用 MAVROS 定位，VIO 不参与控制 |
| VIO 重置 | 发布实际上游 `/vins_restart` | 记录真实 estimator 重启与恢复，禁止伪造健康里程计 |

注入持续时间为墙钟秒，最大 30 秒，单调时钟到期恢复。**请求受理有效期与激活持续时间不同**：请求的 `expires_at_wall_s = issued_at_wall_s + duration_s + 5` 给出受理窗口；激活后以单调时钟运行 `duration_s`，并受剩余请求有效期约束，不额外持续 5 秒。例如临时 10 秒用例的请求受理窗口为 15 秒，实际激活持续约 10.013 秒；跟踪健康用例请求窗口 8 秒，激活约 3.024 秒。

请求带明确 `synthetic` 和 `simulation_only` 标签、唯一 ID、有效期与原射线证据；结束后的 ID 不能重放。近障碍副本保留更近的真实返回。没有有效时间对齐姿态或端点超出合法量程时，保留原扫描并记录拒绝，不能把这类尝试当作已经成功注入。底层另有 `vio_drop` hook，但本阶段只覆盖其单元测试，没有安排实际飞行，不计为实测通过项。

本地 ROS 参数读取已移至独立轮询线程；实时扫描、状态和飞行回调只使用缓存，避免测试参数请求占用传感器回调的锁，并增加拒绝时的源时间偏差及接收年龄诊断。后续批次的合同通过不能反推首轮瞬态拒绝的唯一根因；原始证据仍不足以确认根因。

## 4. 验收要求与测量限制

成功往返要求原 mission 成功，并具备起飞、到目标、返航、落地事件。目标到达还需在 goto 完成事件前后 1.5 秒内，有实际 armed、fresh world pose 的样本，其 ENU 位置距目标不超过 0.65 米。成功和封路用例的唯一目标为 `(13,0,3.5)`；局部故障用例用 `(2,0,3.5)`，不得把短程故障飞行算作完整环境绕障成功。

预期安全失败要求原 mission 保持 failed，没有伪造已完成的 goto 或 return_home，且另一个独立降落 mission 完成，最终落地上锁。报告分别保留 `validation_passed` 与 `original_mission_status`：前者可能表示安全失败处理通过，后者仍是失败。

所有实际用例保持 **1.0 米净空**。轨迹评价独立读取实际机体世界坐标，对记录点和每条相邻点连线计算到纯碰撞 AABB 的精确二维距离，并报告最小值与采样间隔。禁止把许可边界当作已知无障碍地图，也不使用隐藏路线作为控制输入。

故障飞测默认每 **0.25 墙钟秒**记录一次状态。停止指标从暂停附近的世界 XY 位置开始，要求世界 XY 速度不超过 0.15 m/s 的记录连续覆盖至少 1 墙钟秒，停止过程记录的最大 XY 位移不超过 0.5 米。首个暂停后样本晚于 0.5 秒、或停稳窗口出现超过 0.5 秒的采样空档，均不能证明停稳。最近的暂停前样本被保留为距离原点，避免丢失首帧前已经发生的位移。

这些净空值来自**机体中心的世界 XY 坐标到碰撞 AABB 的距离，及相邻记录点之间的连接线**，不是连续时间安全证明。点间实际曲线、未采样瞬间和真实飞行硬件的响应不能由本测试直接证明。0.25 秒是请求采样间隔而非保证：VIO 重置用例实际最大墙钟记录间隔为 **2.575890 秒**，三个随机 seed 分别为 0.637589、0.575145、0.506637 秒；停止观测窗口另受上述 0.5 秒覆盖检查限制。Gazebo 当前速度因子为 0.35，墙钟延迟不能直接作为实机延迟。几何评价使用保守 AABB，暂不依赖连续接触传感器。

临时近障碍必须实际进入本次 mission 的 `navigation_clearance_wait`，随后出现 `observed_clearance` 等待结束，再有新规划事件。持续近障碍必须实际等待约 8 秒并因该期限失败。无等待证据、等待仍在进行时计划、等待失败后伪装成功、近障碍误分为传感器故障或错误 fault ID/kind，都会被评价拒绝。

## 5. 已完成的离线验证

完整 Agent 单元测试 **359 项 OK**，见 [Agent 测试日志](../logs/reliability/unit_tests_agent_unittest.log)。ROS Python 3.8 完整测试 **Ran 297，OK，skipped=16**，见 [ROS 测试日志](../logs/reliability/unit_tests_ros_v2.log)；跳过项仅依赖 Agent 环境。首次测试入口或依赖 guard 的检查失败也保持在日志目录中，修正后使用仓库的 unittest 入口完成回归。

以下专项包含在项目回归中，不与完整总数相加：

| 专项 | 当前验证 | 源码或报告 |
|---|---|---|
| 世界生成与独立几何 | 11 项，Python 3.8/3.11 均通过；生成字节一致 | [test_reliability_worlds.py](../tests/test_reliability_worlds.py) |
| 可信场景注册 | 18 项专项回归 | [test_reliability_scene_config.py](../tests/test_reliability_scene_config.py) |
| 故障适配器与 bridge | 最初 19 项；当前源码扩展至 26 项，包含阻塞参数读取、同步诊断与独立轮询 | [test_reliability_faults.py](../tests/test_reliability_faults.py) |
| 真实 LangChain 图的离线故障流程 | 15 个场景全部符合预期；专项单元测试 16 项 | [agent_offline_v3.json](../logs/reliability/agent_offline_v3.json)、[test_agent_reliability.py](../tests/test_agent_reliability.py) |
| 实际飞测评价与传输回归 | 原 20 项加 3 项 case-goal 校验、3 项 HTTP wrapper 与接受后恢复回归，共 26 项，Python 3.8/3.11 均通过 | [test_navigation_reliability.py](../tests/test_navigation_reliability.py) |
| 套件编排与预检 | 22 项，覆盖归属、地面、预检、恢复、显式重复及失败保留 | [test_reliability_suite.py](../tests/test_reliability_suite.py) |
| 只读统计汇总 | 8 项，覆盖失败分母、未执行项、回收补记和端到端/飞行状态区分 | [test_summarize_reliability.py](../tests/test_summarize_reliability.py) |

离线 Agent 的前 10 个场景覆盖起飞前连接失败、起飞后超时、图像请求连接失败、相机失败后阻止移动、未保存图像过期、已保存报告在图像过期后保留、非法工具参数后禁止重试、任务失败后禁止重试、已有他人飞行的观察失败，以及操作员中断优先。另 5 个场景专门验证最终回复阶段出错时的归因：已完成飞行、已保存观察、尚未确认的接受、已失败飞行和已取消飞行。没有把最终文字回复的模型异常改写成已完成的飞行失败，也没有把未确认任务冒称完成。

评价回归实际构造了假阳性反例：相隔很久的两个低速样本、过晚首帧、错误故障种类、拒绝注入但误称应用、近障碍错误分类、等待未结束便规划、失败等待后标成功，以及只有 goto 事件却始终未到目标的轨迹。这些反例已被修复后的评价器拒绝。

## 6. 本阶段真实飞测记录

### 6.1 真实 Qwen 基线：飞行与最终回复分开记录

| 基线 | 原飞行结果 | Agent 端到端 | 墙钟秒 / 样本 | 记录最小净空 | 真实 Qwen 异常 |
|---|---|---|---|---:|---|
| [baseline](../logs/reliability/baseline/autonomous_goal_round_trip/task_validation.json) | `a565d900a89a` failed；takeoff、goto、hold 完成，返航失败；`33d87090942c` 独立降落成功 | failed | 533.319 / 985 | 1.561714423 米 | 前 2 次正常，随后 `APITimeoutError`，45.199 秒 |
| [baseline2](../logs/reliability/baseline2/autonomous_goal_round_trip/task_validation.json) | `0bdf5cad66b1` succeeded；takeoff、goto、hold、return_home、land 全部完成 | failed | 477.482 / 879 | 1.560785507 米 | 前 2 次正常，最终回复 `APIConnectionError`，0.076 秒 |

第一次 command 为 `de1a8a21fce3460d`，执行器报 `LiDAR navigation sensor/freshness gate closed; holding`。失败时出现瞬态对齐拒绝，其唯一根因没有被原记录确认。异步测试参数读取和诊断改动已完成，但不能把第二次飞行成功解释为已经证明根因。

第二次 command 为 `3f8b28f8c02a4be1`。它已完成整个实际飞行任务，但最终模型文字回复失败，所以原 Agent command 仍 failed。生产归因与回归已修正为保留已完成飞行和已保存观察，单独报告最终回复错误；原基线报告不追溯改成通过。本阶段实际 Qwen 共 **6 次调用、4 次正常、2 次异常**；Agent 端到端 **0/2**，原飞行完整完成 **1/2**。这两份真实基线覆盖文本工具任务；新图像故障处理证据来自已注明替身的离线场景。提供商调用异常没有被本地 scripted model 的 15 个通过场景替代。

### 6.2 套件 01：基础设施失败保持原样

[suite_navigation_01 原报告](../logs/reliability/suite_navigation_01/seed_17_01/runner/task_validation.json) 的首例在处理 `/mission` 响应时发生 `KeyError: 'id'`，记录样本为 0，没有可用航迹评价。实际 API 返回 `{"mission":{"id":...}}`，早期测试 fixture 错误使用裸 `{"id":...}`，使响应合同错误没有被离线发现。原报告中的 `actual_flight:false` 也来自该错误处理顺序，不能据此推断没有提交 worker。

[只读回收补记](../logs/reliability/suite_navigation_01/recovery_addendum.json) 确认实际已提交 `c0a387614685`，其最终 failed，原因 `PX4 rejected arming`，没有实际起飞，后来确认落地上锁。原套件因当时仍有 running worker 而中止，原恢复状态 blocked 不覆盖；其余 9 个计划项保持未执行。该基础设施失败继续计入 13 个已提交实例的分母。

运行器现按真实 wrapper 解析，并在解析字段前记录已接受提交。已接受任务即使尚未解锁，其 worker 仍可能正在运行；响应合同异常时先取消 owned worker，再安全处理降落，最终同时确认落地上锁与 worker 终止。新增回归覆盖正常 wrapper、空缺 ID 时的已解锁与尚未解锁窗口，以及已经落地但恢复 worker 仍运行的窗口。尚未获得成功接受响应的提交不能直接当作拥有他人飞行的控制权。套件预检还增加了真实 PX4 预飞行检查与地面稳定观察；预检重测不会提交飞行动作。

### 6.3 套件 02：10 次实际飞行，9 次合同通过

[suite_navigation_02/summary.json](../logs/reliability/suite_navigation_02/summary.json) 为 `completed_with_failures`，10 个实例全部实际起飞，**9 次预期合同通过、5 次原飞行任务完整完成**。所有用例最终落地上锁，套件正常恢复默认世界、GUI 和关闭的测试 hook。

| 实际用例 | 激励墙钟秒 | 原 mission | 预期合同 | 秒 / 样本 | 规划 / 等待 | 记录最小净空（米） |
|---|---:|---|---|---|---|---:|
| 临时近障碍 | 2 | succeeded | **未通过：等待分支未触发** | 135.028 / 519 | 3 / 0 | 1.933186 |
| 持续近障碍 | 20 | failed，独立降落成功 | 通过 | 111.813 / 432 | 1 / 1 | 3.147361 |
| 雷达中断 | 4 | failed，独立降落成功 | 通过 | 73.827 / 287 | 1 / 0 | 2.921591 |
| 雷达延迟 | 3 | failed，独立降落成功 | 通过 | 73.919 / 287 | 1 / 0 | 3.067093 |
| 非法雷达值 | 3 | failed，独立降落成功 | 通过 | 64.761 / 252 | 1 / 0 | 3.092547 |
| VINS 实际重置 | 3 秒请求参数，实际发布 restart | succeeded | 通过 | 94.486 / 356 | 2 / 0 | 1.967870 |
| seed 17 | 无故障激励 | succeeded | 通过 | 524.004 / 1842 | 31 / 0 | 1.544161297943071 |
| seed 73 | 无故障激励 | succeeded | 通过 | 500.877 / 1782 | 30 / 0 | 1.6883587381082155 |
| seed 211 | 无故障激励 | succeeded | 通过 | 442.096 / 1589 | 24 / 0 | 1.7417774154989925 |
| 完全封路 | 固定连续高墙 | failed，独立降落成功 | 通过 | 140.400 / 529 | 4 / 0 | 2.293073 |

临时 2 秒的副本确实修改了 7 次真实扫描输入并触发正常地图暂停，停止记录位移 0.269559 米，持续低速确认用了 2.729532 秒。但实际停稳处理约 8 秒，返回在进入净空等待前已被新自由射线消除，所以实际四步任务成功，而 `actual_blocked_start_wait_branch` 覆盖检查失败。这个失败保留，不能事后改为通过。

持续近障碍实际进入等待后因不变的 8 秒期限失败；雷达中断、延迟和非法值保持传感器失败分类。四例均保留 failed 原任务并通过独立降落。暂停后的记录停止位移分别为持续近障碍 0.273558、中断 0.345891、延迟 0.291427、非法值 0.328136 米，均小于 0.5 米。完全封路停止于 `no reachable observed frontier; hold instead of crossing unknown space`，没有达到目标或返航完成；其在停止位置独立降落，不能说成返航成功。

三项雷达异常的水平停止观测包含独立 AUTO.LAND 阶段，不能据此声称先在 OFFBOARD 完全停稳后才降落。持续近障碍则有实际停稳、等待 8 秒超时、再独立降落的分段证据。

[VIO 重置独立分析](../logs/reliability/suite_navigation_02/vio_reset_01/independent_fault_analysis.json) 记录了真正 `/vins_restart` 与健康状态恢复，期间 `used_for_flight_control=false`。该例最大记录间隔 2.575890 秒仍原样报告，净空连接线评价不能填补未采样曲线。

三个 seed 的图只绘制实际机体轨迹，目标标记由 goto 完成事件附近的 armed、fresh world pose 样本和 ENU 目标误差确认，不画隐藏 witness 作为飞行路线：

| 场景 | PNG | SVG | 原始绘图数据 |
|---|---|---|---|
| seed 17 | [轨迹图](../logs/reliability/seed_17_trajectory.png) | [SVG](../logs/reliability/seed_17_trajectory.svg) | [JSON](../logs/reliability/seed_17_trajectory.json) |
| seed 73 | [轨迹图](../logs/reliability/seed_73_trajectory.png) | [SVG](../logs/reliability/seed_73_trajectory.svg) | [JSON](../logs/reliability/seed_73_trajectory.json) |
| seed 211 | [轨迹图](../logs/reliability/seed_211_trajectory.png) | [SVG](../logs/reliability/seed_211_trajectory.svg) | [JSON](../logs/reliability/seed_211_trajectory.json) |

### 6.4 套件 03：新的 10 秒激励与健康状态测试

[suite_navigation_03/summary.json](../logs/reliability/suite_navigation_03/summary.json) 是独立计划的两个用例，**2 次合同通过、2 次原任务完成**，正常恢复默认世界。它使用新的 10 秒激励覆盖等待分支，不是覆盖旧记录的自动失败重试。

| 实际用例 | 原 mission / 合同 | 秒 / 样本 | 规划 / 等待 | 记录最小净空（米） |
|---|---|---|---|---:|
| 临时近障碍 10 秒 | succeeded / 通过 | 137.719 / 525 | 5 / 1 | 1.9901326092399345 |
| VIO 跟踪健康失效 3 秒 | succeeded / 通过 | 117.336 / 451 | 2 / 0 | 1.985726058795175 |

[临时近障碍独立分析](../logs/reliability/suite_navigation_03/temporary_obstacle_01/independent_fault_analysis.json) 把 35 次实际修改的 LaserScan 副本与暂停、等待中被占用的格子，按源 stamp、射线和固定端点对应。暂停后 8.071883 秒进入真正等待；激励在约 10.013303 秒后自动结束，地图继续接收原始新射线，累计扫描从 65 增到 77，两个近障碍格子变为 free。等待 **5.236351 秒**后以 `observed_clearance` 结束，持续净空条件满 1 秒，再于约 0.009256 秒后生成新计划，随后四步任务成功。全程没有直接清分数、降低净空或复用隐藏路线。记录停止位移 0.268187 米。

激励结束后的另外两次地图暂停来自不同时间戳的原始近距离返回，独立分析明确不归因于测试副本；原证据未确认其物理原因，不能称为已经解释的自身回波。

[VIO 跟踪健康独立分析](../logs/reliability/suite_navigation_03/vio_tracking_loss_01/independent_fault_analysis.json) 确认 11 次真实 tracking 回调被测试适配器强制为 false，11 次真实 odometry 回调原值继续。451 个样本中 12 个 VIO unready，飞控 451 个均 ready；实际激活 3.023618 秒，结束后首次恢复样本约 0.257895 秒。该用例修改的是执行器的健康输入，没有把估计器关掉、改写里程计或启用 VIO 飞控，`used_for_flight_control` 始终 false。

三份 suite 合计 **13 个已记录实例、11 个合同通过、7 个原任务完成、12 个实际起飞**。真实 Qwen 两份基线单独统计，不与直接执行器或离线 graph 的通过数合并。所有历史报告与补记在 [只读汇总](../logs/reliability/validation.json) 中有路径及 SHA256，没有把结果不同的条件混成一个成功实验。

## 7. 使用方法

以下命令均在 `/home/dut/px4_agent` 下执行。每次实际飞测使用新的输出目录；同一路径的已有报告不能覆盖。

离线检查无需仿真或模型密钥：

```bash
python3 scene/generate_reliability_worlds.py
python3 -m unittest discover -s tests -p test_reliability_worlds.py
python3 -m unittest discover -s tests -p test_navigation_reliability.py
/home/dut/ros_agent/mcp-client-langchain/.venv/bin/python scripts/test_agent_reliability.py
```

查看实际执行器用例配置，不启动飞行或发出请求：

```bash
python3 scripts/test_navigation_reliability.py \
  --case temporary_obstacle \
  --layout scene/obstacle_layout.json \
  --output logs/reliability/preview \
  --dry-run
```

启动测试专用仿真时显式开启 GUI、0.35 仿真速度和测试开关。切换世界前用系统 manager 正常停止上一套仿真；每例必须从落地上锁、飞控/雷达/双目/VIO 就绪开始。

```bash
GUI=true PX4_SIM_SPEED_FACTOR=0.35 UAV_RELIABILITY_TESTING=true \
  bash scripts/start_system.sh \
  --world-file scene/reliability/reliability_seed_17.world
```

在另一终端通过真实执行器运行完整场景，模型不参与：

```bash
python3 scripts/test_navigation_reliability.py \
  --case success \
  --layout scene/reliability/reliability_seed_17_layout.json \
  --output logs/reliability/manual-seed17
```

局部故障用例可选 `lidar_drop`、`lidar_delay`、`lidar_invalid`、`temporary_obstacle`、`persistent_obstacle`、`vio_tracking_loss` 或 `vio_reset`。`blocked_route` 应与 `reliability_blocked_17.world` 及其匹配 layout 配合。每例会保留状态样本、注入证据、mission、飞行事件、轨迹评价和最终验收。

真实 Qwen 自然语言基线另行通过网页 Agent，需已有模型配置可用：

```bash
python3 scripts/test_avoidance_scene.py \
  --task autonomous_goal_round_trip \
  --output logs/reliability/manual-qwen-baseline
```

统一入口 [run_reliability_suite.py](../scripts/run_reliability_suite.py) 已提供以下 CLI。先预览本地注册、命令与世界哈希；`--dry-run` 不创建目录、不发请求、不启动进程或飞行：

```bash
python3 scripts/run_reliability_suite.py \
  --dry-run --only-navigation \
  --case seed_17 --case temporary_obstacle --case persistent_obstacle \
  --repeat 1 \
  --output logs/reliability/suite_preview
```

`--case` 可重复指定不同 case；同一个 case 的显式重复用 `--repeat N`，不允许将重复 case ID 当作隐式重试。`N` 必须为 1–10，每个失败实例都保持独立证据；这不启用自动失败重试。`--only-navigation` 排除真实模型基线 `qwen_baseline`。选择完整环境及局部故障测试：

```bash
python3 scripts/run_reliability_suite.py \
  --only-navigation \
  --case seed_17 --case seed_73 --case seed_211 --case blocked_route \
  --case lidar_drop --case lidar_delay --case lidar_invalid \
  --case temporary_obstacle --case persistent_obstacle --case vio_reset \
  --repeat 1 \
  --output logs/reliability/suite_navigation_manual
```

可另选 `qwen_baseline` 和 `vio_tracking_loss`。只有 `qwen_baseline` 经过实际 Qwen；其余 suite case 使用真实执行器，模型调用标记为 false。整个 suite 不包含离线 graph 结果，离线图仍通过 `test_agent_reliability.py` 单独运行。

每次 case 使用新 manager，显式开启 Gazebo GUI、0.35 速度和测试模式。转换前必须观测本地仿真已落地上锁、没有运行 mission 或 active operator command，并验证 manager 的 PID、启动时间、脚本和用户身份。只停止该调用拥有的整套 manager，不单独杀 Gazebo 或其他节点。场景世界和评价 layout 的 SHA 必须一致。

套件结束后，在重新确认地面与控制权的前提下恢复默认 `obstacle_yard.world`，GUI 保持开启，测试 hook 关闭。如果无法确认安全状态或 manager 归属，套件中止后保留当前系统用于恢复，不强制拆除。新的 suite 输出目录必须不存在。

`summary.json` 分别记录 `requested_mission_completed`、`expected_safety_contract_pass` 与 `safety_confirmed`。例如封路正确停止时，预期安全合同可以通过，但原请求任务完成应为 false。统计保留这一区分，并单独列出启动失败、评价器错误、测量缺失与套件未执行项。

只读汇总入口 [summarize_reliability.py](../scripts/summarize_reliability.py) 不发起模型或飞行请求，只读取已有日志并核对来源。默认或 `--dry-run` 只打印简表；输出新文件使用 `--output`，已有文件拒绝覆盖：

```bash
python3 scripts/summarize_reliability.py \
  --logs-root logs/reliability --dry-run

python3 scripts/summarize_reliability.py \
  --logs-root logs/reliability \
  --output logs/reliability/validation_recomputed.json
```

## 8. 最终环境状态

[final_verification.json](../logs/reliability/final_verification.json) 保存最终只读检查：默认 `obstacle_yard.world` 哈希匹配，飞控 ready、`armed=false`、`landed_state=1`，没有 active command，测试 hook 关闭。左右相机均为 640×480 且可用，VIO ready 且 `used_for_flight_control=false`，雷达可用且每帧 360 个样本。

网页 [只读 UI 检查](../logs/reliability/ui/navigation_smoke.json) 通过，机载图像与航迹监测保留。原生 [Gazebo 图形窗口截图](../logs/reliability/gazebo_overview.png) 和 [截图元数据](../logs/reliability/gazebo_capture.log) 使用已验证的单个 Gazebo 窗口 XComposite pixmap，`desktop_read=false`。套件结束后默认仿真与 GUI 保持运行。

本阶段已经覆盖这些固定场景、故障激励和实际记录，保留了提供商异常与全部失败。它没有验证 VINS 飞控闭环、三维或动态障碍导航，也没有提供统计泛化或连续时间净空保证；后续仍按原优先级推进 VIO 定位闭环。
