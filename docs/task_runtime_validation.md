# 完整任务重构：验收与交接记录

2026-11-06。架构、实现和兼容入口见 [任务运行时架构](task_runtime_architecture.md)。用户参考文档的A～E代码已接入，下面严格区分代码、受控测试与真实任务结果。

**更新入口**：后续同日staged/legacy车前原句、806项实际单进程回归、原始候选交接修复及SIGSEGV core诊断见 [最新专项验收](front_landing_runtime_validation.md)。本页下方763/26/86项分组和两次实飞为前一轮冻结记录。最新staged实飞实际已落在货车前方；软件因超时安全降落仍记录partial，正常目标相对完成链未自动验收。原生故障尚未根治；不能把下方历史“最终”当成当前完成声明。

## 当前状态

| 验收项 | 结果 |
| --- | --- |
| A：父计划、技能交接、统一结果/收尾 | 已实现、受控通过；一次真实搜索→报告→返航通过 |
| B：同一权威地图、真实CPU匹配、别名、分级候选、恢复和三层推理 | 已实现、原图回放/受控通过；真实GLOBAL请求失败暴露的接口问题已修，修复尚未重飞 |
| C：关系/路线、位置/yaw、原A*/票据、实测到位和新图 | 已实现、受控通过；新终点执行尚未实飞验收 |
| D：悬停后复核、静态平地车前落区/下降、普通PhotoTask迁移 | 已实现、正向受控图和实际Controller方法通过；真实车前任务未进入终点/降落阶段 |
| E：现有桌面父阶段/目标/信息需求/终点展示、回归及有限现场验证 | 已实现；两次实际任务和现场截图保留；没有录像 |

**不能将车前降落写成已实飞成功，也不能把原地安全降落当作用户车前目标完成。** 代码中的能力仍取决于实际视觉、可靠米制定位、对象前向、有效坐标和无动态对象的平地支持证据。

## 最终受控回归

最终按原生资源及解释器隔离执行，所有相关用例均运行，不通过跳过来变绿：

| 进程组 | 通过 | 失败 / 错误 / 跳过 | 时间 | 本机日志 |
| --- | ---: | --- | ---: | --- |
| Agent/核心，Python3.11，未载入QtWidgets | 763 | 0 / 0 / 0 | 144.324s | isolated-agent-release.log |
| Qt桌面/工作台，主线程持有QApplication | 26 | 0 / 0 / 0 | 0.847s | isolated-qt.log |
| 实际执行器Python3.8、ROS/OpenCV/报文 | 86 | 0 / 0 / 0 | 1.184s | isolated-native.log |

原相关800项由Agent763、Qt26、原生控制器11项覆盖；原生86还包含重复的平地几何检查及额外双目/EV报文用例，不将三组数量简单相加当成互不重复的新测试。原生/Qt组件在最后Agent修正之后未变化。最终源码哈希：`logs/setup/task-runtime-release-20261106/release-v3-source-sha256.json`。

历史基线603、A626、C652，以及修复前675/795通过均属于各自冻结版本。D和最终汇总中单进程混跑出现过原生segfault（退出139）；**根因尚未证实，不能声称已彻底修复，也不能把这些轮次计为通过**。曾修正新增Qt测试对象回收并通过35项顺序组合，但后一次仍复现，故最终采用进程隔离。产品仍是现有桌面/Agent架构，并未新增独立服务。两次实际桌面任务均没有出现该崩溃，这不能证明长期运行没有此风险。

其他失败轮次保留：扩展回归164项中的2失败/8错误来自旧10m/12m断言和缺prompt夹具，1项venv缺pymavlink后用系统解释器补跑；修正夹具后补充118项通过、原生46项通过。协议来源首轮26项有1失败，修正异常清理前的类型保存后通过；最终分组覆盖此修复。所有日志在上述本机忽略目录，未删除失败或改写原始结果。

### 可复现命令

从项目根目录、没有仿真任务的终端执行。主组：

```bash
UAV_TASK_RUNTIME=legacy PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}" PYTHONPATH=.:tests:scripts \
  .venv/bin/python -X faulthandler -m unittest \
  test_task_runtime test_task_association test_task_reasoning test_goal_pose test_task_goal_graph \
  test_task_landing_graph test_task_route test_task_photography test_flat_landing test_stereo_reports \
  test_spacious_search test_target_observation test_task_contract test_landmark_association test_semantic_map \
  test_semantic_map_runtime test_coverage_search test_photo_request_evidence test_photo_framing test_photo_task \
  test_photo_navigation test_single_vlm test_agent_reliability test_agent_tool_schema test_task_memory \
  test_web_server test_search_termination test_nbv_search_integration test_target_confirmation test_search_graph_integration \
  test_autonomous_search test_desktop_search_data test_bridge_static_navigation test_recovery_ownership test_bridge_camera \
  test_ev_heading_guard test_static_navigation test_scene_config test_viewpoint_planner test_viewpoint_authority \
  test_search_diagnostics test_altitude_limit test_yaw_control test_mission test_analyze_heading_ulog \
  test_heading_reference_probe test_landing_velocity_analysis test_external_world_preview test_forest_environment test_orchard_environment \
  test_ev_launch test_system_shutdown test_profile_recovery test_alternate_environment test_gazebo_seed_evidence \
  test_takeoff_heading_alignment test_ev_fault_safety test_ev_health test_ev_preflight test_production_telemetry \
  test_static_search_permission test_static_search_volume test_search_tools test_nbv_evidence_audit test_semantic_evidence_audit \
  test_confirmation_retry_audit test_nbv_audit test_stereo_evidence_archive
```

Qt独立组（启用实际原生控件测试、13号字体及1220×900/960×720布局）：

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 UAV_TASK_RUNTIME=legacy \
  PYTHONPATH=.:tests:scripts .venv/bin/python -X faulthandler - <<'QT'
import gc, unittest
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QCoreApplication, QEvent
app = QApplication([])
app.setQuitOnLastWindowClosed(False)
suite = unittest.defaultTestLoader.loadTestsFromNames([
    'test_task_desktop', 'test_desktop_workbench', 'test_desktop'])
result = unittest.TextTestRunner().run(suite)
QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
app.processEvents()
gc.collect()
raise SystemExit(0 if result.wasSuccessful() else 1)
QT
```

原生组：

```bash
PYTHONPATH=.:tests:scripts /usr/bin/python3 -X faulthandler -m unittest \
  test_task_runtime_bridge test_goal_executor test_goal_landing_executor \
  test_flat_landing test_stereo_localization test_stereo_positive test_ev_adapter
```

HTTP测试需本机回环权限；系统组需要本机现有ROS/OpenCV/pymavlink。缺依赖或权限失败要如实报告。全部命令/分组也保存在本机 `regression-final-command.txt`、`isolated-groups.json`，原单进程命令只作历史复现，不能称其最新一轮通过。

## 真实任务1：父搜索、报告和返航通过

- 原句：找到前方的白色货车，报告周围环境，然后返航降落。
- 实例/任务：`20261106-003935-jdqTYi` / `7d07fed354ed4eda`。
- 场景：`courtyard_spacious.world`，SHA256 `b624fb8ceabc2d56b8979b3a792678271bbc5165f9bcfb5ce627079b1d324f6a`；实际gzserver命令核验种子17与lockstep。
- 配置：staged、几何NBV、确定性选择、离散航向、多视角；现有qwen3.8-flash，未换模型。起飞前VIO实际用于飞控、静态导航、地面上锁及文字/工具/实际相机3次接口预检通过。
- 6份真实有效照片/报告/地图、5次实际票据换位、13次任务模型请求；3次未确认后1次confirmed，依据single_depth_projection。bearing候选`bearing_007`经真实复核续接米制观测，不补造深度。
- 本次自然出现report_scene.targets字符串类型错误：第一次被schema拦截，在同一capture_id上纠正一次后继续；原始参数保存于实例captures的nbv-diagnostics，模型输出未修改。不是提前返航纠正的真实覆盖。
- 两次实际旋转完成169.920/278.684仿真秒，左右后续帧分别170.048/278.748秒，均在完成之后且原图哈希通过。
- 正常返航降落`1df657ba150e`，实际armed=false、landed_state=1、VIO healthy=true，距home水平约0.0527m。
- TaskResult为completed/satisfied，原技能handoff→报告→父收尾三个阶段完成；无外部回收冒充成功。该运行发生在随后规划版本归档/回放器适配和最终报告摘要改动之前，不能当作这些飞行后修改的新实飞。

证据：`logs/setup/task-runtime-real-20261106/search-summary.json`、`search-state-samples.jsonl`、`search-final-state.json`、`search-report-ui/result.json`。现场11帧来自正在运行的Qt窗口，包含`04-live-camera.png`、`05-nbv-preview.png`、`08-multiview-review.png`和`10-terminal-grounded.png`。

原NBV审计首次因旧回放器没有启用新CPU像素关联而false；原失败目录`search-nbv-audit/`保留。适配真实原图与后端后的只读审计`search-nbv-audit-replay-fixed/`全检查通过，确认审计`search-confirmation-audit/`通过；没有新增模型调用或重飞来修改历史。

## 真实任务2：车前原句失败，安全落地

- 原句：找到白色货车后降落在货车前方。
- 实例/任务：`20261106-005649-A2ktCH` / `9ca3074eb5c142c4`；场景、种子、模型和导航模式同上，独立实例。
- 地面VIO/静态导航/目标位姿与落区能力接口核验通过，文字与当前相机2次预检通过。
- 3份有效报告/地图、2次实际票据换位、6次模型请求。3图均没有形成搜索目标引用；这是当前帧证据缺失，不等于场景中不存在货车。
- GLOBAL_REASONING时模型选择需要保留目标的恢复类策略，程序因无实际目标引用抛出异常，错误为`Cannot recover a target without an actual observation reference`。旧记录没有完整保留重规划原始参数，不能断言具体是两个恢复枚举中的哪一个。
- 未进入relative_land取证阶段，0次新GoalPose执行、0次目标相对降落，未发生多视角确认或落区许可。
- 独立安全降落`7ac167cf5a0f`，实际ground/disarm、VIO健康，离home约8.05m。它是原地失败回收，不是返航或车前任务成功。
- 两次旋转190.580/216.048秒，对应左右新帧190.652/216.400秒。
- 原TaskResult为failed/unmet，历史source是通用safety_fault；未修改原记录。修复后新的协议错误会按异常类型记录protocol_error。

证据：`front-summary.json`、`front-state-samples.jsonl`、`front-final-state.json`、`front-land-ui/`，均在同一本机忽略目录。额外X11抓屏`front-stage-screens/`实际捕获了GNOME锁屏，已标为无效并保留，不计工作台验收。有效11/10张图由正在运行的Qt窗口QWidget.grab直接取得，非离线重绘；它们证明实际窗口内容，不能证明锁屏时工作台在物理屏幕上可见。本轮未解除或绕过用户锁屏。NBV整体审计保持false：任务失败且最后一批选中预览没有执行；两次已受理换位均有完成及新帧，在线图像/地图重放一致。不能为得到绿色审计删掉未执行候选或失败状态。

### 飞行后最小修复与剩余验证

- 当前状态给模型明确available_replan_strategies，无目标引用时只允许continue_search。
- 第一次不可用策略在工具执行前返回typed not_executed、mission_submitted=false，真实状态机给出next_tool；共享父任务唯一纠正额度，图/owner/预算/plan不重置。第二次违规仍失败并独立落地。
- 捕获重规划原参数/可用策略到本机诊断；协议来源按TaskProtocolError类型判定，坐标/执行故障不变，不用异常文案匹配。
- 受控真实Agent图正反例通过；**未再重飞**，没有声称此修复已真实恢复车前任务。
- 同时补齐最终回复中的已保存报告摘要、目标color属性进入实际query、后续接近/关系/路线的观察预留；新候选仍使用原票据和所有安全门禁。

两实例均按stop_system身份退出，PX4实际参数读回恢复成功；VIO仅本次output_path/pose_graph_save_path改动后恢复为各自启动前原字节。最终无本次Gazebo/PX4/VIO/桌面残留。未提交.env、原图、日志、私有网络/VIO路径或自动commit/push。

## 下一次维护者应做什么

先阅读PROJECT_HANDOFF当前条目，核查Git/进程，使用显式staged新实例；默认legacy保留。当前最可靠的正式展示原句是任务1，启动方式见架构文档。下一次有限实测应先验证无目标时的真实重规划能继续，再观察candidate_acquired交接后的车头取证、GoalPose悬停与平地降落；每一层保留真实原图/位姿/票据/到位证据。车头、前后ROI、米制或落区不足时保存未完成项和安全回收，不能强行确认。

真实丢失恢复、车前悬停/降落和普通摄影新父技能尚无本轮成功实飞证据；路线已受控验证，未额外启动路线飞行。整园单帧范围未知继续作为明确限制。混合测试的原生崩溃保留待诊断，不在本轮扩展为新算法或无限飞行任务。
