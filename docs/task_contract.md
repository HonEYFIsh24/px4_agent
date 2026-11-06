# 任务接纳、目标完成与程序收尾

2026-11-05。本轮只修改任务层并分析既有照片；没有启动 Gazebo、PX4、Agent 实飞或真实模型请求，没有修改 NBV 评分、候选生成、地图或确认判据。

## 触发问题

原句“找到白色货车后降落在货车前方。”只执行了搜索部分。模型将 `begin_target_search` 的目标缩为“白色货车”，选择 `end_behavior=hold`；8 次观察结束后，应用状态仍为 succeeded，虽然结构化结果为 budget_exhausted / target_confirmed=false。随后测试执行者独立返航落地，不能算模型完成指定落点任务。

## 本轮行为

1. `task_contract.py` 保留完整 `original_request`，检查已知不支持的目标相对降落/悬停要求。原句现在在接纳阶段返回 failed 和具体原因，不调用模型、不采图、不飞行，也不自动改为普通搜索。接纳结果与错误原因作为任务记录保留。真实操作员停止/降落入口不受该检查限制。
2. `WebApplication.submit()`、single_vlm 的模型/工具入口与 `AdaptiveSearch.begin()` 均检查能力约束。搜索工具从当前用户消息传入完整要求；桌面路径优先从受任务所有权保护的原始命令获取，模型不能通过缩写 query 或改写 end_behavior 删除要求。
3. 新增 `task_outcome`，区分 `search_process_ended`、`user_goal_completed`、确认等级、飞行状态和收尾来源。预算耗尽、无可达视点、没有实际搜索证据、没有满足确认规则或收尾未核实均不能显示整项 succeeded。外部安全回收不能把未完成目标改成已完成。
4. 搜索结束方式由原要求和程序策略决定：找到后明确要求悬停时保留该结束方式，其余正常完成返航降落；搜索未完成时固定在导航健康且仍拥有当前已完成飞行的条件下返航降落。若坐标、任务归属或执行器状态失效，不继续返航，沿用既有独立安全处置。处置不重启搜索、不扩预算。
5. `end_behavior` 参数暂保留作旧调用兼容，但不再有权覆盖程序策略。搜索的失败收尾标记 `purpose=unmet_goal_recovery`，与 `goal_completion`、操作员中断、独立安全回收分开。返航提交前在操作员控制锁内再次检查任务归属和坐标身份。

这是对**已知目标相对结束动作**的有界语言规则，不是通用自然语言任务完备性证明，也未增加车头姿态识别或降落区域定位。规则保留原文和匹配范围，涵盖原句、目标旁/前/顶落点、相对悬停、中英文常见表达，并测试否定句与“寻找前方目标后返航”的区别。更复杂或新类别的语言要求仍需增加明确的能力契约，不能声称任意指令都可完整理解。

## 三轮复核离线诊断

原始实例 `20261105-092828-SMIrkx`，指令 `111e16127d714eb6`。逐张核验左右原图 SHA256，查看真实图像，用原 `pair_geometry()` 重算关联；使用执行器的 `/usr/bin/python3` 和未改动的双目实现回放四次目标定位。没有读取场景目标名称/真值坐标，没有新的模型复核。

| 轮次 | 实际图像和原始定位证据 | 关联限制 |
| --- | --- | --- |
| 1：观察 4→5 | 图4货车位于画面右缘、框右边界为1；图5货车一部分被灰墙遮挡。图4左右一致率0；图5鲁棒内点比例约32.34%，低于35% | 裁切框射线区域得到18对支持，只是 clipped_region_hypothesis，两侧无有效深度，不能确认 |
| 2：观察 5→6 | 图6可见更多白色车厢，但仍被灰墙遮住一部分；有效双目像素372/2856≈13.03%，不足35%。模型“完整车体”的描述不符合原图 | 观察基线2.916 m、中心射线夹角13.741°；最近距离0.71695 m，超过0.5 m；交会中点虽投在两框内，中心视线不相容 |
| 3：观察 7→8 | 图7仅有车厢局部，双目有效像素516/1824≈28.29%；图8灰墙挡住货车，没有当前目标框 | 当前图不支持目标、没有几何对应。正确保持未确认，不能否定之前确实看到的局部车体 |

进一步定位：图4的4189个初筛有效视差像素全部映射到右图 x=534～609，其右视差均为无效值 -129；与目标处于当前反向 SGBM 匹配的右边缘无效区域一致。图5的1517个左右一致像素中仅1286个满足鲁棒内点条件，占总ROI3976个像素的32.34%。图6/7并不是左右一致率低（分别约99.73%/98.10%），而是符合条件的像素覆盖比例低。该证据不能唯一归因于“纹理不足”：这些ROI的拉普拉斯方差均通过现有纹理门槛，遮挡、框内混合表面及匹配有效区域仍须区分。

第二轮即使把0.5 m阈值放宽，中心射线交会仍被现有规则列为候选假设，不具备独立对象对应证据；放宽阈值不能补出深度。本轮不修改它。

后续如改确认视点，先做小范围离线验证：避免目标落在双目匹配边界、保留可辨认的目标区域、减少遮挡和框内背景混入，验证能否改善有效深度及真实区域对应。不能只增加基线或把预测可见当成实际看见。

## 验证和证据

新增 `tests/test_task_contract.py` 覆盖未支持任务零模型/零执行请求、完整要求保留、原始结束意图与模型参数冲突、预算结束不等于目标完成、空口成功、真实操作员降落、坐标失效与任务归属变化不允许返航。

本机忽略目录 `logs/setup/task-contract-20261105/` 保存受控测试输出、`review-diagnosis.json`、`stereo-replay.json` 和只读重放脚本。原始实飞结果与图片没有改写。首次回放误用图像时刻作当前时刻，导致一帧姿态时序检查拒绝；已改用存档图像时刻加测量年龄，四次定位拒绝原因全部复现，原失败记录保留。

最终完整工作区回归 **531 项通过，85.592 秒，0 失败/错误/跳过**，见 `regression-legacy-final.log`。前一轮531项中的2项失败来自旧网格结果结构与旧成功断言，已按上述语义修正；原日志 `regression-final.log` 保留。没有跳过失败用例来获得通过结果。

另在 `HEAD` 加仅本次任务层文件的隔离副本验证：`test_task_contract test_search_termination test_nbv_search_integration test_web_server test_search_graph_integration` 共 **60 项通过，54.243 秒，0 失败/错误/跳过**，见 `commit-isolated.log`。未包含上一轮未提交的地标分组或UI修改，确认任务层提交可独立运行。

测试初轮的3个HTTP错误来自沙箱禁止本机socket；允许回环端口后同组通过。另一个能力规则反例发现“找到前方…”中的“到”被当作落点介词，已修正并增加反例。完整命令和最终结果见交接文档最新记录；不把历史实飞称为本轮飞行验收。

增加“没有搜索证据不能成功”的检查后，完整回归暴露旧网格搜索的嵌套结果结构：已兼容读取真实几何搜索结果，不冒充VLM多视角确认；正常找到目标且已落地的成功路径保留，sampling_complete但没有目标则明确记为目标未完成。原网格算法和路线未改。

完整相关回归命令（HTTP用例需要允许本机回环端口；全部使用替身，不飞行）：

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 PYTHONPATH=.:tests:scripts \
  .venv/bin/python -X faulthandler -m unittest \
  test_task_contract test_landmark_association test_semantic_map test_semantic_map_runtime \
  test_coverage_search test_photo_request_evidence test_photo_framing test_photo_task \
  test_photo_navigation test_single_vlm test_agent_reliability test_agent_tool_schema \
  test_task_memory test_web_server test_search_termination test_nbv_search_integration \
  test_target_confirmation test_search_graph_integration test_autonomous_search \
  test_desktop_search_data test_desktop_workbench test_desktop test_bridge_static_navigation \
  test_recovery_ownership test_bridge_camera test_ev_heading_guard test_static_navigation \
  test_scene_config test_viewpoint_planner test_viewpoint_authority test_search_diagnostics \
  test_altitude_limit test_yaw_control test_mission test_analyze_heading_ulog \
  test_heading_reference_probe test_landing_velocity_analysis test_external_world_preview \
  test_forest_environment test_orchard_environment
```

`test_landmark_association` 及相关地图/UI改动属于上一轮尚未提交的工作区内容；本次只对任务层与本文件作独立提交，不混入它们、VIO本机路径或先前拍摄支线文档。完整工作区回归与提交范围应分别理解。

## 后续开发顺序

先完成任务层回归和审查，再基于上述证据小范围改进确认观察。若继续支持车前任务，独立新增车头方向和车前区域定位，先验收带不确定度与路径验证的目标相对悬停，再验收下降通道和实际落地。本轮尚未实现这些能力；不得重复完整原句飞行来代替补齐后半段能力。
