# 自主搜索终止与飞行收尾

## 问题与本轮边界

林地任务 `78a15175242d475c` 已保存 6 份观察、实际执行 5 次 NBV 换位，但两次目标复核都未满足确认判据。模型在一次提前返航纠正后改用 `land_drone`，随后又请求观察。旧代码允许直接降落和单步 `land` 绕过搜索检查，且未关闭搜索活动；`reason` 没有发往执行器，默认的“操作员要求”不能证明请求来源。

本轮修复 AdaptiveSearch 的生命周期及模型入口，不改变目标确认判据、NBV 评分、坐标身份、票据验证或 PX4/VIO。旧的区域网格搜索流程不是本轮阶段工具过滤的对象，其回归仍需通过。

## 可信来源与状态转换

| 来源 | 授权依据 | 行为 |
| --- | --- | --- |
| `normal_completion` | 程序判定 target_found、budget_exhausted、coverage_exhausted 或 no_reachable_view，且没有待完成复核 | 关闭搜索活动、清除待执行候选，按 end_behavior 执行既有返航降落；hold 不新增飞行 |
| `operator` | 桌面/HTTP 的真实中止入口 | 立即关闭搜索活动，直接下发停止或降落，不等待模型或确认 |
| `protocol_error` | 模型请求未通过搜索阶段/飞行计划门禁，且不可纠正或额度已用完 | 任务失败，由现有独立处理器对本任务拥有的飞行安全收尾 |
| `safety_fault` | 任务执行、坐标、地图、视觉等故障 | 禁止继续搜索，沿用独立安全收尾与所有权检查 |

搜索结束与飞行结束分别记录。`activity_closed` 禁止后续观察/移动；`termination.flight_status=pending` 不声称已经降落。只有对应降落任务 succeeded、`armed=false`、`landed_state=1` 才标为 `grounded_disarmed`。操作员异步降落通过实际状态刷新核对任务 ID；独立收尾等待真实执行结果。未收到核验数据时保持待确认。

模型的理由是请求说明，不是权限声明。`model_end_request` 保存工具、动作及限长理由，来源固定为 model。`termination_events` 保存程序判定的结束来源及结果。`/land` 接收 reason 并写入实际任务描述，但不接收模型可设置的权限/来源字段；默认描述改为中性 API 请求。真实操作员与独立失败收尾由各自入口设置说明。

## 模型入口及一次纠正

- 活动搜索仅向模型展示当前 next_tool 和状态查询；需要换位/采图时展示完整的 observe_target_search_view，不再同时展示分步入口。旧分步调用仅在匹配的阶段保留兼容接纳，仍经过所有权、坐标和执行器验证。拿到未报告照片后只提供报告与状态工具；正常终态由程序完成收尾后不再提供动作工具。
- 隐藏工具不是安全边界。工具中间件再次检查旧调用、阶段不符调用；模型 `land_drone`、单步 land、return_home、return_home+land、stop/hold 均进入搜索检查。完成后重复的等价降落调用仅返回已核验的结果，不提交新飞行；晚到的采图/移动调用被拒绝。
- 正常收尾核验后直接由程序生成最终响应，不再请求一次模型总结。真实实例 `20261103-215933-hnKhwK` 证明仅隐藏工具不够：目标已 confirmed、实际返航落地后，模型仍输出无有效动作的 execute_mission，入口虽正确拒绝，却导致最终协议失败。此失败记录保留，后续用独立实例验证修复，不改写旧结果。
- 搜索仍在进行时，只有当前任务拥有会话及上一成功飞行、坐标/地图有效、未中断且无已知故障时，允许一次“未提交”的纠正，返回状态机的 next_tool。所有等价入口共享 `completion_corrections`，不能换工具再获得额度。
- 被拒绝请求不调用 /mission 或 /land、不消耗票据、不重建地图、不扩大预算。第二次违规立即结束图执行，交给独立收尾，不再让模型循环调用降落。
- 独立安全监控和真实操作员入口不走模型纠正门禁。审计异常不得阻止真实中止；故障收尾保留 expected_mission_id，不能取消新任务的飞行。

## 验证

新增 `tests/test_search_termination.py` 覆盖等价入口共享额度、伪称操作员、一次纠正后继续换位、程序正常收尾及幂等、落地后旧观察拒绝、真实操作员中止、不完整/错任务落地证据拒绝、reason 透传。原有坐标失效、初始化失败、错误归属、执行器运行中、伪造未执行结果及安全收尾回归保留。

补充回归覆盖正常终态不再调用模型（包括模型原本将返回空任务的反例）；操作员降落请求受理期间 Agent 已结束时，受理的降落 ID 仍归属原中止任务，不能依赖仍然存在的 active_id。

2026-11-04 增加模型实际调用的 `mission_arguments` 结构审计：区分 steps 缺失、空列表、标量及列表项类型，不保存任意描述或凭据。无效结构不会额外获得恢复额度。阶段工具测试从搜索会话建立后检查；初始化之前的通用工具列表不属于活动搜索过滤范围。

完整测试命令、最新实际结果及真实 Gazebo 证据见 `PROJECT_HANDOFF.md` 最新条目。脚本模型和本地桥接夹具仅为受控测试，不能当作真实模型或飞行验证。

联合回归命令（需允许本机回环 HTTP 服务）：

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 PYTHONFAULTHANDLER=1 \
PYTHONPATH=.:tests:scripts .venv/bin/python -m unittest -v \
  test_search_diagnostics test_search_termination test_desktop_search_data test_desktop_workbench test_desktop \
  test_web_server test_gazebo_seed_evidence test_target_confirmation \
  test_nbv_search_integration test_single_vlm test_ev_heading_guard test_bridge_camera \
  test_analyze_heading_ulog test_heading_reference_probe test_autonomous_search \
  test_search_graph_integration test_takeoff_heading_alignment test_ekf_landing_evidence \
  test_semantic_map test_viewpoint_planner test_bridge_static_navigation test_yaw_control \
  test_forest_environment test_alternate_environment test_static_navigation \
  test_recovery_ownership test_agent_tool_schema test_autopilot_landing test_mission
```
