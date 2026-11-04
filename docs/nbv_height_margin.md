# NBV 观察高度边界修复

## 2026-11-04：统一已授权的20米上限

用户已将硬高度许可从10 m改为20 m，自动观察点保留0.5 m裕量，即1.5～19.5 m。巡查发现 `ViewpointPlanner.evaluate()` 的完整路径校验仍残留硬编码10 m；新增受控反例复现了19.5 m合法候选路径被拒绝。本次改为引用共享 `mission.MAX_ALTITUDE`，20.5 m中间路径点仍被拒绝；生成候选、碰撞和12 m搜索半径均未放宽。5项高度专项通过，修改前失败和修改后通过日志分别为 `logs/setup/nbv-scene-precheck-20261104/altitude-before.log`、`altitude-after.log`。不据此声明高空NBV实飞通过；下文10米描述属于旧版本历史。

## 真实故障

林地任务 ca2804b4cb434f61 最后一个票据任务 f27a870fc5e0：目标高度 1.0 m，goto 到达事件高度 0.9117 m，rotate 到达/图像姿态高度 0.8282 m。移动的 0.2 m 到达容差允许第一个残差；原旋转初始化把实际位置重新设成目标，使后续控制相对于残差位置保持。观察仍严格要求本地 ENU 1～10 m，最终报告被拒绝。不同任务误差方向不同，不能靠固定补偿修复。

## 修改

- `viewpoint_planner.py`：自动生成的观察候选高度裁剪到 1.5～9.5 m，包含当前高度候选；高度层去重。边界附近不再产生没有高度调整的原地旋转候选。0.5 m 规划裕量大于旋转的 0.35 m 位置偏离上限。硬飞行/观察许可仍为 1～10 m，实际观测越界仍拒绝。
- `bridge.py::rotate_heading`：仅票据授权的 goto+rotate 使用该任务已验证的 goto 目标作为保持点；要求有候选 ID、处于第二步、实际位置距离目标不超过 0.35 m，当前位置至目标线段通过静态碰撞检查。旋转仍保持原任务身份及原有新鲜姿态/时钟/重置/中断检测。普通独立旋转仍使用当前位置。
- 不重新规划未知航点，不修改 NBV 票据、定位配置或安全降落。没有扩大等待或到达容差。

## 受控验证

首轮 72 项中新增测试夹具缺少 mission 字典导致 1 项错误；补全夹具后联合回归 293 项通过，失败/跳过 0。失败日志 `/tmp/px4-altitude-margin-tests.log` 与最终 `/tmp/px4-altitude-margin-full.log` 保留。

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 PYTHONPATH=.:tests:scripts .venv/bin/python -m unittest test_desktop_search_data test_desktop_workbench test_desktop test_web_server test_gazebo_seed_evidence test_target_confirmation test_nbv_search_integration test_single_vlm test_ev_heading_guard test_bridge_camera test_analyze_heading_ulog test_heading_reference_probe test_autonomous_search test_search_graph_integration test_takeoff_heading_alignment test_ekf_landing_evidence test_semantic_map test_viewpoint_planner test_bridge_static_navigation test_yaw_control test_forest_environment test_alternate_environment test_static_navigation
```

实际复测使用同一 forest_rescue 世界、种子 17、橙色帐篷指令、几何 NBV/确定性/离散航向/多视角配置。结果见 PROJECT_HANDOFF.md；原失败不删除。规划高度裕量是工程控制余量，不是概率保证，仍须检查真实位置和最终新帧。

## 真实验证

真实复测实例 `20261103-211352-oweGnt`，任务 `78a15175242d475c`：6 份照片/有效观察、5 次实际 NBV 换位、22 次模型调用。采图高度依次 3.033、1.649、1.702、3.669、5.720、3.845 m，全部满足原 1～10 m 门槛；4 次旋转后的左右帧均晚于实际完成时刻（详见 rotation-frame-checks.json）。高度边界修复已通过本轮实际验证。两轮目标复核均 unconfirmed（none / bearing_intersection），未越级确认。本次自然触发一次 unconfirmed_search 提前返航拒绝及纠正，后续实际换位/采图继续成功；随后模型直接 land_drone 两次，再要求 observe_target_search_view，被已落地/地图未注册门禁拒绝。最终任务 failed、实际落地上锁；总体 NBV/确认审计未通过，不能宣称完整闭环成功。未执行候选及所有失败证据保留。下一步应处理成功降落后搜索会话的终态同步，保持降落工具独立可用，不将它改成可阻止紧急降落的普通计划。
