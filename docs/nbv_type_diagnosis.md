# 确认视点准备异常：诊断状态与证据边界

## 当前结论

**维护决定（2026-11-04）**：用户明确要求保留最新成功基线和完整诊断，停止无证据的生产逻辑修改。该异常按已知限制维护，在后续正常运行中捕获真正的失败输入与调用栈；不继续为它专门反复启动仿真，不要求在使用已通过的 Demo 之前先证明历史异常已消失。

2026-11-03 林地任务 `27e40764f51a4ab4` 第三份有效观察后出现
`'float' object is not iterable`。**尚未复现，也没有证据支持某个根因修复。**
后续阶段工具收敛后的林地任务已经完成正常闭环，不能把它当成该旧异常已修复的证明。

现阶段不修改 NBV 评分、确认判据或飞行安全门禁，不以重复实飞碰运气替代定位。

## 旧失败记录确定了什么

- 实例：`20261103-220900-cefuis`，任务：`27e40764f51a4ab4`。
- 原始结果：`logs/setup/search-termination-20261103/demo-final/result.json`。
- 第三张照片：`d379ce6a9acb49f390f9f7cdbd135e76`，采图仿真时间 120.152 s。
- 三份报告和三次空间地图更新实际有效，第三份报告登记了候选 `target_02`。
- 之后没有发布第三份照片对应的 NBV 预测，也未进入双图复核。
- 两条失败 perception 的 `error_type` 实际都是 **RuntimeError**；保存的只是上述错误文本，没有原始 TypeError 栈。
- 搜索活动关闭，独立降落 succeeded，实际已落地上锁，失败记录保留。

| 事件 | 墙钟 Unix 秒 | 证据 |
| --- | --- | --- |
| 第三张照片保存 | 1793715065.411663 | 照片元数据 created_at |
| 第三份视觉报告完成 | 1793715074.6148522 | 有效 perception.finished_at |
| 该观察的搜索进度记录 | 1793715074.6170063 | search_progress |
| 首条搜索终止事件 | 1793715081.4747872 | search_termination |
| 安全落地状态核验 | 1793715093.6020515 | termination.flight_status |

报告完成至首条终止事件间隔约 6.86 秒；首条终止事件只是异常时间的上界，不是异常发生时刻的精确测量。

`BridgeClient.request()` 会把执行器 HTTP 错误中的 error 文本包装成 RuntimeError。
结合报告和地图已保存、错误出现在下一视点准备期间，**执行器查询错误是当前优先假设**。
这不是调用栈证据：原记录没有端点、请求阶段或错误 HTTP 状态，不能宣称已经唯一定位到 `/navigation/viewpoints` 的某个函数。

## 已完成的离线检查

1. **采图位姿回放**：真实照片哈希、报告、已注册几何、目标探针及离线规划均能处理。
   这只覆盖采图时刻的位姿；原执行器使用查询时刻的实时位姿。
2. **故障窗口实际位姿重建**：找到原始 ULog `/home/fxxk/.ros/log/2026-11-03/14_09_05.ulg`，
   SHA256 `12113f8e7211380766740ae4d641eb3a80900b54b4e5666fa289720303e68836`，无 dropout。
   用同架次 EKF 观察器的墙钟/源时间记录包围故障窗口，从 vehicle_local_position 提取 NED，再按 `[y,x,-z]` 转为 ENU。
   采图附近转换结果与归档 MAVROS 位姿接近，但并非完全相同的接收样本。
3. **窗口抽样重算**：40 个观察器对应源样本覆盖 128.940～136.748 仿真秒；选取 11 个样本，
   使用原地图、原目标证据和原锁定航向重算。规划端 Python 3.11、几何端 Python 3.8，与实际分进程解释器一致。
   11 次均 planned、选择 view_015，无类型异常；每次完成 6 次本地几何查询。
4. **完整执行器方法探查**：在 ROS 导入隔离、没有 ROS 节点或网络的进程中，调用真实
   `Controller.evaluate_viewpoints()`；使用原归档元数据、原 EKF/适配器状态形状和上一批任务身份。
   完成 context、visibility、paths、所有权检查及票据签发，未复现。
   该探查冻结了离线接收时钟，并使用新构造的批次/修订标识；不是原 HTTP 请求回放，也不授权任何实际飞行。

本轮证据与一次性重建脚本均在忽略目录
`logs/setup/nbv-type-diagnosis-20261104/historical-window/`：
`poses.json`、`window-results.json`、`controller-results.json`、`replay.log`、`controller-replay.log`。
没有启动新的 Gazebo、Agent 或模型调用；没有修改生产飞行/规划逻辑。

## 新诊断记录已经覆盖的路径

`search_diagnostics.py` 在启用采图证据目录时，记录局部 JSONL：

```text
confirmation_goal
  → planner_evaluate
    → remote_viewpoints（实际请求和实际响应）
      → executor_geometry（实际查询位姿、相机参数与几何结果）
      → executor_ticket_issue
  → planner_select
report_scene（失败栈）
executor_viewpoints_endpoint（包含阶段外失败的端点栈）
```

不写入原始图像、密钥或可复用执行票据。诊断 I/O 失败不得覆盖原异常，也不改变飞行行为。

后续诊断实例的 10 个真实规划批次已用实际查询响应精确回放通过。
58 次执行器查询全部完成；Python 3.11 回放出现的 13 次 paths 末位数值差异，
用与执行器一致的 Python 3.8 再算后全部精确匹配。该解释器数值差异不解释旧的类型异常。

回放一个真实规划批次：

```bash
PYTHONPATH=.:scripts .venv/bin/python scripts/replay_nbv_diagnostics.py \
  --log <实例>/captures/nbv-diagnostics-<桌面进程PID>.jsonl \
  --index 0 --output <新的结果文件>.json
```

执行器几何重算必须使用对应解释器和哈希匹配的世界：

```bash
PYTHONPATH=.:scripts /usr/bin/python3 scripts/replay_nbv_diagnostics.py \
  --log <实例>/captures/nbv-diagnostics-<执行器PID>.jsonl \
  --world scene/test_environments/forest_rescue.world \
  --index 0 --output <新的结果文件>.json
```

输出必须同时检查 `original_outcome`、`replay_outcome`、`all_recorded_queries_used` 和
`matches_recorded_prediction`。脚本不会覆盖旧输出；执行完成但结果不同也不会退出成功。

## 还缺什么，下一步如何判定

原任务的 LangGraph 检查点只在内存中，实例已关闭；公开验收结果没有完整 tool_results/HTTP 输入，
也没有旧异常原始栈。ULog 能恢复物理位姿，不能恢复当时 Python 对象、请求顺序或并发状态。
现有回放因此不能排除未采到的状态组合，也不能证明原因已经消失。

若带诊断的后续任务再次失败：

1. 先核实独立安全收尾和真实落地上锁，保留整个实例。
2. 从 `report_scene` / `remote_viewpoints` 错误沿 batch_id 查到执行器端栈，确认错误发生在许可、几何、票据还是响应阶段。
3. 使用同解释器及同世界哈希回放该批次；能复现后再做最小修复和回归。
4. 只针对已证明的根因修改，不把异常消息字符串作为恢复授权，不扩大搜索预算或放宽坐标/票据门禁。

没有新故障记录时，将本问题保持为“历史异常根因未确定、已补诊断”，而不是不断新增算法或宣称修复。
最新正常林地闭环仍可独立使用；其通过不改变本异常的状态。

对外报告始终分别给出视觉发现、几何关联/多视角复核、任务结果和实际飞行结果。
例如，“照片中出现疑似目标；几何关联不足，未确认；任务因预算用尽结束；飞机已返航落地上锁”是可成立的组合。
不能因安全降落而宣称目标任务成功，也不能用几何确认代替双图的视觉支持。
