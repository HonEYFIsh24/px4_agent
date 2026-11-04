# 果园降落速度异常与 20 米许可更新（2026-11-04）

## 当前结论

已证明的直接链路是：**触地附近惯性预测与实际运动偏离 → 垂直速度创新过大 → EKF 拒绝该组三轴视觉速度观测 → 持续未融合后重置到视觉速度 → 原坐标守卫锁存**。不是 Agent、NBV 或模型接口引发：3.5 米纯执行器测试和 10 米语言采图测试都出现相同过程，前者没有 Agent。

目前最强的上游嫌疑是 **Gazebo 接触动力学与 IMU 加速度输出不一致**。两次在接近地面停止下降/回升时，飞控 IMU 都报告约 69.6 m/s² 的向下惯性加速度峰值；视觉速度与仿真实际速度却基本吻合。这已将诊断范围缩小到触地及惯性测量链路，但尚不能唯一断定是独立 IMU 链接的约束力、物理求解/回调时序，还是插件测量实现。没有修改 IMU、PX4、VIO 参数或融合门限，没有新飞行。

## 两份真实记录

| 项目 | 3.5 m 最小飞行 | 10 m 语言采图 |
| --- | --- | --- |
| 实例 | `20261104-103833-0N8BER` | `20261104-104828-YNpiUH` |
| 诊断窗口，仿真秒 | 98～112 | 130～145 |
| IMU 向下加速度峰值 | 69.545 m/s²，101.740 s | 69.647 m/s²，134.256 s |
| 首次速度创新拒绝：发布时间 | 101.916 s | 134.420 s |
| 同次视觉测量源时间 | 101.756977 s | 134.261459 s |
| 垂直创新峰值 | 1.673 m/s | 1.646 m/s |
| 垂直检验比峰值（>1 拒绝） | 1.924 | 1.864 |
| vxy/vz 计数均 2→3 | 105.196 s | 137.740 s |
| 重置 delta_vz，NED | −1.24607 m/s | −1.24839 m/s |
| 视觉垂直速度对真值 RMSE | 0.01390 m/s | 0.01550 m/s |
| 同项绝对误差 P95 | 0.02885 m/s | 0.03132 m/s |
| 视觉源最大采样间隔 | 0.060009 s | 0.060008 s |

速度误差比较的是 `estimator_aid_src_ev_vel.observation[2]` 与按 **timestamp_sample** 对齐的 Gazebo NED 垂直速度，未混用 body-FRD 速度与 ENU 速度。真值仅用于离线评估，不发送给飞控或语义地图。IMU 比较使用高频 `sensor_combined`、因果匹配的姿态四元数，将 FRD 比力旋转至 NED 后加重力，不直接把 body-z 当作世界加速度。

两窗口均：视觉 reset_counter=0，quality=1（接口质量字段，不是标定概率），cs_ev_vel=1、cs_ev_yaw=0、cs_yaw_align=1；未记录 accelerometer_clipping、bad_acc_clipping 或 bad_acc_vertical。ULog 无 dropout，不能据此断言所有原始 IMU 样本均被日志保存：窗口内已记录 IMU 积分区间只占约 79.1% / 75.6%，最大日志间隔 8 ms。因此本分析没有把不完整样本积分当作完整碰撞冲量。

这也修正“速度融合中断”的表述：**融合模式没有关闭，数据仍持续到达，但具体观测被创新门禁拒绝**。没有证据支持网络/API、VIO 坐标重置、整体航向变换错误或 IMU 量程饱和是这两次的起点。

## 对应本机源码的触发链

本机 ULog `ver_sw=a7659ddf12612c47bd6dedb8c5c609dd2133fae3`，与 PX4 源码 HEAD 一致。以下路径相对 `/home/fxxk/PX4-Autopilot/`：

1. `src/modules/ekf2/EKF/velocity_fusion.cpp::updateVelocityAidSrcStatus()` 用预测速度减视觉观测生成 innovation，再计算 test_ratio。本次首先超界的是垂直分量。
2. 同文件 `fuseVelocity()` 要求整组 `innovation_rejected=false` 才融合三轴，因此垂直创新拒绝也阻止本轮水平速度更新，不能因 vxy/vz 都增加就推断两条独立故障。
3. `ev_vel_control.cpp::controlEvVelFusion()` 在仍有有效质量、允许继续融合但超过 `_params.no_aid_timeout_max` 时，可以走 `EV velocity fusion failing, resetting` 路径，调用 `resetVelocityTo(measurement, measurement_var)`。视觉 reset_counter、EV yaw 及 yaw_align 在窗口内未变，不支持“新视觉纪元/航向重新对齐”解释。
4. 该版本 `EKF/common.h` 中 `no_aid_timeout_max` 是 **1,000,000 μs 的内部常量**。不要与实际参数 `EKF2_NOAID_TOUT=5,000,000` 混淆，后者用于导航有效性报告。10 m 记录中重置后的融合时间 137.588 s 减此前最后融合 136.564 s = 1.024 s；3.5 m 同项 105.044−104.020=1.024 s。与这条超时重置路径一致，不能错误解释成 5 秒超时。
5. `resetVelocityTo()` 同时调用水平与垂直重置函数，导致两个计数一起增加；现有 bridge/EV 守卫随后禁止旧坐标继续导航。该保护未改动，已开始的安全降落继续完成，实际落地上锁。

两次日志读取参数相同：`EKF2_EV_CTRL=7`、`EKF2_EVV_GATE=3`、`EKF2_EVV_NOISE≈0.3`、`EKF2_EV_DELAY=0`。没有通过扩大创新门限、伪造协方差、忽略计数或打开视觉航向来避免锁存。

## 上游嫌疑及尚缺证据

当前 `vio/models/iris_vio/iris_vio.sdf` 的原 PX4 IMU 安装在独立 `/imu_link` 上，质量 0.015 kg，以限位 revolute joint 连接机体。上游 `gazebo_imu_plugin.cpp::OnUpdate()` 在 Gazebo 11 分支读取该链接的 `RelativeLinearAccel()`，减去机体系重力、加噪声后发出。MAVLink 插件再做 FLU→FRD；本项目 `body_imu_plugin.cpp` 将同一原始消息转发给 VIO，没有另一个人为生成的 IMU。

本机 Gazebo 11.15.1 库的只读反汇编确认 `Link::RelativeLinearAccel()` 调用 `RelativeForce()` 并除以 `Inertial::Mass()`；不是直接对完整机体速度差分。因此触地约束/链接受力与真实机体运动是否一致，是具体且可检验的怀疑方向。反汇编和源码哈希已保存，但仅凭这一实现不能断言约束冲量一定漏掉或符号必然错误。

下一步最小取证：在隔离 SITL 中只加只读记录，同步保存 `base_link` 与 `/imu_link` 的位姿/速度、力/质量加速度、速度差分加速度、接触事件和原始 IMU 消息；明确 WorldUpdateBegin/End 时序。对照两者是否从接触瞬间开始不一致，再决定是修正模型刚性连接、接触建模还是 IMU 生成方式。任何测量修复须同时验证 PX4/VIO 使用相同传感器来源及时间戳；先地面/最小起降，不能用 20 m 任务代替基础复验。

## 本轮 20 米修改

用户明确要求将 10 m 上限提高至 20 m。`mission.MAX_ALTITUDE=20` 作为共享数值，接入 Agent schema、执行器遥测与搜索边界、静态路径检查、区域/自主搜索参数、候选几何检查和 NBV 生成。NBV 仍保留上下 0.5 m 目标余量，因此观察候选最高 19.5 m；搜索执行跟踪容差仍是原来的 0.35 m。静态场景许可高度相应调整，原完整碰撞几何继续检查，碰到低顶/障碍依然拒绝。果园 manifest/注册哈希及生成器同步；世界视觉和碰撞文件本身未变。

水平任务 30 m、搜索 12 m、速度、观察预算、VIO/PX4 纪元、票据及安全收尾均未放宽。旧 lidar_grid 固定高度约束仍保留。相机识别/几何覆盖有效量程也未凭空扩大；20 m 指**本地 ENU 目标高度**，不是严格离地高度或保证整园入镜。界面、提示词与 README 已更新。**20 m 只完成代码/受控验证，尚未真实飞行验收。**

## 可复核证据

根目录 `logs/setup/orchard-velocity-diagnosis-20261104/`：`minimal/`、`overview/` 含 JSON 摘要和高频 CSV；`fusion-details.json` 保存参数/标志/首次拒绝，`source-hashes.json` 和两份 Gazebo 反汇编保存实际实现依据；原失败 ULog 保留原处。

复放命令：

```bash
.venv/bin/python scripts/analyze_landing_velocity.py \
  --ulog logs/setup/orchard-task-20261104/viewpoint-retry/px4.ulg \
  --start 98 --end 112 --output /tmp/orchard-minimal-landing-review
.venv/bin/python scripts/analyze_landing_velocity.py \
  --ulog logs/setup/orchard-overview-20261104/px4.ulg \
  --start 130 --end 145 --output /tmp/orchard-overview-landing-review
```

输出目录必须未存在，避免覆盖旧证据。代码测试包括 20 m 接受/超限拒绝、模型 schema 一致、实际静态图路径、低顶碰撞仍拒绝、12 m 半径不变、NBV 19.5 m 余量及 IMU 坐标变换/坏输入拒绝。

最终联合回归 **442 项通过、失败 0、跳过 0**，78.981 s，记录 `regression-final.log`。首轮同组 442 项有 2 项失败，均仍断言旧 10 米界面/11 米非法；更新为 20 米展示/20.01 米非法后复跑，失败记录 `regression.log` 保留。另有首轮针对性 148 项及新增 7 项通过，不能累加成独立测试数量。Python 3.8 执行器模块、3.11 Agent/诊断模块编译及 diff whitespace 检查通过。

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 PYTHONFAULTHANDLER=1 \
PYTHONPATH=.:tests:scripts .venv/bin/python -m unittest -v \
  test_altitude_limit test_landing_velocity_analysis test_orchard_environment \
  test_scene_config test_coverage_search test_search_tools test_search_diagnostics \
  test_search_termination test_desktop_search_data test_desktop_workbench test_desktop \
  test_web_server test_gazebo_seed_evidence test_target_confirmation \
  test_nbv_search_integration test_single_vlm test_ev_heading_guard test_bridge_camera \
  test_analyze_heading_ulog test_heading_reference_probe test_autonomous_search \
  test_search_graph_integration test_takeoff_heading_alignment test_ekf_landing_evidence \
  test_semantic_map test_viewpoint_planner test_bridge_static_navigation test_yaw_control \
  test_forest_environment test_alternate_environment test_static_navigation \
  test_recovery_ownership test_agent_tool_schema test_autopilot_landing test_mission
```
