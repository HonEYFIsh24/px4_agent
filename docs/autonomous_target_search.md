# 自主目标搜索

适用范围：single_vlm、已验证静态 Gazebo 三维场景、健康的 PX4 本地定位。现有模型配置保持不变。

输入“找到前方的油桶，拍照分析并报告周围情况”即可授权完成目标所需的仿真飞行。不要求用户提供路线、区域坐标或批准每一步。“前方”由任务开始时的实际 ENU 机身姿态确定。

## 执行闭环

1. 读取遥测，`begin_target_search` 保存目标、起点和初始机头方向；`initial_altitude` 默认 3 米，有用户指定高度时按指定值。
2. 地面时工具缓存初始起飞动作，`execute_target_search_view` 直接执行；空中从当前位置观察，不重复起飞。每阶段返回 `next_tool`，不把工具衔接问题交给用户。
3. `capture_scene` 获取同步相机帧，同一 VLM 看图后 `report_scene` 保存观察。
4. 未发现时由模型选择 rise / left / right / forward / auto 并提供理由。`next_target_search_view` 检查候选三维路径，跳过重复位置和不可达点，返回待执行观察点。
5. `execute_target_search_view` 直接执行缓存的已检查点，避免模型转抄浮点坐标。执行器生成避障航点，重新采图并保存报告。兼容原 goto 工具，但只接受与待执行点一致的坐标（1e-6 米浮点往返容差）。
6. 发现目标或达到预算后默认返航降落。显式要求悬停时使用 `end_behavior=hold`。

候选路径通过只读 `/navigation/preview` 检查，候选不可达不是已执行的飞行失败，不触发失败重试禁令。每次实际飞行仍经过独立执行器和原有 VIO/坐标系/碰撞检查；真正的飞行、接口或报告失败保留停止及独立降落机制。

## 预算和证据

- 起点水平半径 12 米，最多 8 次有效观察；900 秒后禁止发起新的观察或搜索移动，已开始的飞行与降落收尾可能额外耗时。
- 后续移动只允许到当前任务待观察点，保存报告后才可选择下一点。
- 报告要求本次指令已传给模型的图像、唯一 capture_id、当前观察点和一致的坐标 epoch。
- 视觉发现允许结束搜索；不要求先得到两视角精确定位。目标 label 必须与搜索 query 相同。米制位置是否可用由双目工具另行判断。
- 没找到只代表实际观察视野未确认目标。没有可达观察点或预算耗尽会如实报告，不声称目标不存在。
- 工具中的搜索状态用于阻止模型用文本回复提前结束任务；连续两次纠正仍不继续调用工具则失败并由现有独立执行器收尾。
- JSON 工具失败也会关闭搜索状态；确认落地后允许结束，不会因残留“搜索中”状态重复请求降落。
- 模型只接收任务相关的精简遥测与观察记忆，完整原始执行证据保留在审计中，减少逐轮重复 EKF 诊断造成的上下文膨胀。

当前候选生成使用升高/横移/前进的离散位置，属于有界自适应观察，不是完整的任意视点优化或动态场景规划。目标识别仍可能误判，不把语言模型判断当精确物体定位。

## 验证

`tests/test_autonomous_search.py` 包含任务级状态/预算/所有权/坐标重置检查，以及真实 LangChain 执行图中的“未发现→模型询问被纠正→升高→新图发现→返航降落”测试。模型、相机与飞行输运在该测试中为本地替身，不能作为真实识别率证明。

真实模型及 Gazebo 验证日志：`logs/setup/autonomous-search-validation.log`；脱敏任务、观察、模型调用、实际飞行结果：`logs/setup/autonomous-target-search/result.json`。真实验收以这些运行证据为准。

可复现的实测脚本：`.venv/bin/python scripts/validate_autonomous_search.py --execute`。不带 `--execute` 仅显示任务说明；执行会向当前配置模型发送仿真相机图像并实际运行 SITL 飞行。必须先启动桌面系统，且验证起始状态必须为落地上锁、VIO 已实际参与飞控。

首次 2026-10-22 实测在五次观察后进入降落，随后服务关闭，记录没有完整结束，不能算自主找到目标通过；保留在 `logs/setup/autonomous-target-search/first-attempt-interrupted.json`。

2026-10-26 首次完整通过：任务 `000ce8ba00c747f1`，实际模型保持 `qwen3-vl-plus`。起飞约 3 米后未发现目标，自主前进、右移、升高至约 5.2 米，再升高至约 7.3 米；第 5 次真实图像观察报告蓝色油桶及周边集装箱/车辆/立柱。随后自主返航降落，结束证据 `armed=false`、`landed_state=1`，用时 284.8 秒，全程未索取用户路线。记录：`logs/setup/autonomous-target-search/20261026-first-complete-success.json`，过程：`logs/setup/autonomous-search-validation-20261026-attempt2.log`。此轮验证脚本未启用图像归档，仅保留图像调用与报告证据；照片归档另由最终复核确认，不混作此轮已存照片。

回归记录 `logs/setup/autonomous-search-regression.json`：161 项检查通过，包括自主搜索闭环、未发现继续观察、预算与坐标系检查、JSON 失败终止、浮点容差、三维路径预检查/提交一致性、旧雷达导航、桌面与本地 HTTP 接口。

最终带照片复核也通过：任务 `04120633a28144d0`，六次观察，模型自主升高、推进及横移后在约 7.5 米观察位置发现蓝色油桶，自主返航降落，最终就绪且落地上锁。记录：`logs/setup/autonomous-target-search/20261026-final-complete-success.json`，过程：`logs/setup/autonomous-search-validation-20261026-final.log`。六组左右相机照片与采图元数据已归档；逐份校验左图 SHA256 与模型报告中的图像哈希相符。目标照片 `logs/setup/autonomous-target-search/captures/72f250ca383b4560ad34a0c1a12ee898.jpg` 已人工目视核对，蓝桶、红色集装箱、白色货车、警示柱和交通锥均可见。可读报告：`logs/setup/autonomous-target-search/report.md`。
