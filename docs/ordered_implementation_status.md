# 按优先级实施结果

更新：2026-10-09。按 VIO 观测、GPS/EV 融合、无 GPS 飞行与失效处理、单 Qwen 视觉任务依次推进。

| 阶段 | 实际结果 | 证据 |
| --- | --- | --- |
| VIO 与观测可靠性 | 已接入机体 IMU、刚性双目相机、VINS-Fusion 与带时间/重置门禁的 EV 适配器；修复记录器启动时先接收里程计而时钟和 timing 尚未就绪的竞态。两套 Python 各 45 项针对性测试通过，原评价门限保持不变。 | [订阅修复](../logs/ev/recorder_startup_order_design_01/READY.json) |
| GPS + EV 基线 | 同一实际实例完成五步飞行、完整落地后观测、420 秒原始记录、17 参数恢复和停机，整轮 PASS。 | [GPS 基线](../logs/ev/gps_fused_server_thread_budget_sequence_01/sequence_result.json) |
| 无 GPS 基线与安全 | GPS 融合关闭，以 EV 为高度参考、保留气压备份；完整基线 PASS。断流、延迟和修复后的首次实际 VINS 重置三类安全案例均完成原 9 项、附加 8 项、原 10 项及联合 37 项检查。每轮独立降落、17 参数恢复和所属进程关闭有实际证据。 | [无 GPS 基线](../logs/ev/no_gps_ev_height_reference_sequence_01/sequence_result.json)、[安全阶段汇总](../logs/ev/no_gps_ev_height_reference_reset_continuation_01/sequence_result.json) |
| 单 Qwen 图像与搜索 | 代码、实际单模型配置、严格配对双目归档、表面点定位和有限区域搜索接口已准备；实际首个悬停任务 FAILED。前两次纯文本 qwen3-vl-plus 请求成功，第三次纯文本请求 45.086 秒后 APITimeoutError，无 HTTP 状态或 request ID，尚未采集/发送图像。已自动降落上锁，无模型/飞行重试，后续观察点和搜索未执行。完整只读记录完成 900.860 秒；17 项原参数已由实际命令恢复并读回，19 个所属进程已关闭，[整轮结果](../logs/ev/qwen_hgt3_lp4_stage_01/sequence_result.json)保持 FAIL。 | [原始失败任务](../logs/ev/qwen_hgt3_lp4_stage_01/hover_task_01/final_task.json)、[安全归属复核](../logs/ev/qwen_hgt3_lp4_stage_01/model_failure_review_01/safety_saved_review.json) |

原三轮序列中的 RESET 地面采集失败及全部历史 FAIL 原样保留。新的安全汇总明确引用已通过的前两轮与诊断修复后的新 RESET，没有把原序列改为通过。

目前不能宣称 Qwen 图像识别、米制定位或搜索的实际验收通过。即使未来通过，输出也只表示模型选择的历史可见表面点；ModelStates 时间关联是接收时间近似，不等于目标中心、身份真值或独立定位精度验证。绕障范围仍为固定高度的静态二维障碍，不包含三维或动态障碍保证。

详细架构、启动方法与原始实验范围见 [VIO 融合与主动视觉搜索](vio_fusion_and_search.md)。

超时调用链的 [源码与 SDK 复核](../logs/ev/qwen_hgt3_lp4_stage_01/model_failure_review_01/source_timeout_review_01/review.json)未确定具体网络/服务端原因；当前不支持盲目重试或调整门限。下一步先做与飞行分离的单次地面调用诊断，补充不含密钥或请求正文的超时阶段证据，再继续图像和搜索验收。
