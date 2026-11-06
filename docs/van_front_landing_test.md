# 白色货车前方降落：首次实际测试

2026-11-05，按用户要求只执行一次任务，不录像，不修改飞行、搜索或确认算法。

## 结论

**原任务未完成。** 实际看到了白色货车，但三次复核均未达到既有多视角确认条件；预算用尽后按模型选择的 `end_behavior=hold` 结束搜索并悬停。未选择货车前方落点，未进行目标相对降落。

原应用返回 `succeeded`，结构化结果却明确是 `budget_exhausted`、`target_confirmed=false`。这是执行完搜索流程的状态，不能充作整句用户任务成功；本次验收单独记为失败。原始结果未修改。

搜索结束后测试执行者另行通过现有执行器提交 `return_home + land`，已实际落地上锁。该操作是测试收尾，不是模型完成了货车前方降落。

## 实际配置和结果

| 项目 | 本次记录 |
| --- | --- |
| 原句任务 | 找到白色货车后降落在货车前方。 |
| 世界 | `scene/test_environments/courtyard_delivery.world` |
| 世界 SHA256 | `4e8bb8b06314def07753afd3fae70ee3f20cbc61add2282fdb1ebf8e3739b30b` |
| 随机种子 | 17，已核验实际 gzserver 参数 |
| 实例 / 指令 | `20261105-092828-SMIrkx` / `111e16127d714eb6` |
| 模型 | qwen3.8-flash；地面文字、工具、实际相机图像预检均通过 |
| 搜索模式 | 几何 NBV、确定性选点、离散航向、多视角确认 |
| 搜索墙钟时间 | 约 213.49 秒，不含独立收尾 |
| 真实观察 / 地图更新 | 8 / 8 |
| 实际 NBV 换位 / 模型调用 | 7 / 21（另有 3 次只读模型预检） |
| 旋转后采图 | 4 次实际旋转，后续左右帧时间均晚于旋转完成时间 |
| 复核结果 | 3 次 unconfirmed；第一次仅裁切框区域假设，后两次没有唯一几何对应 |
| 搜索最终状态 | budget_exhausted，未确认 |
| 提前收尾纠正 | 本次未触发 |
| 独立返航降落任务 | `daaefe557fd2`，实际 succeeded、armed=false、landed_state=1 |
| 录像 | 无，仅真实照片、桌面阶段截图及状态日志 |

第 4 次观察首次报告疑似货车，后续画面能看到更多车体，但遮挡和关联证据不足。模型在第二次复核称车体完整，实际原图仍有墙体遮挡，不能用模型文字替代图像事实。

降落末段再次出现已知 `PX4 local velocity reset during flight`，最终锁存保留、EV healthy=false。状态采样首次记录该异常时，EKF 观测仿真时间约 314.960 s，ENU 高度约 0.100 m；这是采样首次发现时刻，不是 ULog 精确事件时刻。vxy/vz 最终为 3/3。实际落地上锁由新鲜 MAVROS 心跳与落地状态证实，不能据此宣称全程定位连续性通过。本轮未扩展降落异常专项。

## 证据与验证边界

本机忽略目录 `logs/setup/van-front-20261105/`：

- `summary.json`、`task/result.json`、`task-at-search-end.json`：任务结果、搜索结束时原始状态，以及实际收尾后的状态，保留二者区别。
- `ground-state.json`、`state-samples.jsonl`、`recovery-accepted.json`、`recovery-final.json`：地面预检、飞行、独立收尾和落地证据。
- `rotation-frame-checks.json`：4 组完成事件与左右新帧时间戳。
- `task/*.png`：真实 Qt 窗口阶段截图；`08-multiview-review.png` 是首轮未确认复核，`10-terminal-grounded.png` 是独立收尾后落地画面。
- `nbv-audit/` 和 `confirmation-audit/`：两项现有审计通过；审计检查数据链、几何规则和实际执行，不验证“车头前方落点”，也不证明原任务成功。
- `model-preflight-text/`、`model-preflight-image/`：3 次只读模型预检；没有改变模型输出。
- `stop.log`：按实例身份退出。PX4 参数读回恢复在实例的 `ev-profile-restore-1/result.json`；VIO 输出路径已核对仅本次路径变化后恢复至启动前副本。

原图及报告元数据保存在 `logs/system/20261105-092828-SMIrkx/captures/`。代表性第 6 张照片为 `0dc9ba70aea24dc683e6d2737bea0da8.jpg`。原始任务、图像、ULog 和本机日志不提交 Git。

首个启动实例 `20261105-092732-s2uREA` 在显示管理身份检查时退出，未启动飞控。随后采用生命周期工具要求的绝对脚本路径成功启动；原失败记录保留。本轮没有生产代码修改，没有重新运行单元回归，不能把前一轮 520 项测试计为本轮重跑。

审计复现（输出目录必须不存在）：

```bash
PYTHONPATH=.:scripts .venv/bin/python scripts/audit_nbv_evidence.py \
  --result logs/setup/van-front-20261105/task/result.json \
  --output /tmp/van-front-nbv-audit \
  --world scene/test_environments/courtyard_delivery.world --require-auto-prepare
PYTHONPATH=.:scripts .venv/bin/python scripts/audit_target_confirmation.py \
  --result logs/setup/van-front-20261105/task/result.json \
  --output /tmp/van-front-confirmation-audit
```

## 下一步

先完善任务能力匹配：当前 `AdaptiveSearch.finish_normal()` 只提供返航降落或结束悬停，没有目标相对落点流程，不能静默把整句要求简化成搜索。确实要支持该任务，需要可靠识别车头方向、获得有不确定度的目标相对位置、验证落点与下降通道，再由确定性流程执行并核验终态。现有地标只是可见表面代表点，不能直接当车辆中心、车头或落点；不读取 Gazebo 目标语义真值代替视觉定位。

本次没有证据支持放宽多视角确认或绕过票据。应先解决上述明确能力缺口，再考虑正式多路录制，避免重复拍摄一个无法完成指定收尾的任务。
