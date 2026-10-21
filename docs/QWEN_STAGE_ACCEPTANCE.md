# Qwen 单 VLM 真实验收准备

本阶段仍是准备工作。新增 runner 默认只输出计划；只有显式 `--execute` 才提交任务。本轮未发模型请求或飞行指令。

只读配置核对时，网页使用 `single_vlm`，规划和视觉均为 `qwen3-vl-plus`，provider 为 `dashscope.aliyuncs.com`。通过生产 `runtime_configuration` API 核对现有配置，兼容接口路径为 `/compatible-mode/v1`，密钥仅记录配置状态 `true`，未读取或展示 `.env` 内容。网页当时 landed/disarmed、无 active command，实际 EV healthy；这些是当时的观测，执行前 runner 必须重新检查。

## 三个有界自然语言任务

| 顺序 | 场景 | Runner | 能证明什么 |
| --- | --- | --- | --- |
| 1 | 起飞至 3.5 米，在起飞点上方保持位置；模型从真实左目图像自行选择可见静态实体，保存一份框和周围关系报告，然后返航降落 | `test_stereo_positive.py --preset hover` | 首个真实 VLM 框产生符合质量门禁的历史双目表面点 |
| 2 | 起飞至 3.5 米，经真实 LiDAR 导航到唯一 ENU 观察点 `(1,0,3.5)`；进行一次相同观察，然后返航降落 | `test_stereo_positive.py --preset single_view` | 有界自然语言导航、到点采图、真实模型选择与双目定位的组合 |
| 3 | 在目标 ENU `x[3,5], y[0,2]` 按 1 米网格寻找蓝色桶，最多四观察点，逐点保存报告，最后返航降落 | `test_stereo_search.py` | 区域采样、预算/终止、实际视野和报告进度；当前几何下很可能是诚实的未确认预算完成 |

第二项的观察点来自许可范围和受限任务设计，未从对象真值选择。不能把目前蓝色桶区域搜索当作正向发现验收：蓝桶实例真值在导航区域外，预算完成只能表明本次采样未确认，不能说明整个区域不存在目标。

查看计划不会发任何请求：

```bash
/usr/bin/python3 scripts/test_stereo_positive.py --preset hover
/usr/bin/python3 scripts/test_stereo_positive.py --preset single_view
/usr/bin/python3 scripts/test_stereo_search.py
```

完成前序定位融合与故障验收后，一次只执行一条；每次必须使用全新输出目录：

```bash
/usr/bin/python3 scripts/test_stereo_positive.py --execute --preset hover \
  --output logs/ev/qwen_positive_hover_01

/usr/bin/python3 scripts/test_stereo_positive.py --execute --preset single_view \
  --output logs/ev/qwen_positive_single_view_01

/usr/bin/python3 scripts/test_stereo_search.py --execute \
  --output logs/ev/qwen_region_search_01
```

示例目录必须不存在。Positive 默认任务时限 600 wall 秒；区域搜索默认 900 wall 秒。任务无自动重试；请求、解析或定位失败的首次证据保留。执行和回收仍受当前 manager/PX4 身份、command ID 与 accepted mission ID 约束，身份变化不会向不同 manager 发降落。已在执行的、父任务有实际所属证据的 child land 仅等待。

## 正向定位的通过条件

`scripts/test_stereo_positive.py` 复用区域 runner 的身份、原始任务/状态/轨迹日志和 guarded recovery，未改生产 Agent 或 ROS 执行器。任务必须完成一个起飞、一个真实图像报告、报告后的返航与降落。只接受预设观察点，不允许把目标坐标作为航点，不允许重复飞行阶段。`hover` 禁止水平 goto，`single_view` 只接受用户指定的那个 goto；路径中间点由现有真实 LiDAR 执行器生成。

正向通过至少需要一个 **模型选中目标框** 对应 `stereo_location.status=ok`，同时检查：

- 实际图像进入同一 Qwen 的模型调用，使用该 capture 的结构化报告，恰好一次 capture/report。
- 取证 JPEG 与模型输入 hash、640×480 尺寸、历史 image stamp、pair ID、左右源图 hash、完整 VIO/EKF epoch 一致。
- 实际匹配像素数量、有效比例、左右一致性、纹理、深度离散和不确定度满足原双目门禁。
- 代表像素位于模型框中；用标定重新三角化得到相同 optical XYZ，按采图时刻 `base_link` 位姿和固定 optical/body 标定转换后得到相同 ENU XYZ 与协方差。
- 有到点/高度及采图时间附近的真实 airborne telemetry，最终落地上锁；历史点明确不可作为航点。

空目标或全部 `unavailable` 允许如实报告，但 **positive localization NOT passed**。不补手写 ROI、不换框、不重拍或换目标重试。每个失败保留公开任务和状态证据；已落地时不额外发回收飞行。

质量通过的点仍是框内可见表面，框可能包含背景；它不等于物体中心。模型类别/周围关系和 covariance 的统计可信度均未校准。Runner 始终记录 `object_identity_ground_truth_verified=false`、`metric_accuracy_ground_truth_verified=false`。

现有 `UAV_CAPTURE_EVIDENCE_DIR` 开启时，真实 stereo capture 将同时归档 `<capture_id>.jpg`（左目模型输入）、`<capture_id>.right.jpg` 和 `<capture_id>.json`。JSON 的 `stereo_evidence` 保存同一 pair 的源 stamp、baseline、左右 CameraInfo/hash、body ENU pose 和 epoch；无 `image_base64` 或 Gazebo 真值。右目不会送入模型，也不新增采图请求。Positive runner 必须复制并重新验证真实右目 JPEG 才允许通过，旧 mono 取证仍保持兼容。

## 独立真值评价留待真实图像后

现有 `scripts/evaluate_scene_boxes.py` 是离线 SDF AABB 投影评估，不能代表实例分割或定位精度。它要求 `camera_pose_world`；当前 stereo FrameStore 只归档 `body_pose_enu`，不能直接套用而声称已经完成世界坐标精度评价。

真实图像产生后，独立评价器可以单独记录 Gazebo `ModelStates` 与 stereo 的真实源 stamp、车辆 pose、JPEG hash，用相同机体/光学校准转换到 world，再对选中的可见表面进行独立比较。该日志不得加入 Agent snapshot、prompt、工具结果或飞行规划。`ModelStates` 的 stamp 基于接收时刻，评价必须保留图像时刻前后的 truth 样本、bracketing/interpolation 和近似同步限制，不能用模型响应时刻的 pose 替换采图时刻。目标身份和表面对应关系应先检查实际 JPEG，再决定评价对象；不从预想可见物体设定虚假的正向结果。

新增 `scripts/record_stereo_truth.py` 提供独立只读取证入口。默认只显示计划，只有明确的 `--record` 才订阅 `/gazebo/model_states`、`/mavros/local_position/pose`、`/clock`。三个订阅均为 `queue_size=1`；不订阅图像 topic，不创建 Publisher/ServiceProxy，不调用网页、模型、capture 或飞行接口。它固定现有 manager/PX4 进程身份，身份变化立即保留失败并关闭自己的订阅，不执行降落回收。

```bash
/usr/bin/python3 scripts/record_stereo_truth.py
```

进入真实模型任务阶段时，在 Qwen runner **之前**启动独立记录，以覆盖后续采图时刻；输出目录必须不存在。最多记录 1800 wall 秒，默认最多保留 16 个新 capture：

```bash
/usr/bin/python3 scripts/record_stereo_truth.py --record --duration 900 \
  --output logs/ev/qwen_independent_truth_01
```

此命令本轮未执行。Recorder 只扫描被固定 manager 的既有 `captures` 目录；启动前的 capture 排除。每出现新的完整 manifest，读取左右 JPEG 各一次、重新验证 source stamp/hash/CameraInfo/pose/epoch，并复制到自己的 `captures` 子目录。它不请求重拍。源 archive 目前只保存 `Image.header.stamp.to_sec()`；输出明确 `source_stamp_ns=null`，不能把 float 乘以 1e9 冒充原始纳秒 stamp。旧 mono manifest 记为 unavailable，不伪造右目。

`observations.jsonl` 保存原始 body truth 和 MAVROS source-header ENU pose；`captured_pairs.jsonl` 保存实际同帧 pair 来源；`associations.json` 保留图像时刻前后的 receipt truth、间隔与近似插值，及采图时刻的实际 ENU pose/epoch和附近原始 ENU 样本。只有 `before < image_source < after` 且 receipt 间隔不超过 **0.05 sim 秒**才提供近似 body world pose；缺少两侧样本、超界、clock 倒退或端点重复均记 unavailable。`ModelStates` 无 source header，`source_stamp_sim_s` 始终为 null；该门禁限制 receipt brackets，不能证明消息生成时刻一致，也没有界定运输/调度延迟。小队列可能丢帧，因此不声称完整 callback 覆盖。暂停/重置时钟可能使关联失效。

后续表面评价可使用 `optical_surface_to_world`：先按已标定左目到 `base_link` 的旋转和 `[0.12,0.06,0.18]` 米杆臂转换，再用独立 Gazebo body world pose 转换。**不从初始机体 yaw 推导 ENU→world 的精确变换**；磁航向偏差和 ENU gauge 仍须独立评价。当前 recorder 仅记录机体 truth，不预选目标身份、对象真值位置或几何；观察实际 JPEG 后，再由独立评价器读取相应对象几何并说明 receipt 同步限制。记录完成和关联可用都不等于物体定位精度通过；`metric_accuracy_ground_truth_verified=false` 始终保留。

记录器复用已有 bounded GC session：订阅创建后临时暂停本记录进程的 cyclic GC，回收时先注销订阅再恢复原 GC 状态；不修改 source stamp、consumer age 或系统融合门禁。应用记录上限 200000，超限保留失败，不悄悄丢弃样本。

## 离线验证

新增 `tests/test_stereo_positive.py` 覆盖真实模型框契约、空/低质量拒绝、hash/epoch/几何/不确定度拒绝、任务范围、默认零请求、exclusive JPEG 归档及 failed-task 双 ID 回收。新 runner 复用 `scripts/test_stereo_search.py` 的可选 audit callbacks；默认 search 接口和验收语义保留。

首轮准备组合为 `test_stereo_positive` 和 `test_stereo_search_acceptance`，两 Python 各 23 项，证据保存在 `logs/ev/qwen_stage_prepare_01/validation.json`；三个默认计划也在该目录。新增 paired archive 的生产 graph/字节取证测试另保存在 `logs/ev/stereo_paired_archive_contract_01/validation.json`，未覆盖首轮记录。所有准备期测试只有 fakes，没有真实模型、HTTP、ROS service 或飞行。

独立 truth recorder 的纯测试检查 source/receipt 区分、小间隔 bracketing/拒绝、独立 optical/body/world 几何、实际左右 JPEG hash 归档、旧 mono 缺证据、final-poll、foreign owner、部分订阅失败回收和默认零请求。测试与 request-free CLI 证据保存在 `logs/ev/stereo_truth_collector_contract_01/validation.json`。
