# 宽敞庭院、30米搜索与无深度目标接近

2026-11-05。此版本回应实飞中“障碍物挤住货车、搜索空间小、深度失败后反复升高”的问题。完成场景、边界与任务流程修改后，已按用户要求进行一次真实重飞：目标已多视角确认、实际接近并取得双目深度，近照自动验收仍未通过，最终返航落地上锁。下方明确区分实现阶段回归与本次真实运行。

后续代码更新：已分离同帧双目相对测距与绝对地图定位，原图重新匹配和离线质量回放仅第9张通过新近照判据；其他过远、过近、裁切和无深度样本仍拒绝。绝对定位/观察弧约束与NBV未改。修复后已重飞一次，但第二帧报告因targets为字符串被拒绝，未到近照阶段；实现、回归及下一步见 [相对测距修复](target_observation.md)。

## 修复后第二次重飞：报告格式错误中止

实例 `20261105-133438-YwUwer`，指令 `6ece31e9abea4726`，同一场景哈希/种子17/模型/任务。地面预检、6项测距测试和5次模型接口预检通过，代码与603项回归版本一致；本轮没有改生产逻辑。实际2对照片、1有效报告/1地图更新、1次实际票据换位、6次任务模型调用，0多视角复核。旋转156.288秒完成，第二对左右新图均156.448秒；原图哈希及照片坐标身份一致。第二图有货车，但模型将报告targets填成字符串，工具校验拒绝，近照判据未运行。

任务failed后独立执行原地安全降落 `a25e34f201c2`，实际落地上锁，无正常返航。降落期间出现EV融合与速度重置告警，最终EV unhealthy，时间晚于报告错误；不把它写成初始报告失败的原因。实例已按身份关闭、实际参数读回和VIO文件恢复。证据在 `logs/setup/relative-range-flight-20261105/`，无录像；完整入参/原始调用栈未记录，只有500字符错误前缀，不能假称精准复现。三类总体审计均未通过：地图在线回放一致，但任务failed；NBV缺少下一份有效报告而无法完成配对（保留list index out of range），0复核不能证明确认通过。下一步处理此结构化报告阻断项，不修改导航/确认门禁或反复原样起飞。

## 第一次宽敞庭院真实重飞（相对测距修复前）

任务：“找到前方的白色货车，靠近后拍照并报告周围环境，完成后返航降落。”实际实例 `20261105-114711-U319Xn`，指令 `7820572aacfc4a72`，world SHA256 `b624fb8ceabc2d56b8979b3a792678271bbc5165f9bcfb5ce627079b1d324f6a`，实际gzserver种子17。qwen3.8-flash、几何NBV、确定性选点、离散航向、多视角确认，12次/15分钟预算、30米半径、20米最高高度。地面PX4/VIO实际融合、静态导航、真实新鲜双目和文字/工具/图像接口预检均通过，没有使用仿真目标真值指导搜索。

| 项目 | 本次实际结果 |
| --- | --- |
| 有效报告 / 地图更新 | 12 / 12 |
| 票据换位 | 11次，均实际完成，完整路线满足许可和碰撞检查 |
| 目标首次出现 | 第2张；第1张被前景屏障遮挡 |
| 目标双目测量 | 第6～12张有效，共7次 |
| 跨视角复核 | 10次：3次未确认、7次确认 |
| 确认关联依据 | 首次single_depth_projection，后续stereo_unique；bearing_intersection仍只作假设 |
| 模型调用 | 任务36次，另地面预检3次 |
| 实际观察最大水平半径 | 14.191米，超过旧12米，未超过30米 |
| 几何覆盖率 | 14.193%，不表示识别成功率 |
| 起飞后正式绑定 | 成功起飞后103.028仿真秒绑定；相对地面参考偏移0.03497米 |
| 任务墙钟时长 | 462.20秒 |
| 最终搜索/用户目标 | budget_exhausted / 未完成；目标已确认，近照判据未通过 |
| 最终飞行 | 程序返航降落成功，实际上锁，距home 0.058米，EV健康 |

10次真实复核调用都省略源图/当前图UUID，由程序从本任务的待复核状态绑定并核验真实交付图对，没有再发生模型转抄UUID失败。两次实际旋转后采图均验证为新帧：134.484→134.852秒、232.016→232.204秒，左右时间戳一致。12份原图哈希和观察坐标纪元一致。无提前返航纠正自然触发、无独立原地安全降落、无录像；截图来自实际Qt窗口。

### 尚未完成：近照距离不确定度判据

第9张原图 `7ff380e2cbf64a73b422bbd943aa78d5.jpg` 清楚显示货车车头、交通锥、围栏及仓库周边。以下值均由保存的真实双目报告只读计算，没有注入任务或改写结果：

| 观察 | 相机相对表面距离 | 相对径向sigma | 完整ENU协方差上界sigma | 原有构图判据 | 原有近照结果 |
| --- | --- | --- | --- | --- | --- |
| #06 | 10.310米 | 1.312米 | 2.856米 | 通过 | 未通过 |
| #09 | 4.646米 | 0.260米 | 1.388米 | 通过 | 未通过 |
| #10 | 2.653米 | 0.148米 | 0.788米 | 上边缘余量不足 | 未通过 |
| #11 | 5.013米 | 0.693米 | 1.417米 | 通过 | 未通过 |
| #12 | 3.018米 | 0.367米 | 0.806米 | 下边缘余量不足 | 未通过 |

现代码 `target_observation.surface_reference()` 用完整ENU协方差最大绝对行和开方作sigma，超过0.75米就返回null；全部7份有效货车深度因此被近照模块拒绝。实际位姿协方差来自 `configured_conservative_floor`，位置对角为[0.09,0.09,0.16] m²，姿态对角为[0.008,0.008,0.069] rad²（航向约15°标准差）。这份全球表面定位不确定度包含姿态误差，既使世界位置较不确定，也不等同于同帧相机到表面的径向误差。

离线使用原光学系点p及其协方差计算 `r=norm(p)`、`sigma_r=sqrt(uᵀΣ_optical u)`，第9张的两倍sigma区间约[4.127,5.165]米；这说明应检查**照片相对距离**与**绝对地图定位/规划参考**的职责混用。该计算仍使用未标定的模型协方差，只是诊断，不是新验收准则，也不能单凭区间宣称照片可靠或改写本次failed。第10/12张还分别有构图和近端余量不足，不能用任意新判据强行全算成功。

本次实飞确定的修复方向是验证相对测距证据及传播方式、在语义观察中保留完整来源，并继续对目标弧规划保留绝对定位质量约束；随后已完成的离线代码修复见文首链接。不能人为缩小位姿协方差、放宽关联确认、删除失效守卫，或用融合目标坐标减去当前相机位置来冒充独立测距。增加预算不是当前优先方案。该次实飞期间没有修改生产逻辑；后续修复重飞未到近照阶段，以文首最新记录为准。

### 测试、审计与清理

本次新运行的受控回归为 **59项通过，70.405秒，0失败/错误/跳过**：

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 PYTHONPATH=.:tests:scripts \
  .venv/bin/python -m unittest test_spacious_search test_target_observation \
  test_target_confirmation test_bridge_camera
```

13个记录的生产文件与此前597项回归版本哈希一致；没有冒称本次重新运行597项。确认审计通过10轮实际双图交付、几何与结果一致性，**不代表独立人工类别真值审计或近照完成**。地图在线/回放一致、重复图拒绝、原图链、票据和11条实际路线子检查均通过；地图/NBV总体审计仍为未通过，原因包括整项任务failed及此无显式地标关系指令的semantic_field_active=false。原始失败输出保留，没有改写JSON让审计通过。

程序执行未完成目标的返航降落 `d6f70b3d639f`；另一次新鲜遥测核实armed=false、landed_state=1、距home约0.05797米。实例通过既有生命周期脚本退出；恢复PX4实际参数读回值与启动前一致，VIO配置与启动前备份逐字节一致，无本次残留飞行进程。

本机忽略证据目录 `logs/setup/spacious-flight-20261105/`：

- `task/result.json`：原始任务与终态；`task/*.png`：实际桌面关键帧，无录像。
- `summary.json`、`route-checks.json`、`rotation-frame-checks.json`：只读事实汇总与核验。
- `close-observation-diagnosis.json`：逐张双目距离/协方差与生产判据结果。
- `verified-final-ground-state.json`、`stop.log`、`vio-config-restore.json`：落地、退出与配置恢复。
- `preflight-regression.log`、`flight-code-comparison.json`：本次受控回归和生产代码版本核对。
- `nbv-audit/`、`map-audit/`、`confirmation-audit/`及各日志：保留原始整体/子项结论。

原始照片和采图元数据在 `logs/system/20261105-114711-U319Xn/captures/`。这些本机证据、日志和图像不纳入Git提交。

## 场景

新场景是 `scene/test_environments/courtyard_spacious.world`，由 `scene/generate_spacious_courtyard.py` 生成。保留原有建筑、车辆、植被、围栏和材质，地面为76×76米，许可飞行区域在世界XY的[-34,34]米内，仍有0.6米边界余量。主屏障与货车的碰撞包围盒间距从约2.05米增至约9.16米，货车与仓库间距超过8米，与红色集装箱超过12米，留出多个方向的观察通道。这些是离线场景编辑/验收数据，不传给Agent作为目标位置。

原 `courtyard_delivery.world` 和manifest的哈希保持不变，既有实飞记录仍可校验。新world及manifest独立登记于 `scene/test_environments/index.json`。运行时静态地图仍只提供无标签碰撞几何，语义答案只能来自相机报告。

直接 `start_system.sh` 或ROS launch的默认场景已切换为新庭院；显式world参数仍可选择旧场景。桌面启动器默认几何NBV、确定性选择、多视角复核与自动准备。显式选择旧fixed模式时，兼容其单帧/手动准备默认，不强制注入冲突配置。

## 30米权限与地图

`search_limits.SEARCH_RADIUS_M=30` 表示任务地面起点周围的**半径**，不是每一步可任意飞30米。搜索半径、候选过滤、后端查询、票据权限和A*整条轨迹检查一致。场景边界、1米障碍膨胀、坐标纪元、架次注册、原点、一次性票据和操作员中断继续生效；不能通过修改票据或在同一任务中改半径扩大权限。普通飞行本来已有30米上限，本次补齐自主搜索此前的12米限制。有效观察预算仍为12次，时长仍15分钟，最高高度仍20米。

新覆盖网格为60×60、1米分辨率，物理范围60×60米；面积收益按实际单元面积计算。这样扩大覆盖范围时保留了可控计算量。历史48×48、0.5米分辨率、12米半径地图仍可离线回放；直接几何函数保留历史默认参数，正式执行器显式传入当前30米配置。两种profile都有尺寸和索引检查，不接受任意大网格。桌面及离线渲染器按实际列数绘图，不能继续按48列解释新数据。

1米分辨率只用于地面几何观察覆盖；飞行A*的原网格精度及连续线段碰撞检查没有降低。

NBV离线审计也按保存地图的实际半径和当前20米高度许可校验，修正原先硬编码10米而误拒合法高位路线的问题。历史回放保留记录的半径，不把旧12米任务当成30米任务。

## 当前照片有目标但双目深度不足

此时允许基于实际图像方位生成1/2米的短距离接近候选，配合小幅横向偏移和已有航向候选再次观察。它们是无人机相对运动的候选，不是目标坐标；`predicted_target_range_m`仍为null，不能升级为已定位/已确认。

只有源图为当前新鲜观察、方位有效且目标框没有大到接近画面主体时才奖励该接近方向；旧方位、近处大框、遮挡及不可达端点不能取得同样奖励。无深度阶段降低纯视差收益的权重，增加沿方位接近收益和高度变化代价，减少连续升高。若接近被挡，仍从安全侧向/高度候选中选择或按预算结束；不硬性要求每一步缩短未知目标距离。

每一步仍必须由静态几何验证端点和完整路径，再由执行器消费票据，完成后获取新鲜双目帧和真实报告。取得有效深度后，再切换到已有米制观察距离与构图判据。

当新图有唯一且视觉明确支持的当前候选，但无深度跨视角关联不相容时，可从当前图建立**新假设**继续局部观察，并记录与旧假设身份未确定。旧图和地标保留，不合并坐标/ID，不把不相容变成相容。不同物体、明确否定、同类歧义或已有可靠米制参考仍保留原有保护。

## 复核图对与结果

`review_target_candidate` 的源图/当前图ID改为可省略，工具从本任务当前awaiting_review状态绑定实际图对。模型只需提交当前候选ID、视觉描述及支持/不确定判断。旧调用显式提供ID时仍需精确匹配，错误ID继续拒绝；不会把错误调用自动改成另一对图。

实际图像交付、哈希、图对归属及强几何确认判据保持不变。新增真实失败ID反例回归，检查错填仍失败并安全收尾；省略ID的完整Agent图仍能通过当前图对审计。结果统计只计算status=ok的报告，错误条目不再计入有效观察。

## 验证与证据

本机证据位于 `logs/setup/spacious-search-20261105/`：

- `spacious-initial.log`：初始8项专项通过；后续增加真实错误ID和新假设流程用例。
- `regression-initial.log`：557项中1项旧诊断回放失败，原因是FrozenMap未初始化动态cell_count；已修复，保留失败。
- `regression-final.log`：修复后559项通过，105.432秒。
- `regression-release.log`：最终代码、启动入口及实际PX4源码路径的完整回归，**597项通过，108.955秒，0失败/错误/跳过**。
- `world-preview.png` 与 `window-evidence.json`：核对gzclient PID后的实际X11窗口截图，非离线重绘。独立预览目录是 `logs/external-worlds/courtyard-spacious-20261105-113421-336882/`，资源检查无缺失，预览结束码server=0/client=-15，未启动飞行栈。

新增测试覆盖30米整条路径/票据权限、25米可达点与30米外拒绝、历史网格兼容及60列绘图、接近与障碍/旧方位/大框反例、从当前候选建立未关联新假设，以及省略ID和错填ID两条复核路径。早期启动检查有1项因旧机器硬编码路径跳过，已改用实际PX4_DIR或用户主目录，再纳入最终回归。

最终回归命令（本机HTTP测试需要回环端口权限，全部模型/飞行均为替身）：

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}" \
  PYTHONPATH=.:tests:scripts .venv/bin/python -X faulthandler -m unittest \
  test_spacious_search test_target_observation test_task_contract test_landmark_association \
  test_semantic_map test_semantic_map_runtime test_coverage_search \
  test_photo_request_evidence test_photo_framing test_photo_task test_photo_navigation \
  test_single_vlm test_agent_reliability test_agent_tool_schema test_task_memory \
  test_web_server test_search_termination test_nbv_search_integration \
  test_target_confirmation test_search_graph_integration test_autonomous_search \
  test_desktop_search_data test_desktop_workbench test_desktop test_bridge_static_navigation \
  test_recovery_ownership test_bridge_camera test_ev_heading_guard test_static_navigation \
  test_scene_config test_viewpoint_planner test_viewpoint_authority test_search_diagnostics \
  test_altitude_limit test_yaw_control test_mission test_analyze_heading_ulog \
  test_heading_reference_probe test_landing_velocity_analysis test_external_world_preview \
  test_forest_environment test_orchard_environment test_ev_launch test_system_shutdown \
  test_profile_recovery test_alternate_environment
```

## 复现方式

第一次实飞证明新场景中实际接近、获得目标深度并通过多视角确认，未证明近照任务整体成功；距离判据修复后，第二次实飞又暴露文首的报告格式阻断项。先处理此格式问题并受控验证，再依照地面预检和安全链开展获授权测试，不无变化地循环重飞。以下命令保留用于启动，不表示当前应自动再飞。

复现启动命令（当前模型配置仍读取本机.env）：

```bash
# 在仓库根目录执行。
PROJECT_DIR="$(pwd -P)"
env PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}" UAV_NAVIGATION_MODE=static_3d \
  UAV_ENABLE_YAW=1 UAV_ENABLE_NBV_YAW=1 \
  UAV_SEARCH_MODE=nbv_geometry UAV_NBV_SELECTOR=deterministic \
  UAV_NBV_HEADING_MODE=discrete UAV_NBV_AUTO_PREPARE=1 \
  UAV_TARGET_CONFIRMATION=multiview PX4_AGENT_GAZEBO_SEED=17 GUI=true \
  bash "$PROJECT_DIR/scripts/start_system.sh" --control-ui desktop \
  --env-file "$PROJECT_DIR/.env" --ev-profile no_gps \
  --world-file "$PROJECT_DIR/scene/test_environments/courtyard_spacious.world"
```

建议任务：“找到前方的白色货车，靠近后拍照并报告周围环境，完成后返航降落。”车前方向与目标相对降落仍是未实现能力，本轮没有把普通落地当成车前落地。
