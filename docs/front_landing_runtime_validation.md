# 车前降落与单进程原生崩溃验收（2026-11-06）

本页记录完整任务重构之后的真实验收，分别记录实际落点、降落触发原因和程序验收。最新staged实飞的物理落点在货车前方，已经达到原句要求的可见结果；程序则因模型超时走安全降落，仍记录failed/partial，正常目标相对落地流程未完成自动验收。两种事实同时保留。受控替身不能替代真实飞行证据。当前仍保留默认 `legacy`；每次独立启动时显式选择运行时，空中不切换。

## 结果表述修正：实际已落在货车前方

用户在观看完整运行后指出，最新staged任务实际上已降落在货车前方。事后核对末张真实原图 `4ba34b764db04b8faceaf4995cb9eb21.jpg`，相机正对货车车头；其机体ENU位置为 `(10.6155, -5.3944, 1.5014)`，实际落地位置为 `(10.5535, -5.4480, 0.0812)`，水平差仅约 **0.0819米**。结合用户现场观察，支持“飞到货车前方，并在该处完成降落”的物理结果。没有用Gazebo目标真值向Agent补充语义答案。

此前直接称“车前降落未成功”过于绝对，现修正为：**物理落点效果达到；程序自主完成判定及正常目标相对降落链尚未验收通过。** 日志中降落任务 `259152eec707` 的来源仍是模型超时后的独立安全收尾，0次GoalPose/目标相对落地许可表示相应控制分支未执行，不代表飞机没有落在车前。原始任务结果、超时记录和审计失败不改写。

## 真实任务与结果

原句均为 **“找到白色货车后降落在货车前方。”**，未改写为普通搜索。世界为 `scene/test_environments/courtyard_spacious.world`，实际 Gazebo 种子17，world SHA256为 `b624fb8ceabc2d56b8979b3a792678271bbc5165f9bcfb5ce627079b1d324f6a`。实际模型为本机配置的 `qwen3.8-flash`。几何NBV、确定性选点、离散航向和多视角确认；30米/20米/12次观察/900秒限制及全部安全门禁保留。

| 模式与实例 | 实际覆盖 | 任务结果与收尾 |
| --- | --- | --- |
| legacy：`20261106-082056-d9UvoJ`，指令 `e7de7ad554004e29` | 0模型请求、0飞行提交、0采图 | 起飞前能力匹配拒绝目标相对降落。没有把原句静默降级；这不是legacy实飞通过。地面上锁。 |
| staged：`20261106-084148-wAX3Qo`，指令 `32917bb4bc094572` | 真实起飞；6份有效报告/地图；5次实际NBV换位；13次任务模型请求；4轮复核 | 最后一轮 `single_depth_projection` 确认当前候选，但与父阶段最初候选的身份仍未建立。身份守卫拒绝替换；0次GoalPose、0次目标相对降落。独立原地降落 `4e4faaec0d21` 成功，实际地面上锁，EV健康；距home约6.54米，不能称返航。 |
| staged修复后首次复测：`20261106-092834-KpK8CY`，指令 `1f46f8f30c7b421b` | 真实起飞；1次实际双目采图、0份有效报告、0次NBV；3次任务模型请求 | 模型检测框越界或顺序无效，报告拒绝；没有覆盖新的原始候选复核分支。独立原地降落 `6076719be977` 成功，实际地面上锁。降落期间出现本地速度重置，最终EV unhealthy，不能称全程导航健康。 |
| staged最终复测：`20261106-100642-rAxg9s`，指令 `57b6751dedb240bf` | 真实起飞；8对原始双目图、7有效报告/地图更新、7次实际NBV；19次任务模型请求 | 实际落点在货车前方。原始候选身份复核真实触发，父引用 `bearing_002` 接续真实米制证据；三次对象前后取证未建立程序要求的对象轴。第8图模型请求在45.196秒超时；程序记录 `failed/partial`，0次GoalPose/目标相对落地许可。独立降落 `259152eec707` 成功，地面上锁、EV健康，距home约11.879米；物理效果达到，正常目标相对降落链未验收通过。 |

第二次staged的原始诊断仅保存了异常类型、消息、照片ID和调用栈，没有保存具体错误框数值。因此不能断言它究竟是像素坐标、大小顺序还是另一种具体格式错误。不能自动夹紧框、交换端点或补造视觉证据。

## 本轮最小修改

1. `task_runtime/orchestrator.py`：在父任务仍处于原候选的观察阶段、没有执行GoalPose、原候选没有米制定位，而最新确认候选已有米制证据时，安排一次对**最初交接照片与当前真实照片**的既有严格双图复核。复核成功才建立身份链接；仅标签相同、弱视线相容或源图不确定不能授权终点。保留owner、原图hash、原候选、地图、预算和坐标身份，不重启搜索。
2. `single_vlm.py`：staged原图缓存沿用父任务12张/900秒上限，避免原始交接照片被legacy的最近两张缓存淘汰。legacy缓存默认不变。历史证据保留时间与新鲜采图门禁分开；没有让旧图成为新观察。
3. `single_vlm.py::VisualTarget`：把检测框的0～1范围及端点顺序校验前移至工具入参。JSON Schema也包含范围约束。staged首个未执行格式错误可使用已有的**唯一共享纠正额度**，同帧重新报告；第二次错误仍失败安全收尾。没有增加额度、自动重拍或飞行重试。
4. `tests/test_task_goal_graph.py`、`tests/test_task_runtime.py`：增加原始方位候选与后续米制图的身份交接、不确定源图拒绝、复核期间epoch失效、原图按既有预算保留，以及错误框同帧纠正/共享额度耗尽的图级用例。`test_nbv_search_integration.py`仅增加可选受控夹具超时参数，产品任务预算未变。
5. `tests/test_single_vlm.py`、`tests/test_search_graph_integration.py`：原有报告失败保护仍验证。框校验提前后，legacy可在提交下一轮模型调用前直接结束并独立降落；报告体来源错误与报告保存失败仍有独立覆盖。

前三项是实际任务阻断修复，**不是原生崩溃修复**。原始候选复核的正向受控用例验证到身份交接，不代表车头识别或实际降落已通过。

最终复测的原始引用双图为 `8c53c1bd723344bbac14b502e2896ff6` → `7e30777e129a468bb8eaec10b6092570`。此时已有当前图与较近源图的一轮确认，程序再复核最初交接图，未新增照片或扩大预算。工具记录双方支持、同一物体及 `single_depth_projection`，父目标保持原始 `bearing_002`。其后当前图/新图又返回 `stereo_unique` 确认。这些是实际状态机/工具返回结果，独立总体确认审计尚未通过，见下文。

最终8对原始照片哈希已核验，坐标身份均为VIO epoch0、EKF xy/z/heading2/2/2。两次实际旋转达到设定值的时刻分别为235.484、265.084仿真秒，随后左右帧分别为235.700、265.100秒，均晚于完成时刻且左右同步。没有用ROI派生图增加采图计数。第8图已真实采集，但模型超时，没有有效报告/地图更新，故不是第8次有效观察。

最终父结果明确 `process_status=failed`、`goal_status=partial`、完成stage-01、车前stage-02尚未被程序判定完成；`user_goal_completed=false`。这些是软件自动判定，不能用来否定实际已在车前落地的外部观察。目标视觉/关联、物理落点、实际上锁和程序完成授权分别记录。三次 `report_target_frame` 没有建立可用对象轴；目前原记录未单独保存所有拒绝分支的原因，不能精确断言都是车尾遮挡、深度不足或轴方差过大中的哪一项。后续应使系统能用足够的真实证据判断车前到位并正常收尾。

## 单进程原生崩溃：已经定位的事实与边界

- 找到了11月3日原混合回归的真实core。其进程命令来自本项目；从core实际保存的ELF段读取首个页，与当前Python解释器首个页逐字节一致，包含相同GNU build ID `cdbfe905913cada63b97850bdeff2f6e02f92ae9`。不能将GDB的通用文件名警告直接当作版本不符。
- 实际故障线程在 `_PyFrame_Clear`。来源函数为 `stereo_localization.vector`，帧 `stacktop=-6`，`f_code` 已变成 `list_iterator`，`f_locals` 变成 `list`。复制残损帧后访问无效代码指针导致SIGSEGV。调用栈没有足够证据定位首次写坏者；`vector`是暴露位置，不能据此改写数学函数掩盖问题。
- 当前环境为CPython3.11.17/Clang23.1.3、PySide/Shiboken6.6.3.1。单进程Qt布局与Agent路径压力测试再次出现过真实SIGSEGV；其他轮次出现不应发生的参数数量/上下文管理器类型异常。它们与帧损坏是否同源尚未证明。
- 普通GDB整套回归多次通过；低GC阈值压力测试有通过也有失败。隔离构建同版带断言CPython/GCC、复用相同扩展，803项通过且未触发断言；**这不证明Clang错误，也不构成更换正式解释器的依据**。
- 旧Valgrind对当前glibc/CPython产生大量加载器/整数填充告警，并导致两个15秒夹具超时，没有给出可信的首次非法写。不能用这些告警宣称找到Qt越界。
- Qt全局import/frame检查、GC、原生扩展与解释器构建仍是待隔离因素。没有证据支持随意升级依赖、删除Qt组件、关闭GC或修改飞行逻辑。本轮未做这些修改。

**状态：原生故障已取得可核验的C栈和损坏帧证据，尚无经过因果复现验证的根治修复。单次或多次回归通过不能将其关闭。** 实际四次模式尝试的Agent进程均没有原生退出；任务失败分别来自能力匹配、实例身份保护、报告格式及模型请求超时，不能混为SIGSEGV。

## 验证记录

本机忽略目录 `logs/setup/frontlanding-crash-20261106/` 保留所有成功与失败。早期夹具修改产生的失败、两个旧断言失败、一次沙箱完整804项中的5个端口权限错误均保留；端口权限错误不能当作产品逻辑已通过。

已完成的专项：原始引用图链42项通过；缓存与单模型70项通过；最终框校验/legacy保护联合72项通过。最终实际主机单进程联合回归 **806项通过，165.667秒，0失败/错误/跳过**，原始记录 `regression-final.log`，对应 `final-source-sha256.json`。没有因Qt风险跳过界面用例，也没有将进程拆分结果算成单进程通过；这轮未复现崩溃不代表根治。

完整混合回归使用同一进程加载下列模块，包含Qt、Agent及执行器受控用例；HTTP测试需要本机回环端口权限：

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 UAV_TASK_RUNTIME=legacy \
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}" PYTHONPATH=.:tests:scripts \
.venv/bin/python -X faulthandler -m unittest -v \
  test_task_runtime test_task_runtime_bridge test_task_association test_task_reasoning \
  test_goal_pose test_goal_executor test_goal_landing_executor test_task_goal_graph \
  test_task_landing_graph test_task_route test_task_photography test_task_desktop \
  test_flat_landing test_stereo_reports test_spacious_search test_target_observation \
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
  test_forest_environment test_orchard_environment test_ev_launch test_system_shutdown \
  test_profile_recovery test_alternate_environment test_gazebo_seed_evidence \
  test_takeoff_heading_alignment test_ev_fault_safety test_ev_health test_ev_preflight \
  test_production_telemetry test_static_search_permission test_static_search_volume \
  test_search_tools test_nbv_evidence_audit test_semantic_evidence_audit \
  test_confirmation_retry_audit test_nbv_audit test_stereo_evidence_archive
```

原staged四轮复核的确认审计通过。地图在线/回放一致，5张票据与实际移动可配对；但NBV总体审计仍为失败（整项任务失败及父阶段准备来源适配等），不能称总体NBV验收通过。

**最终复测的审计未全部通过，保留失败**：确认审计在原始引用的第二次同capture复核处报 `Current candidates differ from saved visual evidence`。每张照片的不可变map快照保存于报告时，第一轮复核已将当前目标ID链接到源候选，第二轮又链接到父最初引用；旧审计只读取报告时快照，未重放这段连续别名变化，最终审计只列出5轮而原记录有6轮。必须补足独立时间顺序重放后才能关闭此项，不能直接忽略ID检查。NBV审计核对了前6次后续报告，但最后第7次移动后没有有效报告，旧脚本报 `list index out of range`；总体失败。另有7份实际报告可离线重建覆盖图，**没有由此声称在线地图/身份完全一致**。`staged-final-evidence.json`只是8对图片哈希、2次旋转新帧和实际任务结果的补充核验，不能替代上述整体审计。

最终地面接口预检中，SDK诊断先绑定tool再做image会污染同一诊断客户端：图像请求仍带强制只读tool，得到调用而非描述，原失败保留。随后用新客户端单独image预检通过，才发送任务；text/tool也各通过。该问题在只读诊断器 `ReadOnlySDKModel.bind_tools()`，本轮未混入Agent或改写模型输出，记录待维护。

## 本机证据与下一步

- `legacy-ui/`、`staged-ui/`、`staged-rereview-ui/`、`staged-final-ui/`：原指令结果与实际Qt窗口关键帧，无录像；最后一项有11张实际窗口关键帧。
- `*-ground.json`、`*-seed.json`、`*-states.jsonl`、`*-final.json`：预检、真实种子、飞行采样与落地状态；部分final文件使用 `{wall_s, state}` 包装。
- 对应 `logs/system/<instance>/captures/`：原始双JPEG、采集元数据、报告、地图、NBV和局部异常诊断。没有提交这些文件。
- `original-to-metric-geometry.json`：第一次staged的原图与最后米制图离线具备单侧深度投影相容性；不是实际模型已完成两图身份复核。
- `historical-elf-match.json`、`historical-core-stack.log`、`historical-frame-metadata.log`、`historical-function-metadata.log`：原生证据。core在私有目录，含进程内存，禁止提交或上传。诊断钩子的主动空指针自测文件为合成故障，不能冒充项目自然崩溃。
- 四实例均通过生命周期脚本退出，PX4参数按实际读回恢复；VIO只恢复该次生成的两个输出路径，逐字节核对启动前副本。最终无本次仿真、桌面、诊断/取证进程残留。没有更改融合/重置/碰撞/票据/操作员保护，没有自动commit或push。

下一步应为车头/前后部取证补齐具体缺失原因，并使原始引用的连续复核能被独立审计按时间重放，然后再验证GoalPose到位、新图关系复核及平地下降；不凭后续同类物体的标签替换已交接身份。不继续无变化重复整句实飞。原生问题的下一项是捕获首次破坏帧的写入或取得可稳定复现的最小路径，再实施对应修复；不再以单次混跑通过或进程拆分宣告根治。
