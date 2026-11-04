# 跨场景搜索可靠性处理

## 2026-11-04：静态路径通过、最终场景边界拒绝

庭院种子17、qwen3.7-plus 对照实例 `20261104-135225-X40sM7` / 指令 `5b21c48e4f394d22`：6份有效报告、5次实际NBV换位、17次模型调用。两次双图复核均unconfirmed，当前框分别指向货柜/背景建筑，未错误确认。随后候选 `view_123` 的提交被 `flight path leaves this scene's permitted flight region` 拒绝，未创建该移动任务，独立降落 `281470b1b1dd` succeeded，实际落地上锁。第7个perception条目是错误记录，不能统计为第7份有效观察。NBV/最终确认审计均未整体通过，失败保留。

该候选 ENU `[-3.570872,7.725922,3.614780]` 在固定配准下映射世界x约-3.345747，距许可下界-3.4仅约0.054 m。使用末张照片机体姿态和邻近Gazebo状态回放，同一目标世界x约-3.867063，越界约0.467 m。两条来源时间差0.060 s，**这不是被拒绝调用的精确同帧输入，也不能替代实时0.05 s门禁**；用于支持预检与提交使用不同边界依据的诊断。不能由此宣称VIO漂移根因已查明。

源码机制明确：`StaticViewpointEvaluator` 用固定架次注册规划；`Controller.submit()` 还调用 `validate_scene_steps()`，以新鲜对齐的世界/ENU姿态检查许可区域。原票据签发前缺少后者。本次最小修复在 `Controller.evaluate_viewpoints(operation='paths')` 中、签发票据之前对完整折线路径及精确终点调用相同检查；仅 `PathUnavailable` 变成候选不可达，规划器可选择其他候选。起点非法、过期/不齐的位姿和其他状态错误仍中止；最终submit仍再验，票据/epoch/地图注册不变，没有扩大边界或飞行重试。

新增 `executor_scene_path_precheck` 诊断阶段保存实际源位姿、时间戳、所用世界姿态、固定身份、边界与逐候选结果；仍隐藏执行票据。静态几何原始查询独立记录，既有回放不被覆盖。受控测试覆盖边界不一致时只拒绝坏候选、不给坏候选票据、保留有效替代、检查中间路径点、过期/非法起点不签票、签票后条件变化仍在提交时拒绝。

证据 `logs/setup/courtyard-qwen37-regression-20261104/`（summary、boundary-analysis、原始结果、失败审计、现场截图）以及 `logs/setup/nbv-scene-precheck-20261104/`（受控日志）。首轮专项中的新夹具跨用任务身份、错误测试模块名已修正，失败记录保留。

### 修复后真实复测

新实例 `20261104-140519-VYwWsZ` / 指令 `34e6f5aec9e24773`，同场景/种子17/qwen3.7-plus/几何NBV/确定性选择/多视角确认。自然语言仍为“找到前方的白色货车，调整到合适的角度拍摄并报告周围环境，完成后返航降落。”未指定航点、替换模型结果或录像。

- 8份有效照片/报告/地图更新，7次实际NBV换位，21次模型调用。3次双图复核均unconfirmed：前两次只有clipped_region_hypothesis，最后一次当前视角目标被遮挡。没有降低确认规则。
- 最终 `search_status=budget_exhausted / target_confirmed=false`。应用succeeded仅表示正常结束执行流程，**不表示完成“确认目标”的要求**；最终回复明确写未确认、预算用尽。
- 程序返航降落 `366a2084016b` succeeded，实际armed=false、landed_state=1，最终距home约0.102 m，EV healthy=true，无最终坐标重置锁存。没有独立故障原地降落或模型提前收尾纠正。
- 真实签票前检查11批、44个候选均通过，无检查错误；本次没有自然触发候选边界拒绝，**拒绝后改选分支仅受控验证**，不能把“未再失败”当作原失败输入被真实重现并纠正。
- 三次旋转完成仿真时刻144.840/165.436/244.240 s；随后左右图分别同为145.052/165.452/244.252 s，均严格新于旋转完成，两个JPEG哈希分别核验。NBV、地图/照片回放和确认审计通过；审计不证明类别及实例对应为独立真值。

证据 `logs/setup/courtyard-scene-precheck-20261104/`：`summary.json`、`rotation-timing.json`、`run/result.json`、`states.jsonl`、`final-state.json`、`nbv-audit/nbv-evidence-audit.json`、`confirmation-audit/confirmation-audit.json`。`run/04-live-camera.png`、`05-nbv-preview.png`、`08-multiview-review.png`、`10-terminal-grounded.png` 均为实际桌面窗口截图；八张原图副本已核对报告哈希。一次误用单候选旋转专用审计脚本读取桌面结果时产生KeyError，记录 `yaw-audit.log` 保留；没有宣称该脚本通过，上述普通NBV审计和直接双目时间/哈希检查使用正确的数据契约。

实例关闭且实际参数恢复success=true/actual==target，VIO配置恢复启动前副本。最终相关联合回归 **396项通过、0失败/错误/跳过，80.651秒**，日志 `logs/setup/nbv-scene-precheck-20261104/regression-final.log`，命令：

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 PYTHONPATH=.:tests:scripts \
  .venv/bin/python -X faulthandler -m unittest \
  test_photo_navigation test_photo_task test_single_vlm test_agent_reliability \
  test_agent_tool_schema test_task_memory test_web_server test_search_termination \
  test_nbv_search_integration test_target_confirmation test_search_graph_integration \
  test_autonomous_search test_desktop_search_data test_desktop_workbench test_desktop \
  test_bridge_static_navigation test_recovery_ownership test_bridge_camera test_ev_heading_guard \
  test_static_navigation test_scene_config test_viewpoint_planner test_viewpoint_authority \
  test_search_diagnostics test_altitude_limit
```

剩余限制：qwen3.7-plus在局部/遮挡视角中有粗框、错误框及跨帧框近似重复；确认守卫阻止了直接成功，但搜索仍未获得可靠深度关联。不能仅凭这两轮改阈值、改模型或重写NBV评分。保留原图、框和复核，后续应先做小范围真实图像的定位输入诊断。

## 已知失败与证据边界

2026-11-03 obstacle_yard 两次任务均安全落地上锁，任务失败原因分别是复核参数一致性检查和搜索未完成时提前常规返航；不是新的航向重置。原始证据保存在本机忽略目录 `logs/setup/alternate-scene-20261103/`。

油桶任务只有 2 份有效报告、1 次实际 NBV 换位。源图左下角蓝色局部不足以可靠辨认油桶，第二图无目标；复核未通过。旧审计只保存参数名，无法从现有记录唯一还原错误字段。不能把新增诊断当成已查明历史唯一根因。

货车任务有 7 份有效报告、6 次实际 NBV 换位。搜索状态仍为 exploring，预算为 8 份观察，下一动作是 observe_target_search_view；模型却提交 return_home+land。第 7 张票据未执行，不能把 7 批候选当作 7 次换位。

## 本轮修改

- `autonomous_search.py` 的复核校验输出具体字段代码：源/当前照片不匹配、非布尔值、支持与不确定同时成立、理由无效、阶段无效。拒绝时不改变候选或确认等级。
- `single_vlm.py` 在模型调用审计中记录复核参数白名单（原始布尔值、照片/候选 ID、限长描述和理由），在工具 schema 转换前保存；不记录其他工具的任意参数、图像载荷或配置。工具说明明确 supported 与 uncertain 互斥。
- 单次提前返航纠正增加有实际失败证据的探索场景：multiview、exploring、已绑定坐标、已有真实观察、选中 NBV 且有待执行 goto；仍只识别无附加字段的 return_home+land。原有任务所有权、成功的真实上一任务、坐标/地图有效性、操作员中断与故障检查全部保留。纠正返回状态机 next_tool，不提交飞行、不消耗票据、不重置地图或预算；重复错误继续失败并安全收尾。

## 范围与选点的离线检查

离线世界文件中的模型原点距名义起点：obstacle_yard 白色货车约 16.92 m、蓝桶约 16.80 m，均在当前 12 m 地面覆盖地图外。结合该轮实际配准，距离约 17 m。courtyard_delivery 的白色货车约 9.34 m，适合保持原边界进行对照。courtyard 的蓝桶也在边界外，不能笼统宣布整个场景所有目标都适配。

这些是独立评估信息，不传给 Agent、语义地图或选点器。模型原点位于边界外不代表不可从边界内看见；位于边界内也不保证无遮挡或可辨认。

货车失败轮次选中候选的预计新增地面覆盖为 39.25、17.75、9.75、5.75、3、1、0.25 m²，最后一项未执行；语义收益均为 0。实际高度从约 3 m 降至约 1–1.5 m，覆盖率最终 62.7%。这些证据说明局部地面覆盖收益递减，不证明目标区域已经得到充分识别，也不证明第 8 次观察必然失败。

先在范围匹配的白色货车任务上复测当前流程。不要从两次不匹配任务直接推导新的高度下限、识别概率或评分权重；也不要把负的启发式总分当作不可达/未发现目标的证明。后续若匹配场景仍反复出现低空局部覆盖，应根据真实候选的高度、遮挡、像素尺度与成功观察对照，再引入可验证的观察质量指标。当前评分算法未修改。

## 验证记录

首轮相关回归 83 项有 2 项失败：旧错误文本断言需更新、新增脚本模型测试误从末尾系统消息读取工具结果。修正测试后两项单独复测通过；原始失败日志保留 `/tmp/px4-search-reliability-tests.log`。新增复核冲突测试确认拒绝不修改候选；新增探索纠正测试确认原票据不被拒绝计划消耗、后续实际执行一次换位并保存第二份报告。联合回归及真实运行结果见 PROJECT_HANDOFF.md 最新条目。

真实匹配场景复测：courtyard_delivery、种子 17、几何 NBV、确定性选点、多视角确认，任务 abf8f62ff43047e0 succeeded。6 份报告、5 次执行换位，两次复核分别 unconfirmed/confirmed，最终 stereo_unique；实际落地上锁。NBV、地图回放与确认审计通过，复核参数原始白名单已记录。没有自然提前返航，纠正分支仍仅受控覆盖。本次证据不支持立即提高最低高度或重写 NBV；保留当前评分，后续换场景应先完成离线范围与可观察性检查。真实数据保存在 `logs/setup/search-reliability-20261103/courtyard/`，不录视频。
