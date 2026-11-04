# 普通概览拍摄闭环


## 能力透传与上下文精简后的真实复测（2026-11-04）

本节为最新状态，取代下方历史条目中“两项修复尚未重飞”的时态；历史失败与只读对照保留。没有修改生产飞行策略、参数、提示词或确认门禁。

- 平地果园、种子17、qwen3.7-plus、20 m上限，任务原句为“请为我拍摄一张俯瞰整个果园的照片。”；实例 `20261104-161436-MH0L7G`，指令 `47927960491b4e97`。
- 实际起飞8 m (`9a79398e8cbd`) → 右5 m/12 m (`c03407228ea5`) → 后退8 m/15 m候选被路径门禁拒绝且未执行 → 右6 m/14 m (`985cd5d8da07`) → 右5 m/16 m (`4db596c4cf76`)。
- 共4份真实图像/报告/复核、3次实际调整、18次模型调用、0转向。最终 `budget_exhausted/failed`；实际末图底部和近左侧仍裁树，不满足完整果园，不能以报告成功或照片数量称任务成功。
- 程序返航降落 `934749377e1d` 实际执行约15.99 m规划返航路径并落地上锁；实际MAVROS心跳及ExtendedState确认 `armed=false/landed_state=1`。终态ENU水平距home约0.070 m，这是估计位置距离；降落末段发生已知速度重置，最终EV unhealthy且锁存，不能把该距离当独立真值精度或全程定位健康证明。保留全部记录，没有忽略重置后继续导航。
- 运行已关闭，PX4恢复结果success且actual==target，VIO YAML与本轮启动前副本一致，无残留本轮进程；不录像。

4个规划SDK请求全部恢复原JPEG后核验原始参数SHA256、配对响应与0600权限。每份请求均包含 `rotate_photo_view` 及真实 `experimental_yaw_enabled=true`；427份实际遥测也均为true。模型看到的所有普通拍摄报告ToolMessage均不含 `vlm_calls`，4份权威保存报告全部保留该审计；最新原图输入与复核capture一致。**能力透传和上下文精简已实飞验证，但全园构图没有改善到达标。**不能继续将本轮不转向归因于工具未开放，也不能仅因没有旋转就断言旋转一定能拍全。

还检查了四帧已保存的方向反馈：原朝向的画内检测框均保留9/9条采样射线，左右转向只有3～8/9。检测框描述当前看见的部分，而不是完整果园；因此“保留已知框射线数”天然偏向当前朝向，不能作为找回画外主体的收益或完整构图评分。这是现有辅助数据的能力边界，不是投影公式错误，也未证明它是模型不转向的唯一原因。下一步用已有真实输入核查该信息差，再验证决策改动；不要继续无变化重飞、强制转向、套用地面NBV覆盖率或把离线Gazebo真值注入任务。

本轮源码哈希与此前最终429项回归相同；本轮另外重跑命令 `QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 PYTHONPATH=.:tests:scripts .venv/bin/python -m unittest test_photo_navigation test_photo_task`，49项通过，0失败/错误/跳过，1.971秒。没有再次运行429项联合。文字、工具和真实相机预检全部通过。一次离线请求审计在任务尚运行时被脚本终态断言拒绝，未调用模型或飞行；实际落地后重新审计通过，没有把该早期拒绝算飞行失败。

完整证据目录：`logs/setup/orchard-capability-context-20261104/`。

- `summary.json`、`run/result.json`、`states.jsonl`、`final-state.json`：任务、各飞行结果与真实状态。
- `sdk-requests/`、`request-evidence-audit.json`、`audit_requests.py`：实际请求和只读完整性/能力/审计分离核验。
- `bearing-feedback-audit.json`：四帧已知框射线计数及其限制。
- `source-snapshot/`、`source-sha256.json`、`preflight.json`、模型预检、`cleanup-verification.json`：版本、预检与恢复。
- `photo-01.jpg`～`photo-04.jpg`：左右原图哈希均已核验，左图便利副本；完整原始双目仍在实例captures。
- `run/04-first-observation.png`、`04-live-camera.png`、`07-second-observation.png`、`09-return-or-land.png`、`10-terminal-grounded.png`：实际Qt窗口截图，终态截图已人工检查可读且明确显示失败、落地上锁。


## 实际规划请求取证（2026-11-04，可选诊断）

新增 `photo_request_evidence.py`，仅在启动进程显式设置 `UAV_PHOTO_REQUEST_EVIDENCE_DIR` 时使用取证模型类；默认仍使用原ChatOpenAI。保存目录限定为本仓库忽略的 `logs/` 下，仅取景规划回合启用，观察报告/独立构图复核/其他任务不记录。不会执行诊断中返回的工具或改变模型参数、飞行授权、终态判据与安全收尾。

记录点是本机LangChain已完成消息与工具转换后、调用OpenAI兼容SDK之前的 `_get_request_payload()` 返回值。包含真实system/messages、当轮工具schema、模型与生成参数，配对实际模型响应和请求ID。它是**SDK参数取证，不是HTTP抓包**，不记录请求头、地址、代理配置或凭据，也不声称保存了供应商内部状态或每次网络尝试。

JPEG数据URL替换为SHA256引用，图像仍来自原有采图证据目录。请求记录包括原始参数的规范JSON哈希；只有无其他脱敏变更时才标记exact_payload_reconstructable，后续必须恢复经哈希核验的原JPEG并核对完整参数哈希后才能称为同参数重放。已知密钥、认证文本、URL凭据、票据及JSON字符串中的凭据字段脱敏；被脱敏的请求不能冒充原样请求。记录文件用独占创建和0600权限，不覆盖历史记录；目录/序列化/磁盘失败只标记诊断不可用，不替代原模型或飞行错误。规划调用审计的context_memory中关联请求ID与取证状态。

`tests/test_photo_request_evidence.py`验证：原参数完全不变、原图哈希重建、密钥/JSON内票据脱敏、默认关闭与嵌套关闭、非法目录与磁盘故障不改变行为、失败模型调用配对、真实Agent中间件只记录规划回合。最终专项101项通过（2.571秒），完整相关联合 **427项通过、0失败/错误/跳过，82.415秒**；命令为下方421项联合命令增加 `test_photo_request_evidence`。证据 `logs/setup/photo-request-evidence-20261104/`。没有新增网页接口，不向界面或Git分发请求内容。

真实取证实例与后续只读对照保存在 `logs/setup/orchard-request-evidence-20261104/`。对照预先限定为首个、最后一个不同可调整观察的首次实际移动决策，各做一次原SDK参数重放及一次仅删除Tool消息JSON中 `vlm_calls` 审计字段的重放。在查看实际请求时另发现旋转能力字段被丢弃，在运行重放前增加一个仅补回实际旋转开关的对照，共6次。保留原系统指令、当前图像、视觉结论、动作历史和其他飞行状态；不执行任何返回工具。温度0仍可能有模型输出变化，不凭一次不同回答宣布因果关系或成功率提升。

### 本次真实任务及取证结果

实例 `20261104-154916-iESEn5` / 指令 `b113ca673d0449d0`，平地果园、种子17、qwen3.7-plus、原句、20 m上限，飞行策略与上轮相同，仅开启取证。8 m起飞 `85157961deac` 后实际完成3次取景调整、4份图像/报告/复核、19次模型调用，0转向；最终仍budget_exhausted/failed。第四张实际照片的近处果树仍被底边裁切，不能称全园成功。程序返航降落 `e43524f5c973` 已实际完成，最终armed=false、landed_state=1，水平距home约0.133 m，EV healthy=true、无最终锁存。实例已按身份关闭、PX4参数读回恢复、VIO YAML恢复启动前副本，无录像。

5个真实规划SDK请求均有配对响应，JPEG引用及恢复后的完整参数SHA256全部核验通过，文件权限均为0600，均无其他脱敏变更。原始文件、当时源码快照、实际照片、状态、预检、返航结果及窗口截图保存在上述目录；校验见 `request-evidence-audit.json`。不把取证前几轮的重建输入称为同参数重放。

本次发现一个确定的数据流缺口：356份实际遥测中的 `experimental_yaw_enabled` 全部为true，但5个规划请求中的精简状态全部没有该字段。原系统指令要求旋转先有执行器显式启用证据。源码中 `compact_flight_evidence()` 丢弃该字段，`get_drone_state(view='flight')` 也会丢弃它；这两个过滤处现已修正，只透传真实true/false，源状态没有时仍缺失，不推断已启用。后端旋转开关和执行检查保持不变。该修复发生在本次飞行结束后，**修复后的版本尚未重飞**。

### 同参数重放与两个最小修正

两份实际输入各做3个profile，共6次只读调用，供应商请求和工具schema校验均通过，0飞行命令：

| 视角 | 原SDK参数重放 | 仅去除重复审计 | 仅补回有实测依据的旋转标志 |
| --- | --- | --- | --- |
| 第一张图 | get_drone_state(flight) | get_drone_state(flight) | 后退5 m并升至12 m |
| 第三张图 | 后退3 m并升至14 m | 后退3 m并升至14 m | 后退3 m并升至14 m |

六次都没有选择转向。第一张原参数重放返回读状态，而真实原始响应为后退，证明即使温度0和完整参数一致，也不能假设响应逐字相同。第三张重放仍提出一个先前类似的越界后退意图；没有执行它，也不因schema通过称路径安全。证据 `replay-manifest.json` / `replay-results.json` / `replay-validation.json`。

仅去审计时，第一份输入token从16,499降至15,483；第三份从39,995降至32,542，减少7,453（约18.6%），返回动作与相应原参数重放一致。不能据此声称已改善构图或稳定降低耗时，但能证明重复审计占用上下文。因此第二个最小修正是普通拍摄的 `report_scene` **模型ToolMessage**不再重复携带累计 `vlm_calls`；权威inspect_scene报告和应用审计保留完整字段，照片/框/视觉结论/限制/方向反馈/任务进度和实际动作历史均保留。其他非拍摄报告不变。

新增图测试验证all/flight视图到模型的true、false及缺失语义，另验证精简模型回复与完整保存报告分离。两项修正后的专项115项通过（2.878秒，0失败/错误/跳过）；最终联合429项通过，0失败/错误/跳过，82.765秒，日志 `logs/setup/photo-request-evidence-20261104/final-regression.log`。旧427项对应取证接入，428项对应仅加能力透传，不冒充两项修正后的最终版本。

结论：修正了真实能力信息丢失，并减少冗余输入；**尚未证明二者能解决整体取景问题**。下一次真实任务应继续开启取证，首先核对该开关确实出现在模型状态、重复审计确实退出模型回复，再独立评价照片和行为。不要继续将未复现的转向选择当成功，不扩大预算/范围、不增加猜测深度、不硬编码果园视点。

## 已观察视线的转向反馈（2026-11-04，最新）

进一步核查最近4张果园报告：所有“果树”大框的双目定位均为unavailable（有效像素或一致性不足），不能将代表点/猜测深度当作果园范围来预测平移和升高。因此本轮只提供**视线方向**反馈，不建立虚构的三维主体。

新增 `photo_framing.py`：由实际report_scene检测框、左相机标定及同步ENU机体姿态，计算原方向和左右15°/30°/45°时已观察框的9条采样视线如何投影到归一化画面。它是远场方向近似，忽略机身旋转时相机光心杆臂平移及实际姿态收敛；不预测画外区域、遮挡、新图内容、整体覆盖或飞行安全。检测框本身仍是未校准的模型输出，可能包含背景或遗漏主体。没有深度时不给平移/升降结果，有一个双目表面点也不能当作完整物体范围。

`single_vlm.py`在普通拍摄报告保存时计算并审计反馈，`PhotoTask`仅保留当前最新反馈，在已核验原图的取景规划请求中附带。构图复核仍隔离，只看用户原句和当前原图，不接收此反馈。没有新增工具、候选ID接口或飞行权限，不强制选择转向。缺失标定、错误框、不同照片/hash、时刻不一致、非有限投影等返回unavailable，不伪造几何，也不把可选诊断缺失升级为飞行故障。已有图像归属、hash、TTL、epoch及动作执行检查保持不变。

在两张归档图上的额外2次只读模型调用中，首图仍选升高，第三图选择右转30°并引用方向计算。不能据此声称泛化或整园构图已成功；它只为本次最小接入和一次真实对照提供依据。生产计算与该两图诊断数值逐项核对一致，证据 `logs/setup/photo-bearing-feedback-20261104/archived-geometry-check.json`，原始调用在 `logs/setup/photo-choice-diagnosis-20261104/bearing-*.json`。连同上一节的12次只读实验，共14次；新增2次接口校验均通过，0飞行命令。

新增 `tests/test_photo_framing.py`，增强 `tests/test_photo_task.py`，覆盖左右方向、全局航向/±π不变性、居中框不偏向右转、无深度不编造位置、无效标定/同步/身份/数值拒绝、保存报告到规划的读取及复核隔离。初轮95项有1失败：反馈误写入 `FrameStore.get()` 返回的副本，后续规划读不到；已改为当前PhotoTask保存最新反馈，不改变帧缓存所有权。最终专项95项通过（2.312秒）；补充非有限值检查后最终联合 **421项通过、0失败/错误/跳过，82.458秒**。Python3.8编译及diff检查通过。日志 `logs/setup/photo-bearing-feedback-20261104/` 保留初次失败、中间421项及最终结果，不能混算次数。

最终联合回归命令：

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 PYTHONPATH=.:tests:scripts \
  .venv/bin/python -X faulthandler -m unittest \
  test_photo_framing test_photo_task test_photo_navigation test_single_vlm \
  test_agent_reliability test_agent_tool_schema test_task_memory test_web_server \
  test_search_termination test_nbv_search_integration test_target_confirmation \
  test_search_graph_integration test_autonomous_search test_desktop_search_data \
  test_desktop_workbench test_desktop test_bridge_static_navigation \
  test_recovery_ownership test_bridge_camera test_ev_heading_guard test_static_navigation \
  test_scene_config test_viewpoint_planner test_viewpoint_authority test_search_diagnostics \
  test_altitude_limit test_yaw_control
```

### 真实果园结果：数据链通过，完整构图未通过

实例 `20261104-153135-AIp3hL`，指令 `a9c79d922af143f5`，仍为平地果园/种子17/qwen3.7-plus/原句任务/20 m上限/4次观察预算。文本、工具、实际相机接口和连续10次地面预检通过，真实世界哈希与前次一致。

实际执行8 m起飞 `48eb0ec72b0d` → 右移5 m并升至12 m `60867e2ec409` → 一次后退8 m、18 m高度的越界候选未提交 → 改选原地升至15 m `28df2582e3d5` → 右移6 m保持15 m `cf36ef71e2c7`。4份照片、报告、复核；22次模型调用；3次实际取景调整、0转向、0 NBV。四份报告的方向反馈均为ok，8个规划调用均使用对应最新已复核原图。反馈保存与规划注入由该版源码/受控测试及报告、图像调用审计关联；未保存完整供应商请求，不能把这些记录当作逐字请求重放。

第四张真实照片仍裁切近处果树，不能验收为整个果园。模型也判no，photo_status=budget_exhausted、应用failed，未放宽判据或延长预算。离线简化输入中出现的右转选择没有在本次连续任务复现，**不宣称新反馈解决了动作选择或改善了整体成功率**。详细边缘描述仍含模型误判可能，不能将其当作独立标注。

程序返航降落 `9a509a307fec` succeeded，从距home约11 m处返航，实际armed=false、landed_state=1，最终水平距home约0.0462 m；photo_return=ok，safety_landing=null。接触阶段短暂EV融合告警原样保留，最终healthy=true、reason=null、无最终重置锁存；没有开展新的触地异常修复。实例已按身份关闭、实际参数读回恢复、VIO YAML恢复启动前副本，主机无本轮残留Gazebo/PX4/ROS实例，无录像。

证据目录 `logs/setup/orchard-bearing-feedback-20261104/`：summary.json、run/result.json、states.jsonl、final-state.json、source-snapshot和hash、preflight.json、cleanup-verification.json、4张核验左右hash的原图。实际桌面截图包括 `run/04-first-observation.png`、`04-live-camera.png`、`07-second-observation.png`、`09-return-or-land.png`、`10-terminal-grounded.png`。

下一步不要继续加动作提示或原样重飞。先取证实际规划请求中的当前照片、几何反馈、历史报告和工具结果，区分简化只读请求与完整任务上下文的影响。现有运行未归档完整模型请求，不能仅凭这次差异就认定历史上下文是根因；后续诊断应保存脱敏且可重建的请求证据，原始图像仍仅在本地忽略目录。生产方向反馈目前是有明确限制的辅助信息，尚未证明能改善整园构图；不扩展到NBV评分或确认流程。

## 动作选择只读诊断（2026-11-04，最新；未修改生产策略）

当前方向转换与构图未完成后的健康返航已实现。重新运行 `PYTHONPATH=.:tests:scripts QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 .venv/bin/python -m unittest test_photo_navigation test_photo_task`：47项通过，0失败/错误/跳过，1.448秒。复核最近果园原始终态确认程序返航约5 m、实际落地上锁，距home约0.139 m；这是已有真实记录的复核，**不是本轮新飞行**。

为避免继续重复相同失败飞行，在实例 `20261104-145157-XWjDr5` 的第一、第三张真实照片上完成动作选择对照。用户原句、qwen3.7-plus和真实图像不变，图像SHA256与原报告核验；不连接执行器，所有返回动作仅记录。使用当时最近的归档状态（与照片仿真时间差小于1秒），完整历史模型请求未保存，因此是重建输入诊断，不能称为精确历史重放或性能基准。

| 对照 | 次数 | 结果 |
| --- | ---: | --- |
| 现有提示、去掉调整建议、额外说明各动作作用；两个实际视角 | 6 | 全部返回合法相对动作；第一图均选升高，第三图均选右移，无转向 |
| 固定左右转45°、左右前后移3 m、升5 m/降2 m候选，按归档几何过滤；正常/倒序列表 | 4 | 四次都选升高；3次完整比较，1次遗漏两个候选，校验失败 |
| 保持候选接口与顺序，去掉此前视觉摘要与整个构图复核文本 | 2 | 第一图仍选升高且校验通过；第三图选左移但遗漏候选，校验失败；仍无转向 |

候选来源仅为固定相对动作、归档位姿及已知静态导航几何，没有向模型提供树木网格、目标真值或最佳视点。归档路径/边界检查不是实时执行授权。两次不完整比较均保留原始响应，不补造结果或重新调用掩盖失败。本轮新增后两组6次调用，连同前序6次共12次；10次响应契约通过，2次候选比较不完整。候选接口只是忽略目录中的实验，没有加入正式工具集合。

### 独立几何对照

另外使用哈希核验的原始果树视觉网格、真实相机标定、采图姿态与当架次配准，对返回动作及左右转45°做离线投影。此处网格真值**仅用于评估，未发送至模型或执行器**。

| 归档视角 | 原图视锥内树木顶点占比 | 原接口模型动作后的占比 | 同位置右转45° |
| --- | ---: | ---: | ---: |
| 首图，约5 m | 61.6% | 升高后61.7%～62.8% | 90.1% |
| 第三图，约10 m且右移3 m | 67.1% | 右移/升高后71.9%～72.3% | 81.5% |

这些数值按网格顶点计数，会受网格细分密度影响；不含遮挡，不是图像面积、识别率或实际覆盖成功率。假定保持归档姿态的滚转/俯仰，仅改变位置或航向；所有这些动作都没有使全树网格入镜，不能把右转当作全园拍摄的已验证答案。初版诊断脚本因执行网格加载片段未传入 `__file__` 而失败，保留 `geometry.log`；修正后 `geometry-fixed.log` / `geometry-check.json` 完整生成，未覆盖旧证据。

局部地图覆盖不适合直接替代构图收益：首图左转45°的预计新增地面覆盖约40.5 m²，但树木顶点入镜占比降至11.4%；右转45°新增地面覆盖约20.25 m²，却纳入更多果树。现有查询还使用12 m栅格与10 m地面观察距离，较高视角可能返回0个地面单元。这是任务与表示适用范围差异，本轮未修改NBV覆盖评分或距离阈值。

### 结论与下一步

当前证据不支持将问题归结为方向转换、仅缺少原图输入、候选排列顺序，或单独的上一轮调整建议。模型反复把“转向不改变镜头视场角”混同于“转向不能让更多主体入镜”，即使去除旧视觉结论也未消除错误；这仍是有限样本现象，不是模型总体能力评测。去掉错误复核文本未带来一致改善，不能据此删除正式任务记忆。

冻结现有飞行/返航/图像来源实现，不将本次提示或候选接口直接接入生产，也不再无变化重飞果园。下一步应先设计可由真实图像/已有有效定位支撑的**构图几何反馈**：明确哪些主体范围实际已知、哪些仍在画外未知，再评估候选动作对已观察主体的投影与可能丢失，而不是让模型仅凭语言估计“升高必然拍全”。必须先用保存照片验证反馈是否真实且改变错误选择，不得使用Gazebo目标真值作为在线输入、不硬编码果园视点、不把地面NBV覆盖当整园构图完成；若证据不够，仍保留未完成结果。整个果园照片尚未完成，本轮没有Gazebo飞行或录像。

全部本地证据：`logs/setup/photo-choice-diagnosis-20261104/`，含原始与候选/去文字对照的manifest、results、脚本、日志以及独立投影结果；均位于忽略目录，不随源码提交。当前受控联合回归记录仍为此前413项，本轮未修改生产代码，未重新宣称跑过联合回归。

## 取景决策使用真实观察图像（2026-11-04，当前）

最新实飞审计证明：原取景动作选择只有文字摘要，原图在report_scene和review_photo之外未注入。现于普通拍照adjust阶段将最新已复核照片重新提供给同一模型，保留规划所需的历史、当前阶段及能力信息。与上一节独立构图复核不同，选动作仍需要任务上下文。

- 来源必须是当前指令、最新观察的capture_id，并已实际送达与完成复核；复算原JPEG哈希，与保存报告/帧元数据及epoch逐项核对。缓存过期、错任务、内容/身份不一致会拒绝，不通过重新采图或改纪元掩盖错误。
- 请求明确标记`photo_view_selection`、is_live_image=false、counts_as_new_observation=false，并包含采图时真实机体ENU位姿与时间；仅用于相对取景规划。模型不能把像素方向直接当ENU，真正转换仍由当前新鲜姿态与原执行器完成。
- 平移/旋转成功后必须采集新图。复用图像不新增capture、report、地图观察或预算；不改变4次拍摄上限、候选拒绝额度、飞行权限或原返回/故障处理。
- 图像只在该次模型请求中临时注入，保存的Agent检查点和公开审计不含base64图像；审计通过image_purpose与capture_ids关联真实原图。

修改仅`single_vlm.py`和`tests/test_photo_task.py`。专项87项通过、0失败/错误/跳过（2.289秒），包含正确图像/计数/历史不泄露及内容/epoch/过期/其他所有者反例。证据 `logs/setup/photo-selection-image-20261104/`。联合413项通过、0失败/错误/跳过（82.108秒），完整命令沿用上一轮412项模块集合；新增1项含5种坏图像来源反例，并增强正常流的图像/计数/审计断言。日志 `regression.log`。真实选点效果另按实际结果记录，不能把补齐输入直接称为构图成功。

### 本轮真实验证：输入修复通过，取景效果尚未改善

实例 `20261104-145157-XWjDr5` / 指令 `02bfb851278b49bf`，同平地果园/种子17/qwen3.7-plus，仍为原句。实际5 m起飞，一次后退5 m候选越界拒绝后改为升至10 m，再向右3 m及2 m；共4份有效照片/报告/复核、19次模型调用、3次实际调整、0次转向。5个`photo_view_selection`回合（含一次读状态、一次候选拒绝）全部text+image，capture_id均匹配当时最新已复核观察；原图左右哈希、观察数与阶段审计通过。补齐真实图像输入已在实飞验证，**没有证据表明本次选点效果改善**。

最终照片仍底部/右侧裁树，budget_exhausted、应用failed，如实未拍全。程序return_home+land `d89712e9bde4`实际完成，从离home约5 m处返回，armed=false、landed_state=1，终点水平距home约0.139 m。触地附近出现短暂EV融合告警，最终healthy=true、无重置锁存；未宣称触地问题已修复。实例已关闭、参数读回及VIO配置已恢复，无录像。

证据 `logs/setup/orchard-photo-selection-20261104/summary.json`、`run/result.json`、`states.jsonl`、source-snapshot及hash、四张原图和真实桌面截图。不能将5次图像规划调用计为5次新观察，也不能将受理/规划计为实际移动。

下一步先冻结当前安全/方向/图像来源实现，开展只读动作选择对照：固定真实图、用户原句和当时许可信息，保留模型原始工具参数，比较是否能区分“转向居中”与“平移/升降扩大范围”。对照只是诊断模型或上下文的影响，不执行返回的工具，也不把离线真值视点传给模型。若没有新的可重现根因，不继续重复相同实飞或无依据改提示词；全园构图仍作为未完成事项保留，普通闭环如实收尾不替代其完成。

## 构图复核证据与上下文隔离（2026-11-04，当前）

针对上一轮第四图仍裁切果树却被判met的真实反例，本次只修改普通拍照复核，不改搜索确认、飞控、地图、预算和范围。

- 复核请求仅包含专用系统指令、用户原句、当前已保存且哈希校验的原图/capture_id/相机元数据；去除原飞行系统提示、历史报告、工具执行记录与观察预算。原Agent的检查点和后续规划记忆保留。仍为同一个qwen3.7-plus调用，不是第二个独立模型或真值裁判。
- `review_photo`要求top/right/bottom/left逐边缘的clear/cut/uncertain与具体证据。任意cut使有效完整性为no；任意uncertain不能与总体yes共同产生met；四边clear不会将未知整体范围自动提升为完整。保存模型原始总体判断、有效判断与evidence_conflict，冲突按原预算继续调整或结束，不伪装成飞行故障。
- 复核必须有本指令该图的photo_framing_review实际送达记录；只允许当前最新有效报告。模型复核回合不暴露飞行工具，真实操作员停止/降落与独立安全监控继续走原链路。

只读诊断使用两张真实果园反例及一张林地完整帐篷图，冻结提示词调用3次：果园均no、帐篷yes。但模型仍错称果园左边缘裁切；这些样本不是独立留出集，**不构成准确率或泛化提升的证明**。生产工具schema另做一次只读调用，返回可验证的四边结构；同一响应修正诊断脚本误验证隐藏runtime字段后通过，无重复模型调用。全部本地证据在 `logs/setup/photo-framing-evidence-20261104/`，未发送场景模型名或真值坐标，无飞行。

源码：`photo_task.py`、`single_vlm.py`；回归：`tests/test_photo_task.py`。最终联合412项通过、0失败/错误/跳过，81.433秒，日志 `regression.log`。命令为上一节408项相同模块组合，新增4项：边缘矛盾、未知范围/非法字段、末次观察预算与真实模型请求隔离。初轮82项中1错误为测试助手取错纠正后的消息；次轮86项中1失败暴露LangChain还会追加独立system_message，已在生产请求覆盖该字段。初次失败记录均保留，不将离线结果替代Gazebo实飞。

### 本轮真实果园结果与下一步

实例 `20261104-144059-RkcwHG` / 指令 `76b29dbaae974777`，原平地果园/种子17/qwen3.7-plus，仍为原句。实际5 m起飞、一次后退3 m候选越界拒绝后改选升到10 m、右移2 m、再右移1.5 m并升到12 m；4份照片/报告/复核，20次模型调用，0次转向。4次复核均明确no，最后照片仍右侧和底部裁树，故budget_exhausted、应用failed，未发生上一轮的假met。全部复核调用审计均为isolated_current_image；没有自然触发“总体yes但边缘cut”的矛盾分支，该分支只由受控测试验证。

程序返航降落 `f2560147f07b` succeeded、photo_return=ok，从距home约3.47 m处返航，实际armed=false、landed_state=1，最终水平距home约0.037 m。但接触阶段再次出现EV速度融合异常及PX4 local velocity reset锁存，最终EV healthy=false；**实际落地成功不等于估计连续性通过**。按照用户已明确的边界，仅保留已知触地异常，不清除门禁、不扩大专项诊断。

本地证据 `logs/setup/orchard-framing-review-20261104/`：summary.json、source-snapshot及sha256、run/result.json、states.jsonl、四张原图（左右hash已核验）、最终状态及真实桌面截图。实例关闭、实际参数读回恢复、VIO YAML恢复；无录像。不把本轮“如实未完成”写成全园构图完成，也不依据这一轮宣称模型泛化问题已消除。

下一步定位到明确的数据流缺口：实际4次adjust_photo_view决策调用均为text-only，当前照片只在report_scene与review_photo回合注入；选择平移/转向时仅依赖文字。这是已验证的输入缺口，**尚未证明它是动作选择失效的唯一原因**。应复用当前任务已核验原图，让取景选动作回合获得对应图像，并明确标记“复用旧观察做规划”，不计为新观察；先测试来源/所有权/新鲜采图界限，再原句对照。不要硬编码离线全园视点、扩大预算或强制模型选择转向。

## 相对转向入口（2026-11-04）

普通取景现在增加 `rotate_photo_view(direction, angle_deg, reason)`：仅接受 left/right 和 5～90°，在最新照片复核要求调整后才可调用。执行器在同一提交锁内验证当前已完成且属于本指令的飞行、照片坐标纪元、新鲜机体 ENU 姿态、导航及原地静态/场景安全性，再由当前机身航向计算目标角；ENU 正角为左转，并正确跨越 ±π。相机仍使用原固定外参，旋转不会改变相机俯角。

调用共享 `/navigation/photo-action` 的 `turn` 操作，最终提交既有单步 rotate，保留 `UAV_ENABLE_YAW=1` 开关、漂移/坐标/融合守卫、到达证据和旋转后新鲜双目帧门禁。任务归属、操作员中断及故障独立收尾沿用现有链路。活动 NBV 会话不能使用该入口，不接受绝对角度、任意步骤或转向与平移混合请求。

成功转向后状态变为 awaiting_photo，必须采图、报告、复核才能再调整；共用原四次观察预算，不连续盲转。原地安全检查失败属于状态/安全故障，不能伪装为可改选的平移候选拒绝。初轮专项96项中1项新增断言错误（公开结果不包含内部 flight_failure 字段），已修正为公开结果兼容检查，失败日志保留 `logs/setup/photo-relative-yaw-20261104/focused.log`。最终联合回归408项通过、0失败/错误/跳过，80.825秒，证据 `logs/setup/photo-relative-yaw-20261104/regression.log`。命令为交接记录的396项模块加 `test_yaw_control`，并包含本次5项新增测试。Python 3.8 编译与 diff 检查通过。真实果园结果另记，不将受控脚本模型作为真实构图成功。

### 原句果园复测：飞行正常，构图误判仍未解决

实例 `20261104-142313-OfHM3T` / 指令 `5de1184af8a64ee1`，平地果园/种子17/qwen3.7-plus，原样提交“请为我拍摄一张俯瞰整个果园的照片。”：4张真实照片/报告/复核、22次模型调用、一次后退候选拒绝、3次实际平移/升高，最高目标18 m；**未调用相对转向**。模型最终自行提交 return_home+land `f13118f16c62`，实际落地上锁，距home约0.081 m，最终VIO健康且无重置锁存，非独立故障原地降落。实例已关闭，实际参数读回恢复，VIO配置还原，无录像。

应用succeeded和photo_status=met来自模型构图判断，**本次不能验收为全园入镜**。实际第四张图仍有果树在底边和右边被截断；report_scene称不完整，而同图review_photo改称四边均完整。前几次复核又错误地把白色背景当裁切证据。原始模型结果及矛盾保留，没有篡改为失败或成功；独立验收明确为未通过。证据 `logs/setup/orchard-relative-yaw-20261104/summary.json`、`photo-01.jpg`～`photo-04.jpg`、`run/result.json`、`states.jsonl`和真实桌面截图。左右原图哈希均核验。

复测后修正旧进度摘要仅推荐adjust_photo_view的问题：adjust阶段不指定唯一next_tool，新增adjustment_tools明确列出平移与转向；不强制选择转向。相关82项回归通过，0失败/错误/跳过，2.136秒，`logs/setup/photo-relative-yaw-20261104/summary-regression.log`。这属于工具可见信息一致性修正，**不能据此宣称已解决视觉构图误判**；本次自然任务也未覆盖新增转向实际执行。

### 相对转向执行器独立实飞通过

另启实例 `20261104-142936-hJvskL`，同果园/种子17，使用已有SITL旋转验证流程的本地适配脚本（仅存忽略目录），不调用模型。连续地面预检后：3 m起飞 `021561ddcf47` → 真实双目before → 新photo-action/turn入口右转45° `b03b21e61c67` → 真实双目after → return_home+land `82e107bc99fe`。全部实际完成，最终armed=false、landed_state=1，VIO健康无最终重置锁存。

实际航向目标-0.795637 rad、到达-0.799190 rad，误差约0.204°。旋转期间采图被门禁拒绝；完成时刻62.340 s，后续左右帧均62.600 s，严格晚于完成，左右原图均按返回哈希核验，前后epoch一致。没有额外延长新帧等待或重试采图；使用执行器既有等待门禁。该结果证明新增相对入口与既有执行/采图链正常，不证明语言模型自然选择转向或全园构图成功。

证据 `logs/setup/orchard-photo-turn-executor-20261104/flight/result.json`、`flight/before-left.jpg`、`flight/after-left.jpg`、`preflight.json`、`final-state.json`，模型调用0、NBV0、无录像。实例关闭，实际参数读回恢复，VIO配置恢复。原模型果园任务及其误判另存上一节，不将两个任务拼成一次成功闭环。

## 果园构图可行性检查（2026-11-04，仅离线评估）

用上一轮4张真实照片的相机标定、机体姿态与固定外参，对固定哈希的果树原始视觉网格做视锥约束检查（不含地面/基站）。约50.7万个顶点；在各自实际机头朝向下，放宽高度到世界0～21 m并忽略障碍、遮挡和半径后，许可区域内仍不存在能把全部顶点同时放入单帧的机体位置。若放开水平边界，最靠东的可行位置仍需世界x约-8.53～-8.63 m，而当前许可下界为-1.35 m。故在这些固定朝向下，仅继续升高/平移不能完成严格的全园单帧构图。

这不是“20米内所有朝向都拍不全”的证明。额外离线5度航向采样在约-50/-45/-40度、地图内另一侧位置找到几何可行解，使用旧架次配准的静态路径也可规划。没有将这些真值视点传给Agent或执行器，没有新飞行/目标识别/遮挡保证；不能称为完整照片成功。诊断时普通取景接口只有相对平移和升降，因此明确存在相对转向能力缺口；现已按上节补齐入口，真实构图效果仍须验证。通用rotate继续受限，不将这些场景真值坐标传给模型。

独立评估原始输出在 `logs/setup/photo-generalization-20261104/`：`framing-feasibility.json`、`yaw-feasibility-inner.json`、`yaw-static-paths.json`。这是离线几何诊断，不替代用户任务的真实图像验收。

## 路径预检查拒绝与飞行失败的区别（2026-11-04）

真实实例 `20261104-133156-ImCjC2` / 指令 `2c21a76ed5ef47d6` 暴露了收尾分类缺陷：8 m 起飞和第一张照片报告完成后，正确的相机反向目标超出果园许可区域；执行器尚未提交换位，上层已将路径拒绝记为飞行失败，最终原地降落。实际 armed=false、landed_state=1；1 份报告、7 次模型调用，没有执行取景换位。失败记录和现场截图保留于 `logs/setup/orchard-relative-test-20261104/`，不能写成返航成功。

新增 `PathUnavailable` 仅标识候选终点或路径的几何不可达；当前起点非法、状态过期、坐标/归属变化等仍是故障。执行器只有在调整动作尚未提交时才返回 `not_executed / photo_path_unreachable / mission_submitted=false`，附真实方向转换结果、请求任务和 epoch。应用核验这些字段与当前任务/健康状态，不根据异常字符串判断可恢复，也不将拒绝记作已执行的 `/mission`。

每个拍摄会话最多累计 3 次路径拒绝，可改用较短距离、其他方向或垂直升高；不重置观察预算、不扩展场景边界。额度耗尽进入 `limited`，由已有程序返航门禁决定健康返航或安全降落。已受理飞行失败、视觉故障、操作员中止与坐标失效仍不可重试。返航自身规划失败或执行失败仍不重试返航。

专项 104 项受控测试通过（0 失败/跳过），包括真实图中间件的有限改选、拒绝额度耗尽后返航、伪造未执行结果和坐标变化反例。初轮新测试的内部字段读取及重复调用 ID 夹具错误已修正，失败日志保留。联合回归 **352项通过，0失败/错误/跳过，72.971秒**，证据 `logs/setup/photo-candidate-return-20261104/regression.log`。使用下方310项命令，在末尾加 `test_static_navigation test_scene_config`，并包括本次新增6项测试；专项与联合数量不可相加。

真实修复后复测：实例 `20261104-134236-GIk3ie` / 指令 `b0434cd3e4cb4765`，一次后退5 m候选被拒绝后成功改选垂直升高，随后实际相机向右3 m并升高、再次升高；累计四张照片/报告/复核，20次模型调用。构图 limited，程序实际从离home约3 m处返航并落地上锁，`photo_return.status=ok`。任务仍failed，全园构图没有达标。落地阶段短暂融合健康告警保留，未宣称估计连续性完全通过。详情与真实截图/原图/轨迹见 [果园验证](orchard_task_test.md)。

## 2026-11-04：相对方向转换与构图受限返航

最新实现已修正上次果园重飞暴露的方向错误，并区分构图不满足与飞行故障。本节取代下文初版中“模型根据当前 ENU 自行生成 goto”和“所有未达标均直接降落”的描述。

模型在复核要求调整后使用 `adjust_photo_view(direction, distance_m, altitude, reason)`。direction 为 forward/backward/left/right/vertical，距离为水平距离，altitude 仍是 ENU 绝对高度。执行器在提交锁内读取新鲜机体 ENU 姿态，结合现有固定相机外参，将光轴投影到水平面后计算相对方向。不能假定后退是 -Y；本机朝东时后退为 -X，朝北时后退才为 -Y。单次水平调整至多 10 m，垂直调整水平距离为 0；最终目标、路径、地图和原飞行范围仍由现有规划器验证。近垂直光轴无法定义水平朝向时拒绝，过期/未来位姿、坐标改变、错误任务、活动 NBV 会话均拒绝。

新增执行接口 `/navigation/photo-action` 只接受相对调整、受约束相对转向或固定的 `return_home + land`，不接受模型生成的任意步骤。请求需要当前已完成任务 ID 和上一有效照片的坐标 epoch；应用再次核验该任务属于当前指令。路径先预检查，随后仍走原 submit、执行与安全监控。普通拍摄阶段不能用通用 goto/rotate 绕过相对转换；已有 NBV 仍必须使用原执行票据。

构图复核进入 limited/budget_exhausted 后不再调用模型决定收尾。只有没有其他工具/模型/飞行/视觉错误、任务归属仍有效，且实际处于健康的空中 OFFBOARD、VIO/纪元/静态地图有效时，应用才尝试导航返航。目标使用执行器保存的 **home_enu_m 真实起飞点**，不是硬编码 ENU (0,0)。`photo_return` 单独记录程序收尾结果，只有实际 landed_state=1、armed=false 才报告已返航落地上锁；构图任务结果仍为未完成。

返航条件不足、规划拒绝或执行失败时不重试返航，重新读取当前任务归属，沿既有独立降落机制处理。任务被其他飞行替换时不能接管；操作员中断优先。真正的飞行/定位/图像错误不伪装成正常构图结束。本修改没有关闭安全门禁，也没有把所有任务错误改成返航。

提示词同时澄清：“俯瞰”默认允许斜俯视，只有显式要求才必须垂直俯拍；白色背景本身不证明果园被裁切；选过的距离不能冒充系统许可上限。

修改文件：新增 `photo_navigation.py`，更新 `bridge.py`、`single_vlm.py`、`photo_task.py`、`web_server.py`；新增 `tests/test_photo_navigation.py` 并更新拍摄图测试。相对变换在本机实际执行器 Python 3.8 下可导入运行。真实历史首图姿态离线复算“后退8米、高度15米”为 `[-7.9993, 0.0648, 15]`；这是方向复算，**未做该目标的实飞或路径通过声明**。

最终联合回归 **310 项通过，0 失败/错误/跳过，71.672 秒**；31 项为方向/拍摄专项（已包含在 310 项中，不能相加）。证据 `logs/setup/photo-direction-return-20261104/`：`regression-final.log`、`final-focused.log`、`actual-pose-replay.json`。本轮没有新 Gazebo 或在线模型调用，真实相对换位和构图未达标后的程序返航仍待独立实例验证。执行命令：

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 PYTHONPATH=.:tests:scripts \
  .venv/bin/python -X faulthandler -m unittest \
  test_photo_navigation test_photo_task test_single_vlm test_agent_reliability \
  test_agent_tool_schema test_task_memory test_web_server test_search_termination \
  test_nbv_search_integration test_target_confirmation test_search_graph_integration \
  test_autonomous_search test_desktop_search_data test_desktop_workbench test_desktop \
  test_bridge_static_navigation test_recovery_ownership test_bridge_camera test_ev_heading_guard
```

## 初版背景与证据（相对方向和收尾以上方最新实现为准）

2026-11-04。针对果园原句任务，旧 Agent 在保存一张局部照片后就结束；换成 qwen3.7-plus 后仍有相同行为。本次修复的是普通拍摄的完成检查，不改变自主搜索/NBV/多视角目标确认或飞控配置。

## 行为与范围

用户仍可直接输入：`请为我拍摄一张俯瞰整个果园的照片。`

第一版以显式的“拍摄/照片 + 俯瞰/俯视/整个/全景/全貌”等措辞启用取景闭环，支持对应的常见英文表达；普通单次拍照保持原流程，明确历史问题不自动采图。任务识别是有限规则，不代表实现了通用语言意图解析。已有目标搜索会话优先使用原搜索状态机。

流程：读取状态、决定取景位置 → 实际采图 → 保存 report_scene → 同一 VLM 再看这张原图并调用 review_photo → 达标结束，或调整后再拍，或明确说明限制。

`review_photo` 记录两个独立的视觉判断：

- `subject_complete`：主体是否完整入镜，值为 yes/no/uncertain。区域边界不可知或画面裁切不能当成完整覆盖。
- `view_suitable`：视角是否满足用户要求。同样使用 yes/no/uncertain；区分斜俯视、垂直俯视和前向观察。

两项都为 yes 才进入 `met`。其他情况必须提供下一步调整理由，或具体的能力限制。复核结果仍是**模型构图判断，不是独立核验的完整覆盖真值**；不能据此承诺照片绝对满足要求。

程序最多允许 4 次实际采图。每次先保存报告并完成复核，才允许继续取景；未达标的文字结束最多纠正两次，额度按当前任务共享。工具、视觉或飞行失败不属于可重试的文字结束，不重新执行失败飞行。模型可根据当前 ENU 位姿提出取景阶段，由现有执行器验证路径；不根据检测框编造目标坐标或绕障路径。原 20 m 上限、水平范围、静态碰撞、坐标纪元和操作员保护保留。

有限状态为 `awaiting_photo → awaiting_report → awaiting_review → adjust/met/limited/budget_exhausted`。降落/停止后关闭拍摄活动，不允许恢复采图或再次起飞。已满足构图时仍保留 `met` 结果，并单独记录活动关闭。独立安全流程和桌面操作不经过模型取景授权。

应用保存 `photo_events` / `photo_progress`，包含照片编号、报告相机元数据（含图像哈希）、视觉理由、调整建议、预算和结果。`limited`、预算耗尽或缺少复核不能因最终文字说“成功”而标绿。已保存照片继续保留；未完成任务只在能够证明飞行归属时由现有独立流程安全收尾。没有飞行归属的观察不会自动控制另一任务的飞机。

本版每张照片多一次带原图的模型复核请求，会增加延迟和图像 token 成本。先验证完成判据，再考虑合并调用；没有更换模型、改变温度或增加模型网络重试。

## 文件

- `photo_task.py`：有限取景状态、预算、显式判据与拍摄提示。
- `single_vlm.py`：按真实用户轮次建立会话，阶段工具约束、原图复核、过早结束纠正及工具失败处理。
- `agent.py`：明确允许从当前 ENU 位姿选择任务取景点，消除仅允许用户指定目的地的提示冲突。
- `web_server.py`：保存取景审计，区分照片保存与构图完成；未完成时使用现有归属校验和安全收尾。
- `tests/test_photo_task.py`：真实 LangChain 图的本地脚本模型测试，不是 Gazebo 实飞或真实模型效果评测。

## 验证与下一步

回归命令（只运行本地测试；HTTP 用例需要回环套接字）：

```bash
QT_QPA_PLATFORM=offscreen UAV_DESKTOP_UI_TEST=1 PYTHONPATH=.:tests:scripts \
  .venv/bin/python -X faulthandler -m unittest \
  test_photo_task test_single_vlm test_agent_reliability test_agent_tool_schema \
  test_task_memory test_web_server test_search_termination test_nbv_search_integration \
  test_target_confirmation test_search_graph_integration test_autonomous_search \
  test_desktop_search_data test_desktop_workbench test_desktop
```

最终版本回归 **216 项通过、0 失败、0 错误、0 跳过**，耗时 70.459 秒，包含新增 16 项拍摄测试。结果为 `logs/setup/photo-task-20261104/regression-final.log`。初轮暴露的空 store 兼容问题和新测试缺少 execute 夹具已修正，初轮失败记录保留同目录；不能把中间通过的版本代替最终结果。

代码实现轮没有启动 Gazebo 或调用在线模型。随后用户要求的真实重飞已完成：实例 `20261104-130816-F8E7Vm`，4 份照片/报告/复核、3 次普通换位，21 次模型调用；首次照片之后确实继续执行，但最终 limited，未拍全。独立收尾实际落地上锁。具体照片、路径、审计和结论见 [果园重飞记录](orchard_task_test.md)。

上述方向与判据修改已在本页首节实现并受控验证；下一步是按原句、原许可及安全门禁验证实际取景与返航。不能宣称已经实现全园构图。
