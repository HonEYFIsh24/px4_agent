# Semantic Map MVP

本版只新增被动空间语义记忆，不实现 NBV、航向动作或新的目标终止判据。现有 PX4、VIO、静态三维 A*、执行器门禁、升高/横移/前进选点继续工作。地图和评分不进入模型上下文，不干预当前路线；地图身份明确失效时禁止后续搜索移动，安全降落仍可执行。

## 表示与数据来源

`semantic_map.py` 维护任务局部的 24×24 m、0.5 m 分辨率地面栅格（48×48），以及稀疏三维地标/未定位方位线索。分母只包含 12 m 搜索圆、场景许可水平区域内且未被实体占据的地面单元。

每张照片经过本轮所有权、实际模型图像投递、报告成功、采集时效与画质检查后，使用采图时刻的 ENU base_link 位姿和真实相机内参查询 `/navigation/observation-geometry`。后端只接受缓存双目 pair ID 和任务栅格原点；不能提交任意虚构相机位姿。返回证据绑定照片左右 SHA256、仿真时间、位姿、内参、world SHA256、VIO epoch、EKF 重置计数和配准随机标识。

`static_navigation.py` 复用相机投影和线段/AABB 相交，只使用**未膨胀碰撞几何**做遮挡；原 1 m 膨胀仍用于飞行安全。地面栅格中心位于视场与有效 10 m 光轴距离内且无遮挡时，coverage 从 0 变为 1。0/1 是几何观察机会，不是识别概率；一次未检出不能排除目标。斜角/曲面采用碰撞体外包 AABB，可能保守；只采样格中心，不声称整格像素覆盖。许可区域之外不计入统计，不把任务半径当整条飞行轨迹约束已解决。

每格保存 coverage、最近照片/仿真时间、至少相隔 0.5 m 的不同观察位置计数、语义吸引分数、候选目标 ID。照片 ID 和左右图像哈希对共同去重，拒绝时间倒序，最多 128 次观察。重复位置可以更新最近照片，但不增加不同视角次数；本阶段不计航向差异。

## 语义证据与关联

地图只读取 `report_scene.targets` 中真实图像对应的标签/框；允许报告搜索目标以外、与用户明确关系有关的地标。不从 summary 自动臆造框，不读取 Gazebo 目标名称或目标坐标。已知碰撞几何与世界/ENU 配准只用于可见性和飞行。

有效 `stereo_sgbm_robust_surface` 双目位置继续进入原 `coverage_search.TargetMemory`。共享记忆增加显式关联入口，旧调用默认行为保留。新地图要求同标签、表面点间距不超过 6 m、两个表面点分别投影进对方实际检测框，且只有一个兼容候选时才关联；同帧不同物体不融合。协方差保留原混合估计的跨视角散布，不用独立平均假设缩小共享误差。

三维点是**可见表面代表点**，并非物体中心。`candidate` 为单视点；`confirmed` 仅表示至少两个不同位置的几何证据，不代表视觉类别已经独立核实，也不会改变当前搜索的单帧终止逻辑。相似物体重叠、大物体不同表面仍可能关联失败或错误关联；需要后续更多实际场景和外观约束验证。

双目失败时保留 bbox、相机位姿、内参、ENU 归一化视线和失败原因，不造三维坐标。当前最多 64 条未定位线索，不对缺深度的线索强行跨帧合并；成功定位后也不自动把旧线索升级成同一物体。

只有用户原话明确以“附近/旁边/边上/周围”等正向关系连接**已观察地标标签**与当前目标 query 时，才建立配置尺度 3 m 的高斯语义吸引场。关系尺度与定位协方差分开；不把 VLM confidence 当概率，也不引入“油桶通常在集装箱附近”等常识先验。这是严格字符串匹配的首版白名单，别名、反向语序、复杂关系尚不支持，匹配不到则保留零语义场。

`coverage_gain`、`semantic_gain`、`insufficiently_observed_cells`、`semantic_regions` 提供后续 NBV 所需查询；当前无人调用它们选择航点。

## 生命周期与归档

`semantic_map_runtime.py` 每个实际线程/指令 owner 建图，最多保留 8 个任务。世界指纹、配准标识、epoch、许可栅格发生变化会使旧地图失效，不能静默重配准。地图状态传给自主搜索会话的只有薄身份标识，避免第二套搜索状态机。跨重启不自动恢复旧地图。

报告审计中新增 `semantic_map_events`。设置 `UAV_CAPTURE_EVIDENCE_DIR` 后，每张有效图像旁保存 `<capture_id>.geometry.json` 与 `<capture_id>.semantic-map.json`；桌面入口已配置实际照片归档目录。原图、原报告与标定不改写。错误以独立地图事件记录，不伪装成有效观察，也不修改已保存的视觉报告。

## 离线回放与可视化

使用系统 Python 的 matplotlib/numpy 离线绘图，不依赖飞控或模型网络：

```bash
/usr/bin/python3 scripts/replay_semantic_map.py --demo --output logs/setup/semantic-map-mvp/controlled
/usr/bin/python3 scripts/render_semantic_map.py logs/setup/semantic-map-mvp/controlled/semantic-map.json --output logs/setup/semantic-map-mvp/controlled/map.png
/usr/bin/python3 scripts/replay_semantic_map.py --result logs/setup/semantic-map-mvp/live/result.json --output logs/setup/semantic-map-mvp/replay
/usr/bin/python3 scripts/render_semantic_map.py logs/setup/semantic-map-mvp/replay/semantic-map.json --output logs/setup/semantic-map-mvp/replay/map.png --world scene/test_environments/courtyard_delivery.world
```

PNG 与 SVG：灰色未观察、蓝色已有几何观察、深灰不计统计、暖色显式语义吸引；叠加观察点、未定位视线、地标不确定度。绿色若存在仅表示日志中实际规划的路径，不是飞行实测轨迹。尚无 NBV 候选视点。字体使用本机 Noto CJK，避免中文方框。

`--demo` 明确为**受控合成夹具**，只验证遮挡、融合和显式关系逻辑，不能算真实视觉定位成功。真实回放会校验左右 JPEG SHA256、模型投递记录和采图注册证据。旧档案缺失 `.geometry.json` 时拒绝猜测 ENU/世界配准，可加 `--audit-only` 输出缺失清单。

新增的证据审计可以在不飞行、不调用模型的情况下复核归档：

```bash
/usr/bin/python3 scripts/audit_semantic_map_evidence.py --result logs/setup/semantic-map-mvp/live/result.json --output /tmp/px4-semantic-evidence-audit
```

工具检查每份成功报告对应的地图事件、JPEG 哈希与模型投递、在线/回放一致性，并实际尝试重复提交照片验证地图不变。仅派生 cell.semantic_score 允许相对 1e-12/绝对 1e-15 的跨运行时舍入容差，同时保存精确比较和最大偏差；其余字段严格相同。`evidence-audit.json` 分别记录真实多视点几何证据、显式语义热区和未定位方位线索是否存在；单视点不会被记为多视点。可加 `--require-multiview`、`--require-semantic-field`、`--require-bearing` 要求对应项存在，未满足返回 2；归档损坏返回 1。没有这些选项时，只要求归档完整性，不表示所有地图能力验收通过；工具不独立核实类别或坐标精度。

## 验证范围

受控单元测试覆盖遮挡与安全膨胀分离、照片去重、两个视点融合及不确定度、同类邻近目标分离、方位线索、显式/隐式/否定关系、事务回滚、地图身份失效、缓存元数据隔离和过期。运行时集成测试覆盖报告接入、归档和搜索移动门禁。真实闭环和离线回放结果见 `logs/setup/semantic-map-mvp/validation.md`。

2026-10-26 补验在独立架次取得红色集装箱三视点融合、显式关系热区及真实双目失败方位线索，三项要求的离线审计通过。原始第一帧还存在车辆误框并保留错误单视点候选，不能声称地图标签均正确或已解决误报。成功、失败、人工复核与浮点容差说明见 `logs/setup/semantic-map-evidence-20261026/validation.md`，最新下一步以根目录 `PROJECT_HANDOFF.md` 为准。

固定航向三维候选 NBV、预览路径范围约束和几何/语义数值基线现已通过离线验证，见 [NBV 基线说明](viewpoint_planner.md)。后续先接入受约束执行与候选 ID 闭环，再加入旋转控制和主动多视角确认。本版不宣称搜索成功率、耗时或航程已经提高。
