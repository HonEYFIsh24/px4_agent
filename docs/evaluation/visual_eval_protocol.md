# Gazebo 视觉与跨视角评测协议（建设中）

本协议目标是 80～120 张带采集证据的实际 Gazebo 相机图像、至少三个不同布局、20～30 组经人工核对的跨视角图像对，以及真实任务确认结果。当前尚未达到：本地草稿 `logs/setup/visual-eval-dataset-20261102/draft-inventory.json` 有 14 张、1 个世界、0 份人工标注、0 组人工复核配对；它**不是评测集**。`review-contact-sheet.pdf` 是原始图像索引，不代表标签。原图、草稿及运行记录在忽略的 `logs/` 中，不随 Git 发布。

## 数据和审核

- 固定采集清单，按世界和布局先划分 development 与 holdout。布局相近、只改变颜色或随机种子的场景必须归为同一 `layout_group`；同一场景或同一世界哈希不允许跨分组。人工检查布局关系，哈希检查无法发现内容相似的布局。保留不同任务及时间下的关联，避免相邻帧泄漏。
- 每张原始左相机图像记录独立 ID、capture ID、原始 SHA256、图像与几何归档相对路径、世界路径及哈希、场景 ID、布局组、实际种子进程证据、架次 ID、仿真时间、采集时 `base_link` 位姿、相机光心与内参。姿态与图像均来自采图归档；位姿是机体姿态，不能伪称相机姿态。需结合安装外参推导相机姿态。
- 三种标注：`identifiable` 是框内视觉证据足以识别所列类别；`insufficient_evidence` 包括画面太小、遮挡或无法可靠判断，允许真实场景中存在目标；`absent` 指**指定可观察区域**内没有该类别，不能用画面外或被遮挡处的真值作证明。必须保存 `observation_region`（归一化框）；`identifiable` 必须提供目标框、实例 ID；`absent` 的框和实例 ID 必须为空。保存遮挡 `none/partial/heavy/unknown`、主观距离档 `near/medium/far/unknown`；距离档不是实测测距。目标像素尺寸可以从目标框和真实图像尺寸计算。
- 标注来源可为 `human_visual_review` 或 `assistant_visual_review`，须记录标注者。独立人类复核者须亲自查看图像，给出不同的 reviewer ID、`reviewer_kind=human` 和 `review_status=approved`；这些字段是可追溯声明，脚本不能验证身份或证明审核确实发生。存在分歧时先解决并保留原始意见，不得自动替人批准。
- 跨视角对必须有两个不同的实际图像 ID、同一场景和同一分组。`same`/`different` 需要对应的两张图均有可辨认的同类目标及实例 ID；`insufficient_evidence` 用于身份无法判定。实例 ID、身份关系和类别均由图像审阅确定，不从 Gazebo 模型名或坐标提供线上语义答案。人工身份判断也有可能出错。
- 任务级样本引用真实任务 `result.json`、命令 ID 及其实际采图；独立复核只把确认结论标为 `correct`、`incorrect` 或 `insufficient_evidence`。执行器输出必须来自任务中保存的结构化 `search_outcome`，不能由评测脚本或人工捏造。任务标签应覆盖类别、实例及原任务要求的属性/空间关系是否成立。

## 复现和指标

```bash
python3 scripts/inventory_visual_eval.py --root . --output logs/setup/NEW_RUN/inventory.json
/usr/bin/python3 scripts/render_visual_inventory.py --inventory logs/setup/NEW_RUN/inventory.json --root . --output logs/setup/NEW_RUN/contact.pdf
python3 scripts/visual_eval_dataset.py --manifest PATH/manifest.json --root .
python3 scripts/visual_eval_dataset.py --manifest PATH/manifest.json --root . --predictions PATH/predictions.json
```

草稿必须先补全标签、场景分组、配对和任务；完整验收会检查 80～120 张、20～30 对、至少三个不同世界和布局、三种状态、可辨认油桶正例、同一/不同实例配对及已复核任务。无法自动证明三个不同哈希是视觉上独立的布局，也不能自动证明标注质量。`--allow-small` 仅供测试协议与工具，输出会标记没有应用规模门槛，不得当作完整评测集。

预测文件需携带冻结清单的 `manifest_sha256`、`run_id`，以及 `images`、`pairs`、`tasks` 三组精确匹配 ID 的记录；单帧回答为 `present/absent/uncertain`，配对为 `same/different/uncertain`，任务为 `confirmed/not_confirmed/error`。任务答案必须与归档结构化结果一致。应在请求模型之前由版本控制或其他带时间的独立记录冻结清单和实验配置；清单哈希只能检测之后的文件变更，**本工具不能单独证明冻结时间早于模型调用**。

每个分组分别输出分子和分母：单帧误报 = 人工 `absent` 中模型 `present`；可辨认目标的明确漏报 = 人工 `identifiable` 中模型 `absent`；不确定率 = 全部样本中模型 `uncertain`。`insufficient_evidence` 单独计数，不算负例或可辨认正例；空模型响应只有在适配器明确记录为 `uncertain` 时才算未决。配对分别报告不同对象误判为相同、相同误判为不同及未决。任务错误确认同时报告：错误确认数/已确认且可裁定任务数，以及错误确认数/全部任务数；无法裁定的已确认单独计数。分母为零输出 null，不当作零错误率；还应按类别、场景和目标尺寸分层，并保留未决及失败样本。

当前工具验证文件哈希、位姿/时间、相机内参、来源任务中的种子记录及场景隔离，并检查标签、身份声明和预测 ID。它不证明进程取证本身真实、标注者身份、人工标签正确、场景真正独立或模型预测来源可信；这些须保留原始记录并由独立审阅与实际运行验证。不要根据同一开发样本持续改提示并宣称独立留出集性能提升。
