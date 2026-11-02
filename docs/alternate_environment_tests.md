# 新装卸庭院与跨任务测试

新场景为 `scene/test_environments/courtyard_delivery.world`：重新布置低遮挡墙、红色集装箱、白色货车、交通锥、蓝桶、办公/仓储建筑和灯柱。沿用本地已有材质与传感器/物理配置，不需要下载外部模型，也不改写原油桶场景。

## 校验与启动

```bash
/usr/bin/python3 scene/generate_test_environments.py
gz sdf -k scene/test_environments/courtyard_delivery.world
bash scripts/launch_desktop.sh world:="$PWD/scene/test_environments/courtyard_delivery.world"
```

新场景使用独立注册表 `scene/test_environments/index.json`，同时校验世界文件和公开 manifest 的 SHA256。它只允许 `static_3d` 模式，避免低矮车辆、交通锥落入二维雷达固定扫描面的盲区。任意外部 world 或哈希不匹配文件仍不能获得地图信任。原可靠性场景注册表保持不变。

公开遥测只包含场景 ID、指纹、飞行许可和导航参数；目标 ID、位置与包围盒仅存于生成器和离线 `courtyard_delivery_layout.json`，不发送给 Agent。初始起降通道已检查至 10 米；保守物理 AABB 检查只发现外围栅栏的两个结构性角连接，没有其他跨模型重叠。

本地检查包括新场景的可重复生成、旧场景不被改写、注册与篡改拒绝、无目标真值泄露，以及已有场景/导航回归。SDF 校验通过；103 项相关检查通过，记录在 `logs/setup/courtyard-tests/regression.json`。

## 自然语言任务与证据

任务不提供坐标、航点、目标位置或手工路线，沿用现有 `qwen3-vl-plus`、VIO 主定位、1～10 米静态三维规划与 0.65 米/秒巡航配置。

1. “找到前方的白色货车，拍照分析它周围有哪些物体，最后返航降落。”
   - 3 次真实观察，168.5 秒，先推进再升高，确认实际白色货车后返航降落上锁。
   - 保存照片已目视核对。中间报告误称车辆位于屋顶平台，最终回复改为沥青地面；车辆识别通过，描述一致性只算部分通过。
   - 日志：`logs/setup/courtyard-white-van.log`；报告和照片：`logs/setup/courtyard-tests/white-van/report.md`；独立复核：同目录 `manual-review.json`。
2. “查找前方的交通锥，拍照报告实际可见的数量、颜色，以及它们与白色货车的相对位置。不要推测被遮挡部分，最后返航降落。”
   - 在同一仿真实例连续进行第二次起飞时，VIO 检出 `PX4 local coordinate frame reset during flight`，在成功观察前终止任务并安全降落。原始上层错误为 mission replaced，飞行日志和 EV 状态保留真正门禁原因。
   - 失败证据：`logs/setup/courtyard-tests/traffic-cones/result.json`、`failure-review.json`，过程 `logs/setup/courtyard-traffic-cones.log`。
   - 不跳过坐标系检查；重新启动独立 PX4/VIO 初态后再进行交通锥任务，复测写入 `traffic-cones-restarted`，不覆盖失败记录。
   - 独立复测通过：4 次观察、212.6 秒，确认实际可见的 1 个橙色交通锥，位于白色货车画面左前方；下部有遮挡，没有把离线场景中存在的第二个锥桶也推测计入。随后返航降落上锁。照片及人工核对在 `logs/setup/courtyard-tests/traffic-cones-restarted`。

汇总图文报告：`logs/setup/courtyard-tests/report.md`，结构化结果：同目录 `summary.json`。三次试验的有效观察照片均已校验 SHA256 与实际模型输入记录一致。连续第二次起飞时的坐标系重置原因尚未定位，独立重启复测通过不等于解决该问题。

这些是有限场景内的具体测试，不代表任意新环境或所有连续架次均已验证。
