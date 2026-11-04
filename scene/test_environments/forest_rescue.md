# 林地救援营地

`forest_rescue.world` 是本地生成的 100×80 m 静态 Gazebo Classic 测试世界，面积 8000 m²，为旧 40×40 m 场景的五倍。包含 59 个模型：30 棵树、15 组岩块、3 栋木屋、3 个不同颜色的 A 型帐篷、6 根遗迹柱、一座带栏杆的步桥及地面/步道/溪流视觉层。近景低矮岩块、树干/树冠、倾斜帐篷和屋顶产生多尺度遮挡；保留原点垂直起降净空。

这是一版程序生成的低多边形测试环境，岩石目前使用保守盒体，溪流仅是视觉层，没有水体动力学；地面保持平面，尚非山地高程导航。无需联网下载模型，纹理复用本地草地、木材、砖与混凝土资源。物理步长、磁场与飞控/VIO 配置沿用已验证版本。

## 生成与校验

```bash
python3 scene/generate_forest_world.py
gz sdf -k scene/test_environments/forest_rescue.world
PYTHONPATH=.:tests .venv/bin/python -m unittest test_forest_environment test_alternate_environment test_static_navigation
```

生成器保留注册表其他场景；旧 courtyard 生成器也改为仅更新自己的注册项。注册表验证 world 和 manifest 哈希。运行时 manifest 只含几何边界、地面、导航与起降许可；目标真值单独保存在 evaluation_only 的 layout 文件，Agent 不读取。

## 当前自主测试范围

世界与静态导航边界扩大，**单次搜索仍为起点周围 12 m、1～10 m 高度、8 次观察预算**。本版不能表述为“已经完成 100×80 m 全场搜索”。首个真实任务为“找到前方的橙色帐篷，调整到合适角度拍摄，确认后报告周围环境，完成后返航降落”。不提供人工航点或目标真值。

启动（在项目目录下，显示环境使用当前登录桌面）：

```bash
GUI=true UAV_NAVIGATION_MODE=static_3d UAV_ENABLE_YAW=1 UAV_ENABLE_NBV_YAW=1 \
UAV_SEARCH_MODE=nbv_geometry UAV_NBV_SELECTOR=deterministic \
UAV_NBV_HEADING_MODE=discrete UAV_NBV_AUTO_PREPARE=1 \
UAV_TARGET_CONFIRMATION=multiview PX4_AGENT_GAZEBO_SEED=17 \
bash scripts/start_system.sh --control-ui desktop --env-file .env --ev-profile no_gps \
  --world-file scene/test_environments/forest_rescue.world
```

运行生命周期检查要求启动脚本身份可核验；自动化建议将脚本、env 和 world 参数替换为绝对路径。不要与已有实例并行启动。

## 验证

SDF 校验通过。15 项场景、注册和静态导航受控测试通过（失败/跳过 0），包括可重复生成、保留旧注册、拒绝篡改、语义真值隔离、起降净空、岩块碰撞与通路检查。沙箱内首次 SDF 校验因 getifaddrs 权限失败，实际主机环境复测 Check complete。

真实实例及结果见 PROJECT_HANDOFF.md；本机证据目录 `logs/setup/forest-rescue-20261103/`。`gazebo-unobscured.png` 是实际 XComposite 窗口截图，未录制视频；首次 `gazebo-overview.png` 含控制台遮挡，仅保留为现场记录，不作为无遮挡全景。

首轮真实测试（2026-11-03）：任务 `ca2804b4cb434f61`，实例 `20261103-210128-GokvWP`，种子 17。6 张照片/6 份已保存视觉报告、5 次实际票据移动、18 次模型调用；地图只接纳前 5 份。帐篷有视觉检出，一轮复核未确认（无唯一几何关联）；最后目标高度 1 m，goto 实际到达 0.9117 m，旋转完成和图像姿态为 0.8282 m，随后 report_scene 被搜索 1～10 m 门槛拒绝。最终失败并实际落地上锁。NBV 与确认总体审计未通过，不能记为目标确认或完整闭环成功。下一步是给候选高度与实际到达/旋转容差建立一致的边界裕量；不降低观察高度门槛。

高度边界问题已修复并在同场景复测：6 次观察均在 1～10 m，4 次旋转后新双目帧检查通过；任务随后因模型降落后仍请求观察而失败。完整记录见 `docs/nbv_height_margin.md`，仍不可宣称林地多视角闭环通过。
