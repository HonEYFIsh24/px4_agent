# 使用现成 Gazebo 场景

## 果园飞行接入更新（2026-11-04）

用户已选择平地测试副本。已保留 Clearpath 原始视觉资产并导入保守碰撞几何，真实地面 VIO 通过，完成一次起飞与悬停；降落末段发生速度融合异常及 PX4 速度重置，基础飞行验收未通过，未进入 Agent/NBV 搜索。飞机实际落地上锁、实例关闭、参数恢复。详见 [平地果园测试](orchard_task_test.md)。下文“均未接入飞行”的记录是此前仅预览阶段的状态，原始起伏果园仍未通过飞行验收。

## GitHub 场景配置（2026-11-04，最新）

已按用户要求下载三个上游场景，资产放在忽略目录 `scene/external_worlds/`。版本、原世界哈希、资源路径和预览调整记录在 [`scene/external_worlds.json`](../scene/external_worlds.json)。原有 PX4/VIO、搜索与导航生产逻辑未修改，现有成功场景仍保留。

| 名称与来源 | 本机配置 | 实际验证结果 |
| --- | --- | --- |
| [AWS RoboMaker Small Warehouse](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/3c23a698bf0b4e366ddf8b084af507c519bd3483) | `warehouse`，上游去顶棚版本，保留货架、纸箱、搬运车、标线、墙体及碰撞 | 已实际加载，截图可见纹理与设施；适合先做室内场景展示 |
| [Clearpath Orchard](https://github.com/clearpathrobotics/cpr_gazebo/tree/509afbeec942c6aad376a5a76a24330f1c56dc17/cpr_orchard_gazebo) | `orchard`，将上游 xacro/URDF 用 Gazebo 原生工具转换；保留原始地形、树干、树叶、基站和碰撞，不启动地面机器人 | 已实际加载，截图可见果园、土路和树木；适合作为与物流园不同的室外候选 |
| [Open Robotics CitySim](https://github.com/osrf/citysim/tree/3928b08e2598f5ead2e9b24640fe1bde262a136d) | `city`，完整街区；为单独预览移除六个交通/车辆控制及视觉效果插件，设置俯视相机；保留原建筑、道路、车辆、地形与碰撞 | 世界能加载、SDF 引用资源齐全，但当前实际画面仍有地表发白及局部蓝色区域，**暂不作为合格演示画面** |

CitySim 另从 [Open Robotics 模型库](https://github.com/osrf/gazebo_models) 固定树版本 `8163eb4b5e7e21985c6591d1c0bfb56468c0093f` 下载 24 个缺失模型、126 个文件（逐文件 Git blob SHA1 校验）；完整列表保存在下载目录 `.source_revision.json`。其余已有模型通过本机 `.gazebo/models` 读取；这是当前安装依赖，不能认为复制主仓库就包含全部资源。

许可证保留在原下载目录：CitySim Apache-2.0；AWS 仓库 MIT-0；Clearpath 果园 `docs/LICENSE.txt` 为 BSD 三条款。模型库按模型保留配置及原有声明。本项目不提交这些第三方大文件、运行日志和截图。

### 直接打开

在图形桌面的终端运行以下任一条。默认独立端口 11365；有实例占用时会拒绝启动，不接管它。关闭窗口或按 Ctrl+C 会清理本命令创建的客户端和服务器。

```bash
cd /home/fxxk/px4_agent
python3 scripts/preview_external_world.py warehouse
python3 scripts/preview_external_world.py orchard
# 城市仍存在下文记录的渲染问题
python3 scripts/preview_external_world.py city
```

只检查资源、不打开窗口：`python3 scripts/preview_external_world.py orchard --check`。
自动退出的预览：`python3 scripts/preview_external_world.py warehouse --duration 30`。
同时查看另一场景时显式使用不同端口，例如 `--port 11366`。

脚本使用系统 Python 3、Gazebo 11；果园准备步骤还使用现有 ROS Noetic xacro。通过远程会话启动时需要当前桌面的 `DISPLAY`/`XAUTHORITY`，不要把本机授权路径提交成通用配置。准备过程会核对固定提交与原世界 SHA256，递归检查 SDF 引用（包括高度图贴图），拒绝缺少资源和未经列明的插件；这些检查不等于每个 DAE 内部贴图、碰撞精度或飞行可行性都已验证。

派生 `.world` 及来源说明在 `scene/external_worlds/prepared/<名称>/`；原下载世界没有覆盖。每次启动的日志和退出码在 `logs/external-worlds/<名称>-<时间>/`。预览不启动 ROS master、PX4、VIO 或 Agent，不签发飞行许可，也没有注册到可信导航场景索引。

### 实际检查与边界

- 三个配置的递归 SDF 路径检查通过，无缺失资源和未处理插件。仓库检查 13 个 SDF 文件，城市 60 个，转换后的果园 1 个。
- 首次城市预览出现黄黑地形；OGRE 日志指出读取了旧目录 `gazebo_models_worlds_collection` 的失效绝对路径。根据 Gazebo 11 的 [Heightmap 缓存源码](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/rendering/Heightmap.cc) 和 [SystemPaths 源码](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/common/SystemPaths.cc)，入口改为每次使用独立 `GAZEBO_LOG_PATH`，隔离地形缓存；没有清除用户旧缓存。另加入原资产法线贴图目录。复验没有原失效路径错误，黄黑纹理消失，但截图地表仍发白，不能声称全部渲染问题已修复。
- AWS 有旧 SDF/Collada `COLOR` 兼容警告；截图中的主要纹理与设施正常。关闭在线模型库会产生预期的本地资源模式提示。原始日志完整保留。
- 先结束自己的客户端，再结束自己的服务器。限时城市和果园预览实际退出码为 server=0、client=-15（启动器预期发送 SIGTERM），启动器正常退出，未复现上一轮同时关闭后的客户端 SIGSEGV；这不等于已经证明旧崩溃根因。
- **三者均未通过 PX4 自主飞行验收。** 当前静态导航不直接支持这些 included/mesh/heightmap 几何。CitySim 的视觉高度图与碰撞平面尤其需要核验；果园也不是严格平地。不能忽略未解析障碍、使用世界目标名称给 Agent 答案，或将资源预览当成安全起飞许可。
- 下一步应先从仓库或果园选择明确的测试区域，完成几何导入、地面与净空核验，再做基础起飞及搜索验证。搜索半径仍为现有 12 米，本轮没有扩大。

本机真实截图：`logs/setup/github-worlds-20261104/warehouse.png`、`orchard.png`。城市异常与复验均保留为 `citysim.png`、`citysim-fixed.png`、`citysim-final.png`。没有录制视频。

受控测试命令（系统 Python 3）：

```bash
PYTHONPATH=.:tests:scripts python3 -m unittest -v \
  test_external_world_preview test_scene_config test_static_navigation
```

结果 **45 项通过、0 失败、0 跳过**，记录 `logs/setup/github-worlds-20261104/regression.log`。覆盖来源变更拒绝、插件拒绝、递归依赖/高度图检查、端口占用拒绝、客户端启动失败的服务器清理及有界退出。没有重跑或宣称重新通过历史 355 项飞行联合回归。

## 历史：本机随附 city.world 检查（2026-11-04）

用户要求用现成世界替代自建几何体场景，优先改善环境视觉效果。下一轮首选本机 PX4 Gazebo Classic 随附的 `city.world`，保留其原始建筑、布局和贴图。它不是本项目生成的世界，也不是已经通过自主搜索验收的环境。

源文件：`$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/city.world`。

本机 SHA256：`93847fb5e972334e46153d30c8b72bd6cf3bdf6b7b5de6bc6644131e14804ff2`。

已经在独立 Gazebo master 端口 11365、种子 17 下加载未经修改的原文件，并检查实际窗口截图。可见房屋、餐厅、垃圾箱、加油站、围墙、梯子及沥青地面。没有启动 PX4、VIO、Agent，没有起飞或录制视频。

本机证据：`logs/setup/gazebo-existing-world-20261104/`，包含 `source.json`、`inspection.json`、`gzserver.log`、`gzclient.log`、`city-original.png`。这些运行文件不提交仓库。

预览结束后仅向本次端口对应的两个进程发送 SIGINT：服务器退出码 0，图形客户端在退出时发生 SIGSEGV（139）。实际截图此前已成功取得；不能把此次预览称为客户端生命周期全部通过。关闭记录在 `preview-stop.json` / `preview-exit.json`，后续接入时需要复查客户端退出问题。

## 本机候选的实际情况

| 世界 | 当前检查结果 |
| --- | --- |
| PX4 `city.world` | 原始世界实际加载并截图；优先接入的街区场景 |
| PX4 `small_city.world` | 更大的街区布局，但本机缺少 `city_terrain`、`gazebo` 直接引用资源；尚未加载验证 |
| PX4 `warehouse.world` | 直接引用资源存在；含货架、托盘和箱子；尚未加载验证；部分货架引用 `no_collision` 资产，不能直接当作完整导航地图 |
| PX4 `baylands.world` | 直接引用模型存在；尚未验证渲染、地形和导航兼容性 |
| PX4 `yosemite.world` | 本机缺少同名模型，暂不作为离线可直接运行的候选 |
| Gazebo 安装包 `cafe.world` / `willowgarage.world` | 世界文件存在；尚未核验资源和飞行适用性 |

## 自主飞行接入前的具体工作

1. 保留原世界副本及来源哈希，不修改 PX4 安装目录。`city.world` 有 25 个顶层模型、12 个网格碰撞和 49 个基本体碰撞。当前 `collision_boxes()` 不支持网格，且要求 `static=true`；原世界使用 `static=1`。不可跳过未支持碰撞或直接注册为空障碍场景。
2. 采用可审计的场景导入方式，保留原视觉网格和贴图。核验网格缩放、模型/链接/碰撞变换及完整障碍范围；若采用保守包围体，明确记录近似及不能穿行的区域。不能使用视觉目标真值作为语义答案。
3. 处理原文件的历史 `<state>`：当前加载报告其中引用了不存在的 `ground_plane`。飞行副本应使用明确的初始状态，不能继承保存的仿真时间或意外位姿。客户端另报告三个法线贴图路径未找到，实际漫反射贴图可见；保留原日志，不能称为无警告加载。
4. 核验真实地面高度、起降净空、许可范围、PX4 lockstep 物理设置和地图注册；建立可信 manifest 后才接入安全执行器。场景更大不自动扩大当前 12 米搜索半径。
5. 先地面相机/VIO检查，再最小起飞，最后做原场景中可见物体的自主搜索。物体种类与任务应符合真实相机画面，不强求原场景本来没有的白色货车。目标确认、任务结果、实际落地上锁分别记录。

本次只完成原世界的真实视觉预览和兼容性检查；没有修改生产搜索、导航或飞控逻辑，没有新增飞行回归通过记录。此前成功基线及已知类型异常的维护决策继续有效。
