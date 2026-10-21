# 从 GitHub 源码准备运行环境

本仓库保存 Agent、ROS 执行器、仿真场景生成器、launch 配置和已修改的 VINS-Fusion 源码。PX4、ROS、Gazebo、Python 虚拟环境、编译产物、运行日志和模型密钥需要在目标机器上单独准备。历史报告中的成功指标对应原机器的指定运行，不代表克隆后已经通过验收。

## 1. 环境与外部依赖

原开发环境为 Ubuntu 20.04、ROS Noetic、Gazebo Classic 11.15.1、PX4 v1.14，VINS 构建使用 OpenCV 4.2、Ceres 1.14、Eigen 和 catkin。ROS 侧使用系统 `/usr/bin/python3`（Python 3.8）；LangChain Agent 单独使用 Python 3.11 虚拟环境。不要把 ROS Python 解释器替换成 Agent 的解释器。

需要先按各上游项目说明安装、编译并验证：

- ROS Noetic 的 `roslaunch`、`rospy`、`roscpp`、`gazebo_ros`、`mavros`、`mavros_msgs`、`cv_bridge`、`image_transport` 及消息包；MAVROS 所需 GeographicLib 数据也应安装。
- PX4 v1.14 及其 Gazebo Classic 子模块和 SITL 构建依赖；目标目录中应有 `build/px4_sitl_default/bin/px4`、`px4-mavlink` 和 `build_gazebo-classic/libsensor_msgs.so`。
- 系统构建工具、CMake、catkin、Gazebo 开发库、OpenCV、Ceres、Eigen、Boost，以及场景生成器使用的系统 Python OpenCV 和 NumPy。
- 支持图像输入与工具调用的模型接口；网页本身不需要 Node 构建，Node/Playwright 仅用于浏览器测试。

这些外部依赖没有包含在本仓库中。相关来源及许可说明见 [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)。

## 2. 克隆和本机路径

```bash
git clone https://github.com/HonEYFIsh24/px4_agent.git px4_agent
cd px4_agent
export PX4_DIR=/absolute/path/to/PX4-Autopilot
```

开发机的 PX4 还包含四个已跟踪文件的修改，已单独导出为 [PX4 源码补丁](../patches/px4/README.md)。若要复现这套实现，先准备与清单基点 `b8c541dd7277ed735139d7d1bfb829d61fbe29fb` 匹配、无冲突的 PX4 checkout，再核对并应用：

```bash
git -C "$PX4_DIR" apply --check "$PWD/patches/px4/local-sitl-and-cli.patch"
git -C "$PX4_DIR" apply "$PWD/patches/px4/local-sitl-and-cli.patch"
make -C "$PX4_DIR" px4_sitl_default
```

已有相同修改时不要重复应用。按 PX4 的依赖要求另外准备 Gazebo Classic 插件构建，确认本节列出的消息库已存在，再构建本工程 VIO。补丁、基点和源文件哈希见 `patches/px4/source_manifest.json`；源码相符不代表新机已经通过飞行验收。

入口脚本和 `uav_system.launch` 接受 `PX4_DIR`／`px4_dir`，但当前工程尚未完全消除原开发机路径：

| 位置 | 新机器需要核对的内容 |
|---|---|
| `vio/generate_assets.py` | `PX4 = Path('/home/dut/PX4-Autopilot')`，不会自动读取 `PX4_DIR`；若 PX4 安装位置不同，需要先调整该源码常量 |
| `vio_ws/src/uav_body_imu_bridge/CMakeLists.txt` | `PX4_GAZEBO_BUILD` 默认指向原机 PX4 构建目录，可用 CMake cache 参数覆盖 |
| 已生成的 SDF、world、YAML、manifest | 可能包含原机器资源或输出的绝对路径，应在新机重新生成 |
| 文档和历史验收入口 | 某些示例或实验脚本保留 `/home/dut/...` 路径，执行前按新机实际路径核对 |

仅设置 `PX4_DIR` 不足以解决所有路径问题。首次移植建议先搜索 `rg -n '/home/dut' scripts launch vio scene`，核对将要使用的文件，而不要批量替换全部实验记录。

## 3. Agent 环境与模型配置

下面的 `python3.11` 应指向已安装的 Python 3.11 解释器；Ubuntu 20.04 的系统 Python 仍保留给 ROS。

```bash
python3.11 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
cp .env.example .env
```

只在本机编辑 `.env`，配置 `VLM_MODEL`、`VLM_BASE_URL` 和 `VLM_API_KEY`；默认架构为 `single_vlm`。不要把实际密钥放进 `.env.example`、源码、命令参数或 Git 提交。启动时显式传入本项目 `.env`，可避免继承原机器的其他配置文件。

没有模型密钥时，网页可显示遥测和相机，自然语言任务需要先补全模型配置。外部接口的鉴权、配额和当前可用性需要实际验证；历史 Qwen 调用记录不保证新账号可用。

## 4. VIO 构建与场景生成

本仓库保留已修改的 `vio_ws/src/VINS-Fusion`、补丁和 GPL-3.0 许可证。随源码的 `.download_complete` 是下载脚本用于保护本地补丁的清单标记，应一并保留；若在已修改源码上删除它，下载脚本可能重新下载并覆盖修改。

PX4 Gazebo 依赖已构建且路径为原默认值时：

```bash
bash scripts/build_vio.sh
```

如果 PX4 构建目录不同，可先按源码说明检查补丁，再直接用系统 ROS 环境构建并覆盖插件依赖位置：

```bash
source /opt/ros/noetic/setup.bash
/usr/bin/python3 vio/patch_upstream.py
catkin_make -C "$PWD/vio_ws" -j2 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=Release \
  -DPX4_GAZEBO_BUILD="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic" \
  --make-args vins_node uav_body_imu_plugin
```

在修正 `vio/generate_assets.py` 的 PX4 路径之后，生成与本机路径一致的模型、标定和场景：

```bash
/usr/bin/python3 vio/generate_assets.py
/usr/bin/python3 scene/generate_world.py
/usr/bin/python3 scene/generate_obstacle_world.py
```

首次构建不应从仓库复制旧 `build/`、`devel/` 或 `.so`。VINS 的历史性能、相机同步和外部视觉融合应在新机另行验收。

## 5. 统一启动和停止

在 Ubuntu 图形桌面终端启动，确保没有旧 PX4、Gazebo 或 VINS 实例占用端口：

```bash
source scripts/setup_ros.bash
roslaunch --sigint-timeout 180 px4_agent uav_system.launch \
  px4_dir:="$PX4_DIR" env_file:="$PWD/.env" gui:=true
```

默认打开 Gazebo 三维窗口，网页为 <http://127.0.0.1:8080>，相机预览位于网页“机载相机”面板。默认 `ev_profile:=shadow`，用于独立观测 VIO；它不等于无 GPS 飞行。`gps_fused` 和 `no_gps` 应在输入同步、地面质量和实际融合检查完成后单独验证。默认 `gazebo_sync:=false` 使用系统 Gazebo 库；可选私有物理后端的移植限制见 [gazebo_sync/README.md](../vio/gazebo_sync/README.md)。

前台按 `Ctrl+C` 后等待管理器完成降落、上锁和逐项关闭。后台实例使用其实际运行目录停止：

```bash
bash scripts/stop_system.sh /absolute/path/to/px4_agent/logs/system/<run-directory>
```

运行目录由启动器输出。停止脚本核对进程身份；不要用旧 PID 或全局 `pkill` 替代正常退出。

## 6. 验证与仓库范围

先验证地面遥测、真实左右相机输入、VIO 初始化和状态有效性，再发起基础飞行。离线 Python 测试可与真实飞行分开执行：

```bash
.venv/bin/python -m unittest discover -s tests -v
```

ROS 专用测试使用系统 Python 和 ROS 环境。带有 `--execute` 的飞行／模型脚本会实际执行任务或调用付费接口，需先阅读对应文档并确认已启动的是本次实例。

`logs/`、运行图像／bag、虚拟环境和编译产物不属于源码上传范围。因此旧文档中指向 `logs/...` 的证据链接在克隆后可能不可用；需要原始证据时单独归档传递。当前验收范围和模型超时阻断记录见 [实施进度](ordered_implementation_status.md) 和 [VIO 融合与搜索](vio_fusion_and_search.md)。
