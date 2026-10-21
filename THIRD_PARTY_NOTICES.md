# 第三方来源与许可说明

本文件说明仓库内的第三方源码及主要外部依赖。根 [LICENSE](LICENSE) 是仓库所有者在 GitHub 初始化时选定的 MIT 许可；其适用范围不替代 VINS-Fusion 的 GPL-3.0 或其他上游组件的原有许可证、版权标记和文件级声明。根 `package.xml` 中的 `Unspecified` 是既有元数据，本次没有改写该字段、上游许可或版权声明。

| 组件／内容 | 来源与本仓库中的位置 | 已保留的许可信息 |
|---|---|---|
| VINS-Fusion 的 `camera_models`、`vins_estimator` 及本项目修改 | [HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)，源码位于 `vio_ws/src/VINS-Fusion/`，上游文件清单见 `vio/upstream_manifest.json` | [上游 GNU GPL version 3 全文](vio_ws/src/VINS-Fusion/LICENSE)；原文件版权声明保留。当前源码包含兼容、输入健康检查、初始化及诊断等本地修改，不能等同于未修改的上游版本 |
| Gazebo Classic 物理同步补丁 | 上游 [gazebo-classic](https://github.com/gazebosim/gazebo-classic) 的 `gazebo11_11.15.1`，补丁位于 `vio/gazebo_sync/patches/atomic_ray_update_sync.patch` | 已逐字复制上游 [LICENSE](vio/gazebo_sync/licenses/Gazebo-LICENSE) 和 Apache License 2.0 [COPYING](vio/gazebo_sync/licenses/Gazebo-COPYING)；补丁来源、校验值与构建边界见 [说明](vio/gazebo_sync/README.md) |
| `uav_body_imu_bridge` | `vio_ws/src/uav_body_imu_bridge/`，本工程的原机 IMU 转发 ROS 包 | 该包的 `package.xml` 声明 `BSD-3-Clause`；目前目录中没有单独的 LICENSE 文件，本文件不补写未确认的版权人或授权文本 |
| PX4 本机 SITL／CLI 源码差异 | [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)，`patches/px4/local-sitl-and-cli.patch`；基点和源文件哈希见同目录 `source_manifest.json` | 上游 BSD 3-Clause 许可原文保存在 [PX4-LICENSE](patches/px4/PX4-LICENSE)；来源和应用说明见 [PX4 补丁说明](patches/px4/README.md) |
| Iris 派生 SDF 及 PX4 相关模型资源 | `vio/generate_assets.py` 从外部 PX4 Gazebo Classic 的 `models/iris/iris.sdf` 生成 `vio/models/iris_vio/iris_vio.sdf`；完整 PX4 不随仓库提供 | 来源为外部 [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) 及其 Gazebo Classic 子模块；使用模型和资源时保留对应来源仓库、目录和文件的许可声明，不能用本工程根许可替代 |

运行时还依赖 ROS Noetic、MAVROS、Gazebo Classic、OpenCV、Ceres、Eigen、Boost、Python、LangChain、LangGraph、Pydantic、python-dotenv，以及浏览器测试使用的 Node／Playwright 等。这些依赖由系统包、`requirements.txt`、`package.json` 和 `package-lock.json` 安装，其许可证以各依赖实际版本随附的声明为准；未把它们重新声明为本工程所有。

日志、图像、私有 Gazebo 动态库和其他构建产物不随本次源码仓库上传。若以后另外分发二进制或第三方资产，需要一并保留相应组件的许可及来源资料。私有 GitHub 仓库的访问权限不改变上述组件的许可证。
