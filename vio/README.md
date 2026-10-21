# Iris 双目摄像头 + 机载 IMU 的 VINS-Fusion VIO

现有 Iris 已加装同步双目相机，运行 VINS-Fusion 的 stereo + IMU 模式。输入仅为左右图像和飞机原有 IMU 的原始加速度、角速度。GNSS、MAVROS 本地位置、飞控姿态估计和 Gazebo 真值均不进入 VINS-Fusion。双目提供几何尺度，可在静止时初始化；飞控仍使用 PX4 原有本地定位闭环。

```mermaid
flowchart LR
    C[同步双目相机] -->|左右 image_raw| V[VINS-Fusion]
    I[Iris 原有 IMU] --> B[Gazebo 测量转发]
    B -->|/uav/imu/data_raw| V
    I --> P[PX4 飞控]
    V --> O[/vins_estimator/odometry]
    O --> A[Agent 状态查询]
```

## 模型与标定

| 项目 | 配置 |
|---|---|
| 双目相机 | 每路 640×480、20 Hz、水平视场 90°、无畸变 |
| 同步 | 同一 Gazebo MultiCamera sensor，同一测量时间戳 |
| 物理基线 | 0.12 米；右侧 CameraInfo 的 `P[3]=-38.4` |
| 安装位置 | 左右相机相对基座 `(0.12, ±0.06, 0.18)` 米，俯角约 20° |
| IMU | 原有 Iris IMU，直接转发原始插件测量到 `/uav/imu/data_raw`，约 250 Hz |
| 内参 | 两路均为 `fx=fy=320`、`cx=320`、`cy=240` |
| 光学坐标方向 | x 向右、y 向下、z 向前 |
| IMU 坐标 | ROS FLU：x 向前、y 向左、z 向上 |
| 相机到 IMU 平移 | `(0.12, ±0.06, 0.16)` 米；原 IMU 位于基座上方 0.02 米 |
| 时间 | 测量时间戳与 Gazebo 仿真时钟一致，固定 `td=0` |

`generate_assets.py` 在项目内生成带相机的 Iris SDF、标定文件和有纹理的测试场景，保留 PX4 上游模型。新模型名仍为 `iris`，可复用现有执行器。`libgazebo_ros_multicamera.so` 两路消息使用公共 `uav_stereo_optical` 坐标方向；相机中心分别由右侧投影矩阵以及 VINS 外参描述。

带相机的入口使用项目内 `launch/sitl_vio.launch`，向 Gazebo server 传入 `--lockstep`，让物理更新等待传感器渲染，保持相机的仿真采样频率；计算负载高时仿真实时速度可能降低，见 [Gazebo 官方说明](https://classic.gazebosim.org/tutorials?cat=sensors&tut=lockstep_physics_sensors)。这与 Iris 内 PX4 MAVLink 插件的 `enable_lockstep` 是两个设置。VINS 的双目帧对入口也丢弃重复和倒序时间戳，防止特征速度计算中的零时间间隔污染优化。

整套 `uav_system.launch` 默认打开 GUI，并通过 `PX4_SIM_SPEED_FACTOR=0.35` 给 VIO 计算预留时间；`gui:=false` 默认速度为 1，可用 `sim_speed_factor:=1` 显式覆盖。该设置缩放实际运行速度，仿真时间内的相机 20 Hz、IMU 250 Hz 不变。本机 VINS 启动时将 OpenCV 工作线程设为 2，OpenBLAS / OpenMP 各限制为 1，减少与 Gazebo 图形渲染争用 CPU；日志会输出实际 OpenCV 线程数。特征数量、反向跟踪、优化参数及定位有效性门限均保留。整套管理器同时把 PX4→MAVROS 的 `HEARTBEAT` 调为仿真时间 5 Hz，保留执行器的 3 秒现实时间遥测新鲜度要求。

`config/stereo_imu.yaml` 的 `body_T_cam0` 和 `body_T_cam1` 将各自相机光学坐标变换到原有 IMU 坐标。相机安装改变时，应同时更新模型和外参；启动脚本会重新生成文件，因此应修改生成器。

## 启动

推荐直接启动整套仿真、VIO 和网页，默认打开 Gazebo 三维窗口：

```bash
bash /home/dut/px4_agent/scripts/launch_system.sh
```

标准 ROS launch 命令、参数和退出方法见 [项目启动说明](../README.md#整套-launch-启动默认打开-gazebo-窗口)。先关闭旧仿真及控制节点，避免端口冲突；不要与单独启动的节点同时运行。

需要分别管理服务时，依次在不同终端运行：

```bash
cd /home/dut/px4_agent
GUI=true PX4_SIM_SPEED_FACTOR=0.35 bash scripts/start_sitl_vio.sh
```

```bash
cd /home/dut/px4_agent
/home/dut/PX4-Autopilot/build/px4_sitl_default/bin/px4-mavlink stream -u 14580 -s HEARTBEAT -r 5
bash scripts/start_bridge.sh
```

```bash
cd /home/dut/px4_agent
bash scripts/start_vio.sh
```

需要网页控制与相机预览时，再单独运行 `bash scripts/start_web.sh`。默认页面为 <http://127.0.0.1:8080>；[左相机 JPEG](http://127.0.0.1:8080/api/camera/left.jpg)、[右相机 JPEG](http://127.0.0.1:8080/api/camera/right.jpg) 可直接查看。网页每路预览上限 4 Hz，不改变 VINS 使用的原始双目输入。图像过期后隐藏旧帧，恢复后自动重连。网页飞行指令失败后不会自动重试，需要核对飞行状态并重新发布指令；仍可查看状态或使用停止、降落按钮。

`start_sitl_vio.sh` 在 Gazebo 进程内加载原始 IMU 转发插件；`start_vio.sh` 管理 VINS-Fusion，退出时关闭它，不影响仿真。启动 VINS 后等待 5 秒摄像头预热，再检查左右内参、物理基线、实际频率、共同仿真时钟和双目精确同步，失败时等待 5 秒重试一次，详见 `logs/vio/inputs.json`。两次启动诊断失败会明确打印 `WARNING`，保留仍在运行的估计器，此时应查看网页中的 VIO 状态。单独运行 `bash scripts/start_vio.sh --check` 是严格输入验收，任何未达到原有指标的输入都会使该命令失败退出。Gazebo 可能先发送一帧历史缓存图像，检查及 VINS 图像入口会丢弃过期数据。

这里没有新增 IMU，也不修改原 Iris IMU 插件：`uav_body_imu_bridge` 包的 `libuav_body_imu_plugin.so` 在 Gazebo 进程内订阅它的 transport 消息 `~/iris/imu`，仅转发原始加速度、角速度及传感器时间戳，保留 FLU 方向及原有噪声/偏置。原消息自带的仿真姿态被丢弃，ROS `orientation_covariance[0]=-1` 标记姿态未知。转发必须与 Gazebo 同进程，避免额外 transport 客户端阻塞 PX4 lockstep 的 IMU 接收。PX4 v1.14 的 HIGHRES_IMU 会减去 EKF 估计偏置，所以 `/mavros/imu/data_raw` 不用于本版本的独立 VIO。

输出与记录：

- 左右图像：`/uav/camera/left/image_raw`、`/uav/camera/right/image_raw`
- 标定：`/uav/camera/left/camera_info`、`/uav/camera/right/camera_info`
- 优化里程计：`/vins_estimator/odometry`，约 10 Hz；上游多线程模式每两帧处理一次。
- IMU 传播里程计：`/vins_estimator/imu_propagate`；跟踪图：`/vins_estimator/image_track`
- 初始化/失效状态：`/vins_estimator/tracking_ok`
- 优化偏置：`/vins_estimator/imu_bias`，数组依次为三轴加速度偏置（m/s²）、三轴角速度偏置（rad/s）
- 日志：`logs/vio/vins.log`、`logs/vio/vio.csv`
- 原始 IMU 转发插件日志：Gazebo 的 ROS 日志及仿真启动日志
- 左右画面预览：`logs/vio/stereo_preview.png`

Agent 使用原有启动命令，状态新增 `vio` 字段。`ready` 同时检查优化状态、输出及测量时间戳的新鲜程度、有限值、单位四元数和本演示的速度/距离界限。等待初始化、重置或输出异常时为 `false`，不向 Agent 返回异常坐标。这些检查不能单独证明绝对精度。VIO 的 `world` 原点和初始航向独立于 MAVROS ENU，重初始化后也会改变，不能直接作为现有 ENU 航点。

## 飞行验证

ROS Python 3.8 下的 26 项单元测试已通过，命令为 `/usr/bin/python3 -m unittest discover -s tests -v`；它们覆盖任务边界、VIO 健康状态及网页执行行为。VIO 精度仍需以下实际仿真验证。

仿真、执行器和 VINS 均运行且飞机在地面时执行：

```bash
cd /home/dut/px4_agent
bash scripts/test_vio_flight.sh
```

测试执行起飞、水平与高度变化、返航、悬停和降落，然后观察落地静止 30 秒。Gazebo 真值仅作为离线评价基准；比较只对齐首次有效位姿的平移和初始航向，不修正尺度。

也可在已着陆、未解锁时单独检查初始化后的 30 秒静止稳定性：

```bash
source /opt/ros/noetic/setup.bash
cd /home/dut/px4_agent
/usr/bin/python3 vio/check_stationary.py
```

该检查保存 `logs/vio/stationary_validation.json` 和 `stationary_samples.json`，要求最大位移小于 0.3 米、至少 250 个优化位姿、无失效消息且 Agent 最终状态有效。

通过条件为位置 RMSE 小于 0.5 米、最大误差小于 1 米、静止漂移小于 0.3 米、跟踪未重置且最终健康状态有效。`validation_passed` 标记整体验证结果，不能只看飞行任务是否成功。

- 报告：`logs/vio/validation.json`
- 原始轨迹及状态：`logs/vio/trajectory_samples.json`
- 误差和轨迹图：`logs/vio/trajectory_validation.png`

此前原机 IMU 的独立 VIO 验收记录：2026-10-03 的完整九步任务 `e88ad5dd4d42` 覆盖 80.6 秒仿真时间，输出 807 个优化位姿（10 Hz），位置 RMSE 0.1065 米、最大误差 0.1522 米；落地后 30 秒漂移 0.0105 米，无跟踪失效消息，最终已着陆、未解锁且 Agent VIO 有效。初始化后的独立静止段最大位移 0.00272 米。该次验收使用 `/uav/imu/data_raw`；当前运行的任务编号和指标以最新 `logs/vio/validation.json` 为准，输入连接证据保存为 `logs/vio/input_graph.txt`。

## 编译与源码改动

独立工作区为 `vio_ws`，编译 `camera_models`、`vins` 和 `uav_body_imu_bridge`，不启用 GPS 全局融合或回环模块。使用本机 ROS Noetic、OpenCV 4.2 和 Ceres 1.14。IMU 转发插件链接本机 PX4 Gazebo 已编译的原始消息定义库。

```bash
cd /home/dut/px4_agent
bash scripts/build_vio.sh
```

官方源文件按 `upstream_manifest.json` 中的 Git blob SHA-1 校验。首次安装因下载连接问题通过 GitHub 连接器获取源码；本地源码及完成标记保留，再次编译无需网络。`download_upstream.py` 提供按清单下载接口，源文件版权标记和 GPL-3 许可证保留。

`patch_upstream.py` 记录 OpenCV 4 常量兼容、里程计 body frame 命名、过期图像过滤和 ROS 线程退出处理，以及失效检测。上游 `failureDetection()` 首行直接返回 `false`；本机启用有限值、IMU 偏置、特征数及演示速度/距离检查，短暂视觉丢失允许最多 1.5 秒的 IMU 传播，持续丢失则重初始化。清空状态在释放处理锁后进行，以避免重复加锁。最终版本使用上游双目惯性优化，不包含单目试验中的零速度先验。

静止双目不能完全区分重力倾角与加速度计偏置，因此补丁为初始六轴偏置加入零均值高斯先验。加速度标准差 0.196 m/s²、角速度标准差 0.0087 rad/s 均来自原 Iris SDF 的 turn-on bias 分布，并非实际偏置或姿态真值。残差为 `bias / sigma`；先验只在尚无有效边缘化信息时加入初始状态，随后随正常 IMU 因子和边缘化传播，后续偏置仍参与优化和随机游走。它不锁定速度或姿态，不使用落地状态。新编译必须重启 VINS 才能生效，启动日志应显示这两个标准差。

当前参数面向低速、有纹理的仿真场景。弱纹理、动态物体和快速飞行需要另外验证，当前没有回环、长期漂移修正、目标识别或避障。

## 与飞控的关系

已接通独立 VIO 和 Agent 状态反馈。VIO 尚未输入 PX4 EKF2，GNSS 融合仍开启，因此当前飞行验证不代表无 GPS 飞行已经验证。切换到 VIO 闭环还需外部视觉坐标转换、EKF2 融合配置和定位失效行为验证。

参考：[VINS-Fusion 官方项目](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)、[PX4 外部视觉接口](https://docs.px4.io/v1.14/en/ros/external_position_estimation)。
