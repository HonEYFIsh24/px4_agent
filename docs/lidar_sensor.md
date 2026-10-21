# 水平 360° LiDAR 传感器

Iris 已增加真实 Gazebo CPU `ray` 传感器，通过本机已安装的 `libgazebo_ros_laser.so` 发布 `/uav/lidar/scan`。数据来自 Gazebo 碰撞射线，与场景对象配置或模型视觉推断独立。它用于 **ENU 高度 3.5 m 下的静态二维绕障实验**。

## 消息与几何合同

| 项目 | 配置 / 含义 |
| --- | --- |
| ROS 话题 | `/uav/lidar/scan` |
| 消息类型 | `sensor_msgs/LaserScan` |
| `header.frame_id` | `uav_lidar`，与 SDF link 同名 |
| 坐标轴 | FLU：X 向前，Y 向左，Z 向上；0° 朝 X 正方向，正角朝 Y 正方向 |
| 安装外参 | `base_link → uav_lidar` 平移 `[0, 0, 0.35] m`，旋转单位矩阵 |
| 相对本体 IMU | 原 `/imu_link` 位于 base Z=0.02 m，因此 `imu_link → uav_lidar` 平移 `[0,0,0.33] m` |
| 水平角 | `angle_min=-π`、`angle_max=π`，包含两个端点 |
| 水平样本 | 360；分辨率 1，不插值为更多样本 |
| `angle_increment` | `2π / 359 ≈ 0.01750191 rad`，约 1.00279° |
| 垂直线 | 1 条，垂直角为 0 |
| 仿真采集频率 | 10 Hz，即相邻正常采集时间戳间隔约 0.1 仿真秒 |
| 测距 | 0.2–20 m；range resolution 设置 0.01 m |
| 噪声 | 零均值高斯，标准差 0.01 m；属于实验设置，不是特定实物 LiDAR 的标定参数 |
| 理论射线预算 | 3600 条 / 仿真秒；0.35 倍实时率约 1260 条 / 墙钟秒 |

`ranges[i]` 对应 `angle_min + i * angle_increment`，扫描点在传感器坐标系为 `[r*cos(angle), r*sin(angle), 0]`。首尾样本都是背向射线，不能视为两条独立方向。Gazebo 的射线生成先旋转 X 正方向，角度以 Z 正轴为旋转轴，和 ROS 的 LaserScan 约定一致。[Gazebo 11 射线实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/MultiRayShape.cc)，[ROS LaserScan 定义](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/LaserScan.msg)。

本机 `ros-noetic-gazebo-plugins` 为 `2.9.3-1focal.20250521.004734`。相同版本插件读取 `topicName`、`frameName`，使用 Gazebo 的采集时间戳和角度、测距数组。它将 `time_increment`、`scan_time` 都置为 **0**，因此频率和新鲜度应使用相邻 `header.stamp` 与实际接收时间判断。插件填写 frame 名字，但 **不广播 TF**；下游必须使用上述固定外参与同步机体位姿。它按 ROS 订阅连接接入 Gazebo 扫描，运行验收需要实际订阅该话题。[插件 2.9.3 源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_laser.cpp)。

Gazebo 把无回波 / 达到最大量程的射线表示为 `+inf`，过近射线表示为 `-inf`；插件直接转发。有限且在有效范围内的值才可作为障碍端点，`+inf` 可在对应射线方向提供截至量程的无回波信息，`-inf` 和 NaN 需要按无效或过近危险处理，不能编码成虚假的远距离。[Gazebo RaySensor 实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/sensors/RaySensor.cc)。

## 本机自回波与现有传感器

原 Iris 的机体 collision 高度范围是 base Z=±0.055 m；四个桨叶 cylinder 的顶部为 Z=0.0255 m。LiDAR 的 Z=0.35 m 水平射线平面高于这些碰撞体。新增壳体只包含 visual，位于扫描平面下方，没有会遮挡扫描的新增 collision。该结果是对 **SDF 碰撞几何** 的静态核验，实际扫描仍应检查近距离环状自回波。

原 PX4 IMU、MAVLink / 电机插件和碰撞体内容保持原样。双目仍为 640×480、20 Hz、90° 水平视场角、0.12 m 基线和 0.35 rad 下俯角；原本体 IMU ROS 插件继续使用 `/uav/imu/data_raw`。LiDAR 固定 link 增加 0.01 kg 质量，重新启动后的飞行与定位验证仍需完成。

## 当前二维实验边界

安装平面随无人机的 roll / pitch 一起倾斜，不等于恒定世界水平面。ENU Z=3.5 m 也不等于扫描面的世界 Z：本机世界 / ENU 高度存在偏移，加上 0.35 m 安装高度，扫描面可能位于世界 Z≈4 m。实验阻挡设施必须覆盖该扫描高度；低矮车辆、箱子、集装箱、地面和上下方障碍可能没有回波。

下游应在起飞到实验高度后重置二维地图，用同步完整四元数变换0.35米安装偏移，再用近水平姿态的yaw将射线投影到二维XY；这是平面近似，不是完整三维点云变换，并对扫描新鲜度、姿态倾角和高度设置有效性检查。当前总倾角门限为0.2 rad（由机体Z轴与竖直轴夹角计算），建图范围保守截至5米，且机体ENU高度需在3.5±0.25米；超出门限的扫描不应继续更新平面地图。此传感器配置本身没有实现路径规划、轨迹跟踪安全保证、动态避障或三维避障。

## 文件与离线检查

- 生成器：`vio/generate_assets.py`。
- 模型：`vio/models/iris_vio/iris_vio.sdf`。
- 模型增量更新：`python3 vio/generate_assets.py --model-only`。该入口只写 Iris SDF，不重写旧世界、纹理或 VINS 配置。
- SDF 结构检查：本机 Iris mesh 使用 `model://iris`，先为检查器设置本地模型搜索路径，避免联网查模型库：

  ```bash
  GAZEBO_MODEL_DATABASE_URI='' \
  GAZEBO_MODEL_PATH=/home/dut/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models \
  timeout 20 gz sdf -k vio/models/iris_vio/iris_vio.sdf
  ```

  本次退出码为 0、输出 `Check complete`；检查器同时给出原 Iris SDF 根 `version` 属性的兼容提示，没有新增 LiDAR 字段错误。
- 传感器合同检查：`python3 -m unittest discover -s tests -p test_lidar_sensor.py -v`。这些测试不启动仿真。

实际扫描取证见 `logs/avoidance/lidar_probe.json`：360 个角采样，连续时间戳间隔为 0.1 仿真秒，地面最近有效回波约 3.99 m，未见近距离机体自回波。运行时额外用同步的无人机自身世界位姿检查 LiDAR 原点 Z 在 `[2.5,4.3] m`；这只是高墙二维仿真的高度监督，不读取障碍真值，也不提供真实环境的三维保护。真实任务的成功与失败、轨迹及净空分别保存在 [自主绕障联调记录](autonomous_avoidance.md)，不能仅凭传感器合同检查宣称任务完成。
