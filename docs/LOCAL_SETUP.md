# 本机配置记录

项目目录：`/home/fxxk/px4_agent`。PX4 目录：`/home/fxxk/PX4-Autopilot`。

## 已完成

- 启动入口默认从当前用户主目录查找 PX4，仍支持 `PX4_DIR` 覆盖。
- VIO 资源生成器读取 `PX4_DIR`；构建脚本将 PX4 消息库目录传给 CMake。
- 使用本机 ROS Noetic、Gazebo 11.15.1、OpenCV 4.2 C++ 库及 Ceres 1.14 完成 VINS-Fusion 和机载 IMU 插件构建。
- 已重新生成本机双目相机模型、标定、物流园、障碍场景和可靠性场景。
- 已创建 `.env`，权限为 0600，密钥为空，Git 忽略该文件。
- ROS 包查找、两套 launch 的节点解析、动态库依赖检查和修改脚本的 Bash 语法检查通过。
- VINS 原生因子、传感器门限及补丁幂等性两项测试通过。

## Agent 环境与启动

初次配置时，执行会话的沙箱禁止创建网络 socket。后续通过自动审核允许的网络安装，已完成 Agent 依赖修复和本机 HTTP 回归测试；完整仿真的显示环境准备仍未在本会话中通过。

ROS 保留使用系统 Python 3.8，Agent 的独立 Python 3.11 `.venv` 已安装。重新安装时，下列脚本优先使用已有 Python 3.11；否则在项目 `.tools` 安装 uv 并下载独立 Python 3.11，再创建 `.venv` 安装固定的 Agent 依赖。无需 sudo，不替换系统 Python：

```bash
cd /home/fxxk/px4_agent
bash scripts/setup_agent.sh
```

在本机编辑 `.env`，填写 `VLM_API_KEY`（或现有的 `LLM_API_KEY`）。默认使用 DashScope 接口和 `qwen3-vl-plus`；更换提供商时同时设置相应的 `VLM_BASE_URL` 与 `VLM_MODEL`。密钥留在本机文件内。

在图形桌面终端启动：

```bash
cd /home/fxxk/px4_agent
bash scripts/launch_system.sh env_file:=/home/fxxk/px4_agent/.env
```

网页地址：<http://127.0.0.1:8080>。退出时按 Ctrl+C，等待管理器完成关闭。当前尚未通过完整启动或飞行验收。

## 版本与测试限制

本机 PX4 为 `v1.15.4-1-ga7659ddf12`，项目原记录使用 v1.14，补丁基点为 `b8c541dd7277ed735139d7d1bfb829d61fbe29fb`。本次没有切换现有 PX4 checkout，也没有应用项目附带补丁；现有 PX4 工作区包含原有修改。当前版本的参数、诊断话题及 CLI 行为需在实际启动后核验，不能将编译成功视为飞行兼容性通过。

首次逐文件运行 49 个非 VINS 测试模块，共 721 项：678 通过、29 跳过、11 错误、3 失败。受限 socket 导致 HTTP/XMLRPC 和 Gazebo 检查无法运行；其余问题包括历史路径、旧相机布局断言、依赖不存在的历史飞行日志，以及固定原机场景哈希。重建可靠性场景后，可重复生成检查已通过，固定原机哈希断言仍失败。未为通过测试而伪造飞行日志或修改历史验收条件。

标准 `unittest discover` 另有 `tests` / `scripts` 同名模块导入冲突，因此首次使用独立进程逐文件运行。

## 网页启动崩溃修复

2026-10-22 的运行 `20261022-174742-3SBuFZ` 已启动 PX4、MAVROS、执行器、VIO，并通过双目和 IMU 输入检查。随后网页因 `ImportError: cannot import name 'ExecutionInfo' from 'langgraph.runtime'` 退出，管理器因此关闭整套服务。

原因是固定的 `langgraph==1.0.4` 与解析到的 `langgraph-prebuilt==1.0.13` 运行时不兼容。已在 `requirements.txt` 固定 `langgraph-prebuilt==1.0.5`，并在现有 `.venv` 完成降级。安装脚本增加真实模块导入检查，系统启动管理器增加 Agent CLI 预检查。

修复后，依赖元数据检查、Agent/网页导入和 CLI 帮助入口均通过。单 VLM 38 项、配置 10 项及网页 HTTP 11 项测试全部通过；日志分别为 `logs/setup/single-vlm-fixed.log`、`agent-config-fixed.log` 和 `web-fixed.log`。完整系统的后续验证因本会话显示准备失败而停止，需在用户图形桌面终端重试上述启动命令。

构建、生成、安装尝试和测试原始输出均在 `logs/setup/`，不提交到 Git。

## 桌面入口与实际启动验证

原生窗口的 9 项数据及交互测试已通过。快捷方式已注册到 `~/.local/share/applications/px4-agent.desktop`，按 Super 搜索“PX4 桌面控制台”即可启动。桌面同名快捷方式保留；某些文件管理器会将 `.desktop` 当文本打开，应用菜单入口不依赖该打开方式。启动器显式使用 GNOME Terminal，失败时保留终端及 `logs/desktop-launch-*.log`。

2026-10-22 已从图形桌面环境实际启动整套桌面模式，运行目录为 `logs/system/20261022-181343-uLR5HC`。管理器报告桌面窗口已打开；记录中飞控已连接、定位就绪、VIO 有效、飞机已落地且上锁。此次没有发送飞行任务或调用模型，不属于新的飞行验收。

## 模型接口连接诊断

本机系统 DNS 将 `dashscope.aliyuncs.com` 解析为 `198.18.0.12`，Sparkle 使用 fake-IP DNS。直接连接及本机 7897 代理的测试出现 `SSL: UNEXPECTED_EOF_WHILE_READING`。7890 代理曾成功返回模型列表 HTTP 200，确认本机密钥有效且配置的 `qwen3.7-plus` 存在；重复连接仍有间歇性 TLS 失败。

Sparkle 日志 `.config/sparkle/logs/2026-10-22.log` 显示，同一域名的连接分别命中 `GeoIP(cn) using DIRECT` 与 `dial 节点选择 (match Match/) ... unexpected EOF`。证据指向代理路由/节点问题，而非模型名或密钥错误。建议在 Sparkle 的持久自定义规则最前方加入 `DOMAIN,dashscope.aliyuncs.com,DIRECT` 并应用，随后重启控制台验证；本次没有修改全局代理规则或重启代理软件。

项目 `.env` 已加入 `UAV_MODEL_PROXY=http://127.0.0.1:7890`。桌面、网页、CLI 和模型诊断入口在各自 Agent 进程中加载该配置，同时合并 `NO_PROXY` / `no_proxy` 以绕过 `127.0.0.1`、`localhost` 与 `::1`。ROS/PX4 进程和系统代理设置不受该 Python 进程配置影响。设置仅对重新启动的 Agent 生效，尚不能视为已消除代理上游的间歇性失败。

### iKuuuVPN 修复完成（取代上述临时配置）

用户实际使用 iKuuuVPN。系统路由核验确认 `iKuuuVPN` TUN 接管路由表 2022，iKuuu 本地代理端口为 7897；Sparkle 只是同时残留运行的后台服务。早先仅据 Sparkle 日志将当前网络归因于 Sparkle 不够完整。

已通过 iKuuu 官方客户端的 Plugin Marketplace 安装内置 `Custom Proxy Rules` 组件，在 Installed 页打开其齿轮设置，创建 `PX4 DashScope` 组并保存唯一规则 `DOMAIN,dashscope.aliyuncs.com`。在 Servers 页选择该组，将其路由设为 `Bypass node`（DIRECT）。未切换全局规则模式、默认节点或修改订阅/账户。

项目 `.env` 最终配置为 `UAV_MODEL_PROXY=http://127.0.0.1:7897`，不再使用旧 Sparkle 的 7890 端口。本机执行器继续通过 `NO_PROXY` 绕过代理。

验证：TUN 与 7897 各连续三次鉴权查询模型列表，六次全部 HTTP 200 且包含所配置模型；随后通过正常 Agent 配置各调用一次文本和真实机载图像，分别约 1.8 秒和 2.4 秒返回成功。没有执行工具或飞行任务，图像诊断检查飞机已落地上锁。验证记录为 `logs/setup/ikuuu-route-validation.json` 和 `logs/setup/ikuuu-model-validation/diagnostics.json`。

## 桌面中文字体与输入修复（2026-10-22）

旧环境的 Tk 9 主要识别 X11 位图字体，中文显示粗糙，输入法组合输入也不可靠。桌面窗口现已改为 PySide6 / Qt 6，安装 `PySide6-Essentials==6.6.3.1`（兼容本机 Ubuntu 20.04 与 Python 3.11），使用 Noto Sans CJK SC 字体、抗锯齿和 Qt 高 DPI 支持。全界面字号调节及设置保存继续有效。

`start_desktop.sh` 在未指定输入法时使用 IBus，并清理可能由 ROS/OpenCV 注入的 Qt 5 插件目录，避免与 Qt 6 冲突。中文指令框支持拼音预编辑、候选确认及中文粘贴；Ctrl+Enter 发送。通过系统输入法切换键（通常 Super+空格）选择中文拼音即可输入。已打开的旧窗口需要关闭并重新启动快捷方式。

验证：桌面 11 项测试通过，包括中文预编辑/提交、中文字体、字号保存、并发降落、待提交任务取消、只观察保护、断线图像清理及关闭等待落地；共享控制接口 11 项回归测试通过。实际 X11 桌面通过 IBus libpinyin 和原生键盘事件输入 `nihao `，成功得到“你好”，未发送控制指令。验证记录为 `logs/setup/desktop-qt-ime.json`，窗口预览为 `logs/setup/desktop-qt-preview.png`；字体为 Noto Sans CJK SC，设备缩放为 2.0。此验证使用模拟数据，不属于飞行验收。

## 桌面启动前 PX4 崩溃处理（2026-10-22）

19:41 和 19:45 的失败发生在 EV 初始化期间，控制台尚未启动。`sitl.log` 记录 PX4 进程因 `malloc(): unaligned tcache chunk detected` 以 -6 退出，管理器随后清理 Gazebo。因此这两次退出并非 Qt 控制台加载失败。

排查发现 `vio/ekf_monitor.py` 同时运行 4 个 `px4-listener TOPIC -i 0 -n 1`，每次仅等待 0.7 秒。当前 PX4 版本指定 `-i` 后会等待新消息，还会读取守护进程的标准输入；调试堆栈实际观察到这些线程在 stdin read 中被取消，原始采样记录存在大量 TimeoutExpired。已将日常监测改为逐条执行不指定 `-i` 的即时快照，并按输出的 Instance 区块严格选取实例 0，防止多实例数据混合。源时间戳及原有数据有效性检查保留。该调整消除了已观察到的阻塞/取消触发条件；未捕获到最初破坏堆的写入位置，故不将其表述为已证明的 PX4 底层内存缺陷修复。

两次不带调试器的完整启动验证均打开 Gazebo 和 Qt 控制台：`logs/system/20261022-195203-NDTdaN` 与 `logs/system/20261022-195507-GqGIj3`。第一次采样约 88 秒，1,416 次读取无超时，并通过正常停止入口确认落地上锁后清理；第二次通过桌面快捷方式中的同一 GNOME Terminal 命令启动，保留运行供使用。汇总记录在 `logs/setup/desktop-startup-validation.json`。本次未执行飞行任务或调用模型。回归检查：采样及实例解析 29 项、EKF 降落观测 5 项、桌面 11 项通过；启动门禁 13 项中 12 项通过、1 项按环境跳过。

## 关闭后残留进程与重启修复（2026-10-22）

实际发现 `20261022-195507-GqGIj3` 的管理器和控制台已退出，但 Gazebo、PX4、执行器、VIO 及显示辅助仍存活。外层 ROS 启动日志确认：20:02:04 发送 SIGINT 后，管理器以 -13（SIGPIPE）退出，且没有执行完清理。关闭启动终端会切断 stdout/tee 管道，原清理函数向已断开的管道打印信息，导致管理器在收尾前被杀死。

修复：管理器捕获 SIGPIPE 进入正常退出；清理期间忽略重复停止及管道信号，并先将 stdout/stderr 转到本轮 `cleanup.log`，再请求降落、恢复参数和关闭所属进程。桌面启动增加 `wait_system_shutdown.py`，对仍在退出的上一轮核对管理器 PID、启动时间和命令，等待该管理器及外层 roslaunch 完成退出后再启动新 ROS master。正常运行中的系统仍保留原有冲突保护。

已对上述残留核对 `processes.tsv`、PID/启动时间及父子关系；确认实际 FCU 最新状态为落地上锁，按保存的原参数读回核对后逐项清理，记录保存在原运行目录的 `orphan-cleanup.json`。

实际验证两轮完整仿真：第一轮关闭日志读取端并发送终端挂断信号，第二轮重新启动后发送真实窗口关闭事件，并立即调用重启等待入口。每轮追踪 15 个所属进程，退出后剩余均为 0，8765/11311 端口均释放；第二轮正常关闭退出码为 0。记录为 `logs/setup/exit-restart-validation.json`，退出详情分别在 `logs/system/20261022-201046-kcs3TE/cleanup.log` 与 `logs/system/20261022-201135-Yt5pEe/cleanup.log`。5 项新增退出/重启测试通过，启动门禁 12 项通过、1 项按环境跳过。本次未执行飞行任务；验证后系统保持关闭，可直接双击启动。

## 视觉接口再次断连排查（2026-10-22 晚间）

本次实际配置为 `single_vlm` / `qwen3-vl-plus`，沿用 DashScope 兼容接口和 `UAV_MODEL_PROXY=http://127.0.0.1:7897`。密钥存在且经模型列表请求证实有效。故障请求在 TLS 握手阶段出现 `SSL UNEXPECTED_EOF_WHILE_READING`，没有收到鉴权、额度或模型错误 HTTP 响应。直连和 iKuuu 的代理路径均失败；另一条已有代理路径能访问同一接口，支持故障来自本机网络路径而非密钥或模型名。

iKuuu 中 PX4 DashScope 分流组仍存在，原为 Bypass node。将仅该组目标改为香港 S02（全局日本 S02、Rule 模式保持），初次复测仍失败；通过应用重新连接后，直连及控制台使用的 7897 路径均恢复 HTTP 200。具体底层断连原因尚未证明，不将“节点切换”单独认定为充分修复；此次恢复发生在 iKuuu 重连之后。项目配置和运行中的 ROS/PX4/控制台未重启。

恢复后 3 次固定短文本请求通过；另以代码新建的 32×32 纯红色方块进行 2 次视觉请求，均成功返回“红色”。日志为 `logs/setup/model-recheck-20261022-text-recovered/diagnostics.json` 和 `logs/setup/model-synthetic-validation.json`。真实相机图片诊断被自动审批拒绝，因此此次未发送相机图像，也未执行任何飞行工具。若后续出现同一 TLS 错误，可先在 iKuuu 重新连接后重试，避免反复重启整个仿真。

## VIO 主定位与 1～10 米变高任务（2026-10-22）

用户明确选择 VIO 为主要位置/速度来源、任务中可变高度且本地 ENU 上限 10 米，并选择使用已知静态三维地图规划。桌面 `launch_desktop.sh` 现默认 `PX4_DESKTOP_EV_PROFILE=no_gps`、`UAV_NAVIGATION_MODE=static_3d`。原通用 ROS launch 的 shadow/二维默认值保留。无 GPS profile 在 EKF 启动前设置 GPS_CTRL=0、EV_CTRL=7、HGT_REF=3；保留气压计和磁罗盘。启动必须观察到实际 EV 位置、速度及高度融合，并确认 GPS 位置/高度融合关闭，不能仅凭参数或适配器 forwarding 声称完成。

本机 PX4 1.15 的 aid-source 消息移除了 `fusion_enabled`，增加 `estimator_instance`；原 1.14 监测器会把实际融合误判为未融合。`aid_fusion_accepted` 现兼容两种格式，仍要求真实控制标志、fused=true、innovation_rejected=false、有效递增的 time_last_fuse、源时间年龄和正确实例。缺少旧 enabled 字段时，必须有整数类型的 selected estimator_instance，不能以缺失字段默认为已融合。监测现也保存高度 aid-source 证据。

`static_navigation.py` 独立读取经过哈希核对的内置场景 SDF 碰撞体，以保守 AABB 和 1 米间隙进行三维连续线段检查、A* 与路径简化。地图几何不交给 Agent；VIO/PX4 提供飞行位置，Gazebo 机体姿态仅用于本架次地图坐标注册及原有场景许可监督，不替代 EKF 位置控制。未知碰撞形状、动态模型、未校验场景及越界端点拒绝执行。起飞和计划降落的垂直通道也做碰撞检查。固定二维雷达门限仅在旧 lidar_grid 模式保留，新模式明确标注雷达只作遥测，不声称实时三维建图或动态障碍保护。区域搜索使用注册后的本地许可范围及当前 1～10 米观察高度，保留 VIO epoch/坐标重置证据约束。

实际仿真任务 `d0d628611786`（运行 `20261022-210300-8dbkG9`）完成：5 米起飞 → 原地 10 米 → 东侧 1 米/8 米高度 → 返航 → 降落。5 步均成功，确认 landed_state=1、armed=false。完整记录 `logs/setup/vio-variable-height-flight.json`，汇总 `logs/setup/vio-variable-height-summary.json`。本地高度峰值 9.99198 米，世界坐标机体 Z 峰值 10.32468 米；因此“10米”指任务本地 ENU 高度，不是严格的离地高度上界，也不证明定位误差为零。

178 个解锁状态采样中，EV 位置及高度接受融合均为 178，速度为 174，GPS 位置/高度接受融合均为 0。4 个速度未判为有效的采样均在降落阶段（landed_state=4），原记录保留；此次有限采样不能声称连续每一时刻均融合。任务结束后实际检查 VIO 三项融合与健康均恢复，坐标重置保护无锁存错误。未调用模型或发送相机图像。本轮只验证这条有限仿真航线，不替代历史原生 VIO 精度/故障全套验收。

新增/相关检查：三维规划 11 项、三维执行器 4 项、静态地图搜索许可 3 项、1.14/1.15 aid schema 3 项、桌面 13 项、原二维导航 37 项、场景许可 25 项、单 VLM 38 项等均通过。原启动契约 12 项通过、1 项按环境跳过，退出/重启 5 项通过。回退旧桌面模式可用 `UAV_NAVIGATION_MODE=lidar_grid PX4_DESKTOP_EV_PROFILE=shadow bash scripts/launch_desktop.sh`。退出依然恢复原参数，下一次桌面启动重新应用所选 profile。
