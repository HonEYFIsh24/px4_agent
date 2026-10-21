#!/usr/bin/env bash
# Own only the processes started by this invocation; existing services are never reused.
set -Eeuo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PX4_DIR="${PX4_DIR:-/home/dut/PX4-Autopilot}"
WEB_PORT=8080
READY_TIMEOUT=150
LAND_TIMEOUT=90
ENV_FILE=''
WORLD_FILE="${PX4_AGENT_WORLD:-$PROJECT_DIR/scene/worlds/obstacle_yard.world}"
USE_EXISTING_MASTER=0
EV_PROFILE="${PX4_AGENT_EV_PROFILE:-shadow}"
EV_TEST_FAULTS=0
RECOVER_PROFILE_FROM=''

usage() {
    cat <<'EOF'
用法：bash scripts/start_system.sh [选项]

启动本机 PX4 + Gazebo + MAVROS、ROS 执行器、独立 VINS-Fusion 和中文页面。
  --env-file FILE       将模型配置文件交给页面；默认复用项目/已有 ROS Agent 配置
  --world-file FILE     Gazebo 世界；默认使用包含障碍墙的复杂物流园区
  --port PORT           页面端口（默认 8080）；执行器固定为 127.0.0.1:8765
  --ready-timeout SEC   等待飞控就绪的总时间（默认 150 秒）
  --land-timeout SEC    退出前等待落地上锁的时间（默认 90 秒）
  --ev-profile NAME     shadow（默认）、gps_fused 或 no_gps
  --ev-test-faults      允许本次已验证 SITL 的短时 EV 故障注入（默认关闭）
  --recover-profile-from FILE  地面恢复已停止运行的 ev-profile/original.json，再应用本次 profile
  --from-roslaunch      允许外层 roslaunch 创建的本机空 ROS master
  -h, --help           显示本说明

GUI=true 可打开 Gazebo 窗口。Ctrl+C / SIGTERM 会先请求降落，再关闭本次子进程。
后台运行：setsid nohup bash scripts/start_system.sh > logs/system-launch.log 2>&1 < /dev/null &
停止本次后台运行：bash scripts/stop_system.sh [运行日志目录]
EOF
}
fail() { echo "启动失败：$*" >&2; exit 1; }
while (($#)); do
    case "$1" in
        --env-file|--world-file|--port|--ready-timeout|--land-timeout|--ev-profile|--recover-profile-from)
            (($# >= 2)) || fail "$1 缺少参数"
            case "$1" in
                --env-file) ENV_FILE="$2" ;;
                --world-file) WORLD_FILE="$2" ;;
                --port) WEB_PORT="$2" ;;
                --ready-timeout) READY_TIMEOUT="$2" ;;
                --land-timeout) LAND_TIMEOUT="$2" ;;
                --ev-profile) EV_PROFILE="$2" ;;
                --recover-profile-from) RECOVER_PROFILE_FROM="$2" ;;
            esac
            shift 2 ;;
        -h|--help) usage; exit 0 ;;
        --from-roslaunch) USE_EXISTING_MASTER=1; shift ;;
        --ev-test-faults) EV_TEST_FAULTS=1; shift ;;
        *) fail "未知参数：$1；使用 --help 查看用法" ;;
    esac
done
case "$EV_PROFILE" in shadow|gps_fused|no_gps) ;; *) fail 'ev-profile 必须为 shadow、gps_fused 或 no_gps' ;; esac
[[ -z "$RECOVER_PROFILE_FROM" || "$EV_PROFILE" != no_gps ]] \
    || fail '参数恢复不能与 no_gps 冷启动同用；先以 shadow/gps_fused 地面恢复并退出，再独立 no_gps 冷启动'
export PX4_AGENT_EV_PROFILE="$EV_PROFILE"
[[ "$WEB_PORT" =~ ^[0-9]+$ && ${#WEB_PORT} -le 5 ]] || fail '页面端口必须为 1–65535'
WEB_PORT=$((10#$WEB_PORT))
((WEB_PORT >= 1 && WEB_PORT <= 65535 && WEB_PORT != 8765 && WEB_PORT != 11311)) || fail '页面端口无效或与 ROS/执行器冲突'
for timeout in "$READY_TIMEOUT" "$LAND_TIMEOUT"; do
    [[ "$timeout" =~ ^[1-9][0-9]{0,3}$ ]] || fail '超时必须为 1–9999 秒的整数'
done
[[ -z "$ENV_FILE" || -r "$ENV_FILE" ]] || fail "无法读取模型配置：$ENV_FILE"
[[ -n "$WORLD_FILE" && -r "$WORLD_FILE" ]] || fail "无法读取 Gazebo 世界：$WORLD_FILE"
WORLD_FILE="$(readlink -f -- "$WORLD_FILE")"
export PX4_AGENT_WORLD="$WORLD_FILE"
for command in setsid curl timeout /usr/bin/python3; do
    command -v "$command" >/dev/null || fail "缺少命令：$command"
done
for script in configure_ev_profile.py recover_ev_profile.py prepare_ev_boot.py start_ev.sh start_ekf_observer.sh; do
    [[ -r "$PROJECT_DIR/scripts/$script" ]] || fail "缺少 EV 启动组件：$script"
done
[[ -f /opt/ros/noetic/setup.bash ]] || fail '未找到 ROS Noetic'
[[ -f "$PROJECT_DIR/vio_ws/devel/lib/libuav_body_imu_plugin.so" ]] || fail '缺少机载 IMU 插件；先运行 bash scripts/build_vio.sh'
[[ -x "${PX4_DIR:-/home/dut/PX4-Autopilot}/build/px4_sitl_default/bin/px4" ]] || fail '未找到 PX4 SITL 可执行文件'
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
case "$ROS_MASTER_URI" in
    http://127.0.0.1:11311|http://127.0.0.1:11311/|http://localhost:11311|http://localhost:11311/) ;;
    *) fail '整套启动入口要求本机 ROS_MASTER_URI=http://127.0.0.1:11311' ;;
esac
if [[ -n "$RECOVER_PROFILE_FROM" ]]; then
    [[ -f "$RECOVER_PROFILE_FROM" && -r "$RECOVER_PROFILE_FROM" ]] || fail '无法读取旧参数原档'
    # Resolve before this invocation changes logs/system/latest. Never accept
    # a path whose meaning silently changes to the new manager's archive.
    RECOVER_PROFILE_FROM="$(readlink -f -- "$RECOVER_PROFILE_FROM")"
    /usr/bin/python3 "$PROJECT_DIR/scripts/recover_ev_profile.py" \
        --source "$RECOVER_PROFILE_FROM" --validate-source --px4-directory "$PX4_DIR" >/dev/null \
        || fail '旧参数记录或已停止 owner 身份验证失败；本次未启动系统'
fi

# Inspect without stopping or replacing anything. Binding also catches wildcard listeners.
/usr/bin/python3 - "$WEB_PORT" "$USE_EXISTING_MASTER" <<'PY' || exit 1
import os, pathlib, socket, sys, xmlrpc.client
conflicts = []
for path in pathlib.Path('/proc').iterdir():
    if not path.name.isdigit():
        continue
    try:
        name = (path / 'comm').read_text().strip()
        if name in ('px4', 'gzserver', 'vins_node'):
            conflicts.append('{} (PID {})'.format(name, path.name))
    except OSError:
        pass
ports = {8765, int(sys.argv[1])}
if sys.argv[2] == '1':
    # An outer roslaunch owns this master. Never attach to an existing flight
    # graph or stop its master; all simulator children remain owned below.
    socket.setdefaulttimeout(2)
    try:
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        code, _, graph = master.getSystemState('/uav_system_preflight')
        if code != 1:
            raise ValueError('无法读取 ROS 图')
        nodes = {node for entries in graph for _, registered in entries for node in registered}
        occupied = sorted(nodes - {'/rosout'})
        if occupied:
            conflicts.append('ROS master 不是空环境：' + ', '.join(occupied))
    except (OSError, ValueError, xmlrpc.client.Error) as exc:
        conflicts.append('无法验证外层 ROS master ({})'.format(exc))
else:
    ports.add(11311)
for port in sorted(ports):
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            listener.bind(('127.0.0.1', port))
        except OSError as exc:
            conflicts.append('端口 {} 已占用 ({})'.format(port, exc.strerror))
if conflicts:
    print('启动失败：检测到已有服务，本次没有启动或关闭任何服务。', file=sys.stderr)
    print('\n'.join('  ' + item for item in conflicts), file=sys.stderr)
    print('请先停止已有整套服务，或分别使用现有 start_* 脚本。', file=sys.stderr)
    sys.exit(1)
PY

umask 077
mkdir -p "$PROJECT_DIR/logs/system"
RUN_DIR="$(mktemp -d "$PROJECT_DIR/logs/system/$(date +%Y%m%d-%H%M%S)-XXXXXX")"
ln -sfn -- "$RUN_DIR" "$PROJECT_DIR/logs/system/latest"
export ROS_LOG_DIR="$RUN_DIR/ros"
export UAV_CAPTURE_EVIDENCE_DIR="$RUN_DIR/captures"
export UAV_VIO_LOG_DIR="$RUN_DIR/vio_output"
mkdir -p "$ROS_LOG_DIR" "$UAV_CAPTURE_EVIDENCE_DIR"
printf '%s\n' "$WORLD_FILE" > "$RUN_DIR/world-file.txt"
declare -A PIDS STARTS
PIDS=(); STARTS=()
CLEANING=0
VIO_WARNED=0
PROFILE_RESTORE_FILE=''
EV_MODE=shadow
[[ "$EV_PROFILE" == shadow ]] || EV_MODE=fuse
BRIDGE_URL=http://127.0.0.1:8765
WEB_URL="http://127.0.0.1:$WEB_PORT"

start_time() {
    /usr/bin/python3 - "$1" <<'PY'
import pathlib, sys
try:
    print(pathlib.Path('/proc/' + sys.argv[1] + '/stat').read_text().rsplit(')', 1)[1].split()[19])
except OSError:
    sys.exit(1)
PY
}
alive() { [[ -n "${PIDS[$1]:-}" ]] && kill -0 "${PIDS[$1]}" 2>/dev/null; }
launch() {
    local name="$1"; shift
    setsid bash "$PROJECT_DIR/scripts/$1" "${@:2}" > "$RUN_DIR/$name.log" 2>&1 &
    PIDS[$name]=$!
    # A missing optional VINS binary can make its wrapper exit before /proc is read.
    STARTS[$name]="$(start_time "${PIDS[$name]}" 2>/dev/null || true)"
    printf '%s\t%s\t%s\n' "$name" "${PIDS[$name]}" "${STARTS[$name]}" >> "$RUN_DIR/processes.tsv"
    echo "$name 日志：$RUN_DIR/$name.log"
}
get_state() {
    curl -fsS --connect-timeout 1 --max-time 2 "$BRIDGE_URL/state" > "$RUN_DIR/state.json" 2>/dev/null
}
state_is() {
    /usr/bin/python3 - "$RUN_DIR/state.json" "$1" "$EV_PROFILE" "$PROJECT_DIR" <<'PY'
import json, sys
try:
    state = json.load(open(sys.argv[1]))
    landed = state.get('armed') is False and state.get('landed_state') == 1
    ground = landed and state.get('connected') is True and state.get('gazebo', {}).get('available') is True
    if sys.argv[2] == 'landed':
        valid = landed
    elif sys.argv[2] in ('ground_connected', 'recovery_ground_connected'):
        valid = ground and (state.get('mission') or {}).get('status') != 'running'
        if valid and sys.argv[2] == 'recovery_ground_connected':
            sys.path.insert(0, sys.argv[4])
            from scripts.recover_ev_profile import fresh_ground_guard
            fresh_ground_guard(state)
    else:
        external = state.get('external_vision') or {}
        adapter = external.get('adapter') or {}
        ekf = external.get('ekf') or {}
        valid = ground and state.get('ready') is True and adapter.get('ready') is True
        if sys.argv[3] == 'shadow':
            valid = valid and adapter.get('mode') == 'shadow' and adapter.get('forwarding') is False
        else:
            valid = (valid and adapter.get('mode') == 'fuse' and external.get('healthy') is True
                     and external.get('used_for_flight_control') is True and ekf.get('fusion_active') is True)
            if sys.argv[3] == 'no_gps':
                valid = valid and ekf.get('gps_fusion') is False and ekf.get('gps_height_fusion') is False
            else:
                valid = valid and ekf.get('gps_fusion') is True
    sys.exit(0 if valid else 1)
except (OSError, ValueError, TypeError, RuntimeError):
    sys.exit(1)
PY
}

# Record descendant identities before signalling: roslaunch creates separate sessions.
# Identity checks prevent signalling a PID that has since been reused.
shutdown_component() {
    local name="$1" signal="$2"
    [[ -n "${PIDS[$name]:-}" ]] || return 0
    /usr/bin/python3 - "${PIDS[$name]}" "${STARTS[$name]}" "$signal" <<'PY'
import os, pathlib, signal, sys, time
root, started = int(sys.argv[1]), sys.argv[2]
def record(pid):
    try:
        fields = pathlib.Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')', 1)[1].split()
        return int(fields[1]), fields[19], fields[0]
    except (OSError, ValueError):
        return None
item = record(root)
if item is None or item[1] != started:
    sys.exit(0)
records = {}
for path in pathlib.Path('/proc').iterdir():
    if path.name.isdigit():
        entry = record(int(path.name))
        if entry:
            records[int(path.name)] = entry
owned = {root: started}
changed = True
while changed:
    changed = False
    for pid, entry in records.items():
        if pid not in owned and entry[0] in owned:
            owned[pid] = entry[1]
            changed = True
def remaining():
    live = []
    for pid, stamp in owned.items():
        entry = record(pid)
        if entry and entry[1] == stamp and entry[2] != 'Z':
            live.append(pid)
    return live
def send(pid, sig):
    entry = record(pid)
    if entry and entry[1] == owned[pid] and entry[2] != 'Z':
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
send(root, getattr(signal, 'SIG' + sys.argv[3]))
for sig, seconds in ((None, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 1)):
    if sig:
        for pid in remaining():
            send(pid, sig)
    deadline = time.monotonic() + seconds
    while remaining() and time.monotonic() < deadline:
        time.sleep(0.1)
PY
    wait "${PIDS[$name]}" 2>/dev/null || true
}
cleanup() {
    local result=$? deadline
    ((CLEANING == 0)) || return 0
    CLEANING=1
    trap '' INT TERM HUP
    set +e
    echo '准备退出：先请求降落并等待落地上锁。'
    if alive bridge; then
        # The page also interrupts any in-flight model call so it cannot resume later.
        if ! alive web || ! curl -fsS --connect-timeout 1 --max-time 6 \
            -H 'Content-Type: application/json' -d '{}' "$WEB_URL/api/land" > "$RUN_DIR/shutdown-land.json" 2>/dev/null; then
            curl -fsS --connect-timeout 1 --max-time 6 -H 'Content-Type: application/json' \
                -d '{}' "$BRIDGE_URL/land" > "$RUN_DIR/shutdown-land.json" 2>/dev/null
        fi
        deadline=$((SECONDS + LAND_TIMEOUT))
        while ((SECONDS < deadline)) && alive bridge && alive sitl; do
            if get_state && state_is landed; then
                echo '已落地并上锁。'
                break
            fi
            sleep 1
        done
    fi
    # A recorded bridge PID means it was launched, even if it has exited.
    # Keep the landing guard after that point; no endpoint exists before it.
    if [[ -n "${PIDS[bridge]:-}" ]] && alive sitl && (! get_state || ! state_is landed); then
        echo "未在 $LAND_TIMEOUT 秒内确认落地上锁；保留 EV、VIO 和仿真，继续等待。见 $RUN_DIR。" >&2
        printf '%s\n' 'waiting_for_confirmed_landed_disarmed' > "$RUN_DIR/cleanup-blocked"
        while alive sitl && (! get_state || ! state_is landed); do
            # Keep the estimator stream and the manager identity alive. Never
            # remove flight inputs merely because the landing timeout elapsed.
            sleep 2
        done
        rm -f -- "$RUN_DIR/cleanup-blocked"
    fi
    shutdown_component web TERM
    shutdown_component ev TERM
    shutdown_component vio TERM
    if [[ -z "$PROFILE_RESTORE_FILE" && -s "$RUN_DIR/startup-original.txt" ]]; then
        /usr/bin/python3 "$PROJECT_DIR/scripts/prepare_ev_boot.py" --run-directory "$RUN_DIR" \
            --capture-original-only > "$RUN_DIR/startup-restore-capture.log" 2>&1
        [[ ! -s "$RUN_DIR/startup-original.json" ]] || PROFILE_RESTORE_FILE="$RUN_DIR/startup-original.json"
    fi
    if [[ -z "$PROFILE_RESTORE_FILE" && "$EV_PROFILE" != no_gps && -s "$RUN_DIR/ev-profile/original.json" ]]; then
        PROFILE_RESTORE_FILE="$RUN_DIR/ev-profile/original.json"
    fi
    if [[ -n "$PROFILE_RESTORE_FILE" && -s "$PROFILE_RESTORE_FILE" ]] && alive sitl; then
        local restore_attempt=1
        while ! /usr/bin/python3 "$PROJECT_DIR/scripts/configure_ev_profile.py" \
            --restore "$PROFILE_RESTORE_FILE" --apply --run-directory "$RUN_DIR" \
            --output "$RUN_DIR/ev-profile-restore-$restore_attempt" \
            > "$RUN_DIR/ev-profile-restore-$restore_attempt.log" 2>&1; do
            echo "尚未恢复本次 PX4 原参数；保留落地的仿真与执行器。见 $RUN_DIR/ev-profile-restore-$restore_attempt.log" >&2
            printf '%s\n' 'waiting_for_original_px4_parameter_restore' > "$RUN_DIR/cleanup-blocked"
            alive sitl || break
            restore_attempt=$((restore_attempt + 1))
            sleep 3
        done
        rm -f -- "$RUN_DIR/cleanup-blocked"
    fi
    shutdown_component ekf_observer TERM
    shutdown_component bridge TERM
    shutdown_component sitl INT
    # Keep the render display through landing, restoration and SITL shutdown.
    shutdown_component gui_display TERM
    echo "本次启动的服务已关闭。日志保留在 $RUN_DIR"
    printf '%s\n' "$result" > "$RUN_DIR/exit-code"
    return "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
/usr/bin/python3 - "$$" "$PROJECT_DIR/scripts/start_system.sh" "$RUN_DIR" <<'PY'
import json, pathlib, sys
pid = int(sys.argv[1])
started = pathlib.Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')', 1)[1].split()[19]
pathlib.Path(sys.argv[3], 'controller.json').write_text(json.dumps({
    'pid': pid, 'start_time': started, 'script': sys.argv[2]}, ensure_ascii=False) + '\n')
PY
echo "本次运行日志：$RUN_DIR"
echo "本次 Gazebo 世界：$WORLD_FILE"
echo "本次相机取证目录：$UAV_CAPTURE_EVIDENCE_DIR"
printf '%s\n' "$EV_PROFILE" > "$RUN_DIR/ev-profile.txt"
# Both camera rendering and gzclient need a usable display. The helper owns
# only this run's optional nested display and publishes validated readiness.
launch gui_display start_gui_display.sh "$RUN_DIR" "${GUI:-false}"
gui_deadline=$((SECONDS + 60))
while ! /usr/bin/python3 "$PROJECT_DIR/scripts/gui_display.py" \
    --check-ready --run-directory "$RUN_DIR" --helper-pid "${PIDS[gui_display]}" \
    --helper-start "${STARTS[gui_display]}" >/dev/null 2>&1; do
    alive gui_display || fail "显示准备已退出；查看 $RUN_DIR/gui_display.log"
    ((SECONDS < gui_deadline)) || fail "显示准备超时；查看 $RUN_DIR/gui_display.log"
    sleep 0.2
done
# NUL-separated values avoid evaluating shell text or exporting other keys.
mapfile -d '' -t GUI_DISPLAY_ENV < <(/usr/bin/python3 "$PROJECT_DIR/scripts/gui_display.py" \
    --export-environment --run-directory "$RUN_DIR" --helper-pid "${PIDS[gui_display]}" \
    --helper-start "${STARTS[gui_display]}")
[[ ${#GUI_DISPLAY_ENV[@]} -eq 4 && "${GUI_DISPLAY_ENV[0]}" == DISPLAY \
    && "${GUI_DISPLAY_ENV[2]}" == XAUTHORITY && -n "${GUI_DISPLAY_ENV[1]}" ]] \
    || fail '显示准备返回了无效的环境数据'
export DISPLAY="${GUI_DISPLAY_ENV[1]}"
if [[ -n "${GUI_DISPLAY_ENV[3]}" ]]; then
    export XAUTHORITY="${GUI_DISPLAY_ENV[3]}"
else
    unset XAUTHORITY
fi
alive gui_display || fail "显示准备组件已退出；查看 $RUN_DIR/gui_display.log"
export PX4_AGENT_EV_BOOT_RUN_DIR="$RUN_DIR"
if [[ "$EV_PROFILE" == no_gps ]]; then
    /usr/bin/python3 "$PROJECT_DIR/scripts/prepare_ev_boot.py" --run-directory "$RUN_DIR" \
        --px4-directory "$PX4_DIR" > "$RUN_DIR/ev-boot-prepare.log" 2>&1 \
        || fail "无法准备本次 pre-EKF 无 GPS hook；查看 $RUN_DIR/ev-boot-prepare.log"
    export PATH="$RUN_DIR/runtime-bin:$PATH"
fi
launch sitl start_sitl_vio.sh world:="$WORLD_FILE"
deadline=$((SECONDS + READY_TIMEOUT))
echo '等待本机 ROS 仿真服务，然后启动执行器……'
while :; do
    alive sitl || fail "SITL 已退出；查看 $RUN_DIR/sitl.log"
    alive gui_display || fail "显示准备组件已退出；查看 $RUN_DIR/gui_display.log"
    if /usr/bin/python3 - <<'PY' >/dev/null 2>&1
import os, socket, sys, xmlrpc.client
socket.setdefaulttimeout(2)
master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
try:
    simulated = master.getParam('/uav_system_start', '/use_sim_time')
    endpoint = master.getParam('/uav_system_start', '/mavros/fcu_url')
    services = master.getSystemState('/uav_system_start')[2][2]
    okay = (simulated[0] == 1 and simulated[2] is True and endpoint[0] == 1
            and endpoint[2] == 'udp://:14540@localhost:14557'
            and any(name == '/gazebo/get_world_properties' for name, _ in services))
    sys.exit(0 if okay else 1)
except (OSError, xmlrpc.client.Error):
    sys.exit(1)
PY
    then break; fi
    ((SECONDS < deadline)) || fail "等待 ROS 仿真服务超时；查看 $RUN_DIR/sitl.log"
    sleep 1
done
launch bridge start_bridge.sh --world-file "$WORLD_FILE"
echo '等待 MAVROS 连接及 Gazebo 落地状态；此处不依赖 GPS 本地位置。'
while ! get_state || ! state_is ground_connected; do
    alive sitl || fail "SITL 已退出；查看 $RUN_DIR/sitl.log"
    alive gui_display || fail "显示准备组件已退出；查看 $RUN_DIR/gui_display.log"
    alive bridge || fail "执行器已退出；查看 $RUN_DIR/bridge.log"
    ((SECONDS < deadline)) || fail "等待连接和落地状态超时；查看 $RUN_DIR/state.json 和 bridge.log"
    sleep 1
done
echo '设置本次 PX4 SITL 的真实心跳与落地状态频率为 5 Hz，保持慢速仿真下的遥测及时性。'
# The original PX4 offboard link uses local UDP 14580 for instance 0.
# HEARTBEAT cannot be changed through MAV_CMD_SET_MESSAGE_INTERVAL in PX4 v1.14;
# its local module client adjusts the real stream without republishing old state.
timeout 8 "$PX4_DIR/build/px4_sitl_default/bin/px4-mavlink" stream \
    -u 14580 -s HEARTBEAT -r 5 > "$RUN_DIR/telemetry.log" 2>&1 \
    || fail "无法设置 SITL 心跳频率；查看 $RUN_DIR/telemetry.log"
# Fresh landing receipts must cover real FCU land-detector state even at 0.35x.
# Configure the actual source stream, never replay an old ExtendedState locally.
timeout 8 "$PX4_DIR/build/px4_sitl_default/bin/px4-mavlink" stream \
    -u 14580 -s EXTENDED_SYS_STATE -r 5 > "$RUN_DIR/extended-state-telemetry.log" 2>&1 \
    || fail "无法设置 SITL 落地状态频率；查看 $RUN_DIR/extended-state-telemetry.log"
if [[ -n "$RECOVER_PROFILE_FROM" ]]; then
    while ! get_state || ! state_is recovery_ground_connected; do
        alive sitl && alive bridge || fail '等待恢复门禁期间所属仿真或执行器退出'
        ((SECONDS < deadline)) || fail '等待参数恢复的真实新鲜 FCU 地面状态超时'
        sleep 1
    done
    echo '先恢复异常退出前保存的原参数；核对实际读回值后才应用当前 profile。'
    /usr/bin/python3 "$PROJECT_DIR/scripts/recover_ev_profile.py" \
        --source "$RECOVER_PROFILE_FROM" --apply --run-directory "$RUN_DIR" \
        --px4-directory "$PX4_DIR" --output "$RUN_DIR/profile-recovery" \
        > "$RUN_DIR/profile-recovery.log" 2>&1 \
        || fail "旧参数恢复失败；当前 profile 未应用。查看 $RUN_DIR/profile-recovery.log"
    # Until the new profile captures its originals, cleanup must retain the
    # recovered baseline, rather than the contaminated pre-recovery values.
    PROFILE_RESTORE_FILE="$RUN_DIR/profile-recovery/recovered-original.json"
fi
if [[ "$EV_PROFILE" == no_gps ]]; then
    /usr/bin/python3 "$PROJECT_DIR/scripts/prepare_ev_boot.py" --run-directory "$RUN_DIR" --capture \
        > "$RUN_DIR/ev-boot-capture.log" 2>&1 \
        || fail "pre-EKF 无 GPS hook 未通过实际执行取证；拒绝 GPS bootstrap 回退。查看 $RUN_DIR/ev-boot-capture.log"
    PROFILE_RESTORE_FILE="$RUN_DIR/startup-original.json"
fi
launch ekf_observer start_ekf_observer.sh --run-directory "$RUN_DIR" --evidence-dir "$RUN_DIR/ekf-observer"
echo "应用本次地面 EV profile：$EV_PROFILE；保留原参数用于落地退出时恢复。"
/usr/bin/python3 "$PROJECT_DIR/scripts/configure_ev_profile.py" --profile "$EV_PROFILE" --apply \
    --run-directory "$RUN_DIR" --output "$RUN_DIR/ev-profile" > "$RUN_DIR/ev-profile.log" 2>&1 \
    || fail "无法应用 EV profile；查看 $RUN_DIR/ev-profile.log"
[[ "$EV_PROFILE" == no_gps ]] || PROFILE_RESTORE_FILE="$RUN_DIR/ev-profile/original.json"
echo "启动 VIO 与 EV 适配器（$EV_MODE），然后验证实际 EKF 和本地定位。"
launch vio start_vio.sh
ev_args=(--mode "$EV_MODE" --evidence-dir "$RUN_DIR/ev-adapter")
((EV_TEST_FAULTS == 0)) || ev_args+=(--allow-test-faults)
launch ev start_ev.sh "${ev_args[@]}"
while ! get_state || ! state_is ready; do
    for component in sitl bridge vio ev ekf_observer; do
        alive "$component" || fail "$component 在 EV 初始化期间退出；查看 $RUN_DIR/$component.log"
    done
    ((SECONDS < deadline)) || fail "等待 EV profile 和实际定位就绪超时；查看 $RUN_DIR/state.json、ev.log 与 ekf_observer.log"
    sleep 1
done
echo "本次 EV profile 已就绪：$EV_PROFILE；启动控制页面。"
web_args=(--port "$WEB_PORT" --bridge-url "$BRIDGE_URL")
[[ -z "$ENV_FILE" ]] || web_args+=(--env-file "$ENV_FILE")
launch web start_web.sh "${web_args[@]}"
deadline=$((SECONDS + 30))
while ! curl -fsS --connect-timeout 1 --max-time 2 "$WEB_URL/api/state" >/dev/null 2>&1; do
    alive sitl && alive bridge || fail '仿真或执行器在页面启动期间退出'
    alive web || fail "页面已退出；查看 $RUN_DIR/web.log"
    ((SECONDS < deadline)) || fail "页面启动超时；查看 $RUN_DIR/web.log"
    sleep 1
done
echo "打开中文页面：$WEB_URL"
echo "停止：Ctrl+C，或 bash $PROJECT_DIR/scripts/stop_system.sh $RUN_DIR"
while :; do
    for service in sitl bridge web ev ekf_observer gui_display; do
        alive "$service" || fail "$service 已退出；查看 $RUN_DIR/$service.log"
    done
    if ! alive vio && ((VIO_WARNED == 0)); then
        if [[ "$EV_PROFILE" != shadow ]]; then
            fail "VIO 在融合 profile 中退出；将先降落再关闭。检查 $RUN_DIR/vio.log"
        fi
        echo "VIO 已退出；检查 $RUN_DIR/vio.log 和 $PROJECT_DIR/logs/vio/vins.log。" >&2
        VIO_WARNED=1
    fi
    sleep 2
done
