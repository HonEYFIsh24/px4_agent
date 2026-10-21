#!/usr/bin/env bash
# Adapt ROS launch remapping arguments to the existing lifecycle manager.
set -Eeuo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
for argument in "$@"; do
    case "$argument" in
        __name:=*|__log:=*|__ns:=*) ;;
        *) echo "不支持的 ROS 启动参数：$argument" >&2; exit 1 ;;
    esac
done
export GUI="${GUI:-true}"
case "$GUI" in
    true|True|1) export GUI=true ;;
    false|False|0) export GUI=false ;;
    *) echo 'gui 必须为 true 或 false。' >&2; exit 1 ;;
esac
if [[ "$GUI" == true && -z "${DISPLAY:-}" ]]; then
    echo 'Gazebo GUI 需要图形桌面的 DISPLAY；请在当前桌面的终端启动。' >&2
    exit 1
fi
arguments=(--from-roslaunch --port "${PX4_AGENT_WEB_PORT:-8080}"
    --world-file "${PX4_AGENT_WORLD:-$PROJECT_DIR/scene/worlds/obstacle_yard.world}"
    --ready-timeout "${PX4_AGENT_READY_TIMEOUT:-150}"
    --land-timeout "${PX4_AGENT_LAND_TIMEOUT:-90}"
    --ev-profile "${PX4_AGENT_EV_PROFILE:-shadow}")
case "${PX4_AGENT_EV_TEST_FAULTS:-false}" in
    true|True|1) arguments+=(--ev-test-faults) ;;
    false|False|0) ;;
    *) echo 'ev_test_faults 必须为 true 或 false。' >&2; exit 1 ;;
esac
[[ -z "${PX4_AGENT_ENV_FILE:-}" ]] || arguments+=(--env-file "$PX4_AGENT_ENV_FILE")
exec bash "$PROJECT_DIR/scripts/start_system.sh" "${arguments[@]}"
