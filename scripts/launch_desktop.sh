#!/usr/bin/env bash
# Start the complete simulation and native control window.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
/usr/bin/python3 "$PROJECT_DIR/scripts/wait_system_shutdown.py"
export UAV_NAVIGATION_MODE="${UAV_NAVIGATION_MODE:-static_3d}"
export UAV_SEARCH_MODE="${UAV_SEARCH_MODE:-nbv_geometry}"
export UAV_NBV_SELECTOR="${UAV_NBV_SELECTOR:-deterministic}"
if [[ "$UAV_SEARCH_MODE" == fixed ]]; then
    export UAV_NBV_AUTO_PREPARE="${UAV_NBV_AUTO_PREPARE:-0}"
    export UAV_TARGET_CONFIRMATION="${UAV_TARGET_CONFIRMATION:-single_frame}"
else
    export UAV_NBV_AUTO_PREPARE="${UAV_NBV_AUTO_PREPARE:-1}"
    export UAV_TARGET_CONFIRMATION="${UAV_TARGET_CONFIRMATION:-multiview}"
fi
exec bash "$PROJECT_DIR/scripts/launch_system.sh" control_ui:=desktop \
    ev_profile:="${PX4_DESKTOP_EV_PROFILE:-no_gps}" env_file:="$PROJECT_DIR/.env" "$@"
