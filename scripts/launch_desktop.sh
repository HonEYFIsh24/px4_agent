#!/usr/bin/env bash
# Start the complete simulation and native control window.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
/usr/bin/python3 "$PROJECT_DIR/scripts/wait_system_shutdown.py"
export UAV_NAVIGATION_MODE="${UAV_NAVIGATION_MODE:-static_3d}"
exec bash "$PROJECT_DIR/scripts/launch_system.sh" control_ui:=desktop \
    ev_profile:="${PX4_DESKTOP_EV_PROFILE:-no_gps}" env_file:="$PROJECT_DIR/.env" "$@"
