#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
source /opt/ros/noetic/setup.bash
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
    "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PROJECT_DIR/logs/ros}"
mkdir -p "$ROS_LOG_DIR"
sitl_launch=(px4 mavros_posix_sitl.launch)
if [[ -n "${SITL_LAUNCH_FILE:-}" ]]; then
    [[ -r "$SITL_LAUNCH_FILE" ]] || { echo "Cannot read SITL launch: $SITL_LAUNCH_FILE" >&2; exit 1; }
    sitl_launch=("$SITL_LAUNCH_FILE")
fi
exec roslaunch "${sitl_launch[@]}" gui:="${GUI:-false}" interactive:=false "$@"
