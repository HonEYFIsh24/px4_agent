#!/usr/bin/env bash
# Source this file to make the source-space px4_agent package discoverable.
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    echo '未找到 /opt/ros/noetic/setup.bash。' >&2
    return 1 2>/dev/null || exit 1
fi
source /opt/ros/noetic/setup.bash
PX4_AGENT_PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export ROS_PACKAGE_PATH="$PX4_AGENT_PROJECT_DIR:$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:${ROS_PACKAGE_PATH:-}"
