#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PROJECT_DIR/logs/ros}"
mkdir -p "$ROS_LOG_DIR"
exec /usr/bin/python3 "$PROJECT_DIR/bridge.py" "$@"

