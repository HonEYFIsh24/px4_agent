#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
export ROS_LOG_DIR="$PROJECT_DIR/logs/ros"
exec /usr/bin/python3 "$PROJECT_DIR/vio/run_demo.py"
