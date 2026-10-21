#!/usr/bin/env bash
# One command: prepare the source package, then use the standard ROS launch.
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_DIR/scripts/setup_ros.bash"
exec roslaunch --sigint-timeout 180 px4_agent uav_system.launch "$@"
