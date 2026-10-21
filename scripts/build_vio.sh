#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
/usr/bin/python3 "$PROJECT_DIR/vio/download_upstream.py"
/usr/bin/python3 "$PROJECT_DIR/vio/patch_upstream.py"
exec catkin_make -C "$PROJECT_DIR/vio_ws" -j2 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release --make-args vins_node uav_body_imu_plugin
