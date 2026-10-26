#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
AGENT_PYTHON="${AGENT_PYTHON:-$PROJECT_DIR/.venv/bin/python}"
# Respect the selected input method; Ubuntu's default is IBus (Chinese Pinyin).
export QT_IM_MODULE="${QT_IM_MODULE:-ibus}"
export XMODIFIERS="${XMODIFIERS:-@im=ibus}"
# ROS/OpenCV can export Qt 5 plugin directories; this window uses its own Qt 6.
unset QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH
exec "$AGENT_PYTHON" "$PROJECT_DIR/desktop.py" "$@"
