#!/usr/bin/env bash
# Keep startup errors visible when launched from the desktop application menu.
set -uo pipefail
umask 077
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
mkdir -p logs
LAUNCH_LOG="$(mktemp "$PROJECT_DIR/logs/desktop-launch-$(date +%Y%m%d-%H%M%S)-XXXXXX.log")"
echo "PX4 桌面控制台正在启动，完整日志：$LAUNCH_LOG"
bash "$PROJECT_DIR/scripts/launch_desktop.sh" "$@" 2>&1 | tee "$LAUNCH_LOG"
LAUNCH_RESULT=${PIPESTATUS[0]}
if ((LAUNCH_RESULT != 0)); then
    echo "启动失败（退出码 $LAUNCH_RESULT）。请查看上方错误或日志：$LAUNCH_LOG"
    if [[ -t 0 ]]; then
        read -r -p '按 Enter 关闭此窗口。' || true
    fi
fi
exit "$LAUNCH_RESULT"
