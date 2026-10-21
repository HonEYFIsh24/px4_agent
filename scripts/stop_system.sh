#!/usr/bin/env bash
# Signal the validated controller; it owns landing and ordered child shutdown.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-}" in
    -h|--help)
        echo '用法：bash scripts/stop_system.sh [运行日志目录]'
        echo '默认停止 logs/system/latest 对应的本次启动；先核对 PID、启动时间和命令，再请求降落及关闭。'
        exit 0 ;;
esac
(($# <= 1)) || { echo '只能指定一个运行日志目录。' >&2; exit 1; }
RUN_DIR="${1:-$PROJECT_DIR/logs/system/latest}"
exec /usr/bin/python3 - "$RUN_DIR" "$PROJECT_DIR/scripts/start_system.sh" <<'PY'
import json, os, pathlib, signal, sys, time
run_dir = pathlib.Path(sys.argv[1])
expected_script = pathlib.Path(sys.argv[2]).resolve()
try:
    controller = json.loads((run_dir / 'controller.json').read_text())
    pid = int(controller['pid'])
    expected_start = str(controller['start_time'])
    if pathlib.Path(controller['script']).resolve() != expected_script:
        raise ValueError('运行记录不属于此项目的启动脚本')
except (OSError, ValueError, KeyError, TypeError) as exc:
    sys.exit('停止失败：无法验证运行记录：{}'.format(exc))
def identity():
    try:
        fields = pathlib.Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')', 1)[1].split()
        return fields[19], fields[0]
    except OSError:
        return None
entry = identity()
if entry is None or entry[1] == 'Z':
    print('该次启动已结束；没有关闭其他进程。')
    sys.exit(0)
if entry[0] != expected_start:
    sys.exit('停止失败：PID 已被复用，没有发送信号。')
try:
    arguments = pathlib.Path('/proc/{}/cmdline'.format(pid)).read_bytes().split(b'\0')
    cwd = pathlib.Path('/proc/{}/cwd'.format(pid)).resolve()
    scripts = [pathlib.Path(os.fsdecode(arg)) for arg in arguments if arg]
    if not any((path if path.is_absolute() else cwd / path).resolve() == expected_script for path in scripts):
        raise ValueError('进程命令不是本项目的 start_system.sh')
except (OSError, ValueError) as exc:
    sys.exit('停止失败：{}；没有发送信号。'.format(exc))
if identity() != entry:
    sys.exit('停止失败：进程身份发生变化，没有发送信号。')
try:
    os.kill(pid, signal.SIGTERM)
except ProcessLookupError:
    print('该次启动已结束。')
    sys.exit(0)
print('已请求降落并按顺序关闭本次服务；日志：{}'.format(run_dir.resolve()), flush=True)
# Keep the stop request usable with custom landing timeouts without killing children.
deadline = time.monotonic() + 180
while time.monotonic() < deadline:
    current = identity()
    if current is None or current[0] != expected_start or current[1] == 'Z':
        print('本次服务已关闭。')
        sys.exit(0)
    time.sleep(0.5)
sys.exit('启动脚本仍在等待降落或关闭；请查看运行日志。不会强制关闭未经核对的进程。')
PY
