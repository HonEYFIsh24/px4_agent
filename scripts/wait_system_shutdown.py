#!/usr/bin/env python3
"""Wait for this project's previous close to finish before starting a new ROS master."""
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def process(pid):
    try:
        path = Path('/proc')/str(pid)
        if path.stat().st_uid != os.getuid():
            return None
        fields = (path/'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return {'start_time': fields[19], 'ppid': int(fields[1]),
                'arguments': os.fsdecode((path/'cmdline').read_bytes()).split('\0')}
    except (OSError, ValueError):
        return None


def wait_for_shutdown(run, timeout=240, inspect=process, sleep=time.sleep, clock=time.monotonic):
    run = Path(run).resolve()
    try:
        owner = json.loads((run/'controller.json').read_text())
        if owner['script'] != str(ROOT/'scripts/start_system.sh'):
            return False
        current = inspect(owner['pid'])
        if (not current or current['start_time'] != owner['start_time']
                or owner['script'] not in current['arguments']):
            return False
    except (OSError, ValueError, KeyError, TypeError):
        return False
    if not ((run/'desktop.closed').exists() or (run/'cleanup.log').exists()):
        return False
    parent_pid = current['ppid']
    parent = inspect(parent_pid)
    # Also let the old outer launch release its ROS master before the new one starts.
    if not parent or 'uav_system.launch' not in [Path(a).name for a in parent['arguments'] if a]:
        parent = None
    print('上次服务正在退出，等待落地、清理进程并释放端口…', flush=True)
    deadline = clock()+timeout
    while True:
        current = inspect(owner['pid'])
        manager_alive = current and current['start_time'] == owner['start_time']
        current_parent = inspect(parent_pid) if parent else None
        parent_alive = current_parent and current_parent['start_time'] == parent['start_time']
        if not manager_alive and not parent_alive:
            return True
        if clock() >= deadline:
            raise RuntimeError('上次服务仍在退出，请查看 ' + str(run/'cleanup.log'))
        sleep(.25)


if __name__ == '__main__':
    try:
        wait_for_shutdown(ROOT/'logs/system/latest')
    except RuntimeError as exc:
        sys.exit(str(exc))
