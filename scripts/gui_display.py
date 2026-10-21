#!/usr/bin/env python3
"""Keep a run-owned display alive until the manager has stopped its SITL."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import shlex
import signal
import socket
import stat
import struct
import subprocess
import sys
import time

SOURCE = Path(__file__).resolve().with_name('gui_display_probe.c')
LOCAL_DISPLAY = re.compile(r'^:[0-9]+(?:\.[0-9]+)?$')


def process(pid):
    try:
        p = Path('/proc') / str(int(pid))
        if p.stat().st_uid != os.getuid():
            return None
        fields = (p / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return {'pid': int(pid), 'start_time': fields[19], 'ppid': int(fields[1]),
                'executable': str((p / 'exe').resolve()),
                'arguments': (p / 'cmdline').read_bytes().split(b'\0')[:-1]}
    except (OSError, ValueError, IndexError):
        return None


def matches(owner, executable=None, parent=None, argument=None):
    current = process(owner['pid'])
    return bool(current and current['start_time'] == owner['start_time']
                and (executable is None or current['executable'] == str(Path(executable).resolve()))
                and (parent is None or current['ppid'] == parent)
                and (argument is None or os.fsencode(argument) in current['arguments']))


def identity(item):
    return {key: item[key] for key in ('pid', 'start_time')}


def manager_owner(run):
    private_file(run / 'controller.json')
    data = json.loads((run / 'controller.json').read_text())
    if not matches(data, argument=data['script']):
        raise RuntimeError('manager identity mismatch')
    return data


def usable_modes(data):
    """Recompute validity from actual sizes AND advertised rates, including Xephyr's 0Hz entry."""
    try:
        count, current = data['mode_count'], data['current_mode_index']
        modes = data['modes']
        if (not data['display_open'] or not data['randr_extension']
                or not data['screen_config_available'] or data['x_error']
                or not data['mode_pointer_nonnull'] or type(count) is not int
                or not 0 < count <= 4096 or len(modes) != count
                or type(current) is not int or not 0 <= current < count):
            return False
        pairs = 0
        for index, mode in enumerate(modes):
            rates = mode['rates']
            if (mode['index'] != index or type(mode['width']) is not int
                    or type(mode['height']) is not int or mode['width'] <= 0 or mode['height'] <= 0
                    or type(mode['rate_count']) is not int or not 0 <= mode['rate_count'] <= 4096
                    or len(rates) != mode['rate_count']
                    or (rates and not mode['rate_pointer_nonnull'])
                    or any(type(rate) is not int or rate < 0 for rate in rates)):
                return False
            pairs += len(rates)
        return (pairs > 0 and bool(modes[current]['rates'])
                and data['current_rate'] in modes[current]['rates']
                and data['size_rate_pair_count'] == pairs and data['usable'] is True)
    except (KeyError, TypeError, ValueError):
        return False


def environment_text(environment):
    if set(environment) not in ({'DISPLAY'}, {'DISPLAY', 'XAUTHORITY'}):
        raise RuntimeError('unexpected display environment key')
    if not LOCAL_DISPLAY.fullmatch(environment['DISPLAY']):
        raise RuntimeError('display must be a local X display')
    if any(not isinstance(value, str) or '\n' in value or '\r' in value or '\0' in value
           for value in environment.values()):
        raise RuntimeError('invalid environment value')
    return ''.join('export {}={}\n'.format(key, shlex.quote(environment[key]))
                   for key in sorted(environment))


def private_file(path):
    item = path.lstat()
    if not stat.S_ISREG(item.st_mode) or item.st_uid != os.getuid() or stat.S_IMODE(item.st_mode) != 0o600:
        raise RuntimeError('run metadata must be an owned private regular file')


def atomic_write(path, text):
    temporary = path.with_name(path.name + '.tmp')
    fd = os.open(str(temporary), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as stream:
        stream.write(text)
    os.replace(str(temporary), str(path))


def write_authority(path, number):
    # Xau binary format; the cookie never appears in argv, stdout, or metadata.
    cookie = secrets.token_bytes(16)
    fields = (socket.gethostname().encode(), str(number).encode(), b'MIT-MAGIC-COOKIE-1', cookie)
    record = struct.pack('!H', 256) + b''.join(struct.pack('!H', len(value)) + value for value in fields)
    fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(record)


def validate_ready(run, expected_pid, expected_start):
    work = run / 'gui-display'
    directory = work.lstat()
    if not stat.S_ISDIR(directory.st_mode) or directory.st_uid != os.getuid() or stat.S_IMODE(directory.st_mode) != 0o700:
        raise RuntimeError('display directory is not private/owned')
    for name in ('ready.json', 'environment.sh'):
        private_file(work / name)
    data = json.loads((work / 'ready.json').read_text())
    manager = manager_owner(run)
    rows = [row.split('\t') for row in (run / 'processes.tsv').read_text().splitlines()]
    owners = [row for row in rows if len(row) == 3 and row[0] == 'gui_display']
    if len(owners) != 1 or not owners[0][1].isdigit():
        raise RuntimeError('missing or ambiguous helper identity')
    helper_pid, helper_start = int(owners[0][1]), owners[0][2]
    if helper_pid != expected_pid or helper_start != expected_start:
        raise RuntimeError('helper identity does not match manager pinned PID/start')
    helper = {'pid': helper_pid, 'start_time': helper_start}
    if (type(data['version']) is not int or data['version'] != 1 or type(data['gui']) is not bool or data['manager'] != identity(manager) or data['helper'] != helper
            or not matches(helper, executable='/usr/bin/python3', parent=manager['pid'],
                           argument=str(SOURCE.with_name('gui_display.py')))
            or not usable_modes(data['selected_probe'])):
        raise RuntimeError('readiness owner/capability mismatch')
    text = environment_text(data['environment'])
    if ((work / 'environment.sh').read_text() != text
            or data['environment_sha256'] != hashlib.sha256(text.encode()).hexdigest()):
        raise RuntimeError('environment metadata mismatch')
    if data['backend'] == 'nested':
        nested = data['nested']
        if (not matches(nested, executable='/usr/bin/Xephyr', parent=helper_pid,
                        argument=data['title']) or data['environment']['XAUTHORITY'] != str(work / 'Xauthority')
                or data['environment']['DISPLAY'] != ':' + str(data['display_number'])):
            raise RuntimeError('nested owner mismatch')
        private_file(work / 'Xauthority')
        if (not matches(nested, argument=data['environment']['DISPLAY'])
                or not matches(nested, argument=str(work / 'Xauthority'))):
            raise RuntimeError('nested display/auth arguments mismatch')
        window = data['window']
        if (window['owner_valid'] is not True or window['title_match_count'] != 1
                or window['xres_pid'] != nested['pid'] or window['x_error']
                or window['map_state'] != (2 if data['gui'] else 0)):
            raise RuntimeError('owned window readiness mismatch')
    elif (data['backend'] != 'native' or data['window'] is not None or data['nested'] is not None
          or data['environment']['DISPLAY'] != data['parent_display']
          or data['environment'].get('XAUTHORITY') != data['parent_authority']):
        raise RuntimeError('invalid native metadata')
    if (data['selected_probe']['display'] != data['environment']['DISPLAY']
            or data['probe_source_sha256'] != hashlib.sha256(SOURCE.read_bytes()).hexdigest()):
        raise RuntimeError('probe provenance/display mismatch')
    # Validate current capabilities and window owner as well as the ready snapshot.
    selected_environment = dict(os.environ, **data['environment'])
    if 'XAUTHORITY' not in data['environment']:
        selected_environment.pop('XAUTHORITY', None)
    result = subprocess.run([str(work / 'probe'), 'query'], env=selected_environment,
                            capture_output=True, text=True, timeout=5)
    if result.returncode or not usable_modes(json.loads(result.stdout)):
        raise RuntimeError('selected display capability changed')
    if data['backend'] == 'nested':
        parent_environment = os.environ.copy()
        parent_environment['DISPLAY'] = data['parent_display']
        if data['parent_authority'] is None:
            parent_environment.pop('XAUTHORITY', None)
        else:
            parent_environment['XAUTHORITY'] = data['parent_authority']
        result = subprocess.run([str(work / 'probe'), 'window', data['title'],
                                 str(nested['pid']), nested['start_time']], env=parent_environment,
                                 capture_output=True, text=True, timeout=5)
        current = json.loads(result.stdout)
        if result.returncode or not current.get('owner_valid') or current.get('x_error') or current.get('map_state') != (2 if data['gui'] else 0):
            raise RuntimeError('current owned window mismatch')
    return data


class DisplayOwner:
    def __init__(self, run, gui):
        self.run_directory, self.gui = run, gui
        self.work = run / 'gui-display'
        self.manager = manager_owner(run)
        self.helper = identity(process(os.getpid()))
        if os.getppid() != self.manager['pid']:
            raise RuntimeError('helper is not a child of its manager')
        self.child = None
        self.nested = None
        self.title = None
        self.stop_requested = False
        self.notified = False
        self.original = os.environ.copy()
        self.selected = None
        self.binary = self.work / 'probe'

    def journal(self, event, **values):
        with (self.work / 'events.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(event=event, monotonic=time.monotonic(), **values)) + '\n')

    def probe(self, operation, environment, *arguments):
        result = subprocess.run([str(self.binary), operation, *map(str, arguments)],
                                env=environment, capture_output=True, text=True, timeout=5)
        # Never copy the process environment or auth file contents into logs.
        if not result.stdout:
            raise RuntimeError('display probe returned no metadata')
        data = json.loads(result.stdout)
        return result.returncode, data

    def window(self, operation):
        code, data = self.probe(operation, self.original, self.title,
                                self.nested['pid'], self.nested['start_time'])
        if code or not data.get('owner_valid') or data.get('x_error'):
            raise RuntimeError('cannot verify owned Xephyr window')
        return data

    def ensure_window(self):
        data = self.window('window')
        if data['map_state'] != (2 if self.gui else 0):
            data = self.window('map' if self.gui else 'unmap')
        return data

    def initialize(self):
        self.work.mkdir(mode=0o700)
        if not LOCAL_DISPLAY.fullmatch(self.original.get('DISPLAY', '')):
            raise RuntimeError('a local parent X display is required')
        compile_result = subprocess.run(['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                         str(SOURCE), '-o', str(self.binary), '-lX11', '-lXrandr', '-l:libXRes.so.1'],
                         capture_output=True, text=True, timeout=30)
        (self.work / 'compile.log').write_text(compile_result.stdout + compile_result.stderr)
        if compile_result.returncode:
            raise RuntimeError('display probe compile failed; see gui-display/compile.log')
        code, native = self.probe('query', self.original)
        self.journal('parent_probe', result=native, returncode=code)
        environment = {'DISPLAY': self.original['DISPLAY']}
        if 'XAUTHORITY' in self.original:
            environment['XAUTHORITY'] = self.original['XAUTHORITY']
        window = None
        number = None
        if code == 0 and usable_modes(native):
            backend, selected = 'native', native
        else:
            if not native.get('display_open'):
                raise RuntimeError('cannot open parent desktop display')
            # Do not reuse any X server/socket. Xephyr's own lock still arbitrates a race;
            # a failed launch fails this run instead of attaching to the other server.
            number = next((value for value in range(100, 200)
                           if not Path('/tmp/.X{}-lock'.format(value)).exists()
                           and not Path('/tmp/.X11-unix/X{}'.format(value)).exists()), None)
            if number is None:
                raise RuntimeError('no unused nested display number')
            auth = self.work / 'Xauthority'
            write_authority(auth, number)
            self.title = 'PX4 Gazebo display ' + secrets.token_hex(16)
            log = (self.work / 'xephyr.log').open('w')
            try:
                self.child = subprocess.Popen(['/usr/bin/Xephyr', ':' + str(number), '-screen', '1280x720',
                              '-title', self.title, '-nolisten', 'tcp', '-noreset', '-auth', str(auth)],
                              env=self.original, stdout=log, stderr=subprocess.STDOUT)
            finally:
                log.close()
            deadline = time.monotonic() + 20
            while not self.nested and time.monotonic() < deadline:
                if self.child.poll() is not None:
                    raise RuntimeError('owned Xephyr exited during startup')
                item = process(self.child.pid)
                if item and item['executable'] == str(Path('/usr/bin/Xephyr').resolve()):
                    self.nested = identity(item)
                else:
                    time.sleep(0.05)
            if self.nested is None:
                raise RuntimeError('cannot capture Xephyr identity')
            environment = {'DISPLAY': ':' + str(number), 'XAUTHORITY': str(auth)}
            self.selected = dict(self.original, **environment)
            selected = None
            while time.monotonic() < deadline:
                if self.child.poll() is not None or not matches(self.nested, executable='/usr/bin/Xephyr', parent=os.getpid()):
                    raise RuntimeError('owned Xephyr exited before readiness')
                code, candidate = self.probe('query', self.selected)
                if code == 0 and usable_modes(candidate):
                    try:
                        window = self.ensure_window()
                        if window['map_state'] == (2 if self.gui else 0):
                            selected = candidate
                            break
                    except RuntimeError:
                        pass
                time.sleep(0.2)
            if selected is None:
                raise RuntimeError('nested modes/window readiness timed out')
            backend = 'nested'
            self.journal('nested_probe', result=selected)
            self.journal('owned_window', result=window, gui=self.gui)
        self.selected = dict(self.original, **environment)
        text = environment_text(environment)
        atomic_write(self.work / 'environment.sh', text)
        ready = dict(version=1, backend=backend, manager=identity(self.manager), helper=self.helper,
                     gui=self.gui, environment=environment, parent_display=self.original['DISPLAY'],
                     parent_authority=self.original.get('XAUTHORITY'),
                     environment_sha256=hashlib.sha256(text.encode()).hexdigest(),
                     parent_probe=native, selected_probe=selected,
                     title=self.title, nested=self.nested, display_number=number, window=window,
                     probe_source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest())
        atomic_write(self.work / 'ready.json', json.dumps(ready, indent=2) + '\n')
        self.journal('ready', backend=backend, gui=self.gui)

    def sitl_alive(self):
        try:
            rows = (self.run_directory / 'processes.tsv').read_text().splitlines()
        except OSError:
            return False
        for row in rows:
            fields = row.split('\t')
            if len(fields) == 3 and fields[0] == 'sitl' and fields[1].isdigit():
                if matches({'pid': int(fields[1]), 'start_time': fields[2]}):
                    return True
        return False

    def request_stop(self):
        self.stop_requested = True
        if not self.notified and matches(self.manager, argument=self.manager['script']):
            self.notified = True
            self.journal('request_manager_stop', reason='display_helper_stop_or_failure')
            try:
                os.kill(self.manager['pid'], signal.SIGTERM)
            except ProcessLookupError:
                pass

    def run(self):
        signal.signal(signal.SIGTERM, lambda *_: self.request_stop())
        signal.signal(signal.SIGINT, lambda *_: self.request_stop())
        signal.signal(signal.SIGHUP, lambda *_: self.request_stop())
        try:
            self.initialize()
            while True:
                manager_alive = matches(self.manager, argument=self.manager['script'])
                if not manager_alive:
                    self.stop_requested = True
                if self.stop_requested and not self.sitl_alive():
                    break
                if self.child and not self.stop_requested:
                    try:
                        if self.child.poll() is not None or not matches(self.nested, executable='/usr/bin/Xephyr', parent=os.getpid()):
                            raise RuntimeError('owned Xephyr exited')
                        window = self.ensure_window()
                        if window['map_state'] != (2 if self.gui else 0):
                            raise RuntimeError('owned window visibility lost')
                        if window['changed']:
                            self.journal('owned_window', result=window, gui=self.gui)
                    except (RuntimeError, subprocess.TimeoutExpired, ValueError, OSError):
                        self.journal('display_failure')
                        self.request_stop()
                time.sleep(2)
        finally:
            # On ordinary initialization failure no SITL has been launched. On a
            # later stop retain rendering while the manager lands/restores/stops it.
            while self.sitl_alive():
                self.request_stop()
                time.sleep(1)
            self.teardown()

    def teardown(self):
        if self.child and self.child.poll() is None:
            if self.nested and matches(self.nested, executable='/usr/bin/Xephyr', parent=os.getpid()):
                self.child.terminate()
                try:
                    self.child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    if matches(self.nested, executable='/usr/bin/Xephyr', parent=os.getpid()):
                        self.child.kill()
                        self.child.wait(timeout=2)
            else:
                self.journal('teardown_owner_mismatch_no_signal')
        self.journal('stopped')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-directory', required=True, type=Path)
    parser.add_argument('--gui', choices=('true', 'false'), default='false')
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--check-ready', action='store_true')
    action.add_argument('--export-environment', action='store_true')
    parser.add_argument('--helper-pid', type=int)
    parser.add_argument('--helper-start')
    args = parser.parse_args()
    run = args.run_directory.resolve()
    try:
        if args.check_ready or args.export_environment:
            data = validate_ready(run, args.helper_pid, args.helper_start)
            if args.export_environment:
                values = ('DISPLAY', data['environment']['DISPLAY'], 'XAUTHORITY',
                          data['environment'].get('XAUTHORITY', ''))
                sys.stdout.buffer.write(b''.join(value.encode() + b'\0' for value in values))
        else:
            DisplayOwner(run, args.gui == 'true').run()
    except (OSError, ValueError, RuntimeError, KeyError, TypeError, subprocess.TimeoutExpired) as exc:
        print('Display preparation failed: {}'.format(type(exc).__name__), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
