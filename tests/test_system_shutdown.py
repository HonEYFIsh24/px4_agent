"""Exercise the production shell cleanup after its terminal pipe disappears."""
import os
from pathlib import Path
import shlex
import signal
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ClosedTerminalTests(unittest.TestCase):
    def run_closed_terminal(self, trigger):
        source = (ROOT/'scripts/start_system.sh').read_text()
        # Use the actual production functions/traps with one harmless child.
        functions = source[source.index('start_time() {'):source.index('get_state() {')]
        cleanup = source[source.index('shutdown_component() {'):source.index('/usr/bin/python3 - "$$"')]
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            harness = run/'manager.sh'
            harness.write_text('set -Eeuo pipefail\n'
                + 'PROJECT_DIR='+shlex.quote(str(ROOT))+'\nRUN_DIR='+shlex.quote(temporary)+'\n'
                + 'declare -A PIDS STARTS\nPIDS=(); STARTS=()\nCLEANING=0\n'
                + 'CONTROL_UI=desktop\nEV_PROFILE=shadow\nPROFILE_RESTORE_FILE=""\n'
                + functions + cleanup
                + 'sleep 60 &\nPIDS[web]=$!\nSTARTS[web]=$(start_time "$!")\n'
                + 'echo "${PIDS[web]}" > "$RUN_DIR/child.pid"\necho ready\n'
                + 'while :; do sleep 0.05; '+('echo heartbeat' if trigger == 'pipe' else ':')+'; done\n')
            process = subprocess.Popen(['bash', str(harness)], stdout=subprocess.PIPE,
                                       stderr=subprocess.DEVNULL, start_new_session=True)
            child = None
            try:
                self.assertEqual(process.stdout.readline(), b'ready\n')
                child = int((run/'child.pid').read_text())
                process.stdout.close()
                if trigger == 'term':
                    process.send_signal(signal.SIGTERM)
                self.assertEqual(process.wait(timeout=25), 143 if trigger == 'term' else 141)
                self.assertTrue((run/'exit-code').exists())
                self.assertIn('本次启动的服务已关闭', (run/'cleanup.log').read_text())
                with self.assertRaises(ProcessLookupError):
                    os.kill(child, 0)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
                if child:
                    try:
                        os.kill(child, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                process.stdout.close()

    def test_sigterm_cleanup_survives_an_already_closed_terminal(self):
        self.run_closed_terminal('term')

    def test_broken_pipe_enters_cleanup_and_reaps_owned_child(self):
        self.run_closed_terminal('pipe')


class RestartWaitTests(unittest.TestCase):
    def setUp(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location('wait_system_shutdown', ROOT/'scripts/wait_system_shutdown.py')
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.run = Path(self.temporary.name)
        import json
        (self.run/'controller.json').write_text(json.dumps({
            'pid': 100, 'start_time': '10', 'script': str(ROOT/'scripts/start_system.sh')}))
        self.processes = {
            100: {'start_time': '10', 'ppid': 99, 'arguments': ['bash', str(ROOT/'scripts/start_system.sh')]},
            99: {'start_time': '9', 'ppid': 1, 'arguments': ['roslaunch', 'px4_agent', 'uav_system.launch']}}

    def test_active_window_is_not_stopped_or_waited_on(self):
        self.assertFalse(self.module.wait_for_shutdown(self.run, inspect=self.processes.get,
            sleep=lambda _: self.fail('active system must not be waited on')))

    def test_closing_waits_for_both_manager_and_outer_roslaunch(self):
        (self.run/'cleanup.log').touch()
        sleeps = []
        def sleep(_):
            sleeps.append(True)
            self.processes.pop(100 if len(sleeps) == 1 else 99)
        self.assertTrue(self.module.wait_for_shutdown(self.run, inspect=self.processes.get, sleep=sleep))
        self.assertEqual(len(sleeps), 2)

    def test_reused_pid_does_not_delay_unrelated_process(self):
        (self.run/'desktop.closed').touch()
        self.processes[100]['start_time'] = 'new'
        self.assertFalse(self.module.wait_for_shutdown(self.run, inspect=self.processes.get,
            sleep=lambda _: self.fail('reused PID must not be waited on')))


if __name__ == '__main__':
    unittest.main()
