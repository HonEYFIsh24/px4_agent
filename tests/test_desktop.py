"""Native desktop contracts with local fixtures; never connect to a model or ROS."""
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock

try:
    from PIL import Image
    from PySide6.QtWidgets import QApplication
    from PySide6.QtCore import Qt
    from PySide6.QtGui import QInputMethodEvent, QFontInfo
    from desktop import DesktopRoot, DesktopWindow, camera_bytes, vector_text
except ImportError as exc:
    raise unittest.SkipTest("Install requirements-desktop.txt to run desktop tests") from exc

from test_web_server import FakeBridge, application, completed


class DesktopDataTests(unittest.TestCase):
    def test_invalid_or_missing_coordinates_are_never_shown_as_position(self):
        for value in (None, [], [1, 2], [1, float('nan'), 3], [1, float('inf'), 3]):
            self.assertEqual(vector_text(value), "— / — / —")
        self.assertEqual(vector_text([1, 2, 3]), "1.00 / 2.00 / 3.00")

    def test_camera_requires_fresh_file_and_live_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'frame.jpg'
            path.write_bytes(b'frame')
            now = path.stat().st_mtime
            self.assertEqual(camera_bytes(path, {'available': True}, now), b'frame')
            self.assertIsNone(camera_bytes(path, {'available': False}, now))
            self.assertIsNone(camera_bytes(path, {'available': True}, now+3))
            self.assertIsNone(camera_bytes(path, {'available': True}, now-3))
            path.unlink()
            self.assertIsNone(camera_bytes(path, {'available': True}, now))


@unittest.skipUnless(os.getenv('UAV_DESKTOP_UI_TEST') == '1', 'requires explicit desktop display test')
class NativeWindowTests(unittest.TestCase):
    def setUp(self):
        self.qt = QApplication.instance() or QApplication([])
        self.qt.setQuitOnLastWindowClosed(False)
        self.root = DesktopRoot()
        self.root.setWindowTitle('桌面自动测试 · 模拟数据')
        self.bridge = FakeBridge()
        self.release = threading.Event()
        self.addCleanup(self.release.set)
        self.app = application(self.bridge, lambda *_: {'messages': [{'role': 'assistant', 'content': 'fixture'}]})
        self.settings = tempfile.TemporaryDirectory()
        self.addCleanup(self.settings.cleanup)
        self.window = DesktopWindow(self.root, self.app, start_polling=False,
                                    settings_path=Path(self.settings.name)/'desktop.json')
        self.root.show()
        self.root.setWindowTitle('桌面自动测试 · 模拟数据')
        self.addCleanup(self.window.destroy)
        self.state = {**self.bridge.state(), 'connected': True, 'landed_state': 1,
                      'camera': {'left': {'available': False}, 'right': {'available': False}}}
        self.window.render_state(self.state, None)
        self.qt.processEvents()

    def pump_until(self, predicate):
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            self.qt.processEvents()
            if predicate():
                return
            time.sleep(.01)
        self.fail('desktop event did not complete')

    def test_chinese_composition_and_commit_in_command_input(self):
        prompt = self.window.prompt
        prompt.clear()
        prompt.setFocus()
        self.assertTrue(prompt.testAttribute(Qt.WidgetAttribute.WA_InputMethodEnabled))
        preedit = QInputMethodEvent('zhongwen', [])
        QApplication.sendEvent(prompt, preedit)
        self.assertEqual(prompt.toPlainText(), '')
        self.assertIsNone(self.app.active_id, 'composition must never submit a task')
        commit = QInputMethodEvent()
        commit.setCommitString('中文输入：查看无人机状态')
        QApplication.sendEvent(prompt, commit)
        self.assertEqual(prompt.toPlainText(), '中文输入：查看无人机状态')
        self.assertIsNone(self.app.active_id)

    def test_scalable_chinese_font_and_persistent_size(self):
        self.assertIn(QFontInfo(self.window.prompt.font()).family(),
                      ('Noto Sans CJK SC', 'WenQuanYi Micro Hei'))
        self.window.resize_font(2)
        self.assertEqual(self.window.prompt.font().pointSize(), 14)
        from desktop import preferred_font_size
        self.assertEqual(preferred_font_size(self.window.settings_path), 14)

    def test_tracking_alone_does_not_claim_vio_controls_the_aircraft(self):
        self.window.render_state({**self.state, 'vio': {'ready': True},
            'external_vision': {'used_for_flight_control': False}}, None)
        self.assertIn('未用于飞控', self.window.metrics['vio'].text())

    def test_actual_vio_fusion_and_variable_height_are_shown(self):
        self.window.render_state({**self.state,
            'external_vision': {'used_for_flight_control': True, 'ekf': {'gps_fusion': False}},
            'navigation': {'method': 'static_3d'}}, None)
        self.assertIn('VIO 主定位', self.window.metrics['vio'].text())
        self.assertIn('GPS 融合关闭', self.window.metrics['vio'].text())
        self.assertIn('1～10', self.window.hint.text())

    def test_land_is_dispatched_while_the_model_is_blocked(self):
        entered = threading.Event()
        def blocked(*_):
            entered.set()
            self.release.wait(5)
            return {'messages': [{'role': 'assistant', 'content': 'fixture'}]}
        self.app.agent.script = blocked
        self.window.submit()
        self.pump_until(entered.is_set)
        self.window.control('/land')
        self.pump_until(lambda: '/land' in self.bridge.calls)
        with self.app.lock:
            self.assertTrue(self.app.current_command['interrupted'])
        self.assertFalse(self.release.is_set(), 'landing must not wait for the model')

    def test_land_cancels_a_task_that_has_not_been_accepted_yet(self):
        self.app.control_lock.acquire()
        try:
            self.window.submit()
            self.window.control('/land')
        finally:
            self.app.control_lock.release()
        self.pump_until(lambda: not self.window.submitting and '/land' in self.bridge.calls)
        self.assertEqual(self.app.commands, {})

    def test_observe_checkbox_keeps_the_audited_no_flight_guard(self):
        def attempt_flight(client, *_):
            client.request('/mission', {'steps': [{'action': 'takeoff', 'altitude': 3.5}]})
            return {'messages': []}
        self.app.agent.script = attempt_flight
        self.window.observe.setChecked(True)
        self.window.submit()
        self.pump_until(lambda: self.window.command_id is not None)
        result = completed(self.app, self.window.command_id)
        self.assertEqual(result['mode'], 'observe')
        self.assertEqual(result['status'], 'failed')
        self.assertNotIn('/mission', self.bridge.calls)

    def test_disconnection_clears_camera_and_live_trail(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'left.jpg'
            Image.new('RGB', (32, 24), '#227755').save(path)
            self.app.camera_paths['left'] = path
            self.window.state['camera']['left']['available'] = True
            self.window.render_cameras(True)
            self.assertIn('left', self.window.images)
            self.window.render_cameras(False)
            self.assertNotIn('left', self.window.images)
            self.assertTrue(self.window.camera_labels['left'].pixmap().isNull())
        self.window.render_state({'ready': False, 'connected': False}, None)
        self.assertEqual(self.window.trail, [])
        self.window.buttons()
        self.assertFalse(self.window.land_button.isEnabled())

    def test_close_during_flight_waits_for_confirmed_landing(self):
        self.window.render_state({**self.state, 'armed': True, 'landed_state': 2}, None)
        with mock.patch.object(self.window, 'confirm_close', return_value=True):
            self.window.request_close()
        self.pump_until(lambda: '/land' in self.bridge.calls)
        self.assertFalse(self.window.stopped.is_set())
        self.window.render_state({**self.state, 'armed': False, 'landed_state': 1}, None)
        self.assertTrue(self.window.stopped.is_set())

    def test_offline_state_does_not_forget_the_last_armed_measurement(self):
        self.window.render_state({**self.state, 'armed': True, 'landed_state': 2}, None)
        self.window.render_state({'connected': False, 'ready': False}, None)
        with mock.patch.object(self.window, 'confirm_close', return_value=False) as question:
            self.window.request_close()
        question.assert_called_once()
        self.assertFalse(self.window.stopped.is_set())

    def test_window_teardown_blocks_late_model_flight_actions(self):
        entered = threading.Event()
        def late_action(client, *_):
            entered.set()
            self.release.wait(5)
            client.request('/mission', {'steps': [{'action': 'takeoff', 'altitude': 3.5}]})
            return {'messages': []}
        self.app.agent.script = late_action
        self.window.submit()
        self.pump_until(entered.is_set)
        command_id = self.app.active_id
        self.window.destroy()
        self.release.set()
        result = completed(self.app, command_id)
        self.assertEqual(result['status'], 'failed')
        self.assertNotIn('/mission', self.bridge.calls)


if __name__ == '__main__':
    unittest.main()
