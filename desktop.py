"""Native operator window; shares the audited Agent without starting a web server."""
import argparse
import json
import math
import os
from pathlib import Path
import queue
import signal
import threading
import time
from PySide6.QtCore import Qt, QTimer, QPointF
from PySide6.QtGui import QColor, QFont, QFontDatabase, QPainter, QPen, QPixmap, QShortcut, QKeySequence
from PySide6.QtWidgets import (QApplication, QCheckBox, QHBoxLayout, QLabel, QMainWindow,
                               QMessageBox, QPlainTextEdit, QPushButton, QSplitter,
                               QSizePolicy, QTabWidget, QVBoxLayout, QWidget)

from web_server import DEFAULT_ENV_FILE, build_application

STATUS = {"running": "执行中", "succeeded": "已完成", "failed": "失败", "cancelled": "已取消"}
ACTIONS = {"takeoff": "起飞", "goto": "前往目标", "hold": "悬停", "return_home": "返航", "land": "降落"}
LANDED = {0: "未知", 1: "已着陆", 2: "空中", 3: "起飞中", 4: "降落中"}
SETTINGS_PATH = Path(os.getenv("XDG_CONFIG_HOME", str(Path.home()/".config"))) / "px4_agent" / "desktop.json"


def preferred_font_size(path):
    try:
        value = json.loads(Path(path).read_text()).get("font_size")
        return value if type(value) is int and 10 <= value <= 24 else 12
    except (OSError, ValueError, AttributeError):
        return 12


def vector_text(value):
    if (not isinstance(value, (list, tuple)) or len(value) < 3
            or any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in value[:3])):
        return "— / — / —"
    return " / ".join("{:.2f}".format(v) for v in value[:3])


def camera_bytes(path, metadata, now=None):
    """Never display a stale file or a frame marked unavailable by the bridge."""
    if not metadata.get("available"):
        return None
    try:
        path = Path(path)
        if not 0 <= (time.time() if now is None else now) - path.stat().st_mtime <= 2:
            return None
        data = path.read_bytes()
        return data if 0 < len(data) <= 2_000_000 else None
    except OSError:
        return None


class DesktopRoot(QMainWindow):
    """Route window-manager closes through the landing confirmation workflow."""
    def closeEvent(self, event):
        if self.controller.stopped.is_set():
            event.accept()
        else:
            event.ignore()
            self.controller.request_close()


class TrailView(QWidget):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.setMinimumSize(160, 160)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        painter.fillRect(self.rect(), QColor("#172332"))
        trail = self.controller.trail
        extent = max(20., max((max(abs(x), abs(y)) for x, y in trail), default=0.)+2)
        scale = min(self.width(), self.height())/(extent*2+4)
        def point(x, y):
            return QPointF(self.width()/2+x*scale, self.height()/2-y*scale)
        painter.setPen(QColor("#2a3b50"))
        for n in range(-int(extent), int(extent)+1, 5):
            painter.drawLine(point(n, -extent), point(n, extent))
            painter.drawLine(point(-extent, n), point(extent, n))
        painter.setPen(QColor("#a7b6c8"))
        painter.drawText(16, 30, "北 / +Y ↑     东 / +X →")
        painter.setPen(QPen(QColor("#4fd5bf"), 2))
        for previous, current in zip(trail, trail[1:]):
            painter.drawLine(point(*previous), point(*current))
        if trail:
            painter.setBrush(QColor("#4fd5bf"))
            painter.drawEllipse(point(*trail[-1]), 5, 5)
        else:
            painter.setPen(QColor("#a7b6c8"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "等待有效的实时位置")


class DesktopWindow:
    def __init__(self, root, application, start_polling=True, settings_path=SETTINGS_PATH):
        self.root, self.application = root, application
        self.settings_path = Path(settings_path)
        self.font_size = preferred_font_size(self.settings_path)
        self.events = queue.Queue()
        self.stopped = threading.Event()
        self.state = {}
        self.command_id = None
        self.last_state_at = 0.
        self.last_reply = None
        self.control_busy = False
        self.submitting = False
        self.pending_submission = threading.Event()
        self.closing = False
        self.flight_may_be_active = False
        self.trail = []
        self.images = {}
        self.frame_bytes = {}
        self.last_camera_at = 0.
        root.controller = self
        root.setWindowTitle("PX4 Agent · 桌面控制台")
        screen = root.screen().availableGeometry()
        root.resize(min(1220, int(screen.width()*.92)), min(900, int(screen.height()*.9)))
        root.setMinimumSize(min(960, screen.width()), min(720, screen.height()))
        families = QFontDatabase.families()
        self.font_family = next((name for name in ("Noto Sans CJK SC", "WenQuanYi Micro Hei", "sans-serif")
                                 if name in families), "sans-serif")
        self.ui_font = QFont(self.font_family, self.font_size)
        self.ui_font.setStyleStrategy(QFont.StyleStrategy.PreferAntialias)
        root.setFont(self.ui_font)
        root.setStyleSheet("""
            QMainWindow, QWidget { background: #eef2f6; color: #172332; }
            QLabel { background: transparent; }
            QPushButton { background: #ffffff; border: 1px solid #cbd5e1; border-radius: 6px; padding: 9px 14px; }
            QPushButton:hover { background: #dfeaf5; }
            QPushButton:disabled { color: #8995a4; background: #e4e8ed; }
            QPushButton#land { background: #b52e39; color: white; border: 0; }
            QPushButton#land:disabled { background: #d8a9ad; }
            QPlainTextEdit { background: white; border: 1px solid #cbd5e1; border-radius: 6px; padding: 8px; selection-background-color: #bad8f4; }
            QTabBar::tab { padding: 9px 16px; }
            QSplitter::handle { background: #d8e0e9; width: 6px; }
        """)
        central = QWidget()
        root.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(20, 14, 20, 14)
        layout.setSpacing(10)
        header = QHBoxLayout()
        self.title = QLabel("PX4 桌面控制台")
        header.addWidget(self.title)
        header.addStretch()
        self.connection = QLabel("正在连接执行器…")
        header.addWidget(self.connection)
        self.font_label = QLabel()
        for text, increment in (("A−", -1), ("A+", 1)):
            button = QPushButton(text)
            button.clicked.connect(lambda checked=False, step=increment: self.resize_font(step))
            header.addWidget(button)
            if increment == -1:
                header.addWidget(self.font_label)
        layout.addLayout(header)
        self.banner = QLabel("等待实时遥测")
        self.banner.setWordWrap(True)
        layout.addWidget(self.banner)
        self.panes = QSplitter(Qt.Orientation.Horizontal)
        layout.addWidget(self.panes, 1)
        notebook = QTabWidget()
        self.panes.addWidget(notebook)
        cameras = QWidget()
        camera_layout = QVBoxLayout(cameras)
        self.camera_labels = {}
        for side, title in (("left", "左相机"), ("right", "右相机")):
            camera_layout.addWidget(QLabel(title))
            label = QLabel("等待新鲜图像")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setMinimumSize(160, 90)
            label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
            label.setStyleSheet("background: #172332; color: #c3d0df; border-radius: 6px;")
            camera_layout.addWidget(label, 1)
            self.camera_labels[side] = label
        notebook.addTab(cameras, "双目相机")
        map_frame = QWidget()
        map_layout = QVBoxLayout(map_frame)
        map_layout.addWidget(QLabel("Gazebo 世界坐标 · 米 · 仅显示实时测量"))
        self.map = TrailView(self)
        map_layout.addWidget(self.map, 1)
        notebook.addTab(map_frame, "航迹监测")
        right = QWidget()
        right_layout = QVBoxLayout(right)
        self.panes.addWidget(right)
        self.panes.setSizes([650, 500])
        self.metrics = {}
        for name, title in (("flight", "飞控"), ("position", "本地 ENU / m"),
                            ("vio", "视觉惯性定位"), ("mission", "执行器任务")):
            caption = QLabel(title)
            caption.setStyleSheet("color: #657386;")
            right_layout.addWidget(caption)
            self.metrics[name] = QLabel("—")
            self.metrics[name].setWordWrap(True)
            self.metrics[name].setTextFormat(Qt.TextFormat.PlainText)
            right_layout.addWidget(self.metrics[name])
        controls = QHBoxLayout()
        self.stop_button = QPushButton("停止 / 悬停")
        self.stop_button.clicked.connect(lambda: self.control("/stop"))
        self.land_button = QPushButton("立即降落")
        self.land_button.setObjectName("land")
        self.land_button.clicked.connect(lambda: self.control("/land"))
        controls.addWidget(self.stop_button)
        controls.addWidget(self.land_button)
        right_layout.addLayout(controls)
        right_layout.addWidget(QLabel("任务进展与结果"))
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        right_layout.addWidget(self.output, 1)
        footer = QHBoxLayout()
        self.model = QLabel("模型：" + (application.model.get("name") or "未配置，可使用直接控制"))
        footer.addWidget(self.model, 1)
        self.observe = self.observe_button = QCheckBox("只观察（禁止任务移动）")
        footer.addWidget(self.observe_button)
        layout.addLayout(footer)
        entry = QHBoxLayout()
        self.prompt = QPlainTextEdit()
        self.prompt.setObjectName("commandInput")
        self.prompt.setAttribute(Qt.WidgetAttribute.WA_InputMethodEnabled, True)
        self.prompt.setPlaceholderText("输入中文指令，例如：查看无人机当前状态")
        self.prompt.setPlainText("查看无人机当前状态。")
        self.prompt.setMinimumHeight(88)
        self.prompt.setMaximumHeight(160)
        entry.addWidget(self.prompt, 1)
        self.send_button = QPushButton("发送指令")
        self.send_button.clicked.connect(self.submit)
        entry.addWidget(self.send_button)
        layout.addLayout(entry)
        self.hint = hint = QLabel("Ctrl+Enter 发送 · 飞行使用本地 ENU 坐标 · 等待定位与导航状态")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #657386;")
        layout.addWidget(hint)
        self.shortcuts = []
        for key, callback in (("Ctrl++", lambda: self.resize_font(1)), ("Ctrl+=", lambda: self.resize_font(1)),
                              ("Ctrl+-", lambda: self.resize_font(-1)), ("Ctrl+0", lambda: self.resize_font(12-self.font_size)),
                              ("Ctrl+Return", self.submit), ("Ctrl+Enter", self.submit)):
            shortcut = QShortcut(QKeySequence(key), root)
            shortcut.activated.connect(callback)
            self.shortcuts.append(shortcut)
        self.apply_font()
        self.buttons()
        if start_polling:
            threading.Thread(target=self.poll, daemon=True, name="desktop-telemetry").start()
        self.timer = QTimer(root)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)

    def apply_font(self):
        self.ui_font.setPointSize(self.font_size)
        self.root.setFont(self.ui_font)
        # Styles and reparented widgets can otherwise retain the desktop's
        # default font instead of inheriting this window's Chinese font.
        for widget in self.root.findChildren(QWidget):
            widget.setFont(self.ui_font)
        title_font = QFont(self.ui_font)
        title_font.setPointSize(self.font_size+6)
        title_font.setWeight(QFont.Weight.Medium)
        self.title.setFont(title_font)
        self.font_label.setText("字号 {}".format(self.font_size))
        self.draw_map()

    def resize_font(self, increment):
        self.font_size = max(10, min(24, self.font_size+increment))
        self.apply_font()
        try:
            self.settings_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.settings_path.with_suffix(".tmp")
            temporary.write_text(json.dumps({"font_size": self.font_size}) + "\n")
            temporary.replace(self.settings_path)
        except OSError:
            pass

    def background(self, kind, operation):
        def worker():
            try:
                self.events.put((kind, operation()))
            except Exception as exc:
                self.events.put(("error", (kind, self.application.clean_error(exc))))
        threading.Thread(target=worker, daemon=True, name="desktop-" + kind).start()

    def poll(self):
        while not self.stopped.is_set():
            state = self.application.state()
            command = self.application.command(self.command_id) if self.command_id else None
            self.events.put(("telemetry", (state, command)))
            self.stopped.wait(.5)

    def show_output(self, text):
        if text == self.last_reply:
            return
        self.last_reply = text
        self.output.setPlainText(text)

    def buttons(self):
        connected = self.state.get("connected") is True and time.monotonic()-self.last_state_at < 2
        busy = self.submitting or bool(self.state.get("active_command_id")) or self.closing
        self.send_button.setEnabled(bool(connected and self.application.agent and not busy))
        self.observe_button.setEnabled(not busy)
        for button in (self.stop_button, self.land_button):
            button.setEnabled(bool(connected and not self.control_busy))

    def submit(self):
        if self.submitting or self.closing or self.state.get("active_command_id"):
            return
        prompt = self.prompt.toPlainText().strip()
        mode = "observe" if self.observe.isChecked() else "task"
        self.submitting = True
        cancellation = self.pending_submission = threading.Event()
        self.buttons()
        def dispatch():
            # Serialize acceptance with operator interrupts. A click on Land
            # must also cancel a submission that has not reached the app yet.
            with self.application.control_lock:
                if cancellation.is_set():
                    raise RuntimeError("待提交指令已被操作员取消")
                return self.application.submit(prompt, mode)
        self.background("submitted", dispatch)

    def control(self, path):
        if self.control_busy:
            return
        self.pending_submission.set()
        self.control_busy = True
        self.buttons()
        self.background("control", lambda: self.application.interrupt(path))

    def render_state(self, state, command):
        self.state, self.last_state_at = state, time.monotonic()
        connected = state.get("connected") is True
        ready = state.get("ready") is True
        if state.get("armed") or state.get("active_command_id"):
            self.flight_may_be_active = True
        elif connected and state.get("armed") is False and state.get("landed_state") == 1:
            self.flight_may_be_active = False
        self.connection.setText("已就绪" if ready else "已连接，等待定位" if connected else "执行器未连接")
        self.banner.setText("实时遥测已连接；悬停与降落无需模型响应。" if ready else
                              self.application.clean_error(state.get("reason") or "等待有效定位与飞控连接"))
        self.metrics["flight"].setText("{} · {} · {}".format(state.get("mode") or "—",
            "已解锁" if state.get("armed") else "已上锁" if connected else "—", LANDED.get(state.get("landed_state"), "未知")))
        self.metrics["position"].setText(vector_text(state.get("position_enu_m")) if ready else "— / — / —")
        vio = state.get("vio") or {}
        external = state.get("external_vision") or {}
        ekf = external.get("ekf") or {}
        if external.get("used_for_flight_control") is True:
            text = ("VIO 主定位 · GPS 融合关闭" if ekf.get("gps_fusion") is False else "VIO 与 GPS 融合定位")
        else:
            text = "VIO 仅监测，未用于飞控" if vio.get("ready") else "VIO 等待初始化 / 数据无效"
        self.metrics["vio"].setText(text)
        navigation = state.get("navigation") or {}
        self.hint.setText("Ctrl+Enter 发送 · 本地 ENU 坐标 · " + (
            "已知静态三维地图 · 高度 1～10 米，可变高" if navigation.get("method") == "static_3d" else
            "二维雷达绕障 · 使用场景指定的固定高度"))
        mission = state.get("mission") or {}
        self.metrics["mission"].setText("{} · {} · 步骤 {}".format(STATUS.get(mission.get("status"), "空闲"),
            ACTIONS.get(mission.get("action"), "—"), mission.get("step", "—")))
        if command:
            text = "指令：{}\n状态：{}\n\n{}".format(command["prompt"], STATUS.get(command["status"], command["status"]),
                                                         command.get("reply") or "Agent 正在处理，请稍候…")
            if command.get("error"):
                text += "\n\n错误：" + command["error"]
            search_events = command.get("search_events") or []
            if search_events:
                progress = search_events[-1]["result"]
                if "search_status" in progress:
                    labels = {"searching": "自主搜索中", "target_found": "已发现目标",
                              "budget_exhausted": "搜索预算已结束", "no_reachable_view": "没有可到达的新观察点",
                              "failed": "搜索已终止"}
                    text += "\n\n{} · 目标：{} · 已观察 {}/8 次 · 已用 {:.0f} 秒".format(
                        labels.get(progress["search_status"], progress["search_status"]), progress.get("query", "—"),
                        len(progress.get("observations", [])), progress.get("elapsed_seconds", 0))
                    if command['status'] == 'running':
                        phase = {'capture_scene_then_report_scene': '正在采图、分析',
                                 'observe_target_search_view': '正在换位、采图与分析',
                                 'execute_target_search_view': '正在执行起飞或换位观察',
                                 'next_target_search_view': '正在选择新的观察位置',
                                 'return_home_then_land': '正在返航、降落',
                                 'land_drone': '正在降落', 'final_report': '正在整理报告'}
                        if progress.get('next_tool') in phase:
                            text += '\n当前阶段：' + phase[progress['next_tool']]
                    if progress.get("reason"):
                        text += "\n下一步依据：" + progress["reason"]
                    if progress.get("goto"):
                        text += "\n正在前往观察点：" + vector_text([progress["goto"][k] for k in ("x", "y", "altitude")])
            perception = command.get("perception")
            if perception:
                text += "\n\n观察报告：\n" + json.dumps(self.application.clean_perception(perception), ensure_ascii=False, indent=2)
            self.show_output(self.application.clean_perception(text))
        gazebo = state.get("gazebo") or {}
        position = gazebo.get("position_m")
        if ready and gazebo.get("available") and vector_text(position) != "— / — / —":
            if not self.trail or self.trail[-1] != tuple(position[:2]):
                self.trail.append(tuple(position[:2]))
                self.trail = self.trail[-2000:]
        else:
            self.trail.clear()
        self.draw_map()
        if self.closing and connected and state.get("armed") is False and state.get("landed_state") == 1 and not state.get("active_command_id"):
            self.destroy()

    def draw_map(self):
        self.map.update()

    def render_cameras(self, fresh):
        for side, label in self.camera_labels.items():
            metadata = (self.state.get("camera") or {}).get(side) or {}
            data = camera_bytes(self.application.camera_paths[side], metadata) if fresh else None
            if data is None:
                label.clear()
                label.setText("相机未连接 / 图像已过期")
                self.images.pop(side, None)
                self.frame_bytes.pop(side, None)
                continue
            if data != self.frame_bytes.get(side):
                image = QPixmap()
                if not image.loadFromData(data):
                    label.clear()
                    label.setText("图像暂不可用")
                    self.images.pop(side, None)
                    self.frame_bytes.pop(side, None)
                    continue
                self.images[side] = image
                self.frame_bytes[side] = data
            label.setPixmap(self.images[side].scaled(label.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                                     Qt.TransformationMode.SmoothTransformation))

    def tick(self):
        if self.stopped.is_set():
            return
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "telemetry":
                    self.render_state(*value)
                    if self.stopped.is_set():
                        return
                elif kind == "submitted":
                    self.submitting = False
                    self.command_id = value["id"]
                    self.state["active_command_id"] = self.command_id
                    self.show_output("已提交：" + value["prompt"])
                elif kind == "control":
                    self.control_busy = False
                    self.banner.setText("控制请求已发送，等待实际遥测确认。")
                elif kind == "error":
                    operation, error = value
                    if operation == "submitted":
                        self.submitting = False
                    if operation == "control":
                        self.control_busy = False
                        self.closing = False
                    self.banner.setText(error)
        except queue.Empty:
            pass
        fresh = time.monotonic()-self.last_state_at < 2
        if not fresh:
            self.connection.setText("遥测已过期 / 正在重连")
            self.metrics["position"].setText("— / — / —")
            self.trail.clear()
            self.draw_map()
        if time.monotonic()-self.last_camera_at >= .35:
            self.render_cameras(fresh)
            self.last_camera_at = time.monotonic()
        self.buttons()

    def request_close(self):
        if self.closing:
            return
        if self.submitting:
            self.banner.setText("正在提交指令，请稍候再关闭；可随时使用立即降落。")
            return
        # Include live application ownership: the last telemetry poll can lag submission.
        with self.application.lock:
            active = bool(self.application.active_id)
        if self.flight_may_be_active or self.state.get("armed") or active:
            if not self.confirm_close():
                return
            self.closing = True
            self.control("/land")
        else:
            self.destroy()

    def confirm_close(self):
        return QMessageBox.question(self.root, "结束控制",
            "将中断当前指令并请求降落，确认落地上锁后关闭窗口。是否继续？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No) == QMessageBox.StandardButton.Yes

    def destroy(self):
        if self.stopped.is_set():
            return
        with self.application.lock:
            if self.application.current_command is not None:
                self.application.current_command["interrupted"] = True
        self.stopped.set()
        self.timer.stop()
        self.root.close()


def main():
    parser = argparse.ArgumentParser(description="PX4 原生桌面控制台（无需浏览器）")
    parser.add_argument("--env-file", default=os.getenv("UAV_ENV_FILE", str(DEFAULT_ENV_FILE)))
    parser.add_argument("--bridge-url")
    parser.add_argument("--ready-file", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    application = build_application(args.env_file, args.bridge_url)
    qt = QApplication([])
    qt.setApplicationName("PX4 桌面控制台")
    root = DesktopRoot()
    window = DesktopWindow(root, application)
    root.show()
    # TERM interrupts model work; the lifecycle manager then owns safe landing.
    signal.signal(signal.SIGTERM, lambda *_: window.destroy())
    signal.signal(signal.SIGINT, lambda *_: window.request_close())
    if args.ready_file:
        args.ready_file.write_text(str(os.getpid()) + "\n")
    print("PX4 桌面控制台已打开；直接连接执行器，不启动网页服务。", flush=True)
    try:
        qt.exec()
    finally:
        window.stopped.set()
    if args.ready_file:
        args.ready_file.with_suffix(".closed").write_text("closed\n")


if __name__ == "__main__":
    main()
