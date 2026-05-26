#!/usr/bin/env python3
"""
Inspire RH56E2 灵巧手上位机控制面板 (双手)
==========================================
同时控制左右手，支持独立/联动操作。

用法:
    python3 hand_control_panel.py
"""

import os
import sys
import time
import argparse
import threading
import numpy as np
from datetime import datetime

BASE = os.path.dirname(os.path.abspath(__file__))
for p in [os.path.join(BASE, "unitree_sdk2_python"),
          os.path.join(BASE, "inspire_hand")]:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QBrush
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QPushButton, QSlider, QFrame,
    QComboBox,
)

from inspire_sdkpy.inspire_dds import inspire_hand_ctrl
from inspire_sdkpy.inspire_hand_defaut import get_inspire_hand_ctrl
from inspire_sdkpy import inspire_sdk
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher,
)

# ─── 常量 ──────────────────────────────────────────────────────────────────────

FINGERS = 6
FINGER_NAMES = ["小指", "无名指", "中指", "食指", "拇指屈", "拇指旋"]
POS_MAX = 1000
POS_DEFAULT = 500
MODE_ANGLE = 0b0001

HANDS = {
    "left":  {"lr": "l", "ip": "192.168.123.210", "device_id": 1, "name": "左手"},
    "right": {"lr": "r", "ip": "192.168.123.211", "device_id": 1, "name": "右手"},
}

PRESETS = {
    "张开":   [1000]*6,
    "握拳":   [0]*6,
    "指向":   [0, 0, 0, 1000, 0, 500],
    "OK":     [0, 0, 0, 0, 300, 300],
    "点赞":   [0, 0, 0, 0, 1000, 500],
    "摇滚":   [1000, 0, 0, 1000, 1000, 500],
    "三指捏": [0, 0, 300, 300, 300, 300],
    "半开":   [500]*6,
}

STATUS_LABELS = {
    0: "松开", 1: "抓取", 2: "到位", 3: "力控",
    5: "过流", 6: "堵转", 7: "故障", 255: "错误",
}

DARK_STYLE = """
QMainWindow, QWidget { background: #1e1e2e; color: #cdd6f4; }
QGroupBox {
    border: 1px solid #45475a; border-radius: 6px;
    margin-top: 10px; padding-top: 14px; font-weight: bold; color: #89b4fa;
}
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }
QPushButton {
    background: #313244; color: #cdd6f4; border: 1px solid #45475a;
    border-radius: 5px; padding: 6px 14px; font-size: 13px;
}
QPushButton:hover { background: #45475a; border-color: #89b4fa; }
QPushButton:pressed { background: #585b70; }
QPushButton[urgent="true"] {
    background: #f38ba8; color: #1e1e2e; font-weight: bold;
}
QPushButton[active="true"] {
    background: #89b4fa; color: #1e1e2e;
}
QSlider::groove:horizontal { height: 5px; background: #45475a; border-radius: 2px; }
QSlider::handle:horizontal {
    width: 14px; height: 14px; margin: -5px 0;
    background: #89b4fa; border-radius: 7px;
}
QSlider::sub-page:horizontal { background: #74c7ec; border-radius: 2px; }
QComboBox {
    background: #313244; color: #cdd6f4; border: 1px solid #45475a;
    border-radius: 4px; padding: 3px 8px;
}
"""


# ─── 驱动 + 控制线程 ──────────────────────────────────────────────────────────

class HandDriver(threading.Thread):
    """Modbus 驱动 + DDS 发送，每只手一个线程。"""

    def __init__(self, lr, ip, device_id):
        super().__init__(daemon=True)
        self.lr = lr
        self.ip = ip
        self.device_id = device_id
        self.running = False
        self.state = {}
        self.targets = [POS_DEFAULT] * FINGERS
        self.mode = MODE_ANGLE

    def run(self):
        states_structure = [
            ('angle_act', 1546, 6, 'short'),
            ('force_act', 1582, 6, 'short'),
            ('status', 1612, 3, 'byte'),
        ]

        try:
            self.handler = inspire_sdk.ModbusDataHandler(
                LR=self.lr, device_id=self.device_id,
                use_serial=False, ip=self.ip,
                states_structure=states_structure, initDDS=False,
            )
            self.connected = True
        except Exception as e:
            self.connected = False
            self.state = {"error": str(e)}
            return

        ctrl_topic = f"rt/inspire_hand/ctrl/{self.lr}"
        self.pub = ChannelPublisher(ctrl_topic, inspire_hand_ctrl)
        self.pub.Init()

        self.running = True
        tick = 0
        self.baseline = [0] * FINGERS
        calib_samples = []
        calib_done = False
        while self.running:
            # 读状态
            try:
                data = self.handler.read()
                s = data.get('states', {})
                forces = list(s.get('FORCE_ACT', [0]*FINGERS))

                # 自动校准: 收集前 20 帧平均值作为零偏
                if not calib_done:
                    if len(forces) == FINGERS and all(f != 0 for f in forces):
                        calib_samples.append(forces)
                    if len(calib_samples) >= 20:
                        self.baseline = [int(sum(c[i] for c in calib_samples) / len(calib_samples))
                                          for i in range(FINGERS)]
                        print(f"[{self.lr}] 自动校准: baseline={self.baseline}")
                        calib_done = True

                calibrated = [forces[i] - self.baseline[i]
                              if i < len(forces) else 0
                              for i in range(FINGERS)]

                self.state = {
                    'angle': list(s.get('ANGLE_ACT', [])),
                    'force': calibrated,
                    'status': list(s.get('STATUS', [])),
                }
            except Exception:
                pass

            # 发指令
            try:
                cmd = get_inspire_hand_ctrl()
                cmd.mode = self.mode
                if self.mode & MODE_ANGLE:
                    cmd.angle_set = [int(v) for v in self.targets]
                if self.mode & 0b0010:
                    cmd.pos_set = [int(v) for v in self.targets]
                self.pub.Write(cmd)
            except Exception:
                pass

            tick += 1
            time.sleep(0.005)

    def stop(self):
        self.running = False


# ─── UI 组件 ───────────────────────────────────────────────────────────────────

class MotorRow(QFrame):
    value_changed = pyqtSignal(int, float)

    def __init__(self, idx, name):
        super().__init__()
        self.idx = idx
        self.setFrameShape(QFrame.StyledPanel)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(3, 1, 3, 1)
        layout.setSpacing(4)

        lbl = QLabel(name)
        lbl.setFixedWidth(36)
        lbl.setStyleSheet("font-weight:bold; font-size:11px; color:#cdd6f4;")
        layout.addWidget(lbl)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, POS_MAX)
        self.slider.setValue(POS_DEFAULT)
        self.slider.valueChanged.connect(self._on_change)
        layout.addWidget(self.slider, 1)

        self.val = QLabel("500")
        self.val.setFixedWidth(32)
        self.val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.val.setStyleSheet("font-family:monospace; font-size:10px; color:#f9e2af;")
        layout.addWidget(self.val)

        self.fb = QLabel("--")
        self.fb.setFixedWidth(32)
        self.fb.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.fb.setStyleSheet("font-family:monospace; font-size:10px; color:#a6e3a1;")
        layout.addWidget(self.fb)

        # 力传感指示器（不显示，由底部传感器面板替代）
        self.force_bar = None

    def _on_change(self, v):
        self.val.setText(str(v))
        self.value_changed.emit(self.idx, v)

    def set_target(self, v):
        self.slider.blockSignals(True)
        self.slider.setValue(int(v))
        self.slider.blockSignals(False)
        self.val.setText(str(int(v)))

    def set_fb(self, v):
        self.fb.setText(str(int(v)))

    def set_force(self, v):
        """力值（已弃用，由底部传感器面板显示）"""
        pass


class HandCanvas(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(160, 200)
        self._q = [POS_DEFAULT] * FINGERS

    def update_pos(self, q):
        self._q = [int(v) for v in q[:FINGERS]]
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w * 0.5, h * 0.55
        s = min(w, h) / 280.0

        p.setPen(QPen(QColor("#45475a"), 1))
        p.setBrush(QBrush(QColor("#313244")))
        from PyQt5.QtGui import QPolygonF
        from PyQt5.QtCore import QPointF
        palm = QPolygonF([QPointF(x, y) for x, y in [
            (cx - 45*s, cy + 38*s), (cx + 45*s, cy + 38*s),
            (cx + 52*s, cy - 14*s), (cx - 52*s, cy - 14*s)]])
        p.drawPolygon(palm)

        anchors = [
            (cx - 38*s, cy - 14*s), (cx - 14*s, cy - 19*s),
            (cx + 8*s,  cy - 21*s), (cx + 26*s, cy - 16*s),
            (cx - 50*s, cy + 5*s),
        ]
        lens = [38, 44, 48, 40, 34]
        base = [-25, -8, 0, 12, -90]
        colors = ["#f38ba8", "#fab387", "#f9e2af", "#a6e3a1", "#89b4fa"]

        for i, (ax, ay) in enumerate(anchors):
            q = self._q[i] / POS_MAX
            angle = base[i] + (1.0 - q) * 80
            rad = np.radians(angle)
            ex = ax + lens[i]*s * np.sin(rad)
            ey = ay - lens[i]*s * np.cos(rad)
            c = QColor(colors[i])
            p.setPen(QPen(c, 3))
            p.drawLine(int(ax), int(ay), int(ex), int(ey))
            p.setBrush(QBrush(c))
            p.drawEllipse(int(ax)-3, int(ay)-3, 6, 6)

        p.end()


# ─── 单手控制面板 ──────────────────────────────────────────────────────────────

class HandSidePanel(QGroupBox):
    preset_requested = pyqtSignal(str, list)  # preset_name, values

    def __init__(self, side):
        super().__init__()
        self.side = side
        info = HANDS[side]
        self.setTitle(info["name"])
        self.driver = None
        self._logs = []

        layout = QVBoxLayout(self)
        layout.setSpacing(3)
        layout.setContentsMargins(6, 14, 6, 6)

        # 状态栏
        status_row = QHBoxLayout()
        self.start_btn = QPushButton("启动")
        self.start_btn.setFixedHeight(28)
        self.start_btn.clicked.connect(self._start)
        status_row.addWidget(self.start_btn)

        self.stop_btn = QPushButton("停止")
        self.stop_btn.setFixedHeight(28)
        self.stop_btn.clicked.connect(self._stop)
        self.stop_btn.setEnabled(False)
        status_row.addWidget(self.stop_btn)

        self.status_lbl = QLabel("未启动")
        self.status_lbl.setStyleSheet("color:#f38ba8; font-size:11px;")
        status_row.addWidget(self.status_lbl)
        status_row.addStretch()
        layout.addLayout(status_row)

        # 手势按钮
        preset_row = QHBoxLayout()
        for name, vals in PRESETS.items():
            btn = QPushButton(name)
            btn.setFixedHeight(26)
            btn.setStyleSheet("font-size:11px; padding:3px 8px;")
            btn.clicked.connect(lambda _, n=name, v=vals: self._apply_preset(n, v))
            preset_row.addWidget(btn)
        layout.addLayout(preset_row)

        # 滑块 + 可视化
        mid = QHBoxLayout()

        slider_col = QVBoxLayout()
        slider_col.setSpacing(1)
        self.rows = []
        for i, n in enumerate(FINGER_NAMES):
            row = MotorRow(i, n)
            row.value_changed.connect(self._on_slider)
            self.rows.append(row)
            slider_col.addWidget(row)
        mid.addLayout(slider_col, 3)

        self.canvas = HandCanvas()
        mid.addWidget(self.canvas, 2)

        layout.addLayout(mid, 1)

        # 紧急停止
        estop = QPushButton("紧急停止")
        estop.setProperty("urgent", True)
        estop.setFixedHeight(30)
        estop.clicked.connect(self._estop)
        layout.addWidget(estop)

        # 状态反馈
        fb_group = QGroupBox("传感器反馈")
        fb_layout = QVBoxLayout(fb_group)
        fb_layout.setSpacing(2)
        fb_layout.setContentsMargins(6, 12, 6, 6)

        # 每指力值显示
        self.force_bars = []
        self.force_values = []
        for i, n in enumerate(FINGER_NAMES):
            row = QHBoxLayout()
            row.setSpacing(4)
            lbl = QLabel(f"{n}")
            lbl.setFixedWidth(36)
            lbl.setStyleSheet("font-size:10px; color:#cdd6f4;")
            row.addWidget(lbl)

            bar = QLabel("")
            bar.setFixedHeight(14)
            bar.setMinimumWidth(100)
            bar.setStyleSheet("background:#313244; border-radius:3px;")
            row.addWidget(bar, 1)
            self.force_bars.append(bar)

            val = QLabel("0")
            val.setFixedWidth(35)
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setStyleSheet("font-family:monospace; font-size:10px; color:#f9e2af;")
            row.addWidget(val)
            self.force_values.append(val)

            fb_layout.addLayout(row)

        # 状态行
        status_row = QHBoxLayout()
        self.st_lbl = QLabel("--")
        self.st_lbl.setStyleSheet("font-size:10px; color:#a6e3a1; font-weight:bold;")
        status_row.addWidget(self.st_lbl)
        status_row.addStretch()
        fb_layout.addLayout(status_row)

        layout.addWidget(fb_group)

    def _start(self):
        # DDS 初始化 (全局只需一次)
        try:
            ChannelFactoryInitialize(0)
        except Exception:
            pass

        info = HANDS[self.side]
        self.driver = HandDriver(info["lr"], info["ip"], info["device_id"])
        self.driver.start()

        # 等待连接
        for _ in range(100):
            if hasattr(self.driver, 'connected'):
                break
            time.sleep(0.05)

        if self.driver.connected:
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.status_lbl.setText(f"运行中 ({info['ip']})")
            self.status_lbl.setStyleSheet("color:#a6e3a1; font-size:11px;")
        else:
            err = self.driver.state.get("error", "连接失败")
            self.status_lbl.setText(err)
            self.driver = None

    def _stop(self):
        if self.driver:
            self.driver.stop()
            self.driver = None
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_lbl.setText("已停止")
        self.status_lbl.setStyleSheet("color:#f38ba8; font-size:11px;")

    def _on_slider(self, idx, val):
        if self.driver:
            self.driver.targets[idx] = int(val)

    def _apply_preset(self, name, vals):
        for i, v in enumerate(vals):
            self.rows[i].set_target(v)
        if self.driver:
            self.driver.targets = list(vals)

    def _estop(self):
        if self.driver and self.driver.state:
            angles = self.driver.state.get('angle', [POS_DEFAULT]*FINGERS)
        else:
            angles = [POS_DEFAULT] * FINGERS
        if self.driver:
            self.driver.targets = [int(v) for v in angles]
        for i, v in enumerate(angles):
            self.rows[i].set_target(v)

    def refresh(self):
        if not self.driver or not self.driver.state:
            return
        s = self.driver.state
        angles = s.get('angle', [])
        for i, row in enumerate(self.rows):
            if i < len(angles):
                row.set_fb(angles[i])
        self.canvas.update_pos(angles if angles else [POS_DEFAULT]*FINGERS)

        # 每指力值 + 力条（已自动校准零偏）
        forces = s.get('force', [])
        for i in range(FINGERS):
            fv = int(forces[i]) if i < len(forces) else 0
            av = max(0, fv)  # 校准后负值视为 0
            self.force_values[i].setText(str(av))

            # 力条: 宽度按比例, 颜色按级别
            max_w = 140
            bar_w = min(max_w, int(av / 100.0 * max_w))  # 100 = 满条
            if av == 0:
                color = "#313244"
            elif av < 30:
                color = "#a6e3a1"  # 绿
            elif av < 80:
                color = "#f9e2af"  # 黄
            else:
                color = "#f38ba8"  # 红
            self.force_bars[i].setFixedWidth(max(bar_w, 2))
            self.force_bars[i].setStyleSheet(
                f"background:{color}; border-radius:3px;")

        statuses = s.get('status', [])
        if statuses:
            labels = [STATUS_LABELS.get(sv, str(sv)) for sv in statuses]
            self.st_lbl.setText(" | ".join(labels))

    def set_all_targets(self, vals):
        for i, v in enumerate(vals):
            self.rows[i].set_target(v)
        if self.driver:
            self.driver.targets = list(vals)


# ─── 主窗口 ────────────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Inspire RH56E2 灵巧手控制 — 双手")
        self.setMinimumSize(1200, 580)
        self.resize(1300, 650)
        self._build_ui()

        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(50)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(6)
        root.setContentsMargins(8, 8, 8, 8)

        # 顶栏: 联动控制
        top = QHBoxLayout()
        top.addWidget(QLabel("联动:"))

        self.link_check = QPushButton("双手同步: 关")
        self.link_check.setCheckable(True)
        self.link_check.setChecked(False)
        self.link_check.toggled.connect(self._on_link_toggled)
        top.addWidget(self.link_check)

        top.addWidget(QLabel("  联动手势:"))
        for name, vals in PRESETS.items():
            btn = QPushButton(name)
            btn.setFixedHeight(28)
            btn.setStyleSheet("font-size:11px; padding:3px 8px;")
            btn.clicked.connect(lambda _, n=name, v=vals: self._link_preset(n, v))
            top.addWidget(btn)
        top.addStretch()

        self.link_status = QLabel("")
        self.link_status.setStyleSheet("font-size:11px; color:#6c7086;")
        top.addWidget(self.link_status)
        root.addLayout(top)

        sep = QFrame(); sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color:#45475a;"); root.addWidget(sep)

        # 双手面板
        hands_row = QHBoxLayout()
        self.left_panel = HandSidePanel("left")
        self.right_panel = HandSidePanel("right")
        hands_row.addWidget(self.left_panel)
        hands_row.addWidget(self.right_panel)
        root.addLayout(hands_row, 1)

        # 底栏日志
        self.log_lbl = QLabel("")
        self.log_lbl.setWordWrap(True)
        self.log_lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.log_lbl.setStyleSheet(
            "background:#11111b; color:#a6adc8; border:1px solid #313244; "
            "border-radius:3px; padding:4px; font-family:monospace; font-size:10px;")
        self.log_lbl.setFixedHeight(30)
        root.addWidget(self.log_lbl)

    def _on_link_toggled(self, checked):
        self.link_check.setText(f"双手同步: {'开' if checked else '关'}")
        self.link_check.setStyleSheet(
            "background:#89b4fa; color:#1e1e2e;" if checked else "")

    def _link_preset(self, name, vals):
        self.left_panel.set_all_targets(vals)
        self.right_panel.set_all_targets(vals)
        self._log(f"联动: {name}")

    def _refresh(self):
        self.left_panel.refresh()
        self.right_panel.refresh()

        # 联动: 左手滑块变化 → 同步到右手
        if self.link_check.isChecked() and self.left_panel.driver and self.right_panel.driver:
            self.right_panel.driver.targets = list(self.left_panel.driver.targets)
            for i, v in enumerate(self.left_panel.driver.targets):
                self.right_panel.rows[i].set_target(v)

    def _log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_lbl.setText(f"[{ts}] {msg}")

    def closeEvent(self, event):
        self.left_panel._stop()
        self.right_panel._stop()
        event.accept()


# ─── 入口 ──────────────────────────────────────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLE)
    app.setFont(QFont("sans-serif", 10))
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
