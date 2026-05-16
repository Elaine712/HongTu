#!/usr/bin/env python3
"""
G1 导航控制台 — 一体化导航管理、地图加载、遥操作、航点巡航、动作语音
=========================================================================
用法：
    source ~/Desktop/HongTu/G1Nav2D/devel/setup.bash
    python3 main.py

依赖：PyQt5, rospy, unitree_sdk2py
"""

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

from PyQt5.QtCore import (
    QByteArray, QBuffer, QIODevice, QPointF, QRectF, QSize, Qt, QTimer, QThread, pyqtSignal
)
from PyQt5.QtGui import (
    QBrush, QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF, QTransform
)
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QFileDialog, QFormLayout,
    QGraphicsEllipseItem, QGraphicsItem, QGraphicsLineItem, QGraphicsPixmapItem,
    QGraphicsPolygonItem, QGraphicsScene, QGraphicsView, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem, QMainWindow,
    QMessageBox, QPlainTextEdit, QProgressBar, QPushButton, QScrollArea, QSlider,
    QSpinBox, QSplitter, QStatusBar, QTabWidget, QVBoxLayout, QWidget
)

# ============================================================
# ROS 导入
# ============================================================
try:
    import rospy
    from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Pose, Point, Quaternion
    from nav_msgs.msg import Odometry, OccupancyGrid, Path
    from actionlib_msgs.msg import GoalStatusArray
    import tf.transformations as tf_tr
    import tf2_ros
    ROS_OK = True
except Exception:
    ROS_OK = False

# ============================================================
# G1 SDK 导入
# ============================================================
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
    from unitree_sdk2py.g1.arm.g1_arm_action_client import action_map as ARM_ACTIONS
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient as G1AudioClient
    G1_OK = True
except Exception:
    G1_OK = False
    ARM_ACTIONS = {}

# ============================================================
# 配置
# ============================================================
CONFIG_FILE = os.path.expanduser("~/.g1_nav_panel.json")
DEFAULT_CONFIG = {
    "net_if": "eno1",
    "map_yaml": os.path.expanduser("~/Desktop/G1map.yaml"),
    "pcd_path": os.path.expanduser("~/Desktop/HongTu/G1Nav2D/src/fastlio2/PCD/map.pcd"),
    "auto_start_ros": True,
    "window_geometry": None,
}

NAV_STATUS_MAP = {
    0: "排队中", 1: "导航中", 2: "被抢占",
    3: "已到达 ✓", 4: "失败 ✗", 5: "被拒绝",
    6: "抢占中", 7: "召回中", 8: "已召回", 9: "丢失",
}


# ============================================================
# 配置管理
# ============================================================
def load_config():
    cfg = dict(DEFAULT_CONFIG)
    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE) as f:
                cfg.update(json.load(f))
    except Exception:
        pass
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
    except Exception:
        pass


# ============================================================
# ROS 工作线程 — 不阻塞 GUI
# ============================================================
class RosWorker(QThread):
    pose_updated = pyqtSignal(float, float, float)  # x, y, yaw
    map_updated = pyqtSignal(object)  # OccupancyGrid
    nav_status_updated = pyqtSignal(str)
    goal_done = pyqtSignal(bool)  # success/fail
    log_msg = pyqtSignal(str)

    # 从主线程发往 ROS 线程的命令
    request_goal = pyqtSignal(float, float, float)    # x, y, yaw
    request_initpose = pyqtSignal(float, float, float)
    request_cmd_vel = pyqtSignal(float, float, float) # vx, vy, wz
    # 从 ROS 线程发往主线程的导航速度
    nav_cmd_vel = pyqtSignal(float, float, float)  # vx, vy, wz

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False
        self._pub_cmd = None
        self._pub_goal = None
        self._pub_initpose = None
        self._pose = (0.0, 0.0, 0.0)
        self._shutdown = False

    def stop(self):
        self._shutdown = True
        try:
            rospy.signal_shutdown("关闭")
        except Exception:
            pass

    def run(self):
        if not ROS_OK:
            self.log_msg.emit("[ROS] 库不可用")
            return
        # 持续等待 rocore 可用（直到关闭）
        first = True
        while not self._shutdown:
            try:
                import socket as _sock
                s = _sock.socket(_sock.AF_INET, _sock.SOCK_STREAM)
                s.settimeout(0.5)
                s.connect(("localhost", 11311))
                s.close()
                break
            except Exception:
                if first:
                    self.log_msg.emit("[ROS] 等待 roscore（点击启动导航后自动连接）…")
                    first = False
                self.msleep(1000)
        if self._shutdown:
            return

        try:
            rospy.init_node("g1_nav_panel_ros", anonymous=True, disable_signals=True)
            self._pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
            self._pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
            self._pub_initpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

            # 将信号连接到实际的 ROS 发布
            self.request_cmd_vel.connect(self._pub_cmd_vel)
            self.request_goal.connect(self._pub_goal_slot)
            self.request_initpose.connect(self._pub_initpose_slot)

            rospy.Subscriber("/slam_odom", Odometry, self._odom_cb)
            rospy.Subscriber("/map", OccupancyGrid, self._map_cb)
            rospy.Subscriber("/move_base/status", GoalStatusArray, self._status_cb)
            rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_bridge_cb)  # 导航→G1 桥接
            self.log_msg.emit("[ROS] 节点已启动")
            self._running = True
            rospy.spin()
        except Exception as e:
            self.log_msg.emit(f"[ROS] 错误: {e}")

    # ---- ROS 发布槽函数（在 ROS 线程中执行） ----
    def _pub_cmd_vel(self, vx, vy, wz):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
        self._pub_cmd.publish(t)

    def _pub_goal_slot(self, x, y, yaw):
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = rospy.Time.now()
        g.pose.position.x, g.pose.position.y = x, y
        q = tf_tr.quaternion_from_euler(0, 0, yaw)
        g.pose.orientation.x, g.pose.orientation.y = q[0], q[1]
        g.pose.orientation.z, g.pose.orientation.w = q[2], q[3]
        self._pub_goal.publish(g)
        self.log_msg.emit(f"[ROS] 导航目标: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)")

    def _pub_initpose_slot(self, x, y, yaw):
        p = PoseWithCovarianceStamped()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()
        p.pose.pose.position.x, p.pose.pose.position.y = x, y
        q = tf_tr.quaternion_from_euler(0, 0, yaw)
        p.pose.pose.orientation.x, p.pose.pose.orientation.y = q[0], q[1]
        p.pose.pose.orientation.z, p.pose.pose.orientation.w = q[2], q[3]
        self._pub_initpose.publish(p)
        self.log_msg.emit(f"[ROS] 初始位姿: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)")

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = tf_tr.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.pose_updated.emit(*self._pose)

    def _cmd_vel_bridge_cb(self, msg):
        """把 move_base 输出的 cmd_vel 转发给主线程（再转发给 G1）"""
        self.nav_cmd_vel.emit(msg.linear.x, msg.linear.y, msg.angular.z)

    def _map_cb(self, msg):
        self.log_msg.emit(f"[ROS] 收到地图: {msg.info.width}x{msg.info.height}")
        self.map_updated.emit(msg)

    def _status_cb(self, msg):
        if msg.status_list:
            s = msg.status_list[-1].status
            txt = NAV_STATUS_MAP.get(s, f"未知")
            self.nav_status_updated.emit(txt)
            if s == 3:
                self.goal_done.emit(True)
            elif s in (2, 4, 5, 8):
                self.goal_done.emit(False)
        else:
            self.nav_status_updated.emit("空闲")

    def send_cmd_vel(self, vx, vy, wz):
        """发送速度指令（线程安全，通过信号）"""
        self.request_cmd_vel.emit(vx, vy, wz)

    def send_goal(self, x, y, yaw):
        """发送导航目标（线程安全）"""
        self.request_goal.emit(x, y, yaw)
        self.goal_status = 0

    def send_init_pose(self, x, y, yaw):
        """设置初始位姿（线程安全）"""
        self.request_initpose.emit(x, y, yaw)


# ============================================================
# 地图显示视图
# ============================================================
class MapView(QGraphicsView):
    """显示 2D 栅格地图，叠加机器人位置、航点"""

    clicked = pyqtSignal(float, float)  # 鼠标点击的地图坐标

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setBackgroundBrush(QBrush(QColor("#2b2b2b")))

        self._map_item = None
        self._robot_item = None
        self._wp_items = []
        self._res = 0.05
        self._origin = (0.0, 0.0)
        self._width = 0
        self._height = 0
        self._scale = 1.0

    def set_map(self, occ_grid):
        """从 OccupancyGrid 更新地图显示"""
        if self._map_item:
            self._scene.removeItem(self._map_item)
        if self._robot_item:
            self._scene.removeItem(self._robot_item)
            self._robot_item = None

        info = occ_grid.info
        self._res = info.resolution
        self._origin = (info.origin.position.x, info.origin.position.y)
        self._width = info.width
        self._height = info.height

        w, h = info.width, info.height
        img = QImage(w, h, QImage.Format_Indexed8)
        img.setColorCount(256)
        for i in range(256):
            img.setColor(i, QColor(i, i, i).rgb())
        # -1(未知)=128灰色, 0(空闲)=255白色, 100(障碍)=0黑色
        for y in range(h):
            for x in range(w):
                idx = y * w + x
                val = 128  # 默认未知
                if idx < len(occ_grid.data):
                    v = occ_grid.data[idx]
                    if v == -1:
                        val = 128
                    elif v == 0:
                        val = 255
                    else:
                        val = max(0, 255 - int(v * 2.55))
                img.setPixel(x, h - 1 - y, val)

        pix = QPixmap.fromImage(img)
        self._map_item = QGraphicsPixmapItem(pix)
        # 翻转 Y 轴
        transform = QTransform()
        transform.translate(0, h)
        transform.scale(1, -1)
        self._map_item.setTransform(transform)
        self._scene.addItem(self._map_item)

        # 自动适应窗口
        self.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def update_robot(self, x, y, yaw):
        """更新机器人位置"""
        if self._robot_item:
            self._scene.removeItem(self._robot_item)
            self._robot_item = None

        # 如果地图还没加载，不显示
        if self._res <= 0:
            return

        # 地图坐标 → 像素坐标
        px = (x - self._origin[0]) / self._res
        py = (y - self._origin[1]) / self._res

        # 绘制机器人（圆 + 箭头）
        items = []
        ell = self._scene.addEllipse(px - 6, py - 6, 12, 12,
                                      QPen(Qt.white, 2), QBrush(QColor("#00aaff")))
        items.append(ell)
        # 箭头
        arrow_len = 14
        ax = px + arrow_len * math.cos(yaw)
        ay = py + arrow_len * math.sin(yaw)
        line = self._scene.addLine(px, py, ax, ay, QPen(Qt.white, 3))
        items.append(line)
        # 位置标签
        label = self._scene.addText(f"({x:.1f},{y:.1f})", QFont("Arial", 8))
        label.setPos(px + 10, py - 10)
        label.setDefaultTextColor(QColor("#00aaff"))
        items.append(label)
        self._robot_item = self._scene.createItemGroup(items)

    def update_waypoints(self, wps):
        """更新航点标记"""
        for item in self._wp_items:
            self._scene.removeItem(item)
        self._wp_items.clear()
        for i, (name, x, y, *_) in enumerate(wps):
            px = (x - self._origin[0]) / self._res
            py = (y - self._origin[1]) / self._res
            dot = self._scene.addEllipse(px - 5, py - 5, 10, 10,
                                           QPen(Qt.white, 1.5), QBrush(QColor("#ff6600")))
            txt = self._scene.addText(str(i + 1), QFont("Arial", 9, QFont.Bold))
            txt.setPos(px + 6, py - 6)
            txt.setDefaultTextColor(QColor("#ff6600"))
            self._wp_items.extend([dot, txt])

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            # 视图坐标 → 场景坐标 → 地图坐标
            sp = self.mapToScene(e.pos())
            mx = sp.x() * self._res + self._origin[0]
            my = sp.y() * self._res + self._origin[1]
            self.clicked.emit(mx, my)
        super().mousePressEvent(e)

    def wheelEvent(self, e):
        factor = 1.15 if e.angleDelta().y() > 0 else 0.85
        self.scale(factor, factor)


# ============================================================
# 航点编辑对话框
# ============================================================
class WaypointDialog(QDialog):
    def __init__(self, parent=None, name="", x=0, y=0, yaw=0, action="", speech=""):
        super().__init__(parent)
        self.setWindowTitle("编辑航点")
        self.resize(320, 220)
        layout = QFormLayout(self)
        self._name = QLineEdit(name)
        self._x = QLineEdit(f"{x:.2f}")
        self._y = QLineEdit(f"{y:.2f}")
        self._yaw = QLineEdit(f"{math.degrees(yaw):.1f}")
        self._action = QComboBox()
        self._action.addItem("无")
        for a in sorted(ARM_ACTIONS.keys()):
            self._action.addItem(a)
        if action:
            idx = self._action.findText(action)
            if idx >= 0:
                self._action.setCurrentIndex(idx)
        self._speech = QLineEdit(speech)

        layout.addRow("名称:", self._name)
        layout.addRow("X (m):", self._x)
        layout.addRow("Y (m):", self._y)
        layout.addRow("朝向 (°):", self._yaw)
        layout.addRow("动作:", self._action)
        layout.addRow("语音:", self._speech)

        btn = QHBoxLayout()
        ok = QPushButton("确定")
        ok.clicked.connect(self._ok)
        cancel = QPushButton("取消")
        cancel.clicked.connect(self.reject)
        btn.addWidget(ok)
        btn.addWidget(cancel)
        layout.addRow(btn)

    def _ok(self):
        try:
            self._name_text = self._name.text().strip() or "航点"
            self._x_val = float(self._x.text())
            self._y_val = float(self._y.text())
            self._yaw_val = math.radians(float(self._yaw.text()))
            self._act_val = self._action.currentText() if self._action.currentText() != "无" else ""
            self._sp_val = self._speech.text().strip()
            self.accept()
        except ValueError:
            QMessageBox.warning(self, "输入错误", "请检查数值格式")

    def result(self):
        return (self._name_text, self._x_val, self._y_val, self._yaw_val,
                self._act_val, self._sp_val)


# ============================================================
# 主窗口
# ============================================================
class MainWindow(QMainWindow):
    log_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.cfg = load_config()
        self._ros_worker = None
        self._nav_proc = None  # roslaunch 子进程
        self._g1_loco = None
        self._g1_arm = None
        self._g1_audio = None
        self._g1_ready = False
        self._teleop_active = False
        self._waypoints = []  # [(name, x, y, yaw, action, speech)]
        self._tour_running = False
        self._last_pose = (0.0, 0.0, 0.0)
        self._map_data = None

        self._init_ui()
        self._load_settings()
        self._start_ros()
        self.log_message.connect(self._append_log)

    # ================================================================
    # UI 构建
    # ================================================================
    def _init_ui(self):
        self.setWindowTitle("G1 导航控制台")
        self.resize(1300, 850)
        self.setMinimumSize(900, 600)

        # 暗色主题
        self.setStyleSheet("""
            QMainWindow, QDialog, QWidget { background: #1e1e1e; color: #ddd; }
            QTabWidget::pane { border: 1px solid #444; background: #252526; }
            QTabBar::tab { background: #2d2d2d; color: #aaa; padding: 8px 18px; margin-right: 2px; }
            QTabBar::tab:selected { background: #007acc; color: #fff; }
            QGroupBox { border: 1px solid #444; border-radius: 4px; margin-top: 12px; padding-top: 12px; color: #ccc; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; }
            QPushButton { background: #007acc; color: #fff; border: none; padding: 6px 14px; border-radius: 3px; }
            QPushButton:hover { background: #0098ff; }
            QPushButton:pressed { background: #005a99; }
            QPushButton.danger { background: #c0392b; }
            QPushButton.danger:hover { background: #e74c3c; }
            QPushButton.success { background: #27ae60; }
            QPushButton.success:hover { background: #2ecc71; }
            QPushButton.warning { background: #f39c12; color: #222; }
            QLineEdit, QComboBox, QSpinBox { background: #3c3c3c; color: #ddd; border: 1px solid #555; padding: 4px; border-radius: 2px; }
            QLabel { color: #ccc; }
            QPlainTextEdit, QListWidget { background: #1e1e1e; color: #ccc; border: 1px solid #444; }
            QProgressBar { background: #333; border: none; border-radius: 3px; text-align: center; }
            QProgressBar::chunk { background: #007acc; border-radius: 3px; }
            QSplitter::handle { background: #444; width: 2px; }
            QCheckBox { color: #ccc; }
            QStatusBar { background: #007acc; color: #fff; }
        """)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)

        # 工具栏
        toolbar = QHBoxLayout()
        self._btn_nav_start = QPushButton("▶ 启动导航")
        self._btn_nav_start.setProperty("class", "success")
        self._btn_nav_start.setMinimumHeight(36)
        self._btn_nav_start.clicked.connect(self._on_nav_start)
        self._btn_nav_stop = QPushButton("■ 停止导航")
        self._btn_nav_stop.setProperty("class", "danger")
        self._btn_nav_stop.setMinimumHeight(36)
        self._btn_nav_stop.clicked.connect(self._on_nav_stop)
        self._nav_status_label = QLabel("导航: 未启动")
        self._nav_status_label.setStyleSheet("font-size: 13px; font-weight: bold; padding: 4px 12px;")

        toolbar.addWidget(self._btn_nav_start)
        toolbar.addWidget(self._btn_nav_stop)
        toolbar.addWidget(self._nav_status_label)
        toolbar.addStretch()
        main_layout.addLayout(toolbar)

        # 工作流程指引
        self._step_label = QLabel(
            "① 启动导航  →  ② 设置重定位（2D Pose Estimate）  →  ③ 点击地图或航点发送导航目标")
        self._step_label.setStyleSheet(
            "background: #2d2d2d; color: #f39c12; padding: 6px 12px; border-radius: 3px; font-size: 12px;")
        main_layout.addWidget(self._step_label)

        # 标签页
        tabs = QTabWidget()
        tabs.addTab(self._build_nav_tab(), "📍 导航")
        tabs.addTab(self._build_teleop_tab(), "🎮 遥控")
        tabs.addTab(self._build_waypoint_tab(), "📍 航点")
        tabs.addTab(self._build_action_tab(), "🤖 动作")
        tabs.addTab(self._build_settings_tab(), "⚙ 设置")
        main_layout.addWidget(tabs, 1)

        # 状态栏
        self._status_bar = QStatusBar()
        self._status_ros = QLabel("ROS: 初始中…")
        self._status_g1 = QLabel("G1: 未连接")
        self._status_pose = QLabel("位姿: ---")
        self._status_bar.addWidget(self._status_ros)
        self._status_bar.addWidget(self._status_g1)
        self._status_bar.addWidget(self._status_pose)
        self.setStatusBar(self._status_bar)

    # ---- 导航标签页 ----
    def _build_nav_tab(self):
        tab = QWidget()
        layout = QHBoxLayout(tab)

        # 左侧：地图
        self._map_view = MapView()
        layout.addWidget(self._map_view, 3)

        # 右侧：可滚动的控制面板
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(380)
        scroll.setStyleSheet("QScrollArea { border: none; }")

        right_widget = QWidget()
        right = QVBoxLayout(right_widget)
        right.setSpacing(6)
        right.setContentsMargins(4, 4, 4, 4)

        # 字体
        title_font = QFont()
        title_font.setPointSize(9)

        # 地图选择
        grp = QGroupBox("地图配置")
        grp.setFont(title_font)
        fm = QFormLayout(grp)
        fm.setLabelAlignment(Qt.AlignRight)
        self._edit_map = QLineEdit(self.cfg.get("map_yaml", ""))
        self._edit_map.setMinimumHeight(28)
        btn_map = QPushButton("浏览…")
        btn_map.setFixedWidth(60)
        btn_map.clicked.connect(lambda: self._browse_file(self._edit_map, "YAML 地图文件 (*.yaml *.yml)"))
        row = QHBoxLayout()
        row.addWidget(self._edit_map, 1)
        row.addWidget(btn_map)
        fm.addRow("2D 地图:", row)

        self._edit_pcd = QLineEdit(self.cfg.get("pcd_path", ""))
        btn_pcd = QPushButton("浏览…")
        btn_pcd.clicked.connect(lambda: self._browse_file(self._edit_pcd, "PCD 点云文件 (*.pcd)"))
        row = QHBoxLayout()
        row.addWidget(self._edit_pcd, 1)
        row.addWidget(btn_pcd)
        fm.addRow("3D 点云:", row)
        right.addWidget(grp)

        # 重定位
        grp = QGroupBox("重定位")
        grp.setFont(title_font)
        rl = QVBoxLayout(grp)
        self._btn_reloc = QPushButton("📌 2D Pose Estimate (点击地图设置)")
        self._btn_reloc.clicked.connect(self._on_reloc_mode)
        rl.addWidget(self._btn_reloc)

        # 使用当前朝向重定位
        rl2 = QHBoxLayout()
        rl2.addWidget(QLabel("x:"))
        self._reloc_x = QLineEdit("0.0")
        self._reloc_x.setFixedWidth(70)
        rl2.addWidget(self._reloc_x)
        rl2.addWidget(QLabel("y:"))
        self._reloc_y = QLineEdit("0.0")
        self._reloc_y.setFixedWidth(70)
        rl2.addWidget(self._reloc_y)
        rl2.addWidget(QLabel("朝向:自动"))
        btn_reloc_go = QPushButton("设置位姿")
        btn_reloc_go.clicked.connect(self._on_reloc_set)
        rl2.addWidget(btn_reloc_go)
        rl.addLayout(rl2)
        right.addWidget(grp)

        # 导航状态
        grp = QGroupBox("导航状态")
        grp.setFont(title_font)
        st = QVBoxLayout(grp)
        self._nav_state_label = QLabel("空闲")
        self._nav_state_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #aaa; padding: 4px;")
        st.addWidget(self._nav_state_label)
        right.addWidget(grp)

        # G1 连接
        grp = QGroupBox("G1 机器人")
        grp.setFont(title_font)
        gg = QVBoxLayout(grp)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("网卡:"))
        self._nav_net_if = QLineEdit(self.cfg.get("net_if", "eno1"))
        self._nav_net_if.setFixedWidth(100)
        row1.addWidget(self._nav_net_if)
        self._btn_nav_g1 = QPushButton("连接 G1")
        self._btn_nav_g1.clicked.connect(self._on_g1_toggle)
        row1.addWidget(self._btn_nav_g1)
        self._nav_g1_label = QLabel("未连接")
        self._nav_g1_label.setStyleSheet("color: #888;")
        row1.addWidget(self._nav_g1_label)
        row1.addStretch()
        gg.addLayout(row1)

        # 快速动作
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("手臂:"))
        for atxt in ["挥手", "鼓掌", "比心", "握手", "拒绝"]:
            an = {"挥手": "face wave", "鼓掌": "clap", "比心": "heart",
                  "握手": "shake hand", "拒绝": "reject"}[atxt]
            btn = QPushButton(atxt)
            btn.setFixedWidth(56)
            btn.setMinimumHeight(28)
            btn.clicked.connect(lambda checked, n=an: self._g1_arm_action(n))
            row2.addWidget(btn)
        row2.addStretch()
        gg.addLayout(row2)

        # 快速语音
        row3 = QHBoxLayout()
        row3.addWidget(QLabel("语音:"))
        self._nav_tts = QLineEdit()
        self._nav_tts.setPlaceholderText("输入播报文字…")
        self._nav_tts.setMinimumHeight(28)
        self._nav_tts.returnPressed.connect(self._on_nav_tts)
        row3.addWidget(self._nav_tts, 1)
        btn_say = QPushButton("播报")
        btn_say.setMinimumHeight(28)
        btn_say.clicked.connect(self._on_nav_tts)
        row3.addWidget(btn_say)
        gg.addLayout(row3)

        right.addWidget(grp)

        # 日志
        grp = QGroupBox("运行日志")
        grp.setFont(title_font)
        lg = QVBoxLayout(grp)
        self._log_view = QPlainTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.setMaximumBlockCount(200)
        lg.addWidget(self._log_view)
        right.addWidget(grp, 1)

        right.addStretch()
        scroll.setWidget(right_widget)
        layout.addWidget(scroll, 1)
        return tab

    # ---- 遥控标签页 ----
    def _build_teleop_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # 速度控制
        grp = QGroupBox("速度控制")
        gl = QVBoxLayout(grp)
        row = QHBoxLayout()
        row.addWidget(QLabel("线速度:"))
        self._slider_lin = QSlider(Qt.Horizontal)
        self._slider_lin.setRange(1, 100)
        self._slider_lin.setValue(30)
        self._slider_lin.setTickInterval(10)
        self._slider_lin.setTickPosition(QSlider.TicksBelow)
        self._slider_lin.valueChanged.connect(lambda v: self._label_lin.setText(f"{v/100:.2f} m/s"))
        self._label_lin = QLabel("0.30 m/s")
        row.addWidget(self._slider_lin, 1)
        row.addWidget(self._label_lin)
        gl.addLayout(row)

        row = QHBoxLayout()
        row.addWidget(QLabel("角速度:"))
        self._slider_ang = QSlider(Qt.Horizontal)
        self._slider_ang.setRange(1, 100)
        self._slider_ang.setValue(30)
        self._slider_ang.setTickInterval(10)
        self._slider_ang.setTickPosition(QSlider.TicksBelow)
        self._slider_ang.valueChanged.connect(lambda v: self._label_ang.setText(f"{v/100:.2f} rad/s"))
        self._label_ang = QLabel("0.30 rad/s")
        row.addWidget(self._slider_ang, 1)
        row.addWidget(self._label_ang)
        gl.addLayout(row)
        layout.addWidget(grp)

        # 方向控制
        grp = QGroupBox("方向控制（点击按钮或按键盘 WASD / 箭头键）")
        dl = QVBoxLayout(grp)
        grid = QGridLayout()
        grid.setSpacing(4)

        def make_btn(text, vx=0, vy=0, wz=0, big=False):
            btn = QPushButton(text)
            if big:
                btn.setMinimumSize(80, 60)
            else:
                btn.setMinimumSize(64, 48)
            btn.setStyleSheet("font-size: 18px; font-weight: bold;")
            btn.pressEvent = lambda e=None: self._teleop_start(vx, vy, wz)
            btn.releaseEvent = lambda e=None: self._teleop_stop()
            btn.mousePressEvent = btn.pressEvent
            btn.mouseReleaseEvent = btn.releaseEvent
            return btn

        btn_fwd = make_btn("▲\nW", vx=1)
        btn_bwd = make_btn("▼\nS", vx=-1)
        btn_left = make_btn("◀\nA", wz=1)
        btn_right = make_btn("▶\nD", wz=-1)
        btn_stop = make_btn("■\n空格", big=True)
        btn_stop.setStyleSheet("font-size: 20px; font-weight: bold; background: #c0392b; color: #fff;")
        btn_stop.mousePressEvent = lambda e: self._teleop_stop()
        btn_stop.mouseReleaseEvent = None
        btn_lat_left = make_btn("←横\nQ", vy=1)
        btn_lat_right = make_btn("→横\nE", vy=-1)

        grid.addWidget(btn_fwd, 0, 1)
        grid.addWidget(btn_lat_left, 1, 0)
        grid.addWidget(btn_left, 1, 1)
        grid.addWidget(btn_stop, 1, 2)
        grid.addWidget(btn_right, 1, 3)
        grid.addWidget(btn_lat_right, 1, 4)
        grid.addWidget(btn_bwd, 2, 1)
        dl.addLayout(grid)

        hint = QLabel("键盘: W↑ S↓ A← D→ Q左横移 E右横移 空格急停")
        hint.setStyleSheet("color: #888; font-size: 11px;")
        dl.addWidget(hint)
        layout.addWidget(grp, 1)

        # 急停
        estop = QPushButton("🛑 紧急停止")
        estop.setStyleSheet("font-size: 16px; font-weight: bold; background: #c0392b; color: #fff; padding: 12px;")
        estop.clicked.connect(self._emergency_stop)
        layout.addWidget(estop)

        return tab

    # ---- 航点标签页 ----
    def _build_waypoint_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # 工具栏
        tb = QHBoxLayout()
        tb.addWidget(QPushButton("📌 记录当前位姿", clicked=self._wp_add))
        tb.addWidget(QPushButton("📂 加载航点", clicked=self._wp_load))
        tb.addWidget(QPushButton("💾 保存航点", clicked=self._wp_save))
        tb.addWidget(QPushButton("❌ 删除选中", clicked=self._wp_del))
        tb.addWidget(QPushButton("✏ 编辑选中", clicked=self._wp_edit))
        tb.addWidget(QPushButton("▶ 导航到选中", clicked=self._wp_go))
        tb.addStretch()

        # 巡航
        self._btn_tour = QPushButton("🚶 开始多点巡航")
        self._btn_tour.setProperty("class", "success")
        self._btn_tour.clicked.connect(self._tour_toggle)
        tb.addWidget(self._btn_tour)
        self._btn_tour_cancel = QPushButton("■ 取消巡航")
        self._btn_tour_cancel.setProperty("class", "danger")
        self._btn_tour_cancel.clicked.connect(self._tour_cancel)
        tb.addWidget(self._btn_tour_cancel)
        layout.addLayout(tb)

        # 进度
        prog_row = QHBoxLayout()
        self._tour_pb = QProgressBar()
        self._tour_pb.setFixedHeight(20)
        self._tour_label = QLabel("就绪")
        prog_row.addWidget(self._tour_pb, 1)
        prog_row.addWidget(self._tour_label)
        layout.addLayout(prog_row)

        # 航点列表
        self._wp_list = QListWidget()
        self._wp_list.setAlternatingRowColors(True)
        self._wp_list.setStyleSheet("alternate-background-color: #2a2a2a; font-size: 12px;")
        self._wp_list.itemDoubleClicked.connect(lambda i: self._wp_edit())
        layout.addWidget(self._wp_list, 1)

        return tab

    # ---- 动作标签页 ----
    def _build_action_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # G1 连接
        grp = QGroupBox("G1 机器人连接")
        gl = QHBoxLayout(grp)
        self._btn_g1 = QPushButton("连接 G1")
        self._btn_g1.clicked.connect(self._on_g1_toggle)
        gl.addWidget(self._btn_g1)
        self._g1_label = QLabel("状态: 未连接")
        gl.addWidget(self._g1_label)
        gl.addStretch()
        layout.addWidget(grp)

        # FSM 模式
        grp = QGroupBox("FSM 模式切换")
        gl = QHBoxLayout(grp)
        for txt, fid in [("行走模式", 200), ("阻尼模式", 1), ("坐下", 3), ("站起", -1)]:
            btn = QPushButton(txt)
            if fid == -1:
                btn.clicked.connect(self._g1_stand)
            else:
                btn.clicked.connect(lambda checked, f=fid: self._g1_api(lambda: self._g1_loco.SetFsmId(f)))
            gl.addWidget(btn)
        gl.addStretch()
        layout.addWidget(grp)

        # 手臂动作
        grp = QGroupBox("手臂预设动作")
        gl = QHBoxLayout(grp)
        common = ["face wave", "clap", "hug", "heart", "right hand up", "reject", "shake hand", "x-ray", "high five"]
        for aname in common:
            btn = QPushButton(aname.replace("_", " ").title())
            btn.clicked.connect(lambda checked, n=aname: self._g1_arm_action(n))
            gl.addWidget(btn)
        gl.addStretch()
        layout.addWidget(grp)

        # 全部动作展开
        grp = QGroupBox("全部手臂动作")
        gl = QHBoxLayout(grp)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        sw = QWidget()
        fl = QHBoxLayout(sw)
        for aname in sorted(ARM_ACTIONS.keys()):
            btn = QPushButton(aname.replace("_", " ").title())
            btn.setFixedWidth(100)
            btn.clicked.connect(lambda checked, n=aname: self._g1_arm_action(n))
            fl.addWidget(btn)
        fl.addStretch()
        scroll.setWidget(sw)
        gl.addWidget(scroll)
        layout.addWidget(grp)

        # TTS 语音
        grp = QGroupBox("语音播报 (TTS)")
        gl = QVBoxLayout(grp)
        row = QHBoxLayout()
        self._tts_input = QLineEdit()
        self._tts_input.setPlaceholderText("输入播报文字…")
        self._tts_input.returnPressed.connect(self._on_tts)
        row.addWidget(self._tts_input, 1)
        btn_tts = QPushButton("🔊 播报")
        btn_tts.clicked.connect(self._on_tts)
        row.addWidget(btn_tts)
        gl.addLayout(row)

        # 预设语音
        phrases_row = QHBoxLayout()
        for phrase in ["欢迎参观", "请跟我来", "这是我们的展品", "谢谢大家",
                        "请注意安全", "正在前往下一个展品"]:
            btn = QPushButton(phrase)
            btn.clicked.connect(lambda checked, p=phrase: self._g1_speak(p))
            phrases_row.addWidget(btn)
        phrases_row.addStretch()
        gl.addLayout(phrases_row)
        layout.addWidget(grp)

        # LED + 音量
        grp = QGroupBox("LED & 音量")
        gl = QHBoxLayout(grp)
        gl.addWidget(QLabel("LED:"))
        for txt, r, g, b in [("红", 255, 0, 0), ("绿", 0, 255, 0), ("蓝", 0, 0, 255),
                              ("白", 255, 255, 255), ("关", 0, 0, 0)]:
            btn = QPushButton(txt)
            btn.setFixedWidth(32)
            btn.clicked.connect(lambda checked, rr=r, gg=g, bb=b: self._g1_api(
                lambda: self._g1_audio.LedControl(rr, gg, bb)))
            gl.addWidget(btn)
        gl.addStretch()
        gl.addWidget(QLabel("音量:"))
        self._vol_slider = QSlider(Qt.Horizontal)
        self._vol_slider.setRange(0, 100)
        self._vol_slider.setValue(50)
        self._vol_slider.valueChanged.connect(lambda v: self._g1_api(lambda: self._g1_audio.SetVolume(v)))
        gl.addWidget(self._vol_slider)
        layout.addWidget(grp)

        layout.addStretch()
        return tab

    # ---- 设置标签页 ----
    def _build_settings_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        grp = QGroupBox("网络 & 路径")
        fm = QFormLayout(grp)

        self._edit_net = QLineEdit(self.cfg.get("net_if", "eno1"))
        fm.addRow("G1 网卡:", self._edit_net)

        self._chk_auto_ros = QCheckBox("启动时自动连接 ROS")
        self._chk_auto_ros.setChecked(self.cfg.get("auto_start_ros", True))
        fm.addRow(self._chk_auto_ros)

        self._chk_auto_map = QCheckBox("启动时自动加载上次地图")
        self._chk_auto_map.setChecked(True)
        fm.addRow(self._chk_auto_map)

        layout.addWidget(grp)

        # 说明
        info = QLabel(
            "<h3>使用说明</h3>"
            "<ol>"
            "<li><b>启动导航</b> — 设置好地图路径后点击「启动导航」，自动启动 ROS 导航栈</li>"
            "<li><b>重定位</b> — 点击「2D Pose Estimate」然后在地图上点击机器人位置，或手动输入坐标</li>"
            "<li><b>遥控</b> — 使用 WASD 或按钮控制机器人移动</li>"
            "<li><b>航点</b> — 记录当前位置为航点，支持单点导航和多点巡航</li>"
            "<li><b>动作</b> — 连接 G1 后可执行手臂动作和语音播报</li>"
            "</ol>"
            "<p style='color:#888'>项目路径: ~/Desktop/HongTu/g1_nav_panel/</p>"
        )
        info.setWordWrap(True)
        info.setStyleSheet("padding: 16px; font-size: 12px;")
        layout.addWidget(info)

        layout.addStretch()
        return tab

    # ================================================================
    # ROS / G1 管理
    # ================================================================
    def _start_ros(self):
        if not ROS_OK:
            self._log("[ROS] 库不可用，导航和地图功能不可用")
            self._status_ros.setText("ROS: 不可用")
            return
        if self._ros_worker and self._ros_worker.isRunning():
            return
        # 检查 roscore
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect(("localhost", 11311))
            s.close()
        except Exception:
            self._log("[ROS] ⚠ roscore 未运行，请先启动: roscore &")
            self._status_ros.setText("ROS: 无 roscore")
            # 仍然尝试启动 worker，以防后续 roscore 启动
        self._ros_worker = RosWorker()
        self._ros_worker.pose_updated.connect(self._on_pose)
        self._ros_worker.map_updated.connect(self._on_map)
        self._ros_worker.nav_status_updated.connect(self._on_nav_status)
        self._ros_worker.goal_done.connect(self._on_goal_done)
        self._ros_worker.log_msg.connect(self._log)
        self._ros_worker.nav_cmd_vel.connect(self._on_nav_cmd_vel)
        self._ros_worker.start()
        self._status_ros.setText("ROS: 已连接")

    def _on_nav_start(self):
        map_yaml = self._edit_map.text().strip()
        pcd_path = self._edit_pcd.text().strip()

        if not os.path.exists(map_yaml):
            QMessageBox.warning(self, "地图文件不存在", f"请选择有效的 2D 地图文件\n{map_yaml}")
            return
        if not os.path.exists(pcd_path):
            QMessageBox.warning(self, "点云文件不存在", f"请选择有效的 PCD 文件\n{pcd_path}")
            return

        # 保存配置
        self.cfg["map_yaml"] = map_yaml
        self.cfg["pcd_path"] = pcd_path
        save_config(self.cfg)

        # 找到 launch 文件
        launch_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "nav_start.launch")
        if not os.path.exists(launch_file):
            self._log(f"[错误] 找不到 launch 文件: {launch_file}")
            return

        if self._nav_proc and self._nav_proc.poll() is None:
            self._log("[导航] 已在运行中")
            return

        # 启动 roslaunch
        env = os.environ.copy()
        env["ROS_MASTER_URI"] = env.get("ROS_MASTER_URI", "http://localhost:11311")
        cmd = [
            "roslaunch",
            launch_file,
            f"map_yaml:={map_yaml}",
            f"pcd_path:={pcd_path}",
        ]
        self._log(f"[导航] 启动: {' '.join(cmd)}")
        self._nav_proc = subprocess.Popen(
            cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
            universal_newlines=True, bufsize=1
        )
        self._nav_status_label.setText("导航: 启动中…")
        self._nav_status_label.setStyleSheet("font-weight: bold; color: #f39c12; padding: 4px 12px;")

        # 读取输出线程
        def read_output():
            for line in iter(self._nav_proc.stdout.readline, ""):
                self._log(f"[nav] {line.rstrip()}")
            self._nav_proc.stdout.close()
            self._log("[导航] 进程已退出")

        threading.Thread(target=read_output, daemon=True).start()
        self._btn_nav_start.setEnabled(False)
        self._btn_nav_stop.setEnabled(True)

        # 定时检查状态
        QTimer.singleShot(3000, self._check_nav_started)

    def _check_nav_started(self):
        if self._nav_proc and self._nav_proc.poll() is None:
            self._nav_status_label.setText("导航: 运行中")
            self._nav_status_label.setStyleSheet("font-weight: bold; color: #27ae60; padding: 4px 12px;")
        else:
            self._nav_status_label.setText("导航: 启动失败")
            self._nav_status_label.setStyleSheet("font-weight: bold; color: #c0392b; padding: 4px 12px;")
            self._btn_nav_start.setEnabled(True)
            self._btn_nav_stop.setEnabled(False)

    def _on_nav_stop(self):
        if self._nav_proc:
            self._log("[导航] 停止中…")
            self._nav_proc.terminate()
            try:
                self._nav_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._nav_proc.kill()
            self._nav_proc = None
        self._nav_status_label.setText("导航: 已停止")
        self._nav_status_label.setStyleSheet("font-weight: bold; color: #aaa; padding: 4px 12px;")
        self._btn_nav_start.setEnabled(True)
        self._btn_nav_stop.setEnabled(False)

    def _on_g1_toggle(self):
        if self._g1_ready:
            self._g1_ready = False
            self._btn_g1.setText("连接 G1")
            self._g1_label.setText("状态: 已断开")
            self._btn_nav_g1.setText("连接 G1")
            self._nav_g1_label.setText("未连接")
            self._nav_g1_label.setStyleSheet("color: #888;")
            self._status_g1.setText("G1: 未连接")
            return
        if not G1_OK:
            QMessageBox.warning(self, "SDK 不可用", "unitree_sdk2py 未安装")
            return

        net_if = self._nav_net_if.text().strip()
        try:
            ChannelFactoryInitialize(0, net_if)
            self._g1_loco = LocoClient()
            self._g1_loco.SetTimeout(10.0)
            self._g1_loco.Init()
            self._g1_loco.Start()

            self._g1_arm = G1ArmActionClient()
            self._g1_arm.SetTimeout(10.0)
            self._g1_arm.Init()

            self._g1_audio = G1AudioClient()
            self._g1_audio.SetTimeout(10.0)
            self._g1_audio.Init()
            self._g1_audio.SetVolume(85)
            # 修补 SDK bug：原代码 self.tts_index += self.tts_index 永远是 0
            import types
            _real_tts_index = [1]
            def _fixed_tts(client_self, text, speaker_id):
                _real_tts_index[0] += 1
                p = {"index": _real_tts_index[0], "text": text, "speaker_id": speaker_id}
                code, data = client_self._Call(1001, json.dumps(p))
                return code
            self._g1_audio.TtsMaker = types.MethodType(_fixed_tts, self._g1_audio)

            self._g1_ready = True
            self._btn_g1.setText("断开 G1")
            self._g1_label.setText("状态: 已连接 ✓")
            self._btn_nav_g1.setText("断开 G1")
            self._nav_g1_label.setText("已连接")
            self._nav_g1_label.setStyleSheet("color: #27ae60; font-weight: bold;")
            self._status_g1.setText("G1: 已连接")
            self._log("[G1] 连接成功")
        except Exception as e:
            QMessageBox.warning(self, "G1 连接失败", str(e))
            self._log(f"[G1] 连接失败: {e}")

    # ================================================================
    # 回调
    # ================================================================
    def _on_pose(self, x, y, yaw):
        self._last_pose = (x, y, yaw)
        self._status_pose.setText(f"位姿: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)")
        self._map_view.update_robot(x, y, yaw)

    def _on_map(self, occ_grid):
        self._map_data = occ_grid
        self._map_view.set_map(occ_grid)
        self._map_view.update_waypoints(self._waypoints)

    def _on_nav_status(self, text):
        self._nav_state_label.setText(text)
        color = {"已到达": "#27ae60", "失败": "#c0392b", "导航中": "#f39c12"}.get(text, "#aaa")
        self._nav_state_label.setStyleSheet(f"font-size: 14px; font-weight: bold; color: {color};")

    def _on_goal_done(self, success):
        if self._tour_running:
            self._tour_next_step()

    def _on_nav_cmd_vel(self, vx, vy, wz):
        """将 move_base 的 cmd_vel 转发给 G1"""
        if self._g1_ready and self._g1_loco:
            try:
                if abs(vx) < 0.001 and abs(vy) < 0.001 and abs(wz) < 0.001:
                    self._g1_loco.StopMove()
                else:
                    self._g1_loco.Move(vx, vy, wz, continous_move=True)
            except Exception:
                pass

    # ---- 重定位 ----
    _reloc_mode = False

    def _on_reloc_mode(self):
        self._reloc_mode = not self._reloc_mode
        if self._reloc_mode:
            self._btn_reloc.setText("✅ 点击地图 → 设置初始位姿")
            self._btn_reloc.setStyleSheet("background: #27ae60; color: #fff; font-weight: bold;")
            self._step_label.setText("📍 重定位模式: 在地图上点击机器人实际位置 → 自动设置朝向")
            self._step_label.setStyleSheet("background: #2d2d2d; color: #27ae60; padding: 6px 12px; border-radius: 3px; font-size: 12px; font-weight: bold;")
            self._map_view.clicked.connect(self._on_map_click_reloc)
        else:
            self._btn_reloc.setText("📌 2D Pose Estimate (点击地图设置)")
            self._btn_reloc.setStyleSheet("")
            self._step_label.setText("① 启动导航  →  ② 设置重定位  →  ③ 点击地图或航点发送导航目标")
            self._step_label.setStyleSheet("background: #2d2d2d; color: #f39c12; padding: 6px 12px; border-radius: 3px; font-size: 12px;")
            try:
                self._map_view.clicked.disconnect(self._on_map_click_reloc)
            except Exception:
                pass

    def _on_map_click_reloc(self, mx, my):
        _, _, yaw = self._last_pose
        self._reloc_x.setText(f"{mx:.2f}")
        self._reloc_y.setText(f"{my:.2f}")
        self._on_reloc_set()
        self._log(f"[重定位] 设置位姿: ({mx:.2f}, {my:.2f}) 朝向: {math.degrees(yaw):.1f}°")

    def _on_reloc_set(self):
        try:
            x = float(self._reloc_x.text())
            y = float(self._reloc_y.text())
            _, _, yaw = self._last_pose
            if self._ros_worker:
                self._ros_worker.send_init_pose(x, y, yaw)
                self._log(f"[重定位] 发布初始位姿: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)")
        except ValueError:
            QMessageBox.warning(self, "输入错误", "坐标格式错误")

    # ---- 遥控 ----
    def _teleop_start(self, vx=0, vy=0, wz=0):
        self._teleop_active = True
        lin = self._slider_lin.value() / 100.0
        ang = self._slider_ang.value() / 100.0
        self._teleop_vx = vx * lin
        self._teleop_vy = vy * lin
        self._teleop_wz = wz * ang
        if self._ros_worker:
            self._ros_worker.send_cmd_vel(self._teleop_vx, self._teleop_vy, self._teleop_wz)
        self._g1_move(self._teleop_vx, self._teleop_vy, self._teleop_wz)

    def _teleop_stop(self):
        self._teleop_active = False
        self._teleop_vx = self._teleop_vy = self._teleop_wz = 0.0
        if self._ros_worker:
            self._ros_worker.send_cmd_vel(0, 0, 0)
        self._g1_move(0, 0, 0)

    def _emergency_stop(self):
        self._teleop_stop()
        if self._tour_running:
            self._tour_cancel()
        if self._ros_worker:
            self._ros_worker.send_cmd_vel(0, 0, 0)
        self._g1_move(0, 0, 0)
        self._log("[急停] 已停止所有运动")

    # ---- G1 辅助 ----
    def _g1_api(self, func):
        if self._g1_ready:
            try:
                func()
            except Exception:
                pass

    def _g1_move(self, vx, vy, wz):
        if self._g1_ready:
            try:
                self._g1_loco.Move(vx, vy, wz, continous_move=True)
            except Exception:
                pass

    def _g1_arm_action(self, name):
        for aname_str, aid_val in ARM_ACTIONS.items():
            if aname_str == name:
                self._g1_api(lambda i=aid_val: self._g1_arm.ExecuteAction(i))
                self._log(f"[动作] {aname_str}")
                break

    def _g1_stand(self):
        if self._g1_ready:
            try:
                self._g1_loco.Start()  # FSM=200 直接进入行走模式
                self._log("[G1] 行走模式")
            except Exception as e:
                self._log(f"[G1] 站起失败: {e}")

    def _g1_speak(self, text):
        if self._g1_ready and text.strip():
            try:
                self._log(f"[语音] 播报: {text}")
                self._g1_audio.TtsMaker(text.strip(), 0)
            except Exception as e:
                self._log(f"[语音] 失败: {e}")

    def _on_tts(self):
        self._g1_speak(self._tts_input.text())

    def _on_nav_tts(self):
        self._g1_speak(self._nav_tts.text())

    # ---- 航点 ----
    def _wp_add(self):
        x, y, yaw = self._last_pose
        d = WaypointDialog(self, f"航点{len(self._waypoints) + 1}", x, y, yaw)
        if d.exec_():
            self._waypoints.append(d.result())
            self._wp_refresh()

    def _wp_del(self):
        row = self._wp_list.currentRow()
        if row >= 0 and row < len(self._waypoints):
            self._waypoints.pop(row)
            self._wp_refresh()

    def _wp_edit(self):
        row = self._wp_list.currentRow()
        if row < 0 or row >= len(self._waypoints):
            return
        wp = self._waypoints[row]
        d = WaypointDialog(self, *wp)
        if d.exec_():
            self._waypoints[row] = d.result()
            self._wp_refresh()

    def _wp_go(self):
        row = self._wp_list.currentRow()
        if row < 0 or row >= len(self._waypoints):
            QMessageBox.warning(self, "提示", "请先选择一个航点")
            return
        _, x, y, yaw, _, _ = self._waypoints[row]
        if self._ros_worker:
            self._ros_worker.send_goal(x, y, yaw)
        self._log(f"[导航] 发送目标: ({x:.2f}, {y:.2f})")

    def _wp_save(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "保存航点", os.path.expanduser("~"), "JSON (*.json)")
        if not path:
            return
        data = [{"name": n, "x": x, "y": y, "yaw": yaw, "action": a, "speech": s}
                for n, x, y, yaw, a, s in self._waypoints]
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            self._log(f"[航点] 已保存 {len(data)} 个航点")
        except Exception as e:
            QMessageBox.warning(self, "保存失败", str(e))

    def _wp_load(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "加载航点", os.path.expanduser("~"), "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            self._waypoints = [(d["name"], d["x"], d["y"], d["yaw"],
                                d.get("action", ""), d.get("speech", "")) for d in data]
            self._wp_refresh()
            self._log(f"[航点] 加载了 {len(self._waypoints)} 个航点")
        except Exception as e:
            QMessageBox.warning(self, "加载失败", str(e))

    def _wp_refresh(self):
        self._wp_list.clear()
        for i, (name, x, y, yaw, action, speech) in enumerate(self._waypoints):
            txt = f"{i+1}. {name}  ({x:.1f}, {y:.1f}) {math.degrees(yaw):.0f}°"
            if action:
                txt += f"  [动作: {action}]"
            if speech:
                txt += f"  [语音: {speech}]"
            self._wp_list.addItem(txt)
        self._map_view.update_waypoints(self._waypoints)

    # ---- 巡航 ----
    def _tour_toggle(self):
        if self._tour_running:
            return
        if len(self._waypoints) < 1:
            QMessageBox.warning(self, "提示", "请先添加航点")
            return
        self._tour_running = True
        self._tour_idx = 0
        self._tour_pb.setMaximum(len(self._waypoints))
        self._tour_pb.setValue(0)
        self._btn_tour.setEnabled(False)
        self._log(f"[巡航] 开始，共 {len(self._waypoints)} 个航点")
        # 发送第一个目标，由 goal_done 信号驱动后续
        self._tour_idx = 1  # 第一个是"下一个"目标
        name, x, y, yaw, _, _ = self._waypoints[0]
        self._tour_pb.setValue(1)
        self._tour_label.setText(f"→ {name}")
        self._log(f"[巡航] [1/{len(self._waypoints)}] {name}")
        if self._ros_worker:
            self._ros_worker.send_goal(x, y, yaw)

    def _tour_next_step(self):
        """由导航完成或超时触发下一步"""
        if not self._tour_running:
            return

        # 当前航点的动作 + 语音
        cur = self._tour_idx - 1  # _tour_idx 已递增
        if 0 <= cur < len(self._waypoints):
            _, _, _, _, action, speech = self._waypoints[cur]
            if action and action != "无":
                for aname_str, aid_val in ARM_ACTIONS.items():
                    if aname_str == action:
                        self._g1_api(lambda i=aid_val: self._g1_arm.ExecuteAction(i))
                        break
            if speech:
                self._g1_speak(speech)

        # 前往下一个航点
        if self._tour_idx >= len(self._waypoints):
            self._tour_done()
            return

        name, x, y, yaw, _, _ = self._waypoints[self._tour_idx]
        self._tour_pb.setValue(self._tour_idx + 1)
        self._tour_label.setText(f"→ {name}")
        self._log(f"[巡航] [{self._tour_idx + 1}/{len(self._waypoints)}] {name}")

        if self._ros_worker:
            self._ros_worker.send_goal(x, y, yaw)

        self._tour_idx += 1

    def _tour_done(self):
        self._tour_running = False
        self._btn_tour.setEnabled(True)
        self._tour_label.setText("巡航完成" if self._tour_idx >= len(self._waypoints) else "巡航中断")
        self._log(f"[巡航] {'完成' if self._tour_idx >= len(self._waypoints) else '中断'}")

    def _tour_cancel(self):
        self._tour_running = False
        self._btn_tour.setEnabled(True)
        self._tour_label.setText("已取消")
        if self._ros_worker:
            self._ros_worker.send_cmd_vel(0, 0, 0)

    # ---- 工具 ----
    def _browse_file(self, edit, filt):
        path, _ = QFileDialog.getOpenFileName(self, "选择文件", edit.text(), filt)
        if path:
            edit.setText(path)

    def _log(self, msg):
        """线程安全的日志输出"""
        self.log_message.emit(msg)

    def _append_log(self, msg):
        """真正写入日志（仅在 GUI 线程调用）"""
        self._log_view.appendPlainText(msg)
        sb = self._log_view.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _load_settings(self):
        # 自动加载上次的地图路径
        if self.cfg.get("map_yaml"):
            self._edit_map.setText(self.cfg["map_yaml"])
        if self.cfg.get("pcd_path"):
            self._edit_pcd.setText(self.cfg["pcd_path"])
        if self.cfg.get("net_if"):
            self._edit_net.setText(self.cfg["net_if"])

    def closeEvent(self, e):
        self._on_nav_stop()
        if self._ros_worker:
            self._ros_worker.stop()
            self._ros_worker.wait(2000)
        # 保存配置
        self.cfg["net_if"] = self._edit_net.text()
        self.cfg["map_yaml"] = self._edit_map.text()
        self.cfg["pcd_path"] = self._edit_pcd.text()
        save_config(self.cfg)
        super().closeEvent(e)


# ============================================================
# 入口
# ============================================================
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setApplicationName("G1 导航控制台")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
