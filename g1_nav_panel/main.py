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
import atexit
import socket
import subprocess
import sys
import threading
import time
from enum import IntEnum

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
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
    from unitree_sdk2py.g1.arm.g1_arm_action_client import action_map as ARM_ACTIONS
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient as G1AudioClient
    from unitree_sdk2py.core.channel import ChannelPublisher
    G1_OK = True
except Exception:
    G1_OK = False
    ARM_ACTIONS = {}

# ============================================================
# G1 低阶手臂控制 (arm_sdk)
# ============================================================
try:
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    ARM_LOW_OK = True
except Exception:
    ARM_LOW_OK = False

# ============================================================
# Inspire RH56E2 灵巧手导入
# ============================================================
_HAND_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "inspire_hand")
if _HAND_PATH not in sys.path:
    sys.path.insert(0, _HAND_PATH)
try:
    from inspire_sdkpy.inspire_dds import inspire_hand_ctrl as _hand_ctrl_type
    from inspire_sdkpy.inspire_dds import inspire_hand_state as _hand_state_type
    from inspire_sdkpy.inspire_hand_defaut import get_inspire_hand_ctrl
    HAND_OK = True
except Exception:
    HAND_OK = False
    _hand_ctrl_type = None
    _hand_state_type = None
    get_inspire_hand_ctrl = None

# 导入灵巧手控制面板组件（复用 UI 组件）
try:
    _PANEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    if _PANEL_PATH not in sys.path:
        sys.path.insert(0, _PANEL_PATH)
    from hand_control_panel import MotorRow, HandCanvas, FINGER_NAMES as _FN
    HAS_HAND_WIDGETS = True
except Exception:
    HAS_HAND_WIDGETS = False

try:
    from g1_remote_client import G1RemoteClient, G1RemoteError
    REMOTE_G1_OK = True
except Exception:
    G1RemoteClient = None
    G1RemoteError = Exception
    REMOTE_G1_OK = False

# 英文动作名 → 中文显示名
ACTION_CN = {
    "face wave": "挥手",
    "high wave": "高举挥手",
    "clap": "鼓掌",
    "hug": "拥抱",
    "heart": "比心",
    "right heart": "右手比心",
    "reject": "拒绝",
    "right hand up": "举右手",
    "hands up": "双手举高",
    "x-ray": "展示",
    "shake hand": "握手",
    "high five": "击掌",
    "two-hand kiss": "双手飞吻",
    "left kiss": "左手飞吻",
    "right kiss": "右手飞吻",
    "release arm": "释放手臂",
}

# ============================================================
# RH56E2 灵巧手常量
# ============================================================
HAND_PRESETS = {
    "张开":   [1000, 1000, 1000, 1000, 1000, 1000],
    "握拳":   [0, 0, 0, 0, 0, 0],
    "指向":   [0, 0, 0, 1000, 0, 500],
    "OK":     [0, 0, 0, 0, 300, 300],
    "点赞":   [0, 0, 0, 0, 1000, 500],
    "摇滚":   [1000, 0, 0, 1000, 1000, 500],
    "三指捏": [0, 0, 300, 300, 300, 300],
    "半开":   [500, 500, 500, 500, 500, 500],
    "点按":   [0, 0, 0, 800, 0, 500],
}

# G1 手臂动作名 → (G1 动作名, 灵巧手势名)
COORDINATED_ACTIONS = {
    "face wave":   ("face wave",  "张开"),
    "shake hand":  ("shake hand", "握拳"),
    "clap":        ("clap",       "张开"),
    "heart":       ("heart",      "OK"),
    "right hand up": ("right hand up", "张开"),
    "high five":   ("high five",  "张开"),
    "hug":         ("hug",        "半开"),
    "reject":      ("reject",     "张开"),
    "x-ray":       ("x-ray",      "张开"),
    "hands up":    ("hands up",   "张开"),
}

# ============================================================
# G1 低阶手臂控制常量 (arm_sdk)
# ============================================================
G1_ARM_JOINT_NAMES = [
    "左肩前后", "左肩左右", "左肩旋转", "左肘", "左腕旋转", "左腕俯仰", "左腕偏航",
    "右肩前后", "右肩左右", "右肩旋转", "右肘", "右腕旋转", "右腕俯仰", "右腕偏航",
]
G1_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]
# 每个关节的 Kp/Kd 和角度范围（弧度）
G1_ARM_PARAMS = [
    {"kp": 80.0, "kd": 3.0, "min": -2.0, "max": 2.0},
    {"kp": 80.0, "kd": 3.0, "min": -1.5, "max": 1.5},
    {"kp": 80.0, "kd": 3.0, "min": -2.5, "max": 2.5},
    {"kp": 80.0, "kd": 3.0, "min": -2.5, "max": 3.0},
    {"kp": 40.0, "kd": 1.5, "min": -1.5, "max": 1.5},
    {"kp": 40.0, "kd": 1.5, "min": -1.0, "max": 1.0},
    {"kp": 40.0, "kd": 1.5, "min": -1.0, "max": 1.0},
    {"kp": 80.0, "kd": 3.0, "min": -2.0, "max": 2.0},
    {"kp": 80.0, "kd": 3.0, "min": -1.5, "max": 1.5},
    {"kp": 80.0, "kd": 3.0, "min": -2.5, "max": 2.5},
    {"kp": 80.0, "kd": 3.0, "min": -2.5, "max": 3.0},
    {"kp": 40.0, "kd": 1.5, "min": -1.5, "max": 1.5},
    {"kp": 40.0, "kd": 1.5, "min": -1.0, "max": 1.0},
    {"kp": 40.0, "kd": 1.5, "min": -1.0, "max": 1.0},
]
G1_ARM_DOF = len(G1_ARM_JOINT_IDS)
G1_ARM_SDK_ENABLE_JOINT = 29  # kNotUsedJoint, q=1 启用 arm_sdk


class G1JointIndex(IntEnum):
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28
    NotUsedJoint0 = 29
    NotUsedJoint1 = 30
    NotUsedJoint2 = 31
    NotUsedJoint3 = 32
    NotUsedJoint4 = 33
    NotUsedJoint5 = 34


WEAK_MOTORS = {
    G1JointIndex.LeftAnklePitch,
    G1JointIndex.RightAnklePitch,
    G1JointIndex.LeftShoulderPitch,
    G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw,
    G1JointIndex.LeftElbow,
    G1JointIndex.RightShoulderPitch,
    G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,
    G1JointIndex.RightElbow,
}

WRIST_MOTORS = {
    G1JointIndex.LeftWristRoll,
    G1JointIndex.LeftWristPitch,
    G1JointIndex.LeftWristYaw,
    G1JointIndex.RightWristRoll,
    G1JointIndex.RightWristPitch,
    G1JointIndex.RightWristYaw,
}

# ============================================================
# 配置
# ============================================================
APP_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CONFIG_FILE = os.environ.get(
    "HONGTU_CONFIG_FILE",
    os.path.expanduser("~/.g1_nav_panel.json"),
)
DEFAULT_CONFIG = {
    "net_if": os.environ.get("HONGTU_G1_NET_IF", "eno1"),
    "map_yaml": os.environ.get("HONGTU_MAP_YAML", os.path.expanduser("~/Desktop/G1map.yaml")),
    "pcd_path": os.environ.get(
        "HONGTU_PCD_PATH",
        os.path.join(APP_ROOT, "G1Nav2D/src/fastlio2/PCD/map.pcd"),
    ),
    "auto_start_ros": True,
    "window_geometry": None,
}


def init_unitree_channel(net_if):
    """Initialize Unitree DDS; empty/auto lets robot-side SDK choose locally."""
    net_if = (net_if or "").strip()
    if net_if and net_if.lower() not in ("auto", "local", "none"):
        ChannelFactoryInitialize(0, net_if)
    else:
        ChannelFactoryInitialize(0)

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
    request_reloc = pyqtSignal(float, float, float)   # x, y, yaw → ICP 重定位
    # 从 ROS 线程发往主线程的
    nav_cmd_vel = pyqtSignal(float, float, float)  # vx, vy, wz
    reloc_done = pyqtSignal(bool, str)             # success, message

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False
        self._pub_cmd = None
        self._pub_goal = None
        self._pub_initpose = None
        self._pose = (0.0, 0.0, 0.0)
        self._shutdown = False
        self._pcd_path = ""
        self._goal_pending = False
        self._goal_was_active = False

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
            self.request_reloc.connect(self._do_reloc)

            # TF 监听器（用于获取 map→base_link 位姿，包含 ICP 修正）
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
            # 静态 TF 发布：body → base_link（替代 base_to_body 节点）
            self._static_br = tf2_ros.StaticTransformBroadcaster()
            import geometry_msgs.msg
            static_tf = geometry_msgs.msg.TransformStamped()
            static_tf.header.stamp = rospy.Time.now()
            static_tf.header.frame_id = "body"
            static_tf.child_frame_id = "base_link"
            static_tf.transform.rotation.x = 0.0
            static_tf.transform.rotation.y = 0.0
            static_tf.transform.rotation.z = 1.0
            static_tf.transform.rotation.w = 0.0  # 180° around Z
            self._static_br.sendTransform(static_tf)
            self.log_msg.emit("[ROS] 发布静态 TF: body → base_link")

            rospy.Subscriber("/slam_odom", Odometry, self._odom_cb)
            rospy.Subscriber("/map", OccupancyGrid, self._map_cb)
            rospy.Subscriber("/move_base/status", GoalStatusArray, self._status_cb)
            rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_bridge_cb)

            # 重定位服务延迟连接（导航启动后才可用）
            self._reloc_srv = None

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
        self._goal_pending = True
        self._goal_was_active = False  # 新目标，还没进入 active 状态  # 标记正在等待导航完成
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

    def _do_reloc(self, x, y, yaw):
        """重定位：发布 /initialpose（和 RViz 2D Pose Estimate 一样）"""
        try:
            p = PoseWithCovarianceStamped()
            p.header.frame_id = "map"
            p.header.stamp = rospy.Time.now()
            p.pose.pose.position.x = float(x)
            p.pose.pose.position.y = float(y)
            p.pose.pose.position.z = 0.0
            q = tf_tr.quaternion_from_euler(0, 0, float(yaw))
            p.pose.pose.orientation.x, p.pose.pose.orientation.y = q[0], q[1]
            p.pose.pose.orientation.z, p.pose.pose.orientation.w = q[2], q[3]
            self._pub_initpose.publish(p)
            self.reloc_done.emit(True, f"已发送重定位: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)")
            self.log_msg.emit(f"[重定位] 发布 initialpose: ({x:.2f}, {y:.2f}) 朝向 {math.degrees(yaw):.0f}°")
        except Exception as e:
            self.reloc_done.emit(False, str(e))
            self.log_msg.emit(f"[重定位] 失败: {e}")

    def _odom_cb(self, msg):
        """里程计回调：优先用 TF（含 ICP 修正），降级到 odometry"""
        # 先用 odometry 兜底
        q = msg.pose.pose.orientation
        _, _, yaw = tf_tr.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        # 非阻塞尝试 TF（ICP 重定位后 map→body 会更新）
        try:
            t = self._tf_buffer.lookup_transform("map", "body", rospy.Time(0))
            tx, ty = t.transform.translation.x, t.transform.translation.y
            tq = t.transform.rotation
            _, _, tyaw = tf_tr.euler_from_quaternion([tq.x, tq.y, tq.z, tq.w])
            self._pose = (tx, ty, tyaw)
        except Exception:
            pass  # TF 不可用，用 odometry
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
            if self._goal_pending:
                if s == 1:  # active — 确认 move_base 正在执行
                    self._goal_was_active = True
                if self._goal_was_active and s == 3:  # succeeded（必须先 active 过）
                    self._goal_pending = False
                    self._goal_was_active = False
                    self.goal_done.emit(True)
                elif s in (2, 4, 5, 8):  # failed/canceled
                    self._goal_pending = False
                    self._goal_was_active = False
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
    pose_clicked = pyqtSignal(float, float, float)  # 拖拽重定位: x, y, yaw

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setBackgroundBrush(QBrush(QColor("#0d1117")))

        self._map_item = None
        self._robot_item = None
        self._wp_items = []
        self._res = 0.05
        self._origin = (0.0, 0.0)
        self._width = 0
        self._height = 0
        self._scale = 1.0

        # 拖拽重定位状态
        self._reloc_mode = False
        self._drag_start_scene = None  # 拖拽起点（场景坐标）
        self._drag_arrow = None  # 拖拽箭头

    def set_reloc_mode(self, on):
        self._reloc_mode = on
        if on:
            self.setDragMode(QGraphicsView.NoDrag)
        else:
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            self._clear_drag_arrow()

    def _clear_drag_arrow(self):
        if self._drag_arrow:
            for item in self._drag_arrow:
                self._scene.removeItem(item)
            self._drag_arrow = None

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
        """更新机器人位置 — 三角箭头 + 发光效果"""
        if self._robot_item:
            self._scene.removeItem(self._robot_item)
            self._robot_item = None
        if self._res <= 0:
            return

        px = (x - self._origin[0]) / self._res
        py = (y - self._origin[1]) / self._res
        items = []

        # 外圈发光
        glow = self._scene.addEllipse(px - 16, py - 16, 32, 32,
                                       QPen(QColor("#e94560"), 1), QBrush(QColor(233, 69, 96, 40)))
        items.append(glow)

        # 三角箭头 — 指向朝向
        size = 12
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        # 尖端
        tip_x = px + size * 1.5 * cos_y
        tip_y = py + size * 1.5 * sin_y
        # 左翼
        left_x = px + size * (-0.5 * cos_y - 0.8 * sin_y)
        left_y = py + size * (-0.5 * sin_y + 0.8 * cos_y)
        # 右翼
        right_x = px + size * (-0.5 * cos_y + 0.8 * sin_y)
        right_y = py + size * (-0.5 * sin_y - 0.8 * cos_y)
        # 尾部
        tail_x = px - size * 0.3 * cos_y
        tail_y = py - size * 0.3 * sin_y

        poly = QPolygonF([
            QPointF(tip_x, tip_y),
            QPointF(left_x, left_y),
            QPointF(tail_x, tail_y),
            QPointF(right_x, right_y),
        ])
        arrow = self._scene.addPolygon(poly,
                                        QPen(QColor("#ffffff"), 2),
                                        QBrush(QColor("#e94560")))
        items.append(arrow)

        # 中心点
        center = self._scene.addEllipse(px - 3, py - 3, 6, 6,
                                         QPen(Qt.NoPen), QBrush(QColor("#ffffff")))
        items.append(center)

        # 坐标标签（带背景）
        txt = f"({x:.1f}, {y:.1f})"
        label = self._scene.addText(txt, QFont("Arial", 9, QFont.Bold))
        label.setPos(px + 18, py - 20)
        label.setDefaultTextColor(QColor("#ffffff"))
        # 标签背景
        bg = self._scene.addRect(label.boundingRect().translated(px + 18, py - 20),
                                  QPen(Qt.NoPen), QBrush(QColor(15, 52, 96, 200)))
        items.append(bg)
        items.append(label)

        self._robot_item = self._scene.createItemGroup(items)
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
            sp = self.mapToScene(e.pos())
            if self._reloc_mode:
                # 重定位模式：记录拖拽起点
                self._drag_start_scene = sp
                self._clear_drag_arrow()
            else:
                # 普通模式：点击发送导航目标
                mx = sp.x() * self._res + self._origin[0]
                my = sp.y() * self._res + self._origin[1]
                self.clicked.emit(mx, my)
        super().mousePressEvent(e)

    def mouseMoveEvent(self, e):
        if self._reloc_mode and self._drag_start_scene:
            # 实时绘制拖拽箭头 + 更新机器人标记预览
            self._clear_drag_arrow()
            start = self._drag_start_scene
            end = self.mapToScene(e.pos())
            dx = end.x() - start.x()
            dy = end.y() - start.y()
            if abs(dx) > 2 or abs(dy) > 2:
                items = []
                # 箭头线
                pen = QPen(QColor("#e94560"), 3)
                line = self._scene.addLine(start.x(), start.y(), end.x(), end.y(), pen)
                items.append(line)
                # 起点圆
                dot = self._scene.addEllipse(start.x() - 4, start.y() - 4, 8, 8,
                                              QPen(QColor("#e94560"), 2), QBrush(QColor("#e94560")))
                items.append(dot)
                # 朝向文字
                yaw = math.atan2(dy, dx)
                yaw_deg = math.degrees(yaw)
                label = self._scene.addText(f"{yaw_deg:.0f}°", QFont("Arial", 10, QFont.Bold))
                label.setPos(end.x() + 5, end.y() - 15)
                label.setDefaultTextColor(QColor("#e94560"))
                items.append(label)
                self._drag_arrow = items

                # 实时更新机器人预览位置
                mx = start.x() * self._res + self._origin[0]
                my = start.y() * self._res + self._origin[1]
                self.update_robot(mx, my, yaw)
        super().mouseMoveEvent(e)

    def mouseReleaseEvent(self, e):
        if self._reloc_mode and self._drag_start_scene and e.button() == Qt.LeftButton:
            end = self.mapToScene(e.pos())
            start = self._drag_start_scene
            dx = end.x() - start.x()
            dy = end.y() - start.y()
            # 起点 = 机器人位置（地图坐标）
            mx = start.x() * self._res + self._origin[0]
            my = start.y() * self._res + self._origin[1]
            # 拖拽方向 = 机器人朝向
            yaw = math.atan2(dy, dx) if (abs(dx) > 2 or abs(dy) > 2) else 0.0
            self._drag_start_scene = None
            self._clear_drag_arrow()
            self.pose_clicked.emit(mx, my, yaw)
        super().mouseReleaseEvent(e)

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
        self._act_en_to_cn = {}
        self._act_cn_to_en = {}
        for a in sorted(ARM_ACTIONS.keys()):
            cn = ACTION_CN.get(a, a.replace("_", " "))
            self._act_en_to_cn[a] = cn
            self._act_cn_to_en[cn] = a
            self._action.addItem(cn)
        if action:
            cn = self._act_en_to_cn.get(action, action)
            idx = self._action.findText(cn)
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
            act_cn = self._action.currentText()
            self._act_val = self._act_cn_to_en.get(act_cn, "") if act_cn != "无" else ""
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
        self._g1_remote_url = os.environ.get("HONGTU_G1_BACKEND_URL", "").strip()
        self._g1_remote = None
        self._g1_remote_mode = bool(self._g1_remote_url)
        self._hand_pub_l = None
        self._hand_pub_r = None
        self._hand_ready = False
        self._arm_sdk_ready = False
        self._arm_sdk_pub = None
        self._arm_sdk_cmd = None
        self._arm_sdk_targets = [0.0]*G1_ARM_DOF
        self._arm_sdk_current_cmd_q = [0.0]*G1_ARM_DOF
        self._arm_low_state = None
        self._arm_low_state_lock = threading.Lock()
        self._arm_low_state_event = threading.Event()
        self._arm_sdk_target_lock = threading.Lock()
        self._arm_sdk_cmd_lock = threading.Lock()
        self._arm_sdk_stop_event = threading.Event()
        self._arm_sdk_publish_thread = None
        self._arm_sdk_active = False
        self._arm_sdk_weight = 0.0
        self._arm_sdk_publish_dt = 1.0 / 250.0
        self._arm_sdk_velocity_limit = 2.0
        self._arm_sdk_motion_start_error = 0.0
        self._arm_sdk_crc = CRC() if ARM_LOW_OK else None
        self._poses = []
        self._pose_timer = None
        self._teleop_active = False
        self._has_active_goal = False  # 是否有活跃的导航目标，防止启动时误触发
        self._waypoints = []  # [(name, x, y, yaw, action, speech)]
        self._tour_running = False
        self._last_pose = (0.0, 0.0, 0.0)
        self._map_data = None

        self._init_ui()
        self.log_message.connect(self._append_log)
        self._load_settings()
        self._status_ros.setText("ROS: 未启动")

    # ================================================================
    # UI 构建
    # ================================================================
    def _init_ui(self):
        self.setWindowTitle("G1 导航控制台")
        self.resize(1300, 850)
        self.setMinimumSize(900, 600)

        # 暗色主题 - 现代化配色
        self.setStyleSheet("""
            QMainWindow, QDialog { background: #1a1a2e; color: #e0e0e0; }
            QWidget { background: transparent; color: #e0e0e0; }
            QTabWidget::pane { border: 1px solid #3a3a5c; background: #16213e; border-radius: 4px; }
            QTabBar::tab {
                background: #1a1a2e; color: #8888aa; padding: 10px 22px;
                margin-right: 2px; border-top-left-radius: 4px; border-top-right-radius: 4px;
                font-size: 13px; font-weight: bold;
            }
            QTabBar::tab:selected { background: #0f3460; color: #e94560; }
            QTabBar::tab:hover:!selected { background: #2a2a4a; color: #ccc; }
            QGroupBox {
                border: 1px solid #3a3a5c; border-radius: 6px;
                margin-top: 14px; padding-top: 14px; color: #aaa;
                font-size: 11px; font-weight: bold;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; color: #e94560; }
            QPushButton {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #16213e,stop:1 #0f3460);
                color: #e0e0e0; border: 1px solid #3a3a5c; padding: 7px 16px;
                border-radius: 4px; font-size: 12px;
            }
            QPushButton:hover { background: #1a4080; border: 1px solid #e94560; }
            QPushButton:pressed { background: #0a2040; }
            QPushButton[class="success"] {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #27ae60,stop:1 #1e8449);
                border: 1px solid #2ecc71; color: #fff; font-weight: bold;
            }
            QPushButton[class="success"]:hover { background: #2ecc71; }
            QPushButton[class="danger"] {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #c0392b,stop:1 #96281b);
                border: 1px solid #e74c3c; color: #fff; font-weight: bold;
            }
            QPushButton[class="danger"]:hover { background: #e74c3c; }
            QLineEdit, QComboBox, QSpinBox {
                background: #1a1a2e; color: #e0e0e0;
                border: 1px solid #3a3a5c; padding: 5px; border-radius: 4px; font-size: 12px;
            }
            QLineEdit:focus { border: 1px solid #e94560; }
            QComboBox::drop-down { border: none; padding-right: 8px; }
            QComboBox QAbstractItemView { background: #1a1a2e; color: #e0e0e0; selection-background-color: #0f3460; }
            QLabel { color: #c0c0d0; font-size: 12px; }
            QPlainTextEdit, QListWidget {
                background: #0d1117; color: #c0c0d0;
                border: 1px solid #3a3a5c; border-radius: 4px; font-size: 12px;
            }
            QProgressBar { background: #1a1a2e; border: none; border-radius: 4px; text-align: center; color: #fff; font-size: 11px; }
            QProgressBar::chunk { background: qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 #e94560,stop:1 #0f3460); border-radius: 4px; }
            QSplitter::handle { background: #3a3a5c; width: 2px; }
            QCheckBox { color: #c0c0d0; spacing: 6px; }
            QCheckBox::indicator { width: 16px; height: 16px; }
            QStatusBar { background: #0f3460; color: #c0c0d0; font-size: 12px; border-top: 1px solid #3a3a5c; }
            QScrollArea { border: none; }
            QSlider::groove:horizontal { background: #3a3a5c; height: 6px; border-radius: 3px; }
            QSlider::handle:horizontal { background: #e94560; width: 14px; margin: -4px 0; border-radius: 7px; }
            QSlider::sub-page:horizontal { background: #0f3460; border-radius: 3px; }
        """)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)

        # 工具栏
        toolbar = QHBoxLayout()
        toolbar.addWidget(QLabel("G1 网卡:"))
        self._nav_net_if = QLineEdit(self.cfg.get("net_if", "eno1"))
        self._nav_net_if.setPlaceholderText("auto / eth0 / eno1")
        self._nav_net_if.setToolTip(
            "本体部署推荐填写 auto。手动连接时填写 G1 本体上的 DDS 网卡名，"
            "不是 PC 的网卡名；常见为 eth0，老版/PC 有线为 eno1。"
        )
        if self._g1_remote_mode:
            self._nav_net_if.setText(self._g1_remote_url)
            self._nav_net_if.setToolTip("远程后台模式：这里显示 G1 本体 HTTP 服务地址")
        self._nav_net_if.setFixedWidth(220 if self._g1_remote_mode else 96)
        toolbar.addWidget(self._nav_net_if)

        self._btn_g1 = QPushButton("连接 G1")
        self._btn_g1.setMinimumHeight(36)
        self._btn_g1.clicked.connect(self._on_g1_toggle)
        toolbar.addWidget(self._btn_g1)
        self._btn_nav_g1 = self._btn_g1

        self._g1_label = QLabel("G1: 未连接")
        self._g1_label.setStyleSheet("font-size: 13px; font-weight: bold; padding: 4px 10px; color: #888;")
        toolbar.addWidget(self._g1_label)
        self._nav_g1_label = self._g1_label

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
            "① 启动导航  →  ② 点击地图设置重定位  →  ③ 发送导航目标")
        self._step_label.setStyleSheet(
            "background: #16213e; color: #e94560; padding: 8px 14px; border-radius: 4px; "
            "font-size: 13px; font-weight: bold; border: 1px solid #3a3a5c;")
        main_layout.addWidget(self._step_label)

        # 标签页
        self._tabs = QTabWidget()
        self._pose_tab_index = -1
        self._tabs.addTab(self._build_nav_tab(), "📍 导航")
        self._tabs.addTab(self._build_teleop_tab(), "🎮 遥控")
        self._tabs.addTab(self._build_waypoint_tab(), "📍 航点")
        self._tabs.addTab(self._build_action_tab(), "🤖 动作")
        self._tabs.addTab(self._build_hand_tab(), "🖐 灵巧手")
        self._pose_tab_index = self._tabs.addTab(self._build_pose_tab(), "🎬 姿态")
        self._tabs.addTab(self._build_settings_tab(), "⚙ 设置")
        self._tabs.currentChanged.connect(self._on_main_tab_changed)
        main_layout.addWidget(self._tabs, 1)

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
        self._map_view.clicked.connect(self._on_map_nav)  # 点击地图发送导航目标
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
        self._btn_reloc = QPushButton("📌 点击地图重定位（ICP 自动对齐）")
        self._btn_reloc.clicked.connect(self._on_reloc_mode)
        rl.addWidget(self._btn_reloc)

        # 朝向控制 — 带指南针标注
        yaw_row = QHBoxLayout()
        yaw_row.addWidget(QLabel("朝向:"))
        self._reloc_yaw_slider = QSlider(Qt.Horizontal)
        self._reloc_yaw_slider.setRange(-180, 180)
        self._reloc_yaw_slider.setValue(0)
        self._reloc_yaw_slider.setTickInterval(45)
        self._reloc_yaw_slider.setTickPosition(QSlider.TicksBelow)
        self._reloc_yaw_label = QLabel("→ 0° (右)")
        self._reloc_yaw_slider.valueChanged.connect(self._update_yaw_label)
        yaw_row.addWidget(self._reloc_yaw_slider, 1)
        yaw_row.addWidget(self._reloc_yaw_label)
        rl.addLayout(yaw_row)

        # 指南针图例
        compass = QLabel("   0°→右   90°↓前   ±180°←后   -90°↑左")
        compass.setStyleSheet("color: #888; font-size: 11px; padding: 2px 4px;")
        rl.addWidget(compass)

        hint = QLabel("提示: 在地图上点击机器人位置，ICP 会自动对齐 ±3m 范围")
        hint.setStyleSheet("color: #e94560; font-size: 11px;")
        hint.setWordWrap(True)
        rl.addWidget(hint)

        # 手动输入
        rl2 = QHBoxLayout()
        rl2.addWidget(QLabel("X:"))
        self._reloc_x = QLineEdit("0.0")
        self._reloc_x.setFixedWidth(70)
        rl2.addWidget(self._reloc_x)
        rl2.addWidget(QLabel("Y:"))
        self._reloc_y = QLineEdit("0.0")
        self._reloc_y.setFixedWidth(70)
        rl2.addWidget(self._reloc_y)
        btn_reloc_go = QPushButton("重定位到此坐标")
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

        # G1 快捷控制
        grp = QGroupBox("G1 快捷控制")
        grp.setFont(title_font)
        gg = QVBoxLayout(grp)

        # 快速动作
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("手臂:"))
        quick_acts = [("挥手", "face wave"), ("鼓掌", "clap"), ("比心", "heart"),
                       ("握手", "shake hand"), ("拒绝", "reject")]
        for cn_name, an in quick_acts:
            btn = QPushButton(cn_name)
            btn.setMinimumWidth(64)
            btn.setMinimumHeight(28)
            btn.clicked.connect(lambda checked, n=an: self._g1_arm_action(n))
            row2.addWidget(btn)
        row2.addStretch()
        gg.addLayout(row2)

        # 快速灵巧手（集成模式）
        if HAND_OK:
            row_hand = QHBoxLayout()
            row_hand.addWidget(QLabel("手部:"))
            for hname in ["张开", "握拳", "OK", "点赞", "点按"]:
                btn = QPushButton(hname)
                btn.setMinimumWidth(58)
                btn.setMinimumHeight(28)
                btn.clicked.connect(lambda checked, n=hname: self._hand_set_preset("r", n))
                row_hand.addWidget(btn)
            row_hand.addStretch()
            gg.addLayout(row_hand)

        # 快速语音
        row3 = QHBoxLayout()
        row3.addWidget(QLabel("语音:"))
        self._nav_tts = QLineEdit()
        self._nav_tts.setPlaceholderText("输入播报文字…")
        self._nav_tts.setMinimumHeight(30)
        self._nav_tts.returnPressed.connect(self._on_nav_tts)
        row3.addWidget(self._nav_tts, 1)
        btn_say = QPushButton("播报")
        btn_say.setMinimumHeight(30)
        btn_say.setStyleSheet("""
            QPushButton { background: #e94560; color: #fff; font-weight: bold; border-radius: 4px; }
            QPushButton:hover { background: #ff6b81; }
        """)
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

        grid.addWidget(btn_fwd, 0, 2)
        grid.addWidget(btn_lat_left, 1, 0)
        grid.addWidget(btn_left, 1, 1)
        grid.addWidget(btn_stop, 1, 2)
        grid.addWidget(btn_right, 1, 3)
        grid.addWidget(btn_lat_right, 1, 4)
        grid.addWidget(btn_bwd, 2, 2)
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

        estop_hint = QLabel(
            "⚠ 急停 = 发送零速度指令（机器人原地站定，不会摔倒）\n"
            "   不会进入阻尼/零力矩模式，安全可靠")
        estop_hint.setStyleSheet("color: #888; font-size: 11px; padding: 4px 8px;")
        estop_hint.setWordWrap(True)
        layout.addWidget(estop_hint)

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
        outer = QVBoxLayout(tab)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(8)
        layout.setContentsMargins(8, 8, 8, 8)

        # FSM 模式
        grp = QGroupBox("FSM 模式切换")
        gl = QHBoxLayout(grp)
        for txt, fid in [("行走模式", 200), ("阻尼模式", 1), ("坐下", 3), ("站起", -1)]:
            btn = QPushButton(txt)
            if fid == -1:
                btn.clicked.connect(self._g1_stand)
            else:
                btn.clicked.connect(lambda checked, f=fid: self._g1_set_fsm(f))
            gl.addWidget(btn)
        gl.addStretch()
        layout.addWidget(grp)

        # 手臂动作
        grp = QGroupBox("手臂预设动作")
        gl = QHBoxLayout(grp)
        common = [("face wave", "挥手"), ("clap", "鼓掌"), ("hug", "拥抱"),
                   ("heart", "比心"), ("right hand up", "举手"), ("reject", "拒绝"),
                   ("shake hand", "握手"), ("x-ray", "展示"), ("high five", "击掌")]
        for aname, cn in common:
            btn = QPushButton(cn)
            btn.setMinimumWidth(72)
            btn.setMinimumHeight(32)
            btn.clicked.connect(lambda checked, n=aname: self._g1_arm_action(n))
            gl.addWidget(btn)
        gl.addStretch()
        layout.addWidget(grp)

        # 全部动作展开
        grp = QGroupBox("全部手臂动作")
        gl = QHBoxLayout(grp)
        action_scroll = QScrollArea()
        action_scroll.setWidgetResizable(True)
        sw = QWidget()
        fl = QHBoxLayout(sw)
        for aname in sorted(ARM_ACTIONS.keys()):
            cn = ACTION_CN.get(aname, aname.replace("_", " "))
            btn = QPushButton(cn)
            btn.setMinimumWidth(88)
            btn.setMinimumHeight(32)
            btn.clicked.connect(lambda checked, n=aname: self._g1_arm_action(n))
            fl.addWidget(btn)
        fl.addStretch()
        action_scroll.setWidget(sw)
        gl.addWidget(action_scroll)
        layout.addWidget(grp)

        # ---- 灵巧手集成 ----
        if HAND_OK:
            # 灵巧手状态
            grp = QGroupBox("灵巧手 DDS")
            gl = QHBoxLayout(grp)
            self._btn_hand_status = QLabel("未就绪" if not self._hand_ready else "就绪")
            self._btn_hand_status.setStyleSheet(
                "color: #888;" if not self._hand_ready else "color: #27ae60; font-weight: bold;")
            gl.addWidget(QLabel("状态:"))
            gl.addWidget(self._btn_hand_status)
            reinit_hand = QPushButton("重新初始化")
            reinit_hand.clicked.connect(lambda: self._init_hand_dds(
                self._nav_net_if.text().strip()))
            gl.addWidget(reinit_hand)
            gl.addStretch()
            layout.addWidget(grp)

            # 协同动作
            grp = QGroupBox("协同动作（手臂 + 灵巧手）")
            gl = QHBoxLayout(grp)
            for aname, (_, hand_preset) in COORDINATED_ACTIONS.items():
                cn = ACTION_CN.get(aname, aname)
                btn = QPushButton(cn)
                btn.setMinimumWidth(72)
                btn.setMinimumHeight(32)
                btn.clicked.connect(lambda checked, n=aname: self._g1_coordinated_action(n))
                gl.addWidget(btn)
            gl.addStretch()
            layout.addWidget(grp)

        # TTS 语音
        grp = QGroupBox("语音播报 (TTS)")
        gl = QVBoxLayout(grp)
        row = QHBoxLayout()
        self._tts_input = QLineEdit()
        self._tts_input.setPlaceholderText("输入播报文字…")
        self._tts_input.setMinimumHeight(30)
        self._tts_input.returnPressed.connect(self._on_tts)
        row.addWidget(self._tts_input, 1)
        btn_tts = QPushButton("播报")
        btn_tts.setMinimumHeight(30)
        btn_tts.setStyleSheet("""
            QPushButton { background: #e94560; color: #fff; font-weight: bold; border-radius: 4px; }
            QPushButton:hover { background: #ff6b81; }
        """)
        btn_tts.clicked.connect(self._on_tts)
        row.addWidget(btn_tts)
        gl.addLayout(row)

        # 预设语音
        phrases_row = QHBoxLayout()
        for phrase in ["欢迎参观", "请跟我来", "这是我们的展品", "谢谢大家",
                        "请注意安全", "正在前往下一个展品"]:
            btn = QPushButton(phrase)
            btn.setStyleSheet("""
                QPushButton { background: #16213e; border: 1px solid #3a3a5c; border-radius: 4px; }
                QPushButton:hover { background: #0f3460; border: 1px solid #e94560; }
            """)
            btn.clicked.connect(lambda checked, p=phrase: self._g1_speak(p))
            phrases_row.addWidget(btn)
        phrases_row.addStretch()
        gl.addLayout(phrases_row)
        layout.addWidget(grp)

        # LED + 音量
        grp = QGroupBox("LED & 音量")
        gl = QHBoxLayout(grp)
        gl.addWidget(QLabel("LED:"))
        led_colors = [
            ("红", 255, 0, 0, "#ff3333", "#fff"),
            ("绿", 0, 255, 0, "#33cc33", "#fff"),
            ("蓝", 0, 0, 255, "#3366ff", "#fff"),
            ("白", 255, 255, 255, "#ffffff", "#333"),
            ("关", 0, 0, 0, "#555555", "#aaa"),
        ]
        for txt, r, g, b, bg, fg in led_colors:
            btn = QPushButton(txt)
            btn.setFixedSize(48, 32)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background: {bg}; color: {fg};
                    border: 2px solid #3a3a5c; border-radius: 6px;
                    font-weight: bold; font-size: 12px;
                }}
                QPushButton:hover {{ border: 2px solid #e94560; background: {bg}; }}
                QPushButton:pressed {{ background: {bg}; border: 2px solid #fff; }}
            """)
            btn.clicked.connect(lambda checked, rr=r, gg=g, bb=b: self._g1_led(rr, gg, bb))
            gl.addWidget(btn)
        gl.addStretch()
        gl.addWidget(QLabel("音量:"))
        self._vol_slider = QSlider(Qt.Horizontal)
        self._vol_slider.setRange(0, 100)
        self._vol_slider.setValue(100)
        self._vol_slider.valueChanged.connect(self._g1_set_volume)
        gl.addWidget(self._vol_slider)
        layout.addWidget(grp)

        layout.addStretch()
        scroll.setWidget(content)
        outer.addWidget(scroll)
        return tab

    # ---- 灵巧手控制标签页（完整面板） ----
    def _build_hand_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setSpacing(6)

        # ── 工具栏 ──
        top = QHBoxLayout()
        top.addWidget(QLabel("控制:"))
        self._ht_lr_btn = QPushButton("右手")
        self._ht_lr_btn.setCheckable(True)
        self._ht_lr_btn.toggled.connect(
            lambda c: self._ht_lr_btn.setText("左手" if c else "右手"))
        top.addWidget(self._ht_lr_btn)

        self._ht_link_btn = QPushButton("双手同步: 关")
        self._ht_link_btn.setCheckable(True)
        self._ht_link_btn.toggled.connect(
            lambda c: self._ht_link_btn.setText(f"双手同步: {'开' if c else '关'}"))
        top.addWidget(self._ht_link_btn)

        top.addWidget(QLabel("  手势:"))
        for pname in ("张开", "握拳", "指向", "OK", "点赞", "三指捏", "半开", "点按"):
            btn = QPushButton(pname)
            btn.setStyleSheet("font-size:11px; padding:2px 8px;")
            btn.clicked.connect(lambda checked, n=pname: self._ht_preset(n))
            top.addWidget(btn)
        top.addStretch()

        self._ht_status = QLabel("DDS: 未就绪")
        self._ht_status.setStyleSheet("color:#f38ba8; font-weight:bold;")
        top.addWidget(self._ht_status)
        layout.addLayout(top)

        # ── 双手面板 ──
        hands = QHBoxLayout()
        hands.setSpacing(8)

        # 初始化 target 存储
        self._ht_targets = {"l": [500]*6, "r": [500]*6}
        self._ht_sliders = {"l": [], "r": []}   # [(slider, val_label, fb_label)]

        for side, side_name in (("l", "左手"), ("r", "右手")):
            grp = QGroupBox(side_name)
            gl = QVBoxLayout(grp)
            gl.setSpacing(2)
            gl.setContentsMargins(6, 14, 6, 6)
            rows = []
            for i, fname in enumerate(["小指","无名指","中指","食指","拇指屈","拇指旋"]):
                row = QHBoxLayout()
                row.setSpacing(3)
                lbl = QLabel(fname)
                lbl.setFixedWidth(36)
                lbl.setStyleSheet("font-size:10px; font-weight:bold;")
                row.addWidget(lbl)

                sld = QSlider(Qt.Horizontal)
                sld.setRange(0, 1000)
                sld.setValue(500)
                sld.valueChanged.connect(lambda v, s=side, idx=i: self._ht_slider(s, idx, v))
                row.addWidget(sld, 1)

                vl = QLabel("500")
                vl.setFixedWidth(30)
                vl.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
                vl.setStyleSheet("font-size:10px; font-family:monospace;")
                row.addWidget(vl)

                fb_lbl = QLabel("--")
                fb_lbl.setFixedWidth(30)
                fb_lbl.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
                fb_lbl.setStyleSheet("font-size:10px; color:#a6e3a1;")
                row.addWidget(fb_lbl)

                gl.addLayout(row)
                rows.append((sld, vl, fb_lbl))
            self._ht_sliders[side] = rows

            # 手部可视化（复用 HandCanvas）
            if HAS_HAND_WIDGETS:
                canvas = HandCanvas()
                canvas.setFixedHeight(200)
                gl.addWidget(canvas, 1)
                self._ht_canvas = getattr(self, '_ht_canvas', {})
                self._ht_canvas[side] = canvas

            grp.setMinimumWidth(280)
            hands.addWidget(grp)

        layout.addLayout(hands, 1)

        # ── 力传感器反馈 ──
        fb_grp = QGroupBox("力传感器反馈")
        fb_grid = QHBoxLayout(fb_grp)
        self._ht_force_lbls = {"l": [], "r": []}
        for side, sn in (("l", "左手"), ("r", "右手")):
            col = QVBoxLayout()
            col.addWidget(QLabel(sn))
            for fname in ["小指","无名指","中指","食指","拇指屈","拇指旋"]:
                lbl = QLabel(f"{fname}: --")
                lbl.setStyleSheet("font-size:10px;")
                col.addWidget(lbl)
                self._ht_force_lbls[side].append(lbl)
            fb_grid.addLayout(col)
        fb_grid.addStretch()
        layout.addWidget(fb_grp)

        # ── 急停 ──
        estop = QPushButton("🛑 紧急停止灵巧手")
        estop.setStyleSheet("font-size:14px; font-weight:bold; background:#c0392b; color:#fff; padding:8px;")
        estop.clicked.connect(self._ht_estop)
        layout.addWidget(estop)

        # ── 启动定时器（50ms 刷新 DDS 状态和 UI） ──
        self._ht_timer = QTimer()
        self._ht_timer.timeout.connect(self._ht_tick)
        self._ht_timer.start(50)

        return tab

    def _ht_slider(self, side, idx, value):
        """滑块拖动 → 更新 target → DDS 发布"""
        self._ht_targets[side][idx] = value
        # 更新本手数值标签
        sld, vl, _ = self._ht_sliders[side][idx]
        vl.setText(str(int(value)))
        # 联动模式
        if hasattr(self, '_ht_link_btn') and self._ht_link_btn.isChecked():
            other = "r" if side == "l" else "l"
            self._ht_targets[other][idx] = value
            osld, ovl, _ = self._ht_sliders[other][idx]
            osld.blockSignals(True)
            osld.setValue(int(value))
            osld.blockSignals(False)
            ovl.setText(str(int(value)))
        # 立即发布
        self._ht_publish(side)
        if hasattr(self, '_ht_link_btn') and self._ht_link_btn.isChecked():
            other = "r" if side == "l" else "l"
            self._ht_publish(other)

    def _ht_publish(self, side):
        """发布一侧手的 DDS 控制指令"""
        if not self._hand_ready:
            return
        pub = self._hand_pub_r if side == "r" else self._hand_pub_l
        if not pub:
            return
        try:
            cmd = get_inspire_hand_ctrl()
            cmd.mode = 0b0001
            cmd.angle_set = [int(v) for v in self._ht_targets[side]]
            pub.Write(cmd)
        except Exception:
            pass

    def _ht_preset(self, name):
        """在手势面板应用预设"""
        if name not in HAND_PRESETS:
            return
        vals = HAND_PRESETS[name]
        # 判断当前控制哪只手
        is_left = hasattr(self, '_ht_lr_btn') and self._ht_lr_btn.isChecked()
        if is_left:
            sides = ("l",)
        else:
            sides = ("r",)
        if hasattr(self, '_ht_link_btn') and self._ht_link_btn.isChecked():
            sides = ("l", "r")

        for side in sides:
            self._ht_targets[side] = list(vals)
            for i, (sld, vl, _) in enumerate(self._ht_sliders[side]):
                sld.blockSignals(True)
                sld.setValue(int(vals[i]))
                sld.blockSignals(False)
                vl.setText(str(int(vals[i])))
            self._ht_publish(side)
        self._log(f"[手势] {'左手' if is_left else '右手'} → {name}")

    def _ht_tick(self):
        """定时器刷新: 更新力反馈 UI + 可视化 + 发布当前 target"""
        if not self._hand_ready:
            if hasattr(self, '_ht_status'):
                self._ht_status.setText("DDS: 未就绪")
            return
        self._ht_status.setText("DDS: 就绪")
        self._ht_status.setStyleSheet("color:#a6e3a1; font-weight:bold;")

        # 刷新力反馈
        for side in ("l", "r"):
            st = self._hand_state.get(side, {})
            forces = st.get('force', [])
            angles = st.get('angle', [])
            for i in range(6):
                # 力值
                fv = int(forces[i]) if i < len(forces) else 0
                lbl = self._ht_force_lbls[side][i]
                lbl.setText(f"{['小指','无名指','中指','食指','拇指屈','拇指旋'][i]}: {max(0, fv)}")
                # 角度反馈
                if i < len(self._ht_sliders[side]):
                    _, _, fb_lbl = self._ht_sliders[side][i]
                    av = int(angles[i]) if i < len(angles) else 0
                    fb_lbl.setText(str(av))

            # 更新可视化
            if HAS_HAND_WIDGETS and hasattr(self, '_ht_canvas'):
                canvas = self._ht_canvas.get(side)
                if canvas and angles:
                    canvas.update_pos(angles)

        # 持续发布 target（确保 DDS 到手部通路畅通）
        for side in ("l", "r"):
            self._ht_publish(side)

    def _ht_estop(self):
        """灵巧手急停：双手全部张开"""
        open_vals = HAND_PRESETS["张开"]
        for side in ("l", "r"):
            self._ht_targets[side] = list(open_vals)
            for i, (sld, vl, _) in enumerate(self._ht_sliders[side]):
                sld.blockSignals(True)
                sld.setValue(open_vals[i])
                sld.blockSignals(False)
                vl.setText(str(open_vals[i]))
            self._ht_publish(side)
        self._log("[灵巧手] 急停 → 双手张开")

    # ================================================================
    # G1 手臂控制 (rt/arm_sdk, 冻结初始腰位)
    # ================================================================
    def _arm_lowstate_cb(self, msg):
        with self._arm_low_state_lock:
            self._arm_low_state = msg
        self._arm_low_state_event.set()

    def _arm_current_lowstate(self):
        with self._arm_low_state_lock:
            return self._arm_low_state

    def _arm_sdk_get_current_arm_q(self):
        state = self._arm_current_lowstate()
        if not state:
            return None
        return [float(state.motor_state[jid].q) for jid in G1_ARM_JOINT_IDS]

    def _arm_sdk_prepare_cmd_from_lowstate(self):
        """用当前 lowstate 填满整包 LowCmd，避免未写腰腿字段导致塌腰。"""
        state = self._arm_current_lowstate()
        if not state:
            return False
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = state.mode_machine
        arm_set = set(G1_ARM_JOINT_IDS)
        for joint in G1JointIndex:
            idx = int(joint)
            motor = cmd.motor_cmd[idx]
            motor.mode = 1
            motor.q = float(state.motor_state[idx].q)
            motor.dq = 0.0
            motor.tau = 0.0
            if idx in arm_set:
                if joint in WRIST_MOTORS:
                    motor.kp = 40.0
                    motor.kd = 1.5
                else:
                    motor.kp = 80.0
                    motor.kd = 3.0
            elif joint in WEAK_MOTORS:
                motor.kp = 80.0
                motor.kd = 3.0
            else:
                motor.kp = 300.0
                motor.kd = 3.0
        cmd.motor_cmd[G1_ARM_SDK_ENABLE_JOINT].q = float(self._arm_sdk_weight)
        with self._arm_sdk_cmd_lock:
            self._arm_sdk_cmd = cmd
        with self._arm_sdk_target_lock:
            self._arm_sdk_current_cmd_q = [float(state.motor_state[jid].q) for jid in G1_ARM_JOINT_IDS]
        return True

    def _arm_sdk_clip_targets(self, targets):
        max_step = max(self._arm_sdk_velocity_limit * self._arm_sdk_publish_dt, 1e-6)
        next_q = []
        for cur, tgt in zip(self._arm_sdk_current_cmd_q, targets):
            delta = float(tgt) - float(cur)
            if delta > max_step:
                delta = max_step
            elif delta < -max_step:
                delta = -max_step
            next_q.append(float(cur) + delta)
        self._arm_sdk_current_cmd_q = next_q
        return next_q

    def _arm_sdk_publish_once(self):
        if not self._arm_sdk_ready or not self._arm_sdk_pub:
            return
        try:
            with self._arm_sdk_cmd_lock:
                cmd = self._arm_sdk_cmd
            if cmd is None:
                return
            with self._arm_sdk_target_lock:
                targets = list(self._arm_sdk_targets)
                weight = float(self._arm_sdk_weight)
                clipped = self._arm_sdk_clip_targets(targets)
            with self._arm_sdk_cmd_lock:
                cmd.motor_cmd[G1_ARM_SDK_ENABLE_JOINT].q = weight
                for i, jid in enumerate(G1_ARM_JOINT_IDS):
                    motor = cmd.motor_cmd[jid]
                    motor.q = float(clipped[i])
                    motor.dq = 0.0
                    motor.tau = 0.0
                    motor.kp = G1_ARM_PARAMS[i]["kp"]
                    motor.kd = G1_ARM_PARAMS[i]["kd"]
                cmd.crc = self._arm_sdk_crc.Crc(cmd) if self._arm_sdk_crc else CRC().Crc(cmd)
                self._arm_sdk_pub.Write(cmd)
        except Exception as e:
            self._log(f"[姿态] arm_sdk 发布异常: {e}")

    def _arm_sdk_publish_loop(self):
        while not self._arm_sdk_stop_event.is_set():
            start = time.time()
            self._arm_sdk_publish_once()
            sleep_time = self._arm_sdk_publish_dt - (time.time() - start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _arm_sdk_start_publish_loop(self):
        if self._arm_sdk_publish_thread and self._arm_sdk_publish_thread.is_alive():
            return
        self._arm_sdk_stop_event.clear()
        self._arm_sdk_publish_thread = threading.Thread(
            target=self._arm_sdk_publish_loop,
            name="g1-nav-arm-sdk",
            daemon=True,
        )
        self._arm_sdk_publish_thread.start()

    def _arm_sdk_stop_publish_loop(self):
        self._arm_sdk_stop_event.set()
        th = self._arm_sdk_publish_thread
        if th and th.is_alive():
            th.join(timeout=1.0)
        self._arm_sdk_publish_thread = None

    def _arm_sdk_ramp_weight(self, start, end, seconds=1.0):
        steps = max(1, int(seconds / 0.02))
        for step in range(steps + 1):
            ratio = step / steps
            with self._arm_sdk_target_lock:
                self._arm_sdk_weight = float(start + (end - start) * ratio)
            self._arm_sdk_publish_once()
            time.sleep(0.02)

    def _arm_sdk_release(self):
        if self._g1_remote_mode and self._g1_remote:
            try:
                self._g1_remote.arm_release()
            except Exception as e:
                self._log(f"[姿态] 远程释放 arm_sdk 失败: {e}")
            self._arm_sdk_active = False
            with self._arm_sdk_target_lock:
                self._arm_sdk_weight = 0.0
            return
        try:
            if self._arm_sdk_ready and self._arm_sdk_cmd is not None:
                self._arm_sdk_ramp_weight(float(self._arm_sdk_weight), 0.0, seconds=1.0)
        finally:
            self._arm_sdk_stop_publish_loop()
            self._arm_sdk_active = False
            with self._arm_sdk_target_lock:
                self._arm_sdk_weight = 0.0

    def _ensure_arm_sdk_released(self, reason=""):
        """Release low-level arm control before leaving pose mode or running actions."""
        if not getattr(self, "_arm_sdk_active", False):
            return
        if reason:
            self._log(f"[安全] {reason}，自动释放 arm_sdk")
        self._arm_sdk_release()
        if hasattr(self, "_pose_arm_activate"):
            self._pose_arm_activate.setText("⚠ 激活臂控")
            self._pose_arm_activate.setStyleSheet("font-weight:bold; color:#fff; background:#e67e22;")
        if hasattr(self, "_pose_arm_progress"):
            self._pose_arm_progress.setValue(0)

    def _on_main_tab_changed(self, idx):
        if idx != getattr(self, "_pose_tab_index", -1):
            self._ensure_arm_sdk_released("离开姿态臂控界面")

    def _arm_sdk_set_joints(self, angles):
        """设置目标关节角度"""
        target = self._normalize_arm_pose(angles)
        with self._arm_sdk_target_lock:
            current = list(self._arm_sdk_current_cmd_q)
            self._arm_sdk_motion_start_error = max(
                [abs(float(t) - float(c)) for t, c in zip(target, current)] + [0.0]
            )
            for i in range(min(len(target), G1_ARM_DOF)):
                lo = G1_ARM_PARAMS[i]["min"]
                hi = G1_ARM_PARAMS[i]["max"]
                self._arm_sdk_targets[i] = max(lo, min(hi, float(target[i])))
            remote_targets = list(self._arm_sdk_targets)
        if self._g1_remote_mode and self._g1_remote and self._arm_sdk_active:
            try:
                self._g1_remote.arm_joints(remote_targets)
            except Exception as e:
                self._log(f"[姿态] 远程关节设置失败: {e}")

    def _normalize_arm_pose(self, angles):
        """兼容旧版 7 关节右臂姿态；新版使用 14 关节双臂姿态。"""
        values = [float(v) for v in angles] if angles else []
        if len(values) >= G1_ARM_DOF:
            return values[:G1_ARM_DOF]
        with self._arm_sdk_target_lock:
            target = list(self._arm_sdk_targets)
        if len(values) == 7 and G1_ARM_DOF == 14:
            target[7:14] = values
            return target
        for i, value in enumerate(values):
            if i < G1_ARM_DOF:
                target[i] = value
        return target

    def _arm_motion_progress(self):
        with self._arm_sdk_target_lock:
            targets = list(self._arm_sdk_targets)
            current = list(self._arm_sdk_current_cmd_q)
            start_error = float(self._arm_sdk_motion_start_error)
        error = max([abs(float(t) - float(c)) for t, c in zip(targets, current)] + [0.0])
        if error < 0.01:
            return 100
        if start_error <= 0.01:
            return 0
        return max(0, min(100, int((1.0 - error / start_error) * 100)))

    # ================================================================
    # 姿态录制 / 回放系统
    # ================================================================
    def _pose_capture(self):
        """捕获当前手臂+手的姿态 """
        arm = list(self._arm_sdk_targets)
        # 手：从 hand state 或当前 target 读取
        hand_r = self._ht_targets.get("r", [500]*6) if hasattr(self, '_ht_targets') else [500]*6
        return {"arm": arm, "hand_r": hand_r}

    def _pose_save(self, name):
        name = name.strip()
        if not name:
            self._log("[姿态] 名称不能为空")
            return
        pose = self._pose_capture()
        pose["name"] = name
        # 覆盖同名
        for i, p in enumerate(self._poses):
            if p["name"] == name:
                self._poses[i] = pose
                self._log(f"[姿态] 已更新: {name}")
                self._pose_refresh_list()
                return
        self._poses.append(pose)
        self._log(f"[姿态] 已保存: {name}")
        self._pose_refresh_list()

    def _pose_execute(self, idx):
        if idx < 0 or idx >= len(self._poses):
            return
        pose = self._poses[idx]
        name = pose.get("name", f"姿态{idx}")
        # 必须在激活状态下执行
        if not self._arm_sdk_active:
            self._log("[姿态] 请先激活低阶控制再执行")
            return
        # 1. 设置手臂关节
        self._arm_sdk_set_joints(pose.get("arm", [0.0]*G1_ARM_DOF))
        self._log(f"[姿态] 手臂: {name}")
        # 2. 设置手部
        hand = pose.get("hand_r", [500]*6)
        if hasattr(self, '_ht_targets'):
            self._ht_targets["r"] = list(hand)
            self._ht_publish("r")
            # 更新右手滑块 UI
            if hasattr(self, '_ht_sliders') and "r" in self._ht_sliders:
                for i, (sld, vl, _) in enumerate(self._ht_sliders["r"]):
                    if i < len(hand):
                        sld.blockSignals(True)
                        sld.setValue(int(hand[i]))
                        sld.blockSignals(False)
                        vl.setText(str(int(hand[i])))
            self._log(f"[姿态] 右手: {name}")
        self._log(f"[姿态] 执行: {name}")

    def _pose_delete(self, idx):
        if 0 <= idx < len(self._poses):
            name = self._poses[idx].get("name", "")
            self._poses.pop(idx)
            self._log(f"[姿态] 已删除: {name}")
            self._pose_refresh_list()

    def _pose_refresh_list(self):
        """刷新姿态列表 UI"""
        if not hasattr(self, '_pose_list'):
            return
        self._pose_list.clear()
        for i, p in enumerate(self._poses):
            arm_vals = p.get("arm", [])
            arm_str = f"{len(arm_vals)} 关节"
            hand_str = ", ".join(str(h) for h in p.get("hand_r", []))
            self._pose_list.addItem(f"{i+1}. {p.get('name', '未命名')}  臂:[{arm_str}]  手:[{hand_str}]")

    def _pose_export(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "导出姿态", os.path.expanduser("~/g1_poses.json"), "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self._poses, f, ensure_ascii=False, indent=2)
            self._log(f"[姿态] 已导出 {len(self._poses)} 个: {os.path.basename(path)}")
        except Exception as e:
            QMessageBox.warning(self, "导出失败", str(e))

    def _pose_import(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "导入姿态", os.path.expanduser("~"), "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, list):
                QMessageBox.warning(self, "格式错误", "需要 JSON 数组")
                return
            self._poses = data
            self._pose_refresh_list()
            self._log(f"[姿态] 已导入 {len(data)} 个姿态")
        except Exception as e:
            QMessageBox.warning(self, "导入失败", str(e))

    # ---- 姿态编辑标签页 ----
    def _build_pose_tab(self):
        tab = QWidget()
        outer = QVBoxLayout(tab)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(6)
        layout.setContentsMargins(8, 8, 8, 8)

        # ── 手臂关节控制 ──
        grp = QGroupBox("双臂关节 (arm_sdk 低阶控制)")
        gl = QVBoxLayout(grp)
        gl.setSpacing(6)

        self._pose_arm_sliders = []  # (slider, val_label, min, max)
        arms_row = QHBoxLayout()
        arms_row.setSpacing(8)

        for side_name, start_idx in (("左臂", 0), ("右臂", 7)):
            arm_grp = QGroupBox(side_name)
            arm_layout = QVBoxLayout(arm_grp)
            arm_layout.setSpacing(4)
            arm_layout.setContentsMargins(8, 16, 8, 8)
            for offset in range(7):
                i = start_idx + offset
                jname = G1_ARM_JOINT_NAMES[i].replace("左", "").replace("右", "")
                row = QHBoxLayout()
                row.setSpacing(6)
                lbl = QLabel(jname)
                lbl.setMinimumWidth(64)
                lbl.setStyleSheet("font-size:11px;")
                row.addWidget(lbl)

                lo, hi = G1_ARM_PARAMS[i]["min"], G1_ARM_PARAMS[i]["max"]
                sld = QSlider(Qt.Horizontal)
                sld.setRange(int(lo*100), int(hi*100))
                sld.setValue(0)
                sld.setMinimumWidth(180)
                sld.valueChanged.connect(lambda v, idx=i: self._pose_arm_slider(idx, v/100.0))
                row.addWidget(sld, 1)

                vl = QLabel("0.00")
                vl.setMinimumWidth(44)
                vl.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
                vl.setStyleSheet("font-size:11px; font-family:monospace;")
                row.addWidget(vl)
                arm_layout.addLayout(row)
                self._pose_arm_sliders.append((sld, vl, lo, hi))
            arms_row.addWidget(arm_grp, 1)

        gl.addLayout(arms_row)

        # 工具按钮
        tool_row = QHBoxLayout()
        self._pose_arm_activate = QPushButton("⚠ 激活臂控")
        self._pose_arm_activate.setMinimumHeight(34)
        self._pose_arm_activate.setStyleSheet("font-weight:bold; color:#fff; background:#e67e22;")
        self._pose_arm_activate.clicked.connect(self._pose_arm_activate_toggle)
        tool_row.addWidget(self._pose_arm_activate)
        btn_read = QPushButton("读取当前姿态")
        btn_read.setMinimumHeight(34)
        btn_read.clicked.connect(self._pose_arm_read_current)
        tool_row.addWidget(btn_read)
        btn_zero = QPushButton("归零")
        btn_zero.setMinimumHeight(34)
        btn_zero.clicked.connect(self._pose_arm_zero)
        tool_row.addWidget(btn_zero)
        tool_row.addStretch()

        self._pose_arm_status = QLabel("臂控: 未激活")
        self._pose_arm_status.setMinimumWidth(100)
        self._pose_arm_status.setStyleSheet("color:#f38ba8; font-size:11px;")
        tool_row.addWidget(self._pose_arm_status)
        gl.addLayout(tool_row)

        progress_row = QHBoxLayout()
        progress_row.addWidget(QLabel("操作进度:"))
        self._pose_arm_progress = QProgressBar()
        self._pose_arm_progress.setRange(0, 100)
        self._pose_arm_progress.setValue(0)
        self._pose_arm_progress.setMinimumHeight(20)
        progress_row.addWidget(self._pose_arm_progress, 1)
        gl.addLayout(progress_row)
        layout.addWidget(grp)

        # ── 灵巧手手势 ──
        grp2 = QGroupBox("灵巧手手势（配合当前姿态）")
        g2l = QVBoxLayout(grp2)
        preset_row = QHBoxLayout()
        for pname in ("张开", "握拳", "指向", "OK", "点赞", "三指捏", "半开", "点按"):
            btn = QPushButton(pname)
            btn.setStyleSheet("font-size:11px; padding:2px 8px;")
            btn.clicked.connect(lambda checked, n=pname: self._pose_hand_preset(n))
            preset_row.addWidget(btn)
        preset_row.addStretch()
        g2l.addLayout(preset_row)

        # 当前手部状态简示
        self._pose_hand_status = QLabel("右手: 500 500 500 500 500 500")
        self._pose_hand_status.setStyleSheet("font-size:10px; color:#a6e3a1;")
        g2l.addWidget(self._pose_hand_status)
        layout.addWidget(grp2)

        # ── 姿态录制 ──
        grp3 = QGroupBox("姿态录制 / 回放")
        g3l = QVBoxLayout(grp3)

        save_row = QHBoxLayout()
        save_row.addWidget(QLabel("名称:"))
        self._pose_name_input = QLineEdit()
        self._pose_name_input.setPlaceholderText("输入姿态名称…")
        save_row.addWidget(self._pose_name_input, 1)
        btn_save = QPushButton("保存姿态")
        btn_save.clicked.connect(lambda: self._pose_save(self._pose_name_input.text()))
        save_row.addWidget(btn_save)
        g3l.addLayout(save_row)

        # 姿态列表
        self._pose_list = QListWidget()
        self._pose_list.itemDoubleClicked.connect(lambda: self._pose_execute(self._pose_list.currentRow()))
        g3l.addWidget(self._pose_list, 1)

        list_btn_row = QHBoxLayout()
        btn_play = QPushButton("▶ 执行")
        btn_play.clicked.connect(lambda: self._pose_execute(self._pose_list.currentRow()))
        list_btn_row.addWidget(btn_play)
        btn_del = QPushButton("删除")
        btn_del.clicked.connect(lambda: self._pose_delete(self._pose_list.currentRow()))
        list_btn_row.addWidget(btn_del)
        btn_export = QPushButton("导出 JSON")
        btn_export.clicked.connect(self._pose_export)
        list_btn_row.addWidget(btn_export)
        btn_import = QPushButton("导入 JSON")
        btn_import.clicked.connect(self._pose_import)
        list_btn_row.addWidget(btn_import)
        list_btn_row.addStretch()
        g3l.addLayout(list_btn_row)
        layout.addWidget(grp3, 1)

        # ── 急停 ──
        estop = QPushButton("🛑 紧急停止（释放 arm_sdk + 手部保持）")
        estop.setStyleSheet("font-size:14px; font-weight:bold; background:#c0392b; color:#fff; padding:8px;")
        estop.clicked.connect(self._pose_estop)
        layout.addWidget(estop)

        # 初始化姿态列表
        self._poses = []
        self._pose_refresh_list()

        # 创建定时器（但不启动，等待用户点击"激活 arm_sdk 控制"）
        self._arm_sdk_active = False
        self._pose_timer = QTimer()
        self._pose_timer.timeout.connect(self._pose_tick)
        self._pose_timer.start(50)

        scroll.setWidget(content)
        outer.addWidget(scroll)
        return tab

    def _pose_arm_slider(self, idx, val_rad):
        """手臂滑块拖动 → 设置关节角度 → 发布 arm_sdk"""
        if idx < len(G1_ARM_JOINT_IDS) and idx < len(self._arm_sdk_targets):
            with self._arm_sdk_target_lock:
                current = list(self._arm_sdk_current_cmd_q)
                self._arm_sdk_targets[idx] = val_rad
                self._arm_sdk_motion_start_error = max(
                    [abs(float(t) - float(c)) for t, c in zip(self._arm_sdk_targets, current)] + [0.0]
                )
                remote_targets = list(self._arm_sdk_targets)
            self._pose_arm_sliders[idx][1].setText(f"{val_rad:.2f}")
            if self._g1_remote_mode and self._g1_remote and self._arm_sdk_active:
                try:
                    self._g1_remote.arm_joints(remote_targets)
                except Exception as e:
                    self._log(f"[姿态] 远程滑块发送失败: {e}")

    def _pose_arm_read_current(self):
        """从 lowstate 读取当前关节角度到滑块"""
        if self._g1_remote_mode and self._g1_remote:
            try:
                values = self._g1_remote.arm_current().get("data", {}).get("joints", [])
            except Exception as e:
                self._log(f"[姿态] 远程读取失败: {e}")
                return
        else:
            state = self._arm_current_lowstate()
            if not state:
                self._log("[姿态] 无 lowstate 数据")
                return
            values = [float(state.motor_state[jid].q) for jid in G1_ARM_JOINT_IDS]
        for i, val in enumerate(values[:G1_ARM_DOF]):
            with self._arm_sdk_target_lock:
                if i < len(self._arm_sdk_targets):
                    self._arm_sdk_targets[i] = val
                    self._arm_sdk_current_cmd_q[i] = val
                self._arm_sdk_motion_start_error = 0.0
            if i < len(self._pose_arm_sliders):
                sld, vl, lo, hi = self._pose_arm_sliders[i]
                sld.blockSignals(True)
                sld.setValue(int(val * 100))
                sld.blockSignals(False)
                vl.setText(f"{val:.2f}")
        self._log("[姿态] 已读取当前关节角度")

    def _pose_arm_activate_toggle(self):
        """切换 arm_sdk 激活状态（xr 风格全关节初始化，更新双臂）"""
        if not self._arm_sdk_ready:
            self._log("[姿态] arm_sdk 未就绪（G1 未连接）")
            return
        if self._g1_remote_mode and self._g1_remote:
            if not self._arm_sdk_active:
                try:
                    current = self._g1_remote.arm_activate().get("data", {}).get("joints", [])
                except Exception as e:
                    self._log(f"[姿态] 远程激活失败: {e}")
                    return
                if not current:
                    self._log("[姿态] 远程未返回关节角，禁止激活")
                    return
                with self._arm_sdk_target_lock:
                    self._arm_sdk_targets = list(current[:G1_ARM_DOF])
                    self._arm_sdk_current_cmd_q = list(current[:G1_ARM_DOF])
                    self._arm_sdk_motion_start_error = 0.0
                    self._arm_sdk_weight = 1.0
                for i, val in enumerate(current[:G1_ARM_DOF]):
                    if i < len(self._pose_arm_sliders):
                        sld, vl, lo, hi = self._pose_arm_sliders[i]
                        sld.blockSignals(True)
                        sld.setValue(int(float(val) * 100))
                        sld.blockSignals(False)
                        vl.setText(f"{float(val):.2f}")
                self._arm_sdk_active = True
                self._pose_timer.start(50)
                self._pose_arm_activate.setText("■ 停用臂控")
                self._pose_arm_activate.setStyleSheet("font-weight:bold; color:#fff; background:#c0392b;")
                self._log("[姿态] 远程臂控已激活")
            else:
                self._arm_sdk_release()
                if hasattr(self, '_pose_arm_progress'):
                    self._pose_arm_progress.setValue(0)
                self._pose_timer.start(50)
                self._pose_arm_activate.setText("⚠ 激活臂控")
                self._pose_arm_activate.setStyleSheet("font-weight:bold; color:#fff; background:#e67e22;")
                self._log("[姿态] 远程臂控已停用")
            return
        if not self._arm_sdk_active:
            if not self._arm_low_state_event.wait(timeout=2.0):
                self._log("[姿态] 未收到 rt/lowstate，禁止激活臂控")
                return
            current = self._arm_sdk_get_current_arm_q()
            if current is None:
                self._log("[姿态] 无 lowstate 数据，禁止激活臂控")
                return
            with self._arm_sdk_target_lock:
                self._arm_sdk_targets = list(current)
                self._arm_sdk_current_cmd_q = list(current)
                self._arm_sdk_motion_start_error = 0.0
                self._arm_sdk_weight = 0.0
            for i, val in enumerate(current):
                if i < len(self._pose_arm_sliders):
                    sld, vl, lo, hi = self._pose_arm_sliders[i]
                    sld.blockSignals(True)
                    sld.setValue(int(val * 100))
                    sld.blockSignals(False)
                    vl.setText(f"{val:.2f}")
            if not self._arm_sdk_prepare_cmd_from_lowstate():
                self._log("[姿态] LowCmd 初始化失败，禁止激活臂控")
                return
            self._arm_sdk_active = True
            self._arm_sdk_start_publish_loop()
            self._arm_sdk_ramp_weight(0.0, 1.0, seconds=1.0)
            self._pose_timer.start(50)
            self._pose_arm_activate.setText("■ 停用臂控")
            self._pose_arm_activate.setStyleSheet("font-weight:bold; color:#fff; background:#c0392b;")
            self._log("[姿态] 臂控已激活（xr 风格全关节初始化，双臂 14 关节）")
        else:
            self._arm_sdk_release()
            if hasattr(self, '_pose_arm_progress'):
                self._pose_arm_progress.setValue(0)
            self._pose_timer.start(50)
            self._pose_arm_activate.setText("⚠ 激活臂控")
            self._pose_arm_activate.setStyleSheet("font-weight:bold; color:#fff; background:#e67e22;")
            self._log("[姿态] 臂控已停用，arm_sdk weight 已释放")

    def _pose_arm_zero(self):
        """手臂归零"""
        self._arm_sdk_set_joints([0.0]*G1_ARM_DOF)
        for i, (sld, vl, lo, hi) in enumerate(self._pose_arm_sliders):
            sld.blockSignals(True)
            sld.setValue(0)
            sld.blockSignals(False)
            vl.setText("0.00")
        if hasattr(self, '_pose_arm_progress'):
            self._pose_arm_progress.setValue(0)
        self._log("[姿态] 手臂归零")

    def _pose_hand_preset(self, name):
        """在姿态编辑中应用手部预设"""
        if name not in HAND_PRESETS:
            return
        vals = HAND_PRESETS[name]
        if hasattr(self, '_ht_targets'):
            self._ht_targets["r"] = list(vals)
            self._ht_publish("r")
        status = f"右手: {' '.join(str(v) for v in vals)}"
        self._pose_hand_status.setText(status)
        # 也更新手 tab 的滑块
        if hasattr(self, '_ht_sliders') and "r" in self._ht_sliders:
            for i, (sld, vl, _) in enumerate(self._ht_sliders["r"]):
                if i < len(vals):
                    sld.blockSignals(True)
                    sld.setValue(int(vals[i]))
                    sld.blockSignals(False)
                    vl.setText(str(int(vals[i])))
        self._log(f"[姿态] 手: {name}")

    def _pose_tick(self):
        """定时器：更新臂控状态；DDS 发布由 250Hz 后台线程负责。"""
        if self._arm_sdk_ready and self._arm_sdk_active:
            self._pose_arm_status.setText("臂控: 运行中")
            self._pose_arm_status.setStyleSheet("color:#a6e3a1; font-size:10px;")
            if hasattr(self, '_pose_arm_progress'):
                self._pose_arm_progress.setValue(self._arm_motion_progress())
        else:
            self._pose_arm_status.setText("臂控: 未激活")
            self._pose_arm_status.setStyleSheet("color:#f38ba8; font-size:10px;")
            if hasattr(self, '_pose_arm_progress'):
                self._pose_arm_progress.setValue(0)

    def _pose_estop(self):
        """急停：停止低阶控制"""
        self._arm_sdk_release()
        if hasattr(self, '_pose_arm_activate'):
            self._pose_arm_activate.setText("⚠ 激活臂控")
            self._pose_arm_activate.setStyleSheet("font-weight:bold; color:#fff; background:#e67e22;")
        if hasattr(self, '_pose_arm_progress'):
            self._pose_arm_progress.setValue(0)
        self._log("[姿态] 急停（arm_sdk 已平滑释放）")

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
            f"<p style='color:#888'>项目路径: {os.path.join(APP_ROOT, 'g1_nav_panel')}</p>"
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
        self._ros_worker.reloc_done.connect(self._on_reloc_done)
        self._ros_worker._pcd_path = self._edit_pcd.text().strip()
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

        self._start_ros()

        # 保存配置
        self.cfg["map_yaml"] = map_yaml
        self.cfg["pcd_path"] = pcd_path
        save_config(self.cfg)
        # 更新 ROS worker 的 PCD 路径（用于重定位服务）
        if self._ros_worker:
            self._ros_worker._pcd_path = pcd_path

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
            self._arm_sdk_release()
            self._g1_ready = False
            self._arm_sdk_ready = False
            self._arm_sdk_pub = None
            self._arm_sdk_cmd = None
            self._btn_g1.setText("连接 G1")
            self._g1_label.setText("G1: 未连接")
            self._g1_label.setStyleSheet("font-size: 13px; font-weight: bold; padding: 4px 10px; color: #888;")
            self._status_g1.setText("G1: 未连接")
            return
        if self._g1_remote_mode:
            if not REMOTE_G1_OK:
                QMessageBox.warning(self, "远程客户端不可用", "g1_remote_client.py 未加载")
                return
            url = self._g1_remote_url
            if self._nav_net_if.text().strip().startswith("http"):
                url = self._nav_net_if.text().strip()
                self._g1_remote_url = url
            try:
                self._log(f"[G1远程] 连接后台服务: {url}")
                self._g1_remote = G1RemoteClient(url, timeout=3.0)
                status = self._g1_remote.connect().get("data", {})
                self._g1_ready = True
                self._arm_sdk_ready = bool(status.get("arm_ready", True))
                self._hand_ready = bool(status.get("hand_ready", False))
                self._btn_g1.setText("断开 G1")
                self._g1_label.setText("G1: 远程已连接")
                self._g1_label.setStyleSheet("font-size: 13px; font-weight: bold; padding: 4px 10px; color: #27ae60;")
                self._status_g1.setText("G1: 远程已连接")
                self._log("[G1远程] 连接成功，本地 GUI 中文输入保持 PC 输入法")
            except Exception as e:
                QMessageBox.warning(self, "G1 远程连接失败", str(e))
                self._log(f"[G1远程] 连接失败: {e}")
            return
        if not G1_OK:
            QMessageBox.warning(self, "SDK 不可用", "unitree_sdk2py 未安装")
            return

        net_if = self._nav_net_if.text().strip()
        try:
            self._log(f"[G1] 初始化 DDS (net_if={net_if})…")
            init_unitree_channel(net_if)
            self._log("[G1] LocoClient…")
            self._g1_loco = LocoClient()
            self._g1_loco.SetTimeout(10.0)
            self._g1_loco.Init()
            self._g1_loco.Start()
            self._log("[G1] LocoClient 就绪")
            global _g1_loco_ref
            _g1_loco_ref = self._g1_loco

            self._log("[G1] ArmActionClient…")
            self._g1_arm = G1ArmActionClient()
            self._g1_arm.SetTimeout(10.0)
            self._g1_arm.Init()
            self._log("[G1] ArmActionClient 就绪")

            self._log("[G1] AudioClient…")
            self._g1_audio = G1AudioClient()
            self._g1_audio.SetTimeout(10.0)
            self._g1_audio.Init()
            self._g1_audio.SetVolume(100)
            self._log("[G1] AudioClient 就绪")
            # 修补 SDK bug：原代码 self.tts_index += self.tts_index 永远是 0
            import types
            _real_tts_index = [1]
            def _fixed_tts(client_self, text, speaker_id):
                _real_tts_index[0] += 1
                p = {"index": _real_tts_index[0], "text": text, "speaker_id": speaker_id}
                code, data = client_self._Call(1001, json.dumps(p))
                return code
            self._g1_audio.TtsMaker = types.MethodType(_fixed_tts, self._g1_audio)

            # ---- arm_sdk 初始化：命令包等待激活时从 lowstate 完整构造 ----
            self._arm_sdk_ready = False
            self._arm_sdk_targets = [0.0]*G1_ARM_DOF
            self._arm_sdk_current_cmd_q = [0.0]*G1_ARM_DOF
            with self._arm_low_state_lock:
                self._arm_low_state = None
            self._arm_low_state_event.clear()
            self._arm_sdk_pub = None
            self._arm_sdk_cmd = None
            self._arm_sdk_stop_event.set()
            self._arm_sdk_publish_thread = None
            self._arm_sdk_active = False
            self._arm_sdk_weight = 0.0
            if ARM_LOW_OK:
                try:
                    self._arm_sdk_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
                    self._arm_sdk_pub.Init()
                    self._lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
                    self._lowstate_sub.Init(self._arm_lowstate_cb, 10)
                    self._arm_sdk_ready = True
                    self._log("[G1] arm_sdk 就绪（激活时从 lowstate 完整初始化）")
                except Exception as e:
                    self._log(f"[G1] arm_sdk 失败: {e}")

            # ---- 灵巧手 DDS 初始化（不阻塞 G1 连接） ----
            self._init_hand_dds(net_if)

            self._g1_ready = True
            self._btn_g1.setText("断开 G1")
            self._g1_label.setText("G1: 已连接")
            self._g1_label.setStyleSheet("font-size: 13px; font-weight: bold; padding: 4px 10px; color: #27ae60;")
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
        self._has_active_goal = False
        if self._tour_running:
            self._tour_next_step()

    def _on_nav_cmd_vel(self, vx, vy, wz):
        """将 move_base 的 cmd_vel 转发给 G1（仅在有活跃目标时）"""
        if not self._g1_ready or not self._g1_loco:
            return
        if not self._has_active_goal:
            return  # 没有导航目标时不转发，防止启动时误触发
        try:
            if self._g1_remote_mode and self._g1_remote:
                if abs(vx) < 0.001 and abs(vy) < 0.001 and abs(wz) < 0.001:
                    self._g1_remote.stop()
                else:
                    self._g1_remote.move(vx, vy, wz, continuous=True)
                return
            if abs(vx) < 0.001 and abs(vy) < 0.001 and abs(wz) < 0.001:
                self._g1_loco.StopMove()
            else:
                self._g1_loco.Move(vx, vy, wz, continous_move=True)
        except Exception:
            pass

    # ---- 重定位 ----
    _reloc_mode = False

    def _update_yaw_label(self, v):
        """更新朝向标签，显示指南针方向"""
        if v >= -23 and v <= 23:
            self._reloc_yaw_label.setText(f"→ {v}° (右)")
        elif v > 23 and v <= 68:
            self._reloc_yaw_label.setText(f"↘ {v}° (右下)")
        elif v > 68 and v <= 113:
            self._reloc_yaw_label.setText(f"↓ {v}° (前)")
        elif v > 113 and v <= 158:
            self._reloc_yaw_label.setText(f"↙ {v}° (左下)")
        elif v > 158 or v < -158:
            self._reloc_yaw_label.setText(f"← {v}° (后)")
        elif v >= -158 and v < -113:
            self._reloc_yaw_label.setText(f"↖ {v}° (左上)")
        elif v >= -113 and v < -68:
            self._reloc_yaw_label.setText(f"↑ {v}° (左)")
        elif v >= -68 and v < -23:
            self._reloc_yaw_label.setText(f"↗ {v}° (右上)")

    def _on_reloc_mode(self):
        self._reloc_mode = not self._reloc_mode
        self._map_view.set_reloc_mode(self._reloc_mode)
        if self._reloc_mode:
            self._btn_reloc.setText("✅ 重定位模式 — 拖拽设置位置+朝向")
            self._btn_reloc.setStyleSheet("background: #27ae60; color: #fff; font-weight: bold;")
            self._step_label.setText("📍 重定位: 在地图上按住拖拽 → 起点=位置，方向=朝向，松开自动 ICP 对齐")
            self._step_label.setStyleSheet(
                "background: #16213e; color: #27ae60; padding: 8px 14px; border-radius: 4px; "
                "font-size: 13px; font-weight: bold; border: 1px solid #3a3a5c;")
            self._map_view.pose_clicked.connect(self._on_drag_reloc)
        else:
            self._btn_reloc.setText("📌 点击地图重定位（ICP 自动对齐）")
            self._btn_reloc.setStyleSheet("")
            self._step_label.setText("① 启动导航  →  ② 拖拽地图设置重定位  →  ③ 发送导航目标")
            self._step_label.setStyleSheet(
                "background: #16213e; color: #e94560; padding: 8px 14px; border-radius: 4px; "
                "font-size: 13px; font-weight: bold; border: 1px solid #3a3a5c;")
            try:
                self._map_view.pose_clicked.disconnect(self._on_drag_reloc)
            except Exception:
                pass

    def _on_map_nav(self, mx, my):
        """点击地图普通模式 → 发送导航目标"""
        _, _, yaw = self._last_pose
        if self._ros_worker:
            self._has_active_goal = True
            self._ros_worker.send_goal(mx, my, yaw)
            self._log(f"[导航] 点击目标: ({mx:.2f}, {my:.2f})")

    def _on_drag_reloc(self, mx, my, yaw):
        """拖拽重定位：起点=位置，方向=朝向，自动调 ICP"""
        self._reloc_x.setText(f"{mx:.2f}")
        self._reloc_y.setText(f"{my:.2f}")
        self._reloc_yaw_slider.setValue(int(math.degrees(yaw)))
        if self._ros_worker and self._ros_worker._reloc_srv:
            self._ros_worker.request_reloc.emit(mx, my, yaw)
            self._log(f"[重定位] 拖拽 ICP: ({mx:.2f}, {my:.2f}) 朝向: {math.degrees(yaw):.0f}°")
        elif self._ros_worker:
            self._ros_worker.send_init_pose(mx, my, yaw)
            self._log(f"[重定位] 设置位姿: ({mx:.2f}, {my:.2f}) {math.degrees(yaw):.0f}°")

    def _on_map_click_reloc(self, mx, my):
        """点击地图重定位（非拖拽模式降级）"""
        yaw = math.radians(self._reloc_yaw_slider.value())
        self._reloc_x.setText(f"{mx:.2f}")
        self._reloc_y.setText(f"{my:.2f}")
        # 调用 ICP 重定位服务
        if self._ros_worker and self._ros_worker._reloc_srv:
            self._ros_worker.request_reloc.emit(mx, my, yaw)
            self._log(f"[重定位] ICP 重定位: ({mx:.2f}, {my:.2f}) 朝向: {self._reloc_yaw_slider.value()}°")
        else:
            # 降级：只发初始位姿
            self._on_reloc_set()
            self._log(f"[重定位] 设置位姿: ({mx:.2f}, {my:.2f})")

    def _on_reloc_done(self, success, msg):
        if success:
            self._log(f"[重定位] ✓ 成功: {msg}")
        else:
            self._log(f"[重定位] ✗ 失败: {msg}")

    def _on_reloc_set(self):
        """手动输入坐标 → 调用 ICP 重定位服务"""
        try:
            x = float(self._reloc_x.text())
            y = float(self._reloc_y.text())
            yaw = math.radians(self._reloc_yaw_slider.value())
            # 优先调用 ICP 服务（自动对齐点云）
            if self._ros_worker and self._ros_worker._reloc_srv:
                self._ros_worker.request_reloc.emit(x, y, yaw)
                self._log(f"[重定位] ICP 重定位: ({x:.2f}, {y:.2f}) 朝向: {self._reloc_yaw_slider.value()}°")
            elif self._ros_worker:
                # 降级：只发初始位姿
                self._ros_worker.send_init_pose(x, y, yaw)
                self._log(f"[重定位] 设置位姿（无 ICP）: ({x:.2f}, {y:.2f}) {self._reloc_yaw_slider.value()}°")
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
        self._arm_sdk_release()
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

    def _g1_set_fsm(self, fsm_id):
        self._ensure_arm_sdk_released("切换 FSM 模式前")
        if self._g1_remote_mode and self._g1_remote:
            try:
                self._g1_remote.fsm(fsm_id)
            except Exception as e:
                self._log(f"[G1远程] FSM 切换失败: {e}")
            return
        self._g1_api(lambda: self._g1_loco.SetFsmId(fsm_id))

    def _g1_set_volume(self, value):
        if self._g1_remote_mode and self._g1_remote:
            try:
                self._g1_remote.volume(value)
            except Exception as e:
                self._log(f"[G1远程] 音量设置失败: {e}")
            return
        self._g1_api(lambda: self._g1_audio.SetVolume(value))

    def _g1_led(self, r, g, b):
        if self._g1_remote_mode and self._g1_remote:
            try:
                self._g1_remote.led(r, g, b)
            except Exception as e:
                self._log(f"[G1远程] LED 设置失败: {e}")
            return
        self._g1_api(lambda: self._g1_audio.LedControl(r, g, b))

    def _g1_fsm_api(self, func):
        self._ensure_arm_sdk_released("切换 FSM 模式前")
        self._g1_api(func)

    def _g1_move(self, vx, vy, wz):
        if not self._g1_ready:
            now = time.time()
            last = getattr(self, "_last_move_not_ready_log", 0.0)
            if now - last > 2.0:
                self._last_move_not_ready_log = now
                self._log("[遥控] G1 未连接，点击“连接 G1”或等待本体模式自动连接")
            return
        try:
            if self._g1_remote_mode and self._g1_remote:
                self._g1_remote.move(vx, vy, wz, continuous=False)
                return
            self._g1_loco.Move(vx, vy, wz, continous_move=False)
        except Exception as e:
            self._log(f"[遥控] 发送移动失败: {e}")

    def _g1_arm_action(self, name):
        self._ensure_arm_sdk_released("执行预设动作前")
        if self._g1_remote_mode and self._g1_remote:
            try:
                self._g1_remote.action(name=name)
                self._log(f"[动作] {ACTION_CN.get(name, name)}")
            except Exception as e:
                self._log(f"[动作] 远程执行失败: {e}")
            return
        for aname_str, aid_val in ARM_ACTIONS.items():
            if aname_str == name:
                self._g1_api(lambda i=aid_val: self._g1_arm.ExecuteAction(i))
                self._log(f"[动作] {aname_str}")
                break

    def _g1_stand(self):
        self._ensure_arm_sdk_released("切换站立/行走前")
        if self._g1_remote_mode and self._g1_remote:
            try:
                self._g1_remote.stand()
                self._log("[G1远程] 行走模式")
            except Exception as e:
                self._log(f"[G1远程] 站起失败: {e}")
            return
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
                if self._g1_remote_mode and self._g1_remote:
                    self._g1_remote.speak(text.strip(), 0)
                    return
                self._g1_audio.TtsMaker(text.strip(), 0) #0女声 1男声
            except Exception as e:
                self._log(f"[语音] 失败: {e}")

    # ---- 灵巧手控制 ----
    def _init_hand_dds(self, net_if):
        """初始化灵巧手 DDS publisher（G1 连接成功后调用）"""
        if not HAND_OK:
            self._log("[灵巧手] SDK 不可用，跳过初始化")
            return
        self._hand_ready = False
        try:
            self._hand_pub_r = ChannelPublisher("rt/inspire_hand/ctrl/r", _hand_ctrl_type)
            self._hand_pub_r.Init()
            self._hand_pub_l = ChannelPublisher("rt/inspire_hand/ctrl/l", _hand_ctrl_type)
            self._hand_pub_l.Init()

            # 状态订阅（用于力反馈）
            self._hand_state = {"l": {}, "r": {}}
            for _lr in ("l", "r"):
                _sub = ChannelSubscriber(f"rt/inspire_hand/state/{_lr}", _hand_state_type)
                _sub.Init(lambda msg, lr=_lr: self._hand_state_update(lr, msg), 10)

            self._hand_ready = True
            self._log("[灵巧手] DDS 已就绪（含状态订阅）")
            self._hand_targets = {"l": [500]*6, "r": [500]*6}
            if hasattr(self, '_btn_hand_status'):
                self._btn_hand_status.setText("就绪")
                self._btn_hand_status.setStyleSheet("color: #27ae60; font-weight: bold;")
        except Exception as e:
            self._log(f"[灵巧手] DDS 初始化失败: {e}")
            self._hand_pub_r = None
            self._hand_pub_l = None

    def _hand_state_update(self, lr, msg):
        """灵巧手状态回调"""
        self._hand_state[lr] = {
            'angle': list(msg.angle_act) if hasattr(msg, 'angle_act') else [],
            'force': list(msg.force_act) if hasattr(msg, 'force_act') else [],
            'pos': list(msg.pos_act) if hasattr(msg, 'pos_act') else [],
        }

    def _hand_set_preset(self, lr, preset_name):
        """通过 DDS 发送灵巧手预设手势 (lr='l'或'r')"""
        if self._g1_remote_mode and self._g1_remote:
            try:
                self._g1_remote.hand_preset(lr, preset_name)
                self._log(f"[灵巧手] {'右手' if lr == 'r' else '左手'} → {preset_name}")
            except Exception as e:
                self._log(f"[灵巧手] 远程控制失败: {e}")
            return
        if not self._hand_ready:
            return
        if preset_name not in HAND_PRESETS:
            self._log(f"[灵巧手] 未知手势: {preset_name}")
            return
        pub = self._hand_pub_r if lr == "r" else self._hand_pub_l
        if not pub:
            return
        try:
            cmd = get_inspire_hand_ctrl()
            cmd.mode = 0b0001  # 角度模式
            cmd.angle_set = [int(v) for v in HAND_PRESETS[preset_name]]
            pub.Write(cmd)
            self._log(f"[灵巧手] {'右手' if lr == 'r' else '左手'} → {preset_name}")
        except Exception as e:
            self._log(f"[灵巧手] 控制失败: {e}")

    def _hand_set_angles(self, lr, angles):
        """通过 DDS 发送自定义角度 (6 个 0-1000)"""
        if self._g1_remote_mode and self._g1_remote:
            try:
                self._g1_remote.hand_angles(lr, angles)
            except Exception:
                pass
            return
        if not self._hand_ready or len(angles) != 6:
            return
        pub = self._hand_pub_r if lr == "r" else self._hand_pub_l
        if not pub:
            return
        try:
            cmd = get_inspire_hand_ctrl()
            cmd.mode = 0b0001
            cmd.angle_set = [int(v) for v in angles]
            pub.Write(cmd)
        except Exception:
            pass

    def _g1_coordinated_action(self, action_name):
        """同时执行 G1 手臂动作 + 灵巧手手势（右手）"""
        self._ensure_arm_sdk_released("执行协同动作前")
        if action_name not in COORDINATED_ACTIONS:
            self._g1_arm_action(action_name)
            return

        arm_act, hand_preset = COORDINATED_ACTIONS[action_name]
        cn = ACTION_CN.get(arm_act, arm_act)

        if self._g1_remote_mode and self._g1_remote:
            try:
                self._g1_remote.coordinated_action(action_name)
                self._log(f"[协同] {cn} + {hand_preset}")
            except Exception as e:
                self._log(f"[协同] 远程执行失败: {e}")
            return

        # 1. 执行 G1 手臂动作（与 FSM 模式无关，阻尼模式下也可执行）
        if self._g1_ready and self._g1_arm:
            found = False
            for aname_str, aid_val in ARM_ACTIONS.items():
                if aname_str == arm_act:
                    try:
                        self._g1_arm.ExecuteAction(aid_val)
                        self._log(f"[协同] 手臂: {cn}")
                        found = True
                    except Exception as e:
                        self._log(f"[协同] 手臂执行失败: {e}")
                        self._log(f"[协同] 提示: 请检查 G1 是否已站立（点击「行走模式」），"
                                  "吊装调试时也可尝试先切到行走模式再切回阻尼")
                    break
            if not found:
                self._log(f"[协同] 未找到手臂动作: {arm_act}")
        else:
            self._log("[协同] G1 未连接或手臂不可用")

        # 2. 灵巧手手势
        self._hand_set_preset("r", hand_preset)
        self._log(f"[协同] {cn} + {hand_preset}")

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
            self._has_active_goal = True
            self._ros_worker.send_goal(x, y, yaw)
        self._log(f"[导航] 发送目标: ({x:.2f}, {y:.2f})")

    def _wp_save(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "保存航点", os.path.expanduser("~/g1_waypoints.json"),
            "JSON (*.json);;文本 (*.txt);;所有文件 (*)")
        if not path:
            return
        if not path.endswith(('.json', '.txt')):
            path += '.json'
        data = [{"name": n, "x": x, "y": y, "yaw": yaw, "action": a, "speech": s}
                for n, x, y, yaw, a, s in self._waypoints]
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            self._log(f"[航点] 已保存 {len(data)} 个航点: {os.path.basename(path)}")
        except Exception as e:
            QMessageBox.warning(self, "保存失败", str(e))

    def _wp_load(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "加载航点", os.path.expanduser("~"),
            "航点文件 (*.json *.txt);;JSON (*.json);;文本 (*.txt);;所有文件 (*)")
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                content = f.read().strip()
            if not content:
                QMessageBox.warning(self, "加载失败", f"文件为空: {os.path.basename(path)}")
                return
            data = json.loads(content)
            if not isinstance(data, list):
                QMessageBox.warning(self, "加载失败", "文件格式错误：应为 JSON 数组")
                return
            self._waypoints = [(d["name"], d["x"], d["y"], d["yaw"],
                                d.get("action", ""), d.get("speech", "")) for d in data]
            self._wp_refresh()
            self._log(f"[航点] 加载了 {len(self._waypoints)} 个航点: {os.path.basename(path)}")
        except json.JSONDecodeError as e:
            QMessageBox.warning(self, "加载失败",
                                f"JSON 格式错误: {e}\n\n请确认文件是由「保存航点」生成的")
        except KeyError as e:
            QMessageBox.warning(self, "加载失败", f"缺少字段: {e}")
        except Exception as e:
            QMessageBox.warning(self, "加载失败", str(e))

    def _wp_refresh(self):
        self._wp_list.clear()
        for i, (name, x, y, yaw, action, speech) in enumerate(self._waypoints):
            cn_act = ACTION_CN.get(action, action) if action else ""
            txt = f"{i+1}. {name}  ({x:.1f}, {y:.1f}) {math.degrees(yaw):.0f}°"
            if cn_act:
                txt += f"  [{cn_act}]"
            if speech:
                txt += f"  「{speech}」"
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
            self._has_active_goal = True
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
                self._g1_coordinated_action(action)
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
            self._has_active_goal = True
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
        if self._g1_remote_mode:
            self._edit_net.setText(self._g1_remote_url)
            self._nav_net_if.setText(self._g1_remote_url)
        elif self.cfg.get("net_if"):
            self._edit_net.setText(self.cfg["net_if"])
            self._nav_net_if.setText(self.cfg["net_if"])

    def closeEvent(self, e):
        # 紧急停止机器人
        self._emergency_stop()
        # 发送多次停止命令确保生效
        for _ in range(3):
            try:
                self._g1_loco.StopMove()
            except Exception:
                pass
        self._on_nav_stop()
        # 停止灵巧手定时器
        if hasattr(self, '_ht_timer') and self._ht_timer:
            self._ht_timer.stop()
        # 停止姿态定时器
        if hasattr(self, '_pose_timer') and self._pose_timer:
            self._pose_timer.stop()
        self._arm_sdk_release()
        if self._ros_worker:
            self._ros_worker.stop()
            self._ros_worker.wait(2000)
        # 保存配置
        if not self._g1_remote_mode:
            self.cfg["net_if"] = self._nav_net_if.text().strip() or self._edit_net.text().strip()
        self.cfg["map_yaml"] = self._edit_map.text()
        self.cfg["pcd_path"] = self._edit_pcd.text()
        save_config(self.cfg)
        super().closeEvent(e)


# ============================================================
# 安全退出：进程异常退出时紧急停止机器人
# ============================================================
_g1_loco_ref = None  # 全局引用

def _global_emergency_stop():
    """atexit/signal 安全网：无论进程如何退出都发送停止命令"""
    global _g1_loco_ref
    if _g1_loco_ref:
        for _ in range(5):
            try:
                _g1_loco_ref.StopMove()
            except Exception:
                pass
        try:
            _g1_loco_ref.Move(0, 0, 0, continous_move=False)
        except Exception:
            pass

atexit.register(_global_emergency_stop)
# 不注册 SIGTERM handler — Qt 自己管理信号，手动注册会导致段错误


# ============================================================
# 入口
# ============================================================
def main():
    print("[启动] main.py 已进入", flush=True)
    display = os.environ.get("DISPLAY")
    wayland = os.environ.get("WAYLAND_DISPLAY")
    qpa = os.environ.get("QT_QPA_PLATFORM")
    print(f"[启动] DISPLAY={display!r} WAYLAND_DISPLAY={wayland!r} QT_QPA_PLATFORM={qpa!r}", flush=True)
    if sys.platform.startswith("linux") and not display and not wayland and not qpa:
        print("[启动] 未检测到图形显示环境，无法打开 PyQt GUI。请在桌面终端运行，或设置 DISPLAY。", flush=True)
        return 2
    print("[启动] 初始化 QApplication…", flush=True)
    app = QApplication(sys.argv)
    print("[启动] QApplication 完成，开始创建主窗口…", flush=True)
    app.setStyle("Fusion")
    app.setApplicationName("G1 导航控制台")
    win = MainWindow()
    forced_net_if = os.environ.get("HONGTU_G1_NET_IF", "").strip()
    if os.environ.get("HONGTU_FORCE_NET_IF", "").strip() == "1" and forced_net_if:
        try:
            win._nav_net_if.setText(forced_net_if)
            win._edit_net.setText(forced_net_if)
            win.cfg["net_if"] = forced_net_if
        except Exception:
            pass
    print("[启动] 主窗口创建完成，显示窗口…", flush=True)
    win.show()
    print("[启动] 窗口已显示，进入事件循环", flush=True)
    if (
        os.environ.get("HONGTU_ROBOT_MODE", "").strip() == "1"
        or os.environ.get("HONGTU_REMOTE_AUTO_CONNECT", "").strip() == "1"
    ):
        print("[启动] 自动连接 G1", flush=True)
        QTimer.singleShot(1000, win._on_g1_toggle)
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
