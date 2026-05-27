#!/usr/bin/env python3
"""
G1 机器人综合控制面板
=====================
功能：
  1. 键盘/按钮遥控（前后左右横移旋转）
  2. 2D 地图显示（栅格地图 + 机器人位置 + 航点）
  3. 航点管理（记录/编辑/保存/加载/单点导航/多点巡航）
  4. 预设动作（挥手、鼓掌、TTS 语音播报、LED 控制等）
  5. FSM 模式切换（行走/阻尼/坐下/站起）
  6. 状态显示（位姿、连接状态、导航状态）

启动方式（在 ROS 环境 + G1 网络下）：
  python3 g1_control_panel.py

不影响现有 g1_control.py 和导航系统。
"""

import json
import math
import os
import queue
import socket
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

BASE = os.path.dirname(os.path.abspath(__file__))
for p in [os.path.join(BASE, "unitree_sdk2_python"),
          os.path.join(BASE, "inspire_hand")]:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

###############################################################################
# ROS 导入
###############################################################################
try:
    import rospy
    from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path
    from actionlib_msgs.msg import GoalStatusArray
    import tf.transformations as tf_trans
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

###############################################################################
# 宇树 G1 SDK 导入
###############################################################################
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
    from unitree_sdk2py.g1.arm.g1_arm_action_client import action_map as ARM_ACTION_MAP
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
    G1_AVAILABLE = True
except ImportError:
    G1_AVAILABLE = False
    ARM_ACTION_MAP = {}

try:
    from g1_arm_hand_coordinator import ArmHandTrajectoryPlayer, TrajectoryStopped, load_action_names
    COORD_AVAILABLE = True
    COORD_ACTION_NAMES = load_action_names()
except Exception as e:
    COORD_AVAILABLE = False
    COORD_ACTION_NAMES = []
    COORD_IMPORT_ERROR = e

###############################################################################
# 常量
###############################################################################
DEFAULT_NET_IF = "eno1"
CMD_VEL_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/slam_odom"
MAP_TOPIC = "/map"
GOAL_TOPIC = "/move_base_simple/goal"
INITIALPOSE_TOPIC = "/initialpose"
MOVE_BASE_STATUS_TOPIC = "/move_base/status"
GLOBAL_PATH_TOPIC = "/move_base/GlobalPlanner/plan"

MAX_ANG_VEL = 0.8  # 最大角速度 (rad/s)

# 导航状态文字
NAV_STATUS_TEXT = {
    0: "等待中", 1: "导航中", 2: "被抢占",
    3: "已到达", 4: "失败", 5: "被拒绝",
    6: "抢占中", 7: "召回中", 8: "已召回", 9: "丢失",
}


###############################################################################
# 机器人后端
###############################################################################
class RobotBackend:
    """管理 ROS 通信和 G1 SDK 通信，线程安全"""

    def __init__(self):
        # ROS 状态
        self.ros_connected = False
        self._ros_inited = False
        self._pub_cmd_vel = None
        self._pub_goal = None
        self._pub_initialpose = None

        # 共享状态（由 ROS 回调更新）
        self.robot_pose = (0.0, 0.0, 0.0)
        self.map_data = None
        self.nav_status = "未知"
        self.goal_status = -1
        self.global_path = None

        # G1 状态
        self.g1_connected = False
        self.loco = None
        self.arm = None
        self.audio = None
        self.coordinator = None
        self.coord_status = "未启动"
        self._g1_net_if = DEFAULT_NET_IF

        # 导航标记
        self._cancel_flag = threading.Event()
        self._nav_busy = False

    # ---- ROS ----

    def ros_connect(self):
        if not ROS_AVAILABLE:
            return False
        if self.ros_connected:
            return True
        try:
            if not self._ros_inited:
                rospy.init_node("g1_control_panel", anonymous=True)
                self._ros_inited = True

            self._pub_cmd_vel = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
            self._pub_goal = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=10)
            self._pub_initialpose = rospy.Publisher(INITIALPOSE_TOPIC, PoseWithCovarianceStamped, queue_size=10)

            rospy.Subscriber(ODOM_TOPIC, Odometry, self._odom_cb)
            rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self._map_cb)
            rospy.Subscriber(MOVE_BASE_STATUS_TOPIC, GoalStatusArray, self._nav_status_cb)
            rospy.Subscriber(GLOBAL_PATH_TOPIC, Path, self._path_cb)

            self.ros_connected = True
            return True
        except Exception as e:
            print(f"[ROS] 连接失败: {e}")
            return False

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _map_cb(self, msg):
        self.map_data = msg

    def _nav_status_cb(self, msg):
        if msg.status_list:
            s = msg.status_list[-1].status
            self.nav_status = NAV_STATUS_TEXT.get(s, f"未知({s})")
            self.goal_status = s
        else:
            self.nav_status = "空闲"
            self.goal_status = -1

    def _path_cb(self, msg):
        self.global_path = msg

    def send_cmd_vel(self, vx, vy, wz):
        if not self.ros_connected:
            return
        try:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
            self._pub_cmd_vel.publish(t)
        except Exception:
            pass

    def stop_robot(self):
        self.send_cmd_vel(0, 0, 0)

    def send_nav_goal(self, x, y, yaw):
        if not self.ros_connected:
            return False
        try:
            g = PoseStamped()
            g.header.frame_id = "map"
            g.header.stamp = rospy.Time.now()
            g.pose.position.x, g.pose.position.y = float(x), float(y)
            q = tf_trans.quaternion_from_euler(0, 0, float(yaw))
            g.pose.orientation.x, g.pose.orientation.y = q[0], q[1]
            g.pose.orientation.z, g.pose.orientation.w = q[2], q[3]
            self._pub_goal.publish(g)
            self.goal_status = 0
            return True
        except Exception as e:
            print(f"[ROS] 发送导航目标失败: {e}")
            return False

    def set_initial_pose(self, x, y, yaw):
        if not self.ros_connected:
            return
        try:
            p = PoseWithCovarianceStamped()
            p.header.frame_id = "map"
            p.header.stamp = rospy.Time.now()
            p.pose.pose.position.x, p.pose.pose.position.y = float(x), float(y)
            q = tf_trans.quaternion_from_euler(0, 0, float(yaw))
            p.pose.pose.orientation.x, p.pose.pose.orientation.y = q[0], q[1]
            p.pose.pose.orientation.z, p.pose.pose.orientation.w = q[2], q[3]
            self._pub_initialpose.publish(p)
        except Exception:
            pass

    def is_nav_active(self):
        return self.goal_status in (0, 1, 6, 7, 9)

    def is_nav_success(self):
        return self.goal_status == 3

    def cancel_nav(self):
        self._cancel_flag.set()
        self.stop_robot()

    # ---- G1 SDK ----

    def g1_connect(self):
        if not G1_AVAILABLE:
            return False
        if self.g1_connected:
            return True
        try:
            ChannelFactoryInitialize(0, self._g1_net_if)

            self.loco = LocoClient()
            self.loco.SetTimeout(10.0)
            self.loco.Init()
            self.loco.Start()

            self.arm = G1ArmActionClient()
            self.arm.SetTimeout(10.0)
            self.arm.Init()

            self.audio = AudioClient()
            self.audio.SetTimeout(10.0)
            self.audio.Init()

            if COORD_AVAILABLE:
                try:
                    self.coordinator = ArmHandTrajectoryPlayer()
                    self.coordinator.init()
                    self.coord_status = "就绪"
                except Exception as e:
                    self.coordinator = None
                    self.coord_status = f"不可用: {e}"
            else:
                self.coord_status = "不可用"

            self.g1_connected = True
            return True
        except Exception as e:
            print(f"[G1] 连接失败: {e}")
            return False

    def g1_disconnect(self):
        if self.coordinator:
            self.coordinator.stop()
        self.g1_connected = False

    def g1_move(self, vx, vy, wz):
        if not self.g1_connected:
            return
        try:
            self.loco.Move(vx, vy, wz, continous_move=True)
        except Exception:
            pass

    def g1_stop(self):
        self.g1_move(0, 0, 0)

    def g1_action(self, action_id):
        if not self.g1_connected:
            return
        try:
            self.arm.ExecuteAction(action_id)
        except Exception:
            pass

    def is_coord_running(self):
        return bool(self.coordinator and self.coordinator.is_running())

    def g1_play_coord_action(self, action_name):
        if not self.g1_connected or not self.coordinator:
            return False, "请先连接 G1，并确认协同模块可用"
        if self.is_coord_running():
            return False, "已有协同动作正在执行"
        try:
            # 保持高层运控 FSM=200，由机器人原运控继续负责站立/行走平衡。
            self.loco.Start()
            self.coord_status = f"执行: {action_name}"

            def done_cb(error):
                if isinstance(error, TrajectoryStopped):
                    self.coord_status = "已停止"
                elif error:
                    self.coord_status = f"失败: {error}"
                else:
                    self.coord_status = "完成"

            self.coordinator.play_builtin(action_name, done_cb=done_cb)
            return True, "已开始"
        except Exception as e:
            self.coord_status = f"失败: {e}"
            return False, str(e)

    def g1_play_coord_file(self, path):
        if not self.g1_connected or not self.coordinator:
            return False, "请先连接 G1，并确认协同模块可用"
        if self.is_coord_running():
            return False, "已有协同动作正在执行"
        try:
            self.loco.Start()
            self.coord_status = f"执行: {os.path.basename(path)}"

            def done_cb(error):
                if isinstance(error, TrajectoryStopped):
                    self.coord_status = "已停止"
                elif error:
                    self.coord_status = f"失败: {error}"
                else:
                    self.coord_status = "完成"

            self.coordinator.play_file(path, done_cb=done_cb)
            return True, "已开始"
        except Exception as e:
            self.coord_status = f"失败: {e}"
            return False, str(e)

    def g1_stop_coord(self):
        if self.coordinator:
            self.coordinator.stop()
            self.coord_status = "停止中"

    def g1_speak(self, text):
        if not self.g1_connected or not text.strip():
            return
        try:
            self.audio.TtsMaker(text.strip(), 0)
        except Exception:
            pass

    def g1_led(self, r, g, b):
        if not self.g1_connected:
            return
        try:
            self.audio.LedControl(r, g, b)
        except Exception:
            pass

    def g1_volume(self, v):
        if not self.g1_connected:
            return
        try:
            self.audio.SetVolume(int(v))
        except Exception:
            pass

    def g1_set_fsm(self, fsm_id):
        if not self.g1_connected:
            return
        try:
            if fsm_id != 200:
                self.g1_stop_coord()
            self.loco.SetFsmId(fsm_id)
        except Exception:
            pass

    def g1_sit(self):
        if not self.g1_connected:
            return
        try:
            self.loco.Sit()
        except Exception:
            pass

    def g1_stand_up(self):
        if not self.g1_connected:
            return
        try:
            self.loco.Lie2StandUp()
            time.sleep(1.5)
            self.loco.Start()
        except Exception:
            pass

    # ---- 多点巡航 ----

    def execute_tour(self, waypoints, progress_cb=None):
        """
        按序导航到每个航点，可选执行动作 + TTS
        waypoints: [(name, x, y, yaw, action_str, speech), ...]
        progress_cb(idx, total, msg)
        返回 (完成数, 总数)
        """
        self._cancel_flag.clear()
        self._nav_busy = True
        done = 0
        total = len(waypoints)

        for i, (name, x, y, yaw, act_str, speech) in enumerate(waypoints):
            if self._cancel_flag.is_set():
                break
            if progress_cb:
                progress_cb(i + 1, total, f"→ {name}")
            self.send_nav_goal(x, y, yaw)
            if not self._wait_nav(120):
                continue
            # 执行动作
            if act_str and act_str != "无":
                for aid, aname in ARM_ACTION_MAP.items():
                    if aname == act_str:
                        self.g1_action(aid)
                        break
            if speech:
                self.g1_speak(speech)
            time.sleep(1.0)
            done += 1

        if progress_cb:
            progress_cb(done, total, "巡航完成" if done == total else "已中断")
        self._nav_busy = False
        return done, total

    def _wait_nav(self, timeout=120):
        start = time.time()
        while time.time() - start < timeout:
            if self._cancel_flag.is_set():
                return False
            if self.is_nav_success():
                time.sleep(0.5)
                return True
            if self.goal_status in (2, 4, 5, 8):
                return False
            time.sleep(0.2)
        return False


###############################################################################
# 地图显示组件
###############################################################################
class MapWidget(tk.Canvas):
    """在 Canvas 上显示 2D 栅格地图 + 机器人 + 航点"""

    def __init__(self, parent, backend, **kw):
        super().__init__(parent, bg="#2b2b2b", **kw)
        self.bot = backend
        self._wp = []                     # 外部航点列表
        self._markers = []                # Canvas 对象 id 列表
        self._robot_marker = None
        self._ox = self._oy = 0.0
        self._res = 0.05
        self._mw = self._mh = 0
        self._offx = self._offy = 0
        self._scale = 2.0
        self._drag_start = None
        self._auto_fit = True
        self._pixel_buf = None            # PhotoImage 用的字节 buffer（暂不用）

        self.bind("<ButtonPress-3>", self._drag_start_)
        self.bind("<B3-Motion>", self._drag_move_)
        self.bind("<Button-1>", self._click_)
        self.bind("<MouseWheel>", self._wheel_)
        self._redraw()

    def set_waypoints(self, wps):
        self._wp = wps

    def fit(self):
        self._auto_fit = True
        self._offx = self._offy = 0
        self._redraw()

    def _m2c(self, mx, my):
        """地图坐标 → canvas 像素"""
        px = (mx - self._ox) / self._res * self._scale + self._offx
        py = (my - self._oy) / self._res * self._scale + self._offy
        if self._mh:
            py = self._mh * self._scale - py + self._offy * 2
        return px, py

    def _c2m(self, cx, cy):
        if self._mh:
            cy = self._mh * self._scale + self._offy * 2 - cy
        mx = (cx - self._offx) / self._scale * self._res + self._ox
        my = (cy - self._offy) / self._scale * self._res + self._oy
        return mx, my

    def _drag_start_(self, e):
        self._drag_start = (e.x, e.y)

    def _drag_move_(self, e):
        if self._drag_start:
            dx, dy = e.x - self._drag_start[0], e.y - self._drag_start[1]
            self._offx += dx
            self._offy += dy
            self._drag_start = (e.x, e.y)
            self._auto_fit = False
            self._redraw()

    def _click_(self, e):
        mx, my = self._c2m(e.x, e.y)
        _, _, cyaw = self.bot.robot_pose
        self.bot.send_nav_goal(mx, my, cyaw)

    def _wheel_(self, e):
        self._scale *= 1.15 if e.delta > 0 else 0.85
        self._scale = max(0.3, min(10.0, self._scale))
        self._auto_fit = False
        self._redraw()

    def _redraw(self):
        self.delete("all")
        w = self.winfo_width() or 600
        h = self.winfo_height() or 500

        m = self.bot.map_data
        if m and m.info:
            info = m.info
            self._ox = info.origin.position.x
            self._oy = info.origin.position.y
            self._res = info.resolution
            self._mw, self._mh = info.width, info.height

            if self._auto_fit:
                self._scale = min(w / (self._mw * self._res * 2),
                                  h / (self._mh * self._res * 2))
                self._scale = max(0.5, min(self._scale, 8.0))
                self._offx, self._offy = w / 2, h / 2

            step = max(1, int(8 / self._scale))
            ps = max(1, int(self._scale))
            data = m.data
            for py in range(0, self._mh, step):
                for px in range(0, self._mw, step):
                    val = data[py * self._mw + px] if (py * self._mw + px) < len(data) else -1
                    if val == -1:
                        c = "#555"
                    elif val > 50:
                        c = "#000"
                    elif val > 0:
                        g = int(200 - val * 1.5)
                        c = f"#{g:02x}{g:02x}{g:02x}"
                    else:
                        c = "#ddd"
                    cx = px * self._scale + self._offx
                    cy = py * self._scale + self._offy
                    self.create_rectangle(cx, cy, cx + ps, cy + ps, outline="", fill=c)
        else:
            self.create_text(w // 2, h // 2, text="等待地图数据…", fill="#888",
                             font=("Arial", 14))

        # 航点标记
        for mid in self._markers:
            self.delete(mid)
        self._markers = []
        for i, wp in enumerate(self._wp):
            mx, my = wp[1], wp[2]
            cx, cy = self._m2c(mx, my)
            r = 6
            self._markers.append(
                self.create_oval(cx - r, cy - r, cx + r, cy + r,
                                 fill="#ff6600", outline="#fff", width=2))
            self._markers.append(
                self.create_text(cx + r + 4, cy, text=str(i + 1), anchor="w",
                                 fill="#ff6600", font=("Arial", 9, "bold")))

        # 机器人
        px, py, yaw = self.bot.robot_pose
        cx, cy = self._m2c(px, py)
        if self._robot_marker:
            for mid in self._robot_marker:
                self.delete(mid)
        r = 8
        body = self.create_oval(cx - r, cy - r, cx + r, cy + r,
                                 fill="#00aaff", outline="#fff", width=2)
        al = r * 2
        ax, ay = cx + al * math.cos(yaw), cy + al * math.sin(yaw)
        arr = self.create_line(cx, cy, ax, ay, fill="#fff", width=3, arrow=tk.LAST)
        self._robot_marker = [body, arr]

        self.create_text(10, h - 15, anchor="w", fill="#aaa",
                          font=("Arial", 9),
                          text=f"缩放 {self._scale:.1f}x")


###############################################################################
# 主窗口
###############################################################################
class ControlPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("G1 机器人控制面板")
        self.geometry("1320x820")
        self.minsize(1060, 680)

        self.bot = RobotBackend()
        self.waypoints = []           # [(name, x, y, yaw, action, speech)]
        self._tour_run = False
        self._teleop_on = False
        self._teleop_vx = 0.0
        self._teleop_vy = 0.0
        self._teleop_wz = 0.0

        self._build_ui()
        self.after(300, self._refresh)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ========================================================================
    # 构建 UI
    # ========================================================================
    def _build_ui(self):
        pw = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        pw.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        left = ttk.Frame(pw, width=500)
        right = ttk.Frame(pw)
        pw.add(left, weight=1)
        pw.add(right, weight=3)

        self._build_conn(left)
        self._build_teleop(left)
        self._build_action(left)

        # 地图
        tb = ttk.Frame(right)
        tb.pack(fill=tk.X)
        ttk.Button(tb, text="适应视图", command=self._fit_map).pack(side=tk.LEFT, padx=2)
        ttk.Label(tb, text="  左键→导航 | 右键拖动→平移 | 滚轮→缩放").pack(side=tk.LEFT)

        self.map = MapWidget(right, self.bot, height=400)
        self.map.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self._build_wp(right)
        self._build_status()

    # ---- 连接 ----
    def _build_conn(self, parent):
        f = ttk.LabelFrame(parent, text="连接", padding=5)
        f.pack(fill=tk.X, pady=2)
        r = ttk.Frame(f)
        r.pack(fill=tk.X)
        ttk.Label(r, text="网卡").pack(side=tk.LEFT)
        self._net_if = tk.StringVar(value=DEFAULT_NET_IF)
        ttk.Entry(r, textvariable=self._net_if, width=10).pack(side=tk.LEFT, padx=4)
        self._btn_ros = ttk.Button(r, text="ROS", command=self._toggle_ros)
        self._btn_ros.pack(side=tk.LEFT, padx=2)
        self._btn_g1 = ttk.Button(r, text="G1", command=self._toggle_g1)
        self._btn_g1.pack(side=tk.LEFT, padx=2)
        self._conn_hint = ttk.Label(r, text="", foreground="#666")
        self._conn_hint.pack(side=tk.LEFT, padx=4, fill=tk.X, expand=True)

    # ---- 遥控 ----
    def _build_teleop(self, parent):
        f = ttk.LabelFrame(parent, text="遥控", padding=5)
        f.pack(fill=tk.X, pady=2)

        r = ttk.Frame(f)
        r.pack(fill=tk.X)
        ttk.Label(r, text="速度").pack(side=tk.LEFT)
        self._spd = tk.DoubleVar(value=0.3)
        sc = ttk.Scale(r, from_=0.05, to=1.0, variable=self._spd,
                        orient=tk.HORIZONTAL, length=150)
        sc.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        self._spd_lbl = ttk.Label(r, text="0.30")
        self._spd_lbl.pack(side=tk.LEFT)
        sc.configure(command=lambda v: self._spd_lbl.config(text=f"{float(v):.2f}"))

        # 方向按钮
        g = ttk.Frame(f)
        g.pack(pady=5)
        ttk.Button(g, text="↑\nW", width=7,
                   command=lambda: self._teleop_start(1, 0, 0)).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(g, text="←\nA", width=7,
                   command=lambda: self._teleop_start(0, 0, 1)).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(g, text="STOP\n空格", width=7,
                   command=self._teleop_stop).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(g, text="→\nD", width=7,
                   command=lambda: self._teleop_start(0, 0, -1)).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(g, text="↓\nS", width=7,
                   command=lambda: self._teleop_start(-1, 0, 0)).grid(row=2, column=1, padx=2, pady=2)
        ttk.Button(g, text="←横\nQ", width=7,
                   command=lambda: self._teleop_start(0, 1, 0)).grid(row=2, column=0, padx=2, pady=2)
        ttk.Button(g, text="→横\nE", width=7,
                   command=lambda: self._teleop_start(0, -1, 0)).grid(row=2, column=2, padx=2, pady=2)

        self.bind("<KeyPress-w>", lambda e: self._teleop_start(1, 0, 0))
        self.bind("<KeyPress-s>", lambda e: self._teleop_start(-1, 0, 0))
        self.bind("<KeyPress-a>", lambda e: self._teleop_start(0, 0, 1))
        self.bind("<KeyPress-d>", lambda e: self._teleop_start(0, 0, -1))
        self.bind("<KeyPress-q>", lambda e: self._teleop_start(0, 1, 0))
        self.bind("<KeyPress-e>", lambda e: self._teleop_start(0, -1, 0))
        self.bind("<KeyRelease>", self._on_keyup)
        self.bind("<KeyPress-space>", lambda e: self._teleop_stop())

        ttk.Button(f, text="■ 急停", command=self._emergency_stop,
                   style="Stop.TButton").pack(fill=tk.X, pady=2)
        style = ttk.Style()
        style.configure("Stop.TButton", foreground="red", font=("Arial", 10, "bold"))

    # ---- 动作 ----
    def _build_action(self, parent):
        f = ttk.LabelFrame(parent, text="动作", padding=5)
        f.pack(fill=tk.X, pady=2)

        # FSM
        r = ttk.Frame(f)
        r.pack(fill=tk.X, pady=1)
        ttk.Label(r, text="FSM", width=5).grid(row=0, column=0, sticky="w", padx=(0, 4))
        for idx, (txt, fid) in enumerate([("行走", 200), ("阻尼", 1), ("坐下", 3)]):
            ttk.Button(r, text=txt, width=6,
                       command=lambda i=fid: self.bot.g1_set_fsm(i)).grid(row=0, column=idx + 1, padx=1, pady=1)
        ttk.Button(r, text="站起", width=6, command=self.bot.g1_stand_up).grid(row=0, column=4, padx=1, pady=1)

        # 手臂动作
        r = ttk.Frame(f)
        r.pack(fill=tk.X, pady=1)
        ttk.Label(r, text="手臂", width=5).grid(row=0, column=0, sticky="nw", padx=(0, 4), pady=2)
        # 常用动作
        common = [("挥手", "face wave"), ("鼓掌", "clap"), ("拥抱", "hug"),
                  ("比心", "heart"), ("举手", "right hand up"), ("拒绝", "reject"),
                  ("握手", "shake hand"), ("展示", "x-ray")]
        for idx, (txt, an) in enumerate(common):
            ttk.Button(r, text=txt, width=7,
                       command=lambda n=an: self._do_arm(n)).grid(
                           row=idx // 4, column=idx % 4 + 1, padx=1, pady=1)

        # 手臂 + 灵巧手协同动作（保持运控 FSM=200，只接管 arm_sdk）
        r = ttk.Frame(f)
        r.pack(fill=tk.X, pady=1)
        ttk.Label(r, text="协同", width=5).grid(row=0, column=0, sticky="w", padx=(0, 4))
        self._coord_act = tk.StringVar(value=COORD_ACTION_NAMES[0] if COORD_ACTION_NAMES else "不可用")
        self._coord_combo = ttk.Combobox(
            r, textvariable=self._coord_act, values=COORD_ACTION_NAMES,
            width=16, state="readonly" if COORD_ACTION_NAMES else "disabled")
        self._coord_combo.grid(row=0, column=1, columnspan=2, sticky="ew", padx=1, pady=1)
        ttk.Button(r, text="执行", width=7, command=self._do_coord).grid(row=0, column=3, padx=1, pady=1)
        ttk.Button(r, text="加载JSON", width=9, command=self._coord_load).grid(row=1, column=1, padx=1, pady=1)
        ttk.Button(r, text="停止", width=7, command=self._coord_stop).grid(row=1, column=2, padx=1, pady=1)
        r.columnconfigure(1, weight=1)

        # TTS
        r = ttk.Frame(f)
        r.pack(fill=tk.X, pady=1)
        ttk.Label(r, text="语音", width=5).grid(row=0, column=0, sticky="w", padx=(0, 4))
        self._tts = tk.StringVar()
        ttk.Entry(r, textvariable=self._tts, width=20).grid(row=0, column=1, sticky="ew", padx=1, pady=1)
        ttk.Button(r, text="播报", width=7, command=self._do_tts).grid(row=0, column=2, padx=1, pady=1)
        r.columnconfigure(1, weight=1)

        # LED
        r = ttk.Frame(f)
        r.pack(fill=tk.X, pady=1)
        ttk.Label(r, text="LED", width=5).grid(row=0, column=0, sticky="w", padx=(0, 4))
        for idx, (txt, r_, g_, b_) in enumerate([("红", 255, 0, 0), ("绿", 0, 255, 0),
                                                 ("蓝", 0, 0, 255), ("白", 255, 255, 255),
                                                 ("关", 0, 0, 0)]):
            ttk.Button(r, text=txt, width=5,
                       command=lambda rr=r_, gg=g_, bb=b_: self.bot.g1_led(rr, gg, bb)).grid(
                           row=0, column=idx + 1, padx=1, pady=1)

        # 音量
        r = ttk.Frame(f)
        r.pack(fill=tk.X, pady=1)
        ttk.Label(r, text="音量", width=5).grid(row=0, column=0, sticky="w", padx=(0, 4))
        self._vol = tk.IntVar(value=50)
        ttk.Scale(r, from_=0, to=100, variable=self._vol,
                  orient=tk.HORIZONTAL, length=100,
                  command=lambda v: self.bot.g1_volume(int(float(v)))).grid(row=0, column=1, sticky="ew", padx=1)
        r.columnconfigure(1, weight=1)

    # ---- 航点 ----
    def _build_wp(self, parent):
        f = ttk.LabelFrame(parent, text="航点管理", padding=5)
        f.pack(fill=tk.X, pady=2)

        tb = ttk.Frame(f)
        tb.pack(fill=tk.X)
        ttk.Button(tb, text="记录当前位姿", command=self._wp_add).pack(side=tk.LEFT, padx=2)
        ttk.Button(tb, text="加载", command=self._wp_load).pack(side=tk.LEFT, padx=2)
        ttk.Button(tb, text="保存", command=self._wp_save).pack(side=tk.LEFT, padx=2)
        ttk.Button(tb, text="删除选中", command=self._wp_del).pack(side=tk.LEFT, padx=2)

        cols = ("#", "名称", "X", "Y", "朝向", "动作", "语音")
        self._tree = ttk.Treeview(f, columns=cols, show="headings", height=5,
                                   selectmode="browse")
        for c in cols:
            self._tree.heading(c, text=c)
            w = {"#": 30, "名称": 100, "X": 55, "Y": 55, "朝向": 45,
                 "动作": 70, "语音": 120}.get(c, 60)
            self._tree.column(c, width=w)
        sb = ttk.Scrollbar(f, orient=tk.VERTICAL, command=self._tree.yview)
        self._tree.configure(yscrollcommand=sb.set)
        self._tree.pack(side=tk.LEFT, fill=tk.X, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self._tree.bind("<Double-1>", self._wp_edit)

        # 巡航
        nr = ttk.Frame(f)
        nr.pack(fill=tk.X, pady=2)
        self._btn_tour = ttk.Button(nr, text="▶ 开始巡航", command=self._tour_toggle)
        self._btn_tour.pack(side=tk.LEFT, padx=2)
        self._btn_cancel = ttk.Button(nr, text="取消导航", command=self._nav_cancel)
        self._btn_cancel.pack(side=tk.LEFT, padx=2)
        ttk.Button(nr, text="定位到选中", command=self._wp_go).pack(side=tk.LEFT, padx=2)

        self._tour_pb = ttk.Progressbar(nr, length=200, mode="determinate")
        self._tour_pb.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
        self._tour_lbl = ttk.Label(nr, text="就绪", width=15)
        self._tour_lbl.pack(side=tk.LEFT)

    # ---- 状态栏 ----
    def _build_status(self):
        bar = ttk.Frame(self, relief=tk.SUNKEN)
        bar.pack(fill=tk.X, side=tk.BOTTOM)
        self._lbl_ros = ttk.Label(bar, text="ROS: 未连接", width=15)
        self._lbl_ros.pack(side=tk.LEFT, padx=5)
        self._lbl_g1 = ttk.Label(bar, text="G1: 未连接", width=15)
        self._lbl_g1.pack(side=tk.LEFT, padx=5)
        self._lbl_pose = ttk.Label(bar, text="位姿: ---", width=28)
        self._lbl_pose.pack(side=tk.LEFT, padx=5)
        self._lbl_nav = ttk.Label(bar, text="导航: 空闲", width=14)
        self._lbl_nav.pack(side=tk.LEFT, padx=5)
        self._lbl_coord = ttk.Label(bar, text="协同: ---")
        self._lbl_coord.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

    # ========================================================================
    # 回调
    # ========================================================================
    def _toggle_ros(self):
        if self.bot.ros_connected:
            self.bot.ros_connected = False
            self._btn_ros.config(text="ROS")
        else:
            if self.bot.ros_connect():
                self._btn_ros.config(text="断开ROS")

    def _toggle_g1(self):
        if self.bot.g1_connected:
            self.bot.g1_disconnect()
            self._btn_g1.config(text="G1")
        else:
            self.bot._g1_net_if = self._net_if.get()
            if self.bot.g1_connect():
                self._btn_g1.config(text="断开G1")

    def _teleop_start(self, vxd, vyd, wzd):
        s = self._spd.get()
        self._teleop_vx, self._teleop_vy = vxd * s, vyd * s
        self._teleop_wz = wzd * min(s, MAX_ANG_VEL)
        self._teleop_on = True
        self.bot.send_cmd_vel(self._teleop_vx, self._teleop_vy, self._teleop_wz)
        self.bot.g1_move(self._teleop_vx, self._teleop_vy, self._teleop_wz)

    def _teleop_stop(self):
        self._teleop_on = False
        self._teleop_vx = self._teleop_vy = self._teleop_wz = 0.0
        self.bot.stop_robot()
        self.bot.g1_stop()

    def _emergency_stop(self):
        self.bot.g1_stop_coord()
        self._teleop_stop()
        if self._tour_run:
            self._nav_cancel()
        self.bot.send_cmd_vel(0, 0, 0)
        self.bot.g1_stop()

    def _on_keyup(self, e):
        if e.keysym in ("w", "s", "a", "d", "q", "e", "space") and self._teleop_on:
            self._teleop_stop()

    def _do_arm(self, name):
        if self.bot.is_coord_running():
            messagebox.showwarning("提示", "协同动作执行中，请先停止后再执行预设手臂动作")
            return
        for aid, aname in ARM_ACTION_MAP.items():
            if aname == name:
                self.bot.g1_action(aid)
                break

    def _do_coord(self):
        if not COORD_AVAILABLE:
            messagebox.showerror("协同模块不可用", str(globals().get("COORD_IMPORT_ERROR", "")))
            return
        name = self._coord_act.get()
        ok, msg = self.bot.g1_play_coord_action(name)
        if not ok:
            messagebox.showwarning("协同动作", msg)

    def _coord_load(self):
        if not COORD_AVAILABLE:
            messagebox.showerror("协同模块不可用", str(globals().get("COORD_IMPORT_ERROR", "")))
            return
        path = filedialog.askopenfilename(
            title="加载协同动作",
            initialdir=os.path.expanduser("~"),
            filetypes=[("JSON", "*.json"), ("所有", "*.*")])
        if not path:
            return
        ok, msg = self.bot.g1_play_coord_file(path)
        if not ok:
            messagebox.showwarning("协同动作", msg)

    def _coord_stop(self):
        self.bot.g1_stop_coord()

    def _do_tts(self):
        txt = self._tts.get().strip()
        if txt:
            self.bot.g1_speak(txt)

    # ---- 航点 ----
    def _wp_add(self):
        x, y, yaw = self.bot.robot_pose
        d = WpDialog(self, f"航点{len(self.waypoints)+1}", x, y, yaw)
        self.wait_window(d)
        if d.result:
            self.waypoints.append(d.result)
            self._wp_refresh()

    def _wp_del(self):
        sel = self._tree.selection()
        if sel:
            idx = int(self._tree.item(sel[0])["values"][0]) - 1
            if 0 <= idx < len(self.waypoints):
                self.waypoints.pop(idx)
                self._wp_refresh()

    def _wp_edit(self, e=None):
        sel = self._tree.selection()
        if not sel:
            return
        idx = int(self._tree.item(sel[0])["values"][0]) - 1
        if idx < 0 or idx >= len(self.waypoints):
            return
        d = WpDialog(self, *self.waypoints[idx])
        self.wait_window(d)
        if d.result:
            self.waypoints[idx] = d.result
            self._wp_refresh()

    def _wp_go(self):
        sel = self._tree.selection()
        if not sel:
            return
        idx = int(self._tree.item(sel[0])["values"][0]) - 1
        if idx < 0 or idx >= len(self.waypoints):
            return
        _, x, y, yaw, _, _ = self.waypoints[idx]
        self.bot.send_nav_goal(x, y, yaw)

    def _wp_save(self):
        path = filedialog.asksaveasfilename(
            title="保存航点", initialdir=os.path.expanduser("~"),
            defaultextension=".json",
            filetypes=[("JSON", "*.json"), ("所有", "*.*")])
        if not path:
            return
        data = [{"name": n, "x": x, "y": y, "yaw": yaw, "action": a, "speech": s}
                for n, x, y, yaw, a, s in self.waypoints]
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        messagebox.showinfo("保存成功", f"已保存 {len(data)} 个航点")

    def _wp_load(self):
        path = filedialog.askopenfilename(
            title="加载航点", initialdir=os.path.expanduser("~"),
            filetypes=[("JSON", "*.json"), ("所有", "*.*")])
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            self.waypoints = [(d["name"], d["x"], d["y"], d["yaw"],
                               d.get("action", ""), d.get("speech", "")) for d in data]
            self._wp_refresh()
            messagebox.showinfo("加载成功", f"已加载 {len(self.waypoints)} 个航点")
        except Exception as e:
            messagebox.showerror("加载失败", str(e))

    def _wp_refresh(self):
        for item in self._tree.get_children():
            self._tree.delete(item)
        for i, (n, x, y, yaw, a, s) in enumerate(self.waypoints):
            self._tree.insert("", tk.END, values=(
                i + 1, n, f"{x:.2f}", f"{y:.2f}", f"{math.degrees(yaw):.0f}°",
                a or "无", s or ""))
        self.map.set_waypoints(self.waypoints)

    # ---- 巡航 ----
    def _tour_toggle(self):
        if self._tour_run:
            self._nav_cancel()
            return
        if len(self.waypoints) < 1:
            messagebox.showwarning("提示", "请先添加航点")
            return
        if not self.bot.ros_connected:
            messagebox.showwarning("提示", "请先连接 ROS")
            return
        self._tour_run = True
        self._btn_tour.config(text="■ 停止巡航")
        self._tour_pb["maximum"] = len(self.waypoints)
        self._tour_pb["value"] = 0

        def prog(i, t, msg):
            self.after(0, lambda: self._tour_pb.configure(value=i))
            self.after(0, lambda: self._tour_lbl.configure(text=msg))

        def run():
            done, total = self.bot.execute_tour(self.waypoints, progress_cb=prog)
            self.after(0, lambda: self._tour_done(done, total))

        threading.Thread(target=run, daemon=True).start()

    def _tour_done(self, done, total):
        self._tour_run = False
        self._btn_tour.config(text="▶ 开始巡航")
        self._tour_lbl.config(text=f"{done}/{total}")

    def _nav_cancel(self):
        self.bot.cancel_nav()
        self._tour_run = False
        self._btn_tour.config(text="▶ 开始巡航")
        self._tour_lbl.config(text="已取消")

    # ---- 刷新 ----
    def _fit_map(self):
        self.map.fit()

    def _refresh(self):
        self._lbl_ros.config(text=f"ROS: {'已连' if self.bot.ros_connected else '未连'}")
        self._lbl_g1.config(text=f"G1: {'已连' if self.bot.g1_connected else '未连'}")
        x, y, yaw = self.bot.robot_pose
        self._lbl_pose.config(text=f"位姿: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)")
        self._lbl_nav.config(text=f"导航: {self.bot.nav_status}")
        self._lbl_coord.config(text=f"协同: {self.bot.coord_status}")
        self.after(300, self._refresh)

    # ---- 关闭 ----
    def _on_close(self):
        self._emergency_stop()
        self.destroy()


###############################################################################
# 航点编辑对话框
###############################################################################
class WpDialog(tk.Toplevel):
    def __init__(self, parent, name="", x=0, y=0, yaw=0, action="", speech=""):
        super().__init__(parent)
        self.title("编辑航点")
        self.result = None
        self.transient(parent)
        self.grab_set()

        f = ttk.Frame(self, padding=15)
        f.pack()

        r = ttk.Frame(f)
        r.pack(fill=tk.X, pady=2)
        ttk.Label(r, text="名称:").pack(side=tk.LEFT)
        self._name = tk.StringVar(value=name)
        ttk.Entry(r, textvariable=self._name, width=20).pack(side=tk.LEFT, padx=5)

        r = ttk.Frame(f)
        r.pack(fill=tk.X, pady=2)
        ttk.Label(r, text="X:").pack(side=tk.LEFT)
        self._vx = tk.DoubleVar(value=x)
        ttk.Entry(r, textvariable=self._vx, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Label(r, text="Y:").pack(side=tk.LEFT)
        self._vy = tk.DoubleVar(value=y)
        ttk.Entry(r, textvariable=self._vy, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Label(r, text="朝向(°):").pack(side=tk.LEFT)
        self._vyaw = tk.DoubleVar(value=math.degrees(yaw))
        ttk.Entry(r, textvariable=self._vyaw, width=7).pack(side=tk.LEFT, padx=2)

        r = ttk.Frame(f)
        r.pack(fill=tk.X, pady=2)
        ttk.Label(r, text="动作:").pack(side=tk.LEFT)
        self._act = tk.StringVar(value=action if action else "无")
        opts = ["无"] + sorted(ARM_ACTION_MAP.values())
        ttk.Combobox(r, textvariable=self._act, values=opts, width=14).pack(side=tk.LEFT, padx=5)

        r = ttk.Frame(f)
        r.pack(fill=tk.X, pady=2)
        ttk.Label(r, text="语音:").pack(side=tk.LEFT)
        self._sp = tk.StringVar(value=speech)
        ttk.Entry(r, textvariable=self._sp, width=28).pack(side=tk.LEFT, padx=5)

        r = ttk.Frame(f)
        r.pack(pady=10)
        ttk.Button(r, text="确定", command=self._ok).pack(side=tk.LEFT, padx=5)
        ttk.Button(r, text="取消", command=self.destroy).pack(side=tk.LEFT, padx=5)

        self.geometry(f"+{parent.winfo_rootx()+100}+{parent.winfo_rooty()+150}")

    def _ok(self):
        try:
            n = self._name.get().strip() or "航点"
            x, y = self._vx.get(), self._vy.get()
            yaw = math.radians(self._vyaw.get())
            a = self._act.get() if self._act.get() != "无" else ""
            s = self._sp.get().strip()
            self.result = (n, x, y, yaw, a, s)
            self.destroy()
        except Exception as e:
            messagebox.showerror("输入错误", str(e))


###############################################################################
# 入口
###############################################################################
def main():
    # 检查 ROS Master
    if ROS_AVAILABLE:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect(("localhost", 11311))
            s.close()
        except Exception:
            print("⚠ 未检测到 ROS Master，地图和导航功能不可用")
    else:
        print("⚠ ROS 库未安装，地图和导航功能不可用")
    if not G1_AVAILABLE:
        print("⚠ unitree_sdk2py 未安装，G1 动作功能不可用")

    app = ControlPanel()
    app.mainloop()


if __name__ == "__main__":
    main()
