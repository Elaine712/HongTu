#!/usr/bin/env python3
"""G1 body-side control service.

Runs on the robot. The PC GUI talks to this service over HTTP, while all
Unitree DDS/RPC calls stay local to the G1 body.
"""

import argparse
import json
import os
import signal
import sys
import threading
import time
from enum import IntEnum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


BASE = os.path.dirname(os.path.abspath(__file__))
for path in (os.path.join(BASE, "unitree_sdk2_python"), os.path.join(BASE, "inspire_hand")):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)


from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
from unitree_sdk2py.g1.arm.g1_arm_action_client import action_map as ARM_ACTIONS
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient as G1AudioClient
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

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


HAND_PRESETS = {
    "张开": [1000, 1000, 1000, 1000, 1000, 1000],
    "握拳": [0, 0, 0, 0, 0, 0],
    "指向": [0, 0, 0, 1000, 0, 500],
    "OK": [0, 0, 0, 0, 300, 300],
    "点赞": [0, 0, 0, 0, 1000, 500],
    "摇滚": [1000, 0, 0, 1000, 1000, 500],
    "三指捏": [0, 0, 300, 300, 300, 300],
    "半开": [500, 500, 500, 500, 500, 500],
    "点按": [0, 0, 0, 800, 0, 500],
}

COORDINATED_ACTIONS = {
    "face wave": ("face wave", "张开"),
    "shake hand": ("shake hand", "握拳"),
    "clap": ("clap", "张开"),
    "heart": ("heart", "OK"),
    "right hand up": ("right hand up", "张开"),
    "high five": ("high five", "张开"),
    "hug": ("hug", "半开"),
    "reject": ("reject", "张开"),
    "x-ray": ("x-ray", "张开"),
    "hands up": ("hands up", "张开"),
}

G1_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]
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
G1_ARM_SDK_ENABLE_JOINT = 29


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


def init_channel(net_if):
    net_if = (net_if or "").strip()
    if net_if and net_if.lower() not in ("auto", "local", "none"):
        ChannelFactoryInitialize(0, net_if)
    else:
        ChannelFactoryInitialize(0)


class G1Robot:
    def __init__(self, net_if="auto"):
        self.net_if = net_if
        self.lock = threading.RLock()
        self.ready = False
        self.loco = None
        self.arm = None
        self.audio = None
        self.tts_index = 1
        self.last_move_time = 0.0
        self.moving = False
        self.hand_ready = False
        self.hand_pub_l = None
        self.hand_pub_r = None
        self.hand_state = {"l": {}, "r": {}}

        self.arm_ready = False
        self.arm_active = False
        self.arm_pub = None
        self.arm_cmd = None
        self.arm_crc = CRC()
        self.arm_targets = [0.0] * G1_ARM_DOF
        self.arm_current_cmd_q = [0.0] * G1_ARM_DOF
        self.arm_weight = 0.0
        self.arm_publish_dt = 1.0 / 250.0
        self.arm_velocity_limit = 2.0
        self.arm_stop_event = threading.Event()
        self.arm_thread = None
        self.arm_low_state = None
        self.arm_low_state_lock = threading.Lock()
        self.arm_low_state_event = threading.Event()
        self.arm_cmd_lock = threading.Lock()
        self.arm_target_lock = threading.Lock()

    def log(self, msg):
        print(msg, flush=True)

    def connect(self):
        with self.lock:
            if self.ready:
                return
            self.log(f"[G1服务] 初始化 DDS: {self.net_if}")
            init_channel(self.net_if)

            self.loco = LocoClient()
            self.loco.SetTimeout(10.0)
            self.loco.Init()
            self.loco.Start()

            self.arm = G1ArmActionClient()
            self.arm.SetTimeout(10.0)
            self.arm.Init()

            self.audio = G1AudioClient()
            self.audio.SetTimeout(10.0)
            self.audio.Init()
            self.audio.SetVolume(100)
            self._patch_tts()

            self._init_arm_sdk()
            self._init_hand_dds()
            self.ready = True
            self.log("[G1服务] 就绪")

    def _patch_tts(self):
        def tts(text, speaker_id):
            self.tts_index += 1
            payload = {"index": self.tts_index, "text": text, "speaker_id": speaker_id}
            code, _ = self.audio._Call(1001, json.dumps(payload, ensure_ascii=False))
            return code
        self.audio.TtsMaker = tts

    def _init_arm_sdk(self):
        self.arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_pub.Init()
        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_sub.Init(self._arm_lowstate_cb, 10)
        self.arm_ready = True
        self.log("[G1服务] arm_sdk 就绪")

    def _init_hand_dds(self):
        if not HAND_OK:
            self.log("[G1服务] 灵巧手 SDK 不可用，跳过")
            return
        try:
            self.hand_pub_r = ChannelPublisher("rt/inspire_hand/ctrl/r", _hand_ctrl_type)
            self.hand_pub_r.Init()
            self.hand_pub_l = ChannelPublisher("rt/inspire_hand/ctrl/l", _hand_ctrl_type)
            self.hand_pub_l.Init()
            for lr in ("l", "r"):
                sub = ChannelSubscriber(f"rt/inspire_hand/state/{lr}", _hand_state_type)
                sub.Init(lambda msg, side=lr: self._hand_state_update(side, msg), 10)
            self.hand_ready = True
            self.log("[G1服务] 灵巧手 DDS 就绪")
        except Exception as e:
            self.log(f"[G1服务] 灵巧手 DDS 失败: {e}")

    def _hand_state_update(self, lr, msg):
        self.hand_state[lr] = {
            "angle": list(msg.angle_act) if hasattr(msg, "angle_act") else [],
            "force": list(msg.force_act) if hasattr(msg, "force_act") else [],
            "pos": list(msg.pos_act) if hasattr(msg, "pos_act") else [],
        }

    def _require_ready(self):
        if not self.ready:
            self.connect()

    def status(self):
        return {
            "ready": self.ready,
            "net_if": self.net_if,
            "arm_ready": self.arm_ready,
            "arm_active": self.arm_active,
            "hand_ready": self.hand_ready,
            "moving": self.moving,
            "actions": sorted(ARM_ACTIONS.keys()),
        }

    def ensure_arm_released(self, reason=""):
        if self.arm_active:
            self.log(f"[安全] {reason or '切换控制前'}，释放 arm_sdk")
            self.arm_release()

    def move(self, vx, vy, wz, continuous=False):
        with self.lock:
            self._require_ready()
            vx, vy, wz = float(vx), float(vy), float(wz)
            self.last_move_time = time.time()
            self.moving = abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(wz) > 1e-4
            if self.moving:
                self.loco.Move(vx, vy, wz, continous_move=bool(continuous))
            else:
                self.loco.StopMove()

    def stop(self):
        with self.lock:
            self._require_ready()
            self.moving = False
            self.loco.StopMove()

    def stand(self):
        with self.lock:
            self.ensure_arm_released("切换站立/行走前")
            self._require_ready()
            self.loco.Start()

    def fsm(self, fsm_id):
        with self.lock:
            self.ensure_arm_released("切换 FSM 模式前")
            self._require_ready()
            self.loco.SetFsmId(int(fsm_id))

    def action(self, name=None, action_id=None):
        with self.lock:
            self.ensure_arm_released("执行预设动作前")
            self._require_ready()
            if action_id is None:
                if name not in ARM_ACTIONS:
                    raise ValueError(f"未知动作: {name}")
                action_id = ARM_ACTIONS[name]
            self.arm.ExecuteAction(int(action_id))

    def speak(self, text, speaker_id=0):
        text = (text or "").strip()
        if not text:
            return
        with self.lock:
            self._require_ready()
            self.audio.TtsMaker(text, int(speaker_id))

    def volume(self, value):
        with self.lock:
            self._require_ready()
            self.audio.SetVolume(max(0, min(100, int(value))))

    def led(self, r, g, b):
        with self.lock:
            self._require_ready()
            self.audio.LedControl(int(r), int(g), int(b))

    def hand_preset(self, lr, preset):
        if preset not in HAND_PRESETS:
            raise ValueError(f"未知手势: {preset}")
        self.hand_angles(lr, HAND_PRESETS[preset])

    def hand_angles(self, lr, angles):
        with self.lock:
            self._require_ready()
            if not self.hand_ready:
                raise RuntimeError("灵巧手 DDS 未就绪")
            if lr not in ("l", "r"):
                raise ValueError("lr must be l or r")
            if len(angles) != 6:
                raise ValueError("灵巧手角度需要 6 个值")
            pub = self.hand_pub_r if lr == "r" else self.hand_pub_l
            cmd = get_inspire_hand_ctrl()
            cmd.mode = 0b0001
            cmd.angle_set = [int(max(0, min(1000, v))) for v in angles]
            pub.Write(cmd)

    def coordinated_action(self, name):
        if name not in COORDINATED_ACTIONS:
            self.action(name=name)
            return
        arm_name, hand_preset = COORDINATED_ACTIONS[name]
        self.action(name=arm_name)
        try:
            self.hand_preset("r", hand_preset)
        except Exception as e:
            self.log(f"[协同] 灵巧手失败: {e}")

    def _arm_lowstate_cb(self, msg):
        with self.arm_low_state_lock:
            self.arm_low_state = msg
        self.arm_low_state_event.set()

    def _arm_current_lowstate(self):
        with self.arm_low_state_lock:
            return self.arm_low_state

    def arm_current(self):
        state = self._arm_current_lowstate()
        if not state:
            raise RuntimeError("无 rt/lowstate 数据")
        return [float(state.motor_state[jid].q) for jid in G1_ARM_JOINT_IDS]

    def _arm_prepare_cmd_from_lowstate(self):
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
        cmd.motor_cmd[G1_ARM_SDK_ENABLE_JOINT].q = float(self.arm_weight)
        with self.arm_cmd_lock:
            self.arm_cmd = cmd
        with self.arm_target_lock:
            self.arm_current_cmd_q = [float(state.motor_state[jid].q) for jid in G1_ARM_JOINT_IDS]
        return True

    def _arm_clip_targets(self, targets):
        max_step = max(self.arm_velocity_limit * self.arm_publish_dt, 1e-6)
        next_q = []
        for cur, tgt in zip(self.arm_current_cmd_q, targets):
            delta = float(tgt) - float(cur)
            if delta > max_step:
                delta = max_step
            elif delta < -max_step:
                delta = -max_step
            next_q.append(float(cur) + delta)
        self.arm_current_cmd_q = next_q
        return next_q

    def _arm_publish_once(self):
        if not self.arm_ready or not self.arm_pub:
            return
        with self.arm_cmd_lock:
            cmd = self.arm_cmd
        if cmd is None:
            return
        with self.arm_target_lock:
            targets = list(self.arm_targets)
            weight = float(self.arm_weight)
            clipped = self._arm_clip_targets(targets)
        with self.arm_cmd_lock:
            cmd.motor_cmd[G1_ARM_SDK_ENABLE_JOINT].q = weight
            for i, jid in enumerate(G1_ARM_JOINT_IDS):
                motor = cmd.motor_cmd[jid]
                motor.q = float(clipped[i])
                motor.dq = 0.0
                motor.tau = 0.0
                motor.kp = G1_ARM_PARAMS[i]["kp"]
                motor.kd = G1_ARM_PARAMS[i]["kd"]
            cmd.crc = self.arm_crc.Crc(cmd)
            self.arm_pub.Write(cmd)

    def _arm_publish_loop(self):
        while not self.arm_stop_event.is_set():
            start = time.time()
            try:
                self._arm_publish_once()
            except Exception as e:
                self.log(f"[姿态] arm_sdk 发布异常: {e}")
            sleep_time = self.arm_publish_dt - (time.time() - start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _arm_start_loop(self):
        if self.arm_thread and self.arm_thread.is_alive():
            return
        self.arm_stop_event.clear()
        self.arm_thread = threading.Thread(target=self._arm_publish_loop, daemon=True)
        self.arm_thread.start()

    def _arm_stop_loop(self):
        self.arm_stop_event.set()
        if self.arm_thread and self.arm_thread.is_alive():
            self.arm_thread.join(timeout=1.0)
        self.arm_thread = None

    def _arm_ramp_weight(self, start, end, seconds=1.0):
        steps = max(1, int(seconds / 0.02))
        for step in range(steps + 1):
            ratio = step / steps
            with self.arm_target_lock:
                self.arm_weight = float(start + (end - start) * ratio)
            self._arm_publish_once()
            time.sleep(0.02)

    def arm_activate(self):
        with self.lock:
            self._require_ready()
            if not self.arm_ready:
                raise RuntimeError("arm_sdk 未就绪")
            if self.arm_active:
                return self.arm_current()
            if not self.arm_low_state_event.wait(timeout=2.0):
                raise RuntimeError("未收到 rt/lowstate，禁止激活臂控")
            current = self.arm_current()
            with self.arm_target_lock:
                self.arm_targets = list(current)
                self.arm_current_cmd_q = list(current)
                self.arm_weight = 0.0
            if not self._arm_prepare_cmd_from_lowstate():
                raise RuntimeError("LowCmd 初始化失败，禁止激活臂控")
            self.arm_active = True
            self._arm_start_loop()
            self._arm_ramp_weight(0.0, 1.0, seconds=1.0)
            return current

    def arm_release(self):
        try:
            if self.arm_ready and self.arm_cmd is not None:
                self._arm_ramp_weight(float(self.arm_weight), 0.0, seconds=1.0)
        finally:
            self._arm_stop_loop()
            self.arm_active = False
            with self.arm_target_lock:
                self.arm_weight = 0.0

    def arm_set_joints(self, joints):
        values = [float(v) for v in joints]
        if len(values) != G1_ARM_DOF:
            raise ValueError(f"需要 {G1_ARM_DOF} 个关节角")
        with self.lock:
            self._require_ready()
            if not self.arm_active:
                raise RuntimeError("arm_sdk 未激活")
            with self.arm_target_lock:
                for i, val in enumerate(values):
                    lo = G1_ARM_PARAMS[i]["min"]
                    hi = G1_ARM_PARAMS[i]["max"]
                    self.arm_targets[i] = max(lo, min(hi, val))

    def shutdown(self):
        if self.ready:
            try:
                self.stop()
            except Exception:
                pass
        if self.arm_ready or self.arm_active:
            try:
                self.arm_release()
            except Exception:
                pass


def watchdog(robot, timeout):
    while True:
        time.sleep(0.1)
        try:
            if robot.moving and time.time() - robot.last_move_time > timeout:
                robot.log("[安全] 遥控超时，自动 StopMove")
                robot.stop()
        except Exception as e:
            robot.log(f"[安全] watchdog 异常: {e}")


INDEX_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>G1 导航控制台</title>
  <style>
    :root{--bg:#1a1a2e;--panel:#16213e;--panel2:#0f172a;--line:#3a3a5c;--accent:#e94560;--blue:#0f3460;--text:#e0e0e0;--muted:#a8b0c5;--ok:#27ae60;--danger:#c0392b;--warn:#e67e22}
    *{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}
    header{display:flex;align-items:center;gap:12px;padding:10px 12px;background:#111827;border-bottom:1px solid var(--line);position:sticky;top:0;z-index:3}
    h1{font-size:18px;margin:0;color:#fff}.status{font-size:13px;font-weight:700;padding:5px 10px;border-radius:4px;background:var(--panel);color:var(--muted);white-space:nowrap}
    .spacer{flex:1}.tabs{display:flex;gap:2px;overflow:auto;background:#10162a;border-bottom:1px solid var(--line);position:sticky;top:49px;z-index:2}
    .tab-btn{border:0;border-radius:0;background:#1a1a2e;color:#9ca3af;padding:11px 18px;font-weight:700;white-space:nowrap}.tab-btn.active{background:var(--blue);color:var(--accent)}
    main{padding:10px}.tab{display:none}.tab.active{display:block}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));gap:10px}
    section{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:12px}h2{margin:0 0 10px;color:var(--accent);font-size:13px}
    button,input,textarea,select{font:inherit}button{background:linear-gradient(#16213e,#0f3460);color:var(--text);border:1px solid var(--line);border-radius:4px;padding:8px 12px;margin:3px;cursor:pointer}
    button:hover{border-color:var(--accent);background:#1a4080}button.ok{background:linear-gradient(#27ae60,#1e8449);border-color:#2ecc71;color:white;font-weight:700}
    button.danger{background:linear-gradient(#c0392b,#96281b);border-color:#e74c3c;color:white;font-weight:700}button.warn{background:linear-gradient(#e67e22,#a75d12);border-color:#f59e0b;color:white;font-weight:700}
    input,textarea,select{width:100%;background:#0d1117;color:var(--text);border:1px solid var(--line);border-radius:4px;padding:8px;margin:3px 0}
    textarea{resize:vertical}.row{display:flex;gap:6px;align-items:center;flex-wrap:wrap}.row>*{flex:1}.tight button{min-width:76px}.muted{font-size:12px;color:var(--muted)}
    .pad{display:grid;grid-template-columns:80px 80px 80px;gap:6px;justify-content:center;align-items:center}.pad button{height:48px;margin:0}.pad .wide{grid-column:1/4}
    .swatch{width:42px;height:32px;min-width:42px;border-radius:4px}.sl{display:grid;grid-template-columns:72px 1fr 52px;gap:8px;align-items:center;margin:5px 0}.sl span{font-size:12px;color:var(--muted)}input[type=range]{accent-color:var(--accent);padding:0}
    pre{height:210px;overflow:auto;white-space:pre-wrap;background:#0d1117;border:1px solid var(--line);border-radius:4px;padding:10px;color:#c0c0d0}.mapbox{height:420px;display:flex;align-items:center;justify-content:center;background:#0d1117;border:1px dashed var(--line);border-radius:4px;color:#64748b;text-align:center;padding:20px}
    @media(max-width:760px){header{align-items:flex-start;flex-wrap:wrap}.tabs{top:78px}.grid{grid-template-columns:1fr}.row>*{min-width:130px}.pad{grid-template-columns:1fr 1fr 1fr}}
  </style>
</head>
<body>
  <header>
    <h1>G1 导航控制台</h1>
    <span id="status" class="status">未连接</span>
    <span id="detail" class="status">---</span>
    <span class="spacer"></span>
    <button class="ok" onclick="cmd('/api/connect')">连接 G1</button>
    <button class="danger" onclick="cmd('/api/stop')">急停</button>
  </header>
  <nav class="tabs" id="tabs"></nav>
  <main>
    <div class="tab active" id="tab-nav">
      <div class="grid">
        <section><h2>地图 / 导航</h2><div class="mapbox">网页端当前聚焦 G1 本体控制。地图、重定位和航点规划请使用 PC 本地 GUI。</div></section>
        <section><h2>导航联动</h2>
          <textarea id="navTts" rows="3" placeholder="导航到达后的中文播报"></textarea>
          <button onclick="speak(navTts.value)">播报</button>
          <div class="row tight"><button onclick="cmd('/api/stand')">行走模式</button><button onclick="fsm(1)">阻尼模式</button><button onclick="fsm(3)">坐下</button></div>
        </section>
      </div>
    </div>
    <div class="tab" id="tab-teleop">
      <div class="grid">
        <section><h2>遥控</h2>
          <div class="row"><label>线速度<input id="lin" type="range" min="0" max="100" value="30"></label><label>角速度<input id="ang" type="range" min="0" max="100" value="50"></label></div>
          <div class="pad">
            <span></span><button data-move="1,0,0">前进</button><span></span>
            <button data-move="0,1,0">左移</button><button class="danger" onclick="stop()">停止</button><button data-move="0,-1,0">右移</button>
            <button data-move="0,0,1">左转</button><button data-move="-1,0,0">后退</button><button data-move="0,0,-1">右转</button>
          </div>
        </section>
        <section><h2>FSM 模式</h2><div class="row tight"><button onclick="cmd('/api/stand')">行走模式</button><button onclick="fsm(1)">阻尼模式</button><button onclick="fsm(3)">坐下</button><button onclick="cmd('/api/stop')">停止移动</button></div></section>
      </div>
    </div>
    <div class="tab" id="tab-actions">
      <div class="grid">
        <section><h2>常用动作</h2><div id="commonActions" class="tight"></div></section>
        <section><h2>全部动作</h2><div id="allActions" class="tight"></div></section>
        <section><h2>语音播报</h2><textarea id="tts" rows="4" placeholder="输入中文播报内容"></textarea><button class="ok" onclick="speak(tts.value)">播报</button><div id="phrases" class="tight"></div></section>
        <section><h2>LED / 音量</h2><div class="row tight" id="leds"></div><label>音量 <span id="volText">100</span>%<input id="vol" type="range" min="0" max="100" value="100" oninput="volText.textContent=this.value" onchange="cmd('/api/volume',{value:+this.value})"></label></section>
      </div>
    </div>
    <div class="tab" id="tab-hand">
      <div class="grid">
        <section><h2>灵巧手手势</h2><div class="row"><select id="handSide"><option value="r">右手</option><option value="l">左手</option></select><button onclick="handBoth()">双手同步</button></div><div id="hands" class="tight"></div></section>
        <section><h2>自定义角度</h2><div id="handSliders"></div><button class="ok" onclick="sendHandAngles()">发送角度</button></section>
      </div>
    </div>
    <div class="tab" id="tab-pose">
      <div class="grid">
        <section><h2>arm_sdk 低阶臂控</h2><div class="row"><button class="warn" onclick="armActivate()">激活臂控</button><button class="danger" onclick="armRelease()">释放臂控</button><button onclick="armRead()">读取当前姿态</button><button onclick="armZero()">归零</button></div><p class="muted">执行预设动作、FSM 切换、站立前，后台会自动释放 arm_sdk。</p></section>
        <section><h2>双臂关节</h2><div id="armSliders"></div><button class="ok" onclick="sendArm()">发送当前关节</button></section>
      </div>
    </div>
    <div class="tab" id="tab-settings">
      <div class="grid">
        <section><h2>后台状态</h2><pre id="statusJson"></pre></section>
        <section><h2>连接信息</h2><p class="muted">默认端口 5055。若浏览器不能直连，请在 PC 运行 start_pc_remote_gui.sh 自动建立 SSH 隧道，然后访问 http://127.0.0.1:15055/。</p><button onclick="poll()">刷新状态</button></section>
      </div>
    </div>
    <div class="tab" id="tab-log"><section><h2>日志</h2><pre id="log"></pre></section></div>
  </main>
<script>
const tabDefs=[["tab-nav","导航"],["tab-teleop","遥控"],["tab-actions","动作"],["tab-hand","灵巧手"],["tab-pose","姿态"],["tab-settings","设置"],["tab-log","日志"]];
const presets=["张开","握拳","指向","OK","点赞","三指捏","半开","点按"];
const common=[["face wave","挥手"],["clap","鼓掌"],["hug","拥抱"],["heart","比心"],["right hand up","举手"],["reject","拒绝"],["shake hand","握手"],["x-ray","展示"],["high five","击掌"]];
const phrases=["欢迎参观","请跟我来","这是我们的展品","谢谢大家","请注意安全","正在前往下一个展品"];
const leds=[["红",255,0,0,"#ff3333"],["绿",0,255,0,"#33cc33"],["蓝",0,0,255,"#3366ff"],["白",255,255,255,"#fff"],["关",0,0,0,"#555"]];
const armNames=["左肩前后","左肩左右","左肩旋转","左肘","左腕旋转","左腕俯仰","左腕偏航","右肩前后","右肩左右","右肩旋转","右肘","右腕旋转","右腕俯仰","右腕偏航"];
const armRanges=[[-2,2],[-1.5,1.5],[-2.5,2.5],[-2.5,3],[-1.5,1.5],[-1,1],[-1,1],[-2,2],[-1.5,1.5],[-2.5,2.5],[-2.5,3],[-1.5,1.5],[-1,1],[-1,1]];
const fingerNames=["小指","无名指","中指","食指","拇指屈","拇指旋"];
let arm=Array(14).fill(0), hand=Array(6).fill(500), statusData={};
tabs.innerHTML=tabDefs.map((t,i)=>`<button class="tab-btn ${i?'':'active'}" onclick="showTab('${t[0]}',this)">${t[1]}</button>`).join("");
commonActions.innerHTML=common.map(([a,c])=>`<button onclick="action('${a}')">${c}</button>`).join("");
allActions.innerHTML=common.map(([a,c])=>`<button onclick="action('${a}')">${a}</button>`).join("");
phrases.innerHTML=phrases.map(p=>`<button onclick="speak('${p}')">${p}</button>`).join("");
leds.innerHTML=leds.map(([n,r,g,b,c])=>`<button class="swatch" style="background:${c};color:${n==='白'?'#111':'#fff'}" onclick="cmd('/api/led',{r:${r},g:${g},b:${b}})">${n}</button>`).join("");
hands.innerHTML=presets.map(p=>`<button onclick="handPreset('${p}')">${p}</button>`).join("");
armSliders.innerHTML=armNames.map((n,i)=>`<div class="sl"><span>${n}</span><input id="arm${i}" type="range" min="${armRanges[i][0]*100}" max="${armRanges[i][1]*100}" value="0" oninput="setArm(${i},this.value/100)"><span id="armv${i}">0.00</span></div>`).join("");
handSliders.innerHTML=fingerNames.map((n,i)=>`<div class="sl"><span>${n}</span><input id="hand${i}" type="range" min="0" max="1000" value="500" oninput="setHand(${i},+this.value)"><span id="handv${i}">500</span></div>`).join("");
document.querySelectorAll("[data-move]").forEach(b=>{let v=b.dataset.move.split(",").map(Number);["mousedown","touchstart"].forEach(e=>b.addEventListener(e,ev=>{ev.preventDefault();move(v[0],v[1],v[2])}));["mouseup","mouseleave","touchend","touchcancel"].forEach(e=>b.addEventListener(e,ev=>{ev.preventDefault();stop()}));});
function showTab(id,btn){document.querySelectorAll(".tab").forEach(x=>x.classList.remove("active"));document.getElementById(id).classList.add("active");document.querySelectorAll(".tab-btn").forEach(x=>x.classList.remove("active"));btn.classList.add("active")}
function add(s){log.textContent=(new Date().toLocaleTimeString()+" "+s+"\n"+log.textContent).slice(0,6000)}
async function cmd(path,data={}){try{let r=await fetch(path,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(data)});let j=await r.json();add((j.ok?"OK ":"ERR ")+path+" "+JSON.stringify(j.data||j.error||{}));await poll();return j}catch(e){add("失败 "+path+" "+e);return {ok:false,error:String(e)}}}
function move(x,y,z){let lin=+document.getElementById("lin").value/100,ang=+document.getElementById("ang").value/100;cmd("/api/move",{vx:x*lin,vy:y*lin,wz:z*ang,continuous:false})}
function stop(){cmd("/api/stop")}function fsm(id){cmd("/api/fsm",{id})}function action(name){cmd("/api/coordinated",{name})}function speak(text){cmd("/api/speak",{text:text||""})}
function handPreset(preset){cmd("/api/hand/preset",{lr:handSide.value,preset})}function handBoth(){presets.forEach(()=>{});cmd("/api/hand/angles",{lr:"l",angles:hand});cmd("/api/hand/angles",{lr:"r",angles:hand})}
function setHand(i,v){hand[i]=v;document.getElementById("handv"+i).textContent=v}function sendHandAngles(){cmd("/api/hand/angles",{lr:handSide.value,angles:hand})}
function setArm(i,v){arm[i]=+v;document.getElementById("armv"+i).textContent=(+v).toFixed(2)}function sendArm(){cmd("/api/arm/joints",{joints:arm})}
async function armActivate(){let j=await cmd("/api/arm/activate");let joints=j.data&&j.data.joints;if(joints){arm=joints.slice(0,14);syncArm()}}
function armRelease(){cmd("/api/arm/release")}function armZero(){arm=Array(14).fill(0);syncArm();sendArm()}
async function armRead(){try{let r=await fetch("/api/arm/current");let j=await r.json();if(j.ok&&j.data.joints){arm=j.data.joints.slice(0,14);syncArm();add("OK 读取当前姿态")}}catch(e){add("读取失败 "+e)}}
function syncArm(){arm.forEach((v,i)=>{let s=document.getElementById("arm"+i),l=document.getElementById("armv"+i);if(s){s.value=Math.round(v*100);l.textContent=(+v).toFixed(2)}})}
async function poll(){try{let r=await fetch("/api/status");let j=await r.json();statusData=j.data||{};status.textContent=statusData.ready?"G1: 已连接":"G1: 未连接";status.style.color=statusData.ready?"#a6e3a1":"#f38ba8";detail.textContent=`arm:${statusData.arm_active?"运行":"待机"} hand:${statusData.hand_ready?"就绪":"未就绪"}`;statusJson.textContent=JSON.stringify(statusData,null,2);if(statusData.actions){allActions.innerHTML=statusData.actions.map(a=>`<button onclick="action('${a}')">${a}</button>`).join("")}}catch(e){status.textContent="离线";detail.textContent="---"}}
setInterval(poll,1500);poll();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    robot = None

    def log_message(self, fmt, *args):
        print("[HTTP] " + fmt % args, flush=True)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0") or "0")
        if not length:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        return json.loads(raw) if raw else {}

    def _send_json(self, payload, status=200):
        raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _ok(self, data=None):
        self._send_json({"ok": True, "data": data or {}})

    def _error(self, error, status=500):
        self._send_json({"ok": False, "error": str(error)}, status=status)

    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path == "/":
                raw = INDEX_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
            elif path == "/api/status":
                self._ok(self.robot.status())
            elif path == "/api/arm/current":
                self._ok({"joints": self.robot.arm_current()})
            else:
                self._error("not found", 404)
        except Exception as e:
            self._error(e)

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            data = self._read_json()
            r = self.robot
            if path == "/api/connect":
                r.connect()
                self._ok(r.status())
            elif path == "/api/move":
                r.move(data.get("vx", 0), data.get("vy", 0), data.get("wz", 0), data.get("continuous", False))
                self._ok()
            elif path == "/api/stop":
                r.stop()
                self._ok()
            elif path == "/api/stand":
                r.stand()
                self._ok()
            elif path == "/api/fsm":
                r.fsm(data.get("id"))
                self._ok()
            elif path == "/api/action":
                r.action(name=data.get("name"), action_id=data.get("id"))
                self._ok()
            elif path == "/api/coordinated":
                r.coordinated_action(data.get("name"))
                self._ok()
            elif path == "/api/speak":
                r.speak(data.get("text", ""), data.get("speaker_id", 0))
                self._ok()
            elif path == "/api/volume":
                r.volume(data.get("value", 100))
                self._ok()
            elif path == "/api/led":
                r.led(data.get("r", 0), data.get("g", 0), data.get("b", 0))
                self._ok()
            elif path == "/api/hand/preset":
                r.hand_preset(data.get("lr", "r"), data.get("preset"))
                self._ok()
            elif path == "/api/hand/angles":
                r.hand_angles(data.get("lr", "r"), data.get("angles", []))
                self._ok()
            elif path == "/api/arm/activate":
                self._ok({"joints": r.arm_activate()})
            elif path == "/api/arm/release":
                r.arm_release()
                self._ok()
            elif path == "/api/arm/joints":
                r.arm_set_joints(data.get("joints", []))
                self._ok()
            else:
                self._error("not found", 404)
        except Exception as e:
            self._error(e)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=os.environ.get("HONGTU_SERVICE_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("HONGTU_SERVICE_PORT", "5055")))
    parser.add_argument("--net-if", default=os.environ.get("HONGTU_G1_NET_IF", "auto"))
    parser.add_argument("--no-auto-connect", action="store_true")
    parser.add_argument("--move-timeout", type=float, default=0.8)
    args = parser.parse_args()

    robot = G1Robot(args.net_if)
    Handler.robot = robot
    if not args.no_auto_connect:
        robot.connect()

    threading.Thread(target=watchdog, args=(robot, args.move_timeout), daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.port), Handler)

    def stop(_signum=None, _frame=None):
        print("[G1服务] 正在退出", flush=True)
        robot.shutdown()
        server.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    print(f"[G1服务] HTTP listening on {args.host}:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
