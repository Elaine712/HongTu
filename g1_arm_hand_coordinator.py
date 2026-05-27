#!/usr/bin/env python3
"""
Safe G1 upper-body + Inspire hand coordinated trajectory player.

This module keeps locomotion in the high-level FSM and only takes the
upper-body arm_sdk channel. It never releases the motion mode or publishes
full-body /rt/lowcmd, so the walking controller can keep balancing the robot.
"""

import copy
import json
import os
import sys
import threading
import time
from enum import IntEnum

import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(BASE, "unitree_sdk2_python"), os.path.join(BASE, "inspire_hand")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from inspire_sdkpy.inspire_dds import inspire_hand_ctrl
from inspire_sdkpy.inspire_hand_defaut import get_inspire_hand_ctrl


class TrajectoryStopped(Exception):
    pass


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
    kNotUsedJoint = 29
    NotUsedJoint1 = 30
    NotUsedJoint2 = 31
    NotUsedJoint3 = 32
    NotUsedJoint4 = 33
    NotUsedJoint5 = 34


LEFT_ARM_JOINTS = [
    G1JointIndex.LeftShoulderPitch,
    G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw,
    G1JointIndex.LeftElbow,
    G1JointIndex.LeftWristRoll,
    G1JointIndex.LeftWristPitch,
    G1JointIndex.LeftWristYaw,
]

RIGHT_ARM_JOINTS = [
    G1JointIndex.RightShoulderPitch,
    G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,
    G1JointIndex.RightElbow,
    G1JointIndex.RightWristRoll,
    G1JointIndex.RightWristPitch,
    G1JointIndex.RightWristYaw,
]

ALL_WAIST_JOINTS = [
    G1JointIndex.WaistYaw,
    G1JointIndex.WaistRoll,
    G1JointIndex.WaistPitch,
]

DEFAULT_WAIST_HOLD_JOINTS = [G1JointIndex.WaistYaw]

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

HAND_OPEN = [1000, 1000, 1000, 1000, 1000, 500]
HAND_HALF = [500, 500, 500, 500, 500, 500]
HAND_FIST = [0, 0, 0, 0, 0, 500]
HAND_POINT = [0, 0, 0, 1000, 0, 500]


BUILTIN_ACTIONS = {
    "安全复位+半开手": {
        "name": "安全复位+半开手",
        "approach_duration": 1.0,
        "hold_duration": 0.5,
        "keyframes": [
            {
                "t": 0.0,
                "left_arm": [0.0, 0.20, 0.0, 0.75, 0.0, 0.0, 0.0],
                "right_arm": [0.0, -0.20, 0.0, 0.75, 0.0, 0.0, 0.0],
                "left_hand": HAND_HALF,
                "right_hand": HAND_HALF,
            }
        ],
    },
    "双臂打开+张手": {
        "name": "双臂打开+张手",
        "approach_duration": 1.0,
        "hold_duration": 0.8,
        "keyframes": [
            {
                "t": 0.0,
                "left_arm": [0.0, 0.25, 0.0, 0.85, 0.0, 0.0, 0.0],
                "right_arm": [0.0, -0.25, 0.0, 0.85, 0.0, 0.0, 0.0],
                "left_hand": HAND_HALF,
                "right_hand": HAND_HALF,
            },
            {
                "t": 1.4,
                "left_arm": [-0.25, 0.75, 0.05, 0.75, 0.0, 0.0, 0.0],
                "right_arm": [-0.25, -0.75, -0.05, 0.75, 0.0, 0.0, 0.0],
                "left_hand": HAND_OPEN,
                "right_hand": HAND_OPEN,
            },
        ],
    },
    "右手指向": {
        "name": "右手指向",
        "approach_duration": 1.0,
        "hold_duration": 1.2,
        "keyframes": [
            {
                "t": 0.0,
                "left_arm": [0.0, 0.20, 0.0, 0.75, 0.0, 0.0, 0.0],
                "right_arm": [-0.25, -0.40, 0.0, 0.95, 0.0, 0.0, 0.0],
                "left_hand": HAND_HALF,
                "right_hand": HAND_POINT,
            },
            {
                "t": 1.2,
                "left_arm": [0.0, 0.20, 0.0, 0.75, 0.0, 0.0, 0.0],
                "right_arm": [-0.55, -0.65, 0.0, 0.55, 0.0, 0.0, 0.0],
                "left_hand": HAND_HALF,
                "right_hand": HAND_POINT,
            },
        ],
    },
    "挥右手+张手": {
        "name": "挥右手+张手",
        "approach_duration": 1.0,
        "hold_duration": 0.4,
        "keyframes": [
            {
                "t": 0.0,
                "left_arm": [0.0, 0.20, 0.0, 0.75, 0.0, 0.0, 0.0],
                "right_arm": [-0.45, -0.55, 0.0, 0.80, 0.0, 0.0, 0.0],
                "left_hand": HAND_HALF,
                "right_hand": HAND_OPEN,
            },
            {
                "t": 0.7,
                "right_arm": [-0.55, -0.75, -0.25, 0.80, 0.35, 0.0, 0.0],
                "right_hand": HAND_OPEN,
            },
            {
                "t": 1.4,
                "right_arm": [-0.55, -0.75, 0.25, 0.80, -0.35, 0.0, 0.0],
                "right_hand": HAND_OPEN,
            },
            {
                "t": 2.1,
                "right_arm": [-0.55, -0.75, -0.20, 0.80, 0.30, 0.0, 0.0],
                "right_hand": HAND_OPEN,
            },
        ],
    },
}


class ArmHandTrajectoryPlayer:
    """Plays coordinated arm and Inspire hand trajectories without full-body takeover."""

    def __init__(self, control_dt=0.02):
        self.control_dt = float(control_dt)
        self.publish_dt = 0.004
        self.velocity_limit = 2.0
        self.kp_high = 300.0
        self.kd_high = 3.0
        self.kp_low = 80.0
        self.kd_low = 3.0
        self.kp_wrist = 40.0
        self.kd_wrist = 1.5
        self.weight_ramp = 0.8
        self.release_ramp = 0.8
        self.max_body_tilt = 0.45

        self._crc = CRC()
        self._low_state = None
        self._state_lock = threading.Lock()
        self._state_event = threading.Event()
        self._stop_event = threading.Event()
        self._publish_stop_event = threading.Event()
        self._thread = None
        self._publish_thread = None
        self._running_lock = threading.Lock()
        self._target_lock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._target_q = None
        self._target_tau = None
        self._weight = 0.0
        self._low_cmd = None
        self._inited = False

    def init(self):
        if self._inited:
            return
        self._arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._arm_pub.Init()
        self._lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._lowstate_sub.Init(self._lowstate_handler, 10)

        self._hand_pubs = {}
        for lr in ("l", "r"):
            pub = ChannelPublisher(f"rt/inspire_hand/ctrl/{lr}", inspire_hand_ctrl)
            pub.Init()
            self._hand_pubs[lr] = pub
        self._inited = True

    def _lowstate_handler(self, msg):
        with self._state_lock:
            self._low_state = msg
        self._state_event.set()

    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    def stop(self):
        self._stop_event.set()

    def play_builtin(self, name, done_cb=None):
        if name not in BUILTIN_ACTIONS:
            raise ValueError(f"未知协同动作: {name}")
        return self.play(BUILTIN_ACTIONS[name], done_cb=done_cb)

    def play_file(self, path, done_cb=None):
        with open(path, "r", encoding="utf-8") as f:
            action = json.load(f)
        return self.play(action, done_cb=done_cb)

    def play(self, action, done_cb=None):
        self.init()
        with self._running_lock:
            if self.is_running():
                raise RuntimeError("已有协同动作正在执行，请先停止")
            self._stop_event.clear()
            action = self._normalize_action(action)
            self._thread = threading.Thread(
                target=self._run_action, args=(action, done_cb), daemon=True
            )
            self._thread.start()

    def _normalize_action(self, action):
        if not isinstance(action, dict):
            raise ValueError("动作文件必须是 JSON object")
        frames = action.get("keyframes", [])
        if not frames:
            raise ValueError("动作缺少 keyframes")

        normalized = copy.deepcopy(action)
        normalized["keyframes"] = sorted(frames, key=lambda f: float(f.get("t", 0.0)))
        for frame in normalized["keyframes"]:
            frame["t"] = float(frame.get("t", 0.0))
            for key in ("left_arm", "right_arm"):
                if key in frame:
                    frame[key] = self._arm_vector(frame[key])
            for key in ("left_hand", "right_hand"):
                if key in frame:
                    frame[key] = self._hand_vector(frame[key])
        normalized["approach_duration"] = max(0.2, float(normalized.get("approach_duration", 1.0)))
        normalized["hold_duration"] = max(0.0, float(normalized.get("hold_duration", 0.0)))
        return normalized

    def _arm_vector(self, values):
        vals = [float(v) for v in values]
        if len(vals) > 7:
            raise ValueError("arm 向量最多 7 个关节")
        return vals + [0.0] * (7 - len(vals))

    def _hand_vector(self, values):
        vals = [int(max(0, min(1000, int(v)))) for v in values]
        if len(vals) != 6:
            raise ValueError("hand 向量必须是 6 个 0-1000 的数值")
        return vals

    def _run_action(self, action, done_cb):
        error = None
        try:
            if not self._state_event.wait(timeout=3.0):
                raise RuntimeError("未收到 rt/lowstate，无法安全接管手臂")

            start_left = self._read_joints(LEFT_ARM_JOINTS)
            start_right = self._read_joints(RIGHT_ARM_JOINTS)
            self._prepare_xr_style_command()
            self._set_target(start_left, start_right)
            self._start_publish_loop()

            frames = action["keyframes"]
            first_left = self._frame_arm(frames[0], "left_arm", start_left)
            first_right = self._frame_arm(frames[0], "right_arm", start_right)
            last_left = start_left
            last_right = start_right

            t0 = time.monotonic()
            approach = action["approach_duration"]
            while not self._stop_event.is_set():
                elapsed = time.monotonic() - t0
                ratio = min(1.0, elapsed / approach)
                weight = min(1.0, elapsed / self.weight_ramp)
                left = self._lerp_vec(start_left, first_left, self._smooth(ratio))
                right = self._lerp_vec(start_right, first_right, self._smooth(ratio))
                hand = frames[0]
                self._assert_body_safe()
                self._set_weight(weight)
                self._set_target(left, right)
                self._write_hand_frame(hand)
                last_left, last_right = left, right
                if ratio >= 1.0:
                    break
                time.sleep(self.control_dt)

            traj_start = time.monotonic()
            end_t = max(float(frames[-1].get("t", 0.0)), 0.0)
            while not self._stop_event.is_set():
                elapsed = time.monotonic() - traj_start
                left = self._sample_arm(frames, elapsed, "left_arm", last_left)
                right = self._sample_arm(frames, elapsed, "right_arm", last_right)
                hand = self._sample_hand_frame(frames, elapsed)
                self._assert_body_safe()
                self._set_weight(1.0)
                self._set_target(left, right)
                self._write_hand_frame(hand)
                last_left, last_right = left, right
                if elapsed >= end_t:
                    break
                time.sleep(self.control_dt)

            hold_end = time.monotonic() + action["hold_duration"]
            last_hand = self._sample_hand_frame(frames, end_t)
            while not self._stop_event.is_set() and time.monotonic() < hold_end:
                self._assert_body_safe()
                self._set_weight(1.0)
                self._set_target(last_left, last_right)
                self._write_hand_frame(last_hand)
                time.sleep(self.control_dt)

            self._release()
            if self._stop_event.is_set():
                raise TrajectoryStopped("已停止")
        except Exception as exc:
            error = exc
            try:
                self._release()
            except Exception:
                pass
        finally:
            self._stop_publish_loop()
            if done_cb:
                done_cb(error)

    def _prepare_xr_style_command(self):
        state = self._current_state()
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = state.mode_machine
        arm_set = set(int(j) for j in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS)
        for joint in G1JointIndex:
            idx = int(joint)
            motor = cmd.motor_cmd[idx]
            motor.mode = 1
            motor.q = float(state.motor_state[idx].q)
            motor.dq = 0.0
            motor.tau = 0.0
            if idx in arm_set:
                if joint in WRIST_MOTORS:
                    motor.kp = self.kp_wrist
                    motor.kd = self.kd_wrist
                else:
                    motor.kp = self.kp_low
                    motor.kd = self.kd_low
            elif joint in WEAK_MOTORS:
                motor.kp = self.kp_low
                motor.kd = self.kd_low
            else:
                motor.kp = self.kp_high
                motor.kd = self.kd_high
        with self._cmd_lock:
            self._low_cmd = cmd
        self._set_weight(0.0)

    def _start_publish_loop(self):
        self._publish_stop_event.clear()
        if self._publish_thread and self._publish_thread.is_alive():
            return
        self._publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._publish_thread.start()

    def _stop_publish_loop(self):
        self._publish_stop_event.set()
        if self._publish_thread and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=1.0)
        self._publish_thread = None

    def _current_state(self):
        with self._state_lock:
            return self._low_state

    def _read_joints(self, joints):
        with self._state_lock:
            state = self._low_state
            return [float(state.motor_state[int(j)].q) for j in joints]

    def _assert_body_safe(self):
        with self._state_lock:
            state = self._low_state
            rpy = getattr(getattr(state, "imu_state", None), "rpy", None)
        if not rpy or len(rpy) < 2:
            return
        roll, pitch = float(rpy[0]), float(rpy[1])
        if abs(roll) > self.max_body_tilt or abs(pitch) > self.max_body_tilt:
            raise RuntimeError(
                f"身体倾角过大，已释放协同动作: roll={roll:.2f}, pitch={pitch:.2f}"
            )

    def _frame_arm(self, frame, key, fallback):
        return list(frame.get(key, fallback))

    def _sample_arm(self, frames, t, key, fallback):
        prev = None
        next_frame = None
        for frame in frames:
            if key in frame and frame["t"] <= t:
                prev = frame
            if key in frame and frame["t"] >= t:
                next_frame = frame
                break
        if prev is None and next_frame is None:
            return fallback
        if prev is None:
            return self._frame_arm(next_frame, key, fallback)
        if next_frame is None or next_frame is prev:
            return self._frame_arm(prev, key, fallback)
        span = max(1e-6, next_frame["t"] - prev["t"])
        ratio = self._smooth((t - prev["t"]) / span)
        return self._lerp_vec(prev[key], next_frame[key], ratio)

    def _sample_hand_frame(self, frames, t):
        selected = frames[0]
        for frame in frames:
            if frame["t"] <= t:
                selected = frame
            else:
                break
        return selected

    def _write_hand_frame(self, hand_frame):
        if "left_hand" in hand_frame:
            self._write_hand("l", hand_frame["left_hand"])
        if "right_hand" in hand_frame:
            self._write_hand("r", hand_frame["right_hand"])

    def _set_target(self, left, right):
        target = np.array(list(left) + list(right), dtype=float)
        with self._target_lock:
            self._target_q = target
            self._target_tau = np.zeros_like(target)

    def _set_weight(self, weight):
        self._weight = float(max(0.0, min(1.0, weight)))

    def _current_arm_q(self):
        return np.array(self._read_joints(LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS), dtype=float)

    def _clip_target(self, target):
        current = self._current_arm_q()
        delta = target - current
        step = max(self.velocity_limit * self.publish_dt, 1e-6)
        scale = float(np.max(np.abs(delta)) / step) if len(delta) else 1.0
        return current + delta / max(scale, 1.0)

    def _publish_loop(self):
        arm_joints = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
        while not self._publish_stop_event.is_set():
            start = time.time()
            with self._target_lock:
                if self._target_q is None:
                    time.sleep(self.publish_dt)
                    continue
                target_q = self._target_q.copy()
                target_tau = self._target_tau.copy()
            clipped = self._clip_target(target_q)
            with self._cmd_lock:
                if self._low_cmd is None:
                    continue
                self._low_cmd.motor_cmd[int(G1JointIndex.kNotUsedJoint)].q = self._weight
                for idx, joint in enumerate(arm_joints):
                    motor = self._low_cmd.motor_cmd[int(joint)]
                    motor.q = float(clipped[idx])
                    motor.dq = 0.0
                    motor.tau = float(target_tau[idx])
                self._low_cmd.crc = self._crc.Crc(self._low_cmd)
                self._arm_pub.Write(self._low_cmd)
            time.sleep(max(0.0, self.publish_dt - (time.time() - start)))

    def _write_hand(self, lr, values):
        pub = self._hand_pubs.get(lr)
        if not pub:
            return
        cmd = get_inspire_hand_ctrl()
        cmd.mode = 0b0001
        cmd.angle_set = [int(v) for v in values]
        pub.Write(cmd)

    def _release(self):
        start = time.monotonic()
        while time.monotonic() - start < self.release_ramp:
            ratio = (time.monotonic() - start) / self.release_ramp
            self._set_weight(1.0 - self._smooth(ratio))
            time.sleep(self.control_dt)
        self._release_silent()

    def _release_silent(self):
        with self._cmd_lock:
            if self._low_cmd is None:
                cmd = unitree_hg_msg_dds__LowCmd_()
            else:
                cmd = self._low_cmd
            cmd.motor_cmd[int(G1JointIndex.kNotUsedJoint)].q = 0.0
            cmd.crc = self._crc.Crc(cmd)
            self._arm_pub.Write(cmd)
        self._set_weight(0.0)

    def _lerp_vec(self, a, b, ratio):
        return [(1.0 - ratio) * float(x) + ratio * float(y) for x, y in zip(a, b)]

    def _smooth(self, value):
        x = max(0.0, min(1.0, float(value)))
        return x * x * (3.0 - 2.0 * x)


def load_action_names():
    return list(BUILTIN_ACTIONS.keys())
