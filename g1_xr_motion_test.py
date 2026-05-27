#!/usr/bin/env python3
"""
Minimal xr_teleoperate-style G1 arm motion-mode test.

This script is intentionally separate from the main GUI. It validates the
Unitree xr_teleoperate arm-control pattern before we reuse it in production:

  - publish to rt/arm_sdk, not rt/lowcmd
  - use motion mode semantics
  - initialize the command with the current full-body joint positions
  - keep all non-arm joints locked in the command message
  - velocity-limit arm targets at 250 Hz

Start with --test hold. If the robot still collapses during hold, the issue is
not your arm trajectory; it is the robot mode/setup for arm_sdk motion control.
"""

import argparse
import os
import sys
import threading
import time
from enum import IntEnum

import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
SDK_PATH = os.path.join(BASE, "unitree_sdk2_python")
if os.path.isdir(SDK_PATH) and SDK_PATH not in sys.path:
    sys.path.insert(0, SDK_PATH)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


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


G1_29_ARM = [
    G1JointIndex.LeftShoulderPitch,
    G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw,
    G1JointIndex.LeftElbow,
    G1JointIndex.LeftWristRoll,
    G1JointIndex.LeftWristPitch,
    G1JointIndex.LeftWristYaw,
    G1JointIndex.RightShoulderPitch,
    G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,
    G1JointIndex.RightElbow,
    G1JointIndex.RightWristRoll,
    G1JointIndex.RightWristPitch,
    G1JointIndex.RightWristYaw,
]

G1_23_ARM = [
    G1JointIndex.LeftShoulderPitch,
    G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw,
    G1JointIndex.LeftElbow,
    G1JointIndex.LeftWristRoll,
    G1JointIndex.RightShoulderPitch,
    G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,
    G1JointIndex.RightElbow,
    G1JointIndex.RightWristRoll,
]

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


class XrStyleArmMotionController:
    def __init__(self, dof=29, velocity_limit=2.0):
        self.arm_joints = G1_29_ARM if int(dof) == 29 else G1_23_ARM
        self.arm_dim = len(self.arm_joints)
        self.velocity_limit = float(velocity_limit)
        self.control_dt = 1.0 / 250.0
        self.kp_high = 300.0
        self.kd_high = 3.0
        self.kp_low = 80.0
        self.kd_low = 3.0
        self.kp_wrist = 40.0
        self.kd_wrist = 1.5

        self._crc = CRC()
        self._state = None
        self._state_lock = threading.Lock()
        self._state_event = threading.Event()
        self._target_lock = threading.Lock()
        self._stop_event = threading.Event()

        self._target_q = np.zeros(self.arm_dim)
        self._target_tau = np.zeros(self.arm_dim)

        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._sub.Init(self._lowstate_cb, 10)

        if not self._state_event.wait(timeout=5.0):
            raise RuntimeError("未收到 rt/lowstate，无法测试 xr motion arm control")

        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._init_command_from_current_state()
        self._target_q = self.get_current_arm_q()

        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    def _lowstate_cb(self, msg):
        with self._state_lock:
            self._state = msg
        self._state_event.set()

    def _current_state(self):
        with self._state_lock:
            return self._state

    def _init_command_from_current_state(self):
        state = self._current_state()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = state.mode_machine
        arm_set = set(int(j) for j in self.arm_joints)
        for joint in G1JointIndex:
            motor = self._cmd.motor_cmd[int(joint)]
            motor.mode = 1
            motor.q = float(state.motor_state[int(joint)].q)
            motor.dq = 0.0
            motor.tau = 0.0
            if int(joint) in arm_set:
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

    def get_current_arm_q(self):
        state = self._current_state()
        return np.array([state.motor_state[int(j)].q for j in self.arm_joints], dtype=float)

    def set_target(self, q_target, tau_target=None):
        q_target = np.array(q_target, dtype=float)
        if q_target.shape != (self.arm_dim,):
            raise ValueError(f"target dim must be {self.arm_dim}, got {q_target.shape}")
        if tau_target is None:
            tau_target = np.zeros(self.arm_dim)
        with self._target_lock:
            self._target_q = q_target
            self._target_tau = np.array(tau_target, dtype=float)

    def release(self):
        for weight in np.linspace(1.0, 0.0, 101):
            self._cmd.motor_cmd[int(G1JointIndex.NotUsedJoint0)].q = float(weight)
            self._cmd.crc = self._crc.Crc(self._cmd)
            self._pub.Write(self._cmd)
            time.sleep(0.01)
        self._stop_event.set()

    def _clip_target(self, target):
        current = self.get_current_arm_q()
        delta = target - current
        scale = np.max(np.abs(delta)) / max(self.velocity_limit * self.control_dt, 1e-6)
        return current + delta / max(scale, 1.0)

    def _publish_loop(self):
        self._cmd.motor_cmd[int(G1JointIndex.NotUsedJoint0)].q = 1.0
        while not self._stop_event.is_set():
            start = time.time()
            with self._target_lock:
                target_q = self._target_q.copy()
                target_tau = self._target_tau.copy()
            clipped = self._clip_target(target_q)
            for idx, joint in enumerate(self.arm_joints):
                motor = self._cmd.motor_cmd[int(joint)]
                motor.q = float(clipped[idx])
                motor.dq = 0.0
                motor.tau = float(target_tau[idx])
            self._cmd.crc = self._crc.Crc(self._cmd)
            self._pub.Write(self._cmd)
            time.sleep(max(0.0, self.control_dt - (time.time() - start)))


def run_hold_test(ctrl, seconds):
    print(f"[TEST] hold current arm pose for {seconds:.1f}s")
    ctrl.set_target(ctrl.get_current_arm_q())
    time.sleep(seconds)


def run_small_motion_test(ctrl, seconds):
    print(f"[TEST] small sinusoidal right arm motion for {seconds:.1f}s")
    base = ctrl.get_current_arm_q()
    start = time.time()
    while time.time() - start < seconds:
        q = base.copy()
        t = time.time() - start
        if ctrl.arm_dim == 14:
            q[7] += 0.12 * np.sin(2.0 * np.pi * 0.25 * t)
            q[8] += -0.10 * np.sin(2.0 * np.pi * 0.25 * t)
        else:
            q[5] += 0.12 * np.sin(2.0 * np.pi * 0.25 * t)
            q[6] += -0.10 * np.sin(2.0 * np.pi * 0.25 * t)
        ctrl.set_target(q)
        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("network", help="DDS network interface, e.g. eno1")
    parser.add_argument("--dof", choices=["23", "29"], default="29")
    parser.add_argument("--test", choices=["hold", "small"], default="hold")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--velocity-limit", type=float, default=2.0)
    parser.add_argument("--start-loco", action="store_true", help="Call LocoClient.Start() before the test")
    parser.add_argument("--i-understand-risk", action="store_true", help="Required to run on the real robot")
    args = parser.parse_args()

    if not args.i_understand_risk:
        print("Refusing to run without --i-understand-risk.")
        print("Start with: --test hold --seconds 5, with a person ready at the emergency stop.")
        return 2

    ChannelFactoryInitialize(0, args.network)

    if args.start_loco:
        loco = LocoClient()
        loco.SetTimeout(10.0)
        loco.Init()
        loco.Start()
        time.sleep(0.5)

    ctrl = XrStyleArmMotionController(dof=int(args.dof), velocity_limit=args.velocity_limit)
    try:
        if args.test == "hold":
            run_hold_test(ctrl, args.seconds)
        else:
            run_small_motion_test(ctrl, args.seconds)
    finally:
        print("[TEST] releasing arm_sdk weight")
        ctrl.release()
        time.sleep(0.2)

    print("[TEST] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
