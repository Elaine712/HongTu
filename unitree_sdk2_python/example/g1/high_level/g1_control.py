#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

class CmdVelController:
    def __init__(self, network_interface):
        # 初始化 Unitree SDK
        rospy.loginfo("Initializing Unitree LocoClient...")
        ChannelFactoryInitialize(0, network_interface)

        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        # 切换到行走模式（FSM=200），使机器人能够响应运动指令
        rospy.loginfo("Switching to walking mode (Start)...")
        self.sport_client.Start()
        rospy.sleep(0.5)
        rospy.loginfo("Robot is in walking mode.")

        self.can_move = False  # 标志位：是否可以开始运动

        # 订阅全局路径
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_callback)
        rospy.loginfo("Subscribed to global path topic /move_base/GlobalPlanner/plan")

        # 订阅 /cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.loginfo("Subscribed to /cmd_vel")

    def path_callback(self, msg: Path):
        if len(msg.poses) > 0:
            if not self.can_move:
                rospy.loginfo("Global path received, robot can start moving.")
            self.can_move = True
        else:
            if self.can_move:
                rospy.logwarn("Global path is empty, robot cannot move.")
            self.can_move = False

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x      # 前后移动
        vy = msg.linear.y      # 横向移动
        wz = msg.angular.z     # 旋转

        # 1. 如果因为没有全局路径而不能动，拒绝移动指令并确保机器人停止
        if not self.can_move:
            rospy.logwarn_throttle(2.0, "Global path not received or empty. Ignoring cmd_vel.")
            # 向底层发送全0速度确保安全（防止断开路径时机器人还在往前冲）
            try:
                self.sport_client.Move(0.0, 0.0, 0.0, continous_move=True)
            except Exception as e:
                pass
            return

        # 2. 正常接收速度指令，包括全0的刹车指令 (不再改变 self.can_move 的状态)
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            rospy.loginfo_throttle(2.0, "Received zero cmd_vel. Robot stopping.")
        else:
            rospy.loginfo_throttle(2.0, f"Moving: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")

        # 3. 将速度下发给 Unitree 底层
        try:
            self.sport_client.Move(vx, vy, wz, continous_move=True)
        except Exception as e:
            rospy.logerr_throttle(2.0, f"Failed to send Move command: {e}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 g1_control.py networkInterface")
        sys.exit(-1)

    network_interface = sys.argv[1]

    rospy.init_node("unitree_cmd_vel_controller", anonymous=False)
    rospy.logwarn("Make sure the robot is in a safe environment before sending cmd_vel commands!")

    controller = CmdVelController(network_interface)

    rospy.spin()
