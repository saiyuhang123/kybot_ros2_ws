#!/usr/bin/env python3
"""
WASD 键盘遥控差速小车(发布 /cmd_vel)

用法:
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/car/carrar/install/setup.bash
    python3 teleop_wasd.py

按键:
    w/s : 前进 / 后退
    a/d : 左转 / 右转
    q/e : 前进+左转 / 前进+右转
    空格 : 急停
    +/- : 线速度档位加减
    x 或 Ctrl-C : 退出(退出前自动停车)
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LINEAR_STEP = 0.05      # 每按一次 +/- 线速度增减 (m/s)
ANGULAR_STEP = 0.2      # 每按一次 +/- 角速度增减 (rad/s)
MAX_LINEAR = 1.0
MAX_ANGULAR = 2.5
PUBLISH_HZ = 20         # cmd_vel 持续发布频率(节点侧有 1s 超时保护)

HELP = """
------------------------
 w/s : 前进 / 后退
 a/d : 左转 / 右转
 q/e : 前进+左 / 前进+右
 空格 : 急停
 +/- : 速度档位加 / 减
 x   : 退出
------------------------
"""


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 5)
        self.linear = 0.3     # 当前线速度档位 m/s
        self.angular = 1.0    # 当前角速度档位 rad/s
        self.vx = 0.0         # 当前指令
        self.vth = 0.0

    def set_key(self, key):
        if key == 'w':
            self.vx, self.vth = self.linear, 0.0
        elif key == 's':
            self.vx, self.vth = -self.linear, 0.0
        elif key == 'a':
            self.vx, self.vth = 0.0, self.angular
        elif key == 'd':
            self.vx, self.vth = 0.0, -self.angular
        elif key == 'q':
            self.vx, self.vth = self.linear, self.angular
        elif key == 'e':
            self.vx, self.vth = self.linear, -self.angular
        elif key == ' ':
            self.vx, self.vth = 0.0, 0.0
        elif key in ('+', '='):
            self.linear = min(MAX_LINEAR, self.linear + LINEAR_STEP)
            self.angular = min(MAX_ANGULAR, self.angular + ANGULAR_STEP)
        elif key == '-':
            self.linear = max(LINEAR_STEP, self.linear - LINEAR_STEP)
            self.angular = max(ANGULAR_STEP, self.angular - ANGULAR_STEP)

    def publish(self):
        msg = Twist()
        msg.linear.x = float(self.vx)
        msg.angular.z = float(self.vth)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Teleop()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)

    print(HELP)
    try:
        while rclpy.ok():
            # 非阻塞读键盘,有键就更新指令
            if select.select([sys.stdin], [], [], 1.0 / PUBLISH_HZ)[0]:
                key = sys.stdin.read(1)
                if key in ('x', '\x03'):   # x 或 Ctrl-C
                    break
                node.set_key(key)

            node.publish()
            print(f"\r线速度档位 {node.linear:.2f} m/s | 角速度档位 {node.angular:.1f} rad/s"
                  f" | 当前指令 vx={node.vx:+.2f} w={node.vth:+.2f}   ",
                  end='', flush=True)
    finally:
        # 退出前停车,恢复终端
        node.vx = node.vth = 0.0
        for _ in range(5):
            node.publish()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n已停车,退出。")


if __name__ == '__main__':
    main()
