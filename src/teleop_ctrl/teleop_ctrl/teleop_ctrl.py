#!/usr/bin/env python3
# encoding: utf-8
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys, select, os
import tty, termios


settings = termios.tcgetattr(sys.stdin)
LIN_VEL = 0.21  # 0.5 gz
STEER = 0.36    # 0.8 gz
ANG_VEL = LIN_VEL/(0.145/math.tan(STEER))

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
CTRL-C to quit
"""

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopControl(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.cmd_vel = self.create_publisher(Twist,"controller/cmd_vel", 1)

        control_linear_vel = 0.0
        control_angular_vel = 0.0
        last_x = 0
        last_z = 0
        count = 0

        try:
            print(msg)
            while rclpy.ok():
                key = getKey(settings)
                
                if key == 'w':
                    count = 0
                    control_linear_vel = LIN_VEL
                elif key == 'a':
                    count = 0
                    control_angular_vel = ANG_VEL
                elif key == 'd':
                    count = 0
                    control_angular_vel = -ANG_VEL
                elif key == 's':
                    count = 0
                    control_linear_vel = -LIN_VEL
                elif key == '':
                    count += 1
                    if count > 5:
                        count = 0
                        if control_angular_vel != 0:
                            control_angular_vel = 0.0
                            control_linear_vel = 0.0
                else:
                    count = 0
                    if (key == '\x03'):
                        break
                twist = Twist()

                twist.linear.x = control_linear_vel
                twist.linear.y = 0.0
                twist.linear.z = 0.0

                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = control_angular_vel

                if last_x != control_linear_vel or last_z != control_angular_vel or control_angular_vel != 0:
                    self.cmd_vel.publish(twist)
                
                last_x = control_linear_vel
                last_z = control_angular_vel
        except BaseException as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.cmd_vel.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main():
    node = TeleopControl('teleop_control')
    rclpy.spin(node)

if __name__ == "__main__":
    main()
