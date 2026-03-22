#!/usr/bin/env python3
"""
motor_response.py — shows /cmd_vel (joystick input) vs /wheel/odom (actual motor
response) side by side with timestamps so you can see lag and erratic behavior.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 ~/Documents/PortaMailCapstone/motor_response.py
"""
import rclpy, threading, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MotorResponse(Node):
    def __init__(self):
        super().__init__('motor_response')
        self.create_subscription(Twist,    '/cmd_vel',    self.cmd_cb,  10)
        self.create_subscription(Odometry, '/wheel/odom', self.odom_cb, 10)
        self.last_cmd_linear  = 0.0
        self.last_cmd_angular = 0.0
        self.last_odom_linear  = 0.0
        self.last_odom_angular = 0.0
        self.start = time.monotonic()
        print(f"{'TIME(s)':<8} {'CMD_LINEAR':>12} {'CMD_ANG':>10} {'ODOM_LINEAR':>12} {'ODOM_ANG':>10}  NOTE")
        print("-" * 70)

    def ts(self):
        return f"{time.monotonic() - self.start:7.2f}"

    def cmd_cb(self, msg):
        lin = round(msg.linear.x, 3)
        ang = round(msg.angular.z, 3)
        note = ""
        if abs(lin) > 0.01 or abs(ang) > 0.01:
            note = "<-- CMD"
        elif (abs(self.last_cmd_linear) > 0.01 or abs(self.last_cmd_angular) > 0.01):
            note = "<-- CMD ZERO"
        self.last_cmd_linear  = lin
        self.last_cmd_angular = ang
        if note or abs(lin) > 0.01 or abs(ang) > 0.01:
            print(f"{self.ts():<8} {lin:>12.3f} {ang:>10.3f} {self.last_odom_linear:>12.3f} {self.last_odom_angular:>10.3f}  {note}")

    def odom_cb(self, msg):
        lin = round(msg.twist.twist.linear.x, 3)
        ang = round(msg.twist.twist.angular.z, 3)
        note = ""
        if abs(lin) > 0.01 or abs(ang) > 0.01:
            note = "        --> MOTOR MOVING"
        elif (abs(self.last_odom_linear) > 0.01 or abs(self.last_odom_angular) > 0.01):
            note = "        --> MOTOR STOPPED"
        self.last_odom_linear  = lin
        self.last_odom_angular = ang
        if note:
            print(f"{self.ts():<8} {self.last_cmd_linear:>12.3f} {self.last_cmd_angular:>10.3f} {lin:>12.3f} {ang:>10.3f}  {note}")

def main():
    rclpy.init()
    node = MotorResponse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
