#! /usr/bin/env python

from diff_drive.controller import Controller
from diff_drive.util import BaseNode

from diff_drive_interfaces.msg import WheelTicks

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node


class ControllerNode(BaseNode):

    def __init__(self, node):
        super().__init__(node)
        self.controller = Controller()
        self.linearVelocity = 0.0
        self.angularVelocity = 0.0
        self.lastTwistTime = self.get_time()

        self.ticksPerMeter = self.get_parameter('ticks_per_meter',
                                                value_type=float)
        self.wheelSeparation = self.get_parameter('wheel_separation',
                                                  value_type=float)
        self.maxMotorSpeed = self.get_parameter('max_motor_speed',
                                                value_type=float)
        self.rate = self.get_parameter('rate', default=5.0)
        self.timeout = self.get_parameter('timeout', default=0.2)

        self.controller.setWheelSeparation(self.wheelSeparation)
        self.controller.setTicksPerMeter(self.ticksPerMeter)
        self.controller.setMaxMotorSpeed(self.maxMotorSpeed)

        self.pub = self.node.create_publisher(WheelTicks, 'wheel_desired_rates',
                                              10)

        self.node.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)

        self.node.create_timer(1/self.rate, self.publish)

    def publish(self):
        now = self.get_time()
        msg = WheelTicks()
        msg.header.stamp = self.stamp_from_time(now)
        if now - self.lastTwistTime < self.timeout:
            speeds = self.controller.getSpeeds(self.linearVelocity,
                                               self.angularVelocity)
            msg.left = int(speeds.left)
            msg.right = int(speeds.right)
        else:
            msg.left = 0
            msg.right = 0
        self.pub.publish(msg)

    def on_twist(self, twist):
        self.linearVelocity = twist.linear.x
        self.angularVelocity = twist.angular.z
        self.lastTwistTime = self.get_time()


def main():
    rclpy.init()
    node = ControllerNode(Node('diff_drive_mock_controller'))
    rclpy.spin(node.get_node())


if __name__ == '__main__':
    main()
