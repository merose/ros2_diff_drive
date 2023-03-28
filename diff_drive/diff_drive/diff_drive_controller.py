#! /usr/bin/env python

from diff_drive.controller import Controller

from diff_drive_interfaces import WheelTicks

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node


class ControllerNode:

    def __init__(self, node):
        super().__init__(node)
        self.controller = Controller()
        self.linearVelocity = 0.0
        self.angularVelocity = 0.0
        self.lastTwistTime = self.get_time()

        self.declare_parameter('ticks_per_meter', value_type=float)
        self.declare_parameter('wheel_separation', value_type=float)
        self.declare_parameter('max_motor_speed', value_type=float)
        self.declare_parameter('rate', default=5.0)
        self.declare_parameter('timeout', default=0.2)

        self.ticksPerMeter = self.get_double_parameter('ticks_per_meter')
        self.wheelSeparation = self.get_double_parameter('wheel_separation')
        self.maxMotorSpeed = self.get_double_parameter('max_motor_speed')
        self.rate = self.get_double_parameter('rate')
        self.timeout = self.get_double_parameter('timeout')

        self.controller.setWheelSeparation(self.wheelSeparation)
        self.controller.setTicksPerMeter(self.ticksPerMeter)
        self.controller.setMaxMotorSpeed(self.maxMotorSpeed)

        self.pub = self.node.create_publisher(WheelTicks, 'wheel_desired_rates',
                                              10)

        self.node.create_subscription(Twist, 'cmd_vel', self.on_twist)

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
