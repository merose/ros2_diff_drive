from diff_drive.mock_robot import MockRobot
from diff_drive.util import BaseNode

from diff_drive_interfaces.msg import WheelTicks

import rclpy
from rclpy.node import Node


class MockRobotNode(BaseNode):

    def __init__(self, node):
        super().init(node)
        self.robot = MockRobot()
        self.robot.setSpeeds(0, 0)
        self.last_time = self.get_time()

        self.declare_parameter('rate', default=10.0)
        self.declare_parameter('timeout', default=0.5)

        self.rate = self.get_double_parameter('rate')
        self.timeout = self.get_double_parameter('timeout')

        self.pub = self.node.create_publisher(WheelTicks, 'wheel_ticks', 10)
        self.node.create_subscription(WheelTicks, 'wheel_desired_rates',
                                      self.on_desired_rate, 10)
        self.node.create_timer(1/self.rate, self.publish)

    def publish(self):
        newTime = self.get_time()
        diffTime = newTime - self.lastTime
        self.lastTime = newTime

        if diffTime > self.timeout:
            self.robot.setSpeeds(0, 0)

        try:
            self.robot.updateRobot(diffTime)
        except Exception as e:
            self.log_error(f'Got exception updating robot: {e}')

        ticks = self.robot.getTicks()
        msg = WheelTicks()
        msg.header.stamp = self.stamp_from_time(newTime)
        msg.left = ticks.left
        msg.right = ticks.right
        self.pub.publish(msg)

    def on_desired_rates(self, msg):
        self.robot.setSpeeds(msg.left, msg.right)


def main():
    rclpy.init()
    node = MockRobotNode(Node('diff_drive_mock_robot'))
    rclpy.spin(node.get_node())


if __name__ == '__main__':
    main()
