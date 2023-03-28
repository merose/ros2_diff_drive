from math import sin, cos

from diff_drive import odometry
from diff_drive.util import BaseNode
from diff_drive.pose import Pose
from diff_drive.transformations \
    import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node


class OdometryPublisher(BaseNode):

    def __setup__(self, node):
        self.node = node
        self.odometry = odometry.Odometry()
        self.odometry_publisher = self.node.create_publisher(
            Odometry, 'odom', 10)
        self.node.create_subscription(WheelTicks, 'wheel_ticks',
                                      self.on_wheel_ticks)
        self.node.create_subscription(PoseWithCovarianceStamped,
                                      'initialpose', self.on_initial_pose)

    def get_node(self):
        return self.node

    def publish_odometry(self, stamp):
        self.odometry.updatePose(stamp)
        now = rospy.get_rostime()
        pose = self.odometry.getPose()

        q = quaternion_from_euler(0, 0, pose.theta)
        self.tfPub.sendTransform(
            (pose.x, pose.y, 0),
            (q[0], q[1], q[2], q[3]),
            now,
            self.baseFrameID,
            self.odomFrameID
        )

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odomFrameID
        odom.child_frame_id = self.baseFrameID
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = pose.xVel
        odom.twist.twist.angular.z = pose.thetaVel
        self.odometry_publisher.publish(odom)

    def on_initial_pose(self, msg):
        q = (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(q)

        pose = Pose()
        pose.x = msg.pose.pose.position.x
        pose.y = msg.pose.pose.position.y
        pose.theta = yaw

        rospy.loginfo('Setting initial pose to %s', pose)
        self.odometry.setPose(pose)

    def on_wheel_ticks(self, msg):
        self.odometry.updateLeftWheel(msg.left)
        self.odometry.updateRightWheel(msg.right)
        self.publish_odometry(msg.header.stamp)


def main():
    rclpy.init()
    node = Node('diff_drive_odometry')
    publisher = OdometryPublisher(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
