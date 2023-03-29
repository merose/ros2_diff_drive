from diff_drive import odometry
from diff_drive.util import BaseNode
from diff_drive.pose import Pose
from diff_drive.transformations \
    import euler_from_quaternion, quaternion_from_euler

from diff_drive_interfaces.msg import WheelTicks

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class OdometryPublisher(BaseNode):

    def __init__(self, node):
        super().__init__(node)
        self.odometry = odometry.Odometry()

        self.ticksPerMeter = self.get_parameter('ticks_per_meter',
                                                value_type=float)
        self.wheelSeparation = self.get_parameter('wheel_separation',
                                                  value_type=float)
        self.baseFrameID = self.get_parameter('base_frame_id',
                                              default='base_link')
        self.odomFrameID = self.get_parameter('odom_frame_id', default='odom')
        self.encoderMin = self.get_parameter('encoder_min', default=-32768)
        self.encoderMax = self.get_parameter('encoder_max', default=32767)

        self.tf_broadcaster = TransformBroadcaster(self.node)

        self.odometry.setWheelSeparation(self.wheelSeparation)
        self.odometry.setTicksPerMeter(self.ticksPerMeter)
        self.odometry.setEncoderRange(self.encoderMin, self.encoderMax)
        self.odometry.setTime(self.get_time())

        self.odometry_publisher = self.node.create_publisher(
            Odometry, 'odom', 10)
        self.node.create_subscription(WheelTicks, 'wheel_ticks',
                                      self.on_wheel_ticks, 10)
        self.node.create_subscription(PoseWithCovarianceStamped,
                                      'initialpose', self.on_initial_pose, 10)

    def get_node(self):
        return self.node

    def on_wheel_ticks(self, msg):
        stamp = msg.header.stamp
        now = self.time_from_stamp(stamp)
        self.odometry.updateLeftWheel(msg.left)
        self.odometry.updateRightWheel(msg.right)
        self.odometry.updatePose(now)
        pose = self.odometry.getPose()
        self.log_debug(f'New pose: time={now} x={pose.x} y={pose.y} '
                       + f'theta={pose.theta}')

        q = quaternion_from_euler(0, 0, pose.theta)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odomFrameID
        t.child_frame_id = self.baseFrameID

        t.transform.translation.x = pose.x
        t.transform.translation.y = pose.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
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

        self.log_info(f'Setting initial pose to {pose}')
        self.odometry.setPose(pose)
        self.odometry.setTime(self.time_from_stamp(msg.header.stamp))


def main():
    rclpy.init()
    publisher = OdometryPublisher(Node('diff_drive_odometry'))
    rclpy.spin(publisher.get_node())


if __name__ == '__main__':
    main()
