from math import pi

from geometry_msgs.msg import Twist, PoseStamped

from nav_msgs.msg import Odometry

from std_msgs.msg import Float32, Bool

from diff_drive import goal_controller
from diff_drive import pose
from diff_drive.util import BaseNode
from diff_drive.transformations import euler_from_quaternion

#from diff_drive_interfaces.msg import GoToPoseAction, GoToPoseGoal, \
#    GoToPoseResult

import rclpy
from rclpy.node import Node


class GoToGoalNode(BaseNode):

    def __init__(self, node):
        super().__init__(node)
        self.controller = goal_controller.GoalController()
        self.rate = self.get_parameter('rate', default=10.0)
        self.dT = 1 / self.rate
        self.kP = self.get_parameter('kP', default=3.0)
        self.kA = self.get_parameter('kA', default=8.0)
        self.kB = self.get_parameter('kB', default=-1.5)

        self.dist_pub = self.node.create_publisher(
            Float32, 'distance_to_goal', 10)
        self.twist_pub = self.node.create_publisher(
            Twist, 'cmd_vel', 10)
        self.goal_achieved_pub = self.node.create_publisher(
            Bool, 'goal_achieved', 1)

        self.node.create_subscription(Odometry, 'odom', self.on_odometry, 10)
        self.node.create_subscription(PoseStamped, 'move_base_simple/goal',
                                      self.on_goal, 10)

        self.controller.set_constants(self.kP, self.kA, self.kB)

        self.controller.set_linear_tolerance(
            self.get_parameter('linear_tolerance', default=0.05))
        self.controller.set_angular_tolerance(
            self.get_parameter('angular_tolerance', default=3/180*pi))

        self.controller.set_max_linear_speed(
            self.get_parameter('max_linear_speed', default=0.2))
        self.controller.set_min_linear_speed(
            self.get_parameter('min_linear_speed', default=0.0))
        self.controller.set_max_angular_speed(
            self.get_parameter('max_angular_speed', default=1.0))
        self.controller.set_min_angular_speed(
            self.get_parameter('min_angular_speed', default=0.0))
        self.controller.set_max_linear_acceleration(
            self.get_parameter('max_linear_acceleration', default=0.1))
        self.controller.set_max_angular_acceleration(
            self.get_parameter('max_angular_acceleration', default=0.3))

        # Set whether to allow movement backward. Backward movement is
        # safe if the robot can avoid obstacles while traveling in
        # reverse. We default to forward movement only since many
        # sensors are front-facing.
        self.controller.set_forward_movement_only(
            self.get_parameter('forwardMovementOnly', default=True))

        self.init_pose()
        self.goal = None

        self.node.create_timer(1/self.rate, self.publish)

    def init_pose(self):
        self.pose = pose.Pose()
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0

    def publish(self):
        if self.controller.at_goal(self.pose, self.goal):
            desired = pose.Pose()
        else:
            desired = self.controller.get_velocity(self.pose, self.goal,
                                                   self.dT)

        # if self.goal is not None \
        #    and (desired.xVel!=0.0 or desired.thetaVel!=0.0):
        #     rospy.loginfo(
        #         'current=(%f,%f,%f) goal=(%f,%f,%f)  xVel=%f thetaVel=%f',
        #         self.pose.x, self.pose.y, self.pose.theta,
        #         self.goal.x, self.goal.y, self.goal.theta,
        #         desired.xVel, desired.thetaVel)

        d = self.controller.get_goal_distance(self.pose, self.goal)
        msg = Float32()
        msg.data = float(d)
        self.dist_pub.publish(msg)

        self.send_velocity(desired.xVel, desired.thetaVel)

        # Forget the goal if achieved.
        if self.controller.at_goal(self.pose, self.goal):
            self.log_info('Goal achieved')
            self.goal = None
            msg = Bool()
            msg.data = True
            self.goal_achieved_pub.publish(msg)

    def send_velocity(self, xVel, thetaVel):
        twist = Twist()
        twist.linear.x = float(xVel)
        twist.angular.z = float(thetaVel)
        self.twist_pub.publish(twist)

    def on_odometry(self, newPose):
        self.pose = self.get_angle_pose(newPose.pose.pose)

    def on_goal(self, goal):
        self.action_client.wait_for_server()
        action_goal = GoToPoseGoal()
        action_goal.pose.pose = goal.pose
        self.action_client.send_goal(action_goal)

    def get_angle_pose(self, quaternion_pose):
        q = [quaternion_pose.orientation.x,
             quaternion_pose.orientation.y,
             quaternion_pose.orientation.z,
             quaternion_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        angle_pose = pose.Pose()
        angle_pose.x = quaternion_pose.position.x
        angle_pose.y = quaternion_pose.position.y
        angle_pose.theta = yaw
        return angle_pose


def main():
    rclpy.init()
    publisher = GoToGoalNode(Node('diff_drive_go_to_goal'))
    rclpy.spin(publisher.get_node())


if __name__ == '__main__':
    main()
