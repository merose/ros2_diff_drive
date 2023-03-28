from geometry_msgs.msg import Twist, PoseStamped

from nav_msgs.msg import Odometry

from std_msgs.msg import Float32, Bool

from diff_drive import goal_controller
from diff_drive import pose
from diff_drive.util import BaseNode
from diff_drive.transformations import euler_from_quaternion

from diff_drive_interfaces.msg import GoToPoseAction, GoToPoseGoal, \
    GoToPoseResult

import rclpy
from rclpy.node import Node


class GoToGoalNode(BaseNode):

    def __init__(self, node):
        super().init(node)
        self.controller = goal_controller.GoalController()
        self.declare_parameter('rate', 10.0)

#        self.action_name = 'diff_drive_go_to_goal'
#        self.action_server \
#            = actionlib.SimpleActionServer(self.action_name, GoToPoseAction,
#                                           execute_cb=self.on_execute,
#                                           auto_start=False)

#        self.action_client = actionlib.SimpleActionClient(
#            'diff_drive_go_to_goal', GoToPoseAction)

        self.dist_pub = self.node.create_publisher(
            Float32, 'distance_to_goal', 10)
        self.twist_pub = self.node.create_publisher(
            Twist, 'cmd_vel', 10)
        self.goal_achieved_pub = self.node.create_publisher(
            Bool, 'goal_achieved', 1)

        self.node.create_subscription(Odometry, 'odom', self.on_odometry)
        self.node.create_subscription(PoseStamped, 'move_base_simple/goal',
                                      self.on_goal)

        rate = self.get_double_parameter('rate')
        
        self.rate = rospy.Rate(rate)

        self.dT = 1 / rate

        self.kP = rospy.get_param('~kP', 3.0)
        self.kA = rospy.get_param('~kA', 8.0)
        self.kB = rospy.get_param('~kB', -1.5)
        self.controller.set_constants(self.kP, self.kA, self.kB)

        self.controller.set_linear_tolerance(
            rospy.get_param('~linear_tolerance', 0.05))
        self.controller.set_angular_tolerance(
            rospy.get_param('~angular_tolerance', 3/180*pi))

        self.controller.set_max_linear_speed(
            rospy.get_param('~max_linear_speed', 0.2))
        self.controller.set_min_linear_speed(
            rospy.get_param('~min_linear_speed', 0))
        self.controller.set_max_angular_speed(
            rospy.get_param('~max_angular_speed', 1.0))
        self.controller.set_min_angular_speed(
            rospy.get_param('~min_angular_speed', 0))
        self.controller.set_max_linear_acceleration(
            rospy.get_param('~max_linear_acceleration', 0.1))
        self.controller.set_max_angular_acceleration(
            rospy.get_param('~max_angular_acceleration', 0.3))

        # Set whether to allow movement backward. Backward movement is
        # safe if the robot can avoid obstacles while traveling in
        # reverse. We default to forward movement only since many
        # sensors are front-facing.
        self.controller.set_forward_movement_only(
            rospy.get_param('~forwardMovementOnly', True))

        self.init_pose()
        self.goal = None

        self.node.create_timer(1/self.rate, self.publish)
#        self.action_server.start()

    def on_execute(self, goal):
        self.goal = self.get_angle_pose(goal.pose.pose)
        rospy.loginfo('Goal: (%f,%f,%f)', self.goal.x, self.goal.y,
                      self.goal.theta)

        success = True
        while not rospy.is_shutdown() and self.goal is not None:
            # Allow client to preempt the goal.
            if self.action_server.is_preempt_requested():
                rospy.loginfo('Goal preempted')
                self.send_velocity(0, 0)
                self.action_server.set_preempted()
                success = False
                break
            self.publish()
            self.rate.sleep()

        result = GoToPoseResult()
        result.success = success
        self.action_server.set_succeeded(result)

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
        self.dist_pub.publish(d)

        self.send_velocity(desired.xVel, desired.thetaVel)

        # Forget the goal if achieved.
        if self.controller.at_goal(self.pose, self.goal):
            rospy.loginfo('Goal achieved')
            self.goal = None
            msg = Bool()
            msg.data = True
            self.goal_achieved_pub.publish(msg)

    def send_velocity(self, xVel, thetaVel):
        twist = Twist()
        twist.linear.x = xVel
        twist.angular.z = thetaVel
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
