from __future__ import division, print_function

import pytest
from math import pi, sin, cos
from diff_drive.goal_controller import GoalController
from diff_drive.pose import Pose


PKG = 'test_goal_controller'


@pytest.fixture
def controller():
    return GoalController()


class TestGoalController:

    # Test that the robot does not move when already at the goal
    # at the right heading.
    def testAtGoal(self, controller):
        cur = Pose()
        desired = controller.get_velocity(cur, cur, 0.1)
        assert desired.xVel == 0
        assert desired.thetaVel == 0

    # Test that a goal pose ahead of the current position at the same
    # heading causes a straight-ahead move.
    def testStraightAhead(self, controller):
        cur = Pose()
        goal = Pose()
        goal.x = 1
        desired = controller.get_velocity(cur, goal, 0.1)
        assert desired.xVel > 0
        assert desired.thetaVel == 0

    # Test that a goal pose behind the current position at the same
    # heading causes a straight-back move.
    def testStraightBack(self, controller):
        cur = Pose()
        goal = Pose()
        goal.x = -1
        desired = controller.get_velocity(cur, goal, 0.1)
        assert desired.xVel < 0
        assert desired.thetaVel == 0

    # Test that a goal at the current position with a leftward goal
    # heading causes a leftward rotation.
    def testRotateLeft(self, controller):
        cur = Pose()
        goal = Pose()
        goal.theta = pi/2
        desired = controller.get_velocity(cur, goal, 0.1)
        assert desired.xVel == 0
        assert desired.thetaVel > 0

    # Test that a goal at the current position with a rightward goal
    # heading causes a rightward rotation.
    def testRotateRight(self, controller):
        cur = Pose()
        goal = Pose()
        goal.theta = -pi/2
        desired = controller.get_velocity(cur, goal, 0.1)
        assert desired.xVel == 0
        assert desired.thetaVel < 0

    # Test that a goal pose that is reachable with a forward, leftward
    # arc causes a forward movement with some leftward rotation.
    def testCurveLeft(self, controller):
        cur = Pose()
        goal = Pose()
        goal.x = 1
        goal.y = 1
        goal.theta = pi/2
        desired = controller.get_velocity(cur, goal, 0.1)
        assert desired.xVel > 0
        assert desired.thetaVel > 0

    # Test that a goal pose that is reachable with a forward, rightward
    # arc causes a forward movement with some rightward rotation.
    def testCurveRight(self, controller):
        cur = Pose()
        cur.theta = pi/2
        goal = Pose()
        goal.x = 1
        goal.y = 1
        desired = controller.get_velocity(cur, goal, 0.1)
        assert desired.xVel > 0
        assert desired.thetaVel < 0

    # Test that a goal pose behind the robot that is reachable with a
    # leftward arc causes a backward movement with some rightward
    # rotation.
    def testCurveBackLeft(self, controller):
        cur = Pose()
        goal = Pose()
        cur.x = 1
        cur.y = 1
        goal.theta = pi/2
        desired = controller.get_velocity(cur, goal, 0.1)
        assert desired.xVel < 0
        assert desired.thetaVel > 0

    # Test that a goal pose behind the robot that is reachable with a
    # rightward arc causes a backward movement with some leftward
    # rotation.
    def testCurveBackRigth(self, controller):
        cur = Pose()
        goal = Pose()
        cur.x = 1
        cur.y = 1
        cur.theta = pi/2
        desired = controller.get_velocity(cur, goal, 0.1)
        assert desired.xVel < 0
        assert desired.thetaVel < 0

    def testButtonHookLeft(self, controller):
        cur = Pose()
        goal = Pose()
        goal.x = 1
        goal.y = 1
        goal.theta = pi
        desired = controller.get_velocity(cur, goal, 0.1)
        assert desired.xVel > 0
        assert desired.thetaVel > 0

    def testButtonHookRight(self, controller):
        cur = Pose()
        goal = Pose()
        goal.x = 1
        goal.y = -1
        goal.theta = -pi
        desired = controller.get_velocity(cur, goal, 0.1)
        assert desired.xVel > 0
        assert desired.thetaVel < 0

    def testGoToGoal(self, controller):
        self.checkGoToGoal(controller, 0, 0, 0, 1, 0, 0)  # Straight ahead
        self.checkGoToGoal(controller, 0, 0, 0, 1, 1, pi/2)  # Arc left
        self.checkGoToGoal(controller, 0, 0, 0, 1, 1, -pi/2)  # Left, then turn right
        self.checkGoToGoal(controller, 0, 0, 0, 0, 1, pi)  # Go left
        self.checkGoToGoal(controller, 0, 0, 0, 1, 0, pi)  # Go ahead and u-turn
        self.checkGoToGoal(controller, 0, 0, 0, -1, 0, 0)  # Straight back
        self.checkGoToGoal(controller, 0, 0, 0, -1, -1, 0)  # Back up to right
        self.checkGoToGoal(controller, 0, 0, 0, -1, -1, pi)  # Back up and turn left

    def testGoToGoalForwardOnly(self, controller):
        controller.set_forward_movement_only(True)
        self.checkGoToGoal(controller, 0, 0, 0, 1, 0, 0)  # Straight ahead
        self.checkGoToGoal(controller, 0, 0, 0, 1, 1, pi/2)  # Arc left
        self.checkGoToGoal(controller, 0, 0, 0, 1, 1, -pi/2)  # Left, then turn right
        self.checkGoToGoal(controller, 0, 0, 0, 0, 1, pi)  # Go left
        self.checkGoToGoal(controller, 0, 0, 0, 1, 0, pi)  # Go ahead and u-turn
        self.checkGoToGoal(controller, 0, 0, 0, -1, 0, 0)  # Straight back
        self.checkGoToGoal(controller, 0, 0, 0, -1, -1, 0)  # Back up to right
        self.checkGoToGoal(controller, 0, 0, 0, -1, -1, pi)  # Back up and turn left

    def checkGoToGoal(self, controller, x0, y0, th0, x1, y1, th1):
        # 5cm
        dTol = 0.05
        # Approx 2.5 degrees
        thTol = 0.04

        controller.set_linear_tolerance(dTol)
        controller.set_angular_tolerance(thTol)

        cur = Pose()
        cur.x = x0
        cur.y = y0
        cur.theta = th0

        goal = Pose()
        goal.x = x1
        goal.y = y1
        goal.theta = th1

        dT = 0.05
        for i in range(1000):
            if controller.at_goal(cur, goal):
                return

            desired = controller.get_velocity(cur, goal, dT)
            newTheta = cur.theta + dT*desired.thetaVel
            midTheta = (cur.theta + newTheta) / 2.0
            cur.x += dT * desired.xVel * cos(midTheta)
            cur.y += dT * desired.xVel * sin(midTheta)
            cur.theta = newTheta

        # If we get here, we didn't reach the goal.
        assert False, f'Did not reach the goal: p0={(x0,y0,th0)} ' \
            + f'p1={(x1,y1,th1)}'
