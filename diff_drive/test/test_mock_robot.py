from __future__ import division

import pytest
from diff_drive.mock_robot import MockRobot


PKG = 'test_controller'


@pytest.fixture
def robot():
    return MockRobot()


class TestMockRobot:

    def testNoMotion(self, robot):
        robot.updateRobot(1)
        ticks = robot.getTicks()
        assert ticks.left == 0
        assert ticks.right == 0

        robot.updateRobot(1)
        ticks = robot.getTicks()
        assert ticks.left == 0
        assert ticks.right == 0

    def testStraightLine(self, robot):
        robot.setSpeeds(100, 100)
        robot.updateRobot(1)
        ticks = robot.getTicks()
        assert ticks.left == 0
        assert ticks.right == 0

        robot.updateRobot(1)
        ticks = robot.getTicks()
        assert ticks.left == 100
        assert ticks.right == 100

        robot.updateRobot(0.1)
        ticks = robot.getTicks()
        assert ticks.left == 110
        assert ticks.right == 110

    def testRotateLeft(self, robot):
        robot.setSpeeds(-100, 100)
        robot.updateRobot(0.1)

        robot.updateRobot(0.1)
        ticks = robot.getTicks()
        assert ticks.left == -10
        assert ticks.right == 10

        robot.updateRobot(0.1)
        ticks = robot.getTicks()
        assert ticks.left == -20
        assert ticks.right == 20
