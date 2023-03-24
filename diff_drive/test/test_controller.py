from __future__ import division

import pytest
from diff_drive.controller import Controller
from math import isclose


PKG = 'test_controller'


class TestController:

    @pytest.fixture
    def setup(self):
        self.ticksPerMeter = 10000
        self.wheelSeparation = 0.1
        self.controller = Controller()
        self.controller.setTicksPerMeter(self.ticksPerMeter)
        self.controller.setWheelSeparation(self.wheelSeparation)

    def testStraightForward(self, setup):
        speeds = self.controller.getSpeeds(0.1, 0)
        assert isclose(speeds.left, self.ticksPerMeter*0.1)
        assert isclose(speeds.right, self.ticksPerMeter*0.1)

    def testStraightBackward(self, setup):
        speeds = self.controller.getSpeeds(-0.1, 0)
        assert isclose(speeds.left, -self.ticksPerMeter*0.1)
        assert isclose(speeds.right, -self.ticksPerMeter*0.1)

    def testRotateLeft(self, setup):
        speeds = self.controller.getSpeeds(0, 1)
        diffTicks = self.wheelSeparation * self.ticksPerMeter
        assert isclose(speeds.left, -diffTicks)
        assert isclose(speeds.right, diffTicks)

    def testRotateRight(self, setup):
        speeds = self.controller.getSpeeds(0, -1)
        diffTicks = self.wheelSeparation * self.ticksPerMeter
        assert isclose(speeds.left, diffTicks)
        assert isclose(speeds.right, -diffTicks)

    def testCurveLeft(self, setup):
        speeds = self.controller.getSpeeds(0.1, 1)
        aheadTicks = 0.1 * self.ticksPerMeter
        diffTicks = self.wheelSeparation * self.ticksPerMeter
        assert isclose(speeds.left, aheadTicks-diffTicks)
        assert isclose(speeds.right, aheadTicks+diffTicks)

    def testMotorLimitsStraight(self, setup):
        maxTickSpeed = self.ticksPerMeter // 4
        self.controller.setMaxMotorSpeed(maxTickSpeed)
        speeds = self.controller.getSpeeds(1, 0)
        assert speeds.left == maxTickSpeed
        assert speeds.right == maxTickSpeed

    def testMotorLimitsCurved(self, setup):
        maxTickSpeed = self.ticksPerMeter // 4
        self.controller.setMaxMotorSpeed(maxTickSpeed)
        speeds = self.controller.getSpeeds(1, 1)
        aheadTicks = self.ticksPerMeter
        diffTicks = self.wheelSeparation * self.ticksPerMeter
        factor = maxTickSpeed / (aheadTicks + diffTicks)
        assert speeds.left == int((aheadTicks-diffTicks) * factor)
        assert speeds.right == maxTickSpeed
