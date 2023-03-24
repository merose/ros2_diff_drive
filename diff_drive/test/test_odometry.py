#! /usr/bin/env python
from __future__ import division

import pytest
from math import pi
from diff_drive.odometry import Odometry


PKG = 'test_odometry'


class TestOdometry:

    @pytest.fixture
    def setup(self):
        self.wheelSeparation = 0.10
        self.ticksPerMeter = 10000
        self.odom = Odometry()
        self.odom.setWheelSeparation(self.wheelSeparation)
        self.odom.setTicksPerMeter(self.ticksPerMeter)

    def testInitialization(self, setup):
        pose = self.odom.getPose()
        assert pose.x == 0
        assert pose.y == 0
        assert pose.theta == 0
        assert pose.xVel == 0
        assert pose.yVel == 0
        assert pose.thetaVel == 0

    def testTravelForward(self, setup):
        self.checkUpdate(10000, 10000, 2, {'x': 1, 'xVel': 1/2})

    def testSpinLeft(self, setup):
        angle = (200/self.ticksPerMeter / self.wheelSeparation) % (2*pi)
        self.checkUpdate(-100, 100, 2,
                         {'theta': angle, 'thetaVel': angle/2})

    def testSpinRight(self, setup):
        angle = (200/self.ticksPerMeter / self.wheelSeparation) % (2*pi)
        self.checkUpdate(100, -100, 2,
                         {'theta': (-angle) % (2*pi),
                          'thetaVel': -angle/2})

    def testCurveLeft(self, setup):
        radius = self.wheelSeparation / 2
        angle = pi
        s = angle * self.wheelSeparation
        ticks = int(s * self.ticksPerMeter)
        self.checkUpdate(0, ticks, 2,
                         {'x': 0,
                          'y': 2*radius,
                          'theta': angle,
                          'xVel': s/4,
                          'yVel': 0,
                          'thetaVel': angle/2})

    def testCurveRight(self, setup):
        radius = self.wheelSeparation / 2
        angle = pi
        s = angle * self.wheelSeparation
        ticks = int(s * self.ticksPerMeter)
        self.checkUpdate(ticks, 0, 2,
                         {'x': 0,
                          'y': -2*radius,
                          'theta': -angle,
                          'xVel': s/4,
                          'yVel': 0,
                          'thetaVel': -angle/2})

    def testCircle(self, setup):
        self.checkCircle(8, 9)
        self.checkCircle(0, 100)
        self.checkCircle(100, 0)

    def checkCircle(self, vr, vl):
        radius = abs(self.wheelSeparation/2 * (vr+vl)/(vr-vl))
        circumference = 2*pi*radius
        deltaTravel = (vr+vl)/2 * self.ticksPerMeter
        for i in range(int(circumference/deltaTravel)):
            self.odom.updateLeftWheel(vl)
            self.odom.updateRightWheel(vr)
            self.odom.updatePose(i+1)
        self.checkPose(self.odom.getPose(), {'x': 0, 'y': 0})

    def checkUpdate(self, leftTicks, rightTicks, deltaTime, attrs):
        self.odom.updateLeftWheel(leftTicks)
        self.odom.updateRightWheel(rightTicks)
        self.odom.updatePose(deltaTime)
        self.checkPose(self.odom.getPose(), attrs)

    def checkPose(self, pose, attrs):
        for key in ['x', 'y', 'theta', 'xVel', 'yVel', 'thetaVel']:
            if key in attrs:
                self.assertClose(
                    getattr(pose, key), attrs[key],
                    msg="{0}: {1}!={2}".format(key,
                                               getattr(pose, key),
                                               attrs[key]))
            else:
                assert getattr(pose, key) == 0, key

    def assertClose(self, x, y, msg):
        if y == 0:
            assert abs(x) < 0.0001, msg
        else:
            assert abs(x-y)/y < 0.001, msg
