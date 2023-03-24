#! /usr/env python
from __future__ import division

import pytest
from diff_drive.encoder import Encoder


PKG = 'test_encoder'


@pytest.fixture
def encoder():
    return Encoder()


class TestEncoder:

    def testInitialization(self, encoder):
        assert encoder.getDelta() == 0

    def testClearedDelta(self, encoder):
        encoder.update(100)
        assert encoder.getDelta() == 100
        assert encoder.getDelta() == 0

    def testIncrement(self, encoder):
        encoder.update(100)
        assert encoder.getDelta() == 100

        encoder.update(50)
        assert encoder.getDelta() == -50

    def testWraparound(self, encoder):
        defaultRange = 32767 - (-32768) + 1
        encoder.update(20000)
        assert encoder.getDelta() == 20000

        # Wrap around the high end.
        encoder.update(-20000)
        assert encoder.getDelta() == -20000 + defaultRange - 20000

        # Wrap around the low end.
        encoder.update(20000)
        assert encoder.getDelta() == 20000 - defaultRange - (-20000)

    def testCustomRange(self, encoder):
        encoder.setRange(0, 999)
        encoder.update(500)
        assert encoder.getDelta() == 500

        encoder.update(900)
        assert encoder.getDelta() == 400

        # Wrap around the high end.
        encoder.update(100)
        assert encoder.getDelta() == 200

        # Wrap around the low end.
        encoder.update(900)
        assert encoder.getDelta() == -200
