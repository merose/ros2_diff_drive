"""
Transformations of rotations between representations.

Adapted from Wikipedia:
https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
"""

from math import atan2, cos, pi, sin, sqrt


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to a quaternion."""
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)

    # x, y, z, w
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    )


def euler_from_quaternion(q):
    qx, qy, qz, qw = (q[0], q[1], q[2], q[3])
    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = sqrt(1 + 2 * (qw * qy - qx * qz))
    cosp = sqrt(1 - 2 * (qw * qy - qx * qz))
    pitch = 2 * atan2(sinp, cosp) - pi / 2

    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
