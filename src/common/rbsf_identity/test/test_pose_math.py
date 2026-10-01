import math

import pytest
from rbsf_identity.pose_math import compose, inverse, quaternion_multiply, rotate

IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def yaw(angle):
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def test_identity_parent_keeps_the_pose():
    pose = ((1.0, 2.0, 0.0), yaw(0.3))
    translation, rotation = compose(IDENTITY, pose)
    assert translation == pytest.approx(pose[0])
    assert rotation == pytest.approx(pose[1])


def test_rotated_and_shifted_parent():
    # map -> odom: odom is 10 m along x and turned 90 degrees left.
    parent = ((10.0, 0.0, 0.0), yaw(math.pi / 2))
    # 1 m ahead in odom, facing odom's +x
    translation, rotation = compose(parent, ((1.0, 0.0, 0.0), yaw(0.0)))
    assert translation == pytest.approx((10.0, 1.0, 0.0))
    assert rotation == pytest.approx(yaw(math.pi / 2))


def test_yaws_add():
    assert quaternion_multiply(yaw(0.2), yaw(0.5)) == pytest.approx(yaw(0.7))


def test_rotate_vector():
    assert rotate(yaw(math.pi), (1.0, 2.0, 3.0)) == pytest.approx((-1.0, -2.0, 3.0))


def test_inverse_composes_to_identity():
    pose = ((3.0, -1.0, 0.0), yaw(1.1))
    translation, rotation = compose(pose, inverse(pose))
    assert translation == pytest.approx((0.0, 0.0, 0.0))
    assert rotation == pytest.approx((0.0, 0.0, 0.0, 1.0))
