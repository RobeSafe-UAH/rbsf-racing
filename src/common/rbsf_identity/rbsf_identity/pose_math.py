"""Rigid-transform composition on plain tuples (no ROS imports, unit-tested)."""


def quaternion_multiply(a, b):
    """Hamilton product a * b of quaternions given as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate(q, v):
    """Rotate vector v = (x, y, z) by the unit quaternion q = (x, y, z, w)."""
    qx, qy, qz, qw = quaternion_multiply(quaternion_multiply(q, (*v, 0.0)),
                                         (-q[0], -q[1], -q[2], q[3]))
    return qx, qy, qz


def compose(parent, child):
    """
    Compose two poses: `parent` (frame A -> B) followed by `child` (frame B -> C), giving A -> C.

    Each pose is ((x, y, z), (qx, qy, qz, qw)).
    """
    (pt, pq), (ct, cq) = parent, child
    rotated = rotate(pq, ct)
    translation = tuple(p + r for p, r in zip(pt, rotated))
    return translation, quaternion_multiply(pq, cq)


def inverse(pose):
    """Inverse of a pose ((x, y, z), (qx, qy, qz, qw)): B -> A from A -> B."""
    (t, q) = pose
    q_inv = (-q[0], -q[1], -q[2], q[3])
    return tuple(-v for v in rotate(q_inv, t)), q_inv
