import numpy as np
import casadi as cs

from scipy.spatial.transform import Rotation


def quat_mult(A1, A2):
    """
    Multiplies two quaternions

    Args:
        A1: Quaternion 1
        A2: Quaternion 2

    Returns:
        A: A1*A2

    """

    a1 = A1[0]
    b1 = A1[1]
    c1 = A1[2]
    d1 = A1[3]
    a2 = A2[0]
    b2 = A2[1]
    c2 = A2[2]
    d2 = A2[3]
    A = [
        a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2,
        a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2,
        a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2,
        a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2,
    ]
    if isinstance(A1, np.ndarray):
        return np.array(A)
    elif isinstance(A1, (cs.SX, cs.MX)):
        return cs.vertcat(A[0], A[1], A[2], A[3])


def quat_AiB(A, B):
    """
    Calculates pure vector quaternion by multiplying A*i*B, where i=[0,1,0,0]

    Args:
        A: Quaternion 1
        B: Quaternion 2

    Returns:
        AiB: A*i*B

    """

    i = [0, 1, 0, 0]
    return quat_mult(quat_mult(A, i), B)


def quat_conj(A):
    """
    Calculates conjugate of quaternion A

    Args:
        A: Quaternion

    Returns:
        conjA: Conjugate of quaternion

    """
    cA0 = A[0]
    cA1 = -A[1]
    cA2 = -A[2]
    cA3 = -A[3]
    cA = [cA0, cA1, cA2, cA3]
    if isinstance(A, np.ndarray):
        return np.array(cA)
    elif isinstance(A, (cs.SX, cs.MX)):
        return cs.vertcat(cA[0], cA[1], cA[2], cA[3])


def quat_AiconjA(A, v="i"):
    """
    Calculates A*i*conj(A)

    Args:
        A: Quaternion
        i: Middle vector to multiple i, j or k

    Returns:
        AiconjA: A*i*conjA

    """
    a = A[0]
    ax = A[1]
    ay = A[2]
    az = A[3]
    AicA0 = 0
    AicA1 = 0
    AicA2 = 0
    AicA3 = 0
    if v == "i":
        AicA0 = 0
        AicA1 = a**2 + ax**2 - ay**2 - az**2
        AicA2 = 2 * a * az + 2 * ax * ay
        AicA3 = 2 * ax * az - 2 * a * ay
    elif v == "j":
        AicA0 = 0
        AicA1 = -2 * a * az + 2 * ax * ay
        AicA2 = a**2 - ax**2 + ay**2 - az**2
        AicA3 = 2 * a * ax + 2 * ay * az
    elif v == "k":
        AicA0 = 0
        AicA1 = 2 * a * ay + 2 * ax * az
        AicA2 = -2 * a * ax + 2 * ay * az
        AicA3 = a**2 - ax**2 - ay**2 + az**2

    AicA = [AicA0, AicA1, AicA2, AicA3]
    if isinstance(A, np.ndarray):
        return np.array(AicA)
    elif isinstance(A, (cs.SX, cs.MX)):
        return cs.vertcat(AicA0, AicA1, AicA2, AicA3)


def euler_to_quaternion(yaw, pitch, roll):
    """
    Converts euler angles to quaternion

    Args:
        yaw: [rad]
        pitch: [rad]
        roll: [rad]

    Returns:
        q: [qw,qx,qy,qz]

    """
    rot = Rotation.from_euler("zyx", [yaw, pitch, roll])
    q = rot.as_quat()
    q = np.concatenate([[q[-1]], q[:-1]])  # [x,y,z,w] --> [w,x,y,z]
    return q


def euler_to_rotation(yaw, pitch, roll):
    """
    Converts euler angles to rotation matrix

    Args:
        yaw: [rad]
        pitch: [rad]
        roll: [rad]

    Returns:
        R: [e1,e2,e3]

    """
    rot = Rotation.from_euler("zyx", [yaw, pitch, roll])
    R = rot.as_matrix()
    return R


def quaternion_to_rotation(q):
    """
    Converts quaternion to rotation matrix

    Args:
        q: quaternion [qw, qx, qy, qz]

    Returns:
        R: [e1,e2,e3]


    """
    q = np.concatenate([q[1:], [q[0]]])  # [w,x,y,z] --> [x,y,z,w]
    rot = Rotation.from_quat(q)
    R = rot.as_matrix()
    return R


def rotation_to_quaternion(R):
    """
    Converts rotation matrix to quaterion

    Args:
        R: [e1,e2,e3]

    Returns:
        q: quaternion [qw, qx, qy, qz]


    """
    q = Rotation.from_matrix(R).as_quat()
    q = np.concatenate([[q[-1]], q[:-1]])  # [x,y,z,w] --> [w,x,y,z]
    return q


def rotation_to_euler(R):
    """
    Converts rotation matrix to euler angles

    Args:
        R: [e1,e2,e3]

    Returns:
        euler: [yaw,pitch,roll] in rad

    """
    euler = Rotation.from_matrix(R).as_euler("zyx")
    return euler


def tangent_to_rotation(e1, e3_des=np.array([0, 0, 1])):
    """
    Converts tangent vector to a possible rotation matrix. Given that infinite
    rotations are eligible, we choose the one whose e3 is closest to e3_des
    Args:
        e1: tangent vector [e1x,e1y,e1z]
        e3_des: desired e3

    Returns:
        R: Rotation matrix [e1,e2,e3], whose first component is e1 and third
           component is closest to e3_des


    """
    e1 = e1 / np.linalg.norm(e1)
    if (e1 == e3_des).all():
        e3 = np.array([1, 0, 0])
    else:
        e3 = closest_to_A_perpendicular_to_B(e3_des, e1)
    e3 = e3 / np.linalg.norm(e3)
    e2 = np.cross(e3, e1)
    R = np.vstack([e1, e2, e3]).T
    return R


def angle_between(v1, v2):
    """Returns the angle in radians between vectors 'v1' and 'v2'::"""
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def closest_to_A_perpendicular_to_B(A, B):
    """Returns closest vector to A which is perpendicular to B"""
    return A - B * np.dot(B, A)


def rotate_quaternion(q1, rotation):
    """Rotates a quaternion by a rotation matrix"""

    def q_inv(q):
        return np.array([q[0], -q[1], -q[2], -q[3]]) / np.linalg.norm(q) ** 2

    q2 = rotation_to_quaternion(rotation)
    q2_inv = q_inv(q2)

    a = (q1 - np.array([q1[0], -q1[1], -q1[2], -q1[3]])) / 2
    q1_rot = quat_mult(quat_mult(q2, a), q2_inv)

    return q1_rot


def angle_between(v1, v2):
    """Returns the angle in radians between vectors 'v1' and 'v2'::

    >>> angle_between((1, 0, 0), (0, 1, 0))
    1.5707963267948966
    >>> angle_between((1, 0, 0), (1, 0, 0))
    0.0
    >>> angle_between((1, 0, 0), (-1, 0, 0))
    3.141592653589793
    """

    def unit_vector(vector):
        """Returns the unit vector of the vector."""
        return vector / np.linalg.norm(vector)

    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
