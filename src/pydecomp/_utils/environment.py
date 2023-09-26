import numpy as np
import casadi as cs

import cdd
import time

import numpy as np
import matplotlib.pyplot as plt


from .visualize import visualize_environment
from .quaternion import *


def generate_random_path(
    n_points, r_min, r_max, elev_min, elev_max, az_min, az_max, planar=False
):
    """Generates a random path according to bounds in spherical coordinates
    Args:
        n_points: Number of waypoints in the path
        r_min: Minimum radius in m
        r_max: Maximum radius in m
        elev_min: Minimum elevation in rad(inclination)
        elev_max: Maximum elevation in rad (inclination)
        az_min: Minimum azimuth in rad
        az_max: Maximum azimuth in rad

    Returns:
        p: Matrix [n_points x 3] with waypoints

    """
    x = 0
    y = 0
    z = 0
    p = [[x, y, z]]
    elev = 0
    az = 0
    for _ in range(n_points):
        r = np.random.uniform(r_min, r_max)
        elev += np.random.uniform(elev_min, elev_max)
        az += np.random.uniform(az_min, az_max)

        delta_x = r * np.cos(az) * np.sin(elev)
        delta_y = r * np.sin(az) * np.sin(elev)
        delta_z = r * np.cos(elev)

        x += delta_x
        y += delta_y
        z += delta_z
        p += [[x, y, z]]

    p = np.squeeze(p)

    if planar:
        p = p[:, :2]

    return p


def get_halfspace_from_path(p, max_vertices=4, l_min=1, l_max=2, planar=False):
    """Computes the halfpsace representation (Ax - b < 0) for a path described
    by waypoints.

    Args:
        p: Matrix [n_points x 3~2] with waypoints
        max_vertices: Maximum number of vertices in every waypoint. If planar, they are set to 2
        l_min: Minimum distance from center-line to vertices in [m]
        l_max: Maximum distance from center-line to vertices in [m]
        planar: Boolean for planar case

    Returns:
        A: List of A [n_poly] matrixes [n_sides x 3~2]
        b: List of b [n_poly] matrixes [n_sides x 1]

    """

    n_poly = p.shape[0] - 1
    t = np.diff(p, axis=0)

    vertices = []
    A = []
    b = []
    if planar:
        p = np.hstack([p, np.zeros((p.shape[0], 1))])

    for m in range(n_poly):
        e1 = t[m] / np.linalg.norm(t[m])
        if planar:
            e1 = np.concatenate([e1, np.zeros(1)])

        sm = 0.2 * np.linalg.norm(p[m + 1] - p[m])
        min_ind = np.argmin(
            [
                angle_between((1, 0, 0), tuple(e1)),
                angle_between((0, 1, 0), tuple(e1)),
                angle_between((0, 0, 1), tuple(e1)),
            ]
        )
        if min_ind == 0:
            e1_2d = np.array([1, 0, 0])
        elif min_ind == 1:
            e1_2d = np.array([0, 1, 0])
        elif min_ind == 2:
            e1_2d = np.array([0, 0, 1])
        R = tangent_to_rotation(e1=e1)  # ,e3_des=e1_2d)

        vert_poly = []
        # wp1
        if max_vertices < 0:
            n_vertices1 = -max_vertices
        else:
            n_vertices1 = np.random.randint(3, max_vertices)
        l1 = np.random.uniform(l_min, l_max)
        theta_end = 2 * np.pi  # np.random.uniform(0,2*np.pi)
        for n in range(n_vertices1):
            theta = theta_end * n / n_vertices1
            Ryaw = np.array(
                [
                    [0, np.cos(theta), -np.sin(theta)],
                    [0, np.sin(theta), np.cos(theta)],
                    [1, 0, 0],
                ]
            )
            Rrot = np.dot(R, Ryaw.T)
            vert_poly += [p[m] + l1 * Rrot[:, 1] - sm * e1]

        # wp2
        if max_vertices < 0:
            n_vertices2 = -max_vertices
        else:
            n_vertices2 = np.random.randint(3, max_vertices)
        l2 = np.random.uniform(l_min, l_max)
        for n in range(n_vertices2):
            theta = 2 * np.pi * n / n_vertices2
            Ryaw = np.array(
                [
                    [0, np.cos(theta), -np.sin(theta)],
                    [0, np.sin(theta), np.cos(theta)],
                    [1, 0, 0],
                ]
            )
            Rrot = np.dot(R, Ryaw.T)
            vert_poly += [p[m + 1] + l2 * Rrot[:, 1] + sm * e1]

        vertices += [np.squeeze(vert_poly)]

        n_vertices = n_vertices1 + n_vertices2

        if planar:
            v = np.hstack([np.ones((n_vertices, 1)), vertices[-1][:, :2]])
        else:
            v = np.hstack([np.ones((n_vertices, 1)), vertices[-1]])

        mat = cdd.Matrix(v)
        mat.rep_type = cdd.RepType.GENERATOR
        poly = cdd.Polyhedron(mat)
        gen = np.array(poly.get_inequalities())

        b += [gen[:, 0]]  # [np.ones(len(gen[:,0]))]
        A += [-gen[:, 1:]]  # /gen[:,0][:,np.newaxis]]

        # for vv in vertices:
        #     plt.plot(vv[:, 0], vv[:, 1], ".")
        # plt.plot(p[:, 0], p[:, 1], "-o")
        # plt.axis("equal")

    return A, b


def generate_random_orientation(
    e1,
    yaw_min=-np.pi / 4,
    yaw_max=np.pi / 4,
    pitch_min=-np.pi / 4,
    pitch_max=np.pi / 4,
    roll_min=-np.pi,
    roll_max=np.pi,
    planar=False,
):
    """
    Given a tangent vector e1, it defines a random pose within the euler angle
    bounds

    Args:
        e1: List of tangent vectors
        yaw_min: min yaw [rad]
        yaw_max: max yaw [rad]
        pitch_min: min pitch [rad]
        pitch_max: max pitch [rad]
        roll_min: min roll [rad]
        roll_max: max roll [rad]

    Returns:
        q: Matrix [n_list x 4] of quaternions [qw, qx, qy, qz]

    """

    if planar:
        e1 = np.hstack([e1, np.zeros((e1.shape[0], 1))])
        roll_min = 0
        roll_max = 0
        pitch_min = 0
        pitch_max = 0

    q = []
    for ee1 in e1:
        R = tangent_to_rotation(ee1)
        euler = rotation_to_euler(R)

        delta_yaw = np.random.uniform(yaw_min, yaw_max)
        delta_pitch = np.random.uniform(pitch_min, pitch_max)
        delta_roll = np.random.uniform(roll_min, roll_max)
        yaw = euler[0] + delta_yaw
        pitch = euler[1] + delta_pitch
        roll = euler[2] + delta_roll

        q += [euler_to_quaternion(yaw, pitch, roll)]

    q = np.squeeze(q)
    return q


def generate_random_waypoints(p, A, b, l_max, planar=False):
    """
    Given a path, the halfspace representation of the free-space  and the limits
    of the location for the vertices, finds initial and final waypoints randomly
    located within the polyhedrons

    Args:
        p: Matrix [n_points x 3~2] with waypoints
        A: List of A [n_poly] matrixes [n_sides x 3~2]
        b: List of b [n_poly] matrixes [n_sides x 1]
        l_max: Maximum distance from center-line to vertices in [m]

    Returns:
        waypoints: Matrix [2 x 3] with initial and final waypoints


    """

    if planar:
        p = np.hstack([p, np.zeros((p.shape[0], 1))])

    e1s = [p[1] - p[0], p[-1] - p[-2]]

    l_margin = 0.1
    pp = [p[0] + e1s[0] * l_margin, p[-1] - e1s[1] * l_margin]
    A = [A[0], A[-1]]
    b = [b[0], b[-1]]

    wp = []
    for i, (pt, ee1, Aa, bb) in enumerate(zip(pp, e1s, A, b)):
        R = tangent_to_rotation(ee1)

        is_inside = False
        t_start = time.time()
        while not is_inside:
            l = np.random.uniform(0, l_max)
            theta = np.random.uniform(0, 2 * np.pi)
            Ryaw = np.array(
                [
                    [0, np.cos(theta), -np.sin(theta)],
                    [0, np.sin(theta), np.cos(theta)],
                    [1, 0, 0],
                ]
            )
            Rrot = np.dot(R, Ryaw.T)
            wpt = pt + l * Rrot[:, 1]
            if planar:
                wpt = wpt[:2]
            if (np.dot(Aa, wpt) - bb < 0).all():
                is_inside = True

            if time.time() - t_start > 5:
                print(f"\tWaypoint generation in {i} time-out")
                wpt = pt
                is_inside = True

        wp += [wpt]

    wp = np.squeeze(wp)
    return wp


def generate_environment(
    n_poly=4,
    max_vertices=4,
    l_min=1,
    l_max=2,
    r_min=2,
    r_max=5,
    elev_min=-np.pi / 2,
    elev_max=np.pi / 2,
    az_min=-np.pi / 2,
    az_max=np.pi / 2,
    visualize=False,
    verbose=True,
    planar=False,
):
    """Generates a random environment by
        (1) constructing a random path (according to bounds in spherical coordinates)
        (2) finding vertices at every waypoint of the path
        (3) converting the vertices to the halfspace representation (Ax - b < 0)

        The randomization variables are added as inputs to the function.

        NOTE: The randomization of the initial and final orientations are not
        included as inputs.

    Args:
        n_poly: Number of polyhedrons
        max_vertices: Maximum number of vertices in every waypoint of the path
        l_min: Minimum distance from center-line to vertices in [m]
        l_max: Maximum distance from center-line to vertices in [m]
        r_min: Minimum radius in [m]
        r_max: Maximum radius in [m]
        elev_min: Minimum elevation in [rad] (inclination)
        elev_max: Maximum elevation in [rad] (inclination)
        az_min: Minimum azimuth in [rad]
        az_max: Maximum azimuth in [rad]
        visualize: Boolean to plot generated environment
        verbose: Boolean to print start and end of generation

    Returns:
        A: Matrix [n_planes x 3] with hyperplanes
        b: Matrix [n_planes x 1] with hyperplanes
        p: Matrix [3 x n_points] with waypoints
        wp: Matrix [2 x 3] with initial and final waypoints
        q: Matrix [2 x 4] initial and final orientation


    """

    if verbose:
        print("Generating environment ...")

    # generate random path
    p = generate_random_path(
        n_points=n_poly,
        r_min=r_min,
        r_max=r_max,
        elev_min=elev_min,
        elev_max=elev_max,
        az_min=az_min,
        az_max=az_max,
        planar=planar,
    )

    # get halfpace and vertices around the path
    A, b = get_halfspace_from_path(
        p=p, max_vertices=max_vertices, l_min=l_min, l_max=l_max, planar=planar
    )

    # generate random waypoints
    wp = generate_random_waypoints(p=p, A=A, b=b, l_max=l_max, planar=planar)

    # generate random orientations
    q = generate_random_orientation(
        e1=np.array([p[1] - p[0], p[-1] - p[-2]]),
        yaw_min=-np.pi / 4,
        yaw_max=np.pi / 4,
        pitch_min=-np.pi / 4,
        pitch_max=np.pi / 4,
        roll_min=-np.pi / 4,
        roll_max=np.pi / 4,
        planar=planar,
    )

    if verbose:
        print("Done")

    # visualize
    if visualize:
        _ = visualize_environment(A, b, p, wp, q, planar=planar)
        plt.show()
    return A, b, p, wp, q


def transform_to_initial_pose(A, b, wp, q, planar=False):
    if planar:
        wp = np.hstack([wp, np.zeros((wp.shape[0], 1))])
    R_i = quaternion_to_rotation(q[0])
    wp_i = np.vstack([np.zeros(3), np.dot(R_i.T, wp[1] - wp[0])])
    q_i = np.vstack(
        [
            rotation_to_quaternion(np.eye(3)),
            rotation_to_quaternion(np.dot(R_i.T, quaternion_to_rotation(q[1]))),
        ]
    )

    A_i = []
    b_i = []
    for A_w, b_w in zip(A, b):
        if planar:
            A_w = np.hstack([A_w, np.zeros((A_w.shape[0], 1))])
        A_i += [np.dot(A_w, R_i)]
        b_i += [-np.dot(A_w, wp[0]) + b_w]

    return A_i, b_i, wp_i, q_i


def normalize_halfspace(a, b):
    """
    Normalizes a halfspace A*x - b < 0 for ONE sample. The pseudo-code is:

            if b != 0:                                   # CASE 1
                C = sign(b)*(A/b)/norm(A/b)
                d = sign(b)*1/norm(A/b)
            else:
                if norm(A) > 0:                         # CASE 2
                    C = A/norm(A); D = 0
                else:                                   # CASE 3
                    C = 0; D = 0

    Notice that the norms are applied over the x.y,z columns

    Args:
        a: [n_sides_total x 3] or a list [n_poly][n_sides_per_poly x 3]
        b: [n_sides_total] or a list [n_poly][n_sides_per_poly]

    Returns:
        c: [n_sides_total x 3] or a list [n_poly][n_sides_per_poly x 3]
        d: [n_sides_total] or a list [n_poly][n_sides_per_poly]

    """

    # concatenate into a matrix if input is a list
    if isinstance(a, list):
        C = []
        D = []
        for aa, bb in zip(a, b):
            # declare matrixes and identify case indexes
            n_sides = aa.shape[0]
            c = np.zeros((n_sides, 3))
            d = np.zeros(n_sides)

            ind_case1 = np.squeeze(np.argwhere((bb > 1e-6) | (bb < -1e-6)))
            ind_case2 = np.squeeze(
                np.stack([np.linalg.norm(aa, axis=1) > 0, bb <= 0]).all(axis=0)
            )

            # case 1
            c[ind_case1, :] = aa[ind_case1, :] / bb[ind_case1][:, np.newaxis]
            c_norm = np.linalg.norm(c[ind_case1, :], axis=1)
            c[ind_case1, :] = (
                np.sign(bb[ind_case1])[:, np.newaxis]
                * c[ind_case1, :]
                / c_norm[:, np.newaxis]
            )
            d[ind_case1] = np.sign(bb[ind_case1]) * 1 / c_norm

            # case 2
            c[ind_case2, :] = (
                aa[ind_case2, :]
                / np.linalg.norm(aa[ind_case2, :], axis=1)[:, np.newaxis]
            )

            C += [c]
            D += [d]

    else:
        # declare matrixes and identify case indexes
        n_sides = a.shape[0]
        c = np.zeros((n_sides, 3))
        d = np.zeros(n_sides)

        ind_case1 = np.squeeze(np.argwhere((b > 1e-6) | (b < -1e-6)))
        ind_case2 = np.squeeze(
            np.stack([np.linalg.norm(a, axis=1) > 0, b <= 0]).all(axis=0)
        )

        # case 1
        c[ind_case1, :] = a[ind_case1, :] / b[ind_case1][:, np.newaxis]
        c_norm = np.linalg.norm(c[ind_case1, :], axis=1)
        c[ind_case1, :] = (
            np.sign(b[ind_case1])[:, np.newaxis]
            * c[ind_case1, :]
            / c_norm[:, np.newaxis]
        )
        d[ind_case1] = np.sign(b[ind_case1]) * 1 / c_norm

        # case 2
        c[ind_case2, :] = (
            a[ind_case2, :] / np.linalg.norm(a[ind_case2, :], axis=1)[:, np.newaxis]
        )

        C = c
        D = d

    return C, D
