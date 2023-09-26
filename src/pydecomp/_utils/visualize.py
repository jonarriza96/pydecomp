import numpy as np
import casadi as cs
import cdd

import pyny3d.geoms as pyny

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from .quaternion import quaternion_to_rotation


def axis_equal(X, Y, Z, ax=None):
    """
    Sets axis bounds to "equal" according to the limits of X,Y,Z.
    If axes are not given, it generates and labels a 3D figure.

    Args:
        X: Vector of points in coord. x
        Y: Vector of points in coord. y
        Z: Vector of points in coord. z
        ax: Axes to be modified

    Returns:
        ax: Axes with "equal" aspect


    """
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")

    max_range = (
        np.array([X.max() - X.min(), Y.max() - Y.min(), Z.max() - Z.min()]).max() / 2.0
    )
    mid_x = (X.max() + X.min()) * 0.5
    mid_y = (Y.max() + Y.min()) * 0.5
    mid_z = (Z.max() + Z.min()) * 0.5
    ax.set_xlim(mid_x - 1.2 * max_range, mid_x + 1.2 * max_range)
    ax.set_ylim(mid_y - 1.2 * max_range, mid_y + 1.2 * max_range)
    ax.set_zlim(mid_z - 1.2 * max_range, mid_z + 1.2 * max_range)

    return ax


def plot_frames(
    r, e1, e2, e3, interval=0.9, scale=1.0, ax=None, ax_equal=True, planar=False
):
    """
    Plots the moving frame [e1,e2,e3] of the curve r. The amount of frames to
    be plotted can be controlled with "interval".

    Args:
        r: Vector of 3d points (x,y,z) of curve
        e1: Vector of first component of frame
        e2: Vector of second component of frame
        e3: Vector of third component of frame
        interval: Percentage of frames to be plotted, i.e, 1 plots a frame in
                  every point of r, while 0 does not plot any.
        scale: Float to size components of frame
        ax: Axis where plot will be modified

    Returns:
        ax: Modified plot
    """
    # scale = 0.1
    nn = r.shape[0]
    tend = r + e1 * scale
    nend = r + e2 * scale
    bend = r + e3 * scale

    if ax is None:
        ax = plt.figure().add_subplot(111, projection="3d")

    if interval == 1:
        rng = range(nn)
    else:
        rng = range(0, nn, int(nn * (1 - interval)) if nn > 1 else 1)

    if planar:
        for i in rng:  # if nn >1 else 1):
            ax.plot([r[i, 0], tend[i, 0]], [r[i, 1], tend[i, 1]], "r")
            ax.plot([r[i, 0], nend[i, 0]], [r[i, 1], nend[i, 1]], "g")

            # ax.plot([r[i, 0], tend[i, 0]], [r[i, 2], tend[i, 2]], "r")  # , linewidth=2)
            # ax.plot([r[i, 0], bend[i, 0]], [r[i, 2], bend[i, 2]], "g")  # , linewidth=2)
        ax.set_aspect("equal")

    else:
        if ax_equal:
            ax = axis_equal(r[:, 0], r[:, 1], r[:, 2], ax=ax)

        for i in rng:
            ax.plot(
                [r[i, 0], tend[i, 0]], [r[i, 1], tend[i, 1]], [r[i, 2], tend[i, 2]], "r"
            )
            ax.plot(
                [r[i, 0], nend[i, 0]], [r[i, 1], nend[i, 1]], [r[i, 2], nend[i, 2]], "g"
            )
            ax.plot(
                [r[i, 0], bend[i, 0]], [r[i, 1], bend[i, 1]], [r[i, 2], bend[i, 2]], "b"
            )

    return ax


def visualize_environment(
    Al, bl, p=None, p_interp=None, q=None, ax=None, planar=False, ax_view=True
):
    bl = [np.squeeze([b]) for b in bl]

    if ax is None:
        if not planar:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection="3d")

    vert = []
    polyhedrons = []
    if Al is not None:
        for A, b in zip(Al, bl):
            if not (A == 0).all():  # only draw if polyhedron is active
                # remove zeros (inactive sides)
                ind_zero = ~np.all(A == 0, axis=1)
                A = A[ind_zero]
                b = b[ind_zero]

                # convert to vertices and arrange by faces
                mat = cdd.Matrix(np.hstack([b[:, np.newaxis], -A]))
                mat.rep_type = cdd.RepType.INEQUALITY

                # visualize
                try:
                    poly = cdd.Polyhedron(mat)
                    gen = poly.get_generators()
                    vert += [np.array(list(gen))[:, 1:]]
                    intersections = [list(x) for x in poly.get_input_incidence()]
                    if planar:
                        polyhedrons += [pyny.Polygon(vert[-1])]
                    else:
                        polygon = []
                        for inter in intersections[:-1]:
                            if inter:
                                polygon += [pyny.Polygon(vert[-1][inter])]
                        polyhedrons += [pyny.Polyhedron(polygon)]
                except:
                    print("Error in plotting polyhedrons")

    if not planar:
        if len(vert) > 0:
            vert = np.concatenate(vert)
            ax = axis_equal(vert[:, 0], vert[:, 1], vert[:, 2], ax=ax)
        else:
            if Al is None and p_interp is not None:
                ax = axis_equal(p_interp[:, 0], p_interp[:, 1], p_interp[:, 2], ax=ax)
            elif p_interp is not None:
                ax = axis_equal(p_interp[:, 0], p_interp[:, 1], p_interp[:, 2])

    for plh in polyhedrons:
        if planar:
            ax = pyny.Surface(polyhedrons).plot2d(c_poly="b", alpha=0.5, ret=True)
            if p_interp is not None:
                ax.plot(
                    p_interp[0, 0], p_interp[0, 1], "o", color="lime", markersize=15
                )
                ax.plot(
                    p_interp[-1, 0], p_interp[-1, 1], "o", color="red", markersize=15
                )

            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_aspect("equal")
            ax.set_xticks([])
            ax.set_yticks([])

            if not ax_view:
                ax.get_xaxis().set_visible(False)
                ax.get_yaxis().set_visible(False)
                ax.spines["top"].set_visible(False)
                ax.spines["right"].set_visible(False)
                ax.spines["bottom"].set_visible(False)
                ax.spines["left"].set_visible(False)

        else:
            ax = plh.plot(color="#0000FF10", ax=ax, ret=True)

            if p_interp is not None:
                ax.plot(
                    p_interp[0, 0],
                    p_interp[0, 1],
                    p_interp[0, 2],
                    "o",
                    color="lime",
                    markersize=15,
                )
                ax.plot(
                    p_interp[-1, 0],
                    p_interp[-1, 1],
                    p_interp[-1, 2],
                    "o",
                    color="red",
                    markersize=15,
                )
            ax.set_zlabel("z")

        ax.set_xlabel("x")
        ax.set_ylabel("y")

    if q is not None:
        e1_i, e2_i, e3_i = quaternion_to_rotation(q[0, :]).T
        e1_f, e2_f, e3_f = quaternion_to_rotation(q[1, :]).T

        r = np.vstack([p_interp[0, :], p_interp[-1, :]])
        e1 = np.vstack([e1_i, e1_f])
        e2 = np.vstack([e2_i, e2_f])
        e3 = np.vstack([e3_i, e3_f])

        if planar:
            r = np.hstack([r, np.zeros((2, 1))])
        plot_frames(r, e1, e2, e3, interval=1, scale=1.1, ax=ax, planar=planar)

    if p is not None:
        if planar:
            ax.plot(p[:, 0], p[:, 1], "-o", alpha=0.5, color="k")
        else:
            ax.plot(p[:, 0], p[:, 1], p[:, 2], "-o", alpha=0.5, color="k")

    return ax
