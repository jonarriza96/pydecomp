import numpy as np
import matplotlib.pyplot as plt

import pydecomp as pdc

# -------------------------------- User inputs ------------------------------- #
planar = False  # planar environment
n_poly = 4  # number of polygons
max_vertices = 5  # maximum number of vertices per polygon
visualize = True  # visualize environment


# -------------------------- Environment generation -------------------------- #
A, b, p, wp, q = pdc.generate_environment(
    n_poly=n_poly,
    max_vertices=max_vertices,
    visualize=visualize,
    planar=planar,
)

# --------------------- Transformation and normalization --------------------- #
# transform to initial pose and scale (good for generating "learning" datasets)
A, b, wp, q = pdc.transform_to_initial_pose(A, b, wp, q, planar=planar)
A, b = pdc.normalize_halfspace(A, b)
if planar:
    A = [AA[:, :2] for AA in A]
    wp = wp[:, :2]
