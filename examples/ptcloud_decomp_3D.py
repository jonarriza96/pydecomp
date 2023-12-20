import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

import pydecomp as pdc


# ---------------------------- Obstacles and path ---------------------------- #
# obstacles
obstacles = np.load(str(Path(__file__).parent.absolute()) + "/data/office/ptcloud.npy")
obstacles = np.squeeze(
    obstacles[np.argwhere((obstacles[:, 2] < 3) & (np.abs(obstacles[:, 0]) < 15)), :]
)
obstacles = obstacles[:-1:10, :]

# path
path = np.array([[8, -3, 2], [11.5, -5, 2], [11, 11, 2]])

# bounding box
box = np.array([[1, 2, 1]])

# ------------------------------- Decomposition ------------------------------ #
# convex decomposition
A, b = pdc.convex_decomposition_3D(obstacles, path, box)

# ------------------------------- Visualization ------------------------------ #
ax = pdc.visualize_environment(Al=A, bl=b, p=path, planar=False)
ax.scatter(
    obstacles[:, 0],
    obstacles[:, 1],
    obstacles[:, 2],
    c=obstacles[:, 2],
    cmap="turbo",
)
ax.plot(path[:, 0], path[:, 1], path[:, 2], "k-o")
plt.show()
