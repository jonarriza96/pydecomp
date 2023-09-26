import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

import pydecomp as pdc


world = "office"  # "office" or "forest"

# ---------------------------- Obstacles and path ---------------------------- #

# obstacles
obstacles = np.load(
    str(Path(__file__).parent.absolute()) + "/data/" + world + "/ptcloud.npy"
)[:, :2]

# path
if world == "office":
    path = np.array([[8, 1], [11, 3], [11, 12], [8.16, 15.45], [8.8, 16.96], [8.8, 20]])
elif world == "forest":
    path = np.array(
        [[-8, 10], [-6, 2], [-1, 1.5], [-1, -6], [2.52, -5], [4.25, -7.5], [8, -8]]
    )

# --------------------------- Convex decomposition --------------------------- #
A, b = pdc.convex_decomposition_2D(obstacles, path)


# ------------------------------- Visualization ------------------------------ #
ax = pdc.visualize_environment(Al=A, bl=b, p=path, planar=True)
ax.plot(path[:, 0], path[:, 1], "k-o")
ax.plot(obstacles[:, 0], obstacles[:, 1], ".", color="red")
ax.set_xlim([np.min(obstacles[:, 0]), np.max(obstacles[:, 0])])
ax.set_ylim([np.min(obstacles[:, 1]), np.max(obstacles[:, 1])])
plt.show()
