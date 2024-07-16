import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

import pydecomp as pdc

# obstacles, path and bounding box
obstacles = np.loadtxt(
    str(Path(__file__).parent.absolute()) + "/data/decomputil/obstacles.txt"
)
path = np.array([[1, 1], [0, 0], [-1, 1]])
box = np.array([[2, 2]])

# convex decomposition
A, b = pdc.convex_decomposition_2D(obstacles, path, box)

ax = pdc.visualize_environment(Al=A, bl=b, p=path, planar=True)
ax.plot(obstacles[:, 0], obstacles[:, 1], "o", color="red")
plt.show()
