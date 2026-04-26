import numpy as np
import pytest

import pydecomp as pdc


@pytest.fixture
def ellipsoid_2d_inputs():
    obstacles = np.array(
        [
            [-0.2, 1.5],
            [0.0, 1.5],
            [0.0, 1.0],
            [0.5, 1.0],
            [0.5, 0.0],
            [-0.2, -0.5],
            [-0.5, -0.5],
            [-0.8, 0.5],
            [-1.0, 1.5],
            [-1.5, 1.5],
        ]
    )
    path = np.array([[1.0, 1.0], [0.0, 0.0], [-1.0, 1.0]])
    box = np.array([[2.0, 2.0]])
    return obstacles, path, box


def test_convex_decomposition_2d_shapes(ellipsoid_2d_inputs):
    obstacles, path, box = ellipsoid_2d_inputs
    A, b = pdc.convex_decomposition_2D(obstacles, path, box)

    n_segments = path.shape[0] - 1
    assert isinstance(A, list) and isinstance(b, list)
    assert len(A) == n_segments
    assert len(b) == n_segments

    for Ai, bi in zip(A, b):
        Ai = np.asarray(Ai)
        bi = np.asarray(bi)
        assert Ai.ndim == 2
        assert Ai.shape[1] == 2
        assert bi.shape[0] == Ai.shape[0]


def test_visualize_runs_without_error(ellipsoid_2d_inputs):
    """Regression test: pycddlib 3.x removed `cdd.Matrix(arr)` etc.,
    which silently broke visualize_environment. CI's import-only smoke
    test missed it; this test exercises the cdd code path."""
    obstacles, path, box = ellipsoid_2d_inputs
    A, b = pdc.convex_decomposition_2D(obstacles, path, box)

    ax = pdc.visualize_environment(Al=A, bl=b, p=path, planar=True)
    assert ax is not None
