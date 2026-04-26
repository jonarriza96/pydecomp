import re

import pydecomp


def test_version_attr():
    assert hasattr(pydecomp, "__version__")
    assert re.fullmatch(r"\d+\.\d+\.\d+(?:[.\-+].+)?", pydecomp.__version__)


def test_public_api_exposed():
    for name in (
        "convex_decomposition_2D",
        "visualize_environment",
        "generate_environment",
        "transform_to_initial_pose",
        "normalize_halfspace",
    ):
        assert hasattr(pydecomp, name), f"missing: {name}"
