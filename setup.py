import os

from setuptools import setup, Extension


def _system_include_dirs():
    candidates = [
        os.environ.get("EIGEN_INCLUDE_DIR"),
        "/usr/include/eigen3",
        "/usr/local/include/eigen3",
        "/opt/homebrew/include/eigen3",
        "/opt/homebrew/opt/eigen/include/eigen3",
        "/usr/local/include",
        "/opt/homebrew/include",
    ]
    return [c for c in candidates if c and os.path.isdir(c)]


def _pybind11_include():
    try:
        import pybind11
    except ImportError:
        return "src/external/pybind11/include"
    return pybind11.get_include()


ext_modules = [
    Extension(
        "_pydecomp",
        sources=["src/pydecomp/pydecomp.cpp"],
        include_dirs=[
            _pybind11_include(),
            "src/external/DecompUtil/include/",
            *_system_include_dirs(),
        ],
        language="c++",
        extra_compile_args=["-std=c++17"],
    )
]

setup(ext_modules=ext_modules)
