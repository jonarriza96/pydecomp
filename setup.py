from setuptools import setup, find_packages, Extension
from glob import glob

ext_modules = [
    Extension(
        "_pydecomp",
        sources=["src/pydecomp/pydecomp.cpp"],
        include_dirs=[
            "src/external/pybind11/include/",
            "src/external/DecompUtil/include/",
            "/usr/include/eigen3/",
        ],
        language="c++",
        extra_compile_args=["-std=c++17"],
    )
]

setup(
    name="pydecomp",
    version="0.0.1",
    description="A Python package for decomposing obstacle free spaces into convex polygons",
    long_description="",
    url="https://gitlab.lrz.de/mophler/pydecomp/-/tree/main",
    author="Jon Arrizabalaga",
    author_email="arrijon96@gmail.com",
    license="MIT",
    install_requires=[
        "numpy==1.23.0",
        "matplotlib==3.5.2",
        "scipy==1.8.1",
        "pycddlib==2.1.6",
        "pyny3d==0.1.1",
        "casadi==3.5.5",
    ],
    ext_modules=ext_modules,
    packages=find_packages(where="src"),
    package_dir={"": "src"},
)
