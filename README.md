![Logo](https://github.com/jonarriza96/pydecomp/raw/main/docs/logo/logo.png)
**pydecomp — A Python implementation of <a href="https://github.com/sikang/DecompUtil/tree/master">DecompUtil<sup></sup></a> for fast convex decomposition of obstacle-free spaces.**

[![PyPI version](https://badge.fury.io/py/pydecomp.svg)](https://badge.fury.io/py/pydecomp)
![PyPI - Python Version](https://img.shields.io/pypi/pyversions/pydecomp)
![PyPI - License](https://img.shields.io/pypi/l/pydecomp)
![PyPI - Downloads](https://img.shields.io/pypi/dm/pydecomp)

## Quickstart

Install dependencies with

```
    sudo apt-get install libcdd-dev libblas3 libblas-dev liblapack3 liblapack-dev gfortran
```

and (in a virtual environment) install from PyPI with

```
    pip install pydecomp
```

<!-- To install from source, see [here](#installing-from-source). -->

## Examples

Forest - 2D | Office - 2D | Office - 3D
:-------------------------:|:-------------------------:|:-------------------------:
![](https://github.com/jonarriza96/pydecomp/raw/main/docs/forest.png) | ![](https://github.com/jonarriza96/pydecomp/raw/main/docs/office.png) | ![](https://github.com/jonarriza96/pydecomp/raw/main/docs/office_3d.png)


Given an occupancy grid map and a pieciwise linear path, the package returns a convex decomposition of the free space. The free space is represented by a collection of convex sets, whose halspace representation fulfills ` Ax-b<0`. The package returns the matrixes `A` and `b` for each convex set. The matrix `A` is a `n x 3` matrix and `b` is a `n x 1` vector, where `n` is the number of planes in the halfspace.

To check a script to perform a convex decomposition out of a given occupancy grid map, see [this file](examples/ptcloud_decomp_2D.py) for a planar (2D) case or [this file](examples/ptcloud_decomp_3D.py) for a spatial (3D) case. We provide two exemplary maps (forest and office), which you can select by modifying [this line](examples/ptcloud_decomp_2D.py#L8).

For the most minimal example see [this file](examples/ellipsoid_decomp_2D.py), which replicates a [test case](https://github.com/sikang/DecompUtil/blob/master/test/test_ellipsoid_decomp.cpp) in the original <a href="https://github.com/sikang/DecompUtil/tree/master">DecompUtil<sup></sup></a> repository.

https://github.com/jonarriza96/pydecomp/blob/ff4fc8247d79c0f969437192d49f7540f9a92058/examples/ellipsoid_decomp_2D.py#L5-L20

## Installing from source

After installing the dependencies given above, initialize git submodules with

```
    git submodule init
    git submodule update
```

Install the package with

```
    pip install .
```

## Citing

If you use this framework please cite our paper:

```
@inproceedings{arrizabalaga2023sctomp,
  title={SCTOMP: Spatially Constrained Time-Optimal Motion Planning},
  author={Arrizabalaga, Jon and Ryll, Markus},
  booktitle={2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4827--4834},
  year={2023},
  organization={IEEE}
}
```
