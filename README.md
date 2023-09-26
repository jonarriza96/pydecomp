# pydecomp

![Logo](https://github.com/jonarriza96/pydecomp/raw/main/docs/logo/logo.png)
A Python implementation of <a href="https://github.com/sikang/DecompUtil/raw/master">DecompUtil<sup></sup></a> for fast convex decomposition of obstacle-free spaces.

## Quickstart

Install dependencies with

```
    sudo apt-get install libcdd-dev libblas3 libblas-dev liblapack3 liblapack-dev gfortran
```

and (in a virtual environment) install the package with

```
    pip install -i https://test.pypi.org/pypi/ --extra-index-url https://pypi.org/simple pydecomp==0.0.4
```

<!-- To install from source, see [here](#installing-from-source). -->

## Usage

### Convex decomposition of free space from occupancy grid map

Given an occupancy grid map and a pieciwise linear path, the package returns a convex decomposition of the free space. The free space is represented by a collection of convex sets, whose halspace representation fulfills ` Ax-b<0`. The package returns the matrixes `A` and `b` for each convex set. The matrix `A` is a `n x 3` matrix and `b` is a `n x 1` vector, where `n` is the number of planes in the halfspace.

To check a script to perform a convex decomposition out of a given occupancy grid map, see [this file](examples/ptcloud_decomp_2D.py) for a planar (2D) case or [this file](examples/ptcloud_decomp_3D.py) for a spatial (3D) case. We provide two exemplary maps (forest and office), which you can select by modifying [this line](examples/ptcloud_decomp_2D.py#L8).
Forest - 2D | Office - 2D | Office - 3D
:-------------------------:|:-------------------------:|:-------------------------:
![](https://github.com/jonarriza96/pydecomp/raw/main/docs/forest.png) | ![](https://github.com/jonarriza96/pydecomp/raw/main/docs/office.png) | ![](https://github.com/jonarriza96/pydecomp/raw/main/docs/office_3d.png)

For the most minimal example see [this file](examples/ellipsoid_decomp_2D.py), which replicates a [test case](https://github.com/sikang/DecompUtil/blob/master/test/test_ellipsoid_decomp.cpp) in the original repository.

### Environment generation

Run [this file](examples/random_corridor_generator.py) to see how to generate random corridors. When doing so, feel free to change the randomization parameters. These are given in [this function definition](pydecomp/utils/environment.py#L300).

**Note**: Currently the randomization algorithm works better for 3D (set planar=False in [here](examples/random_corridor_generator.py#L7)). For 2D, the randomization algorithm is not very good and, in some cases, the resultant corridor has a weird shape. This should be fixed in the future.

See here two examples of corridors (planar and spatial) generated with the randomization algorithm:
Planar | Spatial
:-------------------------:|:-------------------------:
![](https://github.com/jonarriza96/pydecomp/raw/main/docs/2d.png) | ![](https://github.com/jonarriza96/pydecomp/raw/main/docs/3d.png)

The blue polyhedrons are the convex decomposition of the obstacle-free space. The inner black line depicts the underlying piecewise linear path used to construct the polyhedrons. The green and red dots, alongside their frames, refer to the starting and ending pose of the agent.

### Transformation and normalization of the polyhedrons

When using the environment generator for generating a dataset of corridors, it might be beneficial to represent all of them in a normalized and standard representation. Towards this aim, we provide the feature to normalize and translate the polyhedrons, so that the matrixes for the halfspaces are given by values between 0 and 1 and the origin coincides with the starting pose of the agent (green dot and frame). See [these functions](examples/random_corridor_generator.py#L26-31) for more details.

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
