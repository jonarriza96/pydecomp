#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <decomp_geometry/geometric_utils.h>
#include <decomp_util/ellipsoid_decomp.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <fstream>

namespace py = pybind11;

py::tuple convex_decomposition_2D(const py::array_t<double> obs_np,
                                  const py::array_t<double> path_np) {

  /* ------------------------------ Get obstacles -----------------------------
   */

  // Access the numpy matrix data
  auto obs_matrix = obs_np.unchecked<2>(); // 2D matrix assumption
  ssize_t obs_rows = obs_np.shape(0);

  // Access the elements of the matrix
  vec_Vec2f obs;
  Vecf<2> obs_pt;
  for (ssize_t i = 0; i < obs_rows; ++i) {
    obs_pt << obs_matrix(i, 0), obs_matrix(i, 1);
    obs.push_back(obs_pt);
  }

  /* --------------------------- Get path to dilate ---------------------------
   */

  // Access the numpy matrix data
  auto path_matrix = path_np.unchecked<2>(); // 2D matrix assumption
  ssize_t path_rows = path_np.shape(0);

  // Access the elements of the matrix
  vec_Vec2f path;
  Vecf<2> path_pt;
  for (ssize_t i = 0; i < path_rows; ++i) {
    path_pt << path_matrix(i, 0), path_matrix(i, 1);
    path.push_back(path_pt);
  }

  /* -------------------------- Convex decomposition --------------------------
   */
  EllipsoidDecomp2D decomp; //(origin, range);
  decomp.set_obs(obs);
  decomp.set_local_bbox(Vec2f(2, 2));
  decomp.dilate(path, 0);

  /* ----------------------- Get constraints A x - b < 0 ----------------------
   */
  // Create Python lists
  py::list AList;
  py::list bList;

  // Loop over all the polyhedrons
  auto polys = decomp.get_polyhedrons();
  for (size_t i = 0; i < path.size() - 1; i++) {

    // Get linear constraints
    const auto pt_inside = (path[i] + path[i + 1]) / 2;
    LinearConstraint2D cs(pt_inside, polys[i].hyperplanes());

    // A matrix
    size_t A_rows = static_cast<size_t>(cs.A().rows());
    size_t A_cols = static_cast<size_t>(cs.A().cols());
    py::array_t<double> A_npArray({A_rows, A_cols});
    double *A_npData = static_cast<double *>(A_npArray.request().ptr);
    Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        A_npData, A_rows, A_cols) = cs.A();
    AList.append(A_npArray);

    // b matrix
    size_t b_rows = static_cast<size_t>(cs.b().rows());
    size_t b_cols = static_cast<size_t>(cs.b().cols());
    py::array_t<double> b_npArray({b_rows, b_cols});
    double *b_npData = static_cast<double *>(b_npArray.request().ptr);
    Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        b_npData, b_rows, b_cols) = cs.b();
    bList.append(b_npArray);
  }

  // return halfspace;
  py::tuple hsTuple(2);
  hsTuple[0] = AList;
  hsTuple[1] = bList;

  return hsTuple;
}

py::tuple convex_decomposition_3D(const py::array_t<double> obs_np,
                                  const py::array_t<double> path_np) {

  /* ------------------------------ Get obstacles -----------------------------
   */

  // Access the numpy matrix data
  auto obs_matrix = obs_np.unchecked<2>(); // 2D matrix assumption
  ssize_t obs_rows = obs_np.shape(0);

  // Access the elements of the matrix
  vec_Vec3f obs;
  Vecf<3> obs_pt;
  for (ssize_t i = 0; i < obs_rows; ++i) {
    obs_pt << obs_matrix(i, 0), obs_matrix(i, 1), obs_matrix(i, 2);
    obs.push_back(obs_pt);
  }

  /* --------------------------- Get path to dilate ---------------------------
   */

  // Access the numpy matrix data
  auto path_matrix = path_np.unchecked<2>(); // 2D matrix assumption
  ssize_t path_rows = path_np.shape(0);

  // Access the elements of the matrix
  vec_Vec3f path;
  Vecf<3> path_pt;
  for (ssize_t i = 0; i < path_rows; ++i) {
    path_pt << path_matrix(i, 0), path_matrix(i, 1), path_matrix(i, 2);
    path.push_back(path_pt);
  }

  /* -------------------------- Convex decomposition --------------------------
   */
  EllipsoidDecomp3D decomp; //(origin, range);
  decomp.set_obs(obs);
  decomp.set_local_bbox(Vec3f(1, 2, 1));
  decomp.dilate(path);

  /* ----------------------- Get constraints A x - b < 0 ----------------------
   */
  // Create Python lists
  py::list AList;
  py::list bList;

  // Loop over all the polyhedrons
  auto polys = decomp.get_polyhedrons();
  for (size_t i = 0; i < path.size() - 1; i++) {

    // Get linear constraints
    const auto pt_inside = (path[i] + path[i + 1]) / 2;
    LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());

    // A matrix
    size_t A_rows = static_cast<size_t>(cs.A().rows());
    size_t A_cols = static_cast<size_t>(cs.A().cols());
    py::array_t<double> A_npArray({A_rows, A_cols});
    double *A_npData = static_cast<double *>(A_npArray.request().ptr);
    Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        A_npData, A_rows, A_cols) = cs.A();
    AList.append(A_npArray);

    // b matrix
    size_t b_rows = static_cast<size_t>(cs.b().rows());
    size_t b_cols = static_cast<size_t>(cs.b().cols());
    py::array_t<double> b_npArray({b_rows, b_cols});
    double *b_npData = static_cast<double *>(b_npArray.request().ptr);
    Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        b_npData, b_rows, b_cols) = cs.b();
    bList.append(b_npArray);
  }

  // return halfspace;
  py::tuple hsTuple(2);
  hsTuple[0] = AList;
  hsTuple[1] = bList;

  return hsTuple;
}

PYBIND11_MODULE(_pydecomp, handle) {
  handle.doc() = "Python wrapper for DecompUtil by Sikang";
  handle.def("convex_decomposition_2D", &convex_decomposition_2D);
  handle.def("convex_decomposition_3D", &convex_decomposition_3D);
}
