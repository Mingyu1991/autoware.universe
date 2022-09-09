// Copyright 2022 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "utils_act/writetopath.hpp"
#include "utils_act/act_utils.hpp"
#include <vector>
#include <algorithm>
#include <numeric>
#include <random>
#include "utils_act/timekeep.hpp"
#include "splines/bspline_interpolator_templated.hpp"

int main()
{

  auto log_path = getOutputPath() / "bspline_interp_templated";


  // <--------------- Dimension Expansion -------------------------------->
  // Generate a noisy sinusoidal signal with arc-length parametrization. This is our test signal.
  const size_t Nin = 120;
  const size_t Nout = 50;


  // Generate x.
  double kx = 8;
  auto xvec = ns_utils::linspace<double>(0.0, kx, Nin);
  auto xvec_int = ns_utils::linspace<int>(0, kx, 8);

  // Generate y = sin(x).
  double cy = 10;
  std::vector<double> yvec;
  std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [&](auto const &x)
  {
    return cy * sin(x);
  });

  // Arc-length parametrization.
  std::vector<double> dx; //{1, 0.0};
  std::vector<double> dy; //{1, 0.0};

  std::adjacent_difference(xvec.begin(), xvec.end(), std::back_inserter(dx));
  std::adjacent_difference(yvec.begin(), yvec.end(), std::back_inserter(dy));

  // Define arc-length cumsum()
  std::vector<double> svec;
  std::transform(dx.cbegin(), dx.cend(), dy.cbegin(), std::back_inserter(svec),
                 [](auto dxi, auto dyi)
                 {
                   static double ds = 0.0;
                   ds += std::hypot(dxi, dyi);

                   return ds;
                 });

  // EIGEN IMPLEMENTATION.
  Eigen::MatrixXd
    xe = Eigen::Map<Eigen::Matrix<double, Nin, 1 >>(xvec.data());

  Eigen::MatrixXd ye(xe.unaryExpr([&](auto x)
                                  { return cy * sin(x); }));

  Eigen::MatrixXd ze(xe.unaryExpr([&](auto x)
                                  { return 2 * cos(x) - 3 * sin(x); }));

  Eigen::MatrixXd se;
  se = Eigen::Map<Eigen::Matrix<double, Nin, 1 >>(svec.data());

  // Create a matrix to be interpolated into a new size.
  auto yze = ns_eigen_utils::hstack<double>(ye, ze);
  Eigen::MatrixXd yz_interp_newsize;
  writeToFile(log_path, yze, "yze");

  // Create a new smoothing spline.
  ns_splines::BSplineInterpolatorTemplated<Nin, Nout> interpolating_bspline(0.3, true);

  // Different size multi-column interpolation. Expanding the data points.
  interpolating_bspline.InterpolateImplicitCoordinates(yze, yz_interp_newsize);

  writeToFile(log_path, yz_interp_newsize, "yz_interp_newsize");

  // Save coordinates to plot on the same scale
  auto tvec_base = Eigen::VectorXd::LinSpaced(Nin, 0.0, 1.);
  auto tvec_new = Eigen::VectorXd::LinSpaced(Nout, 0.0, 1.0);

  writeToFile(log_path, tvec_base, "tvec_base");
  writeToFile(log_path, tvec_new, "tvec_new");

  // TEST with New Constructors, base coordinates and new coordinates are given.
  // Save coordinates to plot on the same scale
  // size_t const new_size_1 = 60; //static_cast<size_t>(base_size / 2);
  //    ns_eigen_utils::printEigenMat(tvec_base_1);

  // Test copy
  ns_splines::BSplineInterpolatorTemplated<Nin, Nout> bspline_copy;
  bspline_copy = ns_splines::BSplineInterpolatorTemplated<Nin, Nout>(0.5, true);

  Eigen::MatrixXd yz_interp_copy;
  bspline_copy.InterpolateImplicitCoordinates(yze, yz_interp_copy);
  writeToFile(log_path, yz_interp_copy, "yz_interp_copy");
  ns_eigen_utils::printEigenMat(yz_interp_copy);


  // Test move.
  Eigen::MatrixXd yz_interp_move;
  ns_splines::BSplineInterpolatorTemplated<Nin, Nout> bspline_move(std::move(bspline_copy));
  bspline_move.InterpolateImplicitCoordinates(yze, yz_interp_move);
  writeToFile(log_path, yz_interp_move, "yz_interp_move");
  ns_eigen_utils::printEigenMat(yz_interp_move);

  return 0;
}