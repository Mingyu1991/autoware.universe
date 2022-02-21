/*
 * Copyright 2021 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FRENET_PLANNER__TRANSFORM__SPLINE_TRANSFORM_HPP
#define FRENET_PLANNER__TRANSFORM__SPLINE_TRANSFORM_HPP

#include "frenet_planner/structures.hpp"

#include <vector>

namespace frenet_planner::transform
{
using frenet_planner::FrenetPoint;
using frenet_planner::Point;

class Spline
{
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
  std::vector<double> h_;

public:
  Spline() = delete;
  Spline(const std::vector<double> & base_index, const std::vector<double> & base_value);
  explicit Spline(const std::vector<Point> & points);
  bool interpolate(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index, std::vector<double> & return_value);
  [[nodiscard]] double value(const double query, const std::vector<double> & base_index) const;
  [[nodiscard]] double velocity(const double query, const std::vector<double> & base_index) const;
  [[nodiscard]] double acceleration(
    const double query, const std::vector<double> & base_index) const;

private:
  void generateSpline(
    const std::vector<double> & base_index, const std::vector<double> & base_value);
  [[nodiscard]] bool isIncrease(const std::vector<double> & x) const;
  [[nodiscard]] bool isValidInput(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index) const;
  [[nodiscard]] std::vector<double> solveLinearSystem(
    const double omega, const size_t max_iter) const;
  [[nodiscard]] bool isConvergeL1(
    const std::vector<double> & r1, const std::vector<double> & r2,
    const double converge_range) const;
};

class Spline2D
{
  std::vector<double> s_;
  Spline x_spline_;
  Spline y_spline_;

public:
  Spline2D(const std::vector<double> & x, const std::vector<double> & y);
  [[nodiscard]] FrenetPoint frenet(const Point & p) const;
  [[nodiscard]] Point cartesian(const double s) const;
  [[nodiscard]] Point cartesian(const FrenetPoint & fp) const;
  [[nodiscard]] double curvature(const double s) const;
  [[nodiscard]] double yaw(const double s) const;

private:
  std::vector<double> arcLength(const std::vector<double> & x, const std::vector<double> & y);
};
}  // namespace frenet_planner::transform

#endif  // FRENET_PLANNER__TRANSFORM__SPLINE_TRANSFORM_HPP