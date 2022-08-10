// Copyright 2020 Tier IV, Inc.
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

#ifndef OBSTACLE_AVOIDANCE_PLANNER__EB_PATH_OPTIMIZER_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__EB_PATH_OPTIMIZER_HPP_

#include "eigen3/Eigen/Core"
#include "obstacle_avoidance_planner/common_structs.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

#include "boost/optional.hpp"

#include <memory>
#include <utility>
#include <vector>

class EBPathOptimizer
{
public:
  struct ConstrainLines
  {
    struct Constrain
    {
      Eigen::Vector2d coef;
      double upper_bound;
      double lower_bound;
    };

    Constrain x;
    Constrain y;
  };

  EBPathOptimizer(
    const bool is_showing_debug_info, const TrajectoryParam & traj_param, const EBParam & eb_param,
    const VehicleParam & vehicle_param);

  boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>> getEBTrajectory(
    const geometry_msgs::msg::Pose & ego_pose, const autoware_auto_planning_msgs::msg::Path & path,
    const std::unique_ptr<Trajectories> & prev_trajs, const double current_ego_vel,
    std::shared_ptr<DebugData> debug_data_ptr);

private:
  struct CandidatePoints
  {
    std::vector<geometry_msgs::msg::Pose> fixed_points;
    std::vector<geometry_msgs::msg::Pose> non_fixed_points;
    int begin_path_idx;
    int end_path_idx;
  };

  const bool is_showing_debug_info_;
  const QPParam qp_param_;
  const TrajectoryParam traj_param_;
  const EBParam eb_param_;
  const VehicleParam vehicle_param_;

  std::unique_ptr<autoware::common::osqp::OSQPInterface> osqp_solver_ptr_;

  double current_ego_vel_;

  mutable tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;


  std::vector<double> getRectangleSizeVector(
    const std::vector<geometry_msgs::msg::Pose> & interpolated_points, const int num_fixed_points,
    const int farthest_point_idx);

  void updateConstrain(
    const std::vector<geometry_msgs::msg::Pose> & interpolated_points,
    const std::vector<double> & rect_size_vec);

  ConstrainLines getConstrainLinesFromConstrainRectangle(
    const geometry_msgs::msg::Pose & interpolated_point,
    const double rect_size);

  boost::optional<std::vector<double>>
  optimizeTrajectory(
    const std::vector<geometry_msgs::msg::Pose> & padded_interpolated_points,
    std::shared_ptr<DebugData> debug_data_ptr);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
  convertOptimizedPointsToTrajectory(
    const std::vector<double> optimized_points, const int farthest_idx);




  std::vector<geometry_msgs::msg::Pose> getPaddedInterpolatedPoints(
    const std::vector<geometry_msgs::msg::Pose> & interpolated_points, const int farthest_idx);

  int getNumFixedPoints(
    const std::vector<geometry_msgs::msg::Pose> & fixed_points,
    const std::vector<geometry_msgs::msg::Pose> & interpolated_points, const int farthest_idx);

  boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
  getOptimizedTrajectory(
    const autoware_auto_planning_msgs::msg::Path & path, const CandidatePoints & candidate_points,
    std::shared_ptr<DebugData> debug_data_ptr);

  std::vector<geometry_msgs::msg::Pose> getFixedPoints(
    const geometry_msgs::msg::Pose & ego_pose,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<Trajectories> & prev_optimized_points);

  CandidatePoints getCandidatePoints(
    const geometry_msgs::msg::Pose & ego_pose,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<Trajectories> & prev_trajs, std::shared_ptr<DebugData> debug_data_ptr);

  CandidatePoints getDefaultCandidatePoints(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points);
};

#endif  // OBSTACLE_AVOIDANCE_PLANNER__EB_PATH_OPTIMIZER_HPP_
