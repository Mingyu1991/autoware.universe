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

#include "obstacle_avoidance_planner/eb_path_optimizer.hpp"

#include "obstacle_avoidance_planner/utils.hpp"
#include "motion_utils/motion_utils.hpp"

#include "geometry_msgs/msg/vector3.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <vector>

namespace
{
// make positive semidefinite matrix for objective function
// reference: https://ieeexplore.ieee.org/document/7402333
Eigen::MatrixXd makePMatrix(const int num_points)
{
  // create P block matrix
  Eigen::MatrixXd P_block = Eigen::MatrixXd::Zero(num_points, num_points);
  for (int r = 0; r < num_points; ++r) {
    for (int c = 0; c < num_points; ++c) {
      if (r == c) {
        if (r == 0 || r == num_points - 1) {
          P_block(r, c) = 1.0;
        } else if (r == 1 || r == num_points - 2) {
          P_block(r, c) = 5.0;
        } else {
          P_block(r, c) = 6.0;
        }
      } else if (std::abs(c - r) == 1) {
        if (r == 0 || r == num_points - 1) {
          P_block(r, c) = -2.0;
        } else if (c == 0 || c == num_points - 1) {
          P_block(r, c) = -2.0;
        } else {
          P_block(r, c) = -4.0;
        }
      } else if (std::abs(c - r) == 2) {
        P_block(r, c) = 1.0;
      } else {
        P_block(r, c) = 0.0;
      }
    }
  }

  // create P matrix
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(num_points * 2, num_points * 2);
  P.block(0, 0, num_points, num_points) = P_block;
  P.block(num_points, num_points, num_points, num_points) = P_block;

  return P;
}

// make default linear constrain matrix
Eigen::MatrixXd makeAMatrix(const int num_points)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_points * 2, num_points * 2);
  for (int i = 0; i < num_points * 2; ++i) {
    if (i < num_points) {
      A(i, i + num_points) = 1.0;
    } else {
      A(i, i - num_points) = 1.0;
    }
  }
  return A;
}
}


EBPathOptimizer::EBPathOptimizer(
  const bool is_showing_debug_info, const TrajectoryParam & traj_param, const EBParam & eb_param,
  const VehicleParam & vehicle_param)
: is_showing_debug_info_(is_showing_debug_info),
  qp_param_(eb_param.qp_param),
  traj_param_(traj_param),
  eb_param_(eb_param),
  vehicle_param_(vehicle_param)
{
  const Eigen::MatrixXd p = makePMatrix(eb_param_.num_sampling_points_for_eb);
  const Eigen::MatrixXd a = makeAMatrix(eb_param_.num_sampling_points_for_eb);

  const int num_points = eb_param_.num_sampling_points_for_eb;
  const std::vector<double> q(num_points * 2, 0.0);
  const std::vector<double> lower_bound(num_points * 2, 0.0);
  const std::vector<double> upper_bound(num_points * 2, 0.0);

  osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(
    p, a, q, lower_bound, upper_bound, qp_param_.eps_abs);
  osqp_solver_ptr_->updateEpsRel(qp_param_.eps_rel);
  osqp_solver_ptr_->updateMaxIter(qp_param_.max_iteration);
}

boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
EBPathOptimizer::getEBTrajectory(
  const geometry_msgs::msg::Pose & ego_pose, const autoware_auto_planning_msgs::msg::Path & path,
  const std::unique_ptr<Trajectories> & prev_trajs, const double current_ego_vel,
  std::shared_ptr<DebugData> debug_data_ptr)
{
  stop_watch_.tic(__func__);

  current_ego_vel_ = current_ego_vel;

  // get candidate points for optimization
  // decide fix or non fix, might not required only for smoothing purpose
  const CandidatePoints candidate_points =
    getCandidatePoints(ego_pose, path.points, prev_trajs, debug_data_ptr);
  if (candidate_points.fixed_points.empty() && candidate_points.non_fixed_points.empty()) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
      "return boost::none since empty candidate points");
    return boost::none;
  }

  // get optimized smooth points with elastic band
  const auto eb_traj_points = getOptimizedTrajectory(path, candidate_points, debug_data_ptr);
  if (!eb_traj_points) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
      "return boost::none since smoothing failed");
    return boost::none;
  }

  debug_data_ptr->msg_stream << "      " << __func__ << ":= " << stop_watch_.toc(__func__)
                             << " [ms]\n";
  return eb_traj_points;
}

boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
EBPathOptimizer::getOptimizedTrajectory(
  const autoware_auto_planning_msgs::msg::Path & path, const CandidatePoints & candidate_points,
  std::shared_ptr<DebugData> debug_data_ptr)
{
  stop_watch_.tic(__func__);

  // get constrain rectangles around each point
  auto full_points = candidate_points.fixed_points;
  full_points.insert(
    full_points.end(), candidate_points.non_fixed_points.begin(),
    candidate_points.non_fixed_points.end());

  // interpolate points for logic purpose
  const auto interpolated_points = motion_utils::resamplePath(full_points, eb_param_.delta_arc_length_for_eb);
  if (interpolated_points.empty()) {
    return boost::none;
  }

  // debug_data_ptr->interpolated_points = interpolated_points; // TODO

  // number of optimizing points
  const int farthest_idx = std::min(
    (eb_param_.num_sampling_points_for_eb - 1), static_cast<int>(interpolated_points.size() - 1));
  // number of fixed points in interpolated_points
  const int num_fixed_points = 17; // TODO
  // getNumFixedPoints(candidate_points.fixed_points, interpolated_points, farthest_idx);
  std::cerr << num_fixed_points << std::endl;
  // TODO(murooka) try this printing. something may be wrong
  // std::cerr << num_fixed_points << std::endl;

  // if `farthest_idx` is lower than `number_of_sampling_points`, duplicate the point at the end of
  // `interpolated_points`
  // This aims for using hotstart by not changing the size of matrix
  std::vector<geometry_msgs::msg::Pose> padded_interpolated_points =
    getPaddedInterpolatedPoints(interpolated_points, farthest_idx);

  // calculate rectangle size vector
  const auto rect_size_vec = getRectangleSizeVector(
    padded_interpolated_points, num_fixed_points, farthest_idx);

  // update constrain for elastic band's QP
  updateConstrain(padded_interpolated_points, rect_size_vec);

  // optimize trajectory by elastic band
  const auto optimized_points =
    optimizeTrajectory(padded_interpolated_points, debug_data_ptr);
  if (!optimized_points) {
    return boost::none;
  }

  // convert to trajectory
  const auto traj_points =
    convertOptimizedPointsToTrajectory(optimized_points.get(), farthest_idx);

  debug_data_ptr->msg_stream << "        " << __func__ << ":= " << stop_watch_.toc(__func__)
                             << " [ms]\n";
  return traj_points;
}

std::vector<double> EBPathOptimizer::getRectangleSizeVector(
  const std::vector<geometry_msgs::msg::Pose> & interpolated_points, const int num_fixed_points,
  const int farthest_point_idx)
{
  std::vector<double> rect_size_vec;
  for (int i = 0; i < eb_param_.num_sampling_points_for_eb; ++i) {
    if (i == 0 || i == 1 || i >= farthest_point_idx - 1 || i < num_fixed_points - 1) {
      rect_size_vec.push_back(eb_param_.clearance_for_fixing);
    } else {
      rect_size_vec.push_back(eb_param_.clearance_for_only_smoothing);
    }
  }

  return rect_size_vec;
}

void EBPathOptimizer::updateConstrain(
  const std::vector<geometry_msgs::msg::Pose> & interpolated_points,
  const std::vector<double> & rect_size_vec)
{
  const int num_points = eb_param_.num_sampling_points_for_eb;

  Eigen::MatrixXd A = makeAMatrix(eb_param_.num_sampling_points_for_eb);
  std::vector<double> lower_bound(num_points * 2, 0.0);
  std::vector<double> upper_bound(num_points * 2, 0.0);
  for (int i = 0; i < num_points; ++i) {
    const auto constrain =
      getConstrainLinesFromConstrainRectangle(interpolated_points.at(i), rect_size_vec.at(i));
    // constraint for x
    A(i, i) = constrain.x.coef(0);
    A(i, i + num_points) = constrain.x.coef(1);
    lower_bound[i] = constrain.x.lower_bound;
    upper_bound[i] = constrain.x.upper_bound;
    // constraint for y
    A(i + num_points, i) = constrain.y.coef(0);
    A(i + num_points, i + num_points) = constrain.y.coef(1);
    lower_bound[i + num_points] = constrain.y.lower_bound;
    upper_bound[i + num_points] = constrain.x.upper_bound;
  }

  osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
  osqp_solver_ptr_->updateA(A);
}

EBPathOptimizer::ConstrainLines EBPathOptimizer::getConstrainLinesFromConstrainRectangle(
  const geometry_msgs::msg::Pose & pose, const double rect_size)
{
  ConstrainLines constrain;
  const double theta = tf2::getYaw(pose.orientation);

  { // x constrain
    constrain.x.coef = Eigen::Vector2d(std::sin(theta), -std::cos(theta));
    Eigen::Vector2d upper_point(pose.position.x - rect_size / 2.0 * std::sin(theta), pose.position.y + rect_size / 2.0 * std::cos(theta));
    Eigen::Vector2d lower_point(pose.position.x + rect_size / 2.0 * std::sin(theta), pose.position.y - rect_size / 2.0 * std::cos(theta));
    const double upper_bound = constrain.x.coef.transpose() * upper_point;
    const double lower_bound = constrain.x.coef.transpose() * lower_point;
    constrain.x.upper_bound = std::max(upper_bound, lower_bound);
    constrain.x.lower_bound = std::min(upper_bound, lower_bound);
  }

  { // y constrain
    constrain.y.coef = Eigen::Vector2d(std::cos(theta), -std::sin(theta));
    Eigen::Vector2d upper_point(pose.position.x + rect_size / 2.0 * std::cos(theta), pose.position.y + rect_size / 2.0 * std::sin(theta));
    Eigen::Vector2d lower_point(pose.position.x - rect_size / 2.0 * std::cos(theta), pose.position.y - rect_size / 2.0 * std::sin(theta));
    const double upper_bound = constrain.y.coef.transpose() * upper_point;
    const double lower_bound = constrain.y.coef.transpose() * lower_point;
    constrain.y.upper_bound = std::max(upper_bound, lower_bound);
    constrain.y.lower_bound = std::min(upper_bound, lower_bound);
  }

  return constrain;
}

boost::optional<std::vector<double>>
EBPathOptimizer::optimizeTrajectory(
  const std::vector<geometry_msgs::msg::Pose> & padded_interpolated_points,
  std::shared_ptr<DebugData> debug_data_ptr)
{
  stop_watch_.tic(__func__);

  // update QP param
  osqp_solver_ptr_->updateEpsRel(qp_param_.eps_rel);
  osqp_solver_ptr_->updateEpsAbs(qp_param_.eps_abs);

  // solve QP
  const auto result = osqp_solver_ptr_->optimize();

  // check status
  const auto status = std::get<3>(result);
  utils::logOSQPSolutionStatus(std::get<3>(result), "EB: ");
  if (status != 1) {
    utils::logOSQPSolutionStatus(status, "EB: ");
    return boost::none;
  }

  if (debug_data_ptr) {
    debug_data_ptr->msg_stream << "          " << __func__ << ":= " << stop_watch_.toc(__func__)
                               << " [ms]\n";
  }

  const auto optimized_points = std::get<0>(result);
  return optimized_points;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
EBPathOptimizer::convertOptimizedPointsToTrajectory(
  const std::vector<double> optimized_points, const int farthest_idx)
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> traj_points;
  for (int i = 0; i <= farthest_idx; i++) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = optimized_points[i];
    tmp_point.pose.position.y = optimized_points[i + eb_param_.num_sampling_points_for_eb];
    traj_points.push_back(tmp_point);
  }
  for (size_t i = 0; i < traj_points.size(); i++) {
    if (i > 0) {
      traj_points[i].pose.orientation = geometry_utils::getQuaternionFromPoints(
        traj_points[i].pose.position, traj_points[i - 1].pose.position);
    } else if (i == 0 && traj_points.size() > 1) {
      traj_points[i].pose.orientation = geometry_utils::getQuaternionFromPoints(
        traj_points[i + 1].pose.position, traj_points[i].pose.position);
    }
  }
  return traj_points;
}

std::vector<geometry_msgs::msg::Pose> EBPathOptimizer::getFixedPoints(
  const geometry_msgs::msg::Pose & ego_pose,
  [[maybe_unused]] const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & prev_trajs)
{
  /* use of prev_traj_points(fine resolution) instead of prev_opt_traj(coarse resolution)
     stabilize trajectory's yaw*/
  if (prev_trajs) {
    if (prev_trajs->smoothed_trajectory.empty()) {
      std::vector<geometry_msgs::msg::Pose> empty_points;
      return empty_points;
    }
    const auto opt_begin_idx = motion_utils::findNearestIndex(
      prev_trajs->smoothed_trajectory, ego_pose, std::numeric_limits<double>::max(),
      traj_param_.delta_yaw_threshold_for_closest_point);
    const int begin_idx = opt_begin_idx ? *opt_begin_idx : 0;
    const int backward_fixing_idx = std::max(
      static_cast<int>(
        begin_idx -
        traj_param_.backward_fixing_distance / traj_param_.delta_arc_length_for_trajectory),
      0);

    // NOTE: Fixed length of EB has to be longer than that of MPT.
    constexpr double forward_fixed_length_margin = 5.0;
    const double forward_fixed_length = std::max(
      current_ego_vel_ * traj_param_.forward_fixing_min_time + forward_fixed_length_margin,
      traj_param_.forward_fixing_min_distance);

    const int forward_fixing_idx = std::min(
      static_cast<int>(
        begin_idx + forward_fixed_length / traj_param_.delta_arc_length_for_trajectory),
      static_cast<int>(prev_trajs->smoothed_trajectory.size() - 1));
    std::vector<geometry_msgs::msg::Pose> fixed_points;
    for (int i = backward_fixing_idx; i <= forward_fixing_idx; i++) {
      fixed_points.push_back(prev_trajs->smoothed_trajectory.at(i).pose);
    }
    return fixed_points;
  } else {
    std::vector<geometry_msgs::msg::Pose> empty_points;
    return empty_points;
  }
}

EBPathOptimizer::CandidatePoints EBPathOptimizer::getCandidatePoints(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & prev_trajs, std::shared_ptr<DebugData> debug_data_ptr)
{
  const std::vector<geometry_msgs::msg::Pose> fixed_points =
    getFixedPoints(ego_pose, path_points, prev_trajs);
  if (fixed_points.empty()) {
    CandidatePoints candidate_points = getDefaultCandidatePoints(path_points);
    return candidate_points;
  }

  // try to find non-fix points
  const auto opt_begin_idx = motion_utils::findNearestIndex(
    path_points, fixed_points.back(), std::numeric_limits<double>::max(),
    traj_param_.delta_yaw_threshold_for_closest_point);
  if (!opt_begin_idx) {
    CandidatePoints candidate_points;
    candidate_points.fixed_points = fixed_points;
    candidate_points.begin_path_idx = path_points.size();
    candidate_points.end_path_idx = path_points.size();
    return candidate_points;
  }

  const int begin_idx = std::min(
    static_cast<int>(opt_begin_idx.get()) + eb_param_.num_offset_for_begin_idx,
    static_cast<int>(path_points.size()) - 1);

  std::vector<geometry_msgs::msg::Pose> non_fixed_points;
  for (size_t i = begin_idx; i < path_points.size(); i++) {
    non_fixed_points.push_back(path_points[i].pose);
  }
  CandidatePoints candidate_points;
  candidate_points.fixed_points = fixed_points;
  candidate_points.non_fixed_points = non_fixed_points;
  candidate_points.begin_path_idx = begin_idx;
  candidate_points.end_path_idx = path_points.size() - 1;

  debug_data_ptr->fixed_points = candidate_points.fixed_points;
  debug_data_ptr->non_fixed_points = candidate_points.non_fixed_points;
  return candidate_points;
}

std::vector<geometry_msgs::msg::Pose> EBPathOptimizer::getPaddedInterpolatedPoints(
  const std::vector<geometry_msgs::msg::Pose> & interpolated_points, const int farthest_point_idx)
{
  std::vector<geometry_msgs::msg::Pose> padded_interpolated_points;
  for (int i = 0; i < eb_param_.num_sampling_points_for_eb; ++i) {
    if (farthest_point_idx < i) {
      padded_interpolated_points.push_back(interpolated_points.at(farthest_point_idx));
    } else {
      padded_interpolated_points.push_back(interpolated_points.at(i));
    }
  }
  return padded_interpolated_points;
}

EBPathOptimizer::CandidatePoints EBPathOptimizer::getDefaultCandidatePoints(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points)
{
  double accum_arc_length = 0;
  int end_path_idx = 0;
  std::vector<geometry_msgs::msg::Pose> fixed_points;
  for (size_t i = 0; i < path_points.size(); i++) {
    if (i > 0) {
      accum_arc_length += tier4_autoware_utils::calcDistance2d(
        path_points[i].pose.position, path_points[i - 1].pose.position);
    }
    if (
      accum_arc_length > eb_param_.num_sampling_points_for_eb * eb_param_.delta_arc_length_for_eb) {
      break;
    }
    end_path_idx = i;
    fixed_points.push_back(path_points[i].pose);
  }
  CandidatePoints candidate_points;
  candidate_points.fixed_points = fixed_points;
  candidate_points.begin_path_idx = 0;
  candidate_points.end_path_idx = end_path_idx;
  return candidate_points;
}

int EBPathOptimizer::getNumFixedPoints(
  const std::vector<geometry_msgs::msg::Pose> & fixed_points,
  const std::vector<geometry_msgs::msg::Pose> & interpolated_points, const int farthest_idx)
{
  int num_fixed_points = 0;
  if (!fixed_points.empty() && !interpolated_points.empty()) {
    std::vector<geometry_msgs::msg::Pose> interpolated_points = motion_utils::resamplePath(fixed_points, eb_param_.delta_arc_length_for_eb);
    std::cerr << "PO2 " << interpolated_points.size() << std::endl;
    num_fixed_points = interpolated_points.size();
  }
  num_fixed_points = std::min(num_fixed_points, farthest_idx);
  return num_fixed_points;
}
