// Copyright 2022 Tier IV, Inc.
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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__APPARENT_SAFE_VELOCITY_LIMITER_NODE_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__APPARENT_SAFE_VELOCITY_LIMITER_NODE_HPP_

#include "apparent_safe_velocity_limiter/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/self_pose_listener.hpp>
#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/optional.hpp>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace apparent_safe_velocity_limiter
{

class ApparentSafeVelocityLimiterNode : public rclcpp::Node
{
public:
  explicit ApparentSafeVelocityLimiterNode(const rclcpp::NodeOptions & node_options);

private:
  tier4_autoware_utils::TransformListener transform_listener_{this};
  tier4_autoware_utils::SelfPoseListener self_pose_listener_{this};
  rclcpp::Publisher<Trajectory>::SharedPtr
    pub_trajectory_;  //!< @brief publisher for output trajectory
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pub_debug_markers_;  //!< @brief publisher for debug markers
  rclcpp::Publisher<PointCloud>::SharedPtr
    pub_debug_pointcloud_;  //!< @brief publisher for filtered pointcloud
  rclcpp::Subscription<Trajectory>::SharedPtr
    sub_trajectory_;  //!< @brief subscriber for reference trajectory
  rclcpp::Subscription<PredictedObjects>::SharedPtr
    sub_objects_;  //!< @brief subscribe for dynamic objects
  rclcpp::Subscription<OccupancyGrid>::SharedPtr
    sub_occupancy_grid_;  //!< @brief subscriber for occupancy grid
  rclcpp::Subscription<PointCloud>::SharedPtr
    sub_pointcloud_;  //!< @brief subscriber for obstacle pointcloud
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    sub_odom_;  //!< @brief subscriber for the current velocity

  // cached inputs
  PredictedObjects::ConstSharedPtr dynamic_obstacles_ptr_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_ptr_;
  PointCloud::ConstSharedPtr pointcloud_ptr_;

  // Benchmarking & Debugging
  std::multiset<double> runtimes;

  // parameters
  ProjectionParameters projection_params_;
  ObstacleParameters obstacle_params_;
  VelocityParameters velocity_params_;
  Float distance_buffer_ = static_cast<Float>(declare_parameter<Float>("distance_buffer"));
  Float start_distance_ = static_cast<Float>(declare_parameter<Float>("start_distance"));
  int downsample_factor_ = static_cast<int>(declare_parameter<int>("downsample_factor"));
  Float vehicle_lateral_offset_;
  Float vehicle_front_offset_;
  std::optional<Float> current_ego_velocity_;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /// @brief callback for parameter updates
  /// @param[in] parameters updated parameters and their new values
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  /// @brief callback for input trajectories. Publishes a trajectory with updated velocities
  /// @param[in] msg input trajectory message
  void onTrajectory(const Trajectory::ConstSharedPtr msg);

  /// @brief validate the inputs of the node
  /// @param[in] ego_idx trajectory index closest to the current ego pose
  /// @return true if the inputs are valid
  bool validInputs(const boost::optional<size_t> & ego_idx);
};
}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__APPARENT_SAFE_VELOCITY_LIMITER_NODE_HPP_
