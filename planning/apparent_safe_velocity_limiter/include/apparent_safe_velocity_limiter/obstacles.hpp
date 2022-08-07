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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__OBSTACLES_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__OBSTACLES_HPP_

#include "apparent_safe_velocity_limiter/obstacles.hpp"
#include "apparent_safe_velocity_limiter/parameters.hpp"
#include "apparent_safe_velocity_limiter/types.hpp"

#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <string>

namespace apparent_safe_velocity_limiter
{

/// @brief create a polygon from an object represented by a pose and a size
/// @param [in] pose pose of the object
/// @param [in] dimensions dimensions of the object
/// @param [in] buffer buffer to add to the dimensions of the object
/// @return polygon of the object
polygon_t createObjectPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & dimensions,
  const double buffer);

/// @brief create polygons from a set of predicted object
/// @param [in] objects objects from which to create polygons
/// @param [in] buffer buffer to add to the objects dimensions
/// @param [in] min_velocity objects with velocity lower will be ignored
/// @return polygons of the objects
multipolygon_t createObjectPolygons(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const double buffer,
  const double min_velocity);

/// @brief create linestrings around obstacles
/// @param[in] occupancy_grid occupancy grid
/// @param[in] pointcloud pointcloud
/// @param[in] masks masks used to discard some obstacles
/// @param[in] transform_listener object used to retrieve the latest transform
/// @param[in] target_frame frame of the returned obstacles
/// @param[in] obstacle_params obstacle parameters
/// @return linestrings representing obstacles to avoid
Obstacles createObstacles(
  const OccupancyGrid & occupancy_grid, const PointCloud & pointcloud, const ObstacleMasks & masks,
  tier4_autoware_utils::TransformListener & transform_listener, const std::string & target_frame,
  const ObstacleParameters & obstacle_params);

/// @brief filter obstacles with the given negative and positive masks
/// @param[in] obstacles obstacles to filter
/// @param[in] masks masks used to discard some obstacles
/// @return obstacles that are inside the positive mask and outside of the negative masks
Obstacles filterObstacles(const Obstacles & obstacles, const ObstacleMasks & masks);
}  // namespace apparent_safe_velocity_limiter
#endif  // APPARENT_SAFE_VELOCITY_LIMITER__OBSTACLES_HPP_
