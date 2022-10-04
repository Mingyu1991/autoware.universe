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

#include "static_centerline_optimizer/static_centerline_optimizer_node.hpp"

#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "static_centerline_optimizer/optimization_node.hpp"
#include "static_centerline_optimizer/type_alias.hpp"
#include "static_centerline_optimizer/utils.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using route_handler::RouteHandler;

namespace static_centerline_optimizer
{
namespace
{
Path convert_to_path(const PathWithLaneId & path_with_lane_id)
{
  Path path;
  path.header = path_with_lane_id.header;
  path.drivable_area = path_with_lane_id.drivable_area;
  for (const auto & point : path_with_lane_id.points) {
    path.points.push_back(point.point);
  }

  return path;
}

lanelet::ConstLanelets get_lanelets_from_route(
  const RouteHandler & route_handler, const HADMapRoute & route)
{
  lanelet::ConstLanelets lanelets;
  for (const auto & segment : route.segments) {
    const auto & target_lanelet_id = segment.preferred_primitive_id;
    const auto target_lanelet = route_handler.getLaneletsFromId(target_lanelet_id);
    lanelets.push_back(target_lanelet);
  }

  return lanelets;
}

rclcpp::NodeOptions create_node_options() { return rclcpp::NodeOptions{}; }

rclcpp::QoS create_transient_local_qos() { return rclcpp::QoS{1}.transient_local(); }
}  // namespace

StaticCenterlineOptimizerNode::StaticCenterlineOptimizerNode(const rclcpp::NodeOptions & node_options)
: Node("static_centerlin_optimizer", node_options)
{
}

void StaticCenterlineOptimizerNode::run()
{
  // declare planning setting parameters
  const auto lanelet2_file_name = declare_parameter<std::string>("lanelet2_file_name");
  const int start_lanelet_id = declare_parameter<int>("start_lanelet_id");
  const int end_lanelet_id = declare_parameter<int>("end_lanelet_id");
  const auto lanelet2_output_file_name = "/tmp/lanelet2_map.osm";  // TODO(murooka)

  // load map
  const auto map_bin_ptr = load_map(lanelet2_file_name);

  // create route_handler
  RouteHandler route_handler;
  route_handler.setMap(*map_bin_ptr);

  // calculate check points (= start and goal pose)
  const auto check_points =
    utils::create_check_points(route_handler, start_lanelet_id, end_lanelet_id);
  RCLCPP_INFO(get_logger(), "Calculated check points.");

  // plan route by the mission_planner package
  const auto route = utils::plan_route(map_bin_ptr, check_points);
  RCLCPP_INFO(get_logger(), "Planned route.");

  // calculate center line path with drivable area by the behavior_path_planner package
  const auto lanelets = get_lanelets_from_route(route_handler, route);

  // optimize centerline inside the lane
  const auto optimized_traj_points = optimize_center_line(route_handler, lanelets, check_points);

  // update centerline in map
  utils::update_centerline(route_handler, lanelets, optimized_traj_points);
  RCLCPP_INFO(get_logger(), "Updated centerline in map.");

  // save map with modified center line
  lanelet::write(lanelet2_output_file_name, *route_handler.getLaneletMapPtr());
  RCLCPP_INFO(get_logger(), "Saved map.");
}

HADMapBin::ConstSharedPtr StaticCenterlineOptimizerNode::load_map(
  const std::string & lanelet2_file_name)
{
  // load map by the map_loader package
  const auto map_bin_ptr = utils::create_map(*this, lanelet2_file_name, now());
  if (!map_bin_ptr) {
    RCLCPP_ERROR(get_logger(), "Loading map failed");
    return nullptr;
  }

  // publish map bin msg
  const auto pub_map_bin = create_publisher<HADMapBin>("lanelet2_map_topic", create_transient_local_qos());
  pub_map_bin->publish(*map_bin_ptr);
  RCLCPP_INFO(get_logger(), "Published map.");

  return map_bin_ptr;
}

std::vector<TrajectoryPoint> StaticCenterlineOptimizerNode::optimize_center_line(const RouteHandler & route_handler, const lanelet::ConstLanelets & lanelets, const std::vector<geometry_msgs::msg::Pose> & check_points)
{
  // ego nearest search parameters
  const double ego_nearest_dist_threshold = declare_parameter<double>("ego_nearest_dist_threshold");
  const double ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");

  // extract path with lane id from lanelets
  const auto raw_path_with_lane_id = utils::get_path_with_lane_id(
    route_handler, lanelets, check_points.front(), ego_nearest_dist_threshold,
    ego_nearest_yaw_threshold);

  const auto pub_raw_path_with_lane_id = create_publisher<PathWithLaneId>("raw_path_with_lane_id", create_transient_local_qos());
  pub_raw_path_with_lane_id->publish(raw_path_with_lane_id);
  RCLCPP_INFO(get_logger(), "Calculated raw path with lane id and published.");

  // convert path with lane id to path
  const auto raw_path = convert_to_path(raw_path_with_lane_id);
  const auto pub_raw_path = create_publisher<Path>("debug/raw_centerline", create_transient_local_qos());
  pub_raw_path->publish(raw_path);
  RCLCPP_INFO(get_logger(), "Converted to path and published.");

  // optimize trajectory by the obstacle_avoidance_planner package
  StaticCenterlineOptmizer successive_path_optimizer(create_node_options());
  const auto optimized_traj_points =
    successive_path_optimizer.pathCallback(std::make_shared<Path>(raw_path));
  RCLCPP_INFO(get_logger(), "Optimized trajectory and published.");

  return optimized_traj_points;
}
}  // namespace static_centerline_optimizer
