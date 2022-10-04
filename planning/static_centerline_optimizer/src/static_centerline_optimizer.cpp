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

#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "rclcpp/time.hpp"
#include "static_centerline_optimizer/functions.hpp"
#include "static_centerline_optimizer/optimization_node.hpp"
#include "static_centerline_optimizer/type_alias.hpp"

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
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // create main node
  const auto main_node = rclcpp::Node::make_shared("static_centerline_optimizer");
  const auto current_time = main_node->get_clock()->now();

  // create publisher
  const auto qos = rclcpp::QoS{1}.transient_local();
  const auto pub_map_bin = main_node->create_publisher<HADMapBin>("lanelet2_map_topic", qos);
  const auto pub_raw_path_with_lane_id =
    main_node->create_publisher<PathWithLaneId>("raw_path_with_lane_id", qos);
  const auto pub_raw_path = main_node->create_publisher<Path>("debug/raw_centerline", qos);

  // get ros parameters
  const auto lanelet2_file_name = main_node->declare_parameter<std::string>("lanelet2_file_name");
  const size_t start_lanelet_id = main_node->declare_parameter<int>("start_lanelet_id");
  const size_t end_lanelet_id = main_node->declare_parameter<int>("end_lanelet_id");
  const double ego_nearest_dist_threshold =
    main_node->declare_parameter<double>("ego_nearest_dist_threshold");
  const double ego_nearest_yaw_threshold =
    main_node->declare_parameter<double>("ego_nearest_yaw_threshold");
  const auto lanelet2_output_file_name = "/tmp/lanelet2_map.osm";

  // load map by the map_loader package
  // function0 starts
  const auto map_bin_msg_ptr =
    static_centerline_optimizer::create_map(*main_node, lanelet2_file_name, current_time);
  if (!map_bin_msg_ptr) {
    RCLCPP_ERROR(main_node->get_logger(), "Loading map failed");
    return 0;
  }

  // publish map bin msg
  pub_map_bin->publish(*map_bin_msg_ptr);
  RCLCPP_INFO(main_node->get_logger(), "Published map.");
  // function0 ends

  // create route_handler
  RouteHandler route_handler;
  route_handler.setMap(*map_bin_msg_ptr);

  // calculate check points (= start and goal pose)
  const auto check_points = static_centerline_optimizer::create_check_points(
    route_handler, start_lanelet_id, end_lanelet_id);
  RCLCPP_INFO(main_node->get_logger(), "Calculated check points.");

  // plan route by the mission_planner package
  const auto route = static_centerline_optimizer::plan_route(map_bin_msg_ptr, check_points);
  RCLCPP_INFO(main_node->get_logger(), "Planned route.");

  // calculate center line path with drivable area by the behavior_path_planner package
  const auto lanelets = get_lanelets_from_route(route_handler, route);

  // function1 starts
  const auto raw_path_with_lane_id = static_centerline_optimizer::get_path_with_lane_id(
    route_handler, lanelets, check_points.front(), ego_nearest_dist_threshold,
    ego_nearest_yaw_threshold);
  pub_raw_path_with_lane_id->publish(raw_path_with_lane_id);
  RCLCPP_INFO(main_node->get_logger(), "Calculated raw path with lane id and published.");

  // convert path with lane id to path
  const auto raw_path = convert_to_path(raw_path_with_lane_id);
  pub_raw_path->publish(raw_path);
  RCLCPP_INFO(main_node->get_logger(), "Converted to path and published.");

  // optimize trajectory by the obstacle_avoidance_planner package
  static_centerline_optimizer::StaticCenterlineOptmizer successive_path_optimizer(
    create_node_options());
  const auto optimized_traj_points =
    successive_path_optimizer.pathCallback(std::make_shared<Path>(raw_path));
  RCLCPP_INFO(main_node->get_logger(), "Optimized trajectory and published.");
  // function1 ends

  // update centerline in map
  static_centerline_optimizer::update_centerline(route_handler, lanelets, optimized_traj_points);
  RCLCPP_INFO(main_node->get_logger(), "Updated centerline in map.");

  // save map with modified center line
  lanelet::write(lanelet2_output_file_name, *route_handler.getLaneletMapPtr());
  RCLCPP_INFO(main_node->get_logger(), "Saved map.");

  // NOTE: spin node to show debug path/trajectory in rviz with transient local
  rclcpp::spin(main_node);
  rclcpp::shutdown();

  return 0;
}
