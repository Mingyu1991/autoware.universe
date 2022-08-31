// Copyright 2019 Autoware Foundation
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

#include "default_planner.hpp"

#include "utility_functions.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <tf2/utils.h>

#include <limits>
#include <vector>

namespace
{
using RouteSections = std::vector<autoware_auto_mapping_msgs::msg::HADMapSegment>;
RouteSections combine_consecutive_route_sections(
  const RouteSections & route_sections1, const RouteSections & route_sections2)
{
  RouteSections route_sections;
  route_sections.reserve(route_sections1.size() + route_sections2.size());
  if (!route_sections1.empty()) {
    // remove end route section because it is overlapping with first one in next route_section
    route_sections.insert(route_sections.end(), route_sections1.begin(), route_sections1.end() - 1);
  }
  if (!route_sections2.empty()) {
    route_sections.insert(route_sections.end(), route_sections2.begin(), route_sections2.end());
  }
  return route_sections;
}

bool is_route_looped(const RouteSections & route_sections)
{
  for (std::size_t i = 0; i < route_sections.size(); i++) {
    const auto & route_section = route_sections.at(i);
    for (const auto & lane_id : route_section.primitives) {
      for (std::size_t j = i + 1; j < route_sections.size(); j++) {
        const auto & future_route_section = route_sections.at(j);
        if (exists(future_route_section.primitives, lane_id)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool is_in_lane(const lanelet::ConstLanelet & lanelet, const lanelet::ConstPoint3d & point)
{
  // check if goal is on a lane at appropriate angle
  const auto distance = boost::geometry::distance(
    lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(point).basicPoint());
  constexpr double th_distance = std::numeric_limits<double>::epsilon();
  return distance < th_distance;
}

bool is_in_parking_space(
  const lanelet::ConstLineStrings3d & parking_spaces, const lanelet::ConstPoint3d & point)
{
  for (const auto & parking_space : parking_spaces) {
    lanelet::ConstPolygon3d parking_space_polygon;
    if (!lanelet::utils::lineStringWithWidthToPolygon(parking_space, &parking_space_polygon)) {
      continue;
    }

    const double distance = boost::geometry::distance(
      lanelet::utils::to2D(parking_space_polygon).basicPolygon(),
      lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    if (distance < th_distance) {
      return true;
    }
  }
  return false;
}

bool is_in_parking_lot(
  const lanelet::ConstPolygons3d & parking_lots, const lanelet::ConstPoint3d & point)
{
  for (const auto & parking_lot : parking_lots) {
    const double distance = boost::geometry::distance(
      lanelet::utils::to2D(parking_lot).basicPolygon(), lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    if (distance < th_distance) {
      return true;
    }
  }
  return false;
}

double project_goal_to_map(
  const lanelet::Lanelet & lanelet_component, const lanelet::ConstPoint3d & goal_point)
{
  const lanelet::ConstLineString3d center_line =
    lanelet::utils::generateFineCenterline(lanelet_component);
  lanelet::BasicPoint3d project = lanelet::geometry::project(center_line, goal_point.basicPoint());
  return project.z();
}

}  // anonymous namespace

namespace mission_planner::lanelet2
{

void DefaultPlanner::initialize(rclcpp::Node * node)
{
  node_ = node;
  map_subscriber_ = node_->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/vector_map", rclcpp::QoS{10}.transient_local(),
    std::bind(&DefaultPlanner::map_callback, this, std::placeholders::_1));
}

bool DefaultPlanner::ready() const { return is_graph_ready_; }

void DefaultPlanner::map_callback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  route_handler_.setMap(*msg);
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
  shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);
  is_graph_ready_ = true;
}

PlannerPlugin::MarkerArray DefaultPlanner::visualize(const HADMapRoute & route) const
{
  lanelet::ConstLanelets route_lanelets;
  lanelet::ConstLanelets end_lanelets;
  lanelet::ConstLanelets normal_lanelets;
  lanelet::ConstLanelets goal_lanelets;

  for (const auto & route_section : route.segments) {
    for (const auto & lane_id : route_section.primitives) {
      auto lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id.id);
      route_lanelets.push_back(lanelet);
      if (route_section.preferred_primitive_id == lane_id.id) {
        goal_lanelets.push_back(lanelet);
      } else {
        end_lanelets.push_back(lanelet);
      }
    }
  }

  std_msgs::msg::ColorRGBA cl_route;
  std_msgs::msg::ColorRGBA cl_ll_borders;
  std_msgs::msg::ColorRGBA cl_end;
  std_msgs::msg::ColorRGBA cl_normal;
  std_msgs::msg::ColorRGBA cl_goal;
  set_color(&cl_route, 0.2, 0.4, 0.2, 0.05);
  set_color(&cl_goal, 0.2, 0.4, 0.4, 0.05);
  set_color(&cl_end, 0.2, 0.2, 0.4, 0.05);
  set_color(&cl_normal, 0.2, 0.4, 0.2, 0.05);
  set_color(&cl_ll_borders, 1.0, 1.0, 1.0, 0.999);

  visualization_msgs::msg::MarkerArray route_marker_array;
  insert_marker_array(
    &route_marker_array,
    lanelet::visualization::laneletsBoundaryAsMarkerArray(route_lanelets, cl_ll_borders, false));
  insert_marker_array(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "route_lanelets", route_lanelets, cl_route));
  insert_marker_array(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("end_lanelets", end_lanelets, cl_end));
  insert_marker_array(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "normal_lanelets", normal_lanelets, cl_normal));
  insert_marker_array(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("goal_lanelets", goal_lanelets, cl_goal));

  return route_marker_array;
}

bool DefaultPlanner::is_goal_valid(const geometry_msgs::msg::Pose & goal) const
{
  lanelet::Lanelet closest_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(road_lanelets_, goal, &closest_lanelet)) {
    return false;
  }
  const auto goal_lanelet_pt = lanelet::utils::conversion::toLaneletPoint(goal.position);

  if (is_in_lane(closest_lanelet, goal_lanelet_pt)) {
    const auto lane_yaw = lanelet::utils::getLaneletAngle(closest_lanelet, goal.position);
    const auto goal_yaw = tf2::getYaw(goal.orientation);
    const auto angle_diff = tier4_autoware_utils::normalizeRadian(lane_yaw - goal_yaw);

    constexpr double th_angle = M_PI / 4;

    if (std::abs(angle_diff) < th_angle) {
      return true;
    }
  }

  // check if goal is in parking space
  const auto parking_spaces = lanelet::utils::query::getAllParkingSpaces(lanelet_map_ptr_);
  if (is_in_parking_space(parking_spaces, goal_lanelet_pt)) {
    return true;
  }

  // check if goal is in parking lot
  const auto parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr_);
  if (is_in_parking_lot(parking_lots, goal_lanelet_pt)) {
    return true;
  }

  // check if goal is in shoulder lanelet
  lanelet::Lanelet closest_shoulder_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(
        shoulder_lanelets_, goal, &closest_shoulder_lanelet)) {
    return false;
  }
  // check if goal pose is in shoulder lane
  if (is_in_lane(closest_shoulder_lanelet, goal_lanelet_pt)) {
    const auto lane_yaw = lanelet::utils::getLaneletAngle(closest_shoulder_lanelet, goal.position);
    const auto goal_yaw = tf2::getYaw(goal.orientation);
    const auto angle_diff = tier4_autoware_utils::normalizeRadian(lane_yaw - goal_yaw);

    constexpr double th_angle = M_PI / 4;
    if (std::abs(angle_diff) < th_angle) {
      return true;
    }
  }

  return false;
}

PlannerPlugin::HADMapRoute DefaultPlanner::plan(const RoutePoints & points)
{
  std::stringstream log_ss;
  for (const auto & point : points) {
    log_ss << "x: " << point.position.x << " "
           << "y: " << point.position.y << std::endl;
  }
  RCLCPP_INFO_STREAM(
    node_->get_logger(), "start planning route with checkpoints: " << std::endl
                                                                   << log_ss.str());

  autoware_auto_planning_msgs::msg::HADMapRoute route_msg;
  RouteSections route_sections;

  if (!is_goal_valid(points.back())) {
    RCLCPP_WARN(
      node_->get_logger(), "Goal is not valid! Please check position and angle of goal_pose");
    return route_msg;
  }

  for (std::size_t i = 1; i < points.size(); i++) {
    const auto start_checkpoint = points.at(i - 1);
    const auto goal_checkpoint = points.at(i);
    lanelet::ConstLanelets path_lanelets;
    if (!route_handler_.planPathLaneletsBetweenCheckpoints(
          start_checkpoint, goal_checkpoint, &path_lanelets)) {
      return route_msg;
    }
    // create local route sections
    route_handler_.setRouteLanelets(path_lanelets);
    const auto local_route_sections = route_handler_.createMapSegments(path_lanelets);
    route_sections = combine_consecutive_route_sections(route_sections, local_route_sections);
  }

  if (is_route_looped(route_sections)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Loop detected within route! Be aware that looped route is not debugged!");
  }

  const auto goal = refine_goal_height(points.back(), route_sections);
  RCLCPP_DEBUG(node_->get_logger(), "Goal Pose Z : %lf", goal.position.z);

  // The header is assigned by mission planner.
  route_msg.start_pose = points.front();
  route_msg.goal_pose = goal;
  route_msg.segments = route_sections;
  return route_msg;
}

geometry_msgs::msg::Pose DefaultPlanner::refine_goal_height(
  const Pose & goal, const RouteSections & route_sections)
{
  const auto goal_lane_id = route_sections.back().preferred_primitive_id;
  lanelet::Lanelet goal_lanelet = lanelet_map_ptr_->laneletLayer.get(goal_lane_id);
  const auto goal_lanelet_pt = lanelet::utils::conversion::toLaneletPoint(goal.position);
  double goal_height = project_goal_to_map(goal_lanelet, goal_lanelet_pt);

  Pose refined_goal = goal;
  refined_goal.position.z = goal_height;
  return refined_goal;
}

}  // namespace mission_planner::lanelet2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mission_planner::lanelet2::DefaultPlanner, mission_planner::PlannerPlugin)
