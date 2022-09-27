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

#include "static_path_smoother/functions.hpp"

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/utilities.hpp"
#include "map_loader/lanelet2_map_loader_node.hpp"
#include "mission_planner/mission_planner_lanelet2.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

namespace static_path_smoother
{
namespace
{
rclcpp::NodeOptions create_node_options() { return rclcpp::NodeOptions{}; }

geometry_msgs::msg::PoseStamped::ConstSharedPtr convert_to_pose_stamped(
  const geometry_msgs::msg::Pose & pose)
{
  auto pose_stamped_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose_stamped_ptr->pose = pose;
  return pose_stamped_ptr;
}

geometry_msgs::msg::Pose get_center_pose(
  const route_handler::RouteHandler & route_handler, const size_t lanelet_id)
{
  // get middle idx of the lanelet
  const auto lanelet = route_handler.getLaneletsFromId(lanelet_id);
  const auto center_line = lanelet.centerline();
  const size_t middle_point_idx = std::floor(center_line.size() / 2.0);

  // get middle position of the lanelet
  geometry_msgs::msg::Point middle_pos;
  middle_pos.x = center_line[middle_point_idx].x();
  middle_pos.y = center_line[middle_point_idx].y();

  // get next middle position of the lanelet
  geometry_msgs::msg::Point next_middle_pos;
  next_middle_pos.x = center_line[middle_point_idx + 1].x();
  next_middle_pos.y = center_line[middle_point_idx + 1].y();

  // calculate middle pose
  geometry_msgs::msg::Pose middle_pose;
  middle_pose.position = middle_pos;
  const double yaw = tier4_autoware_utils::calcAzimuthAngle(middle_pos, next_middle_pos);
  middle_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);

  return middle_pose;
}

lanelet::Point3d createPoint3d(const double x, const double y, const double z = 19.0)
{
  lanelet::Point3d point(lanelet::utils::getId());
  point.setAttribute("local_x", x);
  point.setAttribute("local_y", y);
  point.setAttribute("ele", z);

  return point;
}
}  // namespace

HADMapBin::ConstSharedPtr create_map(
  const std::string & lanelet2_file_name, const rclcpp::Time & current_time)
{
  // load map
  lanelet::LaneletMapPtr map_ptr;
  map_ptr = Lanelet2MapLoaderNode::load_map(lanelet2_file_name, "MGRS");
  if (!map_ptr) {
    return nullptr;
  }

  // create map bin msg
  const auto map_bin_msg =
    Lanelet2MapLoaderNode::create_map_bin_msg(map_ptr, lanelet2_file_name, current_time);

  return std::make_shared<HADMapBin>(map_bin_msg);
}

std::vector<geometry_msgs::msg::Pose> create_check_points(
  const route_handler::RouteHandler & route_handler, const size_t start_lanelet_id,
  const size_t end_lanelet_id)
{
  const auto start_pose = get_center_pose(route_handler, start_lanelet_id);
  const auto end_pose = get_center_pose(route_handler, end_lanelet_id);

  return std::vector<geometry_msgs::msg::Pose>{start_pose, end_pose};
}

HADMapRoute plan_route(
  const HADMapBin::ConstSharedPtr map_bin_msg_ptr,
  const std::vector<geometry_msgs::msg::Pose> & check_points)
{
  auto mission_planner_node = mission_planner::MissionPlannerLanelet2(create_node_options());

  mission_planner_node.map_callback(map_bin_msg_ptr);
  const auto route = mission_planner_node.plan_route(check_points);

  return route;
}

PathWithLaneId get_path_with_lane_id(
  const route_handler::RouteHandler & route_handler, const lanelet::ConstLanelets lanelets,
  const geometry_msgs::msg::Pose & start_pose, const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold)
{
  // get center line
  constexpr double s_start = 0.0;
  constexpr double s_end = std::numeric_limits<double>::max();
  auto path_with_lane_id = route_handler.getCenterLinePath(lanelets, s_start, s_end);
  path_with_lane_id.header.frame_id = "map";

  // create planner data
  auto planner_data = std::make_shared<behavior_path_planner::PlannerData>();
  planner_data->route_handler = std::make_shared<route_handler::RouteHandler>(route_handler);
  planner_data->self_pose = convert_to_pose_stamped(start_pose);
  planner_data->parameters.drivable_lane_forward_length = std::numeric_limits<double>::max();
  planner_data->parameters.drivable_lane_backward_length = std::numeric_limits<double>::min();
  planner_data->parameters.drivable_lane_margin = 5.0;
  planner_data->parameters.ego_nearest_dist_threshold = ego_nearest_dist_threshold;
  planner_data->parameters.ego_nearest_yaw_threshold = ego_nearest_yaw_threshold;

  // generate drivable area and store it in path with lane id
  constexpr double drivable_area_resolution = 0.1;
  constexpr double vehicle_length = 0.0;
  path_with_lane_id.drivable_area = behavior_path_planner::util::generateDrivableArea(
    path_with_lane_id, lanelets, drivable_area_resolution, vehicle_length, planner_data);

  return path_with_lane_id;
}

void update_centerline(
  route_handler::RouteHandler & route_handler, const lanelet::ConstLanelets & lanelets,
  const std::vector<TrajectoryPoint> & new_centerline)
{
  // get lanelet as reference to update centerline
  lanelet::Lanelets lanelets_ref;
  for (const auto & lanelet : lanelets) {
    for (auto & lanelet_ref : route_handler.getLaneletMapPtr()->laneletLayer) {
      if (lanelet_ref.id() == lanelet.id()) {
        lanelets_ref.push_back(lanelet_ref);
      }
    }
  }

  // store new centerline in lanelets
  size_t lanelet_idx = 0;
  lanelet::LineString3d centerline(lanelet::utils::getId());
  for (size_t traj_idx = 0; traj_idx < new_centerline.size(); ++traj_idx) {
    const auto & traj_pos = new_centerline.at(traj_idx).pose.position;

    for (; lanelet_idx < lanelets_ref.size(); ++lanelet_idx) {
      auto & lanelet_ref = lanelets_ref.at(lanelet_idx);

      const lanelet::BasicPoint2d point(traj_pos.x, traj_pos.y);
      const bool is_inside = lanelet::geometry::inside(lanelet_ref, point);
      if (is_inside) {
        const auto center_point = createPoint3d(traj_pos.x, traj_pos.y, traj_pos.z);

        // set center point
        centerline.push_back(center_point);
        route_handler.getLaneletMapPtr()->add(center_point);
        break;
      }

      if (!centerline.empty()) {
        // set centerline
        route_handler.getLaneletMapPtr()->add(centerline);
        lanelet_ref.setCenterline(centerline);

        // prepare new centerline
        centerline = lanelet::LineString3d(lanelet::utils::getId());
      }
    }

    if (traj_idx == new_centerline.size() - 1 && !centerline.empty()) {
      auto & lanelet_ref = lanelets_ref.at(lanelet_idx);

      // set centerline
      route_handler.getLaneletMapPtr()->add(centerline);
      lanelet_ref.setCenterline(centerline);
    }
  }
}
}  // namespace static_path_smoother
