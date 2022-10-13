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

#ifndef STATIC_CENTERLINE_OPTIMIZER__UTILS1_HPP_
#define STATIC_CENTERLINE_OPTIMIZER__UTILS1_HPP_

#include "rclcpp/rclcpp.hpp"
#include "static_centerline_optimizer/type_alias.hpp"

namespace static_centerline_optimizer
{
class StaticCenterlineOptimizerNode : public rclcpp::Node
{
public:
  StaticCenterlineOptimizerNode(const rclcpp::NodeOptions & node_options);
  void run();

private:
  void load_map();
  void plan_path();
  void save_map();

  // ros parameters
  std::string lanelet2_input_file_name_;
  std::string lanelet2_output_file_name_;
  int start_lanelet_id_;
  int end_lanelet_id_;

  std::shared_ptr<RouteHandler> route_handler_ptr_{nullptr};
  HADMapRoute::ConstSharedPtr route_ptr_{nullptr};
  std::vector<TrajectoryPoint> optimized_traj_points_{};

  // publisher
  rclcpp::Publisher<HADMapBin>::SharedPtr pub_map_bin_{nullptr};
  rclcpp::Publisher<PathWithLaneId>::SharedPtr pub_raw_path_with_lane_id_{nullptr};
  rclcpp::Publisher<Path>::SharedPtr pub_raw_path_{nullptr};
};
}
#endif  // STATIC_CENTERLINE_OPTIMIZER__UTILS1_HPP_
