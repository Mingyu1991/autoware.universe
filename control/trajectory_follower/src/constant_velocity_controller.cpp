// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "trajectory_follower/constant_velocity_controller.hpp"

#include "motion_common/motion_common.hpp"
#include "motion_common/trajectory_common.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "time_utils/time_utils.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
ConstantVelocityController::ConstantVelocityController(rclcpp::Node & node) : node_{&node}
{
  using std::placeholders::_1;

  // parameters timer
  m_longitudinal_ctrl_period = node_->get_parameter("ctrl_period").as_double();

  // subscriber, publisher
  m_pub_slope =
    node_->create_publisher<autoware_auto_system_msgs::msg::Float32MultiArrayDiagnostic>(
      "~/output/slope_angle", rclcpp::QoS{1});
  m_pub_debug =
    node_->create_publisher<autoware_auto_system_msgs::msg::Float32MultiArrayDiagnostic>(
      "~/output/longitudinal_diagnostic", rclcpp::QoS{1});

}

boost::optional<LongitudinalOutput> ConstantVelocityController::run(){

  autoware_auto_control_msgs::msg::LongitudinalCommand cmd{};
  {
    cmd.stamp = node_->now();
    cmd.speed = 5.0;
    cmd.acceleration = 1.0;
  }
  LongitudinalOutput output;
  {
    output.control_cmd = cmd;
  }
  return output;
}

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
