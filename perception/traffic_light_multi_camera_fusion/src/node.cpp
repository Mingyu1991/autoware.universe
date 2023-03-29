/*
 * Copyright 2023 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Mingyu Li
 *
 */

#include "traffic_light_multi_camera_fusion/node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace traffic_light
{
MultiCameraFusion::MultiCameraFusion(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_multi_camera_fusion", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  is_approxiate_sync_ = declare_parameter("approximate_sync", false);
  std::vector<std::string> camera_namespaces =
    this->declare_parameter("camera_namespaces", std::vector<std::string>{});
  for (const std::string & camera_ns : camera_namespaces) {
    std::string signal_topic = camera_ns + "/traffic_signals";
    std::string roi_topic = camera_ns + "/rois";
    roi_subs_.emplace_back(
      new mf::Subscriber<RoiType>(this, roi_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    signal_subs_.emplace_back(
      new mf::Subscriber<SignalType>(this, signal_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));

    if (is_approxiate_sync_) {
      appro_sync_subs_.emplace_back(
        new ApproSync(ApproSyncPolicy(10), *(roi_subs_.back()), *(signal_subs_.back())));
      appro_sync_subs_.back()->registerCallback(
        std::bind(&MultiCameraFusion::trafficSignalRoiCallback, this, _1, _2));
    } else {
      exact_sync_subs_.emplace_back(
        new ExactSync(ExactSyncPolicy(10), *(roi_subs_.back()), *(signal_subs_.back())));
      exact_sync_subs_.back()->registerCallback(
        std::bind(&MultiCameraFusion::trafficSignalRoiCallback, this, _1, _2));
    }
  }
}

void MultiCameraFusion::trafficSignalRoiCallback(
  const RoiType::ConstSharedPtr roi_msg, const SignalType::ConstSharedPtr signal_msg)
{
  RCLCPP_INFO_STREAM(
    get_logger(), "roi header = " << roi_msg->header.frame_id
                                  << ", signal header = " << signal_msg->header.frame_id);
  double stampDiff = rclcpp::Time(roi_msg->header.stamp).seconds() -
                     rclcpp::Time(signal_msg->header.stamp).seconds();
  RCLCPP_INFO_STREAM(get_logger(), "stamp diff = " << stampDiff);
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::MultiCameraFusion)
