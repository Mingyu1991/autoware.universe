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

#ifndef TRAFFIC_LIGHT_MULTI_CAMERA_FUSION__NODE_HPP_
#define TRAFFIC_LIGHT_MULTI_CAMERA_FUSION__NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <memory>
#include <set>
#include <vector>

namespace traffic_light
{

namespace mf = message_filters;

class MultiCameraFusion : public rclcpp::Node
{
  typedef autoware_auto_perception_msgs::msg::TrafficSignalArray SignalType;
  typedef autoware_auto_perception_msgs::msg::TrafficLightRoiArray RoiType;

public:
  explicit MultiCameraFusion(const rclcpp::NodeOptions & node_options);

private:
  void trafficSignalRoiCallback(
    const RoiType::ConstSharedPtr roi_msg, const SignalType::ConstSharedPtr signal_msg);

  typedef mf::sync_policies::ExactTime<RoiType, SignalType> ExactSyncPolicy;
  typedef mf::Synchronizer<ExactSyncPolicy> ExactSync;

  typedef mf::sync_policies::ApproximateTime<RoiType, SignalType> ApproSyncPolicy;
  typedef mf::Synchronizer<ApproSyncPolicy> ApproSync;

  std::vector<std::unique_ptr<mf::Subscriber<SignalType>>> signal_subs_;
  std::vector<std::unique_ptr<mf::Subscriber<RoiType>>> roi_subs_;
  std::vector<std::unique_ptr<ExactSync>> exact_sync_subs_;
  std::vector<std::unique_ptr<ApproSync>> appro_sync_subs_;
  bool is_approxiate_sync_;
  double last_msg_stamp_;
};
}  // namespace traffic_light
#endif  // TRAFFIC_LIGHT_MULTI_CAMERA_FUSION__NODE_HPP_
