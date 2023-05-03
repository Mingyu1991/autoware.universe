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

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <list>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace traffic_light
{

namespace mf = message_filters;

class MultiCameraFusion : public rclcpp::Node
{
public:
  typedef sensor_msgs::msg::CameraInfo CamInfoType;
  typedef autoware_auto_perception_msgs::msg::TrafficLightRoi RoiType;
  typedef autoware_auto_perception_msgs::msg::TrafficSignal SignalType;
  typedef autoware_auto_perception_msgs::msg::TrafficSignalArray SignalArrayType;
  typedef autoware_auto_perception_msgs::msg::TrafficLightRoiArray RoiArrayType;
  typedef autoware_auto_perception_msgs::msg::TrafficLightRoi::_id_type IdType;
  typedef std::pair<RoiArrayType, SignalArrayType> RecordArrayType;
  typedef std::pair<RoiType, SignalType> RecordType;

  explicit MultiCameraFusion(const rclcpp::NodeOptions & node_options);

private:
  void trafficSignalRoiCallback(
    const CamInfoType::ConstSharedPtr cam_info_msg, const RoiArrayType::ConstSharedPtr roi_msg,
    const SignalArrayType::ConstSharedPtr signal_msg);

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_msg);

  void multiCameraFusion(
    const CamInfoType cam_info, std::map<IdType, RecordType> & fusionedRecordMap);

  void groupFusion(const CamInfoType cam_info, std::map<IdType, RecordType> & fusionedRecordMap);

  typedef mf::sync_policies::ExactTime<CamInfoType, RoiArrayType, SignalArrayType> ExactSyncPolicy;
  typedef mf::Synchronizer<ExactSyncPolicy> ExactSync;
  typedef mf::sync_policies::ApproximateTime<CamInfoType, RoiArrayType, SignalArrayType>
    ApproSyncPolicy;
  typedef mf::Synchronizer<ApproSyncPolicy> ApproSync;

  std::vector<std::unique_ptr<mf::Subscriber<SignalArrayType>>> signal_subs_;
  std::vector<std::unique_ptr<mf::Subscriber<RoiArrayType>>> roi_subs_;
  std::vector<std::unique_ptr<mf::Subscriber<CamInfoType>>> cam_info_subs_;
  std::vector<std::unique_ptr<ExactSync>> exact_sync_subs_;
  std::vector<std::unique_ptr<ApproSync>> appro_sync_subs_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;

  rclcpp::Publisher<SignalArrayType>::SharedPtr signal_pub_;

  std::map<lanelet::Id, lanelet::Id> trafficLightId2RegulatoryEleId_;
  /*
  save record arrays by increasing timestamp order.
  use multimap in case there are multiple cameras publishing images at exactly the same time
  */
  std::multimap<rclcpp::Time, RecordArrayType> record_arr_map_;
  bool is_approximate_sync_;
};
}  // namespace traffic_light
#endif  // TRAFFIC_LIGHT_MULTI_CAMERA_FUSION__NODE_HPP_
