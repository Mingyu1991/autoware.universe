// Copyright 2023 TIER IV, Inc.
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

#ifndef TRAFFIC_LIGHT_MULTI_CAMERA_FUSION__NODE_HPP_
#define TRAFFIC_LIGHT_MULTI_CAMERA_FUSION__NODE_HPP_

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <autoware_perception_msgs/msg/traffic_light.hpp>
#include <autoware_perception_msgs/msg/traffic_light_array.hpp>
#include <autoware_perception_msgs/msg/traffic_signal.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
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

struct FusionRecord
{
  std_msgs::msg::Header header;
  sensor_msgs::msg::CameraInfo cam_info;
  autoware_auto_perception_msgs::msg::TrafficLightRoi roi;
  autoware_perception_msgs::msg::TrafficLight light;
};

struct FusionRecordArr
{
  std_msgs::msg::Header header;
  sensor_msgs::msg::CameraInfo cam_info;
  autoware_auto_perception_msgs::msg::TrafficLightRoiArray rois;
  autoware_perception_msgs::msg::TrafficLightArray lights;
};

bool operator<(const FusionRecordArr & r1, const FusionRecordArr & r2)
{
  return rclcpp::Time(r1.header.stamp) < rclcpp::Time(r2.header.stamp);
}

class MultiCameraFusion : public rclcpp::Node
{
public:
  typedef sensor_msgs::msg::CameraInfo CamInfoType;
  typedef autoware_auto_perception_msgs::msg::TrafficLightRoi RoiType;
  typedef autoware_perception_msgs::msg::TrafficSignal SignalType;
  typedef autoware_perception_msgs::msg::TrafficSignalArray SignalArrayType;
  typedef autoware_perception_msgs::msg::TrafficLight LightType;
  typedef autoware_perception_msgs::msg::TrafficLightArray LightArrayType;
  typedef autoware_auto_perception_msgs::msg::TrafficLightRoiArray RoiArrayType;
  typedef autoware_auto_perception_msgs::msg::TrafficLightRoi::_id_type IdType;

  typedef std::pair<RoiArrayType, SignalArrayType> RecordArrayType;

  explicit MultiCameraFusion(const rclcpp::NodeOptions & node_options);

private:
  void trafficSignalRoiCallback(
    const CamInfoType::ConstSharedPtr cam_info_msg, const RoiArrayType::ConstSharedPtr roi_msg,
    const LightArrayType::ConstSharedPtr signal_msg);

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_msg);

  void multiCameraFusion(std::map<IdType, FusionRecord> & fusioned_record_map);

  void groupFusion(std::map<IdType, FusionRecord> & fusioned_record_map, SignalArrayType & out_msg);

  typedef mf::sync_policies::ExactTime<CamInfoType, RoiArrayType, LightArrayType> ExactSyncPolicy;
  typedef mf::Synchronizer<ExactSyncPolicy> ExactSync;
  typedef mf::sync_policies::ApproximateTime<CamInfoType, RoiArrayType, LightArrayType>
    ApproSyncPolicy;
  typedef mf::Synchronizer<ApproSyncPolicy> ApproSync;

  std::vector<std::unique_ptr<mf::Subscriber<LightArrayType>>> light_subs_;
  std::vector<std::unique_ptr<mf::Subscriber<RoiArrayType>>> roi_subs_;
  std::vector<std::unique_ptr<mf::Subscriber<CamInfoType>>> cam_info_subs_;
  std::vector<std::unique_ptr<ExactSync>> exact_sync_subs_;
  std::vector<std::unique_ptr<ApproSync>> appro_sync_subs_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;

  rclcpp::Publisher<SignalArrayType>::SharedPtr signal_pub_;
  /*
  the mappping from traffic light id (instance id) to regulatory element id (group id)
  */
  std::map<lanelet::Id, lanelet::Id> traffic_light_id_to_regulatory_ele_id_;
  /*
  save record arrays by increasing timestamp order.
  use multiset in case there are multiple cameras publishing images at exactly the same time
  */
  std::multiset<FusionRecordArr> record_arr_set_;
  bool is_approximate_sync_;
  /*
  for every input message input_m, if the timestamp difference between input_m and the latest
  message is smaller than message_lifespan_, then input_m would be used for the fusion. Otherwise,
  it would be discarded
  */
  double message_lifespan_;
};
}  // namespace traffic_light
#endif  // TRAFFIC_LIGHT_MULTI_CAMERA_FUSION__NODE_HPP_
