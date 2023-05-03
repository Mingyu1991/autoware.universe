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

namespace
{

bool isUnknown(const autoware_auto_perception_msgs::msg::TrafficSignal & signal)
{
  return signal.lights.size() == 1 &&
         signal.lights[0].color == autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN &&
         signal.lights[0].shape == autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN;
}

int calVisibleScore(
  const traffic_light::MultiCameraFusion::RoiType & roi,
  const traffic_light::MultiCameraFusion::CamInfoType & cam_info)
{
  uint32_t boundary = 5;
  uint32_t x1 = roi.roi.x_offset;
  uint32_t x2 = roi.roi.x_offset + roi.roi.width;
  uint32_t y1 = roi.roi.y_offset;
  uint32_t y2 = roi.roi.y_offset + roi.roi.height;
  if (
    x1 <= boundary || (cam_info.width - x2) <= boundary || y1 <= boundary ||
    (cam_info.height - y2) <= boundary) {
    return 0;
  } else {
    return 1;
  }
}

int compareRecord(
  const traffic_light::MultiCameraFusion::RecordType & r1,
  const traffic_light::MultiCameraFusion::RecordType & r2,
  const traffic_light::MultiCameraFusion::CamInfoType & cam_info)
{
  bool r1_is_unknown = isUnknown(r1.second);
  bool r2_is_unknown = isUnknown(r2.second);
  // if both are unknown, they are of the same priority
  if (r1_is_unknown && r2_is_unknown) {
    return 0;
  } else if (r1_is_unknown ^ r2_is_unknown) {
    // if either is unknown, the unknown is of lower priority
    return r1_is_unknown ? -1 : 1;
  }
  int visible_score_1 = calVisibleScore(r1.first, cam_info);
  int visible_score_2 = calVisibleScore(r2.first, cam_info);
  if (visible_score_1 == visible_score_2) {
    int area_1 = r1.first.roi.width * r1.first.roi.height;
    int area_2 = r2.first.roi.width * r2.first.roi.height;
    if (area_1 < area_2) {
      return -1;
    } else {
      return static_cast<int>(area_1 > area_2);
    }
  } else {
    return visible_score_1 < visible_score_2 ? -1 : 1;
  }
}

}  // namespace

namespace traffic_light
{

MultiCameraFusion::MultiCameraFusion(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_multi_camera_fusion", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  std::vector<std::string> camera_namespaces =
    this->declare_parameter("camera_namespaces", std::vector<std::string>{});
  is_approximate_sync_ = this->declare_parameter<bool>("approximate_sync", false);
  RCLCPP_INFO_STREAM(get_logger(), "is_approximate_sync_ = " << is_approximate_sync_);
  for (const std::string & camera_ns : camera_namespaces) {
    std::string signal_topic = camera_ns + "/traffic_signals";
    std::string roi_topic = camera_ns + "/rois";
    std::string cam_info_topic = camera_ns + "/camera_info";
    roi_subs_.emplace_back(
      new mf::Subscriber<RoiArrayType>(this, roi_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    signal_subs_.emplace_back(new mf::Subscriber<SignalArrayType>(
      this, signal_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    cam_info_subs_.emplace_back(
      new mf::Subscriber<CamInfoType>(this, cam_info_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    if (is_approximate_sync_ == false) {
      exact_sync_subs_.emplace_back(new ExactSync(
        ExactSyncPolicy(10), *(cam_info_subs_.back()), *(roi_subs_.back()),
        *(signal_subs_.back())));
      exact_sync_subs_.back()->registerCallback(
        std::bind(&MultiCameraFusion::trafficSignalRoiCallback, this, _1, _2, _3));
    } else {
      appro_sync_subs_.emplace_back(new ApproSync(
        ApproSyncPolicy(10), *(cam_info_subs_.back()), *(roi_subs_.back()),
        *(signal_subs_.back())));
      appro_sync_subs_.back()->registerCallback(
        std::bind(&MultiCameraFusion::trafficSignalRoiCallback, this, _1, _2, _3));
    }
  }

  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MultiCameraFusion::mapCallback, this, _1));
  signal_pub_ = create_publisher<SignalArrayType>("~/output/traffic_signals", rclcpp::QoS{1});
}

void MultiCameraFusion::trafficSignalRoiCallback(
  const CamInfoType::ConstSharedPtr cam_info_msg, const RoiArrayType::ConstSharedPtr roi_msg,
  const SignalArrayType::ConstSharedPtr signal_msg)
{
  rclcpp::Time stamp(roi_msg->header.stamp);
  /*
  Insert the received record array to the table.
  Attention should be paied that this record array might not have the newest timestamp
  */
  record_arr_map_.insert(std::make_pair(stamp, std::make_pair(*roi_msg, *signal_msg)));

  std::map<IdType, RecordType> fusionedRecordMap;
  multiCameraFusion(*cam_info_msg, fusionedRecordMap);
  groupFusion(*cam_info_msg, fusionedRecordMap);

  SignalArrayType out_msg;
  out_msg.header = signal_msg->header;
  out_msg.signals.reserve(fusionedRecordMap.size());
  for (const auto & p : fusionedRecordMap) {
    out_msg.signals.emplace_back(p.second.second);
  }
  signal_pub_->publish(out_msg);
}

void MultiCameraFusion::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_msg)
{
  lanelet::LaneletMapPtr lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();

  lanelet::utils::conversion::fromBinMsg(*input_msg, lanelet_map_ptr);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr);
  std::vector<lanelet::AutowareTrafficLightConstPtr> all_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (auto tl_itr = all_lanelet_traffic_lights.begin(); tl_itr != all_lanelet_traffic_lights.end();
       ++tl_itr) {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (const auto & light : lights) {
      trafficLightId2RegulatoryEleId_[light.id()] = tl->id();
    }
  }
}

void MultiCameraFusion::multiCameraFusion(
  const CamInfoType cam_info, std::map<IdType, RecordType> & fusionedRecordMap)
{
  double max_keep_t = 0.10;
  fusionedRecordMap.clear();

  const rclcpp::Time & newest_stamp = record_arr_map_.rbegin()->first;
  for (auto it = record_arr_map_.begin(); it != record_arr_map_.end();) {
    /*
    remove all old record arrays whose timestamp difference with newest record is larger than
    threshold
    */
    if ((newest_stamp - it->first) > rclcpp::Duration::from_seconds(max_keep_t)) {
      it = record_arr_map_.erase(it);
    } else {
      /*
      generate fusioned record result with the saved records
      */
      const RecordArrayType & record_arr = it->second;
      assert(record_arr.first.rois.size() == record_arr.second.signals.size());
      for (size_t i = 0; i < record_arr.first.rois.size(); i++) {
        const RoiType & roi = record_arr.first.rois[i];
        const SignalType & signal = record_arr.second.signals[i];
        assert(roi.id == signal.map_primitive_id);
        IdType tl_id = roi.id;

        RecordType record(roi, signal);
        /*
        if this traffic light is not detected yet or can be updated by higher priority record,
        update it
        */
        if (
          fusionedRecordMap.find(tl_id) == fusionedRecordMap.end() ||
          ::compareRecord(record, fusionedRecordMap[tl_id], cam_info) >= 0) {
          fusionedRecordMap[tl_id] = record;
        }
      }
      it++;
    }
  }
}

void MultiCameraFusion::groupFusion(
  const CamInfoType cam_info, std::map<IdType, RecordType> & fusionedRecordMap)
{
  /*
  the best record for every regulatory element id
  */
  std::map<IdType, RecordType> regEleId2BestRecord;
  for (auto & p : fusionedRecordMap) {
    IdType reg_ele_id = trafficLightId2RegulatoryEleId_[p.second.first.id];
    if (
      regEleId2BestRecord.count(reg_ele_id) == 0 ||
      ::compareRecord(p.second, regEleId2BestRecord[reg_ele_id], cam_info) >= 0) {
      regEleId2BestRecord[reg_ele_id] = p.second;
    }
  }
  for (auto & p : fusionedRecordMap) {
    IdType reg_ele_id = trafficLightId2RegulatoryEleId_[p.second.first.id];
    ;
    p.second.second.lights = regEleId2BestRecord[reg_ele_id].second.lights;
  }
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::MultiCameraFusion)
