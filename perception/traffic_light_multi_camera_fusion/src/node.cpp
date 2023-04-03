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

}  // namespace

namespace traffic_light
{

MultiCameraFusion::MultiCameraFusion(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_multi_camera_fusion", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  std::vector<std::string> camera_namespaces =
    this->declare_parameter("camera_namespaces", std::vector<std::string>{});
  for (const std::string & camera_ns : camera_namespaces) {
    std::string signal_topic = camera_ns + "/traffic_signals";
    std::string roi_topic = camera_ns + "/rois";
    roi_subs_.emplace_back(
      new mf::Subscriber<RoiArrayType>(this, roi_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    signal_subs_.emplace_back(new mf::Subscriber<SignalArrayType>(
      this, signal_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    sync_subs_.emplace_back(
      new ExactSync(ExactSyncPolicy(10), *(roi_subs_.back()), *(signal_subs_.back())));
    sync_subs_.back()->registerCallback(
      std::bind(&MultiCameraFusion::trafficSignalRoiCallback, this, _1, _2));
  }

  signal_pub_ = create_publisher<SignalArrayType>("~/output/traffic_signals", rclcpp::QoS{1});
}

void MultiCameraFusion::trafficSignalRoiCallback(
  const RoiArrayType::ConstSharedPtr roi_msg, const SignalArrayType::ConstSharedPtr signal_msg)
{
  rclcpp::Time stamp(roi_msg->header.stamp);
  /*
  Insert the received record array to the table.
  Attention should be paied that this record array might not have the newest timestamp
  */
  record_arr_map_.insert(std::make_pair(stamp, std::make_pair(*roi_msg, *signal_msg)));

  std::map<IdType, RecordType> fusionedRecordMap;
  multiCameraFusion(fusionedRecordMap);
  groupFusion(fusionedRecordMap);

  SignalArrayType out_msg;
  out_msg.header = signal_msg->header;
  out_msg.signals.reserve(fusionedRecordMap.size());
  for (const auto & p : fusionedRecordMap) {
    out_msg.signals.emplace_back(p.second.second);
  }
  signal_pub_->publish(out_msg);
}

int MultiCameraFusion::compareRecord(const RecordType & r1, const RecordType & r2) const
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
  /*
  ignore the small score difference by dividing by 10,
  so that the area score could be more important
  */
  int visible_score_1 = r1.first.visible_ratio / 10;
  int visible_score_2 = r2.first.visible_ratio / 10;
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

void MultiCameraFusion::multiCameraFusion(std::map<IdType, RecordType> & fusionedRecordMap)
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
          compareRecord(record, fusionedRecordMap[tl_id]) >= 0) {
          fusionedRecordMap[tl_id] = record;
        }
      }
      it++;
    }
  }
}

void MultiCameraFusion::groupFusion(std::map<IdType, RecordType> & fusionedRecordMap)
{
  /*
  the best record for every regulatory element id
  */
  std::map<IdType, RecordType> regEleId2BestRecord;
  for (auto & p : fusionedRecordMap) {
    IdType reg_ele_id = p.second.first.regulatory_element_id;
    if (
      regEleId2BestRecord.count(reg_ele_id) == 0 ||
      compareRecord(p.second, regEleId2BestRecord[reg_ele_id]) >= 0) {
      regEleId2BestRecord[reg_ele_id] = p.second;
    }
  }
  for (auto & p : fusionedRecordMap) {
    IdType reg_ele_id = p.second.first.regulatory_element_id;
    p.second.second.lights = regEleId2BestRecord[reg_ele_id].second.lights;
  }
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::MultiCameraFusion)
