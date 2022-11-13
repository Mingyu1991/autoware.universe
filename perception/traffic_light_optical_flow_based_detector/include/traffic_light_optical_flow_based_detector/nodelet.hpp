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
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#ifndef TRAFFIC_LIGHT_OPTICAL_FLOW_BASED_DETECTOR__NODELET_HPP_
#define TRAFFIC_LIGHT_OPTICAL_FLOW_BASED_DETECTOR__NODELET_HPP_


#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/optflow.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <set>
#include <vector>
#include <algorithm>
#include <string>
#include <unordered_map>

namespace traffic_light
{

struct cvMatMsg
{
cv::Mat image;
std_msgs::msg::Header header;
bool valid;
};

class TrafficLightOpticalFlowBasedDetectorNodelet : public rclcpp::Node
{
public:
  explicit TrafficLightOpticalFlowBasedDetectorNodelet(const rclcpp::NodeOptions & node_options);
private:
  bool rosMsg2CvMat(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, cv::Mat & image);

  inline bool headerValid(const std_msgs::msg::Header& header)
  {
    return header.stamp.sec != 0 || header.stamp.nanosec != 0;
  }
  autoware_auto_perception_msgs::msg::TrafficLightRoi predictRoi(const autoware_auto_perception_msgs::msg::TrafficLightRoi& ssd_roi,
                                                                 const autoware_auto_perception_msgs::msg::TrafficLightRoi& map_roi);
  void imageMapRoiCallback(const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
                           const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg);
  void cameraImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr input_msg);
  void mapRoiCallback(const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr input_msg);
  void lastRoiCallback(const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr input_msg);

  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, autoware_auto_perception_msgs::msg::TrafficLightRoiArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;
  //subscribe the current frame traffic light image
  image_transport::SubscriberFilter image_sub_;
  //subscribe the current frame map cropped traffic light rois
  message_filters::Subscriber<autoware_auto_perception_msgs::msg::TrafficLightRoiArray> map_roi_sub_;
  //subscribe the previous frame detected rois by ssd
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr prev_roi_sub_;
  //publish the current frame predicted rois
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr roi_pub_;
  //previous frame image
  cvMatMsg prev_image_;
  //current frame image
  cvMatMsg curr_image_;
  //previous frame detected rois by ssd
  std::shared_ptr<autoware_auto_perception_msgs::msg::TrafficLightRoiArray> last_roi_;

  const uint8_t step_ = 1;
};
}  // namespace traffic_light
#endif  // TRAFFIC_LIGHT_OPTICAL_FLOW_BASED_DETECTOR__NODELET_HPP_
