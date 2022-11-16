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

using namespace autoware_auto_perception_msgs::msg;

/**
 * struct to save the map/rois, ssd/rois and cv::Mat image of one frame.
 * Sometimes the timestamp of image and rois don't match, in this case, the data of that frame is invalid.
 * We don't bother to set a flag for every member, so just use smart pointer.
 * If the pointer is nullptr, means the data invalid
*/
struct cvMatRoi
{
void setInvalid();

bool isValid();
// use unique pointer to avoid copying data
std::unique_ptr<cv::Mat> image;
std::unique_ptr<TrafficLightRoiArray> map_rois;
std::unique_ptr<TrafficLightRoiArray> ssd_rois;
};

class TrafficLightOpticalFlowBasedDetectorNodelet : public rclcpp::Node
{
public:
  explicit TrafficLightOpticalFlowBasedDetectorNodelet(const rclcpp::NodeOptions & node_options);
private:
  bool rosMsg2CvMat(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, std::unique_ptr<cv::Mat> & image);

  inline bool headerValid(const std_msgs::msg::Header& header)
  {
    return header.stamp.sec != 0 || header.stamp.nanosec != 0;
  }
  
  TrafficLightRoi predictRoi(const TrafficLightRoi& ssd_roi, const TrafficLightRoi& map_roi);

  TrafficLightRoi predictRoi(const TrafficLightRoi& ssd_roi, const TrafficLightRoi& map_roi, const TrafficLightRoi& last_map_roi);

  void imageMapRoiCallback(const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
                           const TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg);

  void mapRoiCallback(const TrafficLightRoiArray::ConstSharedPtr input_msg);

  void lastRoiCallback(const TrafficLightRoiArray::ConstSharedPtr input_msg);

  TrafficLightRoiArray performPartialOpticalFlow(const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
                                                 const TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg);

  TrafficLightRoiArray performOpticalFlow(const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
                                          const TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg);

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, TrafficLightRoiArray> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;
  //subscribe the current frame traffic light image
  image_transport::SubscriberFilter image_sub_;
  //subscribe the current frame map cropped traffic light rois
  message_filters::Subscriber<TrafficLightRoiArray> map_roi_sub_;
  //subscribe the previous frame detected rois by ssd
  rclcpp::Subscription<TrafficLightRoiArray>::SharedPtr prev_roi_sub_;
  //publish the current frame predicted rois
  rclcpp::Publisher<TrafficLightRoiArray>::SharedPtr roi_pub_;
  //previous frame image and rois
  cvMatRoi prev_image_rois_;
  //current frame image and rois
  cvMatRoi curr_image_rois_;

  const uint8_t step_ = 1;
};
}  // namespace traffic_light
#endif  // TRAFFIC_LIGHT_OPTICAL_FLOW_BASED_DETECTOR__NODELET_HPP_
