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
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
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

/**
 * @brief Sometimes because of deviation from map, localization or sensor calibration,
 * the traffic lights are not within the rois estimated by "traffic_light_map_based_detector" nodelet.
 * This nodelet is to optimize the traffic light roi positions so that the traffic lights are close to
 * the center of the rois sent to the ssd detector by using optical flow to track the traffic lights 
 * detected from last frame. 
 * 
 * 
 * ---------------------------------- /map/rois --------------/rought/rois --------------------------------
 * |traffic_light_map_based_detector| --------->|this nodelet|------------>|traffic_light_ssd_fine_detector|
 * ----------------------------------           --------------             --------------------------------
 *                                                     ^       /rois                        |
 *                                                     |____________________________________|
 */
class TrafficLightOpticalFlowBasedDetectorNodelet : public rclcpp::Node
{
public:
  explicit TrafficLightOpticalFlowBasedDetectorNodelet(const rclcpp::NodeOptions & node_options);
private:
/**
 * @brief convert ros image message to cv::Mat
 * 
 * @param image_msg input ros message
 * @param image output cv::Mat image
 * @return true conversion succeed
 * @return false conversion failed
 */
  bool rosMsg2CvMat(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, std::unique_ptr<cv::Mat> & image);
/**
 * @brief receive current frame image and map/rois with same timestamps,
 * perform optical flow and output rois for ssd detection
 * 
 * @param in_image_msg image message
 * @param in_map_roi_msg map/rois message
 */
  void imageMapRoiCallback(const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
                           const TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg);

/**
 * @brief receive ssd bounding boxes(considered as rois) of current frame for tracking the traffic light in the next frame 
 * 
 * @param input_msg ssd rois message
 */
  void ssdRoiCallback(const TrafficLightRoiArray::ConstSharedPtr input_msg);
/**
 * @brief for every map/roi, check if the traffic light was detected by ssd in previous frame.
 * If no, just publish this map/roi.
 * For all the map/roi whose corresponding traffic lights were detected by ssd in previous frame,
 * crop the image containing all of them from current image and previous image, perform sparse 
 * optical flow tracking only the points within the ssd rois to save time.
 * For every ssd/roi from previous frame, find their corresponding points in current frame and calculate
 * the center position (cx, cy), and publish the new roi centered at (cx, cy)
 * @param in_map_roi_msg 
 * @return TrafficLightRoiArray 
 */
  TrafficLightRoiArray performOpticalFlow(const TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg);

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
};
}  // namespace traffic_light
#endif  // TRAFFIC_LIGHT_OPTICAL_FLOW_BASED_DETECTOR__NODELET_HPP_
