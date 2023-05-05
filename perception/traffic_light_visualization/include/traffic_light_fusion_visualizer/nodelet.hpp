// Copyright 2023 Tier IV, Inc.
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
#ifndef TRAFFIC_LIGHT_FUSION_VISUALIZER__NODELET_HPP_
#define TRAFFIC_LIGHT_FUSION_VISUALIZER__NODELET_HPP_

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/imgproc/imgproc_c.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace traffic_light
{
struct ClassificationResult
{
  float prob = 0.0;
  std::string label;
};

class TrafficLightFusionVisualizerNodelet : public rclcpp::Node
{
public:
  explicit TrafficLightFusionVisualizerNodelet(const rclcpp::NodeOptions & options);
  void connectCb();

  void imageRoughRoiCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
    const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr &
      input_tl_rough_roi_msg,
    const autoware_auto_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr &
      input_signal_msg);

private:
  std::map<int, std::string> state2label_{
    // color
    {autoware_auto_perception_msgs::msg::TrafficLight::RED, "red"},
    {autoware_auto_perception_msgs::msg::TrafficLight::AMBER, "yellow"},
    {autoware_auto_perception_msgs::msg::TrafficLight::GREEN, "green"},
    {autoware_auto_perception_msgs::msg::TrafficLight::WHITE, "white"},
    // shape
    {autoware_auto_perception_msgs::msg::TrafficLight::CIRCLE, "circle"},
    {autoware_auto_perception_msgs::msg::TrafficLight::LEFT_ARROW, "left"},
    {autoware_auto_perception_msgs::msg::TrafficLight::RIGHT_ARROW, "right"},
    {autoware_auto_perception_msgs::msg::TrafficLight::UP_ARROW, "straight"},
    {autoware_auto_perception_msgs::msg::TrafficLight::UP_LEFT_ARROW, "up_left"},
    {autoware_auto_perception_msgs::msg::TrafficLight::UP_RIGHT_ARROW, "up_right"},
    {autoware_auto_perception_msgs::msg::TrafficLight::DOWN_ARROW, "down"},
    {autoware_auto_perception_msgs::msg::TrafficLight::DOWN_LEFT_ARROW, "down_left"},
    {autoware_auto_perception_msgs::msg::TrafficLight::DOWN_RIGHT_ARROW, "down_right"},
    {autoware_auto_perception_msgs::msg::TrafficLight::CROSS, "cross"},
    // other
    {autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN, "unknown"},
  };

  bool createRect(
    cv::Mat & image, const autoware_auto_perception_msgs::msg::TrafficLightRoi & tl_roi,
    const cv::Scalar & color);

  bool createRect(
    cv::Mat & image, const autoware_auto_perception_msgs::msg::TrafficLightRoi & tl_roi,
    const ClassificationResult & result);

  bool getClassificationResult(
    int id, const autoware_auto_perception_msgs::msg::TrafficSignalArray & traffic_signals,
    ClassificationResult & result);

  bool trafficSignalChanged(
    const autoware_auto_perception_msgs::msg::TrafficSignalArray & traffic_signals);

  void printSignal(const autoware_auto_perception_msgs::msg::TrafficSignalArray & msg);

  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>
    rough_roi_sub_;
  message_filters::Subscriber<autoware_auto_perception_msgs::msg::TrafficSignalArray> signal_sub_;

  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, autoware_auto_perception_msgs::msg::TrafficLightRoiArray,
    autoware_auto_perception_msgs::msg::TrafficSignalArray>
    SyncPolicyWithRoughRoi;
  typedef message_filters::Synchronizer<SyncPolicyWithRoughRoi> SyncWithRoughRoi;
  std::shared_ptr<SyncWithRoughRoi> sync_with_rough_roi_;

  autoware_auto_perception_msgs::msg::TrafficSignalArray last_signal_;
};

}  // namespace traffic_light

#endif  // TRAFFIC_LIGHT_FUSION_VISUALIZER__NODELET_HPP_
