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

#ifndef TRAFFIC_LIGHT_CLASSIFIER__CNN_CLASSIFIER_HPP_
#define TRAFFIC_LIGHT_CLASSIFIER__CNN_CLASSIFIER_HPP_

#include "traffic_light_classifier/classifier_interface.hpp"

#include <cuda_utils/cuda_unique_ptr.hpp>
#include <cuda_utils/stream_unique_ptr.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tensorrt_classifier/tensorrt_classifier.hpp>
#include <tensorrt_common/tensorrt_common.hpp>

#include <autoware_perception_msgs/msg/traffic_light_array.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace traffic_light
{

using cuda_utils::CudaUniquePtr;
using cuda_utils::CudaUniquePtrHost;
using cuda_utils::makeCudaStream;
using cuda_utils::StreamUniquePtr;

class CNNClassifier : public ClassifierInterface
{
public:
  explicit CNNClassifier(rclcpp::Node * node_ptr);
  virtual ~CNNClassifier() = default;

  bool getTrafficSignals(
    const std::vector<cv::Mat> & images,
    autoware_perception_msgs::msg::TrafficLightArray & traffic_signals);

private:
  void postProcess(
    int cls, float prob, autoware_perception_msgs::msg::TrafficLight & traffic_signal);
  bool readLabelfile(std::string filepath, std::vector<std::string> & labels);
  bool isColorLabel(const std::string label);
  void outputDebugImage(
    cv::Mat & debug_image, const autoware_perception_msgs::msg::TrafficLight & traffic_signal);

private:
  std::map<int, std::string> state2label_{
    // color
    {autoware_perception_msgs::msg::TrafficLightElement::RED, "red"},
    {autoware_perception_msgs::msg::TrafficLightElement::AMBER, "yellow"},
    {autoware_perception_msgs::msg::TrafficLightElement::GREEN, "green"},
    {autoware_perception_msgs::msg::TrafficLightElement::WHITE, "white"},
    // shape
    {autoware_perception_msgs::msg::TrafficLightElement::CIRCLE, "circle"},
    {autoware_perception_msgs::msg::TrafficLightElement::LEFT_ARROW, "left"},
    {autoware_perception_msgs::msg::TrafficLightElement::RIGHT_ARROW, "right"},
    {autoware_perception_msgs::msg::TrafficLightElement::UP_ARROW, "straight"},
    {autoware_perception_msgs::msg::TrafficLightElement::UP_LEFT_ARROW, "up_left"},
    {autoware_perception_msgs::msg::TrafficLightElement::UP_RIGHT_ARROW, "up_right"},
    {autoware_perception_msgs::msg::TrafficLightElement::DOWN_ARROW, "down"},
    {autoware_perception_msgs::msg::TrafficLightElement::DOWN_LEFT_ARROW, "down_left"},
    {autoware_perception_msgs::msg::TrafficLightElement::DOWN_RIGHT_ARROW, "down_right"},
    {autoware_perception_msgs::msg::TrafficLightElement::CROSS, "cross"},
    // other
    {autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN, "unknown"},
  };

  std::map<std::string, int> label2state_{
    // color
    {"red", autoware_perception_msgs::msg::TrafficLightElement::RED},
    {"yellow", autoware_perception_msgs::msg::TrafficLightElement::AMBER},
    {"green", autoware_perception_msgs::msg::TrafficLightElement::GREEN},
    {"white", autoware_perception_msgs::msg::TrafficLightElement::WHITE},
    // shape
    {"circle", autoware_perception_msgs::msg::TrafficLightElement::CIRCLE},
    {"left", autoware_perception_msgs::msg::TrafficLightElement::LEFT_ARROW},
    {"right", autoware_perception_msgs::msg::TrafficLightElement::RIGHT_ARROW},
    {"straight", autoware_perception_msgs::msg::TrafficLightElement::UP_ARROW},
    {"up_left", autoware_perception_msgs::msg::TrafficLightElement::UP_LEFT_ARROW},
    {"up_right", autoware_perception_msgs::msg::TrafficLightElement::UP_RIGHT_ARROW},
    {"down", autoware_perception_msgs::msg::TrafficLightElement::DOWN_ARROW},
    {"down_left", autoware_perception_msgs::msg::TrafficLightElement::DOWN_LEFT_ARROW},
    {"down_right", autoware_perception_msgs::msg::TrafficLightElement::DOWN_RIGHT_ARROW},
    {"cross", autoware_perception_msgs::msg::TrafficLightElement::CROSS},
    // other
    {"unknown", autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN},
  };

  rclcpp::Node * node_ptr_;
  int batch_size_;
  std::unique_ptr<tensorrt_classifier::TrtClassifier> classifier_;
  image_transport::Publisher image_pub_;
  std::vector<std::string> labels_;
  std::vector<float> mean_;
  std::vector<float> std_;
};

}  // namespace traffic_light

#endif  // TRAFFIC_LIGHT_CLASSIFIER__CNN_CLASSIFIER_HPP_
