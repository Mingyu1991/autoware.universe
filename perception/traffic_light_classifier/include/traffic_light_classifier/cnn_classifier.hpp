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

#include <autoware_auto_perception_msgs/msg/traffic_light.hpp>

#include <cv_bridge/cv_bridge.h>

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
    autoware_auto_perception_msgs::msg::TrafficSignalArray & traffic_signals) override;

private:
  void postProcess(
    int cls, float prob, autoware_auto_perception_msgs::msg::TrafficSignal & traffic_signal);
  bool readLabelfile(std::string filepath, std::vector<std::string> & labels);
  bool isColorLabel(const std::string label);
  void outputDebugImage(
    cv::Mat & debug_image,
    const autoware_auto_perception_msgs::msg::TrafficSignal & traffic_signal);

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

  std::map<std::string, int> label2state_{
    // color
    {"red", autoware_auto_perception_msgs::msg::TrafficLight::RED},
    {"yellow", autoware_auto_perception_msgs::msg::TrafficLight::AMBER},
    {"green", autoware_auto_perception_msgs::msg::TrafficLight::GREEN},
    {"white", autoware_auto_perception_msgs::msg::TrafficLight::WHITE},
    // shape
    {"circle", autoware_auto_perception_msgs::msg::TrafficLight::CIRCLE},
    {"left", autoware_auto_perception_msgs::msg::TrafficLight::LEFT_ARROW},
    {"right", autoware_auto_perception_msgs::msg::TrafficLight::RIGHT_ARROW},
    {"straight", autoware_auto_perception_msgs::msg::TrafficLight::UP_ARROW},
    {"up_left", autoware_auto_perception_msgs::msg::TrafficLight::UP_LEFT_ARROW},
    {"up_right", autoware_auto_perception_msgs::msg::TrafficLight::UP_RIGHT_ARROW},
    {"down", autoware_auto_perception_msgs::msg::TrafficLight::DOWN_ARROW},
    {"down_left", autoware_auto_perception_msgs::msg::TrafficLight::DOWN_LEFT_ARROW},
    {"down_right", autoware_auto_perception_msgs::msg::TrafficLight::DOWN_RIGHT_ARROW},
    {"cross", autoware_auto_perception_msgs::msg::TrafficLight::CROSS},
    // other
    {"unknown", autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN},
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
