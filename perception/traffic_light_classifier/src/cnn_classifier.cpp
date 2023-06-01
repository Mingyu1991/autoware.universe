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

#include "traffic_light_classifier/cnn_classifier.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <memory>
#include <string>
#include <vector>

namespace traffic_light
{
CNNClassifier::CNNClassifier(rclcpp::Node * node_ptr) : node_ptr_(node_ptr)
{
  image_pub_ = image_transport::create_publisher(
    node_ptr_, "~/output/debug/image", rclcpp::QoS{1}.get_rmw_qos_profile());

  std::string precision;
  std::string label_file_path;
  std::string model_file_path;
  precision = node_ptr_->declare_parameter("classifier_precision", "fp16");
  label_file_path = node_ptr_->declare_parameter("classifier_label_path", "labels.txt");
  model_file_path = node_ptr_->declare_parameter("classifier_model_path", "model.onnx");
  // ros param does not support loading std::vector<float>
  // we have to load std::vector<double> and transfer to std::vector<float>
  auto mean_d =
    node_ptr->declare_parameter("classifier_mean", std::vector<double>{123.675, 116.28, 103.53});
  auto std_d =
    node_ptr->declare_parameter("classifier_std", std::vector<double>{58.395, 57.12, 57.375});
  mean_ = std::vector<float>(mean_d.begin(), mean_d.end());
  std_ = std::vector<float>(std_d.begin(), std_d.end());
  if (mean_.size() != 3 || std_.size() != 3) {
    RCLCPP_ERROR(node_ptr->get_logger(), "classifier_mean and classifier_std must be of size 3");
    return;
  }

  readLabelfile(label_file_path, labels_);
  nvinfer1::Dims input_dim = tensorrt_common::get_input_dims(model_file_path);
  assert(input_dim.d[0] > 0);
  batch_size_ = input_dim.d[0];

  tensorrt_common::BatchConfig batch_config{batch_size_, batch_size_, batch_size_};
  classifier_ = std::make_unique<tensorrt_classifier::TrtClassifier>(
    model_file_path, precision, batch_config, mean_, std_);
}

bool CNNClassifier::getTrafficSignals(
  const std::vector<cv::Mat> & images,
  autoware_auto_perception_msgs::msg::TrafficSignalArray & traffic_signals)
{
  if (images.size() != traffic_signals.signals.size()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "image number should be equal to traffic signal number!");
    return false;
  }
  std::vector<cv::Mat> image_batch;
  int signal_i = 0;

  for (size_t image_i = 0; image_i < images.size(); image_i++) {
    image_batch.emplace_back(images[image_i]);
    // keep the actual batch size
    size_t true_batch_size = image_batch.size();
    // insert fake image since the TRT model requires static batch size
    if (image_i + 1 == images.size()) {
      while (static_cast<int>(image_batch.size()) < batch_size_) {
        image_batch.emplace_back(image_batch.front());
      }
    }
    if (static_cast<int>(image_batch.size()) == batch_size_) {
      std::vector<float> probs;
      std::vector<int> classes;
      bool res = classifier_->doInference(image_batch, classes, probs);
      if (!res || classes.empty() || probs.empty()) {
        return false;
      }
      for (size_t i = 0; i < true_batch_size; i++) {
        postProcess(classes[i], probs[i], traffic_signals.signals[signal_i]);
        /* debug */
        if (0 < image_pub_.getNumSubscribers()) {
          outputDebugImage(image_batch[i], traffic_signals.signals[signal_i]);
        }
        signal_i++;
      }
      image_batch.clear();
    }
  }
  return true;
}

void CNNClassifier::outputDebugImage(
  cv::Mat & debug_image, const autoware_auto_perception_msgs::msg::TrafficSignal & traffic_signal)
{
  float probability;
  std::string label;
  for (std::size_t i = 0; i < traffic_signal.lights.size(); i++) {
    auto light = traffic_signal.lights.at(i);
    const auto light_label = state2label_[light.color] + "-" + state2label_[light.shape];
    label += light_label;
    // all lamp confidence are the same
    probability = light.confidence;
    if (i < traffic_signal.lights.size() - 1) {
      label += ",";
    }
  }

  const int expand_w = 200;
  const int expand_h =
    std::max(static_cast<int>((expand_w * debug_image.rows) / debug_image.cols), 1);

  cv::resize(debug_image, debug_image, cv::Size(expand_w, expand_h));
  cv::Mat text_img(cv::Size(expand_w, 50), CV_8UC3, cv::Scalar(0, 0, 0));
  std::string text = label + " " + std::to_string(probability);
  cv::putText(
    text_img, text, cv::Point(5, 25), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
  cv::vconcat(debug_image, text_img, debug_image);

  const auto debug_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", debug_image).toImageMsg();
  image_pub_.publish(debug_image_msg);
}

void CNNClassifier::postProcess(
  int class_index, float prob, autoware_auto_perception_msgs::msg::TrafficSignal & traffic_signal)
{
  std::string match_label = labels_[class_index];

  // label names are assumed to be comma-separated to represent each lamp
  // e.g.
  // match_label: "red","red-cross","right"
  // split_label: ["red","red-cross","right"]
  // if shape doesn't have color suffix, set GREEN to color state.
  // if color doesn't have shape suffix, set CIRCLE to shape state.
  std::vector<std::string> split_label;
  boost::algorithm::split(split_label, match_label, boost::is_any_of(","));
  for (auto label : split_label) {
    if (label2state_.find(label) == label2state_.end()) {
      RCLCPP_DEBUG(
        node_ptr_->get_logger(), "cnn_classifier does not have a key [%s]", label.c_str());
      continue;
    }
    autoware_auto_perception_msgs::msg::TrafficLight light;
    if (label.find("-") != std::string::npos) {
      // found "-" delimiter in label string
      std::vector<std::string> color_and_shape;
      boost::algorithm::split(color_and_shape, label, boost::is_any_of("-"));
      light.color = label2state_[color_and_shape.at(0)];
      light.shape = label2state_[color_and_shape.at(1)];
    } else {
      if (label == state2label_[autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN]) {
        light.color = autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN;
        light.shape = autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN;
      } else if (isColorLabel(label)) {
        light.color = label2state_[label];
        light.shape = autoware_auto_perception_msgs::msg::TrafficLight::CIRCLE;
      } else {
        light.color = autoware_auto_perception_msgs::msg::TrafficLight::GREEN;
        light.shape = label2state_[label];
      }
    }
    light.confidence = prob;
    traffic_signal.lights.push_back(light);
  }
}

bool CNNClassifier::readLabelfile(std::string filepath, std::vector<std::string> & labels)
{
  std::ifstream labelsFile(filepath);
  if (!labelsFile.is_open()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Could not open label file. [%s]", filepath.c_str());
    return false;
  }
  std::string label;
  while (getline(labelsFile, label)) {
    labels.push_back(label);
  }
  return true;
}

bool CNNClassifier::isColorLabel(const std::string label)
{
  using autoware_auto_perception_msgs::msg::TrafficSignal;
  if (
    label == state2label_[autoware_auto_perception_msgs::msg::TrafficLight::GREEN] ||
    label == state2label_[autoware_auto_perception_msgs::msg::TrafficLight::AMBER] ||
    label == state2label_[autoware_auto_perception_msgs::msg::TrafficLight::RED] ||
    label == state2label_[autoware_auto_perception_msgs::msg::TrafficLight::WHITE]) {
    return true;
  }
  return false;
}

}  // namespace traffic_light
