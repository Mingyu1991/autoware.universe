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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <traffic_light_fusion_visualizer/nodelet.hpp>

#include <memory>
#include <string>
#include <utility>

namespace traffic_light
{
TrafficLightFusionVisualizerNodelet::TrafficLightFusionVisualizerNodelet(
  const rclcpp::NodeOptions & options)
: Node("traffic_light_fusion_visualizer_node", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  sync_with_rough_roi_.reset(
    new SyncWithRoughRoi(SyncPolicyWithRoughRoi(10), image_sub_, rough_roi_sub_, signal_sub_));
  sync_with_rough_roi_->registerCallback(
    std::bind(&TrafficLightFusionVisualizerNodelet::imageRoughRoiCallback, this, _1, _2, _3));

  image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
  rough_roi_sub_.subscribe(this, "~/input/rough/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
  signal_sub_.subscribe(this, "~/input/traffic_signals", rclcpp::QoS{1}.get_rmw_qos_profile());
}

bool TrafficLightFusionVisualizerNodelet::createRect(
  cv::Mat & image, const autoware_auto_perception_msgs::msg::TrafficLightRoi & tl_roi,
  const cv::Scalar & color)
{
  cv::rectangle(
    image, cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
    cv::Point(tl_roi.roi.x_offset + tl_roi.roi.width, tl_roi.roi.y_offset + tl_roi.roi.height),
    color, 2);
  cv::putText(
    image, std::to_string(tl_roi.id),
    cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset + tl_roi.roi.height + 30),
    cv::FONT_HERSHEY_COMPLEX, 1.0, color, 2, CV_AA);
  return true;
}

bool TrafficLightFusionVisualizerNodelet::createRect(
  cv::Mat & image, const autoware_auto_perception_msgs::msg::TrafficLightRoi & tl_roi,
  const ClassificationResult & result)
{
  cv::Scalar color;
  if (result.label.find("red") != std::string::npos) {
    color = cv::Scalar{255, 0, 0};
  } else if (result.label.find("yellow") != std::string::npos) {
    color = cv::Scalar{0, 255, 0};
  } else if (result.label.find("green") != std::string::npos) {
    color = cv::Scalar{0, 0, 255};
  } else {
    color = cv::Scalar{255, 255, 255};
  }

  cv::rectangle(
    image, cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
    cv::Point(tl_roi.roi.x_offset + tl_roi.roi.width, tl_roi.roi.y_offset + tl_roi.roi.height),
    color, 2);

  int offset = 40;
  // cv::putText(
  //   image, std::to_string(result.prob),
  //   cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset - (offset * 0)), cv::FONT_HERSHEY_COMPLEX,
  //   1.1, color, 3);

  cv::putText(
    image, result.label,
    cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset + tl_roi.roi.height + (offset * 2)),
    cv::FONT_HERSHEY_COMPLEX, 2, color, 2);

  return true;
}

bool TrafficLightFusionVisualizerNodelet::getClassificationResult(
  int id, const autoware_auto_perception_msgs::msg::TrafficSignalArray & traffic_signals,
  ClassificationResult & result)
{
  bool has_correspond_traffic_signal = false;
  for (const auto & traffic_signal : traffic_signals.signals) {
    if (id != traffic_signal.map_primitive_id) {
      continue;
    }
    has_correspond_traffic_signal = true;
    for (size_t i = 0; i < traffic_signal.lights.size(); i++) {
      auto light = traffic_signal.lights.at(i);
      result.prob = light.confidence;
      if (light.shape == autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN) {
        result.label += "Unknown-";
      } else if (light.shape == autoware_auto_perception_msgs::msg::TrafficLight::CIRCLE) {
        result.label += state2label_[light.color] + "-";
      } else {
        result.label += state2label_[light.shape] + "-";
      }
    }
  }
  if (result.label.empty() == false) {
    result.label.pop_back();
  }
  return has_correspond_traffic_signal;
}

void TrafficLightFusionVisualizerNodelet::printSignal(
  const autoware_auto_perception_msgs::msg::TrafficSignalArray & msg)
{
  for (const auto & signal : msg.signals) {
    RCLCPP_INFO_STREAM(get_logger(), "  signal id = " << signal.map_primitive_id);
    for (const auto & light : signal.lights) {
      RCLCPP_INFO_STREAM(
        get_logger(), "    light shape = " << int(light.shape) << ", color = " << int(light.color));
    }
  }
}

bool TrafficLightFusionVisualizerNodelet::trafficSignalChanged(
  const autoware_auto_perception_msgs::msg::TrafficSignalArray & msg)
{
  std::map<int, autoware_auto_perception_msgs::msg::TrafficSignal> id2lastMsg;
  for (const auto & signal : last_signal_.signals) {
    id2lastMsg[signal.map_primitive_id] = signal;
  }
  if (msg.signals.size() != last_signal_.signals.size()) {
    return true;
  }
  for (const auto & signal : msg.signals) {
    if (id2lastMsg.count(signal.map_primitive_id) == 0) {
      return true;
    }
    if (signal.lights.size() != id2lastMsg[signal.map_primitive_id].lights.size()) {
      return true;
    }
    for (size_t i = 0; i < signal.lights.size(); i++) {
      if (
        signal.lights[i].shape != id2lastMsg[signal.map_primitive_id].lights[i].shape ||
        signal.lights[i].color != id2lastMsg[signal.map_primitive_id].lights[i].color) {
        return true;
      }
    }
  }
  return false;
}

void TrafficLightFusionVisualizerNodelet::imageRoughRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr &
    input_tl_rough_roi_msg,
  const autoware_auto_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr & input_signal_msg)
{
  // if(trafficSignalChanged(*input_signal_msg)){
  if (input_tl_rough_roi_msg->rois.size()) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(input_image_msg, input_image_msg->encoding);
      for (auto tl_rough_roi : input_tl_rough_roi_msg->rois) {
        ClassificationResult result;
        bool has_correspond_traffic_signal =
          getClassificationResult(tl_rough_roi.id, *input_signal_msg, result);
        if (has_correspond_traffic_signal) {
          // has fine detection and classification results
          createRect(cv_ptr->image, tl_rough_roi, result);
        } else {
          createRect(cv_ptr->image, tl_rough_roi, cv::Scalar(0, 255, 0));
        }
      }
      cv::Mat image = cv_ptr->image;
      cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
      int width = 1080;
      int height = width * image.rows / image.cols;
      cv::resize(image, image, cv::Size(width, height));
      if (input_image_msg->header.frame_id.find("camera6") != std::string::npos) {
        std::string save_path =
          "/home/mingyu/tmp/fusion_vis_output/camera6/" +
          std::to_string(rclcpp::Time(input_signal_msg->header.stamp).seconds()) + "_camera6.jpg";
        cv::imwrite(save_path, image);
      } else {
        std::string save_path =
          "/home/mingyu/tmp/fusion_vis_output/camera7/" +
          std::to_string(rclcpp::Time(input_signal_msg->header.stamp).seconds()) + "_camera7.jpg";
        cv::imwrite(save_path, image);
      }
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(
        get_logger(), "Could not convert from '%s' to 'bgr8'.", input_image_msg->encoding.c_str());
    }
  }
  last_signal_ = *input_signal_msg;
}

}  // namespace traffic_light

RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightFusionVisualizerNodelet)
