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

#ifndef TRAFFIC_LIGHT_FINE_DETECTOR__NODELET_HPP_
#define TRAFFIC_LIGHT_FINE_DETECTOR__NODELET_HPP_

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tensorrt_yolox/tensorrt_yolox.hpp>

#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

typedef struct Detection
{
  float x, y, w, h, prob;
} Detection;

namespace traffic_light
{
class TrafficLightFineDetectorNodelet : public rclcpp::Node
{
  using TrafficLightRoi = autoware_auto_perception_msgs::msg::TrafficLightRoi;
  using TrafficLightRoiArray = autoware_auto_perception_msgs::msg::TrafficLightRoiArray;
public:
  explicit TrafficLightFineDetectorNodelet(const rclcpp::NodeOptions & options);
  void connectCb();
  void callback(
    const sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    const TrafficLightRoiArray::ConstSharedPtr rough_roi_msg,
    const TrafficLightRoiArray::ConstSharedPtr expect_roi_msg);

private:
  bool cvMat2CnnInput(
    const std::vector<cv::Mat> & in_imgs, const int num_rois, std::vector<float> & data);
  float evalMatchScore(
    std::map<int, TrafficLightRoi> & id2expectRoi,
    std::map<int, tensorrt_yolox::ObjectArray> & id2detections,
    std::map<int, tensorrt_yolox::Object> & id2bestDetection);
  void detectionMatch(
    std::map<int, TrafficLightRoi> & id2expectRoi,
    std::map<int, tensorrt_yolox::ObjectArray> & id2detections,
    TrafficLightRoiArray & out_rois);
  bool rosMsg2CvMat(
    const sensor_msgs::msg::Image::ConstSharedPtr image_msg, cv::Mat & image,
    std::string encode = "rgb8");
  bool fitInFrame(cv::Point & lt, cv::Point & rb, const cv::Size & size);
  bool readLabelFile(const std::string & filepath, int & tlr_id, int & num_class);

  // variables
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<TrafficLightRoiArray> rough_roi_sub_;
  message_filters::Subscriber<TrafficLightRoiArray> expect_roi_sub_;
  std::mutex connect_mutex_;
  rclcpp::Publisher<TrafficLightRoiArray>::SharedPtr output_roi_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr exe_time_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, TrafficLightRoiArray, TrafficLightRoiArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, TrafficLightRoiArray, TrafficLightRoiArray>
    ApproximateSyncPolicy;
  typedef message_filters::Synchronizer<ApproximateSyncPolicy> ApproximateSync;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  bool is_approximate_sync_;
  double score_thresh_;
  int tlr_id_;

  std::unique_ptr<tensorrt_yolox::TrtYoloX> trt_yolox_;
};  // TrafficLightFineDetectorNodelet

}  // namespace traffic_light

#endif  // TRAFFIC_LIGHT_FINE_DETECTOR__NODELET_HPP_
