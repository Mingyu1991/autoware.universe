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

#include "traffic_light_fine_detector/nodelet.hpp"

#if (defined(_MSC_VER) or (defined(__GNUC__) and (7 <= __GNUC_MAJOR__)))
#include <filesystem>
namespace fs = ::std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = ::std::experimental::filesystem;
#endif

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
float calWeightedIou(
  const sensor_msgs::msg::RegionOfInterest & bbox1, const tensorrt_yolox::Object & bbox2)
{
  int x1 = std::max(static_cast<int>(bbox1.x_offset), bbox2.x_offset);
  int x2 = std::min(static_cast<int>(bbox1.x_offset + bbox1.width), bbox2.x_offset + bbox2.width);
  int y1 = std::max(static_cast<int>(bbox1.y_offset), bbox2.y_offset);
  int y2 = std::min(static_cast<int>(bbox1.y_offset + bbox1.height), bbox2.y_offset + bbox2.height);
  int area1 = std::max(x2 - x1, 0) * std::max(y2 - y1, 0);
  int area2 = bbox1.width * bbox1.height + bbox2.width * bbox2.height - area1;
  if (area2 == 0) {
    return 0.0;
  }
  return bbox2.score * area1 / area2;
}

}  // namespace

namespace traffic_light
{
inline std::vector<float> toFloatVector(const std::vector<double> double_vector)
{
  return std::vector<float>(double_vector.begin(), double_vector.end());
}

TrafficLightFineDetectorNodelet::TrafficLightFineDetectorNodelet(
  const rclcpp::NodeOptions & options)
: Node("traffic_light_fine_detector_node", options)
{
  const int num_class = 2;
  width_ = 640;
  height_ = 640;
  using std::placeholders::_1;
  using std::placeholders::_2;

  std::string model_path = declare_parameter("fine_detection_onnx_file", "");
  std::string label_path = declare_parameter("fine_detection_label_file", "");
  std::string precision = declare_parameter("fine_detector_precision", "fp32");
  // Objects with a score lower than this value will be ignored.
  // This threshold will be ignored if specified model contains EfficientNMS_TRT module in it
  float score_threshold = declare_parameter("fine_detection_score_thresh", 0.3);
  // Detection results will be ignored if IoU over this value.
  // This threshold will be ignored if specified model contains EfficientNMS_TRT module in it
  float nms_threshold = declare_parameter("fine_detection_nms_thresh", 0.65);
  is_approximate_sync_ = this->declare_parameter<bool>("approximate_sync", false);

  if (readLabelFile(label_path)) {
    RCLCPP_ERROR(this->get_logger(), "Could not find tlr id");
  }
  trt_yolox_ = std::make_unique<tensorrt_yolox::TrtYoloX>(
    model_path, precision, num_class, score_threshold, nms_threshold);

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrafficLightFineDetectorNodelet::connectCb, this));

  std::lock_guard<std::mutex> lock(connect_mutex_);
  output_roi_pub_ =
    this->create_publisher<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>(
      "~/output/rois", 1);
  exe_time_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/debug/exe_time_ms", 1);
  if (is_approximate_sync_) {
    approximate_sync_.reset(new ApproximateSync(ApproximateSyncPolicy(10), image_sub_, roi_sub_));
    approximate_sync_->registerCallback(
      std::bind(&TrafficLightFineDetectorNodelet::callback, this, _1, _2));
  } else {
    sync_.reset(new Sync(SyncPolicy(10), image_sub_, roi_sub_));
    sync_->registerCallback(std::bind(&TrafficLightFineDetectorNodelet::callback, this, _1, _2));
  }
}

void TrafficLightFineDetectorNodelet::connectCb()
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (output_roi_pub_->get_subscription_count() == 0) {
    image_sub_.unsubscribe();
    roi_sub_.unsubscribe();
  } else if (!image_sub_.getSubscriber()) {
    image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
    roi_sub_.subscribe(this, "~/input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
  }
}

void TrafficLightFineDetectorNodelet::callback(
  const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
  const autoware_auto_perception_msgs::msg::TrafficLightRoughRoiArray::ConstSharedPtr in_roi_msg)
{
  if (in_image_msg->width < 2 || in_image_msg->height < 2) {
    return;
  }

  using std::chrono::high_resolution_clock;
  using std::chrono::milliseconds;
  const auto exe_start_time = high_resolution_clock::now();
  cv::Mat original_image;
  autoware_auto_perception_msgs::msg::TrafficLightRoiArray out_rois;
  std::map<int, autoware_auto_perception_msgs::msg::TrafficLightRoughRoi> id2roughRoi;
  std::map<int, tensorrt_yolox::ObjectArray> id2detections;

  rosMsg2CvMat(in_image_msg, original_image, "bgr8");
  for (const auto & rough_roi : in_roi_msg->rois) {
    id2roughRoi[rough_roi.id] = rough_roi;
    cv::Point lt(rough_roi.rough_roi.x_offset, rough_roi.rough_roi.y_offset);
    cv::Point rb(
      rough_roi.rough_roi.x_offset + rough_roi.rough_roi.width,
      rough_roi.rough_roi.y_offset + rough_roi.rough_roi.height);
    fitInFrame(lt, rb, cv::Size(original_image.size()));
    cv::Mat cropped_img = cv::Mat(original_image, cv::Rect(lt, rb));
    tensorrt_yolox::ObjectArrays inference_results;
    if (!trt_yolox_->doInference({cropped_img}, inference_results)) {
      RCLCPP_WARN(this->get_logger(), "Fail to inference");
      return;
    }
    for (tensorrt_yolox::Object & detection : inference_results[0]) {
      if (detection.score < 0.3) continue;
      cv::Point lt_roi(lt.x + detection.x_offset, lt.y + detection.y_offset);
      cv::Point rb_roi(lt_roi.x + detection.width, lt_roi.y + detection.height);
      fitInFrame(lt_roi, rb_roi, cv::Size(original_image.size()));
      tensorrt_yolox::Object det = detection;
      det.x_offset = lt_roi.x;
      det.y_offset = lt_roi.y;
      det.width = rb_roi.x - lt_roi.x;
      det.height = rb_roi.y - lt_roi.y;
      id2detections[rough_roi.id].push_back(det);
    }
  }
  detectionMatch(id2roughRoi, id2detections, out_rois);
  out_rois.header = in_roi_msg->header;
  output_roi_pub_->publish(out_rois);
  const auto exe_end_time = high_resolution_clock::now();
  const double exe_time =
    std::chrono::duration_cast<milliseconds>(exe_end_time - exe_start_time).count();
  tier4_debug_msgs::msg::Float32Stamped exe_time_msg;
  exe_time_msg.data = exe_time;
  exe_time_msg.stamp = this->now();
  exe_time_pub_->publish(exe_time_msg);
}

float TrafficLightFineDetectorNodelet::evalMatchScore(
  std::map<int, autoware_auto_perception_msgs::msg::TrafficLightRoughRoi> & id2roughRoi,
  std::map<int, tensorrt_yolox::ObjectArray> & id2detections,
  std::map<int, tensorrt_yolox::Object> & id2bestDetection)
{
  float score_sum = 0.0f;
  id2bestDetection.clear();
  for (const auto & rough_roi_p : id2roughRoi) {
    int tlr_id = rough_roi_p.first;
    float max_score = 0.0f;
    const sensor_msgs::msg::RegionOfInterest & expected_roi = rough_roi_p.second.roi;
    for (const tensorrt_yolox::Object & detection : id2detections[tlr_id]) {
      float score = ::calWeightedIou(expected_roi, detection);
      if (score > max_score) {
        max_score = score;
        id2bestDetection[tlr_id] = detection;
      }
    }
    score_sum += max_score;
  }
  return score_sum;
}

void TrafficLightFineDetectorNodelet::detectionMatch(
  std::map<int, autoware_auto_perception_msgs::msg::TrafficLightRoughRoi> & id2roughRoi,
  std::map<int, tensorrt_yolox::ObjectArray> & id2detections,
  autoware_auto_perception_msgs::msg::TrafficLightRoiArray & out_rois)
{
  float max_score = 0.0f;
  std::map<int, tensorrt_yolox::Object> bestDetections;
  for (const auto & rough_roi_pair : id2roughRoi) {
    int tlr_id = rough_roi_pair.first;
    // the expected roi calculated from tf information
    const sensor_msgs::msg::RegionOfInterest & expect_roi = rough_roi_pair.second.roi;
    int expect_cx = expect_roi.x_offset + expect_roi.width / 2;
    int expect_cy = expect_roi.y_offset + expect_roi.height / 2;
    for (const tensorrt_yolox::Object & det : id2detections[tlr_id]) {
      // for every detection, calculate the center offset between the detection and the
      // corresponding expected roi
      int det_cx = det.x_offset + det.width / 2;
      int det_cy = det.y_offset + det.height / 2;
      int dx = det_cx - expect_cx;
      int dy = det_cy - expect_cy;
      // transfer all the rough rois by the offset
      std::map<int, autoware_auto_perception_msgs::msg::TrafficLightRoughRoi> id2roughRoi_copy =
        id2roughRoi;
      for (auto & p : id2roughRoi_copy) {
        p.second.roi.x_offset += dx;
        p.second.roi.y_offset += dy;
      }
      // calculate the "match score" between expected rois and id2detections_copy
      std::map<int, tensorrt_yolox::Object> id2bestDetection;
      float score = evalMatchScore(id2roughRoi_copy, id2detections, id2bestDetection);
      if (score > max_score) {
        max_score = score;
        bestDetections = id2bestDetection;
      }
    }
  }

  out_rois.rois.clear();
  for (const auto & p : bestDetections) {
    autoware_auto_perception_msgs::msg::TrafficLightRoi tlr;
    tlr.id = p.first;
    tlr.roi.x_offset = p.second.x_offset;
    tlr.roi.y_offset = p.second.y_offset;
    tlr.roi.width = p.second.width;
    tlr.roi.height = p.second.height;
    out_rois.rois.push_back(tlr);
  }
}

bool TrafficLightFineDetectorNodelet::rosMsg2CvMat(
  const sensor_msgs::msg::Image::ConstSharedPtr image_msg, cv::Mat & image, std::string encode)
{
  try {
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, encode);
    image = cv_image->image;
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to convert sensor_msgs::msg::Image to cv::Mat \n%s", e.what());
    return false;
  }

  return true;
}

bool TrafficLightFineDetectorNodelet::fitInFrame(
  cv::Point & lt, cv::Point & rb, const cv::Size & size)
{
  const int width = static_cast<int>(size.width);
  const int height = static_cast<int>(size.height);
  {
    const int x_min = 0, x_max = width - 2;
    const int y_min = 0, y_max = height - 2;
    lt.x = std::min(std::max(lt.x, x_min), x_max);
    lt.y = std::min(std::max(lt.y, y_min), y_max);
  }
  {
    const int x_min = lt.x + 1, x_max = width - 1;
    const int y_min = lt.y + 1, y_max = height - 1;
    rb.x = std::min(std::max(rb.x, x_min), x_max);
    rb.y = std::min(std::max(rb.y, y_min), y_max);
  }

  return true;
}

bool TrafficLightFineDetectorNodelet::readLabelFile(const std::string & filepath)
{
  std::ifstream labelsFile(filepath);
  if (!labelsFile.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open label file. [%s]", filepath.c_str());
    return false;
  }
  std::string label;
  int idx = 0;
  while (getline(labelsFile, label)) {
    if (label == "traffic_light") {
      tlr_id_ = idx;
    }
    idx++;
  }
  return true;
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightFineDetectorNodelet)
