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
 * Authors: Yukihiro Saito
 *
 */

#include "traffic_light_optical_flow_based_detector/nodelet.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi.hpp>
#include <boost/filesystem.hpp>
namespace
{
void createRect(cv::Mat & image, const autoware_auto_perception_msgs::msg::TrafficLightRoi & tl_roi)
{
  cv::Scalar color;
  if(tl_roi.id == -1){
    color = cv::Scalar{0, 255, 255};
  }
  else{
    color = cv::Scalar{0, 255, 0};
  }
  cv::rectangle(
    image, cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
    cv::Point(tl_roi.roi.x_offset + tl_roi.roi.width, tl_roi.roi.y_offset + tl_roi.roi.height),
    color, 2);
}
}

namespace traffic_light
{
TrafficLightOpticalFlowBasedDetectorNodelet::TrafficLightOpticalFlowBasedDetectorNodelet(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_optical_flow_based_detector", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
  map_roi_sub_.subscribe(this, "~/input/map/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
  sync_.reset(new Sync(SyncPolicy(10), image_sub_, map_roi_sub_));
  sync_->registerCallback(std::bind(&TrafficLightOpticalFlowBasedDetectorNodelet::imageMapRoiCallback, this, _1, _2));
  
  prev_roi_sub_ = create_subscription<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>(
    "~/input/last_rois", rclcpp::SensorDataQoS(),
    std::bind(&TrafficLightOpticalFlowBasedDetectorNodelet::lastRoiCallback, this, _1));

  roi_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>(
    "~/output/rois", 1);
  prev_image_.valid = false;
  curr_image_.valid = false;
  last_roi_.reset();
}

void TrafficLightOpticalFlowBasedDetectorNodelet::imageMapRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
  const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg)
{
  if(curr_image_ .valid){
    prev_image_ = std::move(curr_image_);
  }
  auto t1 = clock();
  bool convRes = rosMsg2CvMat(in_image_msg, curr_image_.image);
  std::cout << "ros 2 cv t = " << 1.0 * (clock() - t1) / CLOCKS_PER_SEC << std::endl;
  if(!convRes){
    curr_image_.valid = false;
  }
  else{
    curr_image_.header = in_image_msg->header;
    curr_image_.valid = true;
  }

  // check if optical flow can be performed
  if(curr_image_.valid
     && prev_image_.valid
     && last_roi_ != nullptr
     && last_roi_->rois.empty() == false
     && prev_image_.header.stamp.sec == last_roi_->header.stamp.sec
     && prev_image_.header.stamp.nanosec == last_roi_->header.stamp.nanosec){
    std::string parent_dir = "/home/mingyuli/Desktop/tasks/2022/traffic-light/reports/20221111/optical_flow/";
    double stamp = 1.0 * curr_image_.header.stamp.sec + 1e-9 * curr_image_.header.stamp.nanosec;
    std::string img_path = parent_dir + std::to_string(stamp) + ".jpg";
    cv::Mat img_save = cv_bridge::toCvCopy(in_image_msg, "bgr8")->image;


    std::vector<cv::Point2f> prevPts, nextPts;
    std::vector<uchar> status;
    std::vector<float> err;
    for(const auto & roi : last_roi_->rois){
      for(int x = roi.roi.x_offset; x < std::min(int(roi.roi.x_offset + roi.roi.width), curr_image_.image.cols); x += step_ ){
        for(int y = roi.roi.y_offset; y < std::min(int(roi.roi.y_offset + roi.roi.height), curr_image_.image.rows); y += step_){
          prevPts.emplace_back(x, y);
        }
      }
    }
    
    auto t1 = clock();
    cv::calcOpticalFlowPyrLK(prev_image_.image, curr_image_.image, prevPts, nextPts, status, err);
    RCLCPP_INFO_STREAM(get_logger(), "prevPts = " << prevPts.size() << ", optical flow t = " << 1.0 * (clock() - t1) / CLOCKS_PER_SEC);

    std::unordered_map<int, std::unordered_map<int, cv::Point2i> > hash;
    for(size_t i = 0; i < status.size(); i++){
      if(status[i]){
        int x1 = prevPts[i].x;
        int x2 = nextPts[i].x;
        int y1 = prevPts[i].y;
        int y2 = nextPts[i].y;
        hash[x1][y1] = cv::Point2i{x2, y2};
      }
    }

    autoware_auto_perception_msgs::msg::TrafficLightRoiArray out_msg;
    out_msg.header = in_map_roi_msg->header;
    out_msg.rois.resize(in_map_roi_msg->rois.size());
    for(size_t i = 0; i < out_msg.rois.size(); i++){
      const auto& old_roi = in_map_roi_msg->rois[i];
      int next_x_sum = 0, next_y_sum = 0, count = 0;
      for(int x = old_roi.roi.x_offset; x < std::min(int(old_roi.roi.x_offset + old_roi.roi.width), curr_image_.image.cols); x += step_ ){
        for(int y = old_roi.roi.y_offset; y < std::min(int(old_roi.roi.y_offset + old_roi.roi.height), curr_image_.image.rows); y += step_){
          if(hash.count(x) && hash[x].count(y)){
            next_x_sum += hash[x][y].x;
            next_y_sum += hash[x][y].y;
            count++;
          }
        }
      }
      // failed to track any point in the old roi, just use the map roi
      if(count == 0){
        out_msg.rois[i] = old_roi;
      }
      else{
        // center coordinates of the predicted roi
        int cx = next_x_sum / count;
        int cy = next_y_sum / count;
        out_msg.rois[i].roi.x_offset = cx - old_roi.roi.width / 2;
        out_msg.rois[i].roi.y_offset = cy - old_roi.roi.height / 2;
        out_msg.rois[i].roi.width = old_roi.roi.width;
        out_msg.rois[i].roi.height = old_roi.roi.height;
        out_msg.rois[i].id = -1;
        ::createRect(img_save, out_msg.rois[i]);
        ::createRect(img_save, old_roi);
      }
    }

    roi_pub_->publish(out_msg);

    cv::imwrite(img_path, img_save);
  }
  
  //if optical flow can ot be performed to estimate the rough roi,
  //just publish the map_roi ad rough_roi
  else{
    RCLCPP_INFO_STREAM(get_logger(), "can not perform optical flow.");
    roi_pub_->publish(*in_map_roi_msg);
  }
}

void TrafficLightOpticalFlowBasedDetectorNodelet::lastRoiCallback(
  const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr input_msg)
{
  last_roi_ = std::make_shared<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>(*input_msg);
}

bool TrafficLightOpticalFlowBasedDetectorNodelet::rosMsg2CvMat(
  const sensor_msgs::msg::Image::ConstSharedPtr image_msg, cv::Mat & image)
{
  try {
    image = cv_bridge::toCvCopy(image_msg, "rgb8")->image;
    cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to convert sensor_msgs::msg::Image to cv::Mat \n%s", e.what());
    return false;
  }

  return true;
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightOpticalFlowBasedDetectorNodelet)

