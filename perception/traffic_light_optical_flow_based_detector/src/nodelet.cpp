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
// void createRect(cv::Mat & image, const autoware_auto_perception_msgs::msg::TrafficLightRoi & tl_roi)
// {
//   cv::Scalar color;
//   if(tl_roi.id == -1){
//     color = cv::Scalar{0, 255, 255};
//   }
//   else{
//     color = cv::Scalar{0, 255, 0};
//   }
//   cv::rectangle(
//     image, cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
//     cv::Point(tl_roi.roi.x_offset + tl_roi.roi.width, tl_roi.roi.y_offset + tl_roi.roi.height),
//     color, 2);
// }
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

autoware_auto_perception_msgs::msg::TrafficLightRoi TrafficLightOpticalFlowBasedDetectorNodelet::predictRoi(
  const autoware_auto_perception_msgs::msg::TrafficLightRoi& ssd_roi,
  const autoware_auto_perception_msgs::msg::TrafficLightRoi& map_roi)
{
  std::string root_dir = "/home/mingyuli/Desktop/tasks/2022/traffic-light/reports/20221115/test/";
  double stamp = rclcpp::Time(curr_image_.header.stamp).seconds();
  std::string img_src = root_dir + std::to_string(stamp) + "-" + std::to_string(ssd_roi.id) + "_src.jpeg";
  std::string img_tar = root_dir + std::to_string(stamp) + "-" + std::to_string(ssd_roi.id) + "_dst.jpeg";
  int step = 1;
  // int width = std::max(300, int(map_roi.roi.width));
  // int height = std::max(300, int(map_roi.roi.height));
  int width = map_roi.roi.width;
  int height = map_roi.roi.height;
  int cx = ssd_roi.roi.x_offset + ssd_roi.roi.width / 2;
  int cy = ssd_roi.roi.y_offset + ssd_roi.roi.height / 2;
  int bound = 30;
  // top left x of the whole image
  int x1 = std::max(0, cx - width / 2 - bound);
  // top left y of the whole image
  int y1 = std::max(0, cy - height / 2);
  // bottom right x of the whole image
  int x2 = std::min(curr_image_.image.cols - 1, int(cx + width / 2) + bound);
  // bottom right y of the whole image
  int y2 = std::min(curr_image_.image.rows - 1, int(cy + height / 2));
  // use a relatively large image (slightly larger than the map_roi but much smaller than the whole image) for computing the optical flow
  cv::Rect opticalFlowROI(x1, y1, x2 - x1, y2 - y1);
  cv::Mat srcImage(prev_image_.image, opticalFlowROI);
  cv::Mat dstImage(curr_image_.image, opticalFlowROI);
  cv::imwrite(img_src, srcImage);
  cv::imwrite(img_tar, dstImage);
  std::vector<cv::Point2f> prevPts, nextPts;
  std::vector<uchar> status;
  std::vector<float> err;
  for(int ssd_roi_x = ssd_roi.roi.x_offset; ssd_roi_x < int(ssd_roi.roi.x_offset + ssd_roi.roi.width); ssd_roi_x += step){
    for(int ssd_roi_y = ssd_roi.roi.y_offset; ssd_roi_y < int(ssd_roi.roi.y_offset + ssd_roi.roi.height); ssd_roi_y += step){
      prevPts.emplace_back(ssd_roi_x - x1, ssd_roi_y - y1);
    }
  }
  autoware_auto_perception_msgs::msg::TrafficLightRoi out_roi = map_roi;
  if(prevPts.empty()){
    return out_roi;
  }

  int sum_x = 0, sum_y = 0, count = 0;
  cv::calcOpticalFlowPyrLK(srcImage, dstImage, prevPts, nextPts, status, err);
  for(size_t i = 0; i < status.size(); i++){
    if(status[i]){
      count++;
      sum_x += nextPts[i].x;
      sum_y += nextPts[i].y;
    }
  }
  
  if(count == 0){
  }
  else{
    cx = sum_x / count + x1;
    cy = sum_y / count + y1;
    out_roi.roi.x_offset = std::max(0, int(cx - out_roi.roi.width / 2));
    out_roi.roi.y_offset = std::max(0, int(cy - out_roi.roi.height / 2));
    out_roi.roi.width = std::min(out_roi.roi.width, curr_image_.image.cols - out_roi.roi.x_offset);
    out_roi.roi.height = std::min(out_roi.roi.height, curr_image_.image.rows - out_roi.roi.y_offset);
  }
  return out_roi;
}

void TrafficLightOpticalFlowBasedDetectorNodelet::imageMapRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
  const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg)
{
  if(curr_image_ .valid){
    prev_image_ = std::move(curr_image_);
  }
  bool convRes = rosMsg2CvMat(in_image_msg, curr_image_.image);
  
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
    //only for debug. To delete
    // std::string parent_dir = "/home/mingyuli/Desktop/tasks/2022/traffic-light/reports/20221111/optical_flow/";
    // double stamp = 1.0 * curr_image_.header.stamp.sec + 1e-9 * curr_image_.header.stamp.nanosec;
    // std::string img_path = parent_dir + std::to_string(stamp) + ".jpg";
    // cv::Mat img_save = cv_bridge::toCvCopy(in_image_msg, "bgr8")->image;
  
    //create a id-roi hash map
    std::unordered_map<autoware_auto_perception_msgs::msg::TrafficLightRoi::_id_type,
                       autoware_auto_perception_msgs::msg::TrafficLightRoi const*> idRoiHash;
    for(const auto & roi : last_roi_->rois){
      idRoiHash[roi.id] = &roi;
    }

    autoware_auto_perception_msgs::msg::TrafficLightRoiArray out_msg;
    out_msg.header = in_map_roi_msg->header;
    out_msg.rois.resize(in_map_roi_msg->rois.size());
    //for every map roi, check if there's a corresponding ssd roi detected last frame.
    //if so, predict the new position with optical flow.
    //otherwise, just use the map roi.
    for(size_t idx = 0; idx < in_map_roi_msg->rois.size(); idx++){
      const auto & map_roi = in_map_roi_msg->rois[idx];
      out_msg.rois[idx] = idRoiHash.count(map_roi.id)? predictRoi(*idRoiHash[map_roi.id], map_roi) : map_roi;      
      if(out_msg.rois[idx].id == -1){
        //::createRect(img_save, out_msg.rois[idx]);
        //::createRect(img_save, map_roi);
        out_msg.rois[idx].id = map_roi.id;
      }      
    }
    roi_pub_->publish(out_msg);

    //cv::imwrite(img_path, img_save);
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

