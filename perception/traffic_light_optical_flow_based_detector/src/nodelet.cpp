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

#include "traffic_light_optical_flow_based_detector/nodelet.hpp"

namespace traffic_light
{

void cvMatRoi::setInvalid()
{
  image.reset();
  map_rois.reset();
  ssd_rois.reset();
}

bool cvMatRoi::isValid()
{
  return image != nullptr 
         && map_rois != nullptr 
         && ssd_rois != nullptr
         && map_rois->header.stamp == ssd_rois->header.stamp
         && map_rois->header.frame_id == ssd_rois->header.frame_id;
}

TrafficLightOpticalFlowBasedDetectorNodelet::TrafficLightOpticalFlowBasedDetectorNodelet(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_optical_flow_based_detector", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
  map_roi_sub_.subscribe(this, "~/input/map/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
  sync_.reset(new Sync(SyncPolicy(10), image_sub_, map_roi_sub_));
  sync_->registerCallback(std::bind(&TrafficLightOpticalFlowBasedDetectorNodelet::imageMapRoiCallback, this, _1, _2));
  
  prev_roi_sub_ = create_subscription<TrafficLightRoiArray>(
    "~/input/last_rois", rclcpp::SensorDataQoS(),
    std::bind(&TrafficLightOpticalFlowBasedDetectorNodelet::ssdRoiCallback, this, _1));

  roi_pub_ = this->create_publisher<TrafficLightRoiArray>("~/output/rois", 1);
  prev_image_rois_.setInvalid();
  curr_image_rois_.setInvalid();
}

void TrafficLightOpticalFlowBasedDetectorNodelet::imageMapRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
  const TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg)
{
  // update previous image rois and current image rois
  prev_image_rois_.setInvalid();
  if(curr_image_rois_.isValid()){
    prev_image_rois_.image = std::move(curr_image_rois_.image);
    prev_image_rois_.map_rois = std::move(curr_image_rois_.map_rois);
    prev_image_rois_.ssd_rois = std::move(curr_image_rois_.ssd_rois);
  }
  curr_image_rois_.setInvalid();
  bool convRes = rosMsg2CvMat(in_image_msg, curr_image_rois_.image);
  if(!convRes){
    curr_image_rois_.setInvalid();
  }
  else{
    curr_image_rois_.map_rois = std::make_unique<TrafficLightRoiArray>(*in_map_roi_msg);
  }

  // check if optical flow can be performed
  if(curr_image_rois_.image != nullptr
     && curr_image_rois_.map_rois != nullptr
     && prev_image_rois_.isValid()){
    TrafficLightRoiArray out_msg = performOpticalFlow(in_map_roi_msg);
    roi_pub_->publish(out_msg);
  }
  
  //if optical flow can ot be performed to estimate the rough roi,
  //just publish the map_roi ad rough_roi
  else{
    RCLCPP_INFO_STREAM(get_logger(), "can not perform optical flow.");
    roi_pub_->publish(*in_map_roi_msg);
  }
}

TrafficLightRoiArray TrafficLightOpticalFlowBasedDetectorNodelet::performOpticalFlow(
  const TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg)
{
  const int bound = 50;
  std::vector<cv::Point2f> prevPts, nextPts;
  std::vector<uchar> status;
  std::vector<float> err;
  /**
   * it's time consuming to perform optical flow on the whole image.
   * We just need to crop the image containing all the map/rois with safety boundary
   * (x1, y1): top left corner of cropped image
   * (x2, y2): bottom right corner of cropped image
  */
  int x1 = curr_image_rois_.image->cols;
  int x2 = 0;
  int y1 = curr_image_rois_.image->rows;
  int y2 = 0;

  for(const auto & roi : prev_image_rois_.ssd_rois->rois){
    x1 = std::min(x1, int(roi.roi.x_offset));
    y1 = std::min(y1, int(roi.roi.y_offset));
    x2 = std::max(x2, int(roi.roi.x_offset + roi.roi.width));
    y2 = std::max(y2, int(roi.roi.y_offset + roi.roi.height));
  }

  if(y2 - y1 < 2 || x2 - x1 < 2){
    return *in_map_roi_msg;
  }

  x1 = std::max(0, x1 - bound);
  x2 = std::min(x2 + bound, curr_image_rois_.image->cols - 1);
  y1 = std::max(0, y1 - bound);
  y2 = std::min(y2 + bound, curr_image_rois_.image->rows - 1);
  cv::Rect opticalFlowRoi(x1, y1, x2 - x1, y2 - y1);
  cv::Mat srcImage(*prev_image_rois_.image, opticalFlowRoi);
  cv::Mat dstImage(*curr_image_rois_.image, opticalFlowRoi);


  std::unordered_map<int, std::vector<int> > id2ptIdxVec;
  // calculating the tracking points
  for(const auto & roi : prev_image_rois_.ssd_rois->rois){
    for(int x = roi.roi.x_offset; x < std::min(int(roi.roi.x_offset + roi.roi.width), curr_image_rois_.image->cols); x += 1){
      for(int y = roi.roi.y_offset; y < std::min(int(roi.roi.y_offset + roi.roi.height), curr_image_rois_.image->rows); y += 1){
        id2ptIdxVec[roi.id].emplace_back(prevPts.size());
        // the optical flow would be performed on the cropped image.
        // need to translate the coordinates 
        prevPts.emplace_back(x - x1, y - y1);
      }
    }
  }
  cv::calcOpticalFlowPyrLK(srcImage, dstImage, prevPts, nextPts, status, err);

  TrafficLightRoiArray out_msg;
  // for every map/roi, if optical flow can not be performed to track the corresponding traffic light,
  // we still need to output one roi. Therefore, just initialize with map/rois
  out_msg = *in_map_roi_msg;
  for(size_t i = 0; i < out_msg.rois.size(); i++){
    int roi_id = out_msg.rois[i].id;
    if(id2ptIdxVec.count(roi_id) && id2ptIdxVec[roi_id].empty() == false){
      int next_x_sum = 0, next_y_sum = 0, count = 0;
      for(int ptIdx : id2ptIdxVec[roi_id]){
        if(status[ptIdx]){
          next_x_sum += nextPts[ptIdx].x;
          next_y_sum += nextPts[ptIdx].y;
          count++;
        }
      }
      if(count != 0){
        // translate the coordinates from cropped image to original image
        int cx = next_x_sum / count + x1;
        int cy = next_y_sum / count + y1;
        out_msg.rois[i].roi.x_offset = cx - out_msg.rois[i].roi.width / 2;
        out_msg.rois[i].roi.y_offset = cy - out_msg.rois[i].roi.height / 2;
      }
    }
  }
  return out_msg;
}

void TrafficLightOpticalFlowBasedDetectorNodelet::ssdRoiCallback(const TrafficLightRoiArray::ConstSharedPtr input_msg)
{
  if(curr_image_rois_.image != nullptr
     && curr_image_rois_.map_rois != nullptr
     && curr_image_rois_.map_rois->header.stamp == input_msg->header.stamp
     && input_msg->rois.empty() == false){
    curr_image_rois_.ssd_rois = std::make_unique<TrafficLightRoiArray>(*input_msg);
  }
  else{
    curr_image_rois_.setInvalid();
  }
}

bool TrafficLightOpticalFlowBasedDetectorNodelet::rosMsg2CvMat(
  const sensor_msgs::msg::Image::ConstSharedPtr image_msg, std::unique_ptr<cv::Mat> & image)
{
  try {
    image = std::make_unique<cv::Mat>(cv_bridge::toCvCopy(image_msg, "rgb8")->image);
    cv::cvtColor(*image, *image, cv::COLOR_RGB2GRAY);
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

