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
// void createRect(cv::Mat & image, const TrafficLightRoi & tl_roi)
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
  
  prev_roi_sub_ = create_subscription<TrafficLightRoiArray>(
    "~/input/last_rois", rclcpp::SensorDataQoS(),
    std::bind(&TrafficLightOpticalFlowBasedDetectorNodelet::lastRoiCallback, this, _1));

  roi_pub_ = this->create_publisher<TrafficLightRoiArray>(
    "~/output/rois", 1);
  prev_image_rois_.valid = false;
  curr_image_rois_.valid = false;
}


TrafficLightRoi TrafficLightOpticalFlowBasedDetectorNodelet::predictRoi(
  const TrafficLightRoi& ssd_roi,
  const TrafficLightRoi& map_roi)
{
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
  int x2 = std::min(curr_image_rois_.image.cols - 1, int(cx + width / 2) + bound);
  // bottom right y of the whole image
  int y2 = std::min(curr_image_rois_.image.rows - 1, int(cy + height / 2));
  // use a relatively large image (slightly larger than the map_roi but much smaller than the whole image) for computing the optical flow
  cv::Rect opticalFlowROI(x1, y1, x2 - x1, y2 - y1);
  cv::Mat srcImage(prev_image_rois_.image, opticalFlowROI);
  cv::Mat dstImage(curr_image_rois_.image, opticalFlowROI);
  std::vector<cv::Point2f> prevPts, nextPts;
  std::vector<uchar> status;
  std::vector<float> err;
  for(int ssd_roi_x = ssd_roi.roi.x_offset; ssd_roi_x < int(ssd_roi.roi.x_offset + ssd_roi.roi.width); ssd_roi_x += step){
    for(int ssd_roi_y = ssd_roi.roi.y_offset; ssd_roi_y < int(ssd_roi.roi.y_offset + ssd_roi.roi.height); ssd_roi_y += step){
      prevPts.emplace_back(ssd_roi_x - x1, ssd_roi_y - y1);
    }
  }
  TrafficLightRoi out_roi = map_roi;
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
    out_roi.roi.width = std::min(out_roi.roi.width, curr_image_rois_.image.cols - out_roi.roi.x_offset);
    out_roi.roi.height = std::min(out_roi.roi.height, curr_image_rois_.image.rows - out_roi.roi.y_offset);
  }
  return out_roi;
}

TrafficLightRoi TrafficLightOpticalFlowBasedDetectorNodelet::predictRoi(
  const TrafficLightRoi& ssd_roi,
  const TrafficLightRoi& map_roi,
  const TrafficLightRoi& prev_map_roi)
{
  /**
   * take rois from two images and calculate the optical flow of the points within ssd_roi
   * the roi center of previous image is the center of ssd_roi
   * the roi center of current image is the center of ssd_roi plus the offset between map_roi and prev_map_roi
  */
  const int prev_map_roi_cx = prev_map_roi.roi.x_offset + prev_map_roi.roi.width / 2;
  const int prev_map_roi_cy = prev_map_roi.roi.y_offset + prev_map_roi.roi.height / 2;
  const int curr_map_roi_cx = map_roi.roi.x_offset + map_roi.roi.width / 2;
  const int curr_map_roi_cy = map_roi.roi.y_offset + map_roi.roi.height / 2;
  const int ssd_cx = ssd_roi.roi.x_offset + ssd_roi.roi.width / 2;
  const int ssd_cy = ssd_roi.roi.y_offset + ssd_roi.roi.height / 2;
  int roi_width = std::max(map_roi.roi.width, prev_map_roi.roi.width);
  int roi_height = std::max(map_roi.roi.height, prev_map_roi.roi.height);
  const int img_width = curr_image_rois_.image.cols;
  const int img_height = curr_image_rois_.image.rows;
  // the roi center of current image
  const int roi_cx = ssd_cx + curr_map_roi_cx - prev_map_roi_cx;
  const int roi_cy = ssd_cy + curr_map_roi_cy - prev_map_roi_cy;
  // the roi center of previous image
  const int prev_roi_cx = ssd_cx;
  const int prev_roi_cy = ssd_cy;
  // make sure the two rois are within the image
  roi_width = std::min({roi_width, 2 * roi_cx, 2 * (img_width - roi_cx - 1), 2 * prev_roi_cx, 2 * (img_width - prev_roi_cx - 1)});
  roi_height = std::min({roi_height, 2 * roi_cy, 2 * (img_height - roi_cy - 1), 2 * prev_roi_cy, 2 * (img_height - prev_roi_cy - 1)});
  roi_width = std::max(roi_width, 0);
  roi_height = std::max(roi_height, 0);
  // top left corner
  const int curr_x1 = roi_cx - roi_width / 2;
  const int curr_y1 = roi_cy - roi_height / 2;
  const int prev_x1 = prev_roi_cx - roi_width / 2;
  const int prev_y1 = prev_roi_cy - roi_height / 2;
  cv::Rect current_roi(curr_x1, curr_y1, roi_width, roi_height);
  cv::Rect last_roi(prev_x1, prev_y1, roi_width, roi_height);

  const int step = 1;
  const int bound = 20;
  cv::Mat srcImage(prev_image_rois_.image, last_roi);
  cv::Mat dstImage(curr_image_rois_.image, current_roi);
  std::vector<cv::Point2f> prevPts, nextPts;
  std::vector<uchar> status;
  std::vector<float> err;
  for(int ssd_roi_x = ssd_roi.roi.x_offset - bound; ssd_roi_x < int(ssd_roi.roi.x_offset + ssd_roi.roi.width) + bound; ssd_roi_x += step){
    for(int ssd_roi_y = ssd_roi.roi.y_offset - bound; ssd_roi_y < int(ssd_roi.roi.y_offset + ssd_roi.roi.height) + bound; ssd_roi_y += step){
      prevPts.emplace_back(ssd_roi_x - prev_x1, ssd_roi_y - prev_y1);
    }
  }
  TrafficLightRoi out_roi = map_roi;
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
    int cx = sum_x / count + curr_x1;
    int cy = sum_y / count + curr_y1;
    out_roi.roi.x_offset = std::max(0, int(cx - out_roi.roi.width / 2));
    out_roi.roi.y_offset = std::max(0, int(cy - out_roi.roi.height / 2));
    out_roi.roi.width = std::min(out_roi.roi.width, curr_image_rois_.image.cols - out_roi.roi.x_offset);
    out_roi.roi.height = std::min(out_roi.roi.height, curr_image_rois_.image.rows - out_roi.roi.y_offset);
  }
  return out_roi;
}



void TrafficLightOpticalFlowBasedDetectorNodelet::imageMapRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
  const TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg)
{
  if(curr_image_rois_.valid){
    prev_image_rois_ = std::move(curr_image_rois_);
  }
  bool convRes = rosMsg2CvMat(in_image_msg, curr_image_rois_.image);
  if(!convRes){
    curr_image_rois_.valid = false;
  }
  else{
    curr_image_rois_.map_rois = *in_map_roi_msg;
    curr_image_rois_.valid = true;
  }

  // check if optical flow can be performed
  if(curr_image_rois_.valid
     && prev_image_rois_.valid){
    TrafficLightRoiArray out_msg = performOpticalFlow(in_image_msg, in_map_roi_msg);
    //out_msg = performPartialOpticalFlow(in_image_msg, in_map_roi_msg);
    roi_pub_->publish(out_msg);
  }
  
  //if optical flow can ot be performed to estimate the rough roi,
  //just publish the map_roi ad rough_roi
  else{
    RCLCPP_INFO_STREAM(get_logger(), "can not perform optical flow.");
    roi_pub_->publish(*in_map_roi_msg);
  }
}

TrafficLightRoiArray TrafficLightOpticalFlowBasedDetectorNodelet::performPartialOpticalFlow(
  const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
  const TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg
)
{
  std::unordered_map<TrafficLightRoi::_id_type, TrafficLightRoi const*> lastSsdRoiHash, lastMapRoiHash;
  for(const auto & roi : prev_image_rois_.ssd_rois.rois){
    lastSsdRoiHash[roi.id] = &roi;
  }
  for(const auto & roi : prev_image_rois_.map_rois.rois){
    lastMapRoiHash[roi.id] = &roi;
  }

  TrafficLightRoiArray out_msg;
  out_msg.header = in_map_roi_msg->header;
  out_msg.rois.resize(in_map_roi_msg->rois.size());
  std::vector<TrafficLightRoi> curr_map_rois, prev_map_rois, res_rois;
  //for every map roi, check if there's a corresponding ssd roi detected last frame.
  //if so, predict the new position with optical flow.
  //otherwise, just use the map roi.
  for(size_t idx = 0; idx < in_map_roi_msg->rois.size(); idx++){
    const auto & map_roi = in_map_roi_msg->rois[idx];
    if(lastSsdRoiHash.count(map_roi.id)){
      assert(lastMapRoiHash.count(map_roi.id));
      if(lastMapRoiHash.count(map_roi.id) == 0){
        RCLCPP_ERROR_STREAM(get_logger(), "Can not find previous map roi with id: " << map_roi.id);
      }
      out_msg.rois[idx] = predictRoi(*lastSsdRoiHash[map_roi.id], map_roi, *lastMapRoiHash[map_roi.id]);
      curr_map_rois.push_back(map_roi);
      prev_map_rois.push_back(*lastMapRoiHash[map_roi.id]);
      res_rois.push_back(out_msg.rois[idx]);
      //out_msg.rois[idx] = predictRoi(*lastSsdRoiHash[map_roi.id], map_roi);
    }
    else{
      out_msg.rois[idx] = map_roi;
    }
  }
  double stamp = rclcpp::Time(curr_image_rois_.map_rois.header.stamp).seconds();
  std::string path = "/home/mingyuli/Desktop/tasks/2022/traffic-light/reports/20221115/test/" + std::to_string(stamp) + ".jpeg";
  cv::Mat debug_img = cv_bridge::toCvCopy(in_image_msg)->image;
  cv::cvtColor(debug_img, debug_img, cv::COLOR_RGB2BGR);
  for(size_t i = 0; i < curr_map_rois.size(); i++){
    cv::rectangle(debug_img, 
                  cv::Point(curr_map_rois[i].roi.x_offset, curr_map_rois[i].roi.y_offset),
                  cv::Point(curr_map_rois[i].roi.x_offset + curr_map_rois[i].roi.width, curr_map_rois[i].roi.y_offset + curr_map_rois[i].roi.height),
                  cv::Scalar{255, 0, 0}, 2);
    cv::rectangle(debug_img, 
                  cv::Point(prev_map_rois[i].roi.x_offset, prev_map_rois[i].roi.y_offset),
                  cv::Point(prev_map_rois[i].roi.x_offset + prev_map_rois[i].roi.width, prev_map_rois[i].roi.y_offset + prev_map_rois[i].roi.height),
                  cv::Scalar{0, 255, 0}, 2);
    cv::rectangle(debug_img, 
                  cv::Point(res_rois[i].roi.x_offset, res_rois[i].roi.y_offset),
                  cv::Point(res_rois[i].roi.x_offset + res_rois[i].roi.width, res_rois[i].roi.y_offset + res_rois[i].roi.height),
                  cv::Scalar{0, 0, 255}, 2);
  }
  
  cv::imwrite(path, debug_img);
  return out_msg;
}

TrafficLightRoiArray TrafficLightOpticalFlowBasedDetectorNodelet::performOpticalFlow(
  const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
  const TrafficLightRoiArray::ConstSharedPtr in_map_roi_msg)
{
  (void)in_image_msg;
  std::vector<cv::Point2f> prevPts, nextPts;
  std::vector<uchar> status;
  std::vector<float> err;

  // int x1 = std::numeric_limits<int>::max();
  // int x2 = 0;
  // int y1 = std::numeric_limits<int>::max();
  // int y2 = 0;

  for(const auto & roi : prev_image_rois_.ssd_rois.rois){
    for(int x = roi.roi.x_offset; x < std::min(int(roi.roi.x_offset + roi.roi.width), curr_image_rois_.image.cols); x += step_ ){
      for(int y = roi.roi.y_offset; y < std::min(int(roi.roi.y_offset + roi.roi.height), curr_image_rois_.image.rows); y += step_){
        prevPts.emplace_back(x, y);
      }
    }
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  cv::calcOpticalFlowPyrLK(prev_image_rois_.image, curr_image_rois_.image, prevPts, nextPts, status, err);
  auto t2 = std::chrono::high_resolution_clock::now();
  RCLCPP_INFO_STREAM(get_logger(), "prevPts = " << prevPts.size() << ", optical flow t = " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());  
  

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

  TrafficLightRoiArray out_msg;
  out_msg = *in_map_roi_msg;
  for(size_t i = 0; i < out_msg.rois.size(); i++){
    const auto& old_roi = in_map_roi_msg->rois[i];
    int next_x_sum = 0, next_y_sum = 0, count = 0;
    for(int x = old_roi.roi.x_offset; x < std::min(int(old_roi.roi.x_offset + old_roi.roi.width), curr_image_rois_.image.cols); x += step_ ){
      for(int y = old_roi.roi.y_offset; y < std::min(int(old_roi.roi.y_offset + old_roi.roi.height), curr_image_rois_.image.rows); y += step_){
        if(hash.count(x) && hash[x].count(y)){
          next_x_sum += hash[x][y].x;
          next_y_sum += hash[x][y].y;
          count++;
        }
      }
    }
    if(count != 0){
      // center coordinates of the predicted roi
      int cx = next_x_sum / count;
      int cy = next_y_sum / count;
      out_msg.rois[i].roi.x_offset = cx - old_roi.roi.width / 2;
      out_msg.rois[i].roi.y_offset = cy - old_roi.roi.height / 2;
      out_msg.rois[i].roi.width = old_roi.roi.width;
      out_msg.rois[i].roi.height = old_roi.roi.height;
    }
  }
  return out_msg;
}

void TrafficLightOpticalFlowBasedDetectorNodelet::lastRoiCallback(
  const TrafficLightRoiArray::ConstSharedPtr input_msg)
{
  if(curr_image_rois_.valid
     && curr_image_rois_.map_rois.header.stamp == input_msg->header.stamp
     && input_msg->rois.empty() == false){
    curr_image_rois_.ssd_rois = *input_msg;
    RCLCPP_WARN(get_logger(), "curr image roi num = %d", int(curr_image_rois_.ssd_rois.rois.size()));
  }
  else{
    curr_image_rois_.valid = false;
    RCLCPP_WARN(get_logger(), "curr image invalid");
  }
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

