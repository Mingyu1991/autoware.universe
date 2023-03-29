// Copyright 2023-2026 the Autoware Foundation
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
//

#include "traffic_light_map_based_detector/occlusion_predictor.hpp"

namespace
{

traffic_light::Ray point2ray(const pcl::PointXYZ & pt)
{
  traffic_light::Ray ray;
  ray.dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
  ray.elevation = RAD2DEG(std::atan2(pt.y, std::hypot(pt.x, pt.z)));
  ray.azimuth = RAD2DEG(std::atan2(pt.x, pt.z));
  ray.pt = pt;
  return ray;
}

}  // namespace

namespace traffic_light
{

void CloudOcclusionPredictor::receivePointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // the timestamps must be increasing
  double history_stamp = rclcpp::Time(history_clouds_.back().header.stamp).seconds();
  double msg_stamp = rclcpp::Time(msg->header.stamp).seconds();
  if (history_clouds_.empty() == false && msg_stamp < history_stamp) {
    return;
  }
  history_clouds_.push_back(*msg);
  // sometimes the top lidar doesn't give any point. we need to filter out these clouds
}

void CloudOcclusionPredictor::update(
  const sensor_msgs::msg::CameraInfo & camera_info, const tf2_ros::Buffer & tf_buffer,
  const std::vector<autoware_auto_perception_msgs::msg::TrafficLightRoi> & tl_rough_rois)
{
  if (history_clouds_.empty()) {
    return;
  }
  /**
   * find the cloud whose timestamp is closest to the stamp of rough_rois
   */
  std::list<sensor_msgs::msg::PointCloud2>::iterator closest_it;
  double min_stamp_diff = std::numeric_limits<double>::max();
  for (auto it = history_clouds_.begin(); it != history_clouds_.end(); it++) {
    double stamp_diff = std::abs(
      rclcpp::Time(it->header.stamp).seconds() - rclcpp::Time(camera_info.header.stamp).seconds());
    if (stamp_diff < min_stamp_diff) {
      min_stamp_diff = stamp_diff;
      closest_it = it;
    }
  }
  cloud_delay_ = rclcpp::Time(camera_info.header.stamp).seconds() -
                 rclcpp::Time(closest_it->header.stamp).seconds();
  /**
   * erase clouds earlier than closest_it since their timestamps couldn't be closer to following
   * rough_rois than closest_it
   */
  for (auto it = history_clouds_.begin(); it != closest_it;) {
    it = history_clouds_.erase(it);
  }

  /**
   * get necessary tf information
   */
  try {
    camera2cloud_ =
      tf2::transformToEigen(tf_buffer.lookupTransform(
                              camera_info.header.frame_id, history_clouds_.front().header.frame_id,
                              rclcpp::Time(camera_info.header.stamp),
                              rclcpp::Duration::from_seconds(0.2)))
        .matrix();
  } catch (tf2::TransformException & ex) {
    std::cout << "Error: cannot get transform from map frame to "
              << history_clouds_.front().header.frame_id << std::endl;
    return;
  }

  cloudPreprocess(camera_info, tl_rough_rois);
}

void CloudOcclusionPredictor::cloudPreprocess(
  const sensor_msgs::msg::CameraInfo & camera_info,
  const std::vector<autoware_auto_perception_msgs::msg::TrafficLightRoi> & tl_rough_rois)
{
  lidar_rays_.clear();
  if (history_clouds_.empty()) {
    return;
  }
  // points in camera frame
  pcl::PointCloud<pcl::PointXYZ> cloud_camera;
  // filtered points within traffic light rois in camera frame
  pcl::PointCloud<pcl::PointXYZ> cloud_in_rois;
  tier4_autoware_utils::transformPointCloudFromROSMsg(
    history_clouds_.front(), cloud_camera, camera2cloud_);
  filterCloud(cloud_camera, tl_rough_rois, camera_info, cloud_in_rois);

  for (const pcl::PointXYZ & pt : cloud_in_rois) {
    Ray ray = ::point2ray(pt);
    lidar_rays_[static_cast<int>(ray.azimuth)][static_cast<int>(ray.elevation)].push_back(ray);
  }
}

void CloudOcclusionPredictor::filterCloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in,
  const std::vector<autoware_auto_perception_msgs::msg::TrafficLightRoi> & tl_rough_rois,
  const sensor_msgs::msg::CameraInfo & camera_info, pcl::PointCloud<pcl::PointXYZ> & cloud_out)
{
  cloud_out.clear();
  // the points very close to the camera should not be used in the occlusion estimation,
  // since they could be noise or from the other nearby sensors
  const float min_dist_to_cam = 1.0f;
  pinhole_camera_model_.fromCameraInfo(camera_info);

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  // the x and y coordiante of the rough rois are un-rectified coordiantes.
  // we need to convert them to rectified coordiantes.
  for (const auto & roi : tl_rough_rois) {
    cv::Point2d top_left_pt(roi.roi.x_offset, roi.roi.y_offset);
    cv::Point2d bottom_right_pt(
      roi.roi.x_offset + roi.roi.width, roi.roi.y_offset + roi.roi.height);
    top_left_pt = pinhole_camera_model_.rectifyPoint(top_left_pt);
    bottom_right_pt = pinhole_camera_model_.rectifyPoint(bottom_right_pt);
    min_x = std::min(min_x, top_left_pt.x);
    max_x = std::max(max_y, bottom_right_pt.x);
    min_y = std::min(min_y, top_left_pt.y);
    max_y = std::max(max_y, bottom_right_pt.y);
  }

  for (size_t i = 0; i < cloud_in.size(); i++) {
    if (cloud_in[i].z <= 0) {
      continue;
    }
    float dist_to_cam = std::sqrt(
      cloud_in[i].x * cloud_in[i].x + cloud_in[i].y * cloud_in[i].y +
      cloud_in[i].z * cloud_in[i].z);
    if (dist_to_cam <= min_dist_to_cam) {
      continue;
    }
    cv::Point2d pixel = pinhole_camera_model_.project3dToPixel(
      cv::Point3d(cloud_in[i].x, cloud_in[i].y, cloud_in[i].z));
    if (pixel.x >= min_x && pixel.x <= max_x && pixel.y >= min_y && pixel.y <= max_y) {
      cloud_out.push_back(cloud_in[i]);
    }
  }
}

sensor_msgs::msg::PointCloud2 CloudOcclusionPredictor::debug(
  const sensor_msgs::msg::CameraInfo & camera_info)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(debug_cloud_, msg);
  msg.header = camera_info.header;
  debug_cloud_.clear();
  return msg;
}

void CloudOcclusionPredictor::sampleTrafficLightRoi(
  const geometry_msgs::msg::Point & top_left, const geometry_msgs::msg::Point & bottom_right,
  uint32_t horizontal_sample_num, uint32_t vertical_sample_num,
  pcl::PointCloud<pcl::PointXYZ> & cloud_out)
{
  cloud_out.clear();
  float x1 = static_cast<float>(top_left.x);
  float y1 = static_cast<float>(top_left.y);
  float z1 = static_cast<float>(top_left.z);
  float x2 = static_cast<float>(bottom_right.x);
  float y2 = static_cast<float>(bottom_right.y);
  float z2 = static_cast<float>(bottom_right.z);
  for (uint32_t i1 = 0; i1 < horizontal_sample_num; i1++) {
    for (uint32_t i2 = 0; i2 < vertical_sample_num; i2++) {
      float x = x1 + (x2 - x1) * i1 / (horizontal_sample_num - 1);
      float y = y1 + (y2 - y1) * i2 / (vertical_sample_num - 1);
      float z = z1 + (z2 - z1) * i1 / (horizontal_sample_num - 1);
      cloud_out.push_back(pcl::PointXYZ(x, y, z));
    }
  }
}

uint32_t CloudOcclusionPredictor::predict(
  const geometry_msgs::msg::Point & roi_top_left,
  const geometry_msgs::msg::Point & roi_bottom_right, double azimuth_occlusion_resolution,
  double elevation_occlusion_resolution)
{
  if (history_clouds_.empty()) {
    return 0;
  }
  const uint32_t horizontal_sample_num = 20;
  const uint32_t vertical_sample_num = 20;
  static_assert(horizontal_sample_num > 1 && vertical_sample_num > 1);
  const float min_dist_from_occlusion_to_tl = 5.0f;

  pcl::PointCloud<pcl::PointXYZ> tl_sample_cloud;
  sampleTrafficLightRoi(
    roi_top_left, roi_bottom_right, horizontal_sample_num, vertical_sample_num, tl_sample_cloud);
  debug_cloud_ += tl_sample_cloud;
  uint32_t occluded_num = 0;
  for (const pcl::PointXYZ & tl_pt : tl_sample_cloud) {
    Ray tl_ray = ::point2ray(tl_pt);
    bool occluded = false;
    // the azimuth and elevation range to search for points that may occlude tl_pt
    int min_azimuth = static_cast<int>(tl_ray.azimuth - azimuth_occlusion_resolution);
    int max_azimuth = static_cast<int>(tl_ray.azimuth + azimuth_occlusion_resolution);
    int min_elevation = static_cast<int>(tl_ray.elevation - elevation_occlusion_resolution);
    int max_elevation = static_cast<int>(tl_ray.elevation + elevation_occlusion_resolution);
    /**
     * search among lidar rays whose azimuth and elevation angle are close to the tl_ray.
     * for a lidar ray r1 whose azimuth and elevation are very close to tl_pt,
     * and the distance from r1 to camera is smaller than the distance from tl_pt to camera,
     * then tl_pt is occluded by r1.
     */
    for (int azimuth = min_azimuth; (azimuth <= max_azimuth) && !occluded; azimuth++) {
      for (int elevation = min_elevation; (elevation <= max_elevation) && !occluded; elevation++) {
        for (const Ray & lidar_ray : lidar_rays_[azimuth][elevation]) {
          if (
            std::abs(lidar_ray.azimuth - tl_ray.azimuth) <= azimuth_occlusion_resolution &&
            std::abs(lidar_ray.elevation - tl_ray.elevation) <= elevation_occlusion_resolution &&
            lidar_ray.dist < tl_ray.dist - min_dist_from_occlusion_to_tl) {
            occluded = true;
            break;
          }
        }
      }
    }
    occluded_num += occluded;
  }
  return 100 * occluded_num / tl_sample_cloud.size();
}

float CloudOcclusionPredictor::getCloudDelay() { return static_cast<float>(cloud_delay_); }

}  // namespace traffic_light
