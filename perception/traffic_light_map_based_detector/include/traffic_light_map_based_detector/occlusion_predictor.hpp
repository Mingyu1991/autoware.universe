/*
 * Copyright 2023 Autoware Foundation. All rights reserved.
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

#ifndef TRAFFIC_LIGHT_MAP_BASED_DETECTOR__OCCLUSION_PREDICTOR_HPP_
#define TRAFFIC_LIGHT_MAP_BASED_DETECTOR__OCCLUSION_PREDICTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/pcl_conversion.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <list>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace traffic_light
{

struct Ray
{
  float azimuth;
  float elevation;
  float dist;
  // only for debug
  pcl::PointXYZ pt;
};

class CloudOcclusionPredictor
{
public:
  void receivePointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void update(
    const sensor_msgs::msg::CameraInfo & camera_info, const tf2_ros::Buffer & tf_buffer,
    const std::vector<autoware_auto_perception_msgs::msg::TrafficLightRoi> & tl_rough_rois);

  uint32_t predict(
    const geometry_msgs::msg::Point & roi_top_left,
    const geometry_msgs::msg::Point & roi_bottom_right, double azimuth_occlusion_resolution,
    double elevation_occlusion_resolution);

  float getCloudDelay();

  sensor_msgs::msg::PointCloud2 debug(const sensor_msgs::msg::CameraInfo & camera_info);
  sensor_msgs::msg::PointCloud2 cloud_camera_stamp_;

private:
  void cloudPreprocess(
    const sensor_msgs::msg::CameraInfo & camera_info,
    const std::vector<autoware_auto_perception_msgs::msg::TrafficLightRoi> & tl_rough_rois);

  void filterCloud(
    const pcl::PointCloud<pcl::PointXYZ> & cloud_in,
    const std::vector<autoware_auto_perception_msgs::msg::TrafficLightRoi> & tl_rough_rois,
    const sensor_msgs::msg::CameraInfo & camera_info, pcl::PointCloud<pcl::PointXYZ> & cloud_out);

  void sampleTrafficLightRoi(
    const geometry_msgs::msg::Point & top_left, const geometry_msgs::msg::Point & bottom_right,
    uint32_t horizontal_sample_num, uint32_t vertical_sample_num,
    pcl::PointCloud<pcl::PointXYZ> & cloud_out);

  std::list<sensor_msgs::msg::PointCloud2> history_clouds_;
  pcl::PointCloud<pcl::PointXYZ> debug_cloud_;
  Eigen::Matrix4d camera2cloud_;
  image_geometry::PinholeCameraModel pinhole_camera_model_;
  std::map<int, std::map<int, std::vector<Ray> > > lidar_rays_;
  double cloud_delay_;
};

}  // namespace traffic_light

#endif  // TRAFFIC_LIGHT_MAP_BASED_DETECTOR__OCCLUSION_PREDICTOR_HPP_
