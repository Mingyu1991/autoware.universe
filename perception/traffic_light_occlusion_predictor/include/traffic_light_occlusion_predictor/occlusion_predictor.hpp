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
#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/pcl_conversion.hpp>
#include <geometry/bounding_box_3d.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <map>
#include <set>


namespace traffic_light
{

std::string uuid2str(const autoware_auto_perception_msgs::msg::PredictedObject::_object_id_type& uuid);
/**
 * defined for storing object status.
 * The reason for not using autoware_auto_perception_msgs::msg::PredictedObject is that
 * it can't store timestamp information
*/
struct ObjectStatus
{
  double stamp;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  autoware_auto_perception_msgs::msg::Shape shape;
};
/**
 * class for tracking and predicting single object status
*/
class SingleObjectPredictor
{
public:
  /**
   * @brief receive the newest object status
   * 
   * @param obj    object status
   * @param stamp  the timestamp of this status
   */
  void update(const autoware_auto_perception_msgs::msg::PredictedObject& obj, double stamp);
  /**
   * @brief predict the status of the object at the given timestamp
   * 
   * @param stamp  the specified stamp when the object status be predicted
   * @param object the predicted object status at stamp
   * @return true: prediction succeed
   * @return false: prediction failed
   */
  bool predict(double stamp, ObjectStatus& object) const;
private:
  /**
   * @brief the object status saved in increasing order by timestamp
   */
  std::vector<ObjectStatus> status_;
};
/**
 * @brief class for tracking and predicting multiple object status
 * 
 */
class ObjectsPredictor
{
public:
  /**
   * @brief receive the newest objects status
   * 
   * @param msg  object message retrieved from topic "/perception/object_recognition/objects"
   */
  void update(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);
  /**
   * @brief predict the status of the tracked objects at given timestamp
   * 
   * @param stamp the specified stamp when the objects status be predicted
   * @return predicted objects at stamp
   */
  autoware_auto_perception_msgs::msg::PredictedObjects predict(const std_msgs::msg::Header::_stamp_type& stamp);
private:
  using id_type = autoware_auto_perception_msgs::msg::PredictedObject::_object_id_type;
  /**
   * @brief tracked objects uniquely defined by string converted from uuid
   * 
   */
  std::map<std::string, SingleObjectPredictor> tracked_objects_;
};

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

  void receivePerceptionObjects(
    const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);

  void update(
    const sensor_msgs::msg::CameraInfo& camera_info, 
    const tf2_ros::Buffer& tf_buffer,
    const std::vector<autoware_auto_perception_msgs::msg::TrafficLightRoi>& tl_rough_rois,
    bool enable_point_cloud_compensation);

  uint32_t predict(
    const autoware_auto_perception_msgs::msg::TrafficLightRoi& roi,
    double azimuth_occlusion_resolution,
    double elevation_occlusion_resolution
  );

  float getCloudDelay();

  sensor_msgs::msg::PointCloud2 debug(const sensor_msgs::msg::CameraInfo& camera_info);
  sensor_msgs::msg::PointCloud2 cloud_camera_stamp_;
private:
  void compensateObjectMovements(
    pcl::PointCloud<pcl::PointXYZ>& cloud, 
    const sensor_msgs::msg::CameraInfo& camera_info);

  void cloudPreprocess(
    const sensor_msgs::msg::CameraInfo& camera_info,
    const std::vector<autoware_auto_perception_msgs::msg::TrafficLightRoi>& tl_rough_rois,
    bool enable_point_cloud_compensation);

  void filterCloud(
    const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
    const std::vector<autoware_auto_perception_msgs::msg::TrafficLightRoi>& tl_rough_rois,
    const sensor_msgs::msg::CameraInfo& camera_info,
    bool enable_point_cloud_compensation,
    pcl::PointCloud<pcl::PointXYZ>& cloud_out);

  void sampleTrafficLightRoi(
    const geometry_msgs::msg::Vector3& roi_top_left,
    const geometry_msgs::msg::Vector3& roi_bottom_right,
    uint32_t horizontal_sample_num, 
    uint32_t vertical_sample_num,
    pcl::PointCloud<pcl::PointXYZ>& cloud_out);
  
  void sampleTrafficLightRoi(
    const autoware_auto_perception_msgs::msg::TrafficLightRoi& roi,
    uint32_t horizontal_sample_num, 
    uint32_t vertical_sample_num,
    pcl::PointCloud<pcl::PointXYZ>& cloud_out);

  std::list<sensor_msgs::msg::PointCloud2> history_clouds_;
  pcl::PointCloud<pcl::PointXYZ> debug_cloud_;
  Eigen::Matrix4d map2cloud_;
  Eigen::Matrix4d camera2map_;
  Eigen::Matrix4d camera2cloud_;
  image_geometry::PinholeCameraModel pinhole_camera_model_;
  std::map<int, std::map<int, std::vector<Ray> > > lidar_rays_;
  ObjectsPredictor objects_predictor_;
  double cloud_delay_;
};

}

#endif