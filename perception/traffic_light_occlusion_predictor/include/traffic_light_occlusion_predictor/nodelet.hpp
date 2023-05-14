// Copyright 2023 Tier IV, Inc.
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

#ifndef TRAFFIC_LIGHT_OCCLUSION_PREDICTOR__NODELET_HPP_
#define TRAFFIC_LIGHT_OCCLUSION_PREDICTOR__NODELET_HPP_

#include <perception_utils/traffic_light_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_light_occlusion_predictor/occlusion_predictor.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <mutex>

namespace traffic_light
{
class TrafficLightOcclusionPredictorNodelet : public rclcpp::Node
{
public:
  explicit TrafficLightOcclusionPredictorNodelet(const rclcpp::NodeOptions & node_options);

private:
  struct Config
  {
    double azimuth_occlusion_resolution;
    double elevation_occlusion_resolution;
    double max_valid_pt_dist;
    int occlusion_thres;
    int min_cloud_size;
  };

private:
  /**
   * @brief
   * calculate the occlusion probability of each rough roi and publish.
   *
   * @param in_image_msg
   * @param in_roi_msg
   */

  /**
   * @brief receive camera_info, rois, traffic signals and calculate the occlusion ratios of the
   * rois. If the occlusion ratio of a rois is larger than the threshold, set the corresponding
   * traffic signal as UNKNOWN
   *
   * @param in_image_msg
   * @param in_roi_msg
   * @param in_signal_msg
   */
  void callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr in_image_msg,
    const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr in_roi_msg,
    const autoware_auto_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr in_signal_msg);
  /**
   * @brief receive the lanelet2 map
   *
   * @param input_msg
   */
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_msg);
  /**
   * @brief receive the point cloud for calculating the occlusion
   *
   * @param msg
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  /**
   * @brief subscribers
   *
   */
  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::CameraInfo, autoware_auto_perception_msgs::msg::TrafficLightRoiArray,
    autoware_auto_perception_msgs::msg::TrafficSignalArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  message_filters::Subscriber<autoware_auto_perception_msgs::msg::TrafficLightRoiArray> roi_sub_;
  message_filters::Subscriber<autoware_auto_perception_msgs::msg::TrafficSignalArray> signal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  /**
   * @brief publishers
   *
   */
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>::SharedPtr signal_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::map<lanelet::Id, tf2::Vector3> traffic_light_position_map_;
  Config config_;
  /**
   * @brief main class for calculating the occlusion probability
   *
   */
  std::shared_ptr<CloudOcclusionPredictor> cloud_occlusion_predictor_;
};
}  // namespace traffic_light
#endif  // TRAFFIC_LIGHT_OCCLUSION_PREDICTOR__NODELET_HPP_
