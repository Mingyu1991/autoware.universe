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

#include "traffic_light_occlusion_predictor/node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace traffic_light
{

TrafficLightOcclusionPredictorNodelet::TrafficLightOcclusionPredictorNodelet(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_occlusion_predictor_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // subscribers
  point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/cloud", rclcpp::SensorDataQoS(),
    std::bind(&CloudOcclusionPredictor::pointCloudCallback, &this->cloud_occlusion_predictor_, _1));
  objects_sub_ = create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
    "~/input/objects", rclcpp::SensorDataQoS(),
    std::bind(&CloudOcclusionPredictor::perceptionObjectsCallback, &this->cloud_occlusion_predictor_, _1));
  
  sync_.reset(new Sync(SyncPolicy(10), camera_info_sub_, roi_sub_));
  sync_->registerCallback(std::bind(&TrafficLightOcclusionPredictorNodelet::callback, this, _1, _2));
  camera_info_sub_.subscribe(this, "~/input/camera_info", rmw_qos_profile_sensor_data);
  roi_sub_.subscribe(this, "~/input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
  // publishers
  debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/traffic_light/debug_cloud_camera_stamp", 1);
  roi_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>("~/output/rois", 1);
  config_.azimuth_occlusion_resolution = declare_parameter<double>("azimuth_occlusion_resolution", 0.1);
  config_.elevation_occlusion_resolution = declare_parameter<double>("elevation_occlusion_resolution", 0.1);
}

void TrafficLightOcclusionPredictorNodelet::callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr in_image_info_msg,
    const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr in_roi_msg)
{  
  autoware_auto_perception_msgs::msg::TrafficLightRoiArray output_msg = * in_roi_msg;
  cloud_occlusion_predictor_.update(*in_image_info_msg, tf_buffer_, output_msg.rois);
  for(auto& roi : output_msg.rois){
    roi.occlusion_num = cloud_occlusion_predictor_.predict(roi, config_.azimuth_occlusion_resolution, config_.elevation_occlusion_resolution);
  }
  roi_pub_->publish(output_msg);
  debug_cloud_pub_->publish(cloud_occlusion_predictor_.debug(*in_image_info_msg));
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightOcclusionPredictorNodelet)
