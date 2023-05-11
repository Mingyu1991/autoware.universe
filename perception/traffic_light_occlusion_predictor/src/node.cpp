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

#include "traffic_light_occlusion_predictor/node.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/traffic_light_roi.hpp>

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
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

TrafficLightOcclusionPredictorNodelet::TrafficLightOcclusionPredictorNodelet(
  const rclcpp::NodeOptions & node_options)
: Node("traffic_light_occlusion_predictor_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  // subscribers
  point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/cloud", rclcpp::SensorDataQoS(),
    std::bind(&TrafficLightOcclusionPredictorNodelet::pointCloudCallback, this, _1));

  sync_.reset(new Sync(SyncPolicy(10), camera_info_sub_, roi_sub_));
  sync_->registerCallback(
    std::bind(&TrafficLightOcclusionPredictorNodelet::callback, this, _1, _2));
  camera_info_sub_.subscribe(this, "~/input/camera_info", rmw_qos_profile_sensor_data);
  roi_sub_.subscribe(this, "~/input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightOcclusionPredictorNodelet::mapCallback, this, _1));

  // publishers
  debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/traffic_light/debug_cloud_camera_stamp", 1);
  roi_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>(
    "~/output/rois", 1);
  // configuration parameters
  config_.azimuth_occlusion_resolution =
    declare_parameter<double>("azimuth_occlusion_resolution", 0.1);
  config_.elevation_occlusion_resolution =
    declare_parameter<double>("elevation_occlusion_resolution", 0.1);
  config_.max_valid_pt_dist = declare_parameter<double>("max_valid_pt_dist", 50.0);
  config_.min_cloud_size = declare_parameter<int>("min_cloud_size", 100000);

  cloud_occlusion_predictor_ = std::make_shared<CloudOcclusionPredictor>(
    config_.max_valid_pt_dist, config_.azimuth_occlusion_resolution,
    config_.elevation_occlusion_resolution);
  point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/cloud", rclcpp::SensorDataQoS(),
    std::bind(&TrafficLightOcclusionPredictorNodelet::pointCloudCallback, this, _1));
}

void TrafficLightOcclusionPredictorNodelet::callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr in_camera_info_msg,
  const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr in_roi_msg)
{
  (void)in_camera_info_msg;
  (void)in_roi_msg;
  // autoware_auto_perception_msgs::msg::TrafficLightRoiArray output_msg = * in_roi_msg;
  // cloud_occlusion_predictor_.update(*in_image_info_msg, tf_buffer_, output_msg.rois);
  // for(auto& roi : output_msg.rois){
  //   roi.occlusion_num = cloud_occlusion_predictor_.predict(roi,
  //   config_.azimuth_occlusion_resolution, config_.elevation_occlusion_resolution);
  // }
  // roi_pub_->publish(output_msg);
  // debug_cloud_pub_->publish(cloud_occlusion_predictor_.debug(*in_image_info_msg));
}

void TrafficLightOcclusionPredictorNodelet::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_msg)
{
  all_traffic_lights_map_.clear();
  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();

  lanelet::utils::conversion::fromBinMsg(*input_msg, lanelet_map_ptr);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr);
  std::vector<lanelet::AutowareTrafficLightConstPtr> all_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (auto tl_itr = all_lanelet_traffic_lights.begin(); tl_itr != all_lanelet_traffic_lights.end();
       ++tl_itr) {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (auto lsp : lights) {
      if (!lsp.isLineString()) {  // traffic lights must be linestrings
        continue;
      }
      all_traffic_lights_map_[lsp.id()] = static_cast<lanelet::ConstLineString3d>(lsp);
    }
  }
}

void TrafficLightOcclusionPredictorNodelet::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // sometimes the top lidar doesn't publish any point and necessary to filter it out
  int cloud_size = msg->width * msg->height;
  if (cloud_size >= config_.min_cloud_size) {
    cloud_occlusion_predictor_->receivePointCloud(msg);
  }
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightOcclusionPredictorNodelet)
