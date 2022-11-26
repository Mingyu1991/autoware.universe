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
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 */

#include "map_loader/surrounding_pointcloud_map_loader_node.hpp"

#include <glob.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
bool isPcdFile(const std::string & p)
{
  if (fs::is_directory(p)) {
    return false;
  }

  const std::string ext = fs::path(p).extension();

  if (ext != ".pcd" && ext != ".PCD") {
    return false;
  }

  return true;
}
}  // namespace

SurroundingPointCloudMapLoaderNode::SurroundingPointCloudMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("surrounding_pointcloud_map_loader", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_pointcloud_map_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/map/surrounding_pointcloud_map", durable_qos);

  const auto pcd_paths_or_directory =
    declare_parameter("pcd_paths_or_directory", std::vector<std::string>({}));

  std::vector<std::string> pcd_paths{};

  for (const auto & p : pcd_paths_or_directory) {
    if (!fs::exists(p)) {
      RCLCPP_ERROR_STREAM(get_logger(), "invalid path: " << p);
    }

    if (isPcdFile(p)) {
      pcd_paths.push_back(p);
    }

    if (fs::is_directory(p)) {
      for (const auto & file : fs::directory_iterator(p)) {
        const auto filename = file.path().string();
        if (isPcdFile(filename)) {
          pcd_paths.push_back(filename);
        }
      }
    }
  }

  loadPCDFiles(pcd_paths);

  if (map_cloud_.size() == 0) {
    RCLCPP_ERROR(get_logger(), "No PCD was loaded: pcd_paths.size() = %zu", pcd_paths.size());
    return;
  }
  RCLCPP_INFO_STREAM(get_logger(), "load PCD with " << map_cloud_.size() << " points");

  timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SurroundingPointCloudMapLoaderNode::timerCallback, this));
}

void SurroundingPointCloudMapLoaderNode::loadPCDFiles(const std::vector<std::string> & pcd_paths)
{
  map_cloud_.clear();
  sensor_msgs::msg::PointCloud2 partial_pcd;
  pcl::PointCloud<pcl::PointXYZ> partial_pcl;
  for (const auto & path : pcd_paths) {
    if (pcl::io::loadPCDFile(path, partial_pcd) == -1) {
      RCLCPP_ERROR_STREAM(get_logger(), "PCD load failed: " << path);
    }
    pcl::fromROSMsg(partial_pcd, partial_pcl);
    map_cloud_ += partial_pcl;
  }
}

void SurroundingPointCloudMapLoaderNode::timerCallback()
{
  geometry_msgs::msg::TransformStamped transform;
  try{
    transform = tf_buffer_.lookupTransform("map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.2));
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    for(const auto & pt : map_cloud_){
      geometry_msgs::msg::Vector3 p1 = transform.transform.translation;
      float dist = (p1.x - pt.x) * (p1.x - pt.x) + (p1.y - pt.y) * (p1.y - pt.y) + (p1.z - pt.z) * (p1.z - pt.z);
      if(dist <= 250 * 250 && pt.z >= 10){
        pcl_cloud.push_back(pt);
      }
    }
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = "map";
    ros_cloud.header.stamp = transform.header.stamp;
    pub_pointcloud_map_->publish(ros_cloud);
  }
  catch(tf2::TransformException& ex){
    RCLCPP_WARN(get_logger(), "failed to retrieve tf");
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SurroundingPointCloudMapLoaderNode)
