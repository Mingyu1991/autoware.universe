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

#ifndef TRAFFIC_LIGHT_MAP_BASED_DETECTOR__OCCLUSION_PREDICTOR_HPP_
#define TRAFFIC_LIGHT_MAP_BASED_DETECTOR__OCCLUSION_PREDICTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <map>
#include <set>


namespace traffic_light
{

std::string uuid2str(const autoware_auto_perception_msgs::msg::PredictedObject::_object_id_type& uuid);

struct OcclusionObjectStatus
{
  double stamp;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
};

struct OcclusionObject
{
  double stamp;
  geometry_msgs::msg::Pose pose;
  autoware_auto_perception_msgs::msg::Shape shape;
};

class OcclusionObjectStatusSequence
{
public:  
  void update(const autoware_auto_perception_msgs::msg::PredictedObject& obj, double stamp);
  bool predict(double stamp, OcclusionObject& object) const;
private:
  std::vector<OcclusionObjectStatus> status;
  autoware_auto_perception_msgs::msg::Shape shape;
};

class OcclusionPredictor
{
public:
  void update(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);

  autoware_auto_perception_msgs::msg::PredictedObjects predict(const std_msgs::msg::Header::_stamp_type& stamp);
private:
  using id_type = autoware_auto_perception_msgs::msg::PredictedObject::_object_id_type;
  std::map<std::string, OcclusionObjectStatusSequence> tracked_objects;
};

}

#endif