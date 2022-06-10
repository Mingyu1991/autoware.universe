// Copyright 2022 TIER IV, Inc.
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

#include "radar_tracks_msgs_converter/radar_tracks_msgs_converter_node.hpp"

#include <tf2/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>
#include <string>
#include <vector>

using namespace std::literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::placeholders::_1;

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}
}  // namespace

namespace radar_tracks_msgs_converter
{
RadarTracksMsgsConverterNode::RadarTracksMsgsConverterNode(const rclcpp::NodeOptions & node_options)
: Node("radar_tracks_msgs_converter", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarTracksMsgsConverterNode::onSetParam, this, _1));

  // Node Parameter
  node_param_.update_rate_hz = declare_parameter<double>("update_rate_hz", 20.0);
  node_param_.new_frame_id = declare_parameter<std::string>("new_frame_id", "base_link");
  node_param_.use_twist_compensation = declare_parameter<bool>("use_twist_compensation", false);

  // Subscriber
  sub_radar_ = create_subscription<RadarTracks>(
    "~/input/radar_objects", rclcpp::QoS{1},
    std::bind(&RadarTracksMsgsConverterNode::onRadarTracks, this, _1));
  sub_twist_ = create_subscription<TwistStamped>(
    "~/input/twist", rclcpp::QoS{1}, std::bind(&RadarTracksMsgsConverterNode::onTwist, this, _1));
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  // Publisher
  pub_radar_ = create_publisher<TrackedObjects>("~/output/radar_objects", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(node_param_.update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&RadarTracksMsgsConverterNode::onTimer, this));
}

void RadarTracksMsgsConverterNode::onRadarTracks(const RadarTracks::ConstSharedPtr msg)
{
  radar_data_ = msg;
}

void RadarTracksMsgsConverterNode::onTwist(const TwistStamped::ConstSharedPtr msg)
{
  twist_data_ = msg;
}

rcl_interfaces::msg::SetParametersResult RadarTracksMsgsConverterNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      // Copy to local variable
      auto p = node_param_;

      // Update params
      update_param(params, "update_rate_hz", p.update_rate_hz);
      update_param(params, "use_twist_compensation", p.use_twist_compensation);

      // Copy back to member variable
      node_param_ = p;
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  result.successful = true;
  result.reason = "success";
  return result;
}

bool RadarTracksMsgsConverterNode::isDataReady()
{
  if (!radar_data_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for radar msg...");
    return false;
  }

  return true;
}

void RadarTracksMsgsConverterNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }
  const auto & header = radar_data_->header;
  transform_ = transform_listener_->getTransform(
    node_param_.new_frame_id, header.frame_id, header.stamp, rclcpp::Duration::from_seconds(0.01));

  TrackedObjects tracked_objects = convertRadarTrackToTrackedObjects();
  if (!tracked_objects.objects.empty()) {
    pub_radar_->publish(tracked_objects);
  }
}

TrackedObjects RadarTracksMsgsConverterNode::convertRadarTrackToTrackedObjects()
{
  TrackedObjects tracked_objects;
  tracked_objects.header = radar_data_->header;
  tracked_objects.header.frame_id = node_param_.new_frame_id;

  for (auto & radar_track : radar_data_->tracks) {
    TrackedObject tracked_object;

    tracked_object.object_id = radar_track.uuid;
    tracked_object.existence_probability = 1.0;

    tracked_object.shape.type = Shape::BOUNDING_BOX;
    tracked_object.shape.dimensions = radar_track.size;

    // kinematics
    TrackedObjectKinematics kinematics;
    kinematics.orientation_availability = TrackedObjectKinematics::AVAILABLE;
    kinematics.is_stationary = false;

    // convert by tf
    geometry_msgs::msg::PoseStamped radar_pose_stamped{};
    radar_pose_stamped.pose.position = radar_track.position;
    geometry_msgs::msg::PoseStamped transformed_pose_stamped{};
    tf2::doTransform(radar_pose_stamped, transformed_pose_stamped, *transform_);
    kinematics.pose_with_covariance.pose = transformed_pose_stamped.pose;

    kinematics.pose_with_covariance.covariance[0] = radar_track.position_covariance[0];
    kinematics.pose_with_covariance.covariance[1] = radar_track.position_covariance[1];
    kinematics.pose_with_covariance.covariance[2] = radar_track.position_covariance[2];
    kinematics.pose_with_covariance.covariance[6] = radar_track.position_covariance[1];
    kinematics.pose_with_covariance.covariance[7] = radar_track.position_covariance[3];
    kinematics.pose_with_covariance.covariance[8] = radar_track.position_covariance[4];
    kinematics.pose_with_covariance.covariance[12] = radar_track.position_covariance[2];
    kinematics.pose_with_covariance.covariance[13] = radar_track.position_covariance[4];
    kinematics.pose_with_covariance.covariance[14] = radar_track.position_covariance[5];

    // convert by tf
    geometry_msgs::msg::Vector3Stamped radar_velocity_stamped{};
    radar_velocity_stamped.vector = radar_track.velocity;
    geometry_msgs::msg::Vector3Stamped transformed_vector3_stamped{};
    tf2::doTransform(radar_velocity_stamped, transformed_vector3_stamped, *transform_);
    kinematics.twist_with_covariance.twist.linear = transformed_vector3_stamped.vector;

    // twist compensation
    if (node_param_.use_twist_compensation) {
      if (twist_data_) {
        kinematics.twist_with_covariance.twist.linear.x += twist_data_->twist.linear.x;
        kinematics.twist_with_covariance.twist.linear.y += twist_data_->twist.linear.y;
        kinematics.twist_with_covariance.twist.linear.z += twist_data_->twist.linear.z;
      } else {
        RCLCPP_INFO(get_logger(), "Twist data is not coming");
      }
    }

    kinematics.twist_with_covariance.covariance[0] = radar_track.velocity_covariance[0];
    kinematics.twist_with_covariance.covariance[1] = radar_track.velocity_covariance[1];
    kinematics.twist_with_covariance.covariance[2] = radar_track.velocity_covariance[2];
    kinematics.twist_with_covariance.covariance[6] = radar_track.velocity_covariance[1];
    kinematics.twist_with_covariance.covariance[7] = radar_track.velocity_covariance[3];
    kinematics.twist_with_covariance.covariance[8] = radar_track.velocity_covariance[4];
    kinematics.twist_with_covariance.covariance[12] = radar_track.velocity_covariance[2];
    kinematics.twist_with_covariance.covariance[13] = radar_track.velocity_covariance[4];
    kinematics.twist_with_covariance.covariance[14] = radar_track.velocity_covariance[5];

    kinematics.acceleration_with_covariance.covariance[0] = radar_track.acceleration_covariance[0];
    kinematics.acceleration_with_covariance.covariance[1] = radar_track.acceleration_covariance[1];
    kinematics.acceleration_with_covariance.covariance[2] = radar_track.acceleration_covariance[2];
    kinematics.acceleration_with_covariance.covariance[6] = radar_track.acceleration_covariance[1];
    kinematics.acceleration_with_covariance.covariance[7] = radar_track.acceleration_covariance[3];
    kinematics.acceleration_with_covariance.covariance[8] = radar_track.acceleration_covariance[4];
    kinematics.acceleration_with_covariance.covariance[12] = radar_track.acceleration_covariance[2];
    kinematics.acceleration_with_covariance.covariance[13] = radar_track.acceleration_covariance[4];
    kinematics.acceleration_with_covariance.covariance[14] = radar_track.acceleration_covariance[5];

    tracked_object.kinematics = kinematics;

    // classification
    ObjectClassification classification;
    classification.probability = 1.0;
    classification.label = convertClassification(radar_track.classification);
    tracked_object.classification.emplace_back(classification);

    tracked_objects.objects.emplace_back(tracked_object);
  }
  return tracked_objects;
}

uint8_t RadarTracksMsgsConverterNode::convertClassification(const uint16_t classification)
{
  if (classification == 32000) {
    return ObjectClassification::UNKNOWN;
  } else if (classification == 32001) {
    return ObjectClassification::CAR;
  } else if (classification == 32002) {
    return ObjectClassification::TRUCK;
  } else if (classification == 32003) {
    return ObjectClassification::BUS;
  } else if (classification == 32004) {
    return ObjectClassification::TRAILER;
  } else if (classification == 32005) {
    return ObjectClassification::MOTORCYCLE;
  } else if (classification == 32006) {
    return ObjectClassification::BICYCLE;
  } else if (classification == 32007) {
    return ObjectClassification::PEDESTRIAN;
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Receive unknown label for RadarTracks");
    return ObjectClassification::UNKNOWN;
  }
}

}  // namespace radar_tracks_msgs_converter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_tracks_msgs_converter::RadarTracksMsgsConverterNode)
