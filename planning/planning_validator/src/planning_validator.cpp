// Copyright 2022 Tier IV, Inc.
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

#include "planning_validator/planning_validator.hpp"

#include "planning_validator/utils.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <memory>
#include <string>
#include <utility>

namespace planning_validator
{
using diagnostic_msgs::msg::DiagnosticStatus;

PlanningValidator::PlanningValidator(const rclcpp::NodeOptions & options)
: Node("planning_validator", options)
{
  using std::placeholders::_1;

  sub_kinematics_ = create_subscription<Odometry>(
    "~/input/kinematics", 1,
    [this](const Odometry::ConstSharedPtr msg) { current_kinematics_ = msg; });
  sub_traj_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, std::bind(&PlanningValidator::onTrajectory, this, _1));

  pub_traj_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_status_ = create_publisher<PlanningValidatorStatus>("~/output/validation_status", 1);

  debugger_ = std::make_shared<PlanningValidatorDebugPosePublisher>(this);

  setupParameters();
}

void PlanningValidator::setupParameters()
{
  const auto dp = [this](const auto & s) {
    try {
      return declare_parameter<double>(s);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "failed to declare param: %s. msg: %s", s, e.what());
      std::exit(1);
    }
  };
  auto & p = validation_params_;
  p.interval_threshold = dp("interval_threshold");
  p.relative_angle_threshold = dp("relative_angle_threshold");
  p.curvature_threshold = dp("curvature_threshold");
  p.lateral_acc_threshold = dp("lateral_acc_threshold");
  p.longitudinal_max_acc_threshold = dp("longitudinal_max_acc_threshold");
  p.longitudinal_min_acc_threshold = dp("longitudinal_min_acc_threshold");
  p.steering_threshold = dp("steering_threshold");
  p.steering_rate_threshold = dp("steering_rate_threshold");
  p.velocity_deviation_threshold = dp("velocity_deviation_threshold");
  p.distance_deviation_threshold = dp("distance_deviation_threshold");

  try {
    vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "failed to get vehicle info. use default value.");
    vehicle_info_.wheel_base_m = 4.0;
  }
}

void PlanningValidator::setupDiag()
{
  diag_updater_.setHardwareID("planning_validator");

  const auto & s = validation_status_;
  setDiagStatus("finite", s.is_valid_finite_value, "infinite value is found");
  setDiagStatus("interval", s.is_valid_interval, "points interval is too long");
  setDiagStatus("relative_angle", s.is_valid_relative_angle, "relative angle is too large");
  setDiagStatus("curvature", s.is_valid_curvature, "curvature is too large");
  setDiagStatus(
    "lateral_acceleration", s.is_valid_lateral_acc, "lateral acceleration is too large");
  setDiagStatus("acceleration", s.is_valid_longitudinal_max_acc, "acceleration is too large");
  setDiagStatus("deceleration", s.is_valid_longitudinal_min_acc, "deceleration is too large");
  setDiagStatus("steering", s.is_valid_steering, "expected steering is too large");
  setDiagStatus("steering_rate", s.is_valid_steering_rate, "expected steering rate is too large");
  setDiagStatus(
    "velocity_deviation", s.is_valid_velocity_deviation,
    "velocity deviation between planning and actual is too large");
}

void PlanningValidator::setDiagStatus(
  const std::string & name, const bool is_ok, const std::string & error_msg)
{
  diag_updater_.add(
    "trajectory_validation_" + name, [this, is_ok, &error_msg](DiagnosticStatusWrapper & stat) {
      is_ok ? stat.summary(DiagnosticStatus::OK, "ok")
            : stat.summary(DiagnosticStatus::ERROR, error_msg);
    });
}

bool PlanningValidator::isDataReady()
{
  const auto waiting = [this](const auto s) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", s);
    return false;
  };

  if (!current_kinematics_) {
    return waiting("current_kinematics_");
  }
  if (!current_trajectory_) {
    return waiting("current_trajectory_");
  }
  return true;
}

void PlanningValidator::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  current_trajectory_ = msg;

  if (!isDataReady()) return;

  validate(*current_trajectory_);

  diag_updater_.force_update();

  publishTrajectory();

  publishDebugInfo();
}

void PlanningValidator::publishTrajectory()
{
  // Validation check is all green. Publish the trajectory.
  if (isAllValid(validation_status_)) {
    pub_traj_->publish(*current_trajectory_);
    previous_published_trajectory_ = current_trajectory_;
    return;
  }

  // invalid factor is found. Publish previous trajectory.
  if (previous_published_trajectory_) {
    pub_traj_->publish(*previous_published_trajectory_);
    RCLCPP_ERROR(get_logger(), "Invalid Trajectory is detected. Use previous trajectory.");
    return;
  }

  // current_path is invalid and no previous trajectory available.
  // trajectory is not published.
  RCLCPP_ERROR(get_logger(), "Invalid Trajectory is detected. Trajectory is not published.");
  return;
}

void PlanningValidator::publishDebugInfo()
{
  validation_status_.stamp = get_clock()->now();
  pub_status_->publish(validation_status_);
  debugger_->publish();
}

void PlanningValidator::validate(const Trajectory & trajectory)
{
  auto & s = validation_status_;

  s.is_valid_finite_value = checkValidFiniteValue(trajectory);
  s.is_valid_interval = checkValidInterval(trajectory);
  s.is_valid_lateral_acc = checkValidLateralAcceleration(trajectory);
  s.is_valid_longitudinal_max_acc = checkValidMaxLongitudinalAcceleration(trajectory);
  s.is_valid_longitudinal_min_acc = checkValidMinLongitudinalAcceleration(trajectory);
  s.is_valid_velocity_deviation = checkValidVelocityDeviation(trajectory);
  s.is_valid_distance_deviation = checkValidDistanceDeviation(trajectory);

  // use resampled trajectory because the following metrics can not be evaluated for closed points.
  // Note: do not interpolate to keep original trajectory shape.
  constexpr auto min_interval = 1.0;
  const auto resampled = resampleTrajectory(trajectory, min_interval);

  s.is_valid_relative_angle = checkValidRelativeAngle(resampled);
  s.is_valid_curvature = checkValidCurvature(resampled);
  s.is_valid_steering = checkValidSteering(resampled);
  s.is_valid_steering_rate = checkValidSteeringRate(resampled);
}

bool PlanningValidator::checkValidFiniteValue(const Trajectory & trajectory)
{
  for (const auto & p : trajectory.points) {
    if (!checkFinite(p)) return false;
  }
  return true;
}

bool PlanningValidator::checkValidInterval(const Trajectory & trajectory)
{
  // // debug_marker_.clearPoseMarker("trajectory_interval");

  const auto interval_distances = calcIntervalDistance(trajectory);
  const auto [max_interval_distance, i] = getAbsMaxValAndIdx(interval_distances);
  validation_status_.max_interval_distance = max_interval_distance;

  if (max_interval_distance > validation_params_.interval_threshold) {
    // const auto &p = trajectory.points;
    // debug_marker_.pushPoseMarker(p.at(i - 1), "trajectory_interval");
    // debug_marker_.pushPoseMarker(p.at(i), "trajectory_interval");
    return false;
  }

  return true;
}

bool PlanningValidator::checkValidRelativeAngle(const Trajectory & trajectory)
{
  // debug_marker_.clearPoseMarker("trajectory_relative_angle");

  const auto relative_angles = calcRelativeAngles(trajectory);
  const auto [max_relative_angle, i] = getAbsMaxValAndIdx(relative_angles);
  validation_status_.max_relative_angle = max_relative_angle;

  if (max_relative_angle > validation_params_.relative_angle_threshold) {
    // const auto &p = trajectory.points;
    // debug_marker_.pushPoseMarker(p.at(i), "trajectory_relative_angle", 0);
    // debug_marker_.pushPoseMarker(p.at(i + 1), "trajectory_relative_angle", 1);
    // debug_marker_.pushPoseMarker(p.at(i + 2), "trajectory_relative_angle", 2);
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidCurvature(const Trajectory & trajectory)
{
  // // debug_marker.clearPoseMarker("trajectory_curvature");

  const auto curvatures = calcCurvature(trajectory);
  const auto [max_curvature, i] = getAbsMaxValAndIdx(curvatures);
  validation_status_.max_curvature = max_curvature;
  if (max_curvature > validation_params_.curvature_threshold) {
    // const auto &p = trajectory.points;
    // debug_marker_.pushPoseMarker(p.at(i - 1), "trajectory_curvature");
    // debug_marker_.pushPoseMarker(p.at(i), "trajectory_curvature");
    // debug_marker_.pushPoseMarker(p.at(i + 1), "trajectory_curvature");
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidLateralAcceleration(const Trajectory & trajectory)
{
  const auto lateral_acc_arr = calcLateralAcceleration(trajectory);
  const auto [max_lateral_acc, i] = getAbsMaxValAndIdx(lateral_acc_arr);
  validation_status_.max_lateral_acc = max_lateral_acc;
  if (max_lateral_acc > validation_params_.lateral_acc_threshold) {
    // debug_marker_.pushPoseMarker(trajectory.points.at(i), "lateral_acceleration");
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidMinLongitudinalAcceleration(const Trajectory & trajectory)
{
  const auto acc_arr = getLongitudinalAccArray(trajectory);
  const auto [min_longitudinal_acc, i_min] = getMinValAndIdx(acc_arr);
  validation_status_.min_longitudinal_acc = min_longitudinal_acc;

  if (min_longitudinal_acc < validation_params_.longitudinal_min_acc_threshold) {
    // debug_marker_.pushPoseMarker(trajectory.points.at(i).pose, "min_longitudinal_acc");
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidMaxLongitudinalAcceleration(const Trajectory & trajectory)
{
  const auto acc_arr = getLongitudinalAccArray(trajectory);
  const auto [max_longitudinal_acc, i_max] = getAbsMaxValAndIdx(acc_arr);
  validation_status_.max_longitudinal_acc = max_longitudinal_acc;

  if (max_longitudinal_acc > validation_params_.longitudinal_max_acc_threshold) {
    // debug_marker_.pushPoseMarker(trajectory.points.at(i).pose, "max_longitudinal_acc");
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidSteering(const Trajectory & trajectory)
{
  const auto steerings = calcSteeringAngles(trajectory, vehicle_info_.wheel_base_m);
  const auto [max_steering, i_max] = getAbsMaxValAndIdx(steerings);
  validation_status_.max_steering = max_steering;

  if (max_steering > validation_params_.steering_threshold) {
    // debug_marker_.pushPoseMarker(trajectory.points.at(i_max).pose, "max_steering");
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidSteeringRate(const Trajectory & trajectory)
{
  const auto steering_rates = calcSteeringRates(trajectory, vehicle_info_.wheel_base_m);
  const auto [max_steering_rate, i_max] = getAbsMaxValAndIdx(steering_rates);
  validation_status_.max_steering_rate = max_steering_rate;

  if (max_steering_rate > validation_params_.steering_rate_threshold) {
    // debug_marker_.pushPoseMarker(trajectory.points.at(i_max).pose, "max_steering_rate");
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidVelocityDeviation(const Trajectory & trajectory)
{
  // TODO(horibe): set appropriate thresholds for index search
  const auto idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    trajectory.points, current_kinematics_->pose.pose);

  validation_status_.velocity_deviation = std::abs(
    trajectory.points.at(idx).longitudinal_velocity_mps -
    current_kinematics_->twist.twist.linear.x);

  if (validation_status_.velocity_deviation > validation_params_.velocity_deviation_threshold) {
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidDistanceDeviation(const Trajectory & trajectory)
{
  // TODO(horibe): set appropriate thresholds for index search
  const auto idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    trajectory.points, current_kinematics_->pose.pose);

  validation_status_.distance_deviation =
    tier4_autoware_utils::calcDistance2d(trajectory.points.at(idx), current_kinematics_->pose.pose);

  if (validation_status_.distance_deviation > validation_params_.distance_deviation_threshold) {
    return false;
  }
  return true;
}

bool PlanningValidator::isAllValid(const PlanningValidatorStatus & s)
{
  return s.is_valid_finite_value && s.is_valid_interval && s.is_valid_relative_angle &&
         s.is_valid_curvature && s.is_valid_lateral_acc && s.is_valid_longitudinal_max_acc &&
         s.is_valid_longitudinal_min_acc && s.is_valid_steering && s.is_valid_steering_rate &&
         s.is_valid_velocity_deviation && s.is_valid_distance_deviation;
}

}  // namespace planning_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_validator::PlanningValidator)
