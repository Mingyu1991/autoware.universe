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

#include "path_smoother/node.hpp"

#include "interpolation/spline_interpolation_points_2d.hpp"
#include "motion_utils/motion_utils.hpp"
#include "rclcpp/time.hpp"
#include "tf2/utils.h"
#include "tier4_autoware_utils/ros/update_param.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesInfo(
  const VehicleParam & vehicle_param, const size_t circle_num, const double rear_radius_ratio,
  const double front_radius_ratio)
{
  std::vector<double> longitudinal_offsets;
  std::vector<double> radiuses;

  {  // 1st circle (rear)
    longitudinal_offsets.push_back(-vehicle_param.rear_overhang);
    radiuses.push_back(vehicle_param.width / 2.0 * rear_radius_ratio);
  }

  {  // 2nd circle (front)
    const double radius = std::hypot(
      vehicle_param.length / static_cast<double>(circle_num) / 2.0, vehicle_param.width / 2.0);

    const double unit_lon_length = vehicle_param.length / static_cast<double>(circle_num);
    const double longitudinal_offset =
      unit_lon_length / 2.0 + unit_lon_length * (circle_num - 1) - vehicle_param.rear_overhang;

    longitudinal_offsets.push_back(longitudinal_offset);
    radiuses.push_back(radius * front_radius_ratio);
  }

  return {radiuses, longitudinal_offsets};
}

std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesInfo(
  const VehicleParam & vehicle_param, const size_t circle_num, const double radius_ratio)
{
  std::vector<double> longitudinal_offsets;
  std::vector<double> radiuses;

  const double radius =
    std::hypot(
      vehicle_param.length / static_cast<double>(circle_num) / 2.0, vehicle_param.width / 2.0) *
    radius_ratio;
  const double unit_lon_length = vehicle_param.length / static_cast<double>(circle_num);

  for (size_t i = 0; i < circle_num; ++i) {
    longitudinal_offsets.push_back(
      unit_lon_length / 2.0 + unit_lon_length * i - vehicle_param.rear_overhang);
    radiuses.push_back(radius);
  }

  return {radiuses, longitudinal_offsets};
}

template <typename T>
autoware_auto_planning_msgs::msg::TrajectoryPoint convertToTrajectoryPoint(const T & point)
{
  autoware_auto_planning_msgs::msg::TrajectoryPoint traj_point;
  traj_point.pose = tier4_autoware_utils::getPose(point);
  traj_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
  traj_point.lateral_velocity_mps = point.lateral_velocity_mps;
  traj_point.heading_rate_rps = point.heading_rate_rps;
  return traj_point;
}

template <typename T>
std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> convertToTrajectoryPoints(
  const std::vector<T> & points)
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> traj_points;
  for (const auto & point : points) {
    const auto traj_point = convertToTrajectoryPoint(point);
    traj_points.push_back(traj_point);
  }
  return traj_points;
}
}  // namespace

PathSmoother::PathSmoother(const rclcpp::NodeOptions & node_options)
: Node("path_smoother", node_options), logger_ros_clock_(RCL_ROS_TIME)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // qos
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();

  // publisher to other nodes
  traj_pub_ = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/output/path", 1);

  // debug publisher
  debug_eb_traj_pub_ = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/debug/eb_trajectory", durable_qos);
  debug_extended_fixed_traj_pub_ = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/debug/extended_fixed_traj", 1);
  debug_extended_non_fixed_traj_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/debug/extended_non_fixed_traj", 1);
  debug_mpt_fixed_traj_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/debug/mpt_fixed_traj", 1);
  debug_mpt_ref_traj_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/debug/mpt_ref_traj", 1);
  debug_mpt_traj_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/debug/mpt_traj", 1);
  debug_markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", durable_qos);
  debug_wall_markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/wall_marker", durable_qos);
  debug_clearance_map_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/clearance_map", durable_qos);
  debug_object_clearance_map_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/object_clearance_map", durable_qos);
  debug_area_with_objects_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/area_with_objects", durable_qos);
  debug_msg_pub_ =
    create_publisher<tier4_debug_msgs::msg::StringStamped>("~/debug/calculation_time", 1);

  // subscriber
  path_sub_ = create_subscription<autoware_auto_planning_msgs::msg::Path>(
    "~/input/path", rclcpp::QoS{1},
    std::bind(&PathSmoother::pathCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{1},
    std::bind(&PathSmoother::odomCallback, this, std::placeholders::_1));
  objects_sub_ = create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
    "~/input/objects", rclcpp::QoS{10},
    std::bind(&PathSmoother::objectsCallback, this, std::placeholders::_1));

  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  {  // vehicle param
    vehicle_param_ = VehicleParam{};
    vehicle_param_.width = vehicle_info.vehicle_width_m;
    vehicle_param_.length = vehicle_info.vehicle_length_m;
    vehicle_param_.wheelbase = vehicle_info.wheel_base_m;
    vehicle_param_.rear_overhang = vehicle_info.rear_overhang_m;
    vehicle_param_.front_overhang = vehicle_info.front_overhang_m;
  }

  {  // option parameter
    is_publishing_debug_visualization_marker_ =
      declare_parameter<bool>("option.is_publishing_debug_visualization_marker");
    is_publishing_clearance_map_ = declare_parameter<bool>("option.is_publishing_clearance_map");
    is_publishing_object_clearance_map_ =
      declare_parameter<bool>("option.is_publishing_object_clearance_map");
    is_publishing_area_with_objects_ =
      declare_parameter<bool>("option.is_publishing_area_with_objects");

    is_showing_debug_info_ = declare_parameter<bool>("option.is_showing_debug_info");
    is_showing_calculation_time_ = declare_parameter<bool>("option.is_showing_calculation_time");
    is_stopping_if_outside_drivable_area_ =
      declare_parameter<bool>("option.is_stopping_if_outside_drivable_area");

    enable_avoidance_ = declare_parameter<bool>("option.enable_avoidance");
    enable_pre_smoothing_ = declare_parameter<bool>("option.enable_pre_smoothing");
    skip_optimization_ = declare_parameter<bool>("option.skip_optimization");
    reset_prev_optimization_ = declare_parameter<bool>("option.reset_prev_optimization");
  }

  {  // trajectory parameter
    traj_param_ = TrajectoryParam{};

    // common
    traj_param_.num_sampling_points = declare_parameter<int>("common.num_sampling_points");
    traj_param_.trajectory_length = declare_parameter<double>("common.trajectory_length");
    traj_param_.forward_fixing_min_distance =
      declare_parameter<double>("common.forward_fixing_min_distance");
    traj_param_.forward_fixing_min_time =
      declare_parameter<double>("common.forward_fixing_min_time");
    traj_param_.backward_fixing_distance =
      declare_parameter<double>("common.backward_fixing_distance");
    traj_param_.delta_arc_length_for_trajectory =
      declare_parameter<double>("common.delta_arc_length_for_trajectory");

    traj_param_.delta_dist_threshold_for_closest_point =
      declare_parameter<double>("common.delta_dist_threshold_for_closest_point");
    traj_param_.delta_yaw_threshold_for_closest_point =
      declare_parameter<double>("common.delta_yaw_threshold_for_closest_point");
    traj_param_.delta_yaw_threshold_for_straight =
      declare_parameter<double>("common.delta_yaw_threshold_for_straight");

    traj_param_.num_fix_points_for_extending =
      declare_parameter<int>("common.num_fix_points_for_extending");
    traj_param_.max_dist_for_extending_end_point =
      declare_parameter<double>("common.max_dist_for_extending_end_point");

    // object
    traj_param_.max_avoiding_ego_velocity_ms =
      declare_parameter<double>("object.max_avoiding_ego_velocity_ms");
    traj_param_.max_avoiding_objects_velocity_ms =
      declare_parameter<double>("object.max_avoiding_objects_velocity_ms");
    traj_param_.is_avoiding_unknown =
      declare_parameter<bool>("object.avoiding_object_type.unknown", true);
    traj_param_.is_avoiding_car = declare_parameter<bool>("object.avoiding_object_type.car", true);
    traj_param_.is_avoiding_truck =
      declare_parameter<bool>("object.avoiding_object_type.truck", true);
    traj_param_.is_avoiding_bus = declare_parameter<bool>("object.avoiding_object_type.bus", true);
    traj_param_.is_avoiding_bicycle =
      declare_parameter<bool>("object.avoiding_object_type.bicycle", true);
    traj_param_.is_avoiding_motorbike =
      declare_parameter<bool>("object.avoiding_object_type.motorbike", true);
    traj_param_.is_avoiding_pedestrian =
      declare_parameter<bool>("object.avoiding_object_type.pedestrian", true);
    traj_param_.is_avoiding_animal =
      declare_parameter<bool>("object.avoiding_object_type.animal", true);

    // ego nearest search
    traj_param_.ego_nearest_dist_threshold =
      declare_parameter<double>("ego_nearest_dist_threshold");
    traj_param_.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");
  }

  {  // elastic band parameter
    eb_param_ = EBParam{};

    // common
    eb_param_.num_joint_buffer_points =
      declare_parameter<int>("advanced.eb.common.num_joint_buffer_points");
    eb_param_.num_offset_for_begin_idx =
      declare_parameter<int>("advanced.eb.common.num_offset_for_begin_idx");
    eb_param_.delta_arc_length_for_eb =
      declare_parameter<double>("advanced.eb.common.delta_arc_length_for_eb");
    eb_param_.num_sampling_points_for_eb =
      declare_parameter<int>("advanced.eb.common.num_sampling_points_for_eb");

    // clearance
    eb_param_.clearance_for_straight_line =
      declare_parameter<double>("advanced.eb.clearance.clearance_for_straight_line");
    eb_param_.clearance_for_joint =
      declare_parameter<double>("advanced.eb.clearance.clearance_for_joint");
    eb_param_.clearance_for_only_smoothing =
      declare_parameter<double>("advanced.eb.clearance.clearance_for_only_smoothing");

    // qp
    eb_param_.qp_param.max_iteration = declare_parameter<int>("advanced.eb.qp.max_iteration");
    eb_param_.qp_param.eps_abs = declare_parameter<double>("advanced.eb.qp.eps_abs");
    eb_param_.qp_param.eps_rel = declare_parameter<double>("advanced.eb.qp.eps_rel");

    // other
    eb_param_.clearance_for_fixing = 0.0;
  }

  {  // mpt param
    mpt_param_ = MPTParam{};

    // option
    // TODO(murooka) implement plan_from_ego
    mpt_param_.plan_from_ego = declare_parameter<bool>("mpt.option.plan_from_ego");
    mpt_param_.max_plan_from_ego_length =
      declare_parameter<double>("mpt.option.max_plan_from_ego_length");
    mpt_param_.steer_limit_constraint =
      declare_parameter<bool>("mpt.option.steer_limit_constraint");
    mpt_param_.fix_points_around_ego = declare_parameter<bool>("mpt.option.fix_points_around_ego");
    mpt_param_.enable_warm_start = declare_parameter<bool>("mpt.option.enable_warm_start");
    mpt_param_.enable_manual_warm_start =
      declare_parameter<bool>("mpt.option.enable_manual_warm_start");
    mpt_visualize_sampling_num_ = declare_parameter<int>("mpt.option.visualize_sampling_num");

    // common
    mpt_param_.num_curvature_sampling_points =
      declare_parameter<int>("mpt.common.num_curvature_sampling_points");

    mpt_param_.delta_arc_length_for_mpt_points =
      declare_parameter<double>("mpt.common.delta_arc_length_for_mpt_points");

    // kinematics
    mpt_param_.max_steer_rad = vehicle_info.max_steer_angle_rad;

    // By default, optimization_center_offset will be vehicle_info.wheel_base * 0.8
    // The 0.8 scale is adopted as it performed the best.
    constexpr double default_wheelbase_ratio = 0.8;
    mpt_param_.optimization_center_offset = declare_parameter<double>(
      "mpt.kinematics.optimization_center_offset",
      vehicle_param_.wheelbase * default_wheelbase_ratio);

    // bounds search
    mpt_param_.bounds_search_widths =
      declare_parameter<std::vector<double>>("advanced.mpt.bounds_search_widths");

    // collision free constraints
    mpt_param_.l_inf_norm =
      declare_parameter<bool>("advanced.mpt.collision_free_constraints.option.l_inf_norm");
    mpt_param_.soft_constraint =
      declare_parameter<bool>("advanced.mpt.collision_free_constraints.option.soft_constraint");
    mpt_param_.hard_constraint =
      declare_parameter<bool>("advanced.mpt.collision_free_constraints.option.hard_constraint");
    // TODO(murooka) implement two-step soft constraint
    mpt_param_.two_step_soft_constraint = false;
    // mpt_param_.two_step_soft_constraint =
    // declare_parameter<bool>("advanced.mpt.collision_free_constraints.option.two_step_soft_constraint");

    {  // vehicle_circles
       // NOTE: Vehicle shape for collision free constraints is considered as a set of circles
      vehicle_circle_method_ = declare_parameter<std::string>(
        "advanced.mpt.collision_free_constraints.vehicle_circles.method");

      if (vehicle_circle_method_ == "uniform_circle") {
        vehicle_circle_num_for_calculation_ = declare_parameter<int>(
          "advanced.mpt.collision_free_constraints.vehicle_circles.uniform_circle.num");
        vehicle_circle_radius_ratios_.push_back(declare_parameter<double>(
          "advanced.mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio"));

        std::tie(
          mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
          calcVehicleCirclesInfo(
            vehicle_param_, vehicle_circle_num_for_calculation_,
            vehicle_circle_radius_ratios_.front());
      } else if (vehicle_circle_method_ == "rear_drive") {
        vehicle_circle_num_for_calculation_ = declare_parameter<int>(
          "advanced.mpt.collision_free_constraints.vehicle_circles.rear_drive.num_for_calculation");

        vehicle_circle_radius_ratios_.push_back(declare_parameter<double>(
          "advanced.mpt.collision_free_constraints.vehicle_circles.rear_drive.rear_radius_ratio"));
        vehicle_circle_radius_ratios_.push_back(declare_parameter<double>(
          "advanced.mpt.collision_free_constraints.vehicle_circles.rear_drive.front_radius_ratio"));

        std::tie(
          mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
          calcVehicleCirclesInfo(
            vehicle_param_, vehicle_circle_num_for_calculation_,
            vehicle_circle_radius_ratios_.front(), vehicle_circle_radius_ratios_.back());
      } else {
        throw std::invalid_argument(
          "advanced.mpt.collision_free_constraints.vehicle_circles.num parameter is invalid.");
      }
    }

    // clearance
    mpt_param_.hard_clearance_from_road =
      declare_parameter<double>("advanced.mpt.clearance.hard_clearance_from_road");
    mpt_param_.soft_clearance_from_road =
      declare_parameter<double>("advanced.mpt.clearance.soft_clearance_from_road");
    mpt_param_.soft_second_clearance_from_road =
      declare_parameter<double>("advanced.mpt.clearance.soft_second_clearance_from_road");
    mpt_param_.extra_desired_clearance_from_road =
      declare_parameter<double>("advanced.mpt.clearance.extra_desired_clearance_from_road");
    mpt_param_.clearance_from_object =
      declare_parameter<double>("advanced.mpt.clearance.clearance_from_object");

    // weight
    mpt_param_.soft_avoidance_weight =
      declare_parameter<double>("advanced.mpt.weight.soft_avoidance_weight");
    mpt_param_.soft_second_avoidance_weight =
      declare_parameter<double>("advanced.mpt.weight.soft_second_avoidance_weight");

    mpt_param_.lat_error_weight = declare_parameter<double>("advanced.mpt.weight.lat_error_weight");
    mpt_param_.yaw_error_weight = declare_parameter<double>("advanced.mpt.weight.yaw_error_weight");
    mpt_param_.yaw_error_rate_weight =
      declare_parameter<double>("advanced.mpt.weight.yaw_error_rate_weight");
    mpt_param_.steer_input_weight =
      declare_parameter<double>("advanced.mpt.weight.steer_input_weight");
    mpt_param_.steer_rate_weight =
      declare_parameter<double>("advanced.mpt.weight.steer_rate_weight");

    mpt_param_.obstacle_avoid_lat_error_weight =
      declare_parameter<double>("advanced.mpt.weight.obstacle_avoid_lat_error_weight");
    mpt_param_.obstacle_avoid_yaw_error_weight =
      declare_parameter<double>("advanced.mpt.weight.obstacle_avoid_yaw_error_weight");
    mpt_param_.obstacle_avoid_steer_input_weight =
      declare_parameter<double>("advanced.mpt.weight.obstacle_avoid_steer_input_weight");
    mpt_param_.near_objects_length =
      declare_parameter<double>("advanced.mpt.weight.near_objects_length");

    mpt_param_.terminal_lat_error_weight =
      declare_parameter<double>("advanced.mpt.weight.terminal_lat_error_weight");
    mpt_param_.terminal_yaw_error_weight =
      declare_parameter<double>("advanced.mpt.weight.terminal_yaw_error_weight");
    mpt_param_.terminal_path_lat_error_weight =
      declare_parameter<double>("advanced.mpt.weight.terminal_path_lat_error_weight");
    mpt_param_.terminal_path_yaw_error_weight =
      declare_parameter<double>("advanced.mpt.weight.terminal_path_yaw_error_weight");
  }

  {  // replan
    max_path_shape_change_dist_for_replan_ =
      declare_parameter<double>("replan.max_path_shape_change_dist");
    max_ego_moving_dist_for_replan_ =
      declare_parameter<double>("replan.max_ego_moving_dist_for_replan");
    max_delta_time_sec_for_replan_ =
      declare_parameter<double>("replan.max_delta_time_sec_for_replan");
  }

  // TODO(murooka) tune this param when avoiding with path_smoother
  traj_param_.center_line_width = vehicle_param_.width;

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PathSmoother::paramCallback, this, std::placeholders::_1));

  resetPlanning();

  self_pose_listener_.waitForFirstPose();
}

void PathSmoother::resetPlanning()
{
  RCLCPP_WARN(get_logger(), "[ObstacleAvoidancePlanner] Reset planning");

  costmap_generator_ptr_ = std::make_unique<CostmapGenerator>();

  eb_path_optimizer_ptr_ = std::make_unique<EBPathOptimizer>(
    is_showing_debug_info_, traj_param_, eb_param_, vehicle_param_);

  mpt_optimizer_ptr_ =
    std::make_unique<MPTOptimizer>(is_showing_debug_info_, traj_param_, vehicle_param_, mpt_param_);

  prev_path_points_ptr_ = nullptr;
  resetPrevOptimization();
}

void PathSmoother::resetPrevOptimization()
{
  prev_optimal_trajs_ptr_ = nullptr;
  eb_solved_count_ = 0;
}

rcl_interfaces::msg::SetParametersResult PathSmoother::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  {  // option parameter
    updateParam<bool>(
      parameters, "option.is_publishing_debug_visualization_marker",
      is_publishing_debug_visualization_marker_);
    updateParam<bool>(
      parameters, "option.is_publishing_clearance_map", is_publishing_clearance_map_);
    updateParam<bool>(
      parameters, "option.is_publishing_object_clearance_map", is_publishing_object_clearance_map_);
    updateParam<bool>(
      parameters, "option.is_publishing_area_with_objects", is_publishing_area_with_objects_);

    updateParam<bool>(parameters, "option.is_showing_debug_info", is_showing_debug_info_);
    updateParam<bool>(
      parameters, "option.is_showing_calculation_time", is_showing_calculation_time_);
    updateParam<bool>(
      parameters, "option.is_stopping_if_outside_drivable_area",
      is_stopping_if_outside_drivable_area_);

    updateParam<bool>(parameters, "option.enable_avoidance", enable_avoidance_);
    updateParam<bool>(parameters, "option.enable_pre_smoothing", enable_pre_smoothing_);
    updateParam<bool>(parameters, "option.skip_optimization", skip_optimization_);
    updateParam<bool>(parameters, "option.reset_prev_optimization", reset_prev_optimization_);
  }

  {  // trajectory parameter
    // common
    updateParam<int>(parameters, "common.num_sampling_points", traj_param_.num_sampling_points);
    updateParam<double>(parameters, "common.trajectory_length", traj_param_.trajectory_length);
    updateParam<double>(
      parameters, "common.forward_fixing_min_distance", traj_param_.forward_fixing_min_distance);
    updateParam<double>(
      parameters, "common.forward_fixing_min_time", traj_param_.forward_fixing_min_time);
    updateParam<double>(
      parameters, "common.backward_fixing_distance", traj_param_.backward_fixing_distance);
    updateParam<double>(
      parameters, "common.delta_arc_length_for_trajectory",
      traj_param_.delta_arc_length_for_trajectory);

    updateParam<double>(
      parameters, "common.delta_dist_threshold_for_closest_point",
      traj_param_.delta_dist_threshold_for_closest_point);
    updateParam<double>(
      parameters, "common.delta_yaw_threshold_for_closest_point",
      traj_param_.delta_yaw_threshold_for_closest_point);
    updateParam<double>(
      parameters, "common.delta_yaw_threshold_for_straight",
      traj_param_.delta_yaw_threshold_for_straight);
    updateParam<int>(
      parameters, "common.num_fix_points_for_extending", traj_param_.num_fix_points_for_extending);
    updateParam<double>(
      parameters, "common.max_dist_for_extending_end_point",
      traj_param_.max_dist_for_extending_end_point);

    // object
    updateParam<double>(
      parameters, "object.max_avoiding_ego_velocity_ms", traj_param_.max_avoiding_ego_velocity_ms);
    updateParam<double>(
      parameters, "object.max_avoiding_objects_velocity_ms",
      traj_param_.max_avoiding_objects_velocity_ms);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.unknown", traj_param_.is_avoiding_unknown);
    updateParam<bool>(parameters, "object.avoiding_object_type.car", traj_param_.is_avoiding_car);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.truck", traj_param_.is_avoiding_truck);
    updateParam<bool>(parameters, "object.avoiding_object_type.bus", traj_param_.is_avoiding_bus);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.bicycle", traj_param_.is_avoiding_bicycle);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.motorbike", traj_param_.is_avoiding_motorbike);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.pedestrian", traj_param_.is_avoiding_pedestrian);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.animal", traj_param_.is_avoiding_animal);
  }

  {  // elastic band parameter
    // common
    updateParam<int>(
      parameters, "advanced.eb.common.num_joint_buffer_points", eb_param_.num_joint_buffer_points);
    updateParam<int>(
      parameters, "advanced.eb.common.num_offset_for_begin_idx",
      eb_param_.num_offset_for_begin_idx);
    updateParam<double>(
      parameters, "advanced.eb.common.delta_arc_length_for_eb", eb_param_.delta_arc_length_for_eb);
    updateParam<int>(
      parameters, "advanced.eb.common.num_sampling_points_for_eb",
      eb_param_.num_sampling_points_for_eb);

    // clearance
    updateParam<double>(
      parameters, "advanced.eb.clearance.clearance_for_straight_line",
      eb_param_.clearance_for_straight_line);
    updateParam<double>(
      parameters, "advanced.eb.clearance.clearance_for_joint", eb_param_.clearance_for_joint);
    updateParam<double>(
      parameters, "advanced.eb.clearance.clearance_for_only_smoothing",
      eb_param_.clearance_for_only_smoothing);

    // qp
    updateParam<int>(parameters, "advanced.eb.qp.max_iteration", eb_param_.qp_param.max_iteration);
    updateParam<double>(parameters, "advanced.eb.qp.eps_abs", eb_param_.qp_param.eps_abs);
    updateParam<double>(parameters, "advanced.eb.qp.eps_rel", eb_param_.qp_param.eps_rel);
  }

  {  // mpt param
    // option
    updateParam<bool>(parameters, "mpt.option.plan_from_ego", mpt_param_.plan_from_ego);
    updateParam<double>(
      parameters, "mpt.option.max_plan_from_ego_length", mpt_param_.max_plan_from_ego_length);
    updateParam<bool>(
      parameters, "mpt.option.steer_limit_constraint", mpt_param_.steer_limit_constraint);
    updateParam<bool>(
      parameters, "mpt.option.fix_points_around_ego", mpt_param_.fix_points_around_ego);
    updateParam<bool>(parameters, "mpt.option.enable_warm_start", mpt_param_.enable_warm_start);
    updateParam<bool>(
      parameters, "mpt.option.enable_manual_warm_start", mpt_param_.enable_manual_warm_start);
    updateParam<int>(parameters, "mpt.option.visualize_sampling_num", mpt_visualize_sampling_num_);

    // common
    updateParam<int>(
      parameters, "mpt.common.num_curvature_sampling_points",
      mpt_param_.num_curvature_sampling_points);

    updateParam<double>(
      parameters, "mpt.common.delta_arc_length_for_mpt_points",
      mpt_param_.delta_arc_length_for_mpt_points);

    // kinematics
    updateParam<double>(
      parameters, "mpt.kinematics.optimization_center_offset",
      mpt_param_.optimization_center_offset);

    // collision_free_constraints
    updateParam<bool>(
      parameters, "advanced.mpt.collision_free_constraints.option.l_inf_norm",
      mpt_param_.l_inf_norm);
    // updateParam<bool>(
    //   parameters, "advanced.mpt.collision_free_constraints.option.two_step_soft_constraint",
    //   mpt_param_.two_step_soft_constraint);
    updateParam<bool>(
      parameters, "advanced.mpt.collision_free_constraints.option.soft_constraint",
      mpt_param_.soft_constraint);
    updateParam<bool>(
      parameters, "advanced.mpt.collision_free_constraints.option.hard_constraint",
      mpt_param_.hard_constraint);

    {  // vehicle_circles
      // NOTE: Changing method is not supported
      // updateParam<std::string>(
      //   parameters, "advanced.mpt.collision_free_constraints.vehicle_circles.method",
      //   vehicle_circle_method_);

      if (vehicle_circle_method_ == "uniform_circle") {
        updateParam<int>(
          parameters, "advanced.mpt.collision_free_constraints.vehicle_circles.uniform_circle.num",
          vehicle_circle_num_for_calculation_);
        updateParam<double>(
          parameters,
          "advanced.mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio",
          vehicle_circle_radius_ratios_.front());

        std::tie(
          mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
          calcVehicleCirclesInfo(
            vehicle_param_, vehicle_circle_num_for_calculation_,
            vehicle_circle_radius_ratios_.front());
      } else if (vehicle_circle_method_ == "rear_drive") {
        updateParam<int>(
          parameters,
          "advanced.mpt.collision_free_constraints.vehicle_circles.rear_drive.num_for_calculation",
          vehicle_circle_num_for_calculation_);

        updateParam<double>(
          parameters,
          "advanced.mpt.collision_free_constraints.vehicle_circles.rear_drive.rear_radius_ratio",
          vehicle_circle_radius_ratios_.front());

        updateParam<double>(
          parameters,
          "advanced.mpt.collision_free_constraints.vehicle_circles.rear_drive.front_radius_ratio",
          vehicle_circle_radius_ratios_.back());

        std::tie(
          mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
          calcVehicleCirclesInfo(
            vehicle_param_, vehicle_circle_num_for_calculation_,
            vehicle_circle_radius_ratios_.front(), vehicle_circle_radius_ratios_.back());
      } else {
        throw std::invalid_argument(
          "advanced.mpt.collision_free_constraints.vehicle_circles.num parameter is invalid.");
      }
    }

    // clearance
    updateParam<double>(
      parameters, "advanced.mpt.clearance.hard_clearance_from_road",
      mpt_param_.hard_clearance_from_road);
    updateParam<double>(
      parameters, "advanced.mpt.clearance.soft_clearance_from_road",
      mpt_param_.soft_clearance_from_road);
    updateParam<double>(
      parameters, "advanced.mpt.clearance.soft_second_clearance_from_road",
      mpt_param_.soft_second_clearance_from_road);
    updateParam<double>(
      parameters, "advanced.mpt.clearance.extra_desired_clearance_from_road",
      mpt_param_.extra_desired_clearance_from_road);
    updateParam<double>(
      parameters, "advanced.mpt.clearance.clearance_from_object", mpt_param_.clearance_from_object);

    // weight
    updateParam<double>(
      parameters, "advanced.mpt.weight.soft_avoidance_weight", mpt_param_.soft_avoidance_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.soft_second_avoidance_weight",
      mpt_param_.soft_second_avoidance_weight);

    updateParam<double>(
      parameters, "advanced.mpt.weight.lat_error_weight", mpt_param_.lat_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.yaw_error_weight", mpt_param_.yaw_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.yaw_error_rate_weight", mpt_param_.yaw_error_rate_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.steer_input_weight", mpt_param_.steer_input_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.steer_rate_weight", mpt_param_.steer_rate_weight);

    updateParam<double>(
      parameters, "advanced.mpt.weight.obstacle_avoid_lat_error_weight",
      mpt_param_.obstacle_avoid_lat_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.obstacle_avoid_yaw_error_weight",
      mpt_param_.obstacle_avoid_yaw_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.obstacle_avoid_steer_input_weight",
      mpt_param_.obstacle_avoid_steer_input_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.near_objects_length", mpt_param_.near_objects_length);

    updateParam<double>(
      parameters, "advanced.mpt.weight.terminal_lat_error_weight",
      mpt_param_.terminal_lat_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.terminal_yaw_error_weight",
      mpt_param_.terminal_yaw_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.terminal_path_lat_error_weight",
      mpt_param_.terminal_path_lat_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.terminal_path_yaw_error_weight",
      mpt_param_.terminal_path_yaw_error_weight);
  }

  {  // replan
    updateParam<double>(
      parameters, "replan.max_path_shape_change_dist", max_path_shape_change_dist_for_replan_);
    updateParam<double>(
      parameters, "replan.max_ego_moving_dist_for_replan", max_ego_moving_dist_for_replan_);
    updateParam<double>(
      parameters, "replan.max_delta_time_sec_for_replan", max_delta_time_sec_for_replan_);
  }

  resetPlanning();

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void PathSmoother::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_twist_ptr_ = std::make_unique<geometry_msgs::msg::TwistStamped>();
  current_twist_ptr_->header = msg->header;
  current_twist_ptr_->twist = msg->twist.twist;
}

void PathSmoother::objectsCallback(
  const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
{
  objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>(*msg);
}

void PathSmoother::pathCallback(const autoware_auto_planning_msgs::msg::Path::SharedPtr path_ptr)
{
  if (
    path_ptr->points.empty() || path_ptr->drivable_area.data.empty() || !current_twist_ptr_ ||
    !objects_ptr_) {
    return;
  }

  // initialize
  debug_data_ptr_ = std::make_shared<DebugData>();
  resetPlanning();

  // variables
  const double resample_interval = 2.0;
  const double valid_optimized_path_length = 30.0;
  const double path_length = motion_utils::calcArcLength(path_ptr->points);
  const size_t path_segment_num = static_cast<size_t>(path_length / valid_optimized_path_length);

  const auto resampled_path =
    motion_utils::resamplePath(*path_ptr, resample_interval);  // TODO(murooka)
  const auto resampled_traj_points = convertToTrajectoryPoints(resampled_path.points);

  // cv_maps
  const CVMaps cv_maps = costmap_generator_ptr_->getMaps(
    false, *path_ptr, objects_ptr_->objects, traj_param_, debug_data_ptr_);

  const size_t initial_target_index = 3;
  auto target_pose = resampled_path.points.at(initial_target_index).pose;  // TODO(murooka)
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> whole_optimized_traj_points;

  std::cerr << "==============================================" << std::endl;
  for (size_t i = 0; i < path_segment_num; ++i) {
    std::cerr << i << std::endl;
    target_pose = resampled_path.points
                    .at(initial_target_index + valid_optimized_path_length / resample_interval * i)
                    .pose;

    const auto mpt_trajs = mpt_optimizer_ptr_->getModelPredictiveTrajectory(
      false, resampled_traj_points, resampled_path.points, prev_optimal_trajs_ptr_, cv_maps,
      target_pose, 0.0, debug_data_ptr_);
    if (!mpt_trajs) {
      break;
    }

    Trajectories trajectories;
    trajectories.mpt_ref_points = mpt_trajs->ref_points;
    trajectories.model_predictive_trajectory = mpt_trajs->mpt;
    prev_optimal_trajs_ptr_ = std::make_unique<Trajectories>(trajectories);

    for (size_t j = 0; j < whole_optimized_traj_points.size(); ++j) {
      const double dist = tier4_autoware_utils::calcDistance2d(
        whole_optimized_traj_points.at(j), mpt_trajs->mpt.front());
      if (dist < 0.5) {
        const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
          extracted_whole_optimized_traj_points{
            whole_optimized_traj_points.begin(), whole_optimized_traj_points.begin() + j - 1};
        whole_optimized_traj_points = extracted_whole_optimized_traj_points;
      }
    }

    for (size_t j = 0; j < mpt_trajs->mpt.size(); ++j) {
      whole_optimized_traj_points.push_back(mpt_trajs->mpt.at(j));
    }
  }

  auto output_traj_msg = motion_utils::convertToTrajectory(whole_optimized_traj_points);
  output_traj_msg.header = path_ptr->header;
  traj_pub_->publish(output_traj_msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PathSmoother)
