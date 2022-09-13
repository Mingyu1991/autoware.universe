// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef GROUND_SEGMENTATION__GRID_SCAN_GROUND_FILTER_NODELET_HPP_
#define GROUND_SEGMENTATION__GRID_SCAN_GROUND_FILTER_NODELET_HPP_

#include "pointcloud_preprocessor/filter.hpp"

#include <vehicle_info_util/vehicle_info.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <tf2_ros/transform_listener.h>

#include <string>
#include <vector>

namespace ground_segmentation
{
using vehicle_info_util::VehicleInfo;

class GridScanGroundFilterComponent : public pointcloud_preprocessor::Filter
{
private:
  // classified point label
  // (0: not classified, 1: ground, 2: not ground, 3: follow previous point,
  //  4: unkown(currently not used), 5: virtual ground)
  enum class PointLabel {
    INIT = 0,
    GROUND,
    NON_GROUND,
    POINT_FOLLOW,
    UNKNOWN,
    VIRTUAL_GROUND,
    OUT_OF_RANGE
  };
  struct PointRef
  {
    float gama;         // angle in vertical
    float grid_size;    // radius of grid
    uint16_t grid_id;   // id of grid in vertical
    float radius;       // cylindrical coords on XY Plane
    float theta;        // angle deg on XY plane
    size_t radial_div;  // index of the radial division to which this point belongs to
    PointLabel point_state{PointLabel::INIT};

    size_t orig_index;  // index of this point in the source pointcloud
    pcl::PointXYZ * orig_point;
  };
  using PointCloudRefVector = std::vector<PointRef>;

  struct PointsCentroid
  {
    float radius_sum;
    float height_sum;
    float radius_avg;
    float height_avg;
    float height_max;
    uint32_t point_num;

    PointsCentroid()
    : radius_sum(0.0f), height_sum(0.0f), radius_avg(0.0f), height_avg(0.0f), point_num(0)
    {
    }

    void initialize()
    {
      radius_sum = 0.0f;
      height_sum = 0.0f;
      radius_avg = 0.0f;
      height_avg = 0.0f;
      height_max = 0.0f;
      point_num = 0;
    }

    void addPoint(const float radius, const float height)
    {
      radius_sum += radius;
      height_sum += height;
      ++point_num;
      radius_avg = radius_sum / point_num;
      height_avg = height_sum / point_num;
      height_max = height_max < height ? height : height_max;
    }

    float getAverageSlope() { return std::atan2(height_avg, radius_avg); }

    float getAverageHeight() { return height_avg; }

    float getAverageRadius() { return radius_avg; }

    float getMaxheight() { return height_max; }
  };

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pcl_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pcl_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr under_ground_pcl_pub_;
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  std::string base_frame_;
  std::string sensor_frame_;

  float non_ground_height_threshold_;
  float min_height_detection_range_;
  float grid_size_rad_;
  float grid_size_m_;
  float less_interest_dist_ = 20.0f;
  uint16_t gnd_grid_buffer_size_ = 10;
  uint16_t num_prev_grid_height_refer_ = 2;
  float grid_mode_switch_grid_id_ = 0.0f;
  float grid_mode_switch_angle_rad_ = 0.0f;
  float virtual_lidar_z_ = 2.5f;
  float detection_range_z_max_ = 2.5f;
  float center_pcl_shift_ = 0.0f;
  float grid_mode_switch_radius_ =
    5.0f;  // threshold distance for changing the mode of grid division
  float first_ring_distance_ = 20.0f;
  // float center_to_wheel_distance = 0.0f;
  double global_slope_max_angle_rad_;       // radians
  double local_slope_max_angle_rad_;        // radians
  double radial_divider_angle_rad_;         // distance in rads between dividers
  double split_points_distance_tolerance_;  // distance in meters between concentric divisions
  double                                    // minimum height threshold regardless the slope,
    split_height_distance_;                 // useful for close points
  bool use_virtual_ground_point_;
  size_t radial_dividers_num_;
  VehicleInfo vehicle_info_;

  /*!
   * Output transformed PointCloud from in_cloud_ptr->header.frame_id to in_target_frame
   * @param[in] in_target_frame Coordinate system to perform transform
   * @param[in] in_cloud_ptr PointCloud to perform transform
   * @param[out] out_cloud_ptr Resulting transformed PointCloud
   * @retval true transform succeeded
   * @retval false transform failed
   */
  bool transformPointCloud(
    const std::string & in_target_frame, const PointCloud2ConstPtr & in_cloud_ptr,
    const PointCloud2::SharedPtr & out_cloud_ptr);

  /*!
   * Convert pcl::PointCloud to sorted PointCloudRefVector
   * @param[in] in_cloud Input Point Cloud to be organized in radial segments
   * @param[out] out_radial_ordered_points_manager Vector of Points Clouds,
   *     each element will contain the points ordered
   */
  void convertPointcloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
    std::vector<PointCloudRefVector> & out_radial_ordered_points_manager);

  /*!
   * Classifies Points in the PointCloud as Ground and Not Ground
   * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud
   *     ordered by radial distance from the origin
   * @param out_no_ground_indices Returns the indices of the points
   *     classified as not ground in the original PointCloud
   */
  void classifyPointCloud(
    std::vector<PointCloudRefVector> & in_radial_ordered_clouds,
    pcl::PointIndices & out_no_ground_indices, pcl::PointIndices & out_ground_indices,
    pcl::PointIndices & out_unknown_indices, pcl::PointIndices & out_underground_indices);

  /*!
   * Returns the resulting complementary PointCloud, one with the points kept
   * and the other removed as indicated in the indices
   * @param in_cloud_ptr Input PointCloud to which the extraction will be performed
   * @param in_indices Indices of the points to be both removed and kept
   * @param out_object_cloud_ptr Resulting PointCloud with the indices kept
   */
  void extractObjectPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, const pcl::PointIndices & in_indices,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_object_cloud_ptr);

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter> & p);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit GridScanGroundFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace ground_segmentation

#endif  // GROUND_SEGMENTATION__GRID_SCAN_GROUND_FILTER_NODELET_HPP_