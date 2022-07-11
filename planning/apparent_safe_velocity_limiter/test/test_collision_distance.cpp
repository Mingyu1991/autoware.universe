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

#include "apparent_safe_velocity_limiter/collision.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <gtest/gtest.h>

#include <algorithm>

const auto point_in_polygon = [](const auto x, const auto y, const auto & polygon) {
  return std::find_if(polygon.outer().begin(), polygon.outer().end(), [=](const auto & pt) {
           return pt.x() == x && pt.y() == y;
         }) != polygon.outer().end();
};

TEST(TestCollisionDistance, distanceToClosestCollision)
{
  using apparent_safe_velocity_limiter::distanceToClosestCollision;

  apparent_safe_velocity_limiter::segment_t vector = {{0.0, 0.0}, {5.0, 0.0}};
  apparent_safe_velocity_limiter::polygon_t footprint;
  footprint.outer() = {{0.0, 1.0}, {5.0, 1.0}, {5.0, -1.0}, {0.0, -1.0}};
  boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
  apparent_safe_velocity_limiter::multilinestring_t obstacles;

  std::optional<double> result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_FALSE(result.has_value());

  obstacles.push_back({{-1.0, 0.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_FALSE(result.has_value());

  obstacles.push_back({{1.0, 2.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_FALSE(result.has_value());

  obstacles.push_back({{4.0, 0.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 4.0);

  obstacles.push_back({{3.0, 0.5}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 3.0);

  obstacles.push_back({{2.5, -0.75}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 2.5);

  // Change vector and footprint
  vector = {{0.0, 0.0}, {5.0, 5.0}};
  footprint.outer() = {{-1.0, 1.0}, {4.0, 6.0}, {6.0, 4.0}, {1.0, -1.0}};
  boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
  obstacles.clear();

  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_FALSE(result.has_value());

  obstacles.push_back({{4.0, 4.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, std::sqrt(2 * 4.0 * 4.0));

  obstacles.push_back({{1.0, 2.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.121, 1e-3);

  obstacles.push_back({{-2.0, 2.0}, {3.0, -1.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.354, 1e-3);

  obstacles.push_back({{-1.5, 1.5}, {0.0, 0.5}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.141, 1e-3);

  obstacles.push_back({{0.5, 1.0}, {0.5, -0.5}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.0, 1e-3);

  obstacles.clear();
  obstacles.push_back({{0.5, 1.0}, {0.5, 0.0}, {1.5, 0.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.353, 1e-3);

  // Change vector (opposite direction)
  vector = {{5.0, 5.0}, {0.0, 0.0}};
  obstacles.clear();

  obstacles.push_back({{1.0, 1.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, std::sqrt(2 * 4.0 * 4.0));

  obstacles.push_back({{4.0, 3.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.121, 1e-3);
}

TEST(TestCollisionDistance, createObjPolygons)
{
  using apparent_safe_velocity_limiter::createObjectPolygons;
  using autoware_auto_perception_msgs::msg::PredictedObject;
  using autoware_auto_perception_msgs::msg::PredictedObjects;

  PredictedObjects objects;

  auto polygons = createObjectPolygons(objects, 0.0, 0.0);
  EXPECT_TRUE(polygons.empty());

  PredictedObject object1;
  object1.kinematics.initial_pose_with_covariance.pose.position.x = 0.0;
  object1.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  object1.kinematics.initial_pose_with_covariance.pose.orientation =
    tier4_autoware_utils::createQuaternionFromYaw(0.0);
  object1.kinematics.initial_twist_with_covariance.twist.linear.x = 0.0;
  object1.shape.dimensions.x = 1.0;
  object1.shape.dimensions.y = 1.0;
  objects.objects.push_back(object1);

  polygons = createObjectPolygons(objects, 0.0, 1.0);
  EXPECT_TRUE(polygons.empty());

  polygons = createObjectPolygons(objects, 0.0, 0.0);
  ASSERT_EQ(polygons.size(), 1ul);
  EXPECT_TRUE(point_in_polygon(0.5, 0.5, polygons[0]));
  EXPECT_TRUE(point_in_polygon(0.5, -0.5, polygons[0]));
  EXPECT_TRUE(point_in_polygon(-0.5, 0.5, polygons[0]));
  EXPECT_TRUE(point_in_polygon(-0.5, -0.5, polygons[0]));

  polygons = createObjectPolygons(objects, 1.0, 0.0);
  ASSERT_EQ(polygons.size(), 1ul);
  EXPECT_TRUE(point_in_polygon(1.0, 1.0, polygons[0]));
  EXPECT_TRUE(point_in_polygon(1.0, -1.0, polygons[0]));
  EXPECT_TRUE(point_in_polygon(-1.0, 1.0, polygons[0]));
  EXPECT_TRUE(point_in_polygon(-1.0, -1.0, polygons[0]));

  PredictedObject object2;
  object2.kinematics.initial_pose_with_covariance.pose.position.x = 10.0;
  object2.kinematics.initial_pose_with_covariance.pose.position.y = 10.0;
  object2.kinematics.initial_pose_with_covariance.pose.orientation =
    tier4_autoware_utils::createQuaternionFromYaw(M_PI_2);
  object2.kinematics.initial_twist_with_covariance.twist.linear.x = 2.0;
  object2.shape.dimensions.x = 2.0;
  object2.shape.dimensions.y = 1.0;
  objects.objects.push_back(object2);

  polygons = createObjectPolygons(objects, 0.0, 2.0);
  ASSERT_EQ(polygons.size(), 1ul);
  EXPECT_TRUE(point_in_polygon(10.5, 11.0, polygons[0]));
  EXPECT_TRUE(point_in_polygon(10.5, 9.0, polygons[0]));
  EXPECT_TRUE(point_in_polygon(9.5, 11.0, polygons[0]));
  EXPECT_TRUE(point_in_polygon(9.5, 9.0, polygons[0]));

  polygons = createObjectPolygons(objects, 0.0, 0.0);
  EXPECT_EQ(polygons.size(), 2ul);
}
