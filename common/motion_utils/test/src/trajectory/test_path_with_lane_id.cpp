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

#include "motion_utils/trajectory/path_with_lane_id.hpp"
// #include "tier4_autoware_utils/geometry/geometry.hpp"
// #include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

TEST(trajectory, getPathIndexRangeWithLaneId)
{
  using autoware_auto_planning_msgs::msg::PathWithLaneId;
  using motion_utils::getPathIndexRangeWithLaneId;

  // Usual cases
  {
    PathWithLaneId points;
    points.points.resize(6);
    points.points.at(0).lane_ids.push_back(3);
    points.points.at(1).lane_ids.push_back(3);
    points.points.at(2).lane_ids.push_back(1);
    points.points.at(3).lane_ids.push_back(2);
    points.points.at(4).lane_ids.push_back(2);
    points.points.at(5).lane_ids.push_back(2);

    {
      const auto res = getPathIndexRangeWithLaneId(points, 3);
      EXPECT_EQ(res->first, 0);
      EXPECT_EQ(res->second, 1);
    }
    {
      const auto res = getPathIndexRangeWithLaneId(points, 1);
      EXPECT_EQ(res->first, 2);
      EXPECT_EQ(res->second, 2);
    }
    {
      const auto res = getPathIndexRangeWithLaneId(points, 2);
      EXPECT_EQ(res->first, 3);
      EXPECT_EQ(res->second, 5);
    }
    {
      const auto res = getPathIndexRangeWithLaneId(points, 4);
      EXPECT_EQ(res, boost::none);
    }
  }

  // Empty points
  {
    PathWithLaneId points;
    const auto res = getPathIndexRangeWithLaneId(points, 4);
    EXPECT_EQ(res, boost::none);
  }
}
