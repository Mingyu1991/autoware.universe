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

#include "apparent_safe_velocity_limiter/collision_distance.hpp"
#include "apparent_safe_velocity_limiter/occupancy_grid_utils.hpp"

#include <gtest/gtest.h>

TEST(TestOccupancyGridUtils, extractStaticObstaclePolygons)
{
  using apparent_safe_velocity_limiter::extractStaticObstaclePolygons;
  using apparent_safe_velocity_limiter::polygon_t;
  constexpr int8_t occupied_thr = 10;
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.info.height = 5;
  occupancy_grid.info.width = 5;
  occupancy_grid.data =
    std::vector<signed char>(occupancy_grid.info.height * occupancy_grid.info.width);
  occupancy_grid.info.resolution = 1.0;

  auto polygons = extractStaticObstaclePolygons(occupancy_grid, {}, occupied_thr);
  EXPECT_TRUE(polygons.empty());

  occupancy_grid.data.at(5) = 10;
  polygons = extractStaticObstaclePolygons(occupancy_grid, {}, occupied_thr);
  EXPECT_EQ(polygons.size(), 1ul);

  for (auto i = 1; i < 4; ++i)
    for (auto j = 1; j < 4; ++j) occupancy_grid.data[j + i * occupancy_grid.info.width] = 10;
  polygons = extractStaticObstaclePolygons(occupancy_grid, {}, occupied_thr);
  EXPECT_EQ(polygons.size(), 1ul);

  polygon_t full_mask;
  full_mask.outer() = {{0.0, 0.0}, {0.0, 5.0}, {5.0, 5.0}, {5.0, 0.0}};
  boost::geometry::correct(full_mask);
  std::cout << boost::geometry::wkt(full_mask) << std::endl;
  polygons = extractStaticObstaclePolygons(occupancy_grid, {full_mask}, occupied_thr);
  EXPECT_EQ(polygons.size(), 0ul);
}