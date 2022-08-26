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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__MAP_UTILS_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__MAP_UTILS_HPP_

#include "apparent_safe_velocity_limiter/types.hpp"

#include <lanelet2_core/LaneletMap.h>

#include <string>
#include <vector>

namespace apparent_safe_velocity_limiter
{

/// @brief Extract static obstacles from the lanelet map
/// @param[in] lanelet_map lanelet map
/// @param[in] tags tags to identify obstacle linestrings
/// @param[in] obstacle_ids ids to identify obstacle linestrings
/// @return the extracted obstacles
multilinestring_t extractStaticObstacles(
  const lanelet::LaneletMap & lanelet_map, const std::vector<std::string> & tags,
  const std::vector<int64_t> & obstacle_ids);

/// @brief Determine if the given linestring is an obstacle
/// @param[in] ls linestring to check
/// @param[in] tags obstacle tags
/// @param[in] ids obstacle ids
/// @return true if the linestring is an obstacle
bool isObstacle(
  const lanelet::ConstLineString3d & ls, const std::vector<std::string> & tags,
  const std::vector<int64_t> & ids);
}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__MAP_UTILS_HPP_
