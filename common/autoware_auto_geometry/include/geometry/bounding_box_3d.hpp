// Copyright 2023-2026 the Autoware Foundation
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
//
// Developed by Tier IV, Inc.
/// \file
/// \brief Main header for user-facing 3d bounding box algorithms: functions and types
#ifndef GEOMETRY__BOUNDING_BOX_3D_HPP_
#define GEOMETRY__BOUNDING_BOX_3D_HPP_

#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

namespace autoware
{
namespace common
{
namespace geometry
{

/**
 * Two frames are defined in this class:
 * World frame: the coordinate system within that the bounding box is defined
 * Box frame: the coordinate system whose origin is the center of the bounding box
 * and coordinate axes are parallel to the sides of the bounding box
*/
class BoundingBox3D
{
public:
  BoundingBox3D(
    const tf2::Vector3& position = tf2::Vector3(0.0, 0.0, 0.0), 
    const tf2::Quaternion& orientation = tf2::Quaternion(0.0, 0.0, 0.0, 1.0), 
    double length = 0.0, double width = 0.0, double height = 0.0);
  BoundingBox3D(const autoware_auto_perception_msgs::msg::PredictedObject& object);
  /**
   * retrieve the center position of the bounding box in world frame
  */
  inline tf2::Vector3 getPosition() {return position_;}
  /**
   * retrieve the center position of the bounding box in world frame for constant instance
  */
  inline tf2::Vector3 getPosition() const {return position_;}
  /**
   * retrieve the orientation of the bounding box in world frame
  */
  inline tf2::Quaternion getOrientation() {return orientation_;}
  /**
   * retrieve the orientation of the bounding box in world frame for constant instance
  */
  inline tf2::Quaternion getOrientation() const {return orientation_;}
  /**
   * retrieve the length of the bounding box
  */
  inline double getLength() {return length_;}
  /**
   * retrieve the length of the bounding box for constant instance
  */
  inline double getLength() const {return length_;}
  /**
   * retrieve the width of the bounding box
  */
  inline double getWidth() {return width_;}
  /**
   * retrieve the width of the bounding box for constant instance
  */
  inline double getWidth() const {return width_;}
  /**
   * retrieve the height of the bounding box
  */
  inline double getHeight() {return height_;}
  /**
   * retrieve the height of the bounding box for constant instance
  */
  inline double getHeight() const {return height_;}
  /**
   * retrieve the vertices of the bounding box in world frame
  */
  std::vector<tf2::Vector3> getVertices();
  /**
   * retrieve the vertices of the bounding box in world frame for constant instance
  */
  std::vector<tf2::Vector3> getVertices() const;
  /**
   * set the position of the bounding box in world frame
  */
  inline void setPosition(const tf2::Vector3& position)
  {
    this->position_ = position;
  }
  /**
   * set the orientation of the bounding box in world frame
  */
  inline void setOrientation(const tf2::Quaternion& orientation)
  {
    this->orientation_ = orientation;
  }
  /**
   * set the length of the bounding box
  */
  inline void setLength(double length)
  {
    this->length_ = length;
  }
  /**
   * set the width of the bounding box
  */
  inline void setWidth(double width)
  {
    this->width_ = width;
  }
  /**
   * set the height of the bounding box
  */
  inline void setHeight(double height)
  {
    this->height_ = height;
  }
  /**
   * return if a point in world frame is inside the bounding box
  */
  bool contains(const tf2::Vector3& pt);
  /**
   * return if a point in world frame is inside the bounding box for constant instance
  */
  bool contains(const tf2::Vector3& pt) const;

private:
  /**
   * transform a vector from box frame to world frame
  */
  tf2::Vector3 transformToWorldFrame(const tf2::Vector3& vector);
  /**
   * transform a vector from box frame to world frame for constant instance
  */
  tf2::Vector3 transformToWorldFrame(const tf2::Vector3& vector) const;
  /**
   * transform a vector from world frame to box frame
  */
  tf2::Vector3 transformFromWorldFrame(const tf2::Vector3& vector);
  /**
   * transform a vector from world frame to box framefor constant instance
  */
  tf2::Vector3 transformFromWorldFrame(const tf2::Vector3& vector) const;
  /**
   * return if a point in box frame is inside the bounding box
  */
  bool insideBoundingBox(const tf2::Vector3& pt);
  /**
   * return if a point in box frame is inside the bounding box for constant instance
  */
  bool insideBoundingBox(const tf2::Vector3& pt) const;
  /**
   * position of the bounding box in world frame
  */
  tf2::Vector3 position_;
  /**
   * orientation of the bounding box in world frame
  */
  tf2::Quaternion orientation_;
  /**
   * length of the bounding box
  */
  double length_;
  /**
   * width of the bounding box
  */
  double width_;
  /**
   * height of the bounding box
  */
  double height_;
};

}  // namespace geometry
}  // namespace common
}  // namespace autoware
#endif  // GEOMETRY__BOUNDING_BOX_3D_HPP_
