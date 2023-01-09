#include <geometry/bounding_box_3d.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace autoware
{
namespace common
{
namespace geometry
{

BoundingBox3D::BoundingBox3D(
    const tf2::Vector3& position, const tf2::Quaternion& orientation, 
    double length, double width, double height):
    position_(position), orientation_(orientation), length_(length), width_(width), height_(height)
{

}

BoundingBox3D::BoundingBox3D(const autoware_auto_perception_msgs::msg::PredictedObject& object)
{
  position_ = tf2::Vector3(
    object.kinematics.initial_pose_with_covariance.pose.position.x,
    object.kinematics.initial_pose_with_covariance.pose.position.y,
    object.kinematics.initial_pose_with_covariance.pose.position.z
  );
  orientation_ = tf2::Quaternion(
    object.kinematics.initial_pose_with_covariance.pose.orientation.x,
    object.kinematics.initial_pose_with_covariance.pose.orientation.y,
    object.kinematics.initial_pose_with_covariance.pose.orientation.z,
    object.kinematics.initial_pose_with_covariance.pose.orientation.w
  );
  length_ = object.shape.dimensions.x;
  width_ = object.shape.dimensions.y;
  height_ = object.shape.dimensions.z;
}

std::vector<tf2::Vector3> BoundingBox3D::getVertices()
{
  return const_cast<const BoundingBox3D*>(this)->getVertices();
}

std::vector<tf2::Vector3> BoundingBox3D::getVertices() const
{
  std::vector<tf2::Vector3> vertices(8);
  int idx = 0;
  for(int x = -1; x <= 1; x += 2){
    for(int y = -1; y <= 1; y += 2){
      for(int z = -1; z <= 1; z += 2){
        vertices[idx++] = transformToWorldFrame(tf2::Vector3(
          0.5 * x * getLength(), 0.5 * y * getWidth(), 0.5 * z * getHeight()
        ));
      }
    }
  }
  return vertices;
}

bool BoundingBox3D::contains(const tf2::Vector3& pt)
{
  return const_cast<const BoundingBox3D*>(this)->contains(pt);
}

bool BoundingBox3D::contains(const tf2::Vector3& pt) const
{
  return insideBoundingBox(transformFromWorldFrame(pt));
}

tf2::Vector3 BoundingBox3D::transformToWorldFrame(const tf2::Vector3& vector)
{
  return const_cast<const BoundingBox3D*>(this)->transformToWorldFrame(vector);
}

tf2::Vector3 BoundingBox3D::transformToWorldFrame(const tf2::Vector3& vector) const
{
  return tf2::quatRotate(getOrientation(), vector) + getPosition();
}

tf2::Vector3 BoundingBox3D::transformFromWorldFrame(const tf2::Vector3& vector)
{
  return const_cast<const BoundingBox3D*>(this)->transformFromWorldFrame(vector);
}

tf2::Vector3 BoundingBox3D::transformFromWorldFrame(const tf2::Vector3& vector) const
{
  return tf2::quatRotate(getOrientation().inverse(), vector - getPosition());
}

bool BoundingBox3D::insideBoundingBox(const tf2::Vector3& pt)
{
  return const_cast<const BoundingBox3D*>(this)->insideBoundingBox(pt);
}

bool BoundingBox3D::insideBoundingBox(const tf2::Vector3& pt) const
{
  return std::abs(pt.x()) <= getLength() * 0.5
      && std::abs(pt.y()) <= getWidth() * 0.5
      && std::abs(pt.z()) <= getHeight() * 0.5;
}

}  // namespace geometry
}  // namespace common
}  // namespace autoware