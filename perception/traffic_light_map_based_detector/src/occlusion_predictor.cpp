#include "traffic_light_map_based_detector/occlusion_predictor.hpp"

namespace traffic_light
{

std::string uuid2str(const autoware_auto_perception_msgs::msg::PredictedObject::_object_id_type& uuid)
{
  std::string res;
  for(size_t i = 0; i < uuid.uuid.size(); i++){
    res += "-" + std::to_string(uuid.uuid[i]);
  }
  return res;
}

void OcclusionObjectStatusSequence::update(const autoware_auto_perception_msgs::msg::PredictedObject& obj, double stamp)
{
  // the timestamps must be increasing for binary search
  if(status.empty() == false && stamp < status.back().stamp){
    std::cout << "received old object!" << std::endl;
    return;
  }
  status.push_back(OcclusionObjectStatus{stamp, 
    obj.kinematics.initial_pose_with_covariance.pose, 
    obj.kinematics.initial_twist_with_covariance.twist});
  shape = obj.shape;
}

bool OcclusionObjectStatusSequence::predict(double stamp, OcclusionObject& object) const
{
  if(status.empty()){
    return false;
  }
  // binary search the first status that has larger timestamp than stamp
  // low would be the result index
  int low = 0, high = status.size();
  while(low < high){
    int mid = (low + high) / 2;
    if(status[mid].stamp >= stamp){
      high = mid;
    }
    else{
      low = mid + 1;
    }
  }
  object.stamp = stamp;
  object.shape = shape;
  // there's no status with larger timestamp
  // predict with last status
  if(low == int(status.size())){
    const OcclusionObjectStatus& s2 = status.back();
    assert(stamp > s2.stamp);
    double dt = stamp - s2.stamp;
    double dx = s2.twist.linear.x * dt;
    double dy = s2.twist.linear.y * dt;
    double dz = s2.twist.linear.z * dt;
    object.pose = tier4_autoware_utils::calcOffsetPose(s2.pose, dx, dy, dz);
  }
  // all status have larger timestamp
  // predict with first status
  else if(low == 0){
    const OcclusionObjectStatus& s1 = status.front();
    const OcclusionObjectStatus& s2 = status.back();
    assert(stamp >= s1.stamp && stamp < s2.stamp);
    double ratio = (stamp - s1.stamp) / (s2.stamp - s1.stamp);
    object.pose = tier4_autoware_utils::calcInterpolatedPose(s1.pose, s2.pose, ratio);
  }
  // there's at least one with smaller timestamp and one with larger timestamp
  // do the interpolation of status[low - 1] and status[low]
  else{
    const OcclusionObjectStatus& s1 = status.front();
    assert(stamp <= s1.stamp);
    double dt = s1.stamp - stamp;
    double dx = -s1.twist.linear.x * dt;
    double dy = -s1.twist.linear.y * dt;
    double dz = -s1.twist.linear.z * dt;
    object.pose = tier4_autoware_utils::calcOffsetPose(s1.pose, dx, dy, dz);
  }
  return true;
}

void OcclusionPredictor::update(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  assert(msg->header.frame_id == "map");
  double stamp = rclcpp::Time(msg->header.stamp).seconds();
  std::set<std::string> updated_ids;
  for(const auto & input_obj : msg->objects){
    std::string id_str = uuid2str(input_obj.object_id);
    if(tracked_objects.count(id_str) == 0){
      tracked_objects.emplace(id_str, OcclusionObjectStatusSequence());
    }
    tracked_objects.find(id_str)->second.update(input_obj, stamp);
    updated_ids.insert(id_str);
  }
  // remove un-detected objects
  for(auto it = tracked_objects.begin(); it != tracked_objects.end();){
    if(updated_ids.count(it->first) == 0){
      it = tracked_objects.erase(it);
    }
    else{
      it++;
    }
  }
}

autoware_auto_perception_msgs::msg::PredictedObjects OcclusionPredictor::predict(const std_msgs::msg::Header::_stamp_type& input_stamp)
{
  autoware_auto_perception_msgs::msg::PredictedObjects res;
  res.header.frame_id = "map";
  res.header.stamp = input_stamp;
  double stamp = rclcpp::Time(input_stamp).seconds();
  int idx = 1;
  OcclusionObject predicted_obj;
  for(const auto & p : tracked_objects){
    if(p.second.predict(stamp, predicted_obj)){
      autoware_auto_perception_msgs::msg::PredictedObject obj;
      obj.existence_probability = 1.0;
      obj.kinematics.initial_pose_with_covariance.pose = predicted_obj.pose;
      obj.shape = predicted_obj.shape;
      obj.object_id.uuid.fill(idx++);
      res.objects.push_back(obj);
    }
  }
  return res;
}

void CloudOcclusionPredictor::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  history_clouds_.push_back(*msg);
  std::cout << "pointcloud callback" << std::endl;
}

void CloudOcclusionPredictor::update(const sensor_msgs::msg::CameraInfo& camera_info,
                                     const tf2_ros::Buffer& tf_buffer)
{
  std::cout << "enter update. cloud num = " << history_clouds_.size() << std::endl;
  if(history_clouds_.empty()){
    return;
  }
  // find the cloud whose timestamp is closest to the stamp of rough_rois
  std::list<sensor_msgs::msg::PointCloud2>::iterator closest_it;
  double min_stamp_diff = std::numeric_limits<double>::max();
  for(auto it = history_clouds_.begin(); it != history_clouds_.end(); it++){
    double stamp_diff = std::abs(rclcpp::Time(it->header.stamp).seconds() - rclcpp::Time(camera_info.header.stamp).seconds());
    if(stamp_diff < min_stamp_diff){
      min_stamp_diff = stamp_diff;
      closest_it = it;
    }
  }
  // erase clouds earlier than closest_it since their timestamps couldn't be closer to following rough_rois than closest_it
  for(auto it = history_clouds_.begin(); it != closest_it; ){
    it = history_clouds_.erase(it);
  }
  std::cout << "finish update cloud. cloud num = " << history_clouds_.size() << ", min stamp diff = " << min_stamp_diff << std::endl;

  try {
    geometry_msgs::msg::Transform transform = tf_buffer.lookupTransform(
      history_clouds_.front().header.frame_id, "map", rclcpp::Time(camera_info.header.stamp),
      rclcpp::Duration::from_seconds(0.2)).transform;
    tf_cloud2map_ = tf2::Transform (
      tf2::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
      tf2::Vector3(transform.translation.x, transform.translation.y, transform.translation.z));
    transform = tf_buffer.lookupTransform(
      history_clouds_.front().header.frame_id, camera_info.header.frame_id, rclcpp::Time(camera_info.header.stamp),
      rclcpp::Duration::from_seconds(0.2)).transform;
    tf_cloud2camera_ = tf2::Transform (
      tf2::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
      tf2::Vector3(transform.translation.x, transform.translation.y, transform.translation.z));
  } catch (tf2::TransformException & ex) {
    std::cout << "Error: cannot get transform from map frame to " << history_clouds_.front().header.frame_id << std::endl;
  }
  std::cout << "finish update TF" << std::endl;
}

sensor_msgs::msg::PointCloud2 CloudOcclusionPredictor::debug(const std::vector<lanelet::ConstLineString3d>& traffic_lights)
{
  if(history_clouds_.empty()){
    return sensor_msgs::msg::PointCloud2();
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(tf_cloud2camera_.getOrigin().x(), tf_cloud2camera_.getOrigin().y(), tf_cloud2camera_.getOrigin().z()));
  for(const auto & traffic_light : traffic_lights){
    const double tl_height = traffic_light.attributeOr("height", 0.0);
    const auto & tl_left_down_point = traffic_light.front();
    const auto & tl_right_down_point = traffic_light.back();
    tf2::Vector3 tl_center((tl_left_down_point.x() + tl_right_down_point.x()) / 2, 
                           (tl_left_down_point.y() + tl_right_down_point.y()) / 2, 
                           (tl_left_down_point.z() + tl_right_down_point.z() + tl_height) / 2);
    tf2::Vector3 tl_cloud = tf_cloud2map_ * tl_center;
    cloud.push_back(pcl::PointXYZ(tl_cloud.x(), tl_cloud.y(), tl_cloud.z()));
  }
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = history_clouds_.front().header;
  return msg;
}

bool CloudOcclusionPredictor::predict(const lanelet::ConstLineString3d& traffic_light)
{
  const float dis_thres = 0.3;
  const int num_thres = 3;
  if(history_clouds_.empty()){
    return false;
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(history_clouds_.front(), cloud);
  const double tl_height = traffic_light.attributeOr("height", 0.0);
  const auto & tl_left_down_point = traffic_light.front();
  const auto & tl_right_down_point = traffic_light.back();
  tf2::Vector3 tl_center((tl_left_down_point.x() + tl_right_down_point.x()) / 2, 
                          (tl_left_down_point.y() + tl_right_down_point.y()) / 2, 
                          (tl_left_down_point.z() + tl_right_down_point.z() + tl_height) / 2);
  tf2::Vector3 tl_cloud = tf_cloud2map_ * tl_center;
  tf2::Vector3 cam_cloud = tf_cloud2camera_.getOrigin();
  int num_occlusion = 0;
  for(const auto & pt : cloud){
    num_occlusion += closeToLineSegment(pt, cam_cloud, tl_cloud, dis_thres);
  }
  return num_occlusion >= num_thres;
}

bool CloudOcclusionPredictor::closeToLineSegment(const pcl::PointXYZ& pt, const tf2::Vector3& cam, const tf2::Vector3& tl, float dis_thres)
{
  tf2::Vector3 p1(pt.x, pt.y, pt.z);
  if(p1.distance(cam) <= 1 || p1.distance(tl) <= 2){
    return false;
  }
  float x1 = cam.getX();
  float y1 = cam.getY();
  float z1 = cam.getZ();
  float x2 = tl.getX();
  float y2 = tl.getY();
  float z2 = tl.getZ();

  float dx = x2 - x1;
  float dy = y2 - y1;
  float dz = z2 - z1;
  float ddx = pt.x - x1;
  float ddy = pt.y - y1;
  float ddz = pt.z - z1;

  float dot = ddx * dx + ddy * dy + ddz * dz;
  float len = dx * dx + dy * dy + dz * dz;
  if(len == 0){
    return false;
  }

  float param = dot / len;
  if(param > 0 && param < 1){
    float xx = x1 + param * dx;
    float yy = y1 + param * dy;
    float zz = z1 + param * dz;
    return tf2::Vector3(xx, yy, zz).distance(p1) <= dis_thres;
  }
  return false;
}
}