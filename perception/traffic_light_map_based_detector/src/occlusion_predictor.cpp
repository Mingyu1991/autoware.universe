#include "traffic_light_map_based_detector/occlusion_predictor.hpp"

namespace
{
Eigen::Affine3d tf2Eigen(const tf2::Transform& tf)
{
  geometry_msgs::msg::TransformStamped stamped_tf;
  stamped_tf.transform = tf2::toMsg(tf);
  return tf2::transformToEigen(stamped_tf);
}

traffic_light::Ray point2ray(const pcl::PointXYZ& pt)
{
  traffic_light::Ray ray;
  ray.dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
  ray.elevation = RAD2DEG(std::atan2(pt.y, std::hypot(pt.x, pt.z)));
  ray.azimuth = RAD2DEG(std::atan2(pt.x, pt.z));
  return ray;
}

}


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
  // sometimes the top lidar doesn't give any point. we need to filter out these clouds
  size_t cloud_size =  msg->width * msg->height;
  if(cloud_size >= 100000){
    history_clouds_.push_back(*msg);
  }  
}

void CloudOcclusionPredictor::update(const sensor_msgs::msg::CameraInfo& camera_info,
                                     const tf2_ros::Buffer& tf_buffer)
{
  if(history_clouds_.empty()){
    return;
  }
  debug_cloud_.clear();
  /**
   * find the cloud whose timestamp is closest to the stamp of rough_rois
  */
  std::list<sensor_msgs::msg::PointCloud2>::iterator closest_it;
  double min_stamp_diff = std::numeric_limits<double>::max();
  for(auto it = history_clouds_.begin(); it != history_clouds_.end(); it++){
    double stamp_diff = std::abs(rclcpp::Time(it->header.stamp).seconds() - rclcpp::Time(camera_info.header.stamp).seconds());
    if(stamp_diff < min_stamp_diff){
      min_stamp_diff = stamp_diff;
      closest_it = it;
    }
  }
  cloud_delay_ = rclcpp::Time(camera_info.header.stamp).seconds() - rclcpp::Time(closest_it->header.stamp).seconds();
  /**
   * erase clouds earlier than closest_it since their timestamps couldn't be closer to following rough_rois than closest_it
  */
  for(auto it = history_clouds_.begin(); it != closest_it; ){
    it = history_clouds_.erase(it);
  }

  /**
   * get necessary tf information
  */
  try {
    geometry_msgs::msg::Transform transform = tf_buffer.lookupTransform(
      "map", history_clouds_.front().header.frame_id, rclcpp::Time(history_clouds_.front().header.stamp),
      rclcpp::Duration::from_seconds(0.2)).transform;
    tf_map2cloud_ = tf2::Transform (
      tf2::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
      tf2::Vector3(transform.translation.x, transform.translation.y, transform.translation.z));

    transform = tf_buffer.lookupTransform(
      "map", camera_info.header.frame_id, rclcpp::Time(camera_info.header.stamp),
      rclcpp::Duration::from_seconds(0.2)).transform;
    tf_map2camera_ = tf2::Transform (
      tf2::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
      tf2::Vector3(transform.translation.x, transform.translation.y, transform.translation.z));
  } catch (tf2::TransformException & ex) {
    std::cout << "Error: cannot get transform from map frame to " << history_clouds_.front().header.frame_id << std::endl;
    return;
  }  

  cloudPreprocess(camera_info);
}

void CloudOcclusionPredictor::cloudPreprocess(const sensor_msgs::msg::CameraInfo& camera_info)
{
  (void)camera_info;
  // the points very close to the camera should not be used in the occlusion estimation,
  // since they could be noise or from the other nearby sensors
  const float min_dist_to_cam = 1.0f;
  lidar_rays_.clear();
  cloud_size_= 0;
  if(history_clouds_.empty()){
    return;
  }
  image_geometry::PinholeCameraModel pinhole_camera_model;
  pinhole_camera_model.fromCameraInfo(camera_info);
  // origin cloud in lidar frame
  pcl::PointCloud<pcl::PointXYZ> cloud_lidar;
  // cloud in map frame
  pcl::PointCloud<pcl::PointXYZ> cloud_map;
  // cloud in camera frame
  pcl::PointCloud<pcl::PointXYZ> cloud_camera;
  pcl::fromROSMsg(history_clouds_.front(), cloud_lidar);
  // todo: split dynamic pts from static pts
  pcl::PointCloud<pcl::PointXYZ> static_pts_map;
  pcl::transformPointCloud(cloud_lidar, cloud_map, ::tf2Eigen(tf_map2cloud_));
  pcl::transformPointCloud(cloud_map, cloud_camera, ::tf2Eigen(tf_map2camera_.inverse()));
  // the points in map frame that are within camera fov
  pcl::PointCloud<pcl::PointXYZ> cloud_in_camera_fov;
  float min_azimuth = std::numeric_limits<float>::max();
  float max_azimuth = std::numeric_limits<float>::lowest();
  float min_elevation = std::numeric_limits<float>::max();
  float max_elevation = std::numeric_limits<float>::lowest();
  for(size_t i = 0; i < cloud_camera.size(); i++){
    if(cloud_camera[i].z <= 0){
      continue;
    }
    float dist_to_cam = std::sqrt(cloud_camera[i].x * cloud_camera[i].x + cloud_camera[i].y * cloud_camera[i].y + cloud_camera[i].z * cloud_camera[i].z);
    if(dist_to_cam <= min_dist_to_cam){
      continue;
    }
    cv::Point2d pixel = pinhole_camera_model.project3dToPixel(cv::Point3d(cloud_camera[i].x, cloud_camera[i].y, cloud_camera[i].z));
    if(pixel.x >= 0 
    && pixel.x < camera_info.width 
    && pixel.y >= 0 
    && pixel.y < camera_info.height){
      cloud_in_camera_fov.push_back(cloud_map[i]);
      Ray ray = ::point2ray(cloud_camera[i]);
      lidar_rays_[static_cast<int>(ray.azimuth)][static_cast<int>(ray.elevation)].push_back(ray);
      min_azimuth = std::min(min_azimuth, ray.azimuth);
      max_azimuth = std::max(max_azimuth, ray.azimuth);
      min_elevation = std::min(min_elevation, ray.elevation);
      max_elevation = std::max(max_elevation, ray.elevation);
      cloud_size_++;
    }
  }
  max_azimuth_range_ = max_azimuth - min_azimuth;
  max_elevation_range_ = max_elevation - min_elevation;
  static_pts_ = cloud_in_camera_fov;
  dynamic_pts_.clear();
}

sensor_msgs::msg::PointCloud2 CloudOcclusionPredictor::debug(const std::vector<lanelet::ConstLineString3d>& traffic_lights)
{
  (void) traffic_lights;
  if(history_clouds_.empty()){
    return sensor_msgs::msg::PointCloud2();
  }
  debug_cloud_.clear();
  //debug_cloud_.push_back(pcl::PointXYZ(tf_map2camera_.getOrigin().x(), tf_map2camera_.getOrigin().y(), tf_map2camera_.getOrigin().z()));
  debug_cloud_.push_back(pcl::PointXYZ(0, 0, 0));
  for(const auto & traffic_light : traffic_lights){
    double tl_height = traffic_light.attributeOr("height", 0.0);
    tf2::Vector3 tl_bottom_left(traffic_light.front().x(), traffic_light.front().y(), traffic_light.front().z());
    tf2::Vector3 tl_bottom_right(traffic_light.back().x(), traffic_light.back().y(), traffic_light.back().z());
    tf2::Vector3 tl_top_left(traffic_light.front().x(), traffic_light.front().y(), traffic_light.front().z() + tl_height);
    // transform the traffic light from map frame to camera frame
    tl_top_left = tf_map2camera_.inverse() * tl_top_left;
    tl_bottom_right = tf_map2camera_.inverse() * tl_bottom_right;
    debug_cloud_.push_back(pcl::PointXYZ(tl_top_left.x(), tl_top_left.y(), tl_top_left.z()));
    debug_cloud_.push_back(pcl::PointXYZ(tl_bottom_right.x(), tl_bottom_right.y(), tl_bottom_right.z()));
  }
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(debug_cloud_, msg);
  msg.header = history_clouds_.front().header;
  msg.header.frame_id = "traffic_light_left_camera/camera_optical_link";
  return msg;
}

uint32_t CloudOcclusionPredictor::predict(const lanelet::ConstLineString3d& traffic_light)
{
  // const float dis_thres = 0.3;
  // if(history_clouds_.empty()){
  //   return false;
  // }


  // const double tl_height = traffic_light.attributeOr("height", 0.0);
  // const auto & tl_left_down_point = traffic_light.front();
  // const auto & tl_right_down_point = traffic_light.back();
  // tf2::Vector3 tl_center((tl_left_down_point.x() + tl_right_down_point.x()) / 2, 
  //                         (tl_left_down_point.y() + tl_right_down_point.y()) / 2, 
  //                         (tl_left_down_point.z() + tl_right_down_point.z() + tl_height) / 2);
  // tf2::Vector3 cam_position = tf_map2camera_.getOrigin();
  // uint32_t num_occlusion = 0;
  // for(const auto & pt : static_pts_){
  //   auto tmp = num_occlusion;
  //   num_occlusion += closeToLineSegment(pt, cam_position, tl_center, dis_thres);
  //   if(num_occlusion > tmp){
  //     debug_cloud_.push_back(pt);
  //   }
  // }
  // return num_occlusion;


  const float azimuth_window_size = 1.5f;
  const float elevation_window_size = 1.5f;
  const float min_dist_from_occlusion_to_tl = 5.0f;

  double tl_height = traffic_light.attributeOr("height", 0.0);
  tf2::Vector3 tl_bottom_left(traffic_light.front().x(), traffic_light.front().y(), traffic_light.front().z());
  tf2::Vector3 tl_bottom_right(traffic_light.back().x(), traffic_light.back().y(), traffic_light.back().z());
  tf2::Vector3 tl_top_left(traffic_light.front().x(), traffic_light.front().y(), traffic_light.front().z() + tl_height);
  tf2::Vector3 tl_center(
    (tl_top_left.x() + tl_bottom_right.x()) * 0.5,
    (tl_top_left.y() + tl_bottom_right.y()) * 0.5,
    (tl_top_left.z() + tl_bottom_right.z()) * 0.5);
  // transform the traffic light from map frame to camera frame
  tl_top_left = tf_map2camera_.inverse() * tl_top_left;
  tl_bottom_right = tf_map2camera_.inverse() * tl_bottom_right;
  tl_center = tf_map2camera_.inverse() * tl_center;
  // calculate the azimuth and elevation range of the traffic light in camera frame
  Ray tl_top_left_ray = ::point2ray(pcl::PointXYZ(tl_top_left.x(), tl_top_left.y(), tl_top_left.z()));
  Ray tl_bottom_right_ray = ::point2ray(pcl::PointXYZ(tl_bottom_right.x(), tl_bottom_right.y(), tl_bottom_right.z()));

  float tl_azimuth_range = tl_bottom_right_ray.azimuth - tl_top_left_ray.azimuth;
  float tl_elevation_range = tl_bottom_right_ray.elevation - tl_top_left_ray.elevation;

  // the average azimuth angle and elevation angle of the traffic light
  float azimuth_aver = (tl_bottom_right_ray.azimuth + tl_top_left_ray.azimuth) * 0.5;
  float elevation_aver = (tl_bottom_right_ray.elevation + tl_top_left_ray.elevation) * 0.5;
  
  int point_num_in_window = 0;
  for(float azimuth = azimuth_aver - azimuth_window_size; azimuth <= azimuth_aver + azimuth_window_size; azimuth += 1.0f){
    for(float elevation = elevation_aver - elevation_window_size; elevation <= elevation_aver + elevation_window_size; elevation += 1.0f){
      int azimuth_int = static_cast<int>(azimuth);
      int elevation_int = static_cast<int>(elevation);
      point_num_in_window += lidar_rays_[azimuth_int][elevation_int].size();
    }
  }
  

  if(max_azimuth_range_ < 1e-2 || max_elevation_range_ < 1e-2){
    std::cout << "Warning: azimuth range or elevation range close to zero!" << std::endl;
    return false;
  }
  int tl_expect_pt_size;
  if(point_num_in_window != 0){
    tl_expect_pt_size = std::abs(point_num_in_window * tl_azimuth_range * tl_elevation_range / (4 * azimuth_window_size * elevation_window_size));
  }
  else{
    tl_expect_pt_size = std::abs(cloud_size_ * tl_azimuth_range * tl_elevation_range / max_azimuth_range_ / max_elevation_range_);
  }
  (void)tl_expect_pt_size;
  uint32_t occlusion_num = 0;
  /**
   * count the number of point within the traffic light area and closer than the traffic light
  */
  for(float azimuth = tl_top_left_ray.azimuth; azimuth <= tl_bottom_right_ray.azimuth + 1.0f; azimuth += 1.0f){
    for(float elevation = tl_top_left_ray.elevation; elevation <= tl_bottom_right_ray.elevation + 1.0f; elevation += 1.0f){
      int azimuth_int = static_cast<int>(azimuth);
      int elevation_int = static_cast<int>(elevation);
      for(const auto & ray : lidar_rays_[azimuth_int][elevation_int]){
        if(ray.azimuth >= tl_top_left_ray.azimuth
        && ray.azimuth <= tl_bottom_right_ray.azimuth
        && ray.elevation >= tl_top_left_ray.elevation
        && ray.elevation <= tl_bottom_right_ray.elevation){
          if(ray.dist <= tl_center.length() - min_dist_from_occlusion_to_tl){
            occlusion_num++;
          }
        }
      }
    }
  }
  // std::cout << "tl_azimuth_range = " << tl_azimuth_range
  //   << ", tl_elevation_range = " << tl_elevation_range
  //   << ", max_azimuth_range_ = " << max_azimuth_range_
  //   << ", max_elevation_range_ = " << max_elevation_range_
  //   << ", tl_expect_pt_size = " << tl_expect_pt_size << std::endl;
  return occlusion_num;
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

float CloudOcclusionPredictor::getCloudDelay()
{
  return static_cast<float>(cloud_delay_);
}

}