#include "traffic_light_map_based_detector/occlusion_predictor.hpp"

namespace
{
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

void SingleObjectPredictor::update(const autoware_auto_perception_msgs::msg::PredictedObject& obj, double stamp)
{
  // the timestamps must be increasing for binary search
  if(status_.empty() == false && stamp < status_.back().stamp){
    std::cout << "received old object!" << std::endl;
    return;
  }
  status_.push_back(ObjectStatus{stamp, 
    obj.kinematics.initial_pose_with_covariance.pose, 
    obj.kinematics.initial_twist_with_covariance.twist,
    obj.shape});
}

bool SingleObjectPredictor::predict(double stamp, ObjectStatus& object) const
{
  if(status_.empty()){
    return false;
  }
  // binary search the first status that has larger timestamp than stamp
  // low would be the result index
  int low = 0, high = status_.size();
  while(low < high){
    int mid = (low + high) / 2;
    if(status_[mid].stamp >= stamp){
      high = mid;
    }
    else{
      low = mid + 1;
    }
  }
  object.stamp = stamp;
  object.shape = status_.back().shape;
  // there's no status with larger timestamp
  // predict with last status
  if(low == int(status_.size())){
    const ObjectStatus& s2 = status_.back();
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
    const ObjectStatus& s1 = status_.front();
    assert(stamp <= s1.stamp);
    double dt = s1.stamp - stamp;
    double dx = -s1.twist.linear.x * dt;
    double dy = -s1.twist.linear.y * dt;
    double dz = -s1.twist.linear.z * dt;
    object.pose = tier4_autoware_utils::calcOffsetPose(s1.pose, dx, dy, dz);
  }
  // there's at least one with smaller timestamp and one with larger timestamp
  // do the interpolation of status[low - 1] and status[low]
  else{
    const ObjectStatus& s1 = status_[low - 1];
    const ObjectStatus& s2 = status_[low];
    assert(stamp >= s1.stamp && stamp < s2.stamp);
    double ratio = (stamp - s1.stamp) / (s2.stamp - s1.stamp);
    object.pose = tier4_autoware_utils::calcInterpolatedPose(s1.pose, s2.pose, ratio);
  }
  return true;
}

void ObjectsPredictor::update(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  assert(msg->header.frame_id == "map");
  double stamp = rclcpp::Time(msg->header.stamp).seconds();
  std::set<std::string> updated_ids;
  for(const auto & input_obj : msg->objects){
    std::string id_str = uuid2str(input_obj.object_id);
    if(tracked_objects_.count(id_str) == 0){
      tracked_objects_.emplace(id_str, SingleObjectPredictor());
    }
    tracked_objects_.find(id_str)->second.update(input_obj, stamp);
    updated_ids.insert(id_str);
  }
  // remove un-detected objects
  for(auto it = tracked_objects_.begin(); it != tracked_objects_.end();){
    if(updated_ids.count(it->first) == 0){
      it = tracked_objects_.erase(it);
    }
    else{
      it++;
    }
  }
}

autoware_auto_perception_msgs::msg::PredictedObjects 
  ObjectsPredictor::predict(const std_msgs::msg::Header::_stamp_type& input_stamp)
{
  autoware_auto_perception_msgs::msg::PredictedObjects res;
  res.header.frame_id = "map";
  res.header.stamp = input_stamp;
  double stamp = rclcpp::Time(input_stamp).seconds();
  int idx = 1;
  ObjectStatus predicted_obj;
  for(const auto & p : tracked_objects_){
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

void CloudOcclusionPredictor::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // sometimes the top lidar doesn't give any point. we need to filter out these clouds
  size_t cloud_size =  msg->width * msg->height;
  if(cloud_size >= 100000){
    history_clouds_.push_back(*msg);
  }  
}

void CloudOcclusionPredictor::perceptionObjectsCallback(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  objects_predictor_.update(msg);
}

void CloudOcclusionPredictor::update(
  const sensor_msgs::msg::CameraInfo& camera_info, const tf2_ros::Buffer& tf_buffer)
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
    // transformation between lidar and map when the cloud was captured
    map2cloud_ = tf_buffer.lookupTransform(
      "map", history_clouds_.front().header.frame_id, rclcpp::Time(history_clouds_.front().header.stamp),
      rclcpp::Duration::from_seconds(0.2));
    // transformation between map and camera when the image was captured
    camera2map_ = tf_buffer.lookupTransform(
      camera_info.header.frame_id, "map", rclcpp::Time(camera_info.header.stamp),
      rclcpp::Duration::from_seconds(0.2));
    map2base_ = tf_buffer.lookupTransform(
      "map", "base_link", rclcpp::Time(history_clouds_.front().header.stamp),
      rclcpp::Duration::from_nanoseconds(0.2));
  } catch (tf2::TransformException & ex) {
    std::cout << "Error: cannot get transform from map frame to " << history_clouds_.front().header.frame_id << std::endl;
    return;
  }

  cloudPreprocess(camera_info);
}

void CloudOcclusionPredictor::cloudPreprocess(const sensor_msgs::msg::CameraInfo& camera_info)
{
  // the points very close to the camera should not be used in the occlusion estimation,
  // since they could be noise or from the other nearby sensors
  const float min_dist_to_cam = 1.0f;
  lidar_rays_.clear();
  if(history_clouds_.empty()){
    return;
  }
  image_geometry::PinholeCameraModel pinhole_camera_model;
  pinhole_camera_model.fromCameraInfo(camera_info);
  // origin cloud in lidar frame
  pcl::PointCloud<pcl::PointXYZ> cloud_lidar;
  // cloud in camera frame
  pcl::PointCloud<pcl::PointXYZ> cloud_camera;
  // points that are within camera fov
  pcl::PointCloud<pcl::PointXYZ> cloud_in_camera_fov;
  pcl::fromROSMsg(history_clouds_.front(), cloud_lidar);
  pcl::transformPointCloud(cloud_lidar, cloud_camera, (tf2::transformToEigen(camera2map_) * tf2::transformToEigen(map2cloud_)).matrix());
  pcl::transformPointCloud(cloud_lidar, debug_cloud_, tf2::transformToEigen(map2cloud_).matrix());
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
      cloud_in_camera_fov.push_back(cloud_camera[i]);
    }
  }
  compensateObjectMovements(cloud_in_camera_fov, camera_info);
  for(const pcl::PointXYZ& pt : cloud_in_camera_fov){
    Ray ray = ::point2ray(pt);
    lidar_rays_[static_cast<int>(ray.azimuth)][static_cast<int>(ray.elevation)].push_back(ray);
  } 
}

void CloudOcclusionPredictor::compensateObjectMovements(
  pcl::PointCloud<pcl::PointXYZ>& cloud, 
  const sensor_msgs::msg::CameraInfo& camera_info)
{
  debug_cloud_.clear();
  pcl::PointCloud<pcl::PointXYZ> cloud_cloud_stamp, cloud_camera_stamp;
  // transform the cloud from camera frame to map frame
  pcl::transformPointCloud(cloud, cloud, tf2::transformToEigen(camera2map_).inverse().matrix());
  auto cloud_stamp = history_clouds_.front().header.stamp;
  auto camera_stamp = camera_info.header.stamp;
  const auto& objects_cloud_stamp = objects_predictor_.predict(cloud_stamp);
  const auto& objects_camera_stamp = objects_predictor_.predict(rclcpp::Time(camera_stamp) - rclcpp::Duration::from_seconds(0.12));
  for(const auto& object_cloud_stamp : objects_cloud_stamp.objects){
    const auto & old_position = object_cloud_stamp.kinematics.initial_pose_with_covariance.pose.position;
    for(const auto & object_camera_stamp : objects_camera_stamp.objects){
      if(object_cloud_stamp.object_id == object_camera_stamp.object_id){
        const auto & new_position = object_camera_stamp.kinematics.initial_pose_with_covariance.pose.position;
        double dx = new_position.x - old_position.x;
        double dy = new_position.y - old_position.y;
        double dz = new_position.z - old_position.z;
        autoware::common::geometry::BoundingBox3D bbox(object_cloud_stamp);
        for(auto & pt : cloud){
          if(bbox.contains(tf2::Vector3(pt.x, pt.y, pt.z))){
            cloud_cloud_stamp.push_back(pt);
            pt.x += dx;
            pt.y += dy;
            pt.z += dz;
            cloud_camera_stamp.push_back(pt);
          }
        }
        break;
      }
    }
  }
  cloud_objects_ = objects_cloud_stamp;
  camera_objects_ = objects_camera_stamp;
  pcl::transformPointCloud(cloud_camera_stamp, cloud_camera_stamp, tf2::transformToEigen(camera2map_).matrix());
  pcl::toROSMsg(cloud_cloud_stamp, cloud_cloud_stamp_);
  pcl::toROSMsg(cloud_camera_stamp, cloud_camera_stamp_);
  cloud_cloud_stamp_.header = history_clouds_.front().header;
  cloud_cloud_stamp_.header.frame_id = "map";
  cloud_camera_stamp_.header = camera_info.header;
  pcl::transformPointCloud(cloud, cloud, tf2::transformToEigen(camera2map_).matrix());
}

sensor_msgs::msg::PointCloud2 CloudOcclusionPredictor::debug(const std::vector<lanelet::ConstLineString3d>& traffic_lights)
{
  (void) traffic_lights;
  if(history_clouds_.empty()){
    return sensor_msgs::msg::PointCloud2();
  }
  // tf2::Vector3 translation;
  // tf2::Quaternion rotation;
  // tf2::fromMsg(map2base_.transform.translation, translation);
  // tf2::fromMsg(map2base_.transform.rotation, rotation);
  // pcl::PointCloud<pcl::PointXYZ> cloudInBox;
  // autoware::common::geometry::BoundingBox3D bbox3d(translation, rotation, 50.0, 30.0, 100.0);
  // pcl::PointCloud<pcl::PointXYZ> cloud = debug_cloud_;
  // debug_cloud_.clear();
  // for(const auto & pt : cloud){
  //   if(bbox3d.contains(tf2::Vector3(pt.x, pt.y, pt.z))){
  //     debug_cloud_.push_back(pt);
  //   }
  // }
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(debug_cloud_, msg);
  msg.header = history_clouds_.front().header;
  msg.header.frame_id = "map";
  debug_cloud_.clear();
  return msg;
}

pcl::PointCloud<pcl::PointXYZ> CloudOcclusionPredictor::sampleTrafficLight(
    const lanelet::ConstLineString3d& traffic_light, uint32_t horizontal_sample_num, uint32_t vertical_sample_num)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  double tl_height = traffic_light.attributeOr("height", 0.0);
  pcl::PointXYZ tl_top_left(traffic_light.front().x(), traffic_light.front().y(), traffic_light.front().z() + tl_height);
  pcl::PointXYZ tl_bottom_right(traffic_light.back().x(), traffic_light.back().y(), traffic_light.back().z());

  float x1 = tl_top_left.x;
  float y1 = tl_top_left.y;
  float z1 = tl_top_left.z;
  float x2 = tl_bottom_right.x;
  float y2 = tl_bottom_right.y;
  float z2 = tl_bottom_right.z;
  for(uint32_t i1 = 0; i1 < horizontal_sample_num; i1++){
    for(uint32_t i2 = 0; i2 < vertical_sample_num; i2++){
      float x = x1 + (x2 - x1) * i1 / (horizontal_sample_num - 1);
      float y = y1 + (y2 - y1) * i1 / (horizontal_sample_num - 1);
      float z = z1 + (z2 - z1) * i2 / (vertical_sample_num - 1);
      cloud.push_back(pcl::PointXYZ(x, y, z));
    }
  }
  return cloud;
}

uint32_t CloudOcclusionPredictor::predict(const lanelet::ConstLineString3d& traffic_light)
{
  const uint32_t horizontal_sample_num = 6;
  const uint32_t vertical_sample_num = 3;
  static_assert(horizontal_sample_num > 1 && vertical_sample_num > 1);
  const float azimuth_resolution = 0.5f;
  const float elevation_resolution = 0.5f;
  const float min_dist_from_occlusion_to_tl = 5.0f;

  pcl::PointCloud<pcl::PointXYZ> tl_sample_cloud = sampleTrafficLight(traffic_light, horizontal_sample_num, vertical_sample_num);
  // transform the traffic light sample cloud to camera frame from map frame
  pcl::transformPointCloud(tl_sample_cloud, tl_sample_cloud, tf2::transformToEigen(camera2map_).matrix());
  uint32_t occluded_num = 0;
  for(const pcl::PointXYZ& tl_pt : tl_sample_cloud){
    Ray tl_ray = ::point2ray(tl_pt);
    bool occluded = false;
    // the azimuth and elevation range to search for points that may occlude tl_pt
    int min_azimuth = static_cast<int>(tl_ray.azimuth - azimuth_resolution);
    int max_azimuth = static_cast<int>(tl_ray.azimuth + azimuth_resolution);
    int min_elevation = static_cast<int>(tl_ray.elevation - elevation_resolution);
    int max_elevation = static_cast<int>(tl_ray.elevation + elevation_resolution);
    /**
     * search among lidar rays whose azimuth and elevation angle are close to the tl_ray.
     * for a lidar ray r1 whose azimuth and elevation are very close to tl_pt,
     * and the distance from r1 to camera is smaller than the distance from tl_pt to camera,
     * then tl_pt is occluded by r1.
    */
    for(int azimuth = min_azimuth; azimuth <= max_azimuth && !occluded; azimuth++){
      for(int elevation = min_elevation; elevation <= max_elevation && !occluded; elevation++){
        for(const Ray & lidar_ray : lidar_rays_[azimuth][elevation]){
          if(std::abs(lidar_ray.azimuth - tl_ray.azimuth) <= azimuth_resolution
          && std::abs(lidar_ray.elevation - tl_ray.elevation) <= elevation_resolution
          && lidar_ray.dist < tl_ray.dist - min_dist_from_occlusion_to_tl){
            occluded = true;
            break;
          }
        }
      }
    }
    occluded_num += occluded;
  }
  return 100 * occluded_num / tl_sample_cloud.size();
}

float CloudOcclusionPredictor::getCloudDelay()
{
  return static_cast<float>(cloud_delay_);
}

}