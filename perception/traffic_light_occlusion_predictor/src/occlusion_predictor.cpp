#include "traffic_light_occlusion_predictor/occlusion_predictor.hpp"

namespace
{

traffic_light::Ray point2ray(const pcl::PointXYZ& pt)
{
  traffic_light::Ray ray;
  ray.dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
  ray.elevation = RAD2DEG(std::atan2(pt.y, std::hypot(pt.x, pt.z)));
  ray.azimuth = RAD2DEG(std::atan2(pt.x, pt.z));
  ray.pt = pt;
  return ray;
}

cv::Point3d projectPixel2Point3d(
  const cv::Point2d& pixel, 
  const image_geometry::PinholeCameraModel& pinhole_camera_model,
  double dist)
{
  cv::Point2d rectified_pixel = pinhole_camera_model.rectifyPoint(pixel);
  cv::Point3d uv_3d = pinhole_camera_model.projectPixelTo3dRay(rectified_pixel);
  uv_3d.x /= uv_3d.z;
  uv_3d.y /= uv_3d.z;
  return cv::Point3d(uv_3d.x * dist, uv_3d.y * dist, uv_3d.z * dist);
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
  const float length_offset = 1.0;
  const float width_offset = 0.5;
  const float height_offset = 2.0;
  assert(msg->header.frame_id == "map");
  double stamp = rclcpp::Time(msg->header.stamp).seconds();
  std::set<std::string> updated_ids;
  for(const auto & input_obj : msg->objects){
    std::string id_str = uuid2str(input_obj.object_id);
    if(tracked_objects_.count(id_str) == 0){
      tracked_objects_.emplace(id_str, SingleObjectPredictor());
    }
    autoware_auto_perception_msgs::msg::PredictedObject object = input_obj;
    object.shape.dimensions.x += length_offset;
    object.shape.dimensions.y += width_offset;
    object.shape.dimensions.z += height_offset;
    object.kinematics.initial_pose_with_covariance.pose.position.z += height_offset * 0.5;
    tracked_objects_.find(id_str)->second.update(object, stamp);
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

void CloudOcclusionPredictor::receivePointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // the timestamps must be increasing
  double history_stamp = rclcpp::Time(history_clouds_.back().header.stamp).seconds();
  double msg_stamp = rclcpp::Time(msg->header.stamp).seconds();
  if(history_clouds_.empty() == false && msg_stamp < history_stamp){
    return;
  }
  history_clouds_.push_back(*msg);
  // sometimes the top lidar doesn't give any point. we need to filter out these clouds
}

void CloudOcclusionPredictor::receivePerceptionObjects(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  objects_predictor_.update(msg);
}

void CloudOcclusionPredictor::update(
  const sensor_msgs::msg::CameraInfo& camera_info, 
  const tf2_ros::Buffer& tf_buffer,
  const std::vector<autoware_auto_perception_msgs::msg::TrafficLightRoi>& tl_rough_rois,
  bool enable_point_cloud_compensation)
{
  if(history_clouds_.empty()){
    return;
  }
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
    if(enable_point_cloud_compensation){
      // transformation between lidar and map when the cloud was captured
      map2cloud_ = tf2::transformToEigen(
        tf_buffer.lookupTransform(
        "map", history_clouds_.front().header.frame_id, rclcpp::Time(history_clouds_.front().header.stamp),
        rclcpp::Duration::from_seconds(0.2))).matrix();
      // transformation between map and camera when the image was captured
      camera2map_ = tf2::transformToEigen(
        tf_buffer.lookupTransform(
        camera_info.header.frame_id, "map", rclcpp::Time(camera_info.header.stamp),
        rclcpp::Duration::from_seconds(0.2))).matrix();
    }
    else{
      camera2cloud_ = tf2::transformToEigen(
        tf_buffer.lookupTransform(
          camera_info.header.frame_id, history_clouds_.front().header.frame_id, rclcpp::Time(camera_info.header.stamp),
          rclcpp::Duration::from_seconds(0.2))).matrix();
    }
    
  } catch (tf2::TransformException & ex) {
    std::cout << "Error: cannot get transform from map frame to " << history_clouds_.front().header.frame_id << std::endl;
    return;
  }
  
  cloudPreprocess(camera_info, tl_rough_rois, enable_point_cloud_compensation);
}

void CloudOcclusionPredictor::cloudPreprocess(
  const sensor_msgs::msg::CameraInfo& camera_info,
  const std::vector<autoware_auto_perception_msgs::msg::TrafficLightRoi>& tl_rough_rois,
  bool enable_point_cloud_compensation)
{
  lidar_rays_.clear();
  if(history_clouds_.empty()){
    return;
  }
  Eigen::Matrix4d camera2cloud = enable_point_cloud_compensation? camera2map_ * map2cloud_ : camera2cloud_;
  // points in camera frame
  pcl::PointCloud<pcl::PointXYZ> cloud_camera;
  // filtered points within traffic light rois in camera frame
  pcl::PointCloud<pcl::PointXYZ> cloud_in_rois;
  tier4_autoware_utils::transformPointCloudFromROSMsg(history_clouds_.front(), cloud_camera, camera2cloud);
  filterCloud(cloud_camera, tl_rough_rois, camera_info, enable_point_cloud_compensation, cloud_in_rois);
  
  if(enable_point_cloud_compensation){
    compensateObjectMovements(cloud_in_rois, camera_info);
  }
  
  for(const pcl::PointXYZ& pt : cloud_in_rois){
    Ray ray = ::point2ray(pt);
    lidar_rays_[static_cast<int>(ray.azimuth)][static_cast<int>(ray.elevation)].push_back(ray);
  }
}

void CloudOcclusionPredictor::filterCloud(
  const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
  const std::vector<autoware_auto_perception_msgs::msg::TrafficLightRoi>& tl_rough_rois,
  const sensor_msgs::msg::CameraInfo& camera_info,
  bool enable_point_cloud_compensation,
  pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
  cloud_out.clear();
  // the points very close to the camera should not be used in the occlusion estimation,
  // since they could be noise or from the other nearby sensors
  const float min_dist_to_cam = 1.0f;
  pinhole_camera_model_.fromCameraInfo(camera_info);

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  // the x and y coordiante of the rough rois are un-rectified coordiantes.
  // we need to convert them to rectified coordiantes.
  for(const auto & roi : tl_rough_rois){
    cv::Point2d top_left_pt(roi.roi.x_offset, roi.roi.y_offset);
    cv::Point2d bottom_right_pt(roi.roi.x_offset + roi.roi.width, roi.roi.y_offset + roi.roi.height);
    top_left_pt = pinhole_camera_model_.rectifyPoint(top_left_pt);
    bottom_right_pt = pinhole_camera_model_.rectifyPoint(bottom_right_pt);
    min_x = std::min(min_x, top_left_pt.x);
    max_x = std::max(max_y, bottom_right_pt.x);
    min_y = std::min(min_y, top_left_pt.y);
    max_y = std::max(max_y, bottom_right_pt.y);
  }
  if(enable_point_cloud_compensation){
    min_x = 0;
    max_x = camera_info.width - 1;
    min_y = 0;
    max_y = camera_info.height - 1;
  }

  for(size_t i = 0; i < cloud_in.size(); i++){
    if(cloud_in[i].z <= 0){
      continue;
    }
    float dist_to_cam = std::sqrt(cloud_in[i].x * cloud_in[i].x + cloud_in[i].y * cloud_in[i].y + cloud_in[i].z * cloud_in[i].z);
    if(dist_to_cam <= min_dist_to_cam){
      continue;
    }
    cv::Point2d pixel = pinhole_camera_model_.project3dToPixel(cv::Point3d(cloud_in[i].x, cloud_in[i].y, cloud_in[i].z));
    if(pixel.x >= min_x
    && pixel.x <= max_x
    && pixel.y >= min_y
    && pixel.y <= max_y){
      cloud_out.push_back(cloud_in[i]);
    }
  }
}

void CloudOcclusionPredictor::compensateObjectMovements(
  pcl::PointCloud<pcl::PointXYZ>& cloud, 
  const sensor_msgs::msg::CameraInfo& camera_info)
{
  // transform the cloud from camera frame to map frame
  Eigen::Matrix4d map2camera = camera2map_.inverse();
  pcl::transformPointCloud(cloud, cloud, map2camera);
  auto cloud_stamp = history_clouds_.front().header.stamp;
  auto camera_stamp = camera_info.header.stamp;
  const auto& objects_cloud_stamp = objects_predictor_.predict(cloud_stamp);
  const auto& objects_camera_stamp = objects_predictor_.predict(rclcpp::Time(camera_stamp));

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
            pt.x += dx;
            pt.y += dy;
            pt.z += dz;
          }
        }
        break;
      }
    }
  }
  pcl::transformPointCloud(cloud, cloud, camera2map_);
}

sensor_msgs::msg::PointCloud2 CloudOcclusionPredictor::debug(const sensor_msgs::msg::CameraInfo& camera_info)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(debug_cloud_, msg);
  msg.header = camera_info.header;
  debug_cloud_.clear();
  return msg;
}

void CloudOcclusionPredictor::sampleTrafficLightRoi(
  const geometry_msgs::msg::Vector3& roi_top_left,
  const geometry_msgs::msg::Vector3& roi_bottom_right,
  uint32_t horizontal_sample_num, 
  uint32_t vertical_sample_num,
  pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
  pcl::PointXYZ top_left(roi_top_left.x, roi_top_left.y, roi_top_left.z);
  pcl::PointXYZ bottom_right(roi_bottom_right.x, roi_bottom_right.y, roi_bottom_right.z);
  cloud_out.clear();
  float x1 = roi_top_left.x;
  float y1 = roi_top_left.y;
  float z1 = roi_top_left.z;
  float x2 = roi_bottom_right.x;
  float y2 = roi_bottom_right.y;
  float z2 = roi_bottom_right.z;
  for(uint32_t i1 = 0; i1 < horizontal_sample_num; i1++){
    for(uint32_t i2 = 0; i2 < vertical_sample_num; i2++){
      float x = x1 + (x2 - x1) * i1 / (horizontal_sample_num - 1);
      float y = y1 + (y2 - y1) * i2 / (vertical_sample_num - 1);
      float z = z1 + (z2 - z1) * i1 / (horizontal_sample_num - 1);
      cloud_out.push_back(pcl::PointXYZ(x, y, z));
    }
  }
}

void CloudOcclusionPredictor::sampleTrafficLightRoi(
    const autoware_auto_perception_msgs::msg::TrafficLightRoi& roi,
    uint32_t horizontal_sample_num, 
    uint32_t vertical_sample_num,
    pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
  cv::Point2d top_left_pixel(roi.roi.x_offset, roi.roi.y_offset);
  cv::Point2d bottom_right_pixel(roi.roi.x_offset + roi.roi.width, roi.roi.y_offset + roi.roi.height);
  cv::Point3d top_left = ::projectPixel2Point3d(top_left_pixel, pinhole_camera_model_, roi.dist);
  cv::Point3d bottom_right = ::projectPixel2Point3d(bottom_right_pixel, pinhole_camera_model_, roi.dist);
  cloud_out.clear();
  float x1 = top_left.x;
  float y1 = top_left.y;
  float z1 = top_left.z;
  float x2 = bottom_right.x;
  float y2 = bottom_right.y;
  float z2 = bottom_right.z;
  for(uint32_t i1 = 0; i1 < horizontal_sample_num; i1++){
    for(uint32_t i2 = 0; i2 < vertical_sample_num; i2++){
      float x = x1 + (x2 - x1) * i1 / (horizontal_sample_num - 1);
      float y = y1 + (y2 - y1) * i2 / (vertical_sample_num - 1);
      float z = z1 + (z2 - z1) * i1 / (horizontal_sample_num - 1);
      cloud_out.push_back(pcl::PointXYZ(x, y, z));
    }
  }
}

uint32_t CloudOcclusionPredictor::predict(
  const autoware_auto_perception_msgs::msg::TrafficLightRoi&roi, 
  double azimuth_occlusion_resolution,
  double elevation_occlusion_resolution)
{
  if(history_clouds_.empty()){
    return 0;
  }
  const uint32_t horizontal_sample_num = 20;
  const uint32_t vertical_sample_num = 20;
  static_assert(horizontal_sample_num > 1 && vertical_sample_num > 1);
  const float min_dist_from_occlusion_to_tl = 5.0f;

  pcl::PointCloud<pcl::PointXYZ> tl_sample_cloud;
  sampleTrafficLightRoi(roi, horizontal_sample_num, vertical_sample_num, tl_sample_cloud);
  debug_cloud_ += tl_sample_cloud;
  uint32_t occluded_num = 0;
  for(const pcl::PointXYZ& tl_pt : tl_sample_cloud){
    Ray tl_ray = ::point2ray(tl_pt);
    bool occluded = false;
    // the azimuth and elevation range to search for points that may occlude tl_pt
    int min_azimuth = static_cast<int>(tl_ray.azimuth - azimuth_occlusion_resolution);
    int max_azimuth = static_cast<int>(tl_ray.azimuth + azimuth_occlusion_resolution);
    int min_elevation = static_cast<int>(tl_ray.elevation - elevation_occlusion_resolution);
    int max_elevation = static_cast<int>(tl_ray.elevation + elevation_occlusion_resolution);
    /**
     * search among lidar rays whose azimuth and elevation angle are close to the tl_ray.
     * for a lidar ray r1 whose azimuth and elevation are very close to tl_pt,
     * and the distance from r1 to camera is smaller than the distance from tl_pt to camera,
     * then tl_pt is occluded by r1.
    */
    for(int azimuth = min_azimuth; (azimuth <= max_azimuth) && !occluded; azimuth++){
      for(int elevation = min_elevation; (elevation <= max_elevation) && !occluded; elevation++){
        for(const Ray & lidar_ray : lidar_rays_[azimuth][elevation]){
          if(std::abs(lidar_ray.azimuth - tl_ray.azimuth) <= azimuth_occlusion_resolution
          && std::abs(lidar_ray.elevation - tl_ray.elevation) <= elevation_occlusion_resolution
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