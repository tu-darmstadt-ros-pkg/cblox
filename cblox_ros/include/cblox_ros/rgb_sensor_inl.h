#ifndef CBLOX_ROS_RGB_SENSOR_INL_H_
#define CBLOX_ROS_RGB_SENSOR_INL_H_

namespace cblox {
// TODO fix to correct integrators once they are implemented
template <typename SubmapType, typename GeometryVoxelType>
RGBSensor<SubmapType, GeometryVoxelType>::RGBSensor(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    std::string camera_image_topic, std::string camera_info_topic,
    std::string world_frame, double subsample_factor,
    std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>>
        coll_submap_collection_ptr,
    std::shared_ptr<GenericSubmapCollection<voxblox::RGBVoxel>>
        rgb_submap_collection_ptr,
    ProjectionIntegratorConfig& integrator_config, int frames_per_submap)
    : Sensor<RGBSensor<SubmapType, GeometryVoxelType>, SubmapType,
             sensor_msgs::Image::Ptr, voxblox::RGBVoxel,
             RGBProjectionIntegrator<GeometryVoxelType>, ProjectionData<Color>,
             GeometryVoxelType, voxblox::RGBVoxel>(
          /*submap_collection_ptr, */ nh, nh_private, world_frame,
          rgb_submap_collection_ptr, frames_per_submap),
      nh_(nh),
      nh_private_(nh_private),
      world_frame_(world_frame),
      subsample_factor_(subsample_factor),
      valid_info_(false),
      collision_submap_collection_ptr_(coll_submap_collection_ptr) {
  ProjectionConfig<GeometryVoxelType, voxblox::RGBVoxel, Color> config;
  config.geometry_collection = coll_submap_collection_ptr;
  config.data_collection = rgb_submap_collection_ptr;
  config.integrator_config = integrator_config;

  this->resetIntegrator(rgb_submap_collection_ptr, config);
  //TODO remove
  /*auto integ = std::make_shared<RGBProjectionIntegrator<GeometryVoxelType>>(config);
  this->submap_collection_integrator_->setIntegrator(integ);*/
  subscribeAndAdvertise(camera_image_topic, camera_info_topic);

  last_parent_id_ = coll_submap_collection_ptr->getActiveSubmapID();
  Transformation t = coll_submap_collection_ptr->getActiveSubmapPose();
  // creating child map instead of normal map
  rgb_submap_collection_ptr->createNewChildSubMap(t, last_parent_id_);
}

template <typename SubmapType, typename GeometryVoxelType>
RGBSensor<SubmapType, GeometryVoxelType>::RGBSensor(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, Config c,
    ProjectionIntegratorConfig& integrator_config)
    : RGBSensor(nh, nh_private, c.camera_topic, c.camera_info_topic, c.frame,
                c.sub_sample_factor, c.coll_submap_collection_ptr,
                c.rgb_submap_collection_ptr, integrator_config,
                c.frames_per_submap) {}

template <typename SubmapType, typename GeometryVoxelType>
void RGBSensor<SubmapType, GeometryVoxelType>::subscribeAndAdvertise(
    std::string camera_image_topic, std::string camera_info_topic) {
  image_sub_ =
      nh_.subscribe(camera_image_topic, 1,
                    &RGBSensor<SubmapType, GeometryVoxelType>::imageCb, this);
  info_sub_ =
      nh_.subscribe(camera_info_topic, 1,
                    &RGBSensor<SubmapType, GeometryVoxelType>::infoCb, this);
}

template <typename SubmapType, typename GeometryVoxelType>
void RGBSensor<SubmapType, GeometryVoxelType>::imageCb(
    const sensor_msgs::Image::Ptr& image_msg) {
  this->addMessageToQueue(image_msg);
  this->serviceQueue();
}

template <typename SubmapType, typename GeometryVoxelType>
void RGBSensor<SubmapType, GeometryVoxelType>::infoCb(
    const sensor_msgs::CameraInfo::Ptr& info_msg) {
  fx_ = info_msg->P[0];
  valid_info_ = true;
}

//vector in world frame from world center to image points
//passing transform to color layer frame to transform points into correct coordinates
template <typename SubmapType, typename GeometryVoxelType>
void RGBSensor<SubmapType, GeometryVoxelType>::integrateMessage(
    const sensor_msgs::Image::Ptr msg, const Transformation T_G_C) {
  // TODO check here if parent map is updated, if yes update integrator and
  // create new submap

  if (!this->pose_graph_mode_) {
    if (last_parent_id_ !=
        collision_submap_collection_ptr_->getActiveSubmapID()) {
      last_parent_id_ = collision_submap_collection_ptr_->getActiveSubmapID();
      Transformation t =
          collision_submap_collection_ptr_->getActiveSubmapPose();
      // creating child map instead of normal map
      this->submap_collection_ptr_->createNewChildSubMap(t, last_parent_id_);
    }
  }

  if (!valid_info_) {
    ROS_WARN("No Camera Info received");
    return;
  }
  // conversion of image to vectors like intensity server
  cblox::ProjectionData<voxblox::Color> data;

  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
  CHECK(cv_ptr);

  const size_t num_pixels =
      cv_ptr->image.rows * cv_ptr->image.cols / subsample_factor_;

  float half_row = cv_ptr->image.rows / 2.0;
  float half_col = cv_ptr->image.cols / 2.0;

  // pre allocate
  data.bearing_vectors.reserve(num_pixels + 1);
  data.data.reserve(num_pixels + 1);

  size_t k = 0;
  size_t m = 0;

  for (int i = 0; i < cv_ptr->image.rows; i++) {
    for (int j = 0; j < cv_ptr->image.cols; j++) {
      // TODO subsampling
      if (m % subsample_factor_ == 0) {
        cv::Vec3b v = cv_ptr->image.at<cv::Vec3b>(i, j);
        data.data.push_back(Color(v[0], v[1], v[2], 255));
        data.bearing_vectors.push_back(
            T_G_C.getRotation().toImplementation() *
            Point(j - half_col, i - half_row, fx_).normalized());
      }
      m++;
    }
  }

  data.origin = T_G_C.getPosition();

  data.geometry_id = last_parent_id_;
  data.id = this->submap_collection_ptr_->getActiveSubmapID();

  // empty transformation as unused in this case
  Transformation trans;

  // start integration
  this->submap_collection_integrator_->integrate(trans, data);
}

}  // namespace cblox
#endif  // CBLOX_ROS_RGB_SENSOR_INL_H_
