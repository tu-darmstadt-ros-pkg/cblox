#ifndef CBLOX_ROS_THERMAL_SENSOR_INL_H_
#define CBLOX_ROS_THERMAL_SENSOR_INL_H_

namespace cblox {
// TODO fix to correct integrators once they are implemented
template <typename SubmapType, typename GeometryVoxelType>
ThermalSensor<SubmapType, GeometryVoxelType>::ThermalSensor(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    std::string camera_image_topic, std::string camera_info_topic,
    std::string world_frame, FloatingPoint subsample_factor,
    std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>>
        coll_submap_collection_ptr,
    std::shared_ptr<GenericSubmapCollection<voxblox::IntensityVoxel>>
        thermal_submap_collection_ptr,
    ProjectionIntegratorConfig& integrator_config, int frames_per_submap,
    bool normalize, FloatingPoint min_intensity, FloatingPoint max_intensity)
    : Sensor<ThermalSensor<SubmapType, GeometryVoxelType>, SubmapType,
             sensor_msgs::Image::Ptr, voxblox::IntensityVoxel,
             ThermalProjectionIntegrator<GeometryVoxelType>,
             ProjectionData<float>, GeometryVoxelType, voxblox::IntensityVoxel>(
          /*submap_collection_ptr, */ nh, nh_private, world_frame,
          thermal_submap_collection_ptr, frames_per_submap),
      nh_(nh),
      nh_private_(nh_private),
      world_frame_(world_frame),
      subsample_factor_(subsample_factor),
      valid_info_(false),
      collision_submap_collection_ptr_(coll_submap_collection_ptr),
      normalize_(normalize),
      min_intensity_(min_intensity),
      max_intensity_(max_intensity) {
  ProjectionConfig<GeometryVoxelType, voxblox::IntensityVoxel, float> config;
  config.geometry_collection = coll_submap_collection_ptr;
  config.data_collection = thermal_submap_collection_ptr;
  config.integrator_config = integrator_config;

  this->resetIntegrator(thermal_submap_collection_ptr, config);
  //TODO remove
  /*auto integ = std::make_shared<ThermalProjectionIntegrator<GeometryVoxelType>>(config);
  this->submap_collection_integrator_->setIntegrator(integ);*/

  subscribeAndAdvertise(camera_image_topic, camera_info_topic);

  last_parent_id_ = coll_submap_collection_ptr->getActiveSubmapID();
  Transformation t = coll_submap_collection_ptr->getActiveSubmapPose();
  // creating child map instead of normal map
  thermal_submap_collection_ptr->createNewChildSubMap(t, last_parent_id_);
}

template <typename SubmapType, typename GeometryVoxelType>
ThermalSensor<SubmapType, GeometryVoxelType>::ThermalSensor(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, Config c,
    ProjectionIntegratorConfig& integrator_config)
    : ThermalSensor(nh, nh_private, c.camera_topic, c.camera_info_topic,
                    c.frame, c.sub_sample_factor, c.coll_submap_collection_ptr,
                    c.thermal_submap_collection_ptr, integrator_config,
                    c.frames_per_submap, c.normalize, c.min_intensity,
                    c.max_intensity) {}

template <typename SubmapType, typename GeometryVoxelType>
void ThermalSensor<SubmapType, GeometryVoxelType>::subscribeAndAdvertise(
    std::string camera_image_topic, std::string camera_info_topic) {
  image_sub_ = nh_.subscribe(
      camera_image_topic, 1,
      &ThermalSensor<SubmapType, GeometryVoxelType>::imageCb, this);
  info_sub_ = nh_.subscribe(
      camera_info_topic, 1,
      &ThermalSensor<SubmapType, GeometryVoxelType>::infoCb, this);
}

template <typename SubmapType, typename GeometryVoxelType>
void ThermalSensor<SubmapType, GeometryVoxelType>::imageCb(
    const sensor_msgs::Image::Ptr& image_msg) {
  this->addMessageToQueue(image_msg);
  this->serviceQueue();
}

template <typename SubmapType, typename GeometryVoxelType>
void ThermalSensor<SubmapType, GeometryVoxelType>::infoCb(
    const sensor_msgs::CameraInfo::Ptr& info_msg) {
  fx_ = info_msg->P[0];
  valid_info_ = true;
}

template <typename SubmapType, typename GeometryVoxelType>
void ThermalSensor<SubmapType, GeometryVoxelType>::integrateMessage(
    const sensor_msgs::Image::Ptr msg, const Transformation T_G_C) {
  // check if parent map changed

  if (!this->pose_graph_mode_){
  if (last_parent_id_ !=
      collision_submap_collection_ptr_->getActiveSubmapID()) {
    last_parent_id_ = collision_submap_collection_ptr_->getActiveSubmapID();
    Transformation t = collision_submap_collection_ptr_->getActiveSubmapPose();
    // creating child map instead of normal map
    this->submap_collection_ptr_->createNewChildSubMap(t, last_parent_id_);
  }
  }

  if (!valid_info_) {
    ROS_WARN("No Camera Info received");
    return;
  }
  // conversion of image to vectors like intensity server
  cblox::ProjectionData<float> data;

  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
  CHECK(cv_ptr);

  // std::cout << cv_ptr->encoding << std::endl;
  // cv::Mat image;
  // TODO fix for mono 16 //write subclass for correct image handling in
  // sensors? trying to normalize image to range cv::normalize(cv_ptr->image,
  // image,0.,255.,cv::NORM_MINMAX,CV_8U);

  const size_t num_pixels =
      cv_ptr->image.rows * cv_ptr->image.cols / subsample_factor_;

  float half_row = cv_ptr->image.rows / 2.0;
  float half_col = cv_ptr->image.cols / 2.0;

  // pre allocate
  data.bearing_vectors.reserve(num_pixels + 1);
  data.data.reserve(num_pixels + 1);

  size_t k = 0;
  size_t m = 0;

  // double min_i = 1000000.0;
  // double max_i = 0.0;
  // like intensity server using image row
  for (int i = 0; i < cv_ptr->image.rows; i++) {
    for (int j = 0; j < cv_ptr->image.cols; j++) {
      // TODO subsampling
      if (m % subsample_factor_ == 0) {
        // TODO workaround for now, this should be fixed when possible
        // float intensity = cv_ptr->image.at<unsigned short>(i, j) / 65535.0;
        FloatingPoint intensity = cv_ptr->image.at<unsigned short>(i, j);
        /*if (intensity < min_i) {
          min_i = intensity;
        }
        if ( intensity > max_i) {
          max_i = intensity;
        }
        //std::cout << intensity << std::endl;
        if (normalize_) {
          intensity -= min_intensity_;
          intensity /= (max_intensity_ - min_intensity_);
        }*/

        data.data.push_back(intensity);
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

  // std::cout << "min/max" << min_i << " " << max_i << std::endl;
  // empty transformation as unused in this case
  Transformation trans;

  // start integration
  this->submap_collection_integrator_->integrate(trans, data);
}

}  // namespace cblox
#endif  // CBLOX_ROS_RGB_SENSOR_INL_H_
