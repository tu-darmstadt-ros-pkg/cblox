#ifndef CBLOX_ROS_THERMAL_SENSOR_INL_H_
#define CBLOX_ROS_THERMAL_SENSOR_INL_H_



namespace cblox {
  //TODO fix to correct integrators once they are implemented
    template <typename SubmapType, typename GeometryVoxelType>
    ThermalSensor<SubmapType, GeometryVoxelType>::ThermalSensor(
                                    ros::NodeHandle& nh,
                                    ros::NodeHandle& nh_private,
                                    std::string camera_image_topic,
                                    std::string camera_info_topic,
                                    std::string world_frame,
                                    std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>> coll_submap_collection_ptr,
                                    std::shared_ptr<GenericSubmapCollection<voxblox::IntensityVoxel>> thermal_submap_collection_ptr)
                                    : Sensor<ThermalSensor<SubmapType, GeometryVoxelType>, SubmapType, sensor_msgs::Image::Ptr, voxblox::IntensityVoxel, ThermalProjectionIntegrator<GeometryVoxelType>, ProjectionData<float>, GeometryVoxelType, voxblox::IntensityVoxel>
                                    (/*submap_collection_ptr, */nh, nh_private, world_frame, thermal_submap_collection_ptr), 
                                      nh_(nh), 
                                      nh_private_(nh_private),
                                      world_frame_("world"),
                                      subsample_factor_(4),
                                      valid_info_(false),
                                      collision_submap_collection_ptr_(coll_submap_collection_ptr)
                                    {

        auto integ = std::make_shared<ThermalProjectionIntegrator<GeometryVoxelType>>(coll_submap_collection_ptr, thermal_submap_collection_ptr->getActiveMapPtr()->getLayerPtr());
        Sensor<ThermalSensor<SubmapType, GeometryVoxelType>, SubmapType, sensor_msgs::Image::Ptr, voxblox::IntensityVoxel, ThermalProjectionIntegrator<GeometryVoxelType>, ProjectionData<float>, GeometryVoxelType, voxblox::IntensityVoxel>::submap_collection_integrator_->setIntegrator(integ);
        subscribeAndAdvertise(camera_image_topic, camera_info_topic);

        last_parent_id_ = coll_submap_collection_ptr->getActiveSubmapID();
        Transformation t = coll_submap_collection_ptr->getActiveSubmapPose();
        //creating child map instead of normal map
        thermal_submap_collection_ptr->createNewChildSubMap(t, last_parent_id_);
    }

    template <typename SubmapType, typename GeometryVoxelType>
    void ThermalSensor<SubmapType, GeometryVoxelType>::subscribeAndAdvertise(std::string camera_image_topic, 
                                                   std::string camera_info_topic) {
        image_sub_ = nh_.subscribe(camera_image_topic, 1, &ThermalSensor<SubmapType, GeometryVoxelType>::imageCb, this);
        info_sub_ = nh_.subscribe(camera_info_topic, 1, &ThermalSensor<SubmapType, GeometryVoxelType>::infoCb, this);

    }

    template <typename SubmapType, typename GeometryVoxelType>
    void ThermalSensor<SubmapType, GeometryVoxelType>::imageCb(const sensor_msgs::Image::Ptr& image_msg) {
      this->addMessageToQueue(image_msg);
      this->serviceQueue();
    }

    template <typename SubmapType, typename GeometryVoxelType>
    void ThermalSensor<SubmapType, GeometryVoxelType>::infoCb(const sensor_msgs::CameraInfo::Ptr& info_msg) {
      
      fx_ = info_msg->P[0];
      valid_info_ = true;
    }

    template <typename SubmapType, typename GeometryVoxelType>
    void ThermalSensor<SubmapType, GeometryVoxelType>::
    integrateMessage(const sensor_msgs::Image::Ptr msg, const Transformation T_G_C) {


      //check if parent map changed
      if (last_parent_id_ != collision_submap_collection_ptr_->getActiveSubmapID()) {
        last_parent_id_ = collision_submap_collection_ptr_->getActiveSubmapID();
        Transformation t = collision_submap_collection_ptr_->getActiveSubmapPose();
        //creating child map instead of normal map
        this->submap_collection_ptr_->createNewChildSubMap(t, last_parent_id_);
      }


      if (!valid_info_) {
        ROS_WARN("No Camera Info received");
        return;
      }
      //conversion of image to vectors like intensity server  
      cblox::ProjectionData<float> data;

      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      CHECK(cv_ptr);

      const size_t num_pixels = cv_ptr->image.rows * cv_ptr->image.cols / subsample_factor_;

      float half_row = cv_ptr->image.rows / 2.0;
      float half_col = cv_ptr->image.cols / 2.0;

      //pre allocate
      data.bearing_vectors.reserve(num_pixels + 1);
      data.data.reserve(num_pixels + 1);

      size_t k = 0;
      size_t m = 0;

      for (int i = 0; i < cv_ptr->image.rows; i++) {
        for (int j = 0; j < cv_ptr->image.cols; j++) {
          //TODO subsampling
          if (m % subsample_factor_ == 0) {
            float intensity = cv_ptr->image.at<float>(i, j);
            data.data.push_back(intensity);
            data.bearing_vectors.push_back(T_G_C.getRotation().toImplementation() *
            Point(j - half_col, i - half_row, fx_).normalized());

          }
          m++;
        }
      }

      data.origin = T_G_C.getPosition();

      //passing identity matrix here, as data is in world frame
      Transformation identity;

      //start integration
      this->submap_collection_integrator_->integrate(identity, data);
    }

    

    
}
#endif  // CBLOX_ROS_RGB_SENSOR_INL_H_
