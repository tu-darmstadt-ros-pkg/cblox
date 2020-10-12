#ifndef CBLOX_ROS_RGB_SENSOR_INL_H_
#define CBLOX_ROS_RGB_SENSOR_INL_H_


namespace cblox {
  //TODO fix to correct integrators once they are implemented
    template <typename SubmapType, typename GeometryVoxelType>
    RGBSensor<SubmapType, GeometryVoxelType>::RGBSensor(std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>> submap_collection_ptr,
                                    ros::NodeHandle& nh,
                                    ros::NodeHandle& nh_private,
                                    std::string camera_image_topic,
                                    std::string camera_info_topic,
                                    std::string world_frame)
                                    : Sensor<RGBSensor<SubmapType, GeometryVoxelType>, SubmapType, sensor_msgs::Image::Ptr, voxblox::RGBVoxel, TsdfIntegratorWrapper, TsdfIntegrationData, GeometryVoxelType, voxblox::RGBVoxel>(/*submap_collection_ptr, */nh, nh_private, world_frame), 
                                      nh_(nh), 
                                      nh_private_(nh_private),
                                      world_frame_("world")
                                    {

        subscribeAndAdvertise(camera_image_topic, camera_info_topic);
    }

    template <typename SubmapType, typename GeometryVoxelType>
    void RGBSensor<SubmapType, GeometryVoxelType>::subscribeAndAdvertise(std::string camera_image_topic, 
                                                   std::string camera_info_topic) {
        image_sub_ = nh_.subscribe(camera_image_topic, 1, &RGBSensor<SubmapType, GeometryVoxelType>::imageCb, this);
        info_sub_ = nh_.subscribe(camera_info_topic, 1, &RGBSensor<SubmapType, GeometryVoxelType>::infoCb, this);

    }

    template <typename SubmapType, typename GeometryVoxelType>
    void RGBSensor<SubmapType, GeometryVoxelType>::imageCb(const sensor_msgs::Image::Ptr& image_msg) {
      //TODO
      this->addMessageToQueue(image_msg);
      this->serviceQueue();
    }

    template <typename SubmapType, typename GeometryVoxelType>
    void RGBSensor<SubmapType, GeometryVoxelType>::infoCb(const sensor_msgs::CameraInfo::Ptr& info_msg) {
      //TODO
      std::cout << "Got info" << std::endl;
    }

    template <typename SubmapType, typename GeometryVoxelType>
    void RGBSensor<SubmapType, GeometryVoxelType>::
    integrateMessage(const sensor_msgs::Image::Ptr msg, const Transformation T_G_C) {
      //TODO
      //integrator_ptr_->integrate(T_G_C, msg);
      std::cout << "integrating " << std::endl; 
    }

    

    
}
#endif  // CBLOX_ROS_RGB_SENSOR_INL_H_
