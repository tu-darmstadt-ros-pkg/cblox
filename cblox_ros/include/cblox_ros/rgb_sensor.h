#ifndef CBLOX_ROS_RGB_SENSOR_H_
#define CBLOX_ROS_RGB_SENSOR_H_

#include <string>
#include <vector>

#include <ros/ros.h>
#include "cblox_ros/sensor.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cblox/core/voxel.h>
#include <cblox/integrator/projection_integrator.h>
#include "cblox/integrator/tsdf_integrator_wrapper.h"

namespace cblox {

struct RGBIntegration {

};
    //Change integratortype and data to correct one TODO
    template <typename SubmapType, typename GeometryVoxelType>
    class RGBSensor : public Sensor<RGBSensor<SubmapType, GeometryVoxelType>, SubmapType, sensor_msgs::Image::Ptr, voxblox::RGBVoxel, TsdfIntegratorWrapper, TsdfIntegrationData, GeometryVoxelType, voxblox::RGBVoxel>
    {
        public:
            RGBSensor (std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>> submap_collection_ptr,
                       ros::NodeHandle& nh,
                       ros::NodeHandle& nh_private,
                       std::string camera_image_topic,
                       std::string camera_info_topic,
                       std::string world_frame);

            virtual ~RGBSensor() {}

            void subscribeAndAdvertise(std::string camera_image_topic, 
                                         std::string camera_info_topic);

            void imageCb(const sensor_msgs::Image::Ptr& image_msg);
            
            void infoCb(const sensor_msgs::CameraInfo::Ptr& info_msg);

            void integrateMessage(const sensor_msgs::Image::Ptr msg, const Transformation T_G_C);

        protected:
            // Subscribers
            ros::Subscriber image_sub_;
            ros::Subscriber info_sub_;

            ros::NodeHandle nh_;
            ros::NodeHandle nh_private_;

            std::string world_frame_;
    };

}

#endif  // CBLOX_ROS_RGB_SENSOR_H_

#include "cblox_ros/rgb_sensor_inl.h"