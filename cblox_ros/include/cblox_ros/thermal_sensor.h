#ifndef CBLOX_ROS_THERMAL_SENSOR_H_
#define CBLOX_ROS_THERMAL_SENSOR_H_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <voxblox/core/voxel.h>
#include "cblox_ros/sensor.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cblox/integrator/thermal_projection_integrator.h>
#include <cv_bridge/cv_bridge.h>


namespace cblox {

    //Change integratortype and data to correct one TODO
    template <typename SubmapType, typename GeometryVoxelType>
    class ThermalSensor : public Sensor<ThermalSensor<SubmapType, GeometryVoxelType>, SubmapType, sensor_msgs::Image::Ptr, voxblox::IntensityVoxel, ThermalProjectionIntegrator<GeometryVoxelType>, ProjectionData<float>, GeometryVoxelType, voxblox::IntensityVoxel>
    {
        public:
            ThermalSensor (
                       ros::NodeHandle& nh,
                       ros::NodeHandle& nh_private,
                       std::string camera_image_topic,
                       std::string camera_info_topic,
                       std::string world_frame,
                       std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>> coll_submap_collection_ptr,
                       std::shared_ptr<GenericSubmapCollection<voxblox::IntensityVoxel>> thermal_submap_collection_ptr);

            virtual ~ThermalSensor() {}

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

            bool valid_info_;
            double fx_;

            size_t subsample_factor_;
    };

}

#endif  // CBLOX_ROS_THERMAL_SENSOR_H_

#include "cblox_ros/thermal_sensor_inl.h"