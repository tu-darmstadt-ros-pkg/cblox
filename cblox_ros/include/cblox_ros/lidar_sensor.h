#ifndef CBLOX_ROS_LIDAR_SENSOR_H_
#define CBLOX_ROS_LIDAR_SENSOR_H_

#include "cblox/integrator/tsdf_integrator_wrapper.h"
#include "cblox/integrator/generic_submap_collection_integrator.h"
#include "sensor.h"
#include <sensor_msgs/PointCloud2.h>
#include <voxblox/utils/color_maps.h>

namespace cblox {


template <typename SubmapType, typename GeometryVoxelType>
class LIDARSensor : public Sensor<LIDARSensor<SubmapType, GeometryVoxelType>, SubmapType, sensor_msgs::PointCloud2::Ptr, GeometryVoxelType, TsdfIntegratorWrapper, TsdfIntegrationData>
{
    public:
        LIDARSensor (ros::NodeHandle& nh, 
                     ros::NodeHandle& nh_private, 
                     std::string pointcloud_topic,
                     std::string world_frame,
                     std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>> submap_collection_ptr);

        virtual ~LIDARSensor() {}

        void subscribeAndAdvertise(std::string pointcloud_topic);

        //void updateIntegratorSubmap();

        void pointcloudCb(const sensor_msgs::PointCloud2::Ptr& msg);

        void integrateMessage(const sensor_msgs::PointCloud2::Ptr msg, const Transformation& T_G_C);

    protected:
        ros::Subscriber pointcloud_sub_;
        std::string world_frame_;

        std::unique_ptr<voxblox::ColorMap> color_map_;

        //std::shared_ptr<SubmapCollection<SubmapType>> submap_collection_ptr_;
        //std::shared_ptr<GenericSubmapCollectionIntegrator<TsdfIntegratorWrapper, TsdfIntegrationData, GeometryVoxelType>> submap_collection_integrator_;
};
}
#endif //CBLOX_ROS_LIDAR_SENSOR_H_

#include "cblox_ros/lidar_sensor_inl.h"