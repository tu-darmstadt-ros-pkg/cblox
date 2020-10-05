#ifndef CBLOX_ROS_GENERIC_SUBMAP_SERVER_INL_H_
#define CBLOX_ROS_GENERIC_SUBMAP_SERVER_INL_H_


namespace cblox {


GenericSubmapServer::
    GenericSubmapServer(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private) 
                        : nh_(nh), nh_private_(nh_private) {
    readConfig();
    }
/*template <typename SubmapType, typename CollisionVoxelType>
void GenericSubmapServer<SubmapType, CollisionVoxelType>::add_sensor(std::shared_ptr<RGBSensor<SubmapType, CollisionVoxelType>> rgb_sensor) {
    rgb_sensor_ = rgb_sensor;
}*/


void GenericSubmapServer::readConfig() {
    GenericMap<voxblox::TsdfVoxel>::Config c;
    auto tsdf = std::make_shared<GenericSubmapCollection<voxblox::TsdfVoxel>>(c);

    tsdf_maps_.push_back(tsdf);

    auto lidar_sensor = std::make_shared<LIDARSensor<GenericSubmap<voxblox::TsdfVoxel>, TsdfVoxel>>(nh_, nh_private_, "scan_cloud_filtered", "world", tsdf_maps_[0]);
    lidar_sensors_.push_back(lidar_sensor);
    std::cout << "config read " << std::endl;
}

}

#endif  // CBLOX_ROS_GENERIC_SUBMAP_SERVER_INL_H_