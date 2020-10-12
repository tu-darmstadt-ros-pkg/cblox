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

voxblox::Color func(const voxblox::TsdfVoxel* v) {
    voxblox::Color c;
    c.r = 0;
    c.g = 0;
    c.b = 0;
    c.a = 0;
    if(v == nullptr) {
        return c;
    }
    return v->color;
}

void GenericSubmapServer::readConfig() {
    GenericMap<voxblox::TsdfVoxel>::Config c;
    c.voxel_size = 0.1;
    auto tsdf = std::make_shared<GenericSubmapCollection<voxblox::TsdfVoxel>>(c);

    tsdf_maps_.push_back(tsdf);

    auto lidar_sensor = std::make_shared<LIDARSensor<GenericSubmap<voxblox::TsdfVoxel>, voxblox::TsdfVoxel>>(nh_, nh_private_, "colored_cloud", "world", tsdf_maps_[0]);
    lidar_sensors_.push_back(lidar_sensor);
    std::cout << "config read " << std::endl;

    voxblox::MeshIntegratorConfig mesh_config
    = voxblox::getMeshIntegratorConfigFromRosParam(nh_private_);
    auto visualizer = std::make_shared<GenericActiveSubmapVisualizer<voxblox::TsdfVoxel, voxblox::TsdfVoxel>>(mesh_config, tsdf, tsdf, "test_topic", nh_, nh_private_);
    lidar_sensor->register_visualizer(visualizer);
    voxblox::Color (*f)(const voxblox::TsdfVoxel*) {& func};
    //visualizer->setColorFunction(f);
    visualizer->setUseColorMap();
}

}

#endif  // CBLOX_ROS_GENERIC_SUBMAP_SERVER_INL_H_