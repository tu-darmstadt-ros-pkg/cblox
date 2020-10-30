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

voxblox::Color func2(const voxblox::RGBVoxel* v) {
    voxblox::Color c;
    c.r = 255;
    c.g = 0;
    c.b = 255;
    c.a = 0;
    if(v == nullptr) {
        return c;
    }
    if (v->weight > 0.0)
        return v->color;

    return c;
}

void GenericSubmapServer::readConfig() {
    Transformation t;
    GenericMap<voxblox::TsdfVoxel>::Config c;
    c.voxel_size = 0.1;
    auto tsdf = std::make_shared<GenericSubmapCollection<voxblox::TsdfVoxel>>(c);

    GenericMap<voxblox::RGBVoxel>::Config c2;
    auto rgb = std::make_shared<GenericSubmapCollection<voxblox::RGBVoxel>>(c2);

    tsdf_maps_.push_back(tsdf);

    //TODO remove this workaround by fixing submapcollectionintegrator
    tsdf->createNewSubmap(t);
    rgb->createNewSubmap(t);

    auto lidar_sensor = std::make_shared<LIDARSensor<GenericSubmap<voxblox::TsdfVoxel>, voxblox::TsdfVoxel>>(nh_, nh_private_, "colored_cloud", "world", tsdf_maps_[0]);
    lidar_sensors_.push_back(lidar_sensor);
    std::cout << "config read " << std::endl;

    auto rgb_sensor = std::make_shared<RGBSensor<GenericSubmap<voxblox::RGBVoxel>, voxblox::TsdfVoxel>>(nh_, nh_private_, "/realsense_d435_back/color/image_raw", "/realsense_d435_back/color/camera_info", "world", tsdf_maps_[0], rgb);
    rgb_sensors_.push_back(rgb_sensor);

    std::cout << "added rgb" << std::endl;

    voxblox::MeshIntegratorConfig mesh_config
    = voxblox::getMeshIntegratorConfigFromRosParam(nh_private_);
    auto visualizer = std::make_shared<GenericActiveSubmapVisualizer<voxblox::TsdfVoxel, voxblox::TsdfVoxel>>(mesh_config, tsdf, tsdf, "test_topic", nh_, nh_private_);
    lidar_sensor->register_visualizer(visualizer);
    voxblox::Color (*f)(const voxblox::TsdfVoxel*) {& func};
    visualizer->setColorFunction(f);
    //visualizer->setUseColorMap();

    auto visualizer2 = std::make_shared<GenericActiveSubmapVisualizer<voxblox::TsdfVoxel, voxblox::RGBVoxel>>(mesh_config, tsdf, rgb, "test_topic2", nh_, nh_private_);
    rgb_sensor->register_visualizer(visualizer2);
    voxblox::Color (*f2)(const voxblox::RGBVoxel*) {& func2};
    visualizer2->setColorFunction(f2);


}

}

#endif  // CBLOX_ROS_GENERIC_SUBMAP_SERVER_INL_H_