#ifndef CBLOX_ROS_GENERIC_SUBMAP_SERVER_INL_H_
#define CBLOX_ROS_GENERIC_SUBMAP_SERVER_INL_H_

namespace cblox {

GenericSubmapServer::GenericSubmapServer(const ros::NodeHandle& nh,
                                         const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  readConfig();
}
/*template <typename SubmapType, typename CollisionVoxelType>
void GenericSubmapServer<SubmapType,
CollisionVoxelType>::add_sensor(std::shared_ptr<RGBSensor<SubmapType,
CollisionVoxelType>> rgb_sensor) { rgb_sensor_ = rgb_sensor;
}*/

voxblox::Color func(const voxblox::TsdfVoxel* v) {
  voxblox::Color c;
  c.r = 0;
  c.g = 0;
  c.b = 0;
  c.a = 0;
  if (v == nullptr) {
    return c;
  }
  return v->color;
}

voxblox::Color func2(const voxblox::RGBVoxel* v) {
  voxblox::Color c;
  c.r = 0;
  c.g = 0;
  c.b = 0;
  c.a = 0;
  if (v == nullptr) {
    return c;
  }
  if (v->weight > 0.0) return v->color;

  return c;
}

voxblox::Color func3(const voxblox::IntensityVoxel* v) {
  voxblox::Color c;
  c.r = 0;
  c.g = 0;
  c.b = 0;
  c.a = 0;
  if (v == nullptr) {
    return c;
  }
  if (v->weight > 0.0) {
    voxblox::IronbowColorMap m;
    return m.colorLookup(v->intensity);
  }

  return c;
}

void GenericSubmapServer::readConfig() {
  Transformation t;
  GenericMap<voxblox::TsdfVoxel>::Config c;
  c.voxel_size = 0.1;
  auto tsdf = std::make_shared<GenericSubmapCollection<voxblox::TsdfVoxel>>(c);

  GenericMap<voxblox::RGBVoxel>::Config c2;
  auto rgb = std::make_shared<GenericSubmapCollection<voxblox::RGBVoxel>>(c2);

  GenericMap<voxblox::IntensityVoxel>::Config c3;
  auto intensity =
      std::make_shared<GenericSubmapCollection<voxblox::IntensityVoxel>>(c3);

  tsdf_maps_.push_back(tsdf);
  rgb_maps_.push_back(rgb);
  intensity_maps_.push_back(intensity);

  // TODO remove this workaround by fixing submapcollectionintegrator
  tsdf->createNewSubmap(t);
  rgb->createNewSubmap(t);
  intensity->createNewSubmap(t);

  auto lidar_sensor = std::make_shared<
      LIDARSensor<GenericSubmap<voxblox::TsdfVoxel>, voxblox::TsdfVoxel>>(
      nh_, nh_private_, "/colored_cloud", "world", tsdf_maps_[0]);
  lidar_sensors_.push_back(lidar_sensor);
  std::cout << "config read " << std::endl;

  auto rgb_sensor = std::make_shared<
      RGBSensor<GenericSubmap<voxblox::RGBVoxel>, voxblox::TsdfVoxel>>(
      nh_, nh_private_, "/realsense_d435_back/color/image_raw",
      "/realsense_d435_back/color/camera_info", "world", tsdf_maps_[0], rgb);
  rgb_sensors_.push_back(rgb_sensor);
  std::cout << "added rgb" << std::endl;

  /*    auto thermal_sensor =
     std::make_shared<ThermalSensor<GenericSubmap<voxblox::IntensityVoxel>,
     voxblox::TsdfVoxel>>(nh_, nh_private_, "/arm_thermal_cam/image_raw",
     "/arm_thermal_cam/camera_info", "world", tsdf_maps_[0], intensity);
      thermal_sensors_.push_back(thermal_sensor);
      std::cout << "added thermal" << std::endl;*/

  voxblox::MeshIntegratorConfig mesh_config =
      voxblox::getMeshIntegratorConfigFromRosParam(nh_private_);
  auto visualizer = std::make_shared<
      GenericActiveSubmapVisualizer<voxblox::TsdfVoxel, voxblox::TsdfVoxel>>(
      mesh_config, tsdf, tsdf, "test_topic", nh_, nh_private_, 10.0);
  lidar_sensor->register_visualizer(visualizer);
  voxblox::Color (*f)(const voxblox::TsdfVoxel*){&func};
  // visualizer->setColorFunction(f);
  visualizer->setUseColorMap();

  auto visualizer2 = std::make_shared<
      GenericActiveSubmapVisualizer<voxblox::TsdfVoxel, voxblox::RGBVoxel>>(
      mesh_config, tsdf, rgb, "test_topic2", nh_, nh_private_, 10.0);
  rgb_sensor->register_visualizer(visualizer2);
  voxblox::Color (*f2)(const voxblox::RGBVoxel*){&func2};
  visualizer2->setColorFunction(f2);
  visualizer2->setRemoveAlpha(true);

  /*    auto visualizer3 =
     std::make_shared<GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
     voxblox::IntensityVoxel>>(mesh_config, tsdf, intensity, "test_topic3", nh_,
     nh_private_, 10.0); thermal_sensor->register_visualizer(visualizer3);
      voxblox::Color (*f3)(const voxblox::IntensityVoxel*) {& func3};
      visualizer3->setColorFunction(f3);*/
}

}  // namespace cblox

#endif  // CBLOX_ROS_GENERIC_SUBMAP_SERVER_INL_H_