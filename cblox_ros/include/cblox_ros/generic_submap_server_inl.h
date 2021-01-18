#ifndef CBLOX_ROS_GENERIC_SUBMAP_SERVER_INL_H_
#define CBLOX_ROS_GENERIC_SUBMAP_SERVER_INL_H_

namespace cblox {

GenericSubmapServer::GenericSubmapServer(const ros::NodeHandle& nh,
                                         const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  readConfig(nh, nh_private);
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
  std::cout << "func2" << std::endl;
  voxblox::Color c;
  c.r = 0;
  c.g = 0;
  c.b = 0;
  c.a = 0;

  //debug
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

void GenericSubmapServer::readConfig(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private) {
  /*std::shared_ptr<MapVariantsMap> map_collection;
  std::shared_ptr<SensorVariantsMap> sensor_collection;
  std::shared_ptr<VisualizerVariantsMap> visualizer_collection;*/
  maps_ = std::make_shared<MapVariantsMap>();
  sensors_ = std::make_shared<SensorVariantsMap>();
  visualizers_ = std::make_shared<VisualizerVariantsMap>();
  ConfigParser::parseConfig(nh, nh_private, maps_, sensors_, visualizers_);
  
}

}  // namespace cblox

#endif  // CBLOX_ROS_GENERIC_SUBMAP_SERVER_INL_H_