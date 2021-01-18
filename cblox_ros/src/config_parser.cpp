#include "cblox_ros/config_parser.h"

#include <XmlRpcException.h>
// TODO currently only tsdf
namespace cblox {

static voxblox::Color func3(const voxblox::IntensityVoxel* v) {
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

static voxblox::Color func2(const voxblox::RGBVoxel* v) {
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

static voxblox::Color func4(const voxblox::TsdfVoxel* v) {
  voxblox::Color c;
  c.r = 0;
  c.g = 0;
  c.b = 0;
  c.a = 0;
  if (v == nullptr) {
    return c;
  }
  c.r = 255;
  c.a = 255;

  return c;
}

void ConfigParser::parseConfig(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    std::shared_ptr<MapVariantsMap> map_collection,
    std::shared_ptr<SensorVariantsMap> sensor_collection,
    std::shared_ptr<VisualizerVariantsMap> visualizer_collection) {
  std::cout << "parsing config" << std::endl;

  // Maps
  /*if (!map_collection) {
      map_collection = std::make_shared<MapVariantsMap>();
  }*/
  XmlRpc::XmlRpcValue maps;
  if (!nh_private.getParam("maps", maps)) {
    ROS_ERROR("Couldn't load maps from config");
  } else {
    parseMaps(maps, map_collection);
  }

  // TODO maybe sharedptr init not working
  // Sensors
  /*if (!sensor_collection) {
      sensor_collection = std::make_shared<SensorVariantsMap>();
  }*/
  XmlRpc::XmlRpcValue sensors;
  if (!nh_private.getParam("sensors", sensors)) {
    ROS_ERROR("Couldn't load sensors from config");
  } else {
    parseSensors(nh, nh_private, sensors, sensor_collection, map_collection);
  }

  // Visualizers
  /*if (!visualizer_collection) {
      visualizer_collection = std::make_shared<VisualizerVariantsMap>();
  }*/
  XmlRpc::XmlRpcValue visualizers;
  if (!nh_private.getParam("visualizers", visualizers)) {
    ROS_ERROR("Couldn't load sensors from config");
  } else {
    parseVisualizers(nh, nh_private, visualizers, visualizer_collection,
                     map_collection);
  }
}

void ConfigParser::parseMaps(XmlRpc::XmlRpcValue maps,
                             std::shared_ptr<MapVariantsMap> map_collection) {
  // iterate over maps
  ROS_ASSERT(map.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < maps.size(); i++) {
    XmlRpc::XmlRpcValue map = maps[i];

    ROS_ASSERT(map.getType == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(map.getSize() == 3);
    ROS_ASSERT(map[0].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(map[1].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(map[2].getType() == XmlRpc::XmlRpcValue::TypeStruct);

    std::string type = static_cast<std::string>(map[1]);

    if (type.compare("tsdf") == 0) {
      std::cout << "creating tsdf map" << std::endl;
      map_collection->insert(
          std::pair<std::string,
                    typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr>(
              static_cast<std::string>(map[0]), parseTsdfMap(map[2])));
    }

    if (type.compare("rgb") == 0) {
      std::cout << "creating rgb map" << std::endl;
      map_collection->insert(
          std::pair<std::string,
                    typename GenericSubmapCollection<voxblox::RGBVoxel>::Ptr>(
              static_cast<std::string>(map[0]), parseRgbMap(map[2])));
    }

    if (type.compare("thermal") == 0) {
      std::cout << "creating thermal map" << std::endl;
      map_collection->insert(
          std::pair<std::string, typename GenericSubmapCollection<
                                     voxblox::IntensityVoxel>::Ptr>(
              static_cast<std::string>(map[0]), parseThermalMap(map[2])));
    }
  }
}

// TODO maybe set asserts?
typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr
ConfigParser::parseTsdfMap(XmlRpc::XmlRpcValue config) {
  GenericMap<voxblox::TsdfVoxel>::Config c;
  if (config.hasMember("voxel_size")) {
    c.voxel_size = static_cast<double>(config["voxel_size"]);
  }

  if (config.hasMember("voxels_per_side")) {
    c.voxels_per_side = static_cast<int>(config["voxels_per_side"]);
  }
  auto tsdf = std::make_shared<GenericSubmapCollection<voxblox::TsdfVoxel>>(c);
  // TODO fix by fixing submapcollectionintegrator
  Transformation t;
  tsdf->createNewSubmap(t);

  return tsdf;
}

typename GenericSubmapCollection<voxblox::RGBVoxel>::Ptr
ConfigParser::parseRgbMap(XmlRpc::XmlRpcValue config) {
  GenericMap<voxblox::RGBVoxel>::Config c;
  if (config.hasMember("voxel_size")) {
    c.voxel_size = static_cast<double>(config["voxel_size"]);
  }

  if (config.hasMember("voxels_per_side")) {
    c.voxels_per_side = static_cast<int>(config["voxels_per_side"]);
  }
  auto rgb = std::make_shared<GenericSubmapCollection<voxblox::RGBVoxel>>(c);
  // TODO fix by fixing submapcollectionintegrator
  Transformation t;
  rgb->createNewSubmap(t);
  return rgb;
}

typename GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr
ConfigParser::parseThermalMap(XmlRpc::XmlRpcValue config) {
  GenericMap<voxblox::IntensityVoxel>::Config c;
  if (config.hasMember("voxel_size")) {
    c.voxel_size = static_cast<double>(config["voxel_size"]);
  }

  if (config.hasMember("voxels_per_side")) {
    c.voxels_per_side = static_cast<int>(config["voxels_per_side"]);
  }
  auto intensity =
      std::make_shared<GenericSubmapCollection<voxblox::IntensityVoxel>>(c);
  // TODO fix by fixing submapcollectionintegrator
  Transformation t;
  intensity->createNewSubmap(t);
  return intensity;
}

void ConfigParser::parseSensors(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    XmlRpc::XmlRpcValue sensors,
    std::shared_ptr<SensorVariantsMap> sensor_collection,
    std::shared_ptr<MapVariantsMap> map_collection) {
  ROS_ASSERT(sensors.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < sensors.size(); i++) {
    XmlRpc::XmlRpcValue sensor = sensors[i];

    ROS_ASSERT(sensor.getType == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(sensor.getSize() == 4);
    ROS_ASSERT(sensor[0].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(sensor[1].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(sensor[2].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(sensor[3].getType() == XmlRpc::XmlRpcValue::TypeStruct);

    std::string type = static_cast<std::string>(sensor[0]);

    // TODO maybe more asserts?
    std::string collision_layer =
        static_cast<std::string>(sensor[2]["collision_layer"]);
    typename MapVariantsMap::iterator it;
    it = map_collection->find(collision_layer);

    if (it == map_collection->end()) {
      ROS_ERROR("Requested layer for sensor doesn't exist");
      continue;
    }
    typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr coll =
        boost::get<typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr>(
            it->second);

    if (type.compare("lidar_sensor") == 0) {
      std::cout << "creating lidar sensor" << std::endl;
      sensor_collection->insert(
          std::pair<std::string,
                    typename LIDARSensor<GenericSubmap<voxblox::TsdfVoxel>,
                                         voxblox::TsdfVoxel>::Ptr>(
              static_cast<std::string>(sensor[1]),
              parseLidarSensor(nh, nh_private, sensor[2], sensor[3], coll)));
    }

    if (type.compare("rgb_sensor") == 0) {
      std::cout << "creating rgb sensor" << std::endl;
      std::string rgb_layer = static_cast<std::string>(sensor[2]["rgb_layer"]);
      typename MapVariantsMap::iterator it2;
      it2 = map_collection->find(rgb_layer);
      if (it2 == map_collection->end()) {
        ROS_ERROR("Requested layer for sensor doesn't exist");
        continue;
      }
      typename GenericSubmapCollection<voxblox::RGBVoxel>::Ptr rgb =
          boost::get<typename GenericSubmapCollection<voxblox::RGBVoxel>::Ptr>(
              it2->second);
      sensor_collection->insert(
          std::pair<std::string,
                    typename RGBSensor<GenericSubmap<voxblox::RGBVoxel>,
                                       voxblox::TsdfVoxel>::Ptr>(
              static_cast<std::string>(sensor[1]),
              parseRgbSensor(nh, nh_private, sensor[2], sensor[3], coll, rgb)));
    }

    if (type.compare("thermal_sensor") == 0) {
      std::cout << "creating thermal sensor" << std::endl;
      std::string thermal_layer =
          static_cast<std::string>(sensor[2]["thermal_layer"]);
      typename MapVariantsMap::iterator it2;
      it2 = map_collection->find(thermal_layer);
      if (it2 == map_collection->end()) {
        ROS_ERROR("Requested layer for sensor doesn't exist");
        continue;
      }
      typename GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr thermal =
          boost::get<
              typename GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr>(
              it2->second);
      sensor_collection->insert(
          std::pair<std::string, typename ThermalSensor<
                                     GenericSubmap<voxblox::IntensityVoxel>,
                                     voxblox::TsdfVoxel>::Ptr>(
              static_cast<std::string>(sensor[1]),
              parseThermalSensor(nh, nh_private, sensor[2], sensor[3], coll,
                                 thermal)));
    }
  }
}

typename LIDARSensor<GenericSubmap<voxblox::TsdfVoxel>, voxblox::TsdfVoxel>::Ptr
ConfigParser::parseLidarSensor(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    XmlRpc::XmlRpcValue sensor_config, XmlRpc::XmlRpcValue integrator_config,
    typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map) {
  typename LIDARSensor<GenericSubmap<voxblox::TsdfVoxel>,
                       voxblox::TsdfVoxel>::Config sc;
  if (sensor_config.hasMember("pointcloud_topic")) {
    sc.pointcloud_topic =
        static_cast<std::string>(sensor_config["pointcloud_topic"]);
  }
  if (sensor_config.hasMember("frame")) {
    sc.frame = static_cast<std::string>(sensor_config["frame"]);
  }
  sc.submap_collection_ptr = tsdf_map;
  
  //integrator config
  auto integ_config = parseTsdfIntegratorConfig(integrator_config);

  auto lidar_sensor = std::make_shared<
      LIDARSensor<GenericSubmap<voxblox::TsdfVoxel>, voxblox::TsdfVoxel>>(
      nh, nh_private, sc, integ_config);

  return lidar_sensor;
}

typename RGBSensor<GenericSubmap<voxblox::RGBVoxel>, voxblox::TsdfVoxel>::Ptr
ConfigParser::parseRgbSensor(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    XmlRpc::XmlRpcValue sensor_config, XmlRpc::XmlRpcValue integrator_config,
    typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map,
    typename GenericSubmapCollection<voxblox::RGBVoxel>::Ptr rgb_map) {
  typename RGBSensor<GenericSubmap<voxblox::RGBVoxel>,
                     voxblox::TsdfVoxel>::Config sc;
  if (sensor_config.hasMember("camera_topic")) {
    sc.camera_topic = static_cast<std::string>(sensor_config["camera_topic"]);
  }
  if (sensor_config.hasMember("camera_info_topic")) {
    sc.camera_info_topic =
        static_cast<std::string>(sensor_config["camera_info_topic"]);
  }
  if (sensor_config.hasMember("frame")) {
    sc.frame = static_cast<std::string>(sensor_config["frame"]);
  }
  if (sensor_config.hasMember("sub_sample_factor")) {
    sc.sub_sample_factor =
        static_cast<double>(sensor_config["sub_sample_factor"]);
  }
  sc.coll_submap_collection_ptr = tsdf_map;
  sc.rgb_submap_collection_ptr = rgb_map;

  //integrator config
  auto integ_config = parseProjectionConfig(integrator_config);
  auto rgb_sensor = std::make_shared<
      RGBSensor<GenericSubmap<voxblox::RGBVoxel>, voxblox::TsdfVoxel>>(
      nh, nh_private, sc, integ_config);

  return rgb_sensor;
}

typename ThermalSensor<GenericSubmap<voxblox::IntensityVoxel>,
                       voxblox::TsdfVoxel>::Ptr
ConfigParser::parseThermalSensor(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    XmlRpc::XmlRpcValue sensor_config, XmlRpc::XmlRpcValue integrator_config,
    typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map,
    typename GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr
        thermal_map) {
  typename ThermalSensor<GenericSubmap<voxblox::IntensityVoxel>,
                         voxblox::TsdfVoxel>::Config sc;
  if (sensor_config.hasMember("camera_topic")) {
    sc.camera_topic = static_cast<std::string>(sensor_config["camera_topic"]);
  }
  if (sensor_config.hasMember("camera_info_topic")) {
    sc.camera_info_topic =
        static_cast<std::string>(sensor_config["camera_info_topic"]);
  }
  if (sensor_config.hasMember("frame")) {
    sc.frame = static_cast<std::string>(sensor_config["frame"]);
  }
  if (sensor_config.hasMember("sub_sample_factor")) {
    sc.sub_sample_factor =
        static_cast<double>(sensor_config["sub_sample_factor"]);
  }
  sc.coll_submap_collection_ptr = tsdf_map;
  sc.thermal_submap_collection_ptr = thermal_map;
  
  //integrator config
  auto integ_config = parseProjectionConfig(integrator_config);
  auto thermal_sensor =
      std::make_shared<ThermalSensor<GenericSubmap<voxblox::IntensityVoxel>,
                                     voxblox::TsdfVoxel>>(nh, nh_private, sc, integ_config);

  return thermal_sensor;
}

voxblox::TsdfIntegratorBase::Config
ConfigParser::parseTsdfIntegratorConfig(XmlRpc::XmlRpcValue integrator_config) {
  voxblox::TsdfIntegratorBase::Config config;
  if (integrator_config.hasMember("default_truncation_distance")) {
    config.default_truncation_distance =
      static_cast<double>(integrator_config["default_truncation_distance"]);
  }
  if (integrator_config.hasMember("max_weight")) {
    config.max_weight =
        static_cast<double>(integrator_config["max_weight"]);
  }
  if (integrator_config.hasMember("voxel_carving_enabled")) {
    config.voxel_carving_enabled =
        static_cast<bool>(integrator_config["voxel_carving_enabled"]);
  }
  if (integrator_config.hasMember("min_ray_length_m")) {
    config.min_ray_length_m =
        static_cast<double>(integrator_config["min_ray_length_m"]);
  }
  if (integrator_config.hasMember("max_ray_length_m")) {
    config.max_ray_length_m =
        static_cast<double>(integrator_config["max_ray_length_m"]);
  }
  if (integrator_config.hasMember("use_const_weight")) {
    config.use_const_weight =
        static_cast<bool>(integrator_config["use_const_weight"]);
  }
  if (integrator_config.hasMember("allow_clear")) {
    config.allow_clear =
        static_cast<bool>(integrator_config["allow_clear"]);
  }
  if (integrator_config.hasMember("use_weight_dropoff")) {
    config.use_weight_dropoff =
        static_cast<bool>(integrator_config["use_weight_dropoff"]);
  }
  if (integrator_config.hasMember("use_sparsity_compensation_factor")) {
    config.use_sparsity_compensation_factor =
        static_cast<bool>(integrator_config["use_sparsity_compensation_factor"]);
  }
  if (integrator_config.hasMember("sparsity_compensation_factor")) {
    config.sparsity_compensation_factor =
        static_cast<double>(integrator_config["sparsity_compensation_factor"]);
  }
  if (integrator_config.hasMember("integrator_threads")) {
    config.integrator_threads =
        static_cast<int>(integrator_config["integrator_threads"]);
  }
  if (integrator_config.hasMember("integration_order_mode")) {
    config.integration_order_mode =
        static_cast<std::string>(integrator_config["integration_order_mode"]);
  }
  if (integrator_config.hasMember("enable_anti_grazing")) {
    config.enable_anti_grazing =
        static_cast<bool>(integrator_config["enable_anti_grazing"]);
  }
  if (integrator_config.hasMember("start_voxel_subsampling_factor")) {
    config.start_voxel_subsampling_factor =
        static_cast<double>(integrator_config["start_voxel_subsampling_factor"]);
  }
  if (integrator_config.hasMember("max_consecutive_ray_collisions")) {
    config.max_consecutive_ray_collisions =
        static_cast<int>(integrator_config["max_consecutive_ray_collisions"]);
  }
  if (integrator_config.hasMember("clear_checks_every_n_frames")) {
    config.clear_checks_every_n_frames =
        static_cast<int>(integrator_config["clear_checks_every_n_frames"]);
  }
  if (integrator_config.hasMember("max_integration_time_s")) {
    config.max_integration_time_s =
        static_cast<double>(integrator_config["max_integration_time_s"]);
  }
  return config;
}

ProjectionIntegratorConfig
ConfigParser::parseProjectionConfig(XmlRpc::XmlRpcValue integrator_config) {
  ProjectionIntegratorConfig config;
  if (integrator_config.hasMember("max_weight")) {
    config.max_weight =
        static_cast<double>(integrator_config["max_weight"]);
  }
  if (integrator_config.hasMember("max_distance")) {
    config.max_distance =
        static_cast<double>(integrator_config["max_distance"]);
  }
  if (integrator_config.hasMember("prop_voxel_radius")) {
    config.prop_voxel_radius =
        static_cast<int>(integrator_config["prop_voxel_radius"]);
  }

  return config;
}

void ConfigParser::parseVisualizers(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    XmlRpc::XmlRpcValue visualizers,
    std::shared_ptr<VisualizerVariantsMap> visualizer_collection,
    std::shared_ptr<MapVariantsMap> map_collection) {
  ROS_ASSERT(visualizers.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < visualizers.size(); i++) {
    XmlRpc::XmlRpcValue visualizer = visualizers[i];

    ROS_ASSERT(visualizer.getType == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(visualizer.getSize() == 4);
    ROS_ASSERT(visualizer[0].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(visualizer[1].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(visualizer[2].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(visualizer[3].getType() == XmlRpc::XmlRpcValue::TypeStruct);

    std::string type = static_cast<std::string>(visualizer[1]);

    std::string collision_layer =
        static_cast<std::string>(visualizer[2]["collision_layer"]);
    typename MapVariantsMap::iterator it;
    it = map_collection->find(collision_layer);

    if (it == map_collection->end()) {
      ROS_ERROR("Requested layer for sensor doesn't exist");
      continue;
    }
    typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr coll =
        boost::get<typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr>(
            it->second);

    std::string color_layer =
        static_cast<std::string>(visualizer[2]["color_layer"]);
    typename MapVariantsMap::iterator it2;
    it2 = map_collection->find(color_layer);
    if (it2 == map_collection->end()) {
      ROS_ERROR("Requested layer for sensor doesn't exist");
      continue;
    }

    if (type.compare("tsdf") == 0) {
      std::cout << "creating visualizer for tsdf" << std::endl;
      typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf =
          boost::get<typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr>(
              it2->second);

      auto vis = parseTsdfVisualizer(nh, nh_private, visualizer[2],
                                     visualizer[3], coll, tsdf);
      visualizer_collection->insert(
          std::pair<std::string,
                    typename GenericActiveSubmapVisualizer<
                        voxblox::TsdfVoxel, voxblox::TsdfVoxel>::Ptr>(
              static_cast<std::string>(visualizer[1]), vis));

      //voxblox::Color (*f4)(const voxblox::TsdfVoxel*){&func4};
      //vis->setColorFunction(f4);
    }

    if (type.compare("rgb") == 0) {
      std::cout << "creating visualizer for rgb" << std::endl;
      typename GenericSubmapCollection<voxblox::RGBVoxel>::Ptr rgb =
          boost::get<typename GenericSubmapCollection<voxblox::RGBVoxel>::Ptr>(
              it2->second);
      auto vis = parseRgbVisualizer(nh, nh_private, visualizer[2],
                                    visualizer[3], coll, rgb);
      visualizer_collection->insert(
          std::pair<std::string,
                    typename GenericActiveSubmapVisualizer<
                        voxblox::TsdfVoxel, voxblox::RGBVoxel>::Ptr>(
              static_cast<std::string>(visualizer[1]), vis));
      voxblox::Color (*f2)(const voxblox::RGBVoxel*){&func2};
      vis->setColorFunction(f2);
      vis->setRemoveAlpha(true);
    }

    if (type.compare("thermal") == 0) {
      std::cout << "creating visualizer for thermal" << std::endl;
      typename GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr thermal =
          boost::get<
              typename GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr>(
              it2->second);
      auto vis = parseThermalVisualizer(nh, nh_private, visualizer[2],
                                        visualizer[3], coll, thermal);
      visualizer_collection->insert(
          std::pair<std::string,
                    typename GenericActiveSubmapVisualizer<
                        voxblox::TsdfVoxel, voxblox::IntensityVoxel>::Ptr>(
              static_cast<std::string>(visualizer[1]), vis));
      voxblox::Color (*f3)(const voxblox::IntensityVoxel*){&func3};
      vis->setColorFunction(f3);
      vis->setRemoveAlpha(true);
    }

    // TODO register visualizer here + add additional functions like coloring
    // here
  }
}

voxblox::MeshIntegratorConfig ConfigParser::parseMeshConfig(
    XmlRpc::XmlRpcValue config) {
  voxblox::MeshIntegratorConfig mesh_config;
  if (config.hasMember("use_color")) {
    mesh_config.use_color = static_cast<bool>(config["use_color"]);
  }

  if (config.hasMember("min_weight")) {
    mesh_config.min_weight = static_cast<double>(config["min_weight"]);
  }

  if (config.hasMember("integrator_threads")) {
    mesh_config.integrator_threads =
        static_cast<int>(config["integrator_threads"]);
  }
  return mesh_config;
}

// TODO reduce repitition by adding function for generic parsing??
typename GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                       voxblox::TsdfVoxel>::Ptr
ConfigParser::parseTsdfVisualizer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    XmlRpc::XmlRpcValue visualizer_config, XmlRpc::XmlRpcValue mesh_config,
    GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map,
    GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map2) {
  voxblox::MeshIntegratorConfig mesh_c = parseMeshConfig(mesh_config);
  typename GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                         voxblox::TsdfVoxel>::Config c;
  if (visualizer_config.hasMember("update_interval")) {
    c.update_interval =
        static_cast<double>(visualizer_config["update_interval"]);
  }
  if (visualizer_config.hasMember("topic")) {
    c.topic = static_cast<std::string>(visualizer_config["topic"]);
  }
  c.mesh_config = mesh_c;
  c.geometry_submap_collection_ptr = tsdf_map;
  c.color_submap_collection_ptr = tsdf_map2;

  auto visualizer = std::make_shared<
      GenericActiveSubmapVisualizer<voxblox::TsdfVoxel, voxblox::TsdfVoxel>>(
      nh, nh_private, c);

  return visualizer;
}

typename GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                       voxblox::RGBVoxel>::Ptr
ConfigParser::parseRgbVisualizer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    XmlRpc::XmlRpcValue visualizer_config, XmlRpc::XmlRpcValue mesh_config,
    GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map,
    GenericSubmapCollection<voxblox::RGBVoxel>::Ptr rgb_map) {
  voxblox::MeshIntegratorConfig mesh_c = parseMeshConfig(mesh_config);
  typename GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                         voxblox::RGBVoxel>::Config c;
  if (visualizer_config.hasMember("update_interval")) {
    c.update_interval =
        static_cast<double>(visualizer_config["update_interval"]);
  }
  if (visualizer_config.hasMember("topic")) {
    c.topic = static_cast<std::string>(visualizer_config["topic"]);
  }
  c.mesh_config = mesh_c;
  c.geometry_submap_collection_ptr = tsdf_map;
  c.color_submap_collection_ptr = rgb_map;

  auto visualizer = std::make_shared<
      GenericActiveSubmapVisualizer<voxblox::TsdfVoxel, voxblox::RGBVoxel>>(
      nh, nh_private, c);

  return visualizer;
}

typename GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                       voxblox::IntensityVoxel>::Ptr
ConfigParser::parseThermalVisualizer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    XmlRpc::XmlRpcValue visualizer_config, XmlRpc::XmlRpcValue mesh_config,
    GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map,
    GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr thermal_map) {
  voxblox::MeshIntegratorConfig mesh_c = parseMeshConfig(mesh_config);
  typename GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                         voxblox::IntensityVoxel>::Config c;
  if (visualizer_config.hasMember("update_interval")) {
    c.update_interval =
        static_cast<double>(visualizer_config["update_interval"]);
  }
  if (visualizer_config.hasMember("topic")) {
    c.topic = static_cast<std::string>(visualizer_config["topic"]);
  }
  c.mesh_config = mesh_c;
  c.geometry_submap_collection_ptr = tsdf_map;
  c.color_submap_collection_ptr = thermal_map;

  auto visualizer =
      std::make_shared<GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                                     voxblox::IntensityVoxel>>(
          nh, nh_private, c);

  return visualizer;
}



}  // namespace cblox