#ifndef CBLOX_ROS_GENERIC_SUBMAP_SERVER_H_
#define CBLOX_ROS_GENERIC_SUBMAP_SERVER_H_

// This is a more generic approach to the submap server,
// allowing to add more information than pointclouds

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <cblox/core/tsdf_submap.h>
#include <cblox_ros/submap_server.h>
#include "cblox_ros/sensor.h"

#include "cblox/core/common.h"

#include <voxblox/core/color.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/voxel.h>
#include <voxblox/utils/color_maps.h>

#include <cblox/core/generic_map.h>
#include <cblox/core/voxel.h>
#include <cblox_ros/lidar_sensor.h>
#include <cblox_ros/rgb_sensor.h>
#include <cblox_ros/thermal_sensor.h>

#include "cblox/core/generic_submap.h"
#include "cblox/core/map_config.h"

#include <cblox/mesh/submap_mesher.h>

#include <cblox_ros/config_parser.h>
//#include <cblox_ros/rgb_sensor.h>

// TODO replace tsdf with geometry

namespace cblox {

class GenericSubmapServer {
 public:
  // Constructor
  explicit GenericSubmapServer(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private);
  virtual ~GenericSubmapServer() {}

  // void add_sensor(std::shared_ptr<RGBSensor<SubmapType, CollisionVoxelType>>
  // rgb_sensor);

  void readConfig(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // voxblox::Color func3(const voxblox::IntensityVoxel* v);

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // use map or dict later TODO
  std::vector<std::shared_ptr<GenericSubmapCollection<voxblox::TsdfVoxel>>>
      tsdf_maps_;
  std::vector<std::shared_ptr<GenericSubmapCollection<voxblox::RGBVoxel>>>
      rgb_maps_;
  std::vector<std::shared_ptr<GenericSubmapCollection<voxblox::IntensityVoxel>>>
      intensity_maps_;

  // std::vector<std::shared_ptr<RGBSensor>> rgb_sensors;
  std::vector<std::shared_ptr<
      LIDARSensor<GenericSubmap<voxblox::TsdfVoxel>, voxblox::TsdfVoxel>>>
      lidar_sensors_;
  std::vector<std::shared_ptr<
      RGBSensor<GenericSubmap<voxblox::RGBVoxel>, voxblox::TsdfVoxel>>>
      rgb_sensors_;
  std::vector<std::shared_ptr<ThermalSensor<
      GenericSubmap<voxblox::IntensityVoxel>, voxblox::TsdfVoxel>>>
      thermal_sensors_;
  // TODO error on compile with submap server

  std::shared_ptr<MapVariantsMap> maps_;
  std::shared_ptr<SensorVariantsMap> sensors_;
  std::shared_ptr<VisualizerVariantsMap> visualizers_;

  // voxblox::IronbowColorMap m_;
  // std::shared_ptr<RGBSensor<SubmapType, CollisionVoxelType>> rgb_sensor_;
};

}  // namespace cblox

#endif  // CBLOX_ROS_GENERIC_SUBMAP_SERVER_H_

#include "cblox_ros/generic_submap_server_inl.h"