#ifndef CBLOX_ROS_CONFIG_PARSER_H_
#define CBLOX_ROS_CONFIG_PARSER_H_

#include <XmlRpcValue.h>
#include <ros/ros.h>

#include <string>

#include <cblox_ros/definitions.h>

#include <cblox/core/voxel.h>
#include <cblox_ros/generic_active_submap_visualizer.h>
#include <cblox_ros/lidar_sensor.h>
#include <cblox_ros/rgb_sensor.h>
#include <cblox_ros/thermal_sensor.h>
#include <cblox_ros/pose_graph_updater.h>


namespace cblox {

class ConfigParser {
 public:
  static void parseConfig(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      std::shared_ptr<MapVariantsMap> map_collection,
      std::shared_ptr<SensorVariantsMap> sensor_collection,
      std::shared_ptr<VisualizerVariantsMap> visualizer_collection,
      PoseGraphUpdater::Ptr& pose_graph_updater);

 private:
  /*struct map_getter : public boost::static_visitor<> {
    void operator(GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr m) const
  {return m;} void operator(GenericSubmapCollection<voxblox::RGBVoxel>::Ptr m)
  const {return m;} void
  operator(GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr m) const
  {return m;}
  }*/

  static void parseMaps(XmlRpc::XmlRpcValue maps,
                        std::shared_ptr<MapVariantsMap> map_collection);
  static typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr parseTsdfMap(
      std::string name, XmlRpc::XmlRpcValue config);
  static typename GenericSubmapCollection<voxblox::RGBVoxel>::Ptr parseRgbMap(
      std::string name, XmlRpc::XmlRpcValue config);
  static typename GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr
  parseThermalMap(std::string name, XmlRpc::XmlRpcValue config);

  static void parseSensors(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private,
                           XmlRpc::XmlRpcValue sensors,
                           std::shared_ptr<SensorVariantsMap> sensor_collection,
                           std::shared_ptr<MapVariantsMap> map_collection);
  static typename LIDARSensor<GenericSubmap<voxblox::TsdfVoxel>,
                              voxblox::TsdfVoxel>::Ptr
  parseLidarSensor(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      XmlRpc::XmlRpcValue sensor_config, XmlRpc::XmlRpcValue integrator_config,
      typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map);
  static typename RGBSensor<GenericSubmap<voxblox::RGBVoxel>,
                            voxblox::TsdfVoxel>::Ptr
  parseRgbSensor(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      XmlRpc::XmlRpcValue sensor_config, XmlRpc::XmlRpcValue integrator_config,
      typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map,
      typename GenericSubmapCollection<voxblox::RGBVoxel>::Ptr rgb_map);
  static typename ThermalSensor<GenericSubmap<voxblox::IntensityVoxel>,
                                voxblox::TsdfVoxel>::Ptr
  parseThermalSensor(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      XmlRpc::XmlRpcValue sensor_config, XmlRpc::XmlRpcValue integrator_config,
      typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map,
      typename GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr
          thermal_map);

  static voxblox::TsdfIntegratorBase::Config
    parseTsdfIntegratorConfig(XmlRpc::XmlRpcValue integrator_config);

  static ProjectionIntegratorConfig
    parseProjectionConfig(XmlRpc::XmlRpcValue integrator_config);

  static voxblox::MeshIntegratorConfig parseMeshConfig(
      XmlRpc::XmlRpcValue config);
  static void parseVisualizers(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      XmlRpc::XmlRpcValue visualizers,
      std::shared_ptr<VisualizerVariantsMap> visualizer_collection,
      std::shared_ptr<MapVariantsMap> map_collection);
  static typename GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                                voxblox::TsdfVoxel>::Ptr
  parseTsdfVisualizer(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      XmlRpc::XmlRpcValue visualizer_config, XmlRpc::XmlRpcValue mesh_config,
      GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map,
      GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map2);
  static typename GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                                voxblox::RGBVoxel>::Ptr
  parseRgbVisualizer(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private,
                     XmlRpc::XmlRpcValue visualizer_config,
                     XmlRpc::XmlRpcValue mesh_config,
                     GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map,
                     GenericSubmapCollection<voxblox::RGBVoxel>::Ptr rgb_map);
  static typename GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                                voxblox::IntensityVoxel>::Ptr
  parseThermalVisualizer(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      XmlRpc::XmlRpcValue visualizer_config, XmlRpc::XmlRpcValue mesh_config,
      GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr tsdf_map,
      GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr thermal_map);

  static PoseGraphUpdater::Ptr parsePoseGraphSettings(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, XmlRpc::XmlRpcValue pose_graph_settings);
};
}  // namespace cblox

#endif  // CBLOX_ROS_CONFIG_PARSER_H_