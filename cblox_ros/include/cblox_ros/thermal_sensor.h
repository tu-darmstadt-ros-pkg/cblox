#ifndef CBLOX_ROS_THERMAL_SENSOR_H_
#define CBLOX_ROS_THERMAL_SENSOR_H_

#include <string>
#include <vector>

#include <cblox/integrator/thermal_projection_integrator.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <voxblox/core/voxel.h>
#include "cblox_ros/sensor.h"

namespace cblox {

// Change integratortype and data to correct one TODO
template <typename SubmapType, typename GeometryVoxelType>
class ThermalSensor
    : public Sensor<ThermalSensor<SubmapType, GeometryVoxelType>, SubmapType,
                    sensor_msgs::Image::Ptr, voxblox::IntensityVoxel,
                    ThermalProjectionIntegrator<GeometryVoxelType>,
                    ProjectionData<float>, GeometryVoxelType,
                    voxblox::IntensityVoxel> {
 public:
  typedef std::shared_ptr<ThermalSensor<SubmapType, GeometryVoxelType>> Ptr;
  struct Config {
    std::string camera_topic = "";
    std::string camera_info_topic = "";
    std::string frame = "";
    double sub_sample_factor = 1.0;
    typename GenericSubmapCollection<GeometryVoxelType>::Ptr
        coll_submap_collection_ptr;
    typename GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr
        thermal_submap_collection_ptr;
    int frames_per_submap = 20;
  };
  ThermalSensor(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      std::string camera_image_topic, std::string camera_info_topic,
      std::string world_frame, double subsample_factor,
      std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>>
          coll_submap_collection_ptr,
      std::shared_ptr<GenericSubmapCollection<voxblox::IntensityVoxel>>
          thermal_submap_collection_ptr,
      ProjectionIntegratorConfig& integrator_config, int frames_per_submap);
  ThermalSensor(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                Config c, ProjectionIntegratorConfig& integrator_config);

  virtual ~ThermalSensor() {}

  void subscribeAndAdvertise(std::string camera_image_topic,
                             std::string camera_info_topic);

  void imageCb(const sensor_msgs::Image::Ptr& image_msg);

  void infoCb(const sensor_msgs::CameraInfo::Ptr& info_msg);

  void integrateMessage(const sensor_msgs::Image::Ptr msg,
                        const Transformation T_G_C);

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

  std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>>
      collision_submap_collection_ptr_;

  SubmapID last_parent_id_;
};

}  // namespace cblox

#endif  // CBLOX_ROS_THERMAL_SENSOR_H_

#include "cblox_ros/thermal_sensor_inl.h"