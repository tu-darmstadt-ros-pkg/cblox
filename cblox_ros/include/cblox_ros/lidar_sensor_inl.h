#ifndef CBLOX_ROS_LIDAR_SENSOR_INL_H_
#define CBLOX_ROS_LIDAR_SENSOR_INL_H_

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "cblox_ros/pointcloud_conversions.h"

namespace cblox {

template <typename SubmapType, typename GeometryVoxelType>
LIDARSensor<SubmapType, GeometryVoxelType>::LIDARSensor(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    std::string pointcloud_topic, std::string world_frame,
    std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>>
        submap_collection_ptr,
    voxblox::TsdfIntegratorBase::Config& integrator_config,
    int frames_per_submap, std::string integrator_type)
    : Sensor<LIDARSensor<SubmapType, GeometryVoxelType>, SubmapType,
             sensor_msgs::PointCloud2::Ptr, GeometryVoxelType,
             TsdfIntegratorWrapper, TsdfIntegrationData, GeometryVoxelType,
             GeometryVoxelType>(nh, nh_private, world_frame,
                                submap_collection_ptr, frames_per_submap),
      color_map_(new voxblox::GrayscaleColorMap()) {
  // submap_collection_ptr_ = submap_collection_ptr;

  Sensor<LIDARSensor<SubmapType, GeometryVoxelType>, SubmapType,
         sensor_msgs::PointCloud2::Ptr, GeometryVoxelType,
         TsdfIntegratorWrapper, TsdfIntegrationData, GeometryVoxelType,
         GeometryVoxelType>::num_integrated_frames_per_submap_ = 200;

  // TODO remove this workaround by fixing submapcollectionintegrator
  //voxblox::TsdfIntegratorBase::Config config;
  //config.default_truncation_distance = 0.4;
  TsdfConfig c(integrator_type, integrator_config, submap_collection_ptr);
  /*std::shared_ptr<TsdfIntegratorWrapper> integ =
      std::make_shared<TsdfIntegratorWrapper>(c);
  Sensor<LIDARSensor<SubmapType, GeometryVoxelType>, SubmapType,
         sensor_msgs::PointCloud2::Ptr, GeometryVoxelType,
         TsdfIntegratorWrapper, TsdfIntegrationData, GeometryVoxelType,
         GeometryVoxelType>::submap_collection_integrator_
      ->setIntegrator(integ);*/
  this->resetIntegrator(submap_collection_ptr, c);
  subscribeAndAdvertise(pointcloud_topic);
}

template <typename SubmapType, typename GeometryVoxelType>
LIDARSensor<SubmapType, GeometryVoxelType>::LIDARSensor(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    LIDARSensor<SubmapType, GeometryVoxelType>::Config c,
    voxblox::TsdfIntegratorBase::Config& integrator_config)
    : LIDARSensor(nh, nh_private, c.pointcloud_topic, c.frame,
                  c.submap_collection_ptr, integrator_config,
                  c.frames_per_submap, c.integrator_type) {}

template <typename SubmapType, typename GeometryVoxelType>
void LIDARSensor<SubmapType, GeometryVoxelType>::subscribeAndAdvertise(
    std::string pointcloud_topic) {
  pointcloud_sub_ = this->nh_.subscribe(
      pointcloud_topic, 1000,
      &LIDARSensor<SubmapType, GeometryVoxelType>::pointcloudCb, this);
}

template <typename SubmapType, typename GeometryVoxelType>
void LIDARSensor<SubmapType, GeometryVoxelType>::pointcloudCb(
    const sensor_msgs::PointCloud2::Ptr& msg) {
  this->addMessageToQueue(msg);
  this->serviceQueue();
}

template <typename SubmapType, typename GeometryVoxelType>
void LIDARSensor<SubmapType, GeometryVoxelType>::integrateMessage(
    const sensor_msgs::PointCloud2::Ptr msg, const Transformation& T_G_C) {
  // debug prints
  int id = static_cast<int>(this->submap_collection_ptr_->getActiveSubmapID());
  if (id >= 0) {
    auto blocks = (this->submap_collection_ptr_->getActiveSubmap())
                      .getNumberOfAllocatedBlocks();
  }
  // TODO freespace pointcloud

  // TODO think about adding extra step for message conversion

  // converting to voxblox pcl
  Pointcloud points_C;
  Colors colors;
  convertPointcloudMsg(*color_map_, msg, &points_C, &colors);

  // TODO verbose
  bool verbose_ = false;

  if (verbose_) {
    ROS_INFO("[CbloxServer] Integrating a pointcloud with %lu points.",
             points_C.size());
  }

  if (!this->mapInitialized()) {
    ROS_INFO("[CbloxServer] Initializing map.");
    this->initializeMap(T_G_C);
  }

  ros::WallTime start = ros::WallTime::now();
  TsdfIntegrationData data;
  data.points_C = points_C;
  data.colors = colors;
  data.freespace_points = false;
  data.id = this->submap_collection_ptr_->getActiveSubmapID();
  this->submap_collection_integrator_->integrate(T_G_C, data);

  ros::WallTime end = ros::WallTime::now();
  /*num_integrated_frames_current_submap_++;
  if (verbose_) {
      ROS_INFO(
          "[CbloxServer] Finished integrating in %f seconds, have %lu blocks. "
          "%u frames integrated to current submap.",
          (end - start).toSec(),
          submap_collection_ptr_->getActiveMap()
              .getTsdfLayer()
              .getNumberOfAllocatedBlocks(),
          num_integrated_frames_current_submap_);
  }*/
}

/*template <typename SubmapType, typename GeometryVoxelType>
void LIDARSensor<SubmapType, GeometryVoxelType>::
updateIntegratorSubmap() {
    submap_collection_integrator_->switchToActiveSubmap();
}*/

}  // namespace cblox

#endif  // CBLOS_ROS_LIDAR_SENSOR_INL_H_