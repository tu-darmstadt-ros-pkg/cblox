#ifndef CBLOX_ROS_CBLOX_FACTORY_H_
#define CBLOX_ROS_CBLOX_FACTORY_H_

#include <memory>

#include <cblox/integrator/generic_submap_collection_integrator.h>
#include <cblox_ros/lidar_sensor.h>
#include <cblox_ros/rgb_sensor.h>

namespace cblox {

class cblox_factory {
 public:
  // TODO template based and non template based functions (create TsdfRgbSensor,
  // etc.)

  template <typename SubmapType, typename CollisionVoxelType>
  static std::shared_ptr<RGBSensor<SubmapType, CollisionVoxelType>>
  createRGBSensor();

  template <typename SubmapType, typename GeometryVoxelType>
  static std::shared_ptr<LIDARSensor<SubmapType, GeometryVoxelType>>
  createLIDARSensor();

  // TODO needed, do different
  // static std::shared_ptr<GenericSubmapCollectionIntegrator>
  // createGenericSubmapCollectionIntegrator();
};
}  // namespace cblox

#endif  // CBLOX_ROS_CBLOX_FACTORY_H_