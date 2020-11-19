#include "cblox_ros/cblox_factory.h"

namespace cblox {
template <typename SubmapType, typename CollisionVoxelType>
static std::shared_ptr<RGBSensor<SubmapType, CollisionVoxelType>>
createRGBSensor() {}

template <typename SubmapType, typename GeometryVoxelType>
static std::shared_ptr<LIDARSensor<SubmapType, GeometryVoxelType>>
createLIDARSensor() {}
}  // namespace cblox