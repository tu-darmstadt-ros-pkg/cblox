#include "cblox/integrator/rgb_projection_integrator.h"

namespace cblox {

template <typename VoxelType1>
RGBProjectionIntegrator<VoxelType1>::RGBProjectionIntegrator(
    std::shared_ptr<GenericSubmapCollection<VoxelType1>> collision_collection,
    std::shared_ptr<GenericSubmapCollection<voxblox::RGBVoxel>> data_collection)
    : ProjectionIntegrator<RGBProjectionIntegrator<VoxelType1>, VoxelType1,
                           voxblox::RGBVoxel, Color>(collision_collection,
                                                     data_collection)
// msg_transformation_(msg_transformation)
{}

template <typename VoxelType1>
RGBProjectionIntegrator<VoxelType1>::RGBProjectionIntegrator(
    ProjectionConfig<VoxelType1, voxblox::RGBVoxel, Color> config)
    : ProjectionIntegrator<RGBProjectionIntegrator<VoxelType1>, VoxelType1,
                           voxblox::RGBVoxel, Color>(config) {}

template <typename VoxelType1>
void RGBProjectionIntegrator<VoxelType1>::integrationFunction(
    voxblox::RGBVoxel& voxel, Color& data) {
  // blend
  voxel.color = Color::blendTwoColors(voxel.color, voxel.weight, data, 1.0);
  voxel.weight += 1.0;

  if (voxel.weight > this->max_weight_) {
    voxel.weight = this->max_weight_;
  }
}
}  // namespace cblox
// explicit instantiations
template class cblox::RGBProjectionIntegrator<voxblox::TsdfVoxel>;