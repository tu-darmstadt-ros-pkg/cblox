#include "cblox/integrator/thermal_projection_integrator.h"

namespace cblox {

template <typename VoxelType1>
ThermalProjectionIntegrator<VoxelType1>::ThermalProjectionIntegrator(
    std::shared_ptr<GenericSubmapCollection<VoxelType1>> collision_collection,
    std::shared_ptr<GenericSubmapCollection<voxblox::IntensityVoxel>>
        data_collection,
    ProjectionIntegratorConfig integrator_config)
    : ProjectionIntegrator<ThermalProjectionIntegrator<VoxelType1>, VoxelType1,
                           voxblox::IntensityVoxel, float>(collision_collection,
                                                           data_collection, integrator_config)
// msg_transformation_(msg_transformation)
{}

template <typename VoxelType1>
ThermalProjectionIntegrator<VoxelType1>::ThermalProjectionIntegrator(
    ProjectionConfig<VoxelType1, voxblox::IntensityVoxel, float> config)
    : ProjectionIntegrator<ThermalProjectionIntegrator<VoxelType1>, VoxelType1,
                           voxblox::IntensityVoxel, float>(config) {}

template <typename VoxelType1>
void ThermalProjectionIntegrator<VoxelType1>::integrationFunction(
    voxblox::IntensityVoxel& voxel, float& data) {
  // like voxblox did
  voxel.intensity =
      (voxel.weight * voxel.intensity + data) / (voxel.weight + 1);
  voxel.weight += 1.0;

  if (voxel.weight > this->max_weight_) {
    voxel.weight = this->max_weight_;
  }
}
}  // namespace cblox
// explicit instantiations
template class cblox::ThermalProjectionIntegrator<voxblox::TsdfVoxel>;