#include "cblox/integrator/thermal_projection_integrator.h"

namespace cblox{

template<typename VoxelType1>
ThermalProjectionIntegrator<VoxelType1>::
ThermalProjectionIntegrator(std::shared_ptr<GenericSubmapCollection<VoxelType1>> collision_collection,
                     Layer<voxblox::IntensityVoxel>* integration_layer)
    : ProjectionIntegrator<ThermalProjectionIntegrator<VoxelType1>, VoxelType1, voxblox::IntensityVoxel, float>(collision_collection, integration_layer)
      //msg_transformation_(msg_transformation) 
      {}

template<typename VoxelType1>
ThermalProjectionIntegrator<VoxelType1>::
ThermalProjectionIntegrator(ProjectionConfig<VoxelType1, voxblox::IntensityVoxel, float> config, Layer<voxblox::IntensityVoxel>* l)
    : ProjectionIntegrator<ThermalProjectionIntegrator<VoxelType1>, VoxelType1, voxblox::IntensityVoxel, float>(config, l) 
    {}

template<typename VoxelType1>
void ThermalProjectionIntegrator<VoxelType1>::
integrationFunction(voxblox::IntensityVoxel& voxel, float& data) {
    //like voxblox did
    voxel.intensity = (voxel.weight * voxel.intensity + data) / (voxel.weight + 1);
    voxel.weight += 1.0;

    if (voxel.weight > this->max_weight_) {
        voxel.weight = this->max_weight_;
    }
}
}
//explicit instantiations
template class cblox::ThermalProjectionIntegrator<voxblox::TsdfVoxel>;