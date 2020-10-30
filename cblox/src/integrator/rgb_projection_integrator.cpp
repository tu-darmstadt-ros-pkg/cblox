#include "cblox/integrator/rgb_projection_integrator.h"

namespace cblox{

template<typename VoxelType1>
RGBProjectionIntegrator<VoxelType1>::
RGBProjectionIntegrator(std::shared_ptr<GenericSubmapCollection<VoxelType1>> collision_collection,
                     Layer<voxblox::RGBVoxel>* integration_layer)
    : ProjectionIntegrator<RGBProjectionIntegrator<VoxelType1>, VoxelType1, voxblox::RGBVoxel, Color>(collision_collection, integration_layer)
      //msg_transformation_(msg_transformation) 
      {}

template<typename VoxelType1>
RGBProjectionIntegrator<VoxelType1>::
RGBProjectionIntegrator(ProjectionConfig<VoxelType1, voxblox::RGBVoxel, Color> config, Layer<voxblox::RGBVoxel>* l)
    : ProjectionIntegrator<RGBProjectionIntegrator<VoxelType1>, VoxelType1, voxblox::RGBVoxel, Color>(config, l) 
    {}

template<typename VoxelType1>
void RGBProjectionIntegrator<VoxelType1>::
integrationFunction(voxblox::RGBVoxel& voxel, Color& data) {
    //blend
    voxel.color = Color::blendTwoColors(voxel.color, voxel.weight,
                                        data, 1.0);
    voxel.weight += 1.0;
}
}
//explicit instantiations
template class cblox::RGBProjectionIntegrator<voxblox::TsdfVoxel>;