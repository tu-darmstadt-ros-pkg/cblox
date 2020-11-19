#ifndef CBLOX_THERMAL_PROJECTION_INTEGRATOR_H_
#define CBLOX_THERMAL_PROJECTION_INTEGRATOR_H_

#include "cblox/integrator/projection_integrator.h"
#include <voxblox/core/color.h>


namespace cblox {



    template <typename VoxelType1>
    class ThermalProjectionIntegrator : public ProjectionIntegrator<ThermalProjectionIntegrator<VoxelType1>, VoxelType1, voxblox::IntensityVoxel, float>
    {
        public:
            ThermalProjectionIntegrator(std::shared_ptr<GenericSubmapCollection<VoxelType1>> collision_collection,
                                        std::shared_ptr<GenericSubmapCollection<voxblox::IntensityVoxel>> data_collection);

            ThermalProjectionIntegrator(ProjectionConfig<VoxelType1, voxblox::IntensityVoxel, float> c);

            void integrationFunction(voxblox::IntensityVoxel& voxel, float& data);
    };

}
#endif  //CBLOX_RGB_PROJECTION_INTEGRATOR_H_