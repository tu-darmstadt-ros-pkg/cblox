#ifndef CBLOX_RGB_PROJECTION_INTEGRATOR_H_
#define CBLOX_RGB_PROJECTION_INTEGRATOR_H_

#include "cblox/integrator/projection_integrator.h"
#include <voxblox/core/color.h>
#include "cblox/core/voxel.h"

namespace cblox {



    template <typename VoxelType1>
    class RGBProjectionIntegrator : public ProjectionIntegrator<RGBProjectionIntegrator<VoxelType1>, VoxelType1, voxblox::RGBVoxel, Color>
    {
        public:
            RGBProjectionIntegrator(std::shared_ptr<GenericSubmapCollection<VoxelType1>> collision_collection,
                                    std::shared_ptr<GenericSubmapCollection<voxblox::RGBVoxel>> data_collection);

            RGBProjectionIntegrator(ProjectionConfig<VoxelType1, voxblox::RGBVoxel, Color> c);

            void integrationFunction(voxblox::RGBVoxel& voxel, Color& data);
    };

}
#endif  //CBLOX_RGB_PROJECTION_INTEGRATOR_H_