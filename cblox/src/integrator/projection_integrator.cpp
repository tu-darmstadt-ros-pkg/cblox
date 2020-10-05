#include "cblox/integrator/projection_integrator.h"

#include "voxblox/utils/distance_utils.h"

//adapted from intensity integrator

namespace voxblox {

template <typename VoxelType1, typename VoxelType2, typename IntegrationData>
ProjectionIntegrator<VoxelType1, VoxelType2, IntegrationData>::
ProjectionIntegrator(Layer<VoxelType1>* collision_layer,
                     Layer<VoxelType2>* integration_layer,
                     void (*integration_function)(VoxelType2&, IntegrationData&))
    : max_distance_(5.0),
      max_weight_(100.0),
      prop_voxel_radius_(2),
      collision_layer_(collision_layer),
      integration_layer_(integration_layer),
      integration_function_(integration_function)
      //msg_transformation_(msg_transformation) 
      {}

template <typename VoxelType1, typename VoxelType2, typename IntegrationData>
void ProjectionIntegrator<VoxelType1, VoxelType2, IntegrationData>::
integrate(const Transformation& T_G_C, const IntegrationData& data) {
    //TODO
}

template <typename VoxelType1, typename VoxelType2, typename IntegrationData>
void ProjectionIntegrator<VoxelType1, VoxelType2, IntegrationData>::
addBearingVectors(const Point& origin, const Pointcloud& bearing_vectors,
                  const std::vector<IntegrationData>& data) {
    timing::Timer intensity_timer("intensity/integrate");

    CHECK_EQ(bearing_vectors.size(), data.size())
        << "Intensity and bearing vector size does not match!";
    const FloatingPoint voxel_size = collision_layer_.voxel_size();

    for (size_t i = 0; i < bearing_vectors.size(); ++i) {
        Point surface_intersection = Point::Zero();
        // Cast ray from the origin in the direction of the bearing vector until
        // finding an intersection with a surface.
        bool success = getSurfaceDistanceAlongRay<VoxelType1>(
            collision_layer_, origin, bearing_vectors[i], max_distance_,
            &surface_intersection);

        if (!success) {
            continue;
        }

        // Now look up the matching voxels in the intensity layer and mark them.
        // Let's just start with 1.
        typename Block<VoxelType2>::Ptr block_ptr =
            integration_layer_->allocateBlockPtrByCoordinates(surface_intersection);
        VoxelType2& voxel =
            block_ptr->getVoxelByCoordinates(surface_intersection);
        
        (*integration_function_)(voxel, data[i]);

        // Now check the surrounding voxels along the bearing vector. If they have
        // never been observed, then fill in their value. Otherwise don't.
        Point close_voxel = surface_intersection;
        for (int voxel_offset = -prop_voxel_radius_;
            voxel_offset <= prop_voxel_radius_; voxel_offset++) {
            close_voxel =
            surface_intersection + bearing_vectors[i] * voxel_offset * voxel_size;
            typename Block<VoxelType2>::Ptr new_block_ptr =
                integration_layer_->allocateBlockPtrByCoordinates(close_voxel);
            VoxelType2& new_voxel = block_ptr->getVoxelByCoordinates(close_voxel);
            if (new_voxel.weight < 1e-6) {
                (*integration_function_)(voxel, data[i]);
            }
        }
    }
}

}

//explicit instantiations
//template ProjectionIntegrator