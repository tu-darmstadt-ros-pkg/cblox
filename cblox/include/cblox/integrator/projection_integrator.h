#ifndef CBLOX_INTEGRATOR_PROJECTION_INTEGRATOR_H_
#define CBLOX_INTEGRATOR_PROJECTION_INTEGRATOR_H_

#include <queue>
#include <vector>

#include <Eigen/Core>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/timing.h"
#include "voxblox/utils/distance_utils.h"

namespace voxblox {

template <typename T>
struct ProjectionData {
    Point origin;
    Pointcloud bearing_vectors;
    std::vector<T> data;
};

//TODO maybe derive from interpolator???

template <typename VoxelType1, typename VoxelType2, typename IntegrationData>
class ProjectionIntegrator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //TODO add integration function
    ProjectionIntegrator(Layer<VoxelType1>* collision_layer,
                         Layer<VoxelType2>* integration_layer,
                         void (*integration_function)(VoxelType2&, IntegrationData&));

    void setMaxDistance(const FloatingPoint max_distance) {
        max_distance_ = max_distance;
    }
    FloatingPoint getMaxDistance() const { return max_distance_; }

    //TODO like intensity server
    void integrate(const Transformation& T_G_C, const IntegrationData& data);

    void addBearingVectors(const Point& origin,
                           const Pointcloud& bearing_vectors,
                           const std::vector<IntegrationData>& data);

    void (*integration_function_)(VoxelType2&, IntegrationData&);

    //TODO think about changing collision layer
    void setCollisionLayer(Layer<VoxelType1>* layer);

    void setIntegrationLayer(Layer<VoxelType2>* layer);

  private:
    FloatingPoint max_distance_;
    FloatingPoint max_weight_;

    int prop_voxel_radius_;

    Layer<VoxelType1>* collision_layer_;
    Layer<VoxelType2>* integration_layer_;

};

}


#endif  //CBLOX_INTEGRATOR_PROJECTION_INTEGRATOR_H_