#ifndef CBLOX_INTEGRATOR_PROJECTION_INTEGRATOR_H_
#define CBLOX_INTEGRATOR_PROJECTION_INTEGRATOR_H_

#include <queue>
#include <vector>

#include <Eigen/Core>

#include <cblox/core/generic_submap_collection.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/utils/distance_utils.h>
#include <voxblox/utils/timing.h>

namespace cblox {

template <typename Data>
struct ProjectionData {
  Point origin;
  Pointcloud bearing_vectors;
  std::vector<Data> data;
  SubmapID id;
  SubmapID geometry_id;
  std::shared_ptr<std::vector<int>> integrated;
};

struct ProjectionIntegratorConfig {
    float max_distance = 40.0;
    float max_weight = 100.0;
    int prop_voxel_radius = 2;
    int use_previous_maps = 0;
};

template <typename VoxelType1, typename VoxelType2, typename Data>
struct ProjectionConfig {
  std::shared_ptr<GenericSubmapCollection<VoxelType1>> geometry_collection;
  std::shared_ptr<GenericSubmapCollection<VoxelType2>> data_collection;
  ProjectionIntegratorConfig integrator_config;
};

// TODO maybe derive from interpolator???

template <typename T, typename VoxelType1, typename VoxelType2, typename Data>
class ProjectionIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // TODO add integration function
  ProjectionIntegrator(
      std::shared_ptr<GenericSubmapCollection<VoxelType1>> geometry_collection,
      std::shared_ptr<GenericSubmapCollection<VoxelType2>> data_collection,
      ProjectionIntegratorConfig integrator_config
      // Layer<VoxelType2>* integration_layer
      //(*integration_function)(VoxelType2&, IntegrationData&)
  );

  ProjectionIntegrator(ProjectionConfig<VoxelType1, VoxelType2, Data> c);

  void setMaxDistance(const FloatingPoint max_distance) {
    max_distance_ = max_distance;
  }
  FloatingPoint getMaxDistance() const { return max_distance_; }

  // TODO like intensity server
  void integrate(const Transformation& T_G_C, const ProjectionData<Data>& data);

  void addBearingVectors(const Point& origin, const Pointcloud& bearing_vectors,
                         const std::vector<Data>& data, 
  std::shared_ptr<std::vector<int>> integrated);

  // void (*integration_function_)(VoxelType2&, IntegrationData&);

  void integrationFunction(VoxelType2& voxel, Data data) {
    static_cast<T*>(this)->integrationFunction(voxel, data);
  }

  // TODO think about changing collision layer
  void setCollisionLayer(Layer<VoxelType1>* layer);

  void setIntegrationLayer(Layer<VoxelType2>* layer);

  void setActiveLayers();

  void setLayers(SubmapID geometry_id, SubmapID data_id);

  void setTransformations(SubmapID geometry_id, SubmapID data_id);

 protected:
  FloatingPoint max_distance_;
  FloatingPoint max_weight_;

  int prop_voxel_radius_;
  std::shared_ptr<GenericSubmapCollection<VoxelType1>> geometry_collection_;
  std::shared_ptr<GenericSubmapCollection<VoxelType2>> data_collection_;
  Layer<VoxelType1>* collision_layer_;
  Layer<VoxelType2>* integration_layer_;
  Transformation T_Geometry_W_;
  Transformation T_W_Data_;

  bool debug_;
  int submap_nr_;
  int use_previous_maps_;

};

}  // namespace cblox

#endif  // CBLOX_INTEGRATOR_PROJECTION_INTEGRATOR_H_

#include "cblox/integrator/projection_integrator_inl.h"