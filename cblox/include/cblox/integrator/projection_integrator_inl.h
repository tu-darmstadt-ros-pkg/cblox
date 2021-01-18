#ifndef CBLOX_INTEGRATOR_PROJECTION_INTEGRATOR_INL_H_
#define CBLOX_INTEGRATOR_PROJECTION_INTEGRATOR_INL_H_

#include "voxblox/utils/distance_utils.h"

// adapted from intensity integrator

namespace cblox {

template <typename T, typename VoxelType1, typename VoxelType2, typename Data>
ProjectionIntegrator<T, VoxelType1, VoxelType2, Data>::ProjectionIntegrator(
    std::shared_ptr<GenericSubmapCollection<VoxelType1>> collision_collection,
    std::shared_ptr<GenericSubmapCollection<VoxelType2>> data_collection,
    ProjectionIntegratorConfig integrator_config)
    : max_distance_(integrator_config.max_distance),
      max_weight_(integrator_config.max_weight),
      prop_voxel_radius_(integrator_config.prop_voxel_radius),
      collision_collection_(collision_collection),
      data_collection_(data_collection),
      integration_layer_(data_collection->getActiveMapPtr()->getLayerPtr())
// msg_transformation_(msg_transformation)
{}

template <typename T, typename VoxelType1, typename VoxelType2, typename Data>
ProjectionIntegrator<T, VoxelType1, VoxelType2, Data>::ProjectionIntegrator(
    ProjectionConfig<VoxelType1, VoxelType2, Data> config)
    : max_distance_(config.integrator_config.max_distance),
      max_weight_(config.integrator_config.max_weight),
      prop_voxel_radius_(config.integrator_config.prop_voxel_radius),
      collision_collection_(config.collision_collection),
      data_collection_(config.data_collection),
      integration_layer_(
          config.data_collection->getActiveMapPtr()->getLayerPtr()) {}

// TODO set transformations for layers

template <typename T, typename VoxelType1, typename VoxelType2, typename Data>
void ProjectionIntegrator<T, VoxelType1, VoxelType2, Data>::integrate(
    const Transformation& T_S_C, const ProjectionData<Data>& data) {
  // get active collision layer

  // TODO rethink updating of layers
  setActiveLayers();
  // collision_layer_ = collision_collection_->getActiveMapPtr()->getLayerPtr();
  Transformation T_S2_C =
      collision_collection_->getActiveSubmapPose().inverse();

  // lock TODO rethink
  //maybe lock before getting layers?
  std::unique_lock<std::mutex> lock1(collision_collection_->collection_mutex_,
                                     std::defer_lock);
  std::unique_lock<std::mutex> lock2(data_collection_->collection_mutex_,
                                     std::defer_lock);
  std::lock(lock1, lock2);
  addBearingVectors(data.origin, data.bearing_vectors, data.data, T_S_C,
                    T_S2_C);
  lock1.unlock();
  lock2.unlock();
  // using transform to project back from world frame after collision
}

// TODO rename transformations

//vectors in world frame are transformed into mesh frame (also origin is in mesh frame)
//after that they are transformed back into the projected layer frame
template <typename T, typename VoxelType1, typename VoxelType2, typename Data>
void ProjectionIntegrator<T, VoxelType1, VoxelType2, Data>::addBearingVectors(
    const Point& origin, const Pointcloud& bearing_vectors,
    const std::vector<Data>& data, const Transformation& T_S_C,
    const Transformation& T_S2_C) {
  // timing::Timer intensity_timer("intensity/integrate");
  int hit = 0.0;
  int loss = 0.0;
  CHECK_EQ(bearing_vectors.size(), data.size())
      << "Intensity and bearing vector size does not match!";
  const FloatingPoint voxel_size = collision_layer_->voxel_size();

  Transformation T_w_d = collision_collection_->getActiveSubmapPose();
  //std::cout << "proj m: " << T_S_C.getRotationMatrix() << std::endl;

  for (size_t i = 0; i < bearing_vectors.size(); ++i) {
    Point surface_intersection = Point::Zero();
    // Cast ray from the origin in the direction of the bearing vector until
    // finding an intersection with a surface.
    // like intensity integrator
    bool success = voxblox::getSurfaceDistanceAlongRay<VoxelType1>(
        *collision_layer_, T_S2_C * origin, T_S2_C * bearing_vectors[i],
        max_distance_, &surface_intersection);

    if (!success) {
      loss+=1.0;
      continue;
    }
    hit+=1.0;


    //TODO this transforms back into color layer coordinates, but we want to keep
    Point real_surface_intersection = T_w_d.inverse() * T_S2_C.inverse() * surface_intersection;
    //std::cout << "projecting: " << surface_intersection << std::endl;
    
    // Now look up the matching voxels in the intensity layer and mark them.
    // Let's just start with 1.
    typename Block<VoxelType2>::Ptr block_ptr =
        integration_layer_->allocateBlockPtrByCoordinates(real_surface_intersection);
    VoxelType2& voxel = block_ptr->getVoxelByCoordinates(real_surface_intersection);

    //(*integration_function_)(voxel, data[i]);
    integrationFunction(voxel, data[i]);

    // Now check the surrounding voxels along the bearing vector. If they have
    // never been observed, then fill in their value. Otherwise don't.
    Point close_voxel = surface_intersection;
    for (int voxel_offset = -prop_voxel_radius_;
         voxel_offset <= prop_voxel_radius_; voxel_offset++) {
      close_voxel =
          surface_intersection + T_S2_C * bearing_vectors[i] * voxel_offset * voxel_size;
      Point real_close_voxel = T_w_d.inverse() * T_S2_C.inverse() * close_voxel;
      typename Block<VoxelType2>::Ptr new_block_ptr =
          integration_layer_->allocateBlockPtrByCoordinates(real_close_voxel);
      VoxelType2& new_voxel = block_ptr->getVoxelByCoordinates(real_close_voxel);
      if (new_voxel.weight < 1e-6) {
        //(*integration_function_)(voxel, data[i]);
        integrationFunction(voxel, data[i]);
      }
    }
  }
  /*std::cout << "hit/loss projecting" << std::endl;
  std::cout << hit << " " << loss << std::endl;*/
}

template <typename T, typename VoxelType1, typename VoxelType2, typename Data>
void ProjectionIntegrator<T, VoxelType1, VoxelType2, Data>::setActiveLayers() {
  integration_layer_ = data_collection_->getActiveMapPtr()->getLayerPtr();
  collision_layer_ = collision_collection_->getActiveMapPtr()->getLayerPtr();
}

}  // namespace cblox

#endif  // CBLOX_INTEGRATOR_PROJECTION_INTEGRATOR_INL_H_

// explicit instantiations
// template ProjectionIntegrator
// template class
// cblox::ProjectionIntegrator<cblox::RGBProjectionIntegrator<voxblox::TsdfVoxel>,
// voxblox::TsdfVoxel, voxblox::RGBVoxel, voxblox::Color>;