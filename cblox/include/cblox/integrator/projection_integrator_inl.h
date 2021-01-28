#ifndef CBLOX_INTEGRATOR_PROJECTION_INTEGRATOR_INL_H_
#define CBLOX_INTEGRATOR_PROJECTION_INTEGRATOR_INL_H_

#include "voxblox/utils/distance_utils.h"

// adapted from intensity integrator

namespace cblox {

template <typename T, typename VoxelType1, typename VoxelType2, typename Data>
ProjectionIntegrator<T, VoxelType1, VoxelType2, Data>::ProjectionIntegrator(
    std::shared_ptr<GenericSubmapCollection<VoxelType1>> geometry_collection,
    std::shared_ptr<GenericSubmapCollection<VoxelType2>> data_collection,
    ProjectionIntegratorConfig integrator_config)
    : max_distance_(integrator_config.max_distance),
      max_weight_(integrator_config.max_weight),
      prop_voxel_radius_(integrator_config.prop_voxel_radius),
      geometry_collection_(geometry_collection),
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
      geometry_collection_(config.geometry_collection),
      data_collection_(config.data_collection),
      integration_layer_(
          config.data_collection->getActiveMapPtr()->getLayerPtr()) {}

// TODO set transformations for layers

template <typename T, typename VoxelType1, typename VoxelType2, typename Data>
void ProjectionIntegrator<T, VoxelType1, VoxelType2, Data>::integrate(
    const Transformation& T_G_C, const ProjectionData<Data>& data) {
  // get active collision layer
  // ignoring transform, as data was already projected to world

  // TODO rethink updating of layers
  setLayers(data.geometry_id, data.id);
  setTransformations(data.geometry_id, data.id);

  addBearingVectors(data.origin, data.bearing_vectors, data.data);

  // using transform to project back from world frame after collision
}

// TODO rename transformations

//vectors in world frame are transformed into mesh frame (also origin is in mesh frame)
//after that they are transformed back into the projected layer frame
template <typename T, typename VoxelType1, typename VoxelType2, typename Data>
void ProjectionIntegrator<T, VoxelType1, VoxelType2, Data>::addBearingVectors(
    const Point& origin, const Pointcloud& bearing_vectors,
    const std::vector<Data>& data) {
  // creating mutex
  std::unique_lock<std::mutex> lock1(geometry_collection_->collection_mutex_,
                                     std::defer_lock);
  std::unique_lock<std::mutex> lock2(data_collection_->collection_mutex_,
                                     std::defer_lock);
  std::lock(lock1, lock2);
  // TODO check if locks should be set directly

  // TODO change mutex to only lock on access of maps, directly pass ids of
  // map/submap pair, dont use get active/set active

  // timing::Timer intensity_timer("intensity/integrate");
  CHECK_EQ(bearing_vectors.size(), data.size())
      << "Intensity and bearing vector size does not match!";
  const FloatingPoint voxel_size = collision_layer_->voxel_size();

  for (size_t i = 0; i < bearing_vectors.size(); ++i) {
    Point surface_intersection = Point::Zero();
    // Cast ray from the origin in the direction of the bearing vector until
    // finding an intersection with a surface.
    // like intensity integrator
    bool success = voxblox::getSurfaceDistanceAlongRay<VoxelType1>(
        *collision_layer_, T_Geometry_W_ * origin,
        T_Geometry_W_.getRotationMatrix() * bearing_vectors[i], max_distance_,
        &surface_intersection);

    if (!success) {
      continue;
    }

    //TODO this transforms back into color layer coordinates, but we want to keep
    Point real_surface_intersection =
        T_W_Data_.inverse() * T_Geometry_W_.inverse() * surface_intersection;

    typename Block<VoxelType2>::Ptr block_ptr =
        integration_layer_->allocateBlockPtrByCoordinates(real_surface_intersection);
    VoxelType2& voxel = block_ptr->getVoxelByCoordinates(real_surface_intersection);

    integrationFunction(voxel, data[i]);

    // Now check the surrounding voxels along the bearing vector. If they have
    // never been observed, then fill in their value. Otherwise don't.
    Point close_voxel = surface_intersection;
    for (int voxel_offset = -prop_voxel_radius_;
         voxel_offset <= prop_voxel_radius_; voxel_offset++) {
      close_voxel = surface_intersection + T_Geometry_W_ * bearing_vectors[i] *
                                               voxel_offset * voxel_size;
      Point real_close_voxel =
          T_W_Data_.inverse() * T_Geometry_W_.inverse() * close_voxel;
      typename Block<VoxelType2>::Ptr new_block_ptr =
          integration_layer_->allocateBlockPtrByCoordinates(real_close_voxel);
      VoxelType2& new_voxel = block_ptr->getVoxelByCoordinates(real_close_voxel);
      if (new_voxel.weight < 1e-6) {
        integrationFunction(voxel, data[i]);
      }
    }
  }

  // TODO maybe improve by locking and relocking only on data access?
  lock1.unlock();
  lock2.unlock();
}

// sets the layers of geometry and data to the newest. This is not
// multithreading safe though
template <typename T, typename VoxelType1, typename VoxelType2, typename Data>
void ProjectionIntegrator<T, VoxelType1, VoxelType2, Data>::setActiveLayers() {
  integration_layer_ = data_collection_->getActiveMapPtr()->getLayerPtr();
  collision_layer_ = geometry_collection_->getActiveMapPtr()->getLayerPtr();
}

// sets the layers of geometry and data to the desired ones
template <typename T, typename VoxelType1, typename VoxelType2, typename Data>
void ProjectionIntegrator<T, VoxelType1, VoxelType2, Data>::setLayers(
    SubmapID geometry_id, SubmapID data_id) {
  integration_layer_ = data_collection_->getMapPtr(data_id)->getLayerPtr();
  collision_layer_ =
      geometry_collection_->getMapPtr(geometry_id)->getLayerPtr();
}

// set transformations
template <typename T, typename VoxelType1, typename VoxelType2, typename Data>
void ProjectionIntegrator<T, VoxelType1, VoxelType2, Data>::setTransformations(
    SubmapID geometry_id, SubmapID data_id) {
  T_Geometry_W_ =
      geometry_collection_->getSubmap(geometry_id).getPose().inverse();
  T_W_Data_ = data_collection_->getSubmap(data_id).getPose();
}

}  // namespace cblox

#endif  // CBLOX_INTEGRATOR_PROJECTION_INTEGRATOR_INL_H_

// explicit instantiations
// template ProjectionIntegrator
// template class
// cblox::ProjectionIntegrator<cblox::RGBProjectionIntegrator<voxblox::TsdfVoxel>,
// voxblox::TsdfVoxel, voxblox::RGBVoxel, voxblox::Color>;