#include "cblox_ros/generic_active_submap_visualizer.h"

#include <cblox/mesh/submap_mesher.h>

#include <voxblox/core/common.h>
#include <voxblox/core/voxel.h>

namespace cblox {

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::
    switchToSubmap(const SubmapID submap_id) {
  CHECK(geometry_submap_collection_ptr_);
  active_submap_id_ = submap_id;
  active_submap_color_idx_ = submap_id % color_cycle_length_;
  active_submap_mesh_layer_ptr_ = geometry_submap_collection_ptr_->getSubmapMeshLayer(submap_id);
  updateIntegrator();
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType,
                                   ColorVoxelType>::switchToActiveSubmap() {
  // Getting the active submap ID
  const SubmapID geometry_submap_id =
      geometry_submap_collection_ptr_->getActiveSubmapID();

  if (color_layer_) {
    color_active_submap_id_ = color_submap_collection_ptr_->getActiveSubmapID();
  }
  switchToSubmap(geometry_submap_id);
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::
    updateMeshCallback(const ros::TimerEvent&) {
  if (active_submap_mesh_integrator_ptr_ != nullptr) {
    // TODO rethink mutex

    if (typeid(geometry_submap_collection_ptr_.get()) !=
        typeid(color_submap_collection_ptr_.get())) {
      // TODO rethink mutex
      std::unique_lock<std::mutex> lock1(
          geometry_submap_collection_ptr_->collection_mutex_, std::defer_lock);
      std::unique_lock<std::mutex> lock2(
          color_submap_collection_ptr_->collection_mutex_, std::defer_lock);
      std::lock(lock1, lock2);
      publishCurrentMesh();
      lock1.unlock();
      lock2.unlock();
    } else {
      std::unique_lock<std::mutex> lock1(
          geometry_submap_collection_ptr_->collection_mutex_);
      // lock1.lock();
      publishCurrentMesh();
      lock1.unlock();
    }
  } else {
    switchToActiveSubmap();
  }
}

template <typename GeometryVoxelType, typename ColorVoxelType>
bool GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::
    republishMeshCallback(std_srvs::Empty::Request& /*request*/,
                          std_srvs::Empty::Response& /*response*/) {
  // locking like before to ensure no race conditions/segment faults are created
  if (typeid(geometry_submap_collection_ptr_.get()) !=
      typeid(color_submap_collection_ptr_.get())) {
    // TODO rethink mutex
    std::unique_lock<std::mutex> lock1(
        geometry_submap_collection_ptr_->collection_mutex_, std::defer_lock);
    std::unique_lock<std::mutex> lock2(
        color_submap_collection_ptr_->collection_mutex_, std::defer_lock);
    std::lock(lock1, lock2);
    publishCompleteMesh();
    lock1.unlock();
    lock2.unlock();
  } else {
    std::unique_lock<std::mutex> lock1(
        geometry_submap_collection_ptr_->collection_mutex_);
    // lock1.lock();
    publishCompleteMesh();
    lock1.unlock();
  }
  return true;
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType,
                                   ColorVoxelType>::publishCurrentMesh() {
  // saving and reusing meshes? TODO
  // only working if subscribers exists
  if (publisher_.getNumSubscribers() < 1) {
    return;
  }
  std::vector<SubmapID> geometry_ids =
      geometry_submap_collection_ptr_->getIDs();

  // check if current valid submap id / find it in vector
  std::vector<SubmapID>::iterator it =
      std::find(geometry_ids.begin(), geometry_ids.end(), geometry_id_);

  // reset if invalid id
  if (it == geometry_ids.end()) {
    it = geometry_ids.begin();
    geometry_id_ = *it;
  }

  // moved out of loop
  visualization_msgs::MarkerArray marker_array;
  while (it != geometry_ids.end()) {
    switchToSubmap(*it);
    updateMeshLayer();
    visualization_msgs::Marker marker;
    getDisplayMesh(&marker);
    marker.header.frame_id = "world";
    marker.id = message_id_;
    message_id_++;
    //check for size, if 0 skip
    if(marker.points.size() > 0) {
      marker_array.markers.push_back(marker);

    } else {
      ROS_INFO("Map marker %d is empty and not published!", *it);
    }
    geometry_id_ = *it;
    it++;
  }
  publisher_.publish(marker_array);
  ROS_INFO("published mesh");
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType,
                                   ColorVoxelType>::publishCompleteMesh() {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.action = 3;
  marker_array.markers.push_back(marker);
  publisher_.publish(marker_array);

  // set geometry_id_ to 0 and publish mesh
  geometry_id_ = 0;
  publishCurrentMesh();
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType,
                                   ColorVoxelType>::updateIntegrator() {
  CHECK(active_submap_mesh_layer_ptr_) << "MeshLayer not initialized.";
  // New integrator operating on the mesh.
  active_submap_mesh_integrator_ptr_.reset(
      new voxblox::MeshIntegrator<TsdfVoxel>(
          mesh_config_,
          geometry_submap_collection_ptr_->getMapPtr(active_submap_id_)
              ->getLayerPtr(),
          active_submap_mesh_layer_ptr_.get()));
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType,
                                   ColorVoxelType>::updateMeshLayer() {
  CHECK(active_submap_mesh_integrator_ptr_) << "Integrator not initialized.";

  // Updating the mesh layer
  // TODO save mesh layers so this can be easier
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  active_submap_mesh_integrator_ptr_->generateMesh(only_mesh_updated_blocks,
                                                   clear_updated_flag);
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::
    transformMeshLayerToGlobalFrame(const MeshLayer& mesh_layer_S,
                                    MeshLayer* mesh_layer_G_ptr) const {
  CHECK_NOTNULL(mesh_layer_G_ptr);
  // Transforming all triangles in the mesh and adding to the combined layer
  Transformation T_G_S;
  geometry_submap_collection_ptr_->getSubmapPose(active_submap_id_, &T_G_S);
  SubmapMesher::transformAndAddTrianglesToLayer(mesh_layer_S, T_G_S,
                                                mesh_layer_G_ptr);
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::
    colorMeshWithCurrentIndex(MeshLayer* mesh_layer_ptr) const {
  CHECK_NOTNULL(mesh_layer_ptr);
  // Coloring
  double color_map_float = static_cast<double>(active_submap_color_idx_) /
                           static_cast<double>(color_cycle_length_ - 1);
  const Color color = voxblox::rainbowColorMap(color_map_float);
  SubmapMesher::colorMeshLayer(color, mesh_layer_ptr);
}

// like previous function but with use of the color function and layer
//reverses points from mesh world into color frame to get collisions
template <typename GeometryVoxelType, typename ColorVoxelType>
bool GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::
    recolorWithColorFunction(MeshLayer* mesh_layer_ptr) const {
  CHECK_NOTNULL(mesh_layer_ptr);
  if (!color_layer_) {
    ROS_INFO("Not able to color without color layer");
    return false;
  }
  // TODO get color layers here by using new function in map, default in same
  // layer if empty
  // TODO remove color id
  // also remove active color id

  // get transform from color layer

  std::vector<typename GenericSubmap<ColorVoxelType>::Ptr> color_layers =
      color_submap_collection_ptr_->getChildMaps(active_submap_id_);

  if (color_layers.size() == 0) {
    ROS_INFO("couldn't find any child maps from the color map");
    return false;
  }

  // TODO make sure color layers Transforms are aligned (in submaps)
  if (color_layers.size() == 0) return false;
  Transformation T_w_s = (color_layers[0]->getPose());

  // Loop over Index

  voxblox::BlockIndexList index_list;
  mesh_layer_ptr->getAllAllocatedMeshes(&index_list);
  for (const voxblox::BlockIndex& block_index : index_list) {
    
    Mesh::Ptr mesh_ptr = mesh_layer_ptr->getMeshPtrByIndex(block_index);

    for (int i = 0; i < mesh_ptr->vertices.size(); i++) {
      voxblox::Point vertex = T_w_s.inverse() * mesh_ptr->vertices[i];

      // loop over all color layers here
      voxblox::Color color(0.0, 0.0, 0.0, 0.0);
      //TODO maybe merge color based on weight.
      //This would need higher logic
      for (auto color_layer : color_layers) {
        const ColorVoxelType* voxel =
            color_layer->getMap().getLayer().getVoxelPtrByCoordinates(vertex);
        voxblox::Color c = (*color_function_)(voxel);
        if (c.a > 0) {
          color = c;
        }
      }

      if(color.a > 0) {
      }
      else {
      }
      mesh_ptr->colors[i] = color;
    }
  }
  return true;
}

template <typename GeometryVoxelType, typename ColorVoxelType>
MeshLayer::Ptr GenericActiveSubmapVisualizer<
    GeometryVoxelType, ColorVoxelType>::getDisplayMeshLayer() {
  // Transforming the mesh layer into G.
  auto mesh_layer_G_ptr = std::make_shared<MeshLayer>(
      geometry_submap_collection_ptr_->block_size());
  transformMeshLayerToGlobalFrame(*active_submap_mesh_layer_ptr_,
                                  mesh_layer_G_ptr.get());

  // Coloring the mesh
  if (use_color_map_) colorMeshWithCurrentIndex(mesh_layer_G_ptr.get());

  if (use_function_) {
    if(!recolorWithColorFunction(mesh_layer_G_ptr.get())) {
      mesh_layer_G_ptr = std::make_shared<MeshLayer>(
      geometry_submap_collection_ptr_->block_size());
      return  mesh_layer_G_ptr;
      //if recoloring was unsuccessful, return empty mesh
    }
  }

  if (remove_alpha_) removeAlphaChanneled(mesh_layer_G_ptr.get());

  return mesh_layer_G_ptr;
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::
    getDisplayMesh(visualization_msgs::Marker* marker_ptr) {
  CHECK_NOTNULL(marker_ptr);
  // Getting the mesh layer
  std::shared_ptr<MeshLayer> mesh_layer_ptr = getDisplayMeshLayer();
  // Filling the marker
  const voxblox::ColorMode color_mode = voxblox::ColorMode::kLambertColor;
  // const voxblox::ColorMode color_mode = voxblox::ColorMode::kColor;
  // TODO currently only mode with transparency

  voxblox::fillMarkerWithMesh(mesh_layer_ptr, color_mode, marker_ptr);
  marker_ptr->id = active_submap_id_;
  // Setting opacity of marker
  marker_ptr->color.a = opacity_;

  for (std_msgs::ColorRGBA& color : marker_ptr->colors) {
    color.a *= opacity_;  // added * because it deletes existing transparency
  }
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::
    setColorFunction(voxblox::Color (*color_function)(const ColorVoxelType*)) {
  color_function_ = color_function;
  use_function_ = true;
  use_color_map_ = false;
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType,
                                   ColorVoxelType>::setUseColorMap() {
  use_color_map_ = true;
  use_function_ = false;
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType,
                                   ColorVoxelType>::setUseDefault() {
  use_color_map_ = false;
  use_function_ = false;
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType,
                                   ColorVoxelType>::setRemoveAlpha(bool val) {
  remove_alpha_ = val;
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::
    removeAlphaChanneled(MeshLayer* mesh_layer_ptr) const {
  CHECK_NOTNULL(mesh_layer_ptr);

  // Loop over Index
  voxblox::BlockIndexList index_list;
  mesh_layer_ptr->getAllAllocatedMeshes(&index_list);
  for (const voxblox::BlockIndex& block_index : index_list) {
    Mesh::Ptr mesh_ptr = mesh_layer_ptr->getMeshPtrByIndex(block_index);

    int i = 0;
    bool has_normals = mesh_ptr->hasNormals();
    bool has_indices = mesh_ptr->hasTriangles();
    // go in steps of 3

    while (i < mesh_ptr->vertices.size()) {
      if (mesh_ptr->colors[i].a == 0 || mesh_ptr->colors[i + 1].a == 0 ||
          mesh_ptr->colors[i + 2].a == 0) {
        mesh_ptr->colors.erase(mesh_ptr->colors.begin() + i,
                               mesh_ptr->colors.begin() + i + 3);
        mesh_ptr->vertices.erase(mesh_ptr->vertices.begin() + i,
                                 mesh_ptr->vertices.begin() + i + 3);
        if (has_normals) {
          mesh_ptr->normals.erase(mesh_ptr->normals.begin() + i,
                                  mesh_ptr->normals.begin() + i + 3);
        }
        if (has_indices) {
          mesh_ptr->indices.erase(mesh_ptr->indices.begin() + i,
                                  mesh_ptr->indices.begin() + i + 3);
        }
        // fix index list? What even does it?
      } else
        i += 3;
    }
  }
}

}  // namespace cblox

#include <cblox/core/voxel.h>
// instantiation
template class cblox::GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                                    voxblox::TsdfVoxel>;
template class cblox::GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                                    voxblox::RGBVoxel>;
template class cblox::GenericActiveSubmapVisualizer<voxblox::TsdfVoxel,
                                                    voxblox::IntensityVoxel>;