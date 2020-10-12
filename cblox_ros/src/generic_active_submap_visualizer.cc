#include "cblox_ros/generic_active_submap_visualizer.h"

#include <cblox/mesh/submap_mesher.h>

#include <voxblox/core/voxel.h>
#include <voxblox/core/common.h>

namespace cblox {

template <typename GeometryVoxelType, typename ColorVoxelType> 
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::switchToSubmap(const SubmapID submap_id) {
  CHECK(geometry_submap_collection_ptr_);
  active_submap_id_ = submap_id;
  if (mesh_layers_.find(submap_id) == mesh_layers_.end()) {
    if (verbose_) {
      ROS_INFO_STREAM("Creating mesh layer for submap id: " << submap_id);
    }
    createMeshLayer();
    updateIntegrator();
  } else {
    if (verbose_) {
      ROS_INFO_STREAM("Recovering mesh layer for submap id: " << submap_id);
    }
    recoverMeshLayer();
    updateIntegrator();
  }
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::switchToActiveSubmap() {
  
  if(active_submap_mesh_integrator_ptr_ != nullptr){
      //Temporary TODO, not thread safe
      updateMeshLayer();
      visualization_msgs::Marker marker;
      getDisplayMesh(&marker);
      marker.header.frame_id = "world";
      visualization_msgs::MarkerArray marker_array;
      marker_array.markers.push_back(marker);
      publisher_.publish(marker_array);
      std::cout << "published mesh" << std::endl;

  }

    // Getting the active submap ID
  const SubmapID geometry_submap_id = geometry_submap_collection_ptr_->getActiveSubmapID();
  
  if(color_layer_) {
    color_active_submap_id_ = color_submap_collection_ptr_->getActiveSubmapID();
  }
  switchToSubmap(geometry_submap_id);
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::createMeshLayer() {
  // Active layer stuff
  CHECK(geometry_submap_collection_ptr_);
  active_submap_mesh_layer_ptr_.reset(
      new voxblox::MeshLayer(geometry_submap_collection_ptr_->block_size()));
  active_submap_color_idx_ = current_color_idx_;
  // Saving mesh layer and color for later recovery
  mesh_layers_[active_submap_id_] = active_submap_mesh_layer_ptr_;
  mesh_color_indices_[active_submap_id_] = active_submap_color_idx_;
  // Updating the color index for the next map.
  current_color_idx_ = (current_color_idx_ + 1) % color_cycle_length_;
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::recoverMeshLayer() {
  auto mesh_it = mesh_layers_.find(active_submap_id_);
  auto color_it = mesh_color_indices_.find(active_submap_id_);
  CHECK(mesh_it != mesh_layers_.end())
      << "Tried to recover layer not already created";
  CHECK(color_it != mesh_color_indices_.end())
      << "Tried to recover layer not already created";
  active_submap_mesh_layer_ptr_ = mesh_it->second;
  active_submap_color_idx_ = color_it->second;
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::updateIntegrator() {
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
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::updateMeshLayer() {
  CHECK(active_submap_mesh_integrator_ptr_) << "Integrator not initialized.";
  // Updating the mesh layer
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  active_submap_mesh_integrator_ptr_->generateMesh(only_mesh_updated_blocks,
                                                   clear_updated_flag);
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::transformMeshLayerToGlobalFrame(
    const MeshLayer& mesh_layer_S, MeshLayer* mesh_layer_G_ptr) const {
  CHECK_NOTNULL(mesh_layer_G_ptr);
  // Transforming all triangles in the mesh and adding to the combined layer
  Transformation T_G_S;
  geometry_submap_collection_ptr_->getSubmapPose(active_submap_id_, &T_G_S);
  SubmapMesher::transformAndAddTrianglesToLayer(mesh_layer_S, T_G_S,
                                                mesh_layer_G_ptr);
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::colorMeshWithCurrentIndex(
    MeshLayer* mesh_layer_ptr) const {
  CHECK_NOTNULL(mesh_layer_ptr);
  // Coloring
  double color_map_float = static_cast<double>(active_submap_color_idx_) /
                           static_cast<double>(color_cycle_length_ - 1);
  const Color color = voxblox::rainbowColorMap(color_map_float);
  SubmapMesher::colorMeshLayer(color, mesh_layer_ptr);
}

//like previous function but with use of the color function and layer
template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::recolorWithColorFunction(
    MeshLayer* mesh_layer_ptr) const {
  CHECK_NOTNULL(mesh_layer_ptr);

  if(!color_layer_) {
    ROS_INFO("Not able to color without color layer");
    return;
  }

  //get transform from color layer
  
  Transformation T_w_s;
  color_submap_collection_ptr_->getSubmapPose(color_active_submap_id_, &T_w_s);;
  const voxblox::Layer<ColorVoxelType>* color_layer = color_submap_collection_ptr_->getMapPtr(color_active_submap_id_)->getLayerPtr();

  //Loop over Index
  voxblox::BlockIndexList index_list;
  mesh_layer_ptr->getAllAllocatedMeshes(&index_list);
  for (const voxblox::BlockIndex& block_index : index_list) {
    Mesh::Ptr mesh_ptr = mesh_layer_ptr->getMeshPtrByIndex(block_index);
      voxblox::Point origin = mesh_ptr->origin;

    for(int i = 0; i < mesh_ptr->vertices.size(); i++) {
      //std::cout << mesh_ptr->vertices[i] << std::endl;
      voxblox::Point vertex = T_w_s.inverse() * mesh_ptr->vertices[i];
      const ColorVoxelType* voxel = color_layer->getVoxelPtrByCoordinates(vertex);
      voxblox::Color color = (*color_function_)(voxel);
      mesh_ptr->colors[i] = color;
    }
  }
}

template <typename GeometryVoxelType, typename ColorVoxelType>
MeshLayer::Ptr GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::getDisplayMeshLayer() {
  // Transforming the mesh layer into G.
  auto mesh_layer_G_ptr =
      std::make_shared<MeshLayer>(geometry_submap_collection_ptr_->block_size());
  transformMeshLayerToGlobalFrame(*active_submap_mesh_layer_ptr_,
                                  mesh_layer_G_ptr.get());
  // Coloring the mesh
  if(use_color_map_)
    colorMeshWithCurrentIndex(mesh_layer_G_ptr.get());

  if(use_function_)
    recolorWithColorFunction(mesh_layer_G_ptr.get());
  //TODO add coloring function with function pointer here, transformation to 2nd layer needed, if used
  //test transform by recoloring tsdf with its own colors, test all combinations.
  //TODO own use of 2nd layer set not found voxels to black/transparent
  //(missing voxel behavior) 
  //also add option to not recolor
  return mesh_layer_G_ptr;
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::getDisplayMesh(
    visualization_msgs::Marker* marker_ptr) {
  CHECK_NOTNULL(marker_ptr);
  // Getting the mesh layer
  std::shared_ptr<MeshLayer> mesh_layer_ptr = getDisplayMeshLayer();
  // Filling the marker
  const voxblox::ColorMode color_mode = voxblox::ColorMode::kLambertColor;
  voxblox::fillMarkerWithMesh(mesh_layer_ptr, color_mode, marker_ptr);
  marker_ptr->id = active_submap_id_;
  // Setting opacity of marker
  marker_ptr->color.a = opacity_;
  for (std_msgs::ColorRGBA& color : marker_ptr->colors) {
    color.a = opacity_;
  }
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::setColorFunction(voxblox::Color (*color_function)(const ColorVoxelType*)) {
  color_function_ = color_function;
  use_function_ = true;
  //use_color_map_ = false;
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::setUseColorMap() {
  use_color_map_ = true;
  //use_function_ = false;
}

template <typename GeometryVoxelType, typename ColorVoxelType>
void GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>::setUseDefault() {
  use_color_map_ = false;
  use_function_ = false;
}

}  // namespace cblox

//instantiation
template class cblox::GenericActiveSubmapVisualizer<voxblox::TsdfVoxel, voxblox::TsdfVoxel>;