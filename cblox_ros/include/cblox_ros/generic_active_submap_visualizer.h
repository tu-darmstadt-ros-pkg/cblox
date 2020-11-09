#ifndef CBLOX_ROS_GENERIC_ACTIVE_SUBMAP_VISUALIZER_H_
#define CBLOX_ROS_GENERIC_ACTIVE_SUBMAP_VISUALIZER_H_

#include <memory>

#include <visualization_msgs/Marker.h>

#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/mesh/mesh_layer.h>
#include <voxblox_ros/mesh_vis.h>

#include <cblox/core/generic_submap_collection.h>
#include <cblox/mesh/submap_mesher.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

#include <voxblox/core/color.h>

namespace cblox {

using voxblox::Color;
using voxblox::MeshIntegrator;
using voxblox::MeshIntegratorConfig;
using voxblox::MeshLayer;

template <typename GeometryVoxelType, typename ColorVoxelType>
class GenericActiveSubmapVisualizer {
 public:
  typedef std::shared_ptr<GenericActiveSubmapVisualizer> Ptr;
  typedef std::shared_ptr<const GenericActiveSubmapVisualizer> ConstPtr;

  // Constructor
  GenericActiveSubmapVisualizer(const MeshIntegratorConfig& mesh_config,
                         const std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>>&
                             geometry_submap_collection_ptr,
                         std::string topic,
                         ros::NodeHandle nh,
                         ros::NodeHandle nh_private)
      : mesh_config_(mesh_config),
        geometry_submap_collection_ptr_(geometry_submap_collection_ptr),
        color_layer_(false),
        color_cycle_length_(kDefaultColorCycleLength),
        current_color_idx_(0),
        verbose_(false),
        opacity_(1.0),
        nh_(nh),
        nh_private_(nh_private_),
        use_function_(false),
        use_color_map_(false),
        remove_alpha_(false),
        geometry_id_(0),
        color_id_(0),
        message_id_(0) 
        {
            publisher_ = nh_private_.advertise<visualization_msgs::MarkerArray>(topic, 1);
        }

    GenericActiveSubmapVisualizer(const MeshIntegratorConfig& mesh_config,
                         const std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>>&
                             geometry_submap_collection_ptr,
                         const std::shared_ptr<GenericSubmapCollection<ColorVoxelType>>&
                             color_submap_collection_ptr,
                         std::string topic,
                         ros::NodeHandle nh,
                         ros::NodeHandle nh_private) 
      : mesh_config_(mesh_config),
        geometry_submap_collection_ptr_(geometry_submap_collection_ptr),
        color_layer_(true),
        color_submap_collection_ptr_(color_submap_collection_ptr),
        color_cycle_length_(kDefaultColorCycleLength),
        current_color_idx_(0),
        verbose_(false),
        opacity_(1.0),
        nh_(nh),
        nh_private_(nh_private),
        use_function_(false),
        use_color_map_(false) {
            publisher_ = nh_private_.advertise<visualization_msgs::MarkerArray>(topic, 1);
        }


  void switchToSubmap(const SubmapID submap_id);
  void switchToActiveSubmap();

  void updateMeshLayer();

  void getDisplayMesh(visualization_msgs::Marker* marker_ptr);
  MeshLayer::Ptr getDisplayMeshLayer();

  void setVerbose(const bool& verbose) { verbose_ = verbose; }
  void setOpacity(const float& opacity) { opacity_ = opacity; }

  void setColorFunction(voxblox::Color (*color_function)(const ColorVoxelType*));
  void setUseColorMap();
  void setUseDefault();
  void setRemoveAlpha(bool val);

 private:
  // Functions called when swapping active submaps
  void createMeshLayer();
  void recoverMeshLayer();
  void updateIntegrator();

  // The active mesh is produced in the submap frame (S), and is transformed
  // into the global frame (G).
  void transformMeshLayerToGlobalFrame(const MeshLayer& mesh_layer_S,
                                       MeshLayer* mesh_layer_G_ptr) const;
  void colorMeshWithCurrentIndex(MeshLayer* mesh_layer_ptr) const;

  void recolorWithColorFunction(MeshLayer* mesh_layer_ptr) const;

  void removeAlphaChanneled(MeshLayer* mesh_layer_ptr) const;

  // Config
  const MeshIntegratorConfig mesh_config_;

  // The mesh layer for the active submap
  std::shared_ptr<MeshLayer> active_submap_mesh_layer_ptr_;
  int active_submap_color_idx_;

  // The integrator
  std::unique_ptr<MeshIntegrator<GeometryVoxelType>> active_submap_mesh_integrator_ptr_;

  // The submap collection
  std::shared_ptr<GenericSubmapCollection<GeometryVoxelType>> geometry_submap_collection_ptr_;
  SubmapID active_submap_id_;

  SubmapID color_active_submap_id_;

  // Storing the mesh layers
  std::map<SubmapID, std::shared_ptr<MeshLayer>> mesh_layers_;
  std::map<SubmapID, int> mesh_color_indices_;

  // Color stuff
  const int color_cycle_length_;
  int current_color_idx_;

  bool verbose_;
  float opacity_;

  bool color_layer_;
  std::shared_ptr<GenericSubmapCollection<ColorVoxelType>> color_submap_collection_ptr_;

  bool use_function_;
  bool use_color_map_;
  bool remove_alpha_;
  voxblox::Color (*color_function_)(const ColorVoxelType*);

  ros::Publisher publisher_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  int geometry_id_;
  int color_id_;
  int message_id_;
};

}  // namespace cblox

#endif  // CBLOX_ROS_GENERIC_ACTIVE_SUBMAP_VISUALIZER_H_
