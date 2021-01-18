#ifndef CBLOX_CORE_GENERIC_SUBMAP_COLLECTION_H_
#define CBLOX_CORE_GENERIC_SUBMAP_COLLECTION_H_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "cblox/core/generic_map.h"
#include "cblox/core/generic_submap.h"

#include "cblox/GenericSubmapCollection.pb.h"
#include "cblox/core/common.h"
#include "voxblox/mesh/mesh_layer.h"

namespace cblox {

// Doing this, as the normal submap collection interface even though it allows
// different subtypes, requires to have a tsdf layer in a map

// A interface for use where the type of submap doesnt matter.
template <typename VoxelType>
class GenericSubmapCollectionInterface {
 public:
  typedef std::shared_ptr<GenericSubmapCollectionInterface> Ptr;
  typedef std::shared_ptr<const GenericSubmapCollectionInterface> ConstPtr;

  GenericSubmapCollectionInterface() {}

  // NOTE(alexmillane): I'm moving methods over only as I need them. There's no
  // design intent here in leaving some out. There is only the intent to be
  // lazy.
  virtual const Transformation& getActiveSubmapPose() const = 0;
  virtual SubmapID getActiveSubmapID() const = 0;
  virtual bool getSubmapPose(const SubmapID submap_id,
                             Transformation* pose_ptr) const = 0;

  virtual typename GenericMap<VoxelType>::Ptr getActiveMapPtr() = 0;
  virtual const GenericMap<VoxelType>& getActiveMap() const = 0;
  virtual typename GenericMap<VoxelType>::Ptr getMapPtr(
      const SubmapID submap_id) = 0;

  virtual bool empty() const = 0;
  virtual size_t size() const = 0;
  virtual size_t num_patches() const = 0;
  virtual FloatingPoint block_size() const = 0;
};

// Collection of submaps
template <typename VoxelType>
class GenericSubmapCollection
    : public GenericSubmapCollectionInterface<VoxelType> {
 public:
  typedef std::shared_ptr<GenericSubmapCollection> Ptr;
  typedef std::shared_ptr<const GenericSubmapCollection> ConstPtr;

  // Constructor. Constructs an empty submap collection map
  explicit GenericSubmapCollection(
      const typename GenericSubmap<VoxelType>::Config& submap_config)
      : GenericSubmapCollectionInterface<VoxelType>(),
        submap_config_(submap_config),
        active_submap_id_(-1) {}

  // Constructor. Constructs a submap collection from a list of submaps
  GenericSubmapCollection(
      const typename GenericSubmap<VoxelType>::Config& submap_config,
      const std::vector<typename GenericSubmap<VoxelType>::Ptr>& sub_maps);

  // Gets a vector of the linked IDs
  std::vector<SubmapID> getIDs() const;
  bool exists(const SubmapID submap_id) const;

  // Creates a new submap on the top of the collection
  // NOTE(alexmillane): T_G_S - Transformation between submap frame (S) and
  //                           the global tracking frame (G).
  // NOTE(alexmillane): Creating a new submap automatically makes it active.
  void createNewSubmap(const Transformation& T_G_S, const SubmapID submap_id);
  SubmapID createNewSubmap(const Transformation& T_G_S);

  void addSubmap(const typename GenericSubmap<VoxelType>::Ptr submap);

  // Create a new submap which duplicates an existing source submap
  bool duplicateSubmap(const SubmapID source_submap_id,
                       const SubmapID new_submap_id);

  // Gets a const pointer to a raw submap
  // NOTE(alexmillane): This function hard fails when the submap doesn't
  // exist... This puts the onus on the caller to call exists() first. I don't
  // like this but I can't see a solution.
  const GenericSubmap<VoxelType>& getSubmap(const SubmapID submap_id) const;
  // Note(alexmillane): Unlike the above method, the two methods below return a
  // nullptr when the map doesn't exist. No hard crash.
  typename GenericSubmap<VoxelType>::Ptr getSubmapPtr(const SubmapID submap_id);
  typename GenericSubmap<VoxelType>::ConstPtr getSubmapConstPtr(
      const SubmapID submap_id) const;
  // A list of the submaps
  const std::vector<typename GenericSubmap<VoxelType>::Ptr> getSubmapPtrs()
      const;
  const std::vector<typename GenericSubmap<VoxelType>::ConstPtr>
  getSubmapConstPtrs() const;

  // Interactions with the active submap
  const GenericSubmap<VoxelType>& getActiveSubmap() const;
  typename GenericSubmap<VoxelType>::Ptr getActiveSubmapPtr();
  const Transformation& getActiveSubmapPose() const;
  SubmapID getActiveSubmapID() const;

  // Access the tsdf_map member of the active submap
  typename GenericMap<VoxelType>::Ptr getActiveMapPtr();
  const GenericMap<VoxelType>& getActiveMap() const;
  // Access the tsdf_map member of any submap
  virtual typename GenericMap<VoxelType>::Ptr getMapPtr(
      const SubmapID submap_id);

  // Activate a submap
  // NOTE(alexmillane): Note that creating a new submap automatically activates
  //                    it.
  void activateSubmap(const SubmapID submap_id);

  // Interacting with the submap poses
  bool setSubmapPose(const SubmapID submap_id, const Transformation& pose);
  void setSubmapPoses(const SubmapIdPoseMap& id_pose_map);
  bool getSubmapPose(const SubmapID submap_id, Transformation* pose_ptr) const;
  void getSubmapPoses(TransformationVector* submap_poses) const;

  // Clears the collection, leaving an empty map
  void clear() { id_to_submap_.clear(); }

  // Size information
  bool empty() const { return id_to_submap_.empty(); }
  size_t size() const { return id_to_submap_.size(); }
  size_t num_patches() const { return id_to_submap_.size(); }
  FloatingPoint block_size() const {
    return (id_to_submap_.begin()->second)->block_size();
  }
  size_t getNumberOfAllocatedBlocks() const;

  // Returns the config of the submaps
  const typename GenericSubmap<VoxelType>::Config& getConfig() const {
    return submap_config_;
  }

  // Save the collection to file
  bool saveToFile(const std::string& file_path) const;
  void getProto(GenericSubmapCollectionProto* proto) const;

  // Fusing the submap pairs
  // Note(alexmillane): This function is not thread-safe. The user must take
  // care to ensure the fused submaps are not being modified while the fusion
  // takes place. Or someone can implement locking of the underlying submaps and
  // make a PR. :).
  void fuseSubmapPair(const SubmapIdPair& submap_id_pair);

  // Flattens the collection map down to a normal TSDF map
  typename GenericMap<VoxelType>::Ptr getProjectedMap() const;

  // Gets the combined memory size of the layers in this collection.
  size_t getMemorySize() const;

  // Loading from file
  static bool LoadFromFile(
      const std::string& file_path,
      typename GenericSubmapCollection<VoxelType>::Ptr* submap_collection_ptr);

  // TODO rethink mutex/check if needed later
  mutable std::mutex collection_mutex_;

  // returns child maps, if this hasn't a parent, return the ID itself
  std::vector<SubmapID> getChildMapIDs(SubmapID parent);

  std::vector<typename GenericSubmap<VoxelType>::Ptr> getChildMaps(
      SubmapID parent);

  std::vector<typename GenericSubmap<VoxelType>::Ptr> getAllMaps();

  void createNewChildSubMap(const Transformation& T_P_S,
                            const SubmapID submap_id, const SubmapID parent);

  SubmapID createNewChildSubMap(const Transformation& T_P_S,
                                const SubmapID parent);


  std::shared_ptr<voxblox::MeshLayer> getSubmapMeshLayer(const SubmapID submap_id);

  std::shared_ptr<voxblox::MeshLayer> createSubmapMeshLayer(const SubmapID submap_id);

  std::shared_ptr<voxblox::MeshLayer> recoverSubmapMeshLayer(const SubmapID submap_id);

 private:
  // TODO(alexmillane): Get some concurrency guards

  // The config used for the patches
  typename GenericSubmap<VoxelType>::Config submap_config_;

  // The active SubmapID
  SubmapID active_submap_id_;

  // Submap storage and access
  std::map<SubmapID, typename GenericSubmap<VoxelType>::Ptr> id_to_submap_;

  // Mapping of parent maps to child maps. Empty if there is no parent map
  std::map<SubmapID, std::vector<SubmapID>> parent_to_child_;

  bool has_parent_;

  SubmapID last_parent_id_;
  Transformation last_parent_transform_;

  std::map<SubmapID, std::shared_ptr<voxblox::MeshLayer>> mesh_collection_;
};

}  // namespace cblox

#endif  // CBLOX_CORE_GENERIC_SUBMAP_COLLECTION_H_

#include "cblox/core/generic_submap_collection_inl.h"
