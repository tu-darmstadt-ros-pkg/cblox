#ifndef CBLOX_CORE_GENERIC_SUBMAP_COLLECTION_INL_H_
#define CBLOX_CORE_GENERIC_SUBMAP_COLLECTION_INL_H_

#include <string>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include <voxblox/integrator/merge_integration.h>
#include <voxblox/utils/protobuf_utils.h>


namespace cblox {

template <typename VoxelType>
GenericSubmapCollection<VoxelType>::GenericSubmapCollection(
    const typename GenericSubmap<VoxelType>::Config& submap_config,
    const std::vector<typename GenericSubmap<VoxelType>::Ptr>& sub_maps)
    : submap_config_(submap_config),
    has_parent_(false),
    last_parent_id_(0) {
  // Constructing from a list of existing submaps
  // NOTE(alexmillane): Relies on the submaps having unique submap IDs...
  for (const auto& submap_ptr : sub_maps) {
    const auto ret =
        id_to_submap_.insert({submap_ptr->getID(), submap_ptr});
    CHECK(ret.second) << "Attempted to construct collection from vector of "
                         "submaps containing at least one duplicate ID.";
  }
}

template <typename VoxelType>
std::vector<SubmapID> GenericSubmapCollection<VoxelType>::getIDs() const {
  std::vector<SubmapID> submap_ids;
  submap_ids.reserve(id_to_submap_.size());
  for (const auto& id_submap_pair : id_to_submap_) {
    submap_ids.emplace_back(id_submap_pair.first);
  }
  return submap_ids;
}

template <typename VoxelType>
bool GenericSubmapCollection<VoxelType>::exists(const SubmapID submap_id) const {
  // Searching for the passed submap ID
  const auto it = id_to_submap_.find(submap_id);
  return (it != id_to_submap_.end());
}

template <typename VoxelType>
void GenericSubmapCollection<VoxelType>::createNewSubmap(const Transformation& T_G_S,
                                                   const SubmapID submap_id) {
  // Checking if the submap already exists
  // NOTE(alexmillane): This hard fails the program if the submap already
  // exists. This is fairly brittle behaviour and we may want to change it at a
  // later date. Currently the onus is put in the caller to exists() before
  // creating a submap.

  const auto it = id_to_submap_.find(submap_id);
  CHECK(it == id_to_submap_.end());
  // Creating the new submap and adding it to the list
  Transformation T_G;
  if (has_parent_) {
    T_G = last_parent_transform_;
  } else{
    T_G = T_G_S;
  }
  typename GenericSubmap<VoxelType>::Ptr sub_map(
      new GenericSubmap<VoxelType>(T_G, submap_id, submap_config_));
  id_to_submap_.emplace(submap_id, std::move(sub_map));
  // Updating the active submap
  active_submap_id_ = submap_id;

  if (has_parent_) {
    if (parent_to_child_.find(last_parent_id_) == parent_to_child_.end())
      parent_to_child_.emplace(last_parent_id_, std::vector<SubmapID>());
    parent_to_child_.find(last_parent_id_)->second.push_back(submap_id);
  }
}

template <typename VoxelType>
SubmapID GenericSubmapCollection<VoxelType>::createNewSubmap(
    const Transformation& T_G_S) {
  // Creating a submap with a generated SubmapID
  // NOTE(alexmillane): rbegin() returns the pair with the highest key.
  SubmapID new_ID = 0;
  if (!id_to_submap_.empty()) {
    new_ID = id_to_submap_.rbegin()->first + 1;
  }
  createNewSubmap(T_G_S, new_ID);
  return new_ID;
}

template <typename VoxelType>
void GenericSubmapCollection<VoxelType>::addSubmap(
    const typename GenericSubmap<VoxelType>::Ptr submap) {
  // Check ID not already in the collection
  const SubmapID submap_id = submap->getID();
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it == id_to_submap_.end());
  // Add
  id_to_submap_.emplace(submap_id, std::move(submap));
  active_submap_id_ = submap_id;
}

template <typename VoxelType>
bool GenericSubmapCollection<VoxelType>::duplicateSubmap(
    const SubmapID source_submap_id, const SubmapID new_submap_id) {
  // Get pointer to the source submap
  const auto src_submap_ptr_it = id_to_submap_.find(source_submap_id);
  if (src_submap_ptr_it != id_to_submap_.end()) {
    typename GenericSubmap<VoxelType>::Ptr src_submap_ptr = src_submap_ptr_it->second;
    // Create a new submap with the same pose and get its pointer
    const Transformation T_G_S = src_submap_ptr->getPose();
    // Creating the new submap and adding it to the list
    typename GenericSubmap<VoxelType>::Ptr new_sub_map(
        new GenericSubmap<VoxelType>(T_G_S, new_submap_id, submap_config_));
    // Reset the GenericMap<VoxelType> based on a copy of the source submap's TSDF layer
    // TODO(victorr): Find a better way to do this, however with .reset(...) as
    // below the new submap appears empty
    // new_tsdf_sub_map->getGenericMap<VoxelType>Ptr().reset(new
    // GenericMap<VoxelType>(src_submap_ptr->getGenericMap<VoxelType>().getTsdfLayer()));
    *(new_sub_map->getMapPtr()) =
        *(new GenericMap<VoxelType>(src_submap_ptr->getMap().getLayer()));
    id_to_submap_.emplace(new_submap_id, new_sub_map);
    return true;
  }
  return false;
}

template <typename VoxelType>
const GenericSubmap<VoxelType>& GenericSubmapCollection<VoxelType>::getSubmap(
    const SubmapID submap_id) const {
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it != id_to_submap_.end());
  return *it->second;
}

template <typename VoxelType>
const std::vector<typename GenericSubmap<VoxelType>::Ptr>
GenericSubmapCollection<VoxelType>::getSubmapPtrs() const {
  std::vector<typename GenericSubmap<VoxelType>::Ptr> submap_ptrs;
  for (const auto& id_submap_pair : id_to_submap_) {
    submap_ptrs.emplace_back(id_submap_pair.second);
  }
  return submap_ptrs;
}

template <typename VoxelType>
const std::vector<typename GenericSubmap<VoxelType>::ConstPtr>
GenericSubmapCollection<VoxelType>::getSubmapConstPtrs() const {
  std::vector<typename GenericSubmap<VoxelType>::ConstPtr> submap_ptrs;
  for (const auto& id_submap_pair : id_to_submap_) {
    submap_ptrs.emplace_back(id_submap_pair.second);
  }
  return submap_ptrs;
}

template <typename VoxelType>
typename GenericMap<VoxelType>::Ptr GenericSubmapCollection<VoxelType>::getActiveMapPtr() {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return (it->second)->getMapPtr();
}

template <typename VoxelType>
typename GenericMap<VoxelType>::Ptr GenericSubmapCollection<VoxelType>::getMapPtr(
    const SubmapID submap_id) {
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it != id_to_submap_.end());
  return (it->second)->getMapPtr();
}

template <typename VoxelType>
const GenericMap<VoxelType>& GenericSubmapCollection<VoxelType>::getActiveMap() const {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return (it->second)->getMap();
}

template <typename VoxelType>
const GenericSubmap<VoxelType>& GenericSubmapCollection<VoxelType>::getActiveSubmap() const {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return *(it->second);
}

// Gets a pointer to the active submap
template <typename VoxelType>
typename GenericSubmap<VoxelType>::Ptr GenericSubmapCollection<VoxelType>::getActiveSubmapPtr() {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return it->second;
}

template <typename VoxelType>
const Transformation& GenericSubmapCollection<VoxelType>::getActiveSubmapPose()
    const {
  return getActiveSubmap().getPose();
}
template <typename VoxelType>
SubmapID GenericSubmapCollection<VoxelType>::getActiveSubmapID() const {
  return active_submap_id_;
}

template <typename VoxelType>
void GenericSubmapCollection<VoxelType>::activateSubmap(const SubmapID submap_id) {
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it != id_to_submap_.end());
  active_submap_id_ = submap_id;
}

template <typename VoxelType>
typename GenericMap<VoxelType>::Ptr GenericSubmapCollection<VoxelType>::getProjectedMap() const {
  // Creating the global tsdf map and getting its tsdf layer
  typename GenericMap<VoxelType>::Ptr projected_map_ptr =
      GenericMap<VoxelType>::Ptr(new GenericMap<VoxelType>(submap_config_));
  Layer<VoxelType>* combined_layer_ptr =
      projected_map_ptr->getTsdfLayerPtr();
  // Looping over the current submaps
  for (const auto& id_submap_pair : id_to_submap_) {
    // Getting the tsdf submap and its pose
    const GenericMap<VoxelType>& map = (id_submap_pair.second)->getMap();
    const Transformation& T_G_S = (id_submap_pair.second)->getPose();
    // Merging layers the submap into the global layer
    mergeLayerAintoLayerB(map.getLayer(), T_G_S,
                          combined_layer_ptr);
  }
  // Returning the new map
  return projected_map_ptr;
}

template <typename VoxelType>
bool GenericSubmapCollection<VoxelType>::setSubmapPose(const SubmapID submap_id,
                                                 const Transformation& pose) {
  // Looking for the submap
  const auto submap_ptr_it = id_to_submap_.find(submap_id);
  if (submap_ptr_it != id_to_submap_.end()) {
    typename GenericSubmap<VoxelType>::Ptr submap_ptr = (*submap_ptr_it).second;
    submap_ptr->setPose(pose);
    return true;
  } else {
    LOG(WARNING) << "Tried to set the pose of the submap with submap_id: "
                 << submap_id << " and could not find the linked submap.";
    return false;
  }
}

template <typename VoxelType>
void GenericSubmapCollection<VoxelType>::setSubmapPoses(
    const SubmapIdPoseMap& id_pose_map) {
  for (const SubmapIdPosePair& id_pose_pair : id_pose_map) {
    setSubmapPose(id_pose_pair.first, id_pose_pair.second);
  }
}

template <typename VoxelType>
bool GenericSubmapCollection<VoxelType>::getSubmapPose(
    const SubmapID submap_id, Transformation* pose_ptr) const {
  // Looking for the submap
  const auto submap_ptr_it = id_to_submap_.find(submap_id);
  if (submap_ptr_it != id_to_submap_.end()) {
    typename GenericSubmap<VoxelType>::Ptr submap_ptr = (*submap_ptr_it).second;
    *pose_ptr = submap_ptr->getPose();
    return true;
  } else {
    LOG(INFO) << "Tried to get the pose of the submap with submap_id: "
              << submap_id << " and could not find the linked submap.";
    return false;
  }
}

template <typename VoxelType>
void GenericSubmapCollection<VoxelType>::getSubmapPoses(
    AlignedVector<Transformation>* submap_poses_ptr) const {
  CHECK_NOTNULL(submap_poses_ptr);
  // Extracting transforms
  submap_poses_ptr->clear();
  submap_poses_ptr->reserve(id_to_submap_.size());
  for (const auto& id_submap_pair : id_to_submap_) {
    submap_poses_ptr->push_back((id_submap_pair.second)->getPose());
  }
}

template <typename VoxelType>
typename GenericSubmap<VoxelType>::Ptr GenericSubmapCollection<VoxelType>::getSubmapPtr(
    const SubmapID submap_id) {
  const auto submap_ptr_it = id_to_submap_.find(submap_id);
  if (submap_ptr_it != id_to_submap_.end()) {
    return submap_ptr_it->second;
  } else {
    return typename GenericSubmap<VoxelType>::Ptr();
  }
}

template <typename VoxelType>
typename GenericSubmap<VoxelType>::ConstPtr GenericSubmapCollection<VoxelType>::getSubmapConstPtr(
    const SubmapID submap_id) const {
  const auto submap_ptr_it = id_to_submap_.find(submap_id);
  if (submap_ptr_it != id_to_submap_.end()) {
    return submap_ptr_it->second;
  } else {
    return typename GenericSubmap<VoxelType>::ConstPtr();
  }
}

template <typename VoxelType>
bool GenericSubmapCollection<VoxelType>::saveToFile(
    const std::string& file_path) const {
  // Opening the file (if we can)
  CHECK(!file_path.empty());
  std::fstream outfile;
  outfile.open(file_path, std::fstream::out | std::fstream::binary);
  if (!outfile.is_open()) {
    LOG(ERROR) << "Could not open file for writing: " << file_path;
    return false;
  }
  // Saving the submap collection header object
  GenericSubmapCollectionProto submap_collection_proto;
  getProto(&submap_collection_proto);
  // Write out the layer header.
  if (!voxblox::utils::writeProtoMsgToStream(submap_collection_proto,
                                             &outfile)) {
    LOG(ERROR) << "Could not write submap collection header message.";
    outfile.close();
    return false;
  }
  // Saving the tsdf submaps
  for (const auto& id_submap_pair : id_to_submap_) {
    LOG(INFO) << "Saving submap with ID: " << id_submap_pair.first;
    // Saving the submap
    bool success = (id_submap_pair.second)->saveToStream(&outfile);
    if (success) {
      LOG(INFO) << "Saving successful";
    } else {
      LOG(WARNING) << "Saving unsuccessful";
    }
  }
  // Closing the file
  outfile.close();
  return true;
}

template <typename VoxelType>
void GenericSubmapCollection<VoxelType>::getProto(
    GenericSubmapCollectionProto* proto) const {
  CHECK_NOTNULL(proto);
  // Filling out the description of the submap collection
  proto->set_num_submaps(num_patches());
}

template <typename VoxelType>
bool GenericSubmapCollection<VoxelType>::LoadFromFile(
    const std::string& file_path,
    typename GenericSubmapCollection<VoxelType>::Ptr* submap_collection_ptr) {
  CHECK_NOTNULL(submap_collection_ptr);
  // Open and check the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load layer: " << file_path;
    return false;
  }
  // Unused byte offset result.
  uint64_t tmp_byte_offset = 0u;
  // Loading the header
  GenericSubmapCollectionProto submap_collection_proto;
  if (!voxblox::utils::readProtoMsgFromStream(
          &proto_file, &submap_collection_proto, &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read generic submap collection map protobuf message.";
    return false;
  }

  LOG(INFO) << "submap_collection_proto.num_submaps(): "
            << submap_collection_proto.num_submaps();

  // Loading each of the submaps
  for (size_t sub_map_index = 0u;
       sub_map_index < submap_collection_proto.num_submaps(); ++sub_map_index) {
    LOG(INFO) << "Loading submap number: " << sub_map_index;
    // Loading the submaps
    typename GenericSubmap<VoxelType>::Ptr submap_ptr = GenericSubmap<VoxelType>::LoadFromStream(
        (*submap_collection_ptr)->getConfig(), &proto_file, &tmp_byte_offset);
    if (submap_ptr == nullptr) {
      LOG(ERROR) << "Could not load the submap from stream.";
      return false;
    }
    (*submap_collection_ptr)->addSubmap(submap_ptr);
  }
  // Because grown ups clean up after themselves
  proto_file.close();
  return true;
}

template <typename VoxelType>
void GenericSubmapCollection<VoxelType>::fuseSubmapPair(
    const SubmapIdPair& submap_id_pair) {
  // Extracting the submap IDs
  SubmapID submap_id_1 = submap_id_pair.first;
  SubmapID submap_id_2 = submap_id_pair.second;
  LOG(INFO) << "Fusing submap pair: (" << submap_id_1 << ", " << submap_id_2
            << ")";
  // Getting the requested submaps
  const auto id_submap_pair_1_it = id_to_submap_.find(submap_id_1);
  const auto id_submap_pair_2_it = id_to_submap_.find(submap_id_2);
  // If the submaps are found
  if ((id_submap_pair_1_it != id_to_submap_.end()) &&
      (id_submap_pair_2_it != id_to_submap_.end())) {
    // Getting the submaps
    typename GenericSubmap<VoxelType>::Ptr submap_ptr_1 = (*id_submap_pair_1_it).second;
    typename GenericSubmap<VoxelType>::Ptr submap_ptr_2 = (*id_submap_pair_2_it).second;
    // Checking that we're not trying to fuse a submap into itself. This can
    // occur due to fusing submap pairs in a triangle.
    if (submap_ptr_1->getID() == submap_ptr_2->getID()) {
      return;
    }
    // Getting the tsdf submap and its pose
    const Transformation& T_G_S1 = submap_ptr_1->getPose();
    const Transformation& T_G_S2 = submap_ptr_2->getPose();
    const Transformation T_S1_S2 = T_G_S1.inverse() * T_G_S2;
    // Merging the submap layers
    mergeLayerAintoLayerB(submap_ptr_2->getMap().getLayer(), T_S1_S2,
                          submap_ptr_1->getMapPtr()->getLayerPtr());
    // Deleting Submap #2
    const size_t num_erased = id_to_submap_.erase(submap_id_2);
    CHECK_EQ(num_erased, 1);
    LOG(INFO) << "Erased the submap: " << submap_ptr_2->getID()
              << " from the submap collection";

  } else {
    LOG(WARNING) << "Could not find the requested submap pair during fusion.";
  }
}

template <typename VoxelType>
size_t GenericSubmapCollection<VoxelType>::getNumberOfAllocatedBlocks() const {
  // Looping over the submaps totalling the sizes
  size_t total_blocks = 0;
  for (const auto& id_submap_pair : id_to_submap_) {
    total_blocks += (id_submap_pair.second)->getNumberOfAllocatedBlocks();
  }
  return total_blocks;
}

template <typename VoxelType>
size_t GenericSubmapCollection<VoxelType>::getMemorySize() const {
  // Looping over the submaps totalling the sizes
  size_t size = 0u;
  for (const auto& id_submap_pair : id_to_submap_) {
    size += (id_submap_pair.second)->getMemorySize();
  }
  return size;
}
template <typename VoxelType>
std::vector<SubmapID> GenericSubmapCollection<VoxelType>::getChildMapIDs(SubmapID parent) {
  std::vector<SubmapID> vec;
  if (!has_parent_) {
    vec.push_back(parent);
    return vec;
  }

  auto res = parent_to_child_.find(parent);
  if (res != parent_to_child_.end())
    vec = res->second;
  return vec;
}

template <typename VoxelType>
std::vector<typename GenericSubmap<VoxelType>::Ptr> GenericSubmapCollection<VoxelType>::getChildMaps(SubmapID parent) {
  std::vector<typename GenericSubmap<VoxelType>::Ptr> vec;
  if (!has_parent_) {
    vec.push_back(id_to_submap_.find(parent)->second);
    return vec;
  }

  for (auto child : getChildMapIDs(parent)) {
    vec.push_back((id_to_submap_.find(child)->second));
  }  
  return vec;
}

//like before but use parent transform and do id handling
template <typename VoxelType>
void GenericSubmapCollection<VoxelType>::createNewChildSubMap(const Transformation& T_G_P, const SubmapID submap_id, const SubmapID parent) {
  has_parent_ = true;
  last_parent_id_ = parent;
  last_parent_transform_ = T_G_P;
  createNewSubmap(T_G_P, submap_id);
}

template <typename VoxelType>
SubmapID GenericSubmapCollection<VoxelType>::createNewChildSubMap(const Transformation& T_G_P, const SubmapID parent) {
  SubmapID new_ID = 0;
  if (!id_to_submap_.empty()) {
    new_ID = id_to_submap_.rbegin()->first + 1;
  }
  createNewChildSubMap(T_G_P, new_ID, last_parent_id_);
  return new_ID;
}
}  // namespace cblox

#endif  // CBLOX_CORE_GENERIC_SUBMAP_COLLECTION_INL_H_
