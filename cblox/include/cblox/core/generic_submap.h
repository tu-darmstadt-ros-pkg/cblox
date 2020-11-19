#ifndef CBLOX_CORE_GENERIC_SUBMAP_H_
#define CBLOX_CORE_GENERIC_SUBMAP_H_

#include "cblox/Submap.pb.h"
#include "cblox/core/common.h"
#include "cblox/core/generic_map.h"
#include "cblox/core/submap.h"

// generic version of tsdf submap

namespace cblox {

template <typename VoxelType>
class GenericSubmap : public Submap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<GenericSubmap<VoxelType>> Ptr;
  typedef std::shared_ptr<const GenericSubmap<VoxelType>> ConstPtr;
  typedef typename GenericMap<VoxelType>::Config Config;
  // TODO rethink if different maps need different configs
  // typedef GenericMap<VoxelType>::Config Config;

  GenericSubmap(const Transformation& T_M_S, SubmapID submap_id, Config config)
      : Submap(T_M_S, submap_id) {
    map_.reset(new GenericMap<VoxelType>(config));
  }

  ~GenericSubmap() {
    if (!map_.unique()) {
      LOG(WARNING) << "Underlying tsdf map from SubmapID: " << submap_id_
                   << " is NOT unique. Therefore its memory may leak.";
    } else {
      LOG(INFO) << "TsdfSubmap " << submap_id_ << " is being deleted.";
    }
  }

  // Returns the underlying TSDF map pointers.
  inline std::shared_ptr<GenericMap<VoxelType>> getMapPtr() { return map_; }
  inline const GenericMap<VoxelType>& getMap() const { return *map_; }

  inline FloatingPoint block_size() const { return map_->block_size(); }

  // Set interval in which submap was actively mapping.
  inline void startMappingTime(int64_t time) { mapping_interval_.first = time; }
  inline void stopMappingTime(int64_t time) { mapping_interval_.second = time; }

  // Access mapping interval.
  inline const std::pair<int64_t, int64_t>& getMappingInterval() const {
    return mapping_interval_;
  }

  virtual size_t getNumberOfAllocatedBlocks() const override {
    return map_->getLayer().getNumberOfAllocatedBlocks();
  }

  virtual size_t getMemorySize() const override {
    return map_->getLayer().getMemorySize();
  }

  virtual void finishSubmap() override;

  virtual void prepareForPublish() override;

  // Getting the proto for this submap.
  virtual void getProto(SubmapProto* proto) const;

  // Save the submap to file.
  virtual bool saveToStream(std::fstream* outfile_ptr) const;

  // Load a submap from stream.
  // Note(alexmillane): Returns a nullptr if load is unsuccessful.
  static std::shared_ptr<GenericSubmap<VoxelType>> LoadFromStream(
      const Config& config, std::fstream* proto_file_ptr,
      uint64_t* tmp_byte_offset_ptr);

 protected:
  std::shared_ptr<GenericMap<VoxelType>> map_;
  std::pair<int64_t, int64_t> mapping_interval_;
};

}  // namespace cblox

#endif  // CBLOX_CORE_GENERIC_SUBMAP_H_