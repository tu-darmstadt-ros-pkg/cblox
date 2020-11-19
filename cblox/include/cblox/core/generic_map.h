#ifndef CBLOX_CORE_GENERIC_MAP_H_
#define CBLOX_CORE_GENERIC_MAP_H_

#include <memory>
#include <mutex>
#include <string>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/interpolator/interpolator.h"

#include <voxblox/core/common.h>

// Generic Version of Tsdf Layer

namespace cblox {

template <typename VoxelType>
class GenericMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<GenericMap<VoxelType>> Ptr;
  typedef std::shared_ptr<const GenericMap<VoxelType>> ConstPtr;

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    voxblox::FloatingPoint voxel_size = 0.2;
    size_t voxels_per_side = 16u;

    std::string print() const;
  };

  explicit GenericMap(const Config& config)
      : layer_(new voxblox::Layer<VoxelType>(config.voxel_size,
                                             config.voxels_per_side)),
        interpolator_(layer_.get()) {
    block_size_ = config.voxel_size * config.voxels_per_side;
  }

  explicit GenericMap(std::shared_ptr<voxblox::Layer<VoxelType>> layer)
      : layer_(layer), interpolator_(layer_.get()) {
    if (!layer) {
      // For Python
      throw std::runtime_error(std::string("Null Layer<TsdfVoxel>::Ptr") +
                               " in TsdfMap constructor");
    }

    CHECK(layer);
    block_size = layer->block_size();
  }

  virtual ~GenericMap() {}

  voxblox::Layer<VoxelType>* getLayerPtr() { return layer_.get(); }
  const voxblox::Layer<VoxelType>* getLayerConstPtr() const {
    return layer_.get();
  }
  const voxblox::Layer<VoxelType>& getLayer() const { return *layer_; }

  voxblox::FloatingPoint block_size() const { return block_size_; }
  voxblox::FloatingPoint voxel_size() const { return layer_->voxel_size(); }

  /* NOTE(mereweth@jpl.nasa.gov)
   * EigenDRef is fully dynamic stride type alias for Numpy array slices
   * Use column-major matrices; column-by-column traversal is faster
   * Convenience alias borrowed from pybind11
   */
  using EigenDStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
  template <typename MatrixType>
  using EigenDRef = Eigen::Ref<MatrixType, 0, EigenDStride>;

  // TODO function to extract layer as vector??

  // TODO function for voxel at position??

 protected:
  voxblox::FloatingPoint block_size_;

  std::shared_ptr<voxblox::Layer<VoxelType>> layer_;

  voxblox::Interpolator<VoxelType> interpolator_;
};
}  // namespace cblox

#endif  // CBLOX_CORE_GENERIC_MAP_H_