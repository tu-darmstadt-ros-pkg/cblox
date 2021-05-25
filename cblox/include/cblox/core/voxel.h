// maybe merge to voxel.h in voxblox
#ifndef CBLOX_CORE_VOXEL_H_
#define CBLOX_CORE_VOXEL_H_
#include <voxblox/core/block.h>
#include <voxblox/core/color.h>
#include <voxblox/core/voxel.h>
namespace voxblox {

// Just for testing purposes
struct RGBVoxel {
  float weight = 0.0f;
  Color color;
};

namespace voxel_types {
const std::string kRGB = "rgb";
}  // namespace voxel_types

template <>
inline std::string getVoxelType<RGBVoxel>() {
  return voxel_types::kRGB;
}

}  // namespace voxblox

#endif  // CBLOX_CORE_VOXEL_H_