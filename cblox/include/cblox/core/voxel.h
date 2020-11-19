// maybe merge to voxel.h in voxblox
#ifndef CBLOX_CORE_VOXEL_H_
#define CBLOX_CORE_VOXEL_H_
#include <voxblox/core/color.h>

#include <voxblox/core/block.h>
namespace voxblox {

// Just for testing purposes
struct RGBVoxel {
  float weight = 0.0f;
  Color color;
};

}  // namespace voxblox
#endif  // CBLOX_CORE_VOXEL_H_