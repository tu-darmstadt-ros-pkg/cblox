//maybe merge to voxel.h in voxblox
#ifndef CBLOX_CORE_VOXEL_H_
#define CBLOX_CORE_VOXEL_H_
#include <voxblox/core/color.h>
namespace voxblox{



//Just for testing purposes
struct RGBVoxel {
    float weight = 0.0f;
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    Color color;
};

}
#endif  //CBLOX_CORE_VOXEL_H_