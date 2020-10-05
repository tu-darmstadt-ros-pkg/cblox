#ifndef CBLOX_CORE_MAP_CONFIG_H_
#define CBLOX_CORE_MAP_CONFIG_H_

#include "voxblox/core/common.h"

namespace cblox {
    //TODO
    struct RGBMapConfig {

    };

    struct TsdfMapConfig {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FloatingPoint voxel_size = 0.2;
        size_t voxels_per_side = 16u;

        std::string print() const;
    };
}

#endif // CBLOX_CORE_MAP_CONFIG_H_