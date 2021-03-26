#ifndef CBLOX_ROS_COLOR_FUNCTIONS_H_
#define CBLOX_ROS_COLOR_FUNCTIONS_H_

#include <cblox/core/voxel.h>

namespace cblox {


static voxblox::Color func2(const voxblox::RGBVoxel* v);
static voxblox::Color func3(const voxblox::IntensityVoxel* v);
static voxblox::Color func4(const voxblox::TsdfVoxel* v);

//TODO rethink this
static std::function<voxblox::Color(const voxblox::RGBVoxel* v)> get_rgb_color_function(std::string name);

}  // namespace cblox
#endif  // CBLOX_ROS_COLOR_FUNCTIONS_H_