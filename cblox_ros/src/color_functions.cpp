#include "cblox_ros/color_functions.h"

namespace cblox {

  static voxblox::Color func3(const voxblox::IntensityVoxel* v) {
  voxblox::Color c;
  c.r = 0;
  c.g = 0;
  c.b = 0;
  c.a = 0;
  if (v == nullptr) {
    return c;
  }
  if (v->weight > 0.0) {
    voxblox::IronbowColorMap m;
    //std::cout << v->intensity << std::endl;
    return m.colorLookup(v->intensity);
  }

  return c;
}

static voxblox::Color func2(const voxblox::RGBVoxel* v) {
  voxblox::Color c;
  c.r = 0;
  c.g = 0;
  c.b = 0;
  c.a = 0;
  if (v == nullptr) {
    return c;
  }
  if (v->weight > 0.0) return v->color;

  return c;
}

static voxblox::Color func4(const voxblox::TsdfVoxel* v) {
  voxblox::Color c;
  c.r = 0;
  c.g = 0;
  c.b = 0;
  c.a = 0;
  if (v == nullptr) {
    return c;
  }
  c.r = 255;
  c.a = 255;

  return c;
}
}