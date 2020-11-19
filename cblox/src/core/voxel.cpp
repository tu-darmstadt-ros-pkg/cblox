#include "cblox/core/voxel.h"
namespace voxblox {
template <>
void Block<RGBVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data) {
  constexpr size_t kNumDataPacketsPerVoxel = 3u;
  const size_t num_data_packets = data.size();
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, num_data_packets);
  for (size_t voxel_idx = 0u, data_idx = 0u;
       voxel_idx < num_voxels_ && data_idx < num_data_packets;
       ++voxel_idx, data_idx += kNumDataPacketsPerVoxel) {
    const uint32_t bytes_1 = data[data_idx];
    const uint32_t bytes_2 = data[data_idx + 1u];

    RGBVoxel& voxel = voxels_[voxel_idx];

    memcpy(&(voxel.weight), &bytes_1, sizeof(bytes_1));

    voxel.color.r = static_cast<uint8_t>(bytes_2 >> 24);
    voxel.color.g = static_cast<uint8_t>((bytes_2 & 0x00FF0000) >> 16);
    voxel.color.b = static_cast<uint8_t>((bytes_2 & 0x0000FF00) >> 8);
    voxel.color.a = static_cast<uint8_t>(bytes_2 & 0x000000FF);
  }
}

template <>
void Block<RGBVoxel>::serializeToIntegers(std::vector<uint32_t>* data) const {
  CHECK_NOTNULL(data);
  constexpr size_t kNumDataPacketsPerVoxel = 3u;
  data->clear();
  data->reserve(num_voxels_ * kNumDataPacketsPerVoxel);
  for (size_t voxel_idx = 0u; voxel_idx < num_voxels_; ++voxel_idx) {
    const RGBVoxel& voxel = voxels_[voxel_idx];

    const uint32_t* bytes_1_ptr =
        reinterpret_cast<const uint32_t*>(&voxel.weight);
    data->push_back(*bytes_1_ptr);

    data->push_back(static_cast<uint32_t>(voxel.color.a) |
                    (static_cast<uint32_t>(voxel.color.b) << 8) |
                    (static_cast<uint32_t>(voxel.color.g) << 16) |
                    (static_cast<uint32_t>(voxel.color.r) << 24));
  }
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, data->size());
}

template <>
void mergeVoxelAIntoVoxelB(const RGBVoxel& voxel_A, RGBVoxel* voxel_B) {
  float combined_weight = voxel_A.weight + voxel_B->weight;
  if (combined_weight > 0) {
    voxel_B->color = Color::blendTwoColors(voxel_A.color, voxel_A.weight,
                                           voxel_B->color, voxel_B->weight);

    voxel_B->weight = combined_weight;
  }
}

template <>
void mergeVoxelAIntoVoxelB(const IntensityVoxel& voxel_A,
                           IntensityVoxel* voxel_B) {
  float combined_weight = voxel_A.weight + voxel_B->weight;
  if (combined_weight > 0) {
    voxel_B->intensity = (voxel_A.weight * voxel_A.intensity +
                          voxel_B->weight * voxel_B->intensity) /
                         (voxel_A.weight + voxel_B->weight);

    voxel_B->weight = combined_weight;
  }
}
}  // namespace voxblox