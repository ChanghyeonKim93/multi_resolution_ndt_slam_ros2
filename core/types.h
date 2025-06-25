#ifndef TYPES_H_
#define TYPES_H_

#include <unordered_map>

#include "Eigen/Dense"

using Pose2 = Eigen::Transform<double, 2, 1>;
using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec2i = Eigen::Matrix<int, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec3i = Eigen::Matrix<int, 3, 1>;
using Mat2x2 = Eigen::Matrix<double, 2, 2>;
using Mat3x3 = Eigen::Matrix<double, 3, 3>;

struct TimedPose2 {
  double time{0.0};
  Pose2 pose{Pose2::Identity()};
};

struct PointCloud {
  double time{0.0};
  std::vector<Vec2> data;
};

using CoarseVoxelIndex = Vec2i;
using FineVoxelIndex = Vec2i;
using LocalIndex = Vec2i;

struct VoxelIndexHash {
  uint64_t operator()(const CoarseVoxelIndex& index) const {
    // Morton order hash function
    const int x_key = index.x() < 0 ? -2 * index.x() - 1 : 2 * index.x();
    const int y_key = index.y() < 0 ? -2 * index.y() - 1 : 2 * index.y();
    return (x_key + y_key) * (x_key + y_key + 1) / 2 + y_key;
  }
};

struct SubVoxel {
  Vec2 sum{Vec2::Zero()};
  Mat2x2 moment{Mat2x2::Zero()};
  uint16_t count{0};
  Vec2 left_bottom{Vec2::Zero()};
  double voxel_size{0.0};
  uint8_t depth{0};
};

struct Voxel {
  Vec2 mean{Vec2::Zero()};
  Mat2x2 information{Mat2x2::Zero()};
  Vec2 normal_vector{Vec2::Zero()};
  uint16_t count{0};
  bool is_valid{false};
  bool is_planar{false};

  std::unordered_map<uint64_t, SubVoxel> sub_voxels;
};

using VoxelMap = std::unordered_map<CoarseVoxelIndex, Voxel, VoxelIndexHash>;
using VoxelOccupancyMap =
    std::unordered_map<FineVoxelIndex, uint8_t, VoxelIndexHash>;

#endif  // TYPES_H_