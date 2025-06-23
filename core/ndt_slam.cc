#include "ndt_slam.h"

namespace ndt_slam {

NDTSLAM::NDTSLAM(const Parameters& parameters) : parameters_(parameters) {
  voxel_map_.clear();
}

void NDTSLAM::Update(const PointCloud& point_cloud) {
  // Initialize map
  if (!condition_.is_initialized) {
    const Pose2 initial_pose = Pose2::Identity();
    InitializeMap(initial_pose, point_cloud);

    // Initialize the pose
    pose_ = Pose2::Identity();
    condition_.is_initialized = true;
  }

  // Estimate pose

  // Update voxel map
}

void NDTSLAM::InitializeMap(const Pose2& initial_pose,
                            const PointCloud& point_cloud) {
  for (const auto& point : point_cloud.data) {
    const auto global_point = initial_pose * point;
    // Raycast
    const auto hit_index = ComputeVoxelIndex(global_point);
    const auto miss_indices =
        ComputeMissVoxelIndices(initial_pose, global_point);
  }
}

std::vector<VoxelIndex> NDTSLAM::ComputeMissVoxelIndices(
    const Pose2& global_pose, const Vec2& global_point) {
  auto current_index = ComputeVoxelIndex(global_pose.translation());
  const auto end_index = ComputeVoxelIndex(global_point);

  Vec2 direction_vector = global_point - global_pose.translation();
  double remaining_distance = direction_vector.norm();
  direction_vector /= remaining_distance;

  const double kMaxDouble = std::numeric_limits<double>::max();
  Vec2 tMax{Vec2::Zero()};
  Vec2 tDelta{kMaxDouble, kMaxDouble};
  Vec2i index_step{Vec2::Zero()};

  for (int i = 0; i < 2; ++i) {
    index_step[i] = (direction_vector[i] > 0) ? 1 : -1;
    tDelta[i] = std::abs(direction_vector[i]) < 1e-6
                    ? kMaxDouble
                    : parameters_.voxel_size / direction_vector[i];
  }
  std::vector<VoxelIndex> miss_voxel_indices;
  while (remaining_distance > 0.0) {
    int axis = tMax[0] > tMax[1] ? 0 : 1;
    tMax[axis] += tDelta[axis];
    remaining_distance -= tDelta[axis];
    current_index[axis] += index_step[axis];
    if (remaining_distance <= 0.0 || current_index == end_index) break;
    miss_voxel_indices.push_back(current_index);
  }
  return miss_voxel_indices;
}

VoxelIndex NDTSLAM::ComputeVoxelIndex(const Vec2& global_point) const {
  const double inverse_voxel_size = 1.0 / parameters_.voxel_size;
  return Vec2i(
      static_cast<int>(std::floor(global_point.x() * inverse_voxel_size)),
      static_cast<int>(std::floor(global_point.y() * inverse_voxel_size)));
}

}  // namespace ndt_slam