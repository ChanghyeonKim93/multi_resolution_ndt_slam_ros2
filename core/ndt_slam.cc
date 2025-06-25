#include "ndt_slam.h"

#include <unordered_set>

namespace ndt_slam {

NDTSLAM::NDTSLAM(const Parameters& parameters) : parameters_(parameters) {
  fine_voxel_size_ =
      parameters_.voxel_size / (1 << parameters_.quadtree.max_depth);
}

void NDTSLAM::Update(const PointCloud& point_cloud) {
  // Initialize map
  if (!condition_.is_initialized) {
    pose_ = Pose2::Identity();
    UpdateMap(pose_.value(), point_cloud);
    condition_.is_initialized = true;
    return;
  }

  // Estimate pose

  // Update voxel map
}

void NDTSLAM::UpdateMap(const Pose2& initial_pose,
                        const PointCloud& point_cloud) {
  static std::unordered_set<CoarseVoxelIndex, VoxelIndexHash> hit_subindex_set;
  static std::unordered_set<CoarseVoxelIndex, VoxelIndexHash> miss_subindex_set;
  hit_subindex_set.clear();
  miss_subindex_set.clear();

  for (const auto& point : point_cloud.data) {
    const auto global_point = initial_pose * point;

    const auto hit_subindex = GetFineIndex(global_point);
    hit_subindex_set.insert(hit_subindex);
    const auto miss_subindices =
        ComputeMissVoxelIndices(initial_pose, global_point);
    for (const auto& miss_index : miss_subindices)
      miss_subindex_set.insert(miss_index);
  }

  for (const auto& hit_subindex : hit_subindex_set) {
    const auto hit_index = GetCoarseVoxelIndex(hit_subindex);
    if (voxel_occupancy_map_.find(hit_index) == voxel_occupancy_map_.end())
      voxel_occupancy_map_.insert({hit_index, 128});

    auto& current_occupancy = voxel_occupancy_map_[hit_subindex];
    ++current_occupancy;
    current_occupancy = current_occupancy > 255 ? 255 : current_occupancy;

    // Update voxel map
    if (voxel_map_.find(hit_index) == voxel_map_.end())
      voxel_map_.insert({hit_index, Voxel()});

    // Go to the leaf node and insert point
    auto& voxel = voxel_map_[hit_index];

    // Update voxel from the leaf nodes
  }

  for (const auto& miss_index : miss_subindex_set) {
    if (hit_subindex_set.find(miss_index) != hit_subindex_set.end())
      continue;  // skip if already hit

    if (voxel_occupancy_map_.find(miss_index) == voxel_occupancy_map_.end())
      voxel_occupancy_map_.insert({miss_index, 128});

    auto& current_occupancy = voxel_occupancy_map_[miss_index];
    --current_occupancy;
    current_occupancy = current_occupancy < 0 ? 0 : current_occupancy;
  }

  constexpr uint8_t kMinOccupancy = 30;
  for (auto& [voxel_index, occupancy] : voxel_occupancy_map_) {
    if (occupancy < kMinOccupancy) {
      voxel_occupancy_map_.erase(voxel_index);
      voxel_map_.erase(voxel_index);
    }
  }

  // Remove empty voxels
  std::vector<CoarseVoxelIndex> empty_voxel_indices;
  for (const auto& [voxel_index, occupancy] : voxel_occupancy_map_) {
    if (occupancy < kMinOccupancy) empty_voxel_indices.push_back(voxel_index);
  }
  for (const auto& voxel_index : empty_voxel_indices) {
    voxel_occupancy_map_.erase(voxel_index);
    voxel_map_.erase(voxel_index);
  }
}

std::vector<CoarseVoxelIndex> NDTSLAM::ComputeMissVoxelIndices(
    const Pose2& global_pose, const Vec2& global_point) {
  auto current_index = GetFineIndex(global_pose.translation());
  const auto end_index = GetFineIndex(global_point);

  Vec2 direction_vector = global_point - global_pose.translation();
  double remaining_distance = direction_vector.norm();
  direction_vector /= remaining_distance;

  const double kMaxDouble = std::numeric_limits<double>::max();
  Vec2 tMax{Vec2::Zero()};
  Vec2 tDelta{kMaxDouble, kMaxDouble};
  Vec2i index_step{Vec2i::Zero()};

  for (int i = 0; i < 2; ++i) {
    index_step[i] = (direction_vector[i] > 0) ? 1 : -1;
    tDelta[i] = std::abs(direction_vector[i]) < 1e-6
                    ? kMaxDouble
                    : fine_voxel_size_ / direction_vector[i];
  }
  std::vector<CoarseVoxelIndex> miss_voxel_indices;
  while (remaining_distance > 0.0) {
    int axis = tMax[0] < tMax[1] ? 0 : 1;
    tMax[axis] += tDelta[axis];
    remaining_distance -= tDelta[axis];
    current_index[axis] += index_step[axis];
    if (remaining_distance <= 0.0 || current_index == end_index) break;
    miss_voxel_indices.push_back(current_index);
  }
  return miss_voxel_indices;
}

CoarseVoxelIndex NDTSLAM::GetCoarseIndex(const Vec2& global_point) const {
  const double inverse_voxel_size = 1.0 / parameters_.voxel_size;
  return Vec2i(
      static_cast<int>(std::floor(global_point.x() * inverse_voxel_size)),
      static_cast<int>(std::floor(global_point.y() * inverse_voxel_size)));
}

CoarseVoxelIndex NDTSLAM::GetFineIndex(const Vec2& global_point) const {
  const double inverse_fine_voxel_size = 1.0 / fine_voxel_size_;
  return Vec2i(
      static_cast<int>(std::floor(global_point.x() * inverse_fine_voxel_size)),
      static_cast<int>(std::floor(global_point.y() * inverse_fine_voxel_size)));
}

CoarseVoxelIndex NDTSLAM::GetCoarseVoxelIndex(
    const FineVoxelIndex& fine_voxel_index) const {
  static int denominator = 1 << parameters_.quadtree.max_depth;
  const Vec2i coarse_voxel_index(fine_voxel_index.x() / denominator,
                                 fine_voxel_index.y() / denominator);
  return coarse_voxel_index;
}

LocalIndex NDTSLAM::GetLocalIndexInCoarseVoxel(
    const FineVoxelIndex& fine_voxel_index) const {
  static int denominator = 1 << parameters_.quadtree.max_depth;
  const Vec2i local_index(fine_voxel_index.x() % denominator,
                          fine_voxel_index.y() % denominator);
  return local_index;
}

}  // namespace ndt_slam