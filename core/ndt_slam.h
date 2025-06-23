#ifndef NDT_SLAM_H_
#define NDT_SLAM_H_

#include <optional>
#include <vector>

#include "types.h"

namespace ndt_slam {

struct Parameters {
  double voxel_size{0.2};
  struct {
    int max_depth{2};
  } quadtree;
};

struct Condition {
  bool is_initialized{false};
};

class NDTSLAM {
 public:
  explicit NDTSLAM(const Parameters& parameters);

  void Update(const PointCloud& point_cloud);

 private:
  void UpdateMap(const Pose2& global_pose, const PointCloud& point_cloud);
  std::vector<VoxelIndex> ComputeMissVoxelIndices(const Pose2& global_pose,
                                                  const Vec2& global_point);
  bool OptimizePose(const std::vector<Vec2>& points, Pose2* pose);

  VoxelIndex ComputeVoxelIndex(const Vec2& point) const;
  VoxelIndex ComputeVoxelIndex(const VoxelIndex& fine_voxel_index) const;
  VoxelIndex ComputeSubVoxelIndex(const Vec2& point) const;

  const Parameters parameters_;
  double fine_voxel_size_{0.0};

  VoxelMap voxel_map_;
  VoxelOccupancyMap voxel_occupancy_map_;

  std::optional<Pose2> pose_;

  Condition condition_;
};

}  // namespace ndt_slam

#endif  // NDT_SLAM_H_