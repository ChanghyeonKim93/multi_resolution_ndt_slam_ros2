#include <iostream>

#include "ndt_slam.h"
#include "types.h"

int main(int argc, char** argv) {
  // Create parameters for NDTSLAM
  ndt_slam::Parameters parameters;
  parameters.voxel_size = 0.4;        // Example voxel size
  parameters.quadtree.max_depth = 2;  // Example quadtree depth

  // Initialize NDTSLAM instance
  ndt_slam::NDTSLAM ndt_slam(parameters);

  Vec2 p1(1.0, 2.0);
  Vec2 p2(1.05, 2.15);
  const auto cidx1 = ndt_slam.GetCoarseIndex(p1);
  const auto fidx1 = ndt_slam.GetFineIndex(p1);
  std::cerr << "Coarse Voxel Index: (" << cidx1.x() << ", " << cidx1.y()
            << ")\n";
  std::cerr << "Fine Voxel Index: (" << fidx1.x() << ", " << fidx1.y() << ")\n";

  const auto cidx2 = ndt_slam.GetCoarseIndex(p2);
  const auto fidx2 = ndt_slam.GetFineIndex(p2);
  const auto lidx2 = ndt_slam.GetLocalIndex(fidx2);
  std::cerr << "Coarse Voxel Index: (" << cidx2.x() << ", " << cidx2.y()
            << ")\n";
  std::cerr << "Fine Voxel Index: (" << fidx2.x() << ", " << fidx2.y() << ")\n";
  std::cerr << "Local Index: (" << lidx2.x() << ", " << lidx2.y() << ")\n";

  return 0;
}