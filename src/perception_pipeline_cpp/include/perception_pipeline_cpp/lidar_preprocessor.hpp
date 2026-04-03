#pragma once

#include <cstdint>
#include <vector>

namespace perception_pipeline_cpp {

struct LidarPreprocessorConfig {
  float roi_x_min{0.0f},   roi_x_max{50.0f};
  float roi_y_min{-10.0f}, roi_y_max{10.0f};
  float roi_z_min{-3.0f},  roi_z_max{2.0f};
  float voxel_size{0.1f};
  float ransac_dist{0.2f};
  int   ransac_iter{100};
  float max_depth{50.0f};
};

struct LidarPreprocessorResult {
  // Flat float32 buffers: N*4 values in [x, y, z, intensity] order
  std::vector<float> filtered;   // non-ground points
  std::vector<float> ground;     // ground plane points
  struct {
    uint32_t n_input, n_roi, n_voxel, n_ground, n_output;
  } stats{};
};

/**
 * LidarPreprocessor
 * =================
 * Pure C++ preprocessing pipeline — no ROS dependency, fully unit-testable.
 *
 * Stages (in order):
 *   1. ROI crop           — discard points outside a forward-facing box
 *   2. Voxel downsample   — PCL VoxelGrid uniform density reduction
 *   3. Distance filter    — discard points beyond max_depth
 *   4. RANSAC ground removal — PCL SACSegmentation plane fitting
 */
class LidarPreprocessor {
public:
  explicit LidarPreprocessor(const LidarPreprocessorConfig & cfg = {});

  /**
   * Run the full preprocessing pipeline.
   *
   * @param points_xyzi  Pointer to flat float32 array, N*4 values [x,y,z,intensity]
   * @param n_points     Number of points (not floats)
   */
  LidarPreprocessorResult process(const float * points_xyzi, uint32_t n_points) const;

private:
  LidarPreprocessorConfig cfg_;
};

}  // namespace perception_pipeline_cpp
