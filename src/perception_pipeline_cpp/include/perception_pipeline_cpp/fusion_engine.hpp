#pragma once

/**
 * fusion_engine.hpp
 * =================
 * Pure-logic LiDAR-camera fusion — no ROS dependency, fully unit-testable.
 *
 * For each 2D bounding box:
 *   1. Use Projector::points_in_bbox() to find LiDAR points inside the frustum
 *   2. Reject clusters with fewer than min_cluster_points
 *   3. Fit an axis-aligned 3D bounding box via min/max over the cluster
 *   4. Carry class label + confidence from the 2D detection
 *
 * Output coordinates are in the Velodyne (LiDAR) frame — same as /lidar/filtered.
 */

#include <cstdint>
#include <string>
#include <vector>

#include "perception_pipeline_cpp/projector.hpp"

namespace perception_pipeline_cpp {

// ── Config ────────────────────────────────────────────────────────────────────

struct FusionConfig {
    int   min_cluster_points{5};     // minimum LiDAR points to accept a 3D detection
    float depth_gate_min_m{4.0f};    // minimum depth slice thickness from nearest hit
    float depth_gate_scale{0.20f};   // additional slice thickness relative to nearest depth
};

// ── Input: one 2D detection ───────────────────────────────────────────────────

struct BBox2D {
    float       x1, y1, x2, y2;   // pixel coords in original image space
    std::string class_id;
    float       score{0.f};
};

// ── Output: one 3D detection ──────────────────────────────────────────────────

struct Detection3D {
    // Axis-aligned box centre in Velodyne frame (metres)
    float cx{0.f}, cy{0.f}, cz{0.f};
    // Full extents (not half-extents): x_max - x_min etc.
    float size_x{0.f}, size_y{0.f}, size_z{0.f};
    std::string class_id;
    float       score{0.f};
    uint32_t    n_points{0};  // LiDAR points in the cluster
};

struct FusionResult {
    std::vector<Detection3D> detections;
};

// ── Engine ────────────────────────────────────────────────────────────────────

class FusionEngine {
public:
    explicit FusionEngine(const FusionConfig & cfg = {});

    /**
     * Associate filtered LiDAR points with 2D detections and fit 3D boxes.
     *
     * @param detections_2d  2D bounding boxes from /detections_2d
     * @param points_xyzi    Flat float32 buffer [x,y,z,intensity, ...], N×4
     * @param n_points       Number of points (not floats)
     * @param projector      Projector built from loaded calibration (Phase 5)
     * @return               One Detection3D per accepted frustum cluster
     */
    FusionResult fuse(
        const std::vector<BBox2D> & detections_2d,
        const float *               points_xyzi,
        uint32_t                    n_points,
        const Projector &           projector) const;

private:
    FusionConfig cfg_;
};

}  // namespace perception_pipeline_cpp
