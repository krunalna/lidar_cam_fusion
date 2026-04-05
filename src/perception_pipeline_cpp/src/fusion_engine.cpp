/**
 * fusion_engine.cpp
 * =================
 * Frustum-based LiDAR-camera fusion.
 *
 * For each 2D bbox:
 *   - Project all LiDAR points onto the image plane via Projector
 *   - Collect indices of points whose pixel falls inside the bbox
 *   - If count >= min_cluster_points, compute min/max in Velodyne frame
 *   - Emit a Detection3D with centre, size, label, and confidence
 *
 * All coordinates are in the Velodyne frame (same as /lidar/filtered).
 */

#include "perception_pipeline_cpp/fusion_engine.hpp"

#include <algorithm>
#include <limits>

namespace perception_pipeline_cpp {

FusionEngine::FusionEngine(const FusionConfig & cfg)
: cfg_(cfg) {}

FusionResult FusionEngine::fuse(
    const std::vector<BBox2D> & detections_2d,
    const float *               points_xyzi,
    uint32_t                    n_points,
    const Projector &           projector) const
{
    FusionResult result;

    if (n_points == 0 || detections_2d.empty()) {
        return result;
    }

    for (const auto & bbox : detections_2d) {
        // ── Step 1: frustum association ───────────────────────────────────────
        const auto indices = projector.points_in_bbox(
            points_xyzi, n_points,
            bbox.x1, bbox.y1, bbox.x2, bbox.y2);

        // ── Step 2: minimum evidence check ───────────────────────────────────
        if (static_cast<int>(indices.size()) < cfg_.min_cluster_points) {
            continue;
        }

        // ── Step 3: axis-aligned min/max box in Velodyne frame ────────────────
        float x_min =  std::numeric_limits<float>::max();
        float x_max = -std::numeric_limits<float>::max();
        float y_min =  std::numeric_limits<float>::max();
        float y_max = -std::numeric_limits<float>::max();
        float z_min =  std::numeric_limits<float>::max();
        float z_max = -std::numeric_limits<float>::max();

        for (const uint32_t idx : indices) {
            const float * p = points_xyzi + idx * 4;
            x_min = std::min(x_min, p[0]);  x_max = std::max(x_max, p[0]);
            y_min = std::min(y_min, p[1]);  y_max = std::max(y_max, p[1]);
            z_min = std::min(z_min, p[2]);  z_max = std::max(z_max, p[2]);
        }

        // ── Step 4: fill Detection3D ──────────────────────────────────────────
        Detection3D det;
        det.cx = (x_min + x_max) * 0.5f;
        det.cy = (y_min + y_max) * 0.5f;
        det.cz = (z_min + z_max) * 0.5f;
        det.size_x = x_max - x_min;
        det.size_y = y_max - y_min;
        det.size_z = z_max - z_min;
        det.class_id = bbox.class_id;
        det.score    = bbox.score;
        det.n_points = static_cast<uint32_t>(indices.size());

        result.detections.push_back(det);
    }

    return result;
}

}  // namespace perception_pipeline_cpp
