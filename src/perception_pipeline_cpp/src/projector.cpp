/**
 * projector.cpp
 * =============
 * Implements Velodyne → image-plane projection using:
 *
 *   p = P × (T × P_lidar)   followed by homogeneous divide
 *
 * where T is the 4×4 Velodyne→camera extrinsic and P is the 3×4 KITTI
 * projection matrix for the target camera. All matrix ops use Eigen.
 */

#include "perception_pipeline_cpp/projector.hpp"

namespace perception_pipeline_cpp {

// ── Constructor ───────────────────────────────────────────────────────────────

Projector::Projector(const CalibrationData & cal)
: T_(cal.T), width_(cal.width), height_(cal.height)
{
    P_ = cal.P;
}

// ── Single-point projection ───────────────────────────────────────────────────

Projector::PixelCoord Projector::project(float x, float y, float z) const
{
    // Step 1: transform Velodyne point to rectified camera frame
    const Eigen::Vector4f P_velo(x, y, z, 1.f);
    const Eigen::Vector4f P_cam = T_ * P_velo;

    // Step 2: depth check — point must be in front of the camera
    if (P_cam[2] <= 0.f) {
        return {};   // default: valid=false
    }

    // Step 3: project to image plane with KITTI P2
    const Eigen::Vector3f p = P_ * P_cam;

    // Step 4: homogeneous divide
    const float u = p[0] / p[2];
    const float v = p[1] / p[2];

    // Step 5: image bounds check
    if (u < 0.f || u >= static_cast<float>(width_) ||
        v < 0.f || v >= static_cast<float>(height_)) {
        return {u, v, false};
    }

    return {u, v, true};
}

// ── Cloud projection ──────────────────────────────────────────────────────────

std::vector<Projector::PixelCoord> Projector::project_cloud(
    const float * points_xyzi, uint32_t n_points) const
{
    std::vector<PixelCoord> out;
    out.reserve(n_points);
    for (uint32_t i = 0; i < n_points; ++i) {
        const float * p = points_xyzi + i * 4;
        out.push_back(project(p[0], p[1], p[2]));
    }
    return out;
}

// ── Frustum association ───────────────────────────────────────────────────────

std::vector<uint32_t> Projector::points_in_bbox(
    const float * points_xyzi, uint32_t n_points,
    float x1, float y1, float x2, float y2) const
{
    std::vector<uint32_t> indices;
    for (uint32_t i = 0; i < n_points; ++i) {
        const float * p = points_xyzi + i * 4;
        const auto px = project(p[0], p[1], p[2]);
        if (px.valid && px.u >= x1 && px.u <= x2 && px.v >= y1 && px.v <= y2) {
            indices.push_back(i);
        }
    }
    return indices;
}

}  // namespace perception_pipeline_cpp
