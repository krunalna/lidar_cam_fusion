/**
 * projector.cpp
 * =============
 * Implements Velodyne → image-plane projection using:
 *
 *   p = K × (T × P_lidar)[0:3]   followed by homogeneous divide
 *
 * where T is the 4×4 Velodyne→camera extrinsic and K is built from
 * (fx, fy, cx, cy) stored in CalibrationData.
 *
 * All matrix operations use plain array arithmetic — no Eigen dependency.
 */

#include "perception_pipeline_cpp/projector.hpp"

namespace perception_pipeline_cpp {

// ── Constructor ───────────────────────────────────────────────────────────────

Projector::Projector(const CalibrationData & cal)
: T_(cal.T), width_(cal.width), height_(cal.height)
{
    // Build 3×3 K from scalar intrinsics (row-major)
    K_[0] = cal.fx; K_[1] = 0.f;    K_[2] = cal.cx;
    K_[3] = 0.f;    K_[4] = cal.fy; K_[5] = cal.cy;
    K_[6] = 0.f;    K_[7] = 0.f;    K_[8] = 1.f;
}

// ── Single-point projection ───────────────────────────────────────────────────

Projector::PixelCoord Projector::project(float x, float y, float z) const
{
    // Step 1: transform Velodyne point to rectified camera frame
    // P_cam = T * [x, y, z, 1]^T   (4×4 row-major × 4×1)
    const float Xc = T_[0]*x + T_[1]*y + T_[2]*z  + T_[3];
    const float Yc = T_[4]*x + T_[5]*y + T_[6]*z  + T_[7];
    const float Zc = T_[8]*x + T_[9]*y + T_[10]*z + T_[11];

    // Step 2: depth check — point must be in front of the camera
    if (Zc <= 0.f) {
        return {};   // default: valid=false
    }

    // Step 3: project to image plane with intrinsics
    // p = K * [Xc, Yc, Zc]^T   (3×3 row-major × 3×1)
    const float pu = K_[0]*Xc + K_[1]*Yc + K_[2]*Zc;   // su
    const float pv = K_[3]*Xc + K_[4]*Yc + K_[5]*Zc;   // sv
    const float pw = K_[6]*Xc + K_[7]*Yc + K_[8]*Zc;   // s  (= Zc since K[6]=K[7]=0, K[8]=1)

    // Step 4: homogeneous divide
    const float u = pu / pw;
    const float v = pv / pw;

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
