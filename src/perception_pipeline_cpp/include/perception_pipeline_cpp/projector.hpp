#pragma once

/**
 * projector.hpp
 * =============
 * Projects Velodyne LiDAR points into image-plane pixel coordinates.
 *
 * Core equation:  p_image = P × T × P_lidar
 *
 *   T  (4×4) — extrinsic: Velodyne frame → rectified camera frame
 *   P  (3×4) — KITTI P2 projection matrix for the image-producing camera
 *
 * No ROS dependency — pure math utility used by the fusion node (Phase 6).
 */

#include <cstdint>
#include <vector>

#include <Eigen/Dense>
#include "perception_pipeline_cpp/calibration.hpp"

namespace perception_pipeline_cpp {

class Projector {
public:
    explicit Projector(const CalibrationData & cal);

    // ── Single point ──────────────────────────────────────────────────────────

    struct PixelCoord {
        float u{0.f}, v{0.f};
        bool  valid{false};  // false if behind camera or outside image bounds
    };

    /**
     * Project a single point from the Velodyne frame to image pixel coordinates.
     *
     * @param x  Point X in Velodyne frame (metres)
     * @param y  Point Y in Velodyne frame (metres)
     * @param z  Point Z in Velodyne frame (metres)
     * @return   Pixel coord; valid=false if Z_cam <= 0 or outside image bounds
     */
    PixelCoord project(float x, float y, float z) const;

    // ── Cloud projection ──────────────────────────────────────────────────────

    /**
     * Project all points in a flat XYZI buffer.
     *
     * @param points_xyzi  Flat float32 buffer, N×4: [x,y,z,intensity, ...]
     * @param n_points     Number of points (not floats)
     * @return             N-length vector of PixelCoord, same order as input
     */
    std::vector<PixelCoord> project_cloud(
        const float * points_xyzi, uint32_t n_points) const;

    // ── Frustum association ───────────────────────────────────────────────────

    /**
     * Return indices of points whose projection falls inside a 2D bounding box.
     * Only points with valid projections are included.
     *
     * @param points_xyzi  Flat XYZI buffer (see project_cloud)
     * @param n_points     Number of points
     * @param x1,y1        Top-left corner of bbox (pixels)
     * @param x2,y2        Bottom-right corner of bbox (pixels)
     * @return             Indices into the original point array
     */
    std::vector<uint32_t> points_in_bbox(
        const float * points_xyzi, uint32_t n_points,
        float x1, float y1, float x2, float y2) const;

private:
    Eigen::Matrix<float, 3, 4> P_;  // 3×4 KITTI projection matrix
    Eigen::Matrix4f T_;             // 4×4 Velodyne→camera extrinsic
    int width_{0}, height_{0};
};

}  // namespace perception_pipeline_cpp
