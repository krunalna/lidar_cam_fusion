#pragma once

/**
 * calibration.hpp
 * ===============
 * Loads KITTI calibration YAML and exposes typed intrinsic / extrinsic data.
 *
 * No ROS dependency — pure C++ utility used by the fusion node (Phase 6).
 *
 * Fields populated from calibration.yaml:
 *   camera.fx, fy, cx, cy, width, height
 *   camera.P   → 3×4 projection matrix (row-major, 12 floats)
 *   lidar_to_camera.T → 4×4 extrinsic Velodyne→rectified-camera (row-major, 16 floats)
 */

#include <string>
#include <Eigen/Dense>

namespace perception_pipeline_cpp {

struct CalibrationData {
    // Camera intrinsics
    float fx{0.f}, fy{0.f}, cx{0.f}, cy{0.f};
    int   width{0}, height{0};

    // 3×4 projection matrix P2: projects rectified-camera points to pixels
    Eigen::Matrix<float, 3, 4> P = Eigen::Matrix<float, 3, 4>::Zero();

    // 4×4 extrinsic T: Velodyne frame → rectified camera frame
    Eigen::Matrix4f T = Eigen::Matrix4f::Zero();
};

class Calibration {
public:
    /**
     * Load calibration from a YAML file.
     * Throws std::runtime_error if the file cannot be opened or required
     * fields are missing.
     */
    explicit Calibration(const std::string & yaml_path);

    const CalibrationData & data() const { return data_; }

private:
    CalibrationData data_;
};

}  // namespace perception_pipeline_cpp
