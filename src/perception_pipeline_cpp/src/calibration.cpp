/**
 * calibration.cpp
 * ===============
 * Parses KITTI calibration YAML using OpenCV FileStorage (already a project dep).
 *
 * Expected YAML structure:
 *   camera:
 *     fx, fy, cx, cy  (floats)
 *     width, height   (ints)
 *     P: [12 floats]  — 3×4 projection matrix, row-major
 *   lidar_to_camera:
 *     T: [16 floats]  — 4×4 extrinsic, row-major
 */

#include "perception_pipeline_cpp/calibration.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace perception_pipeline_cpp {

Calibration::Calibration(const std::string & yaml_path)
{
    // OpenCV FileStorage requires the YAML file to begin with "%YAML:1.0".
    // KITTI calibration files are plain YAML without this header.
    // Read the file into a string, prepend the header, then parse from memory.
    std::ifstream ifs(yaml_path);
    if (!ifs.is_open()) {
        throw std::runtime_error(
            "Calibration: cannot open YAML file: " + yaml_path);
    }
    std::ostringstream buf;
    buf << "%YAML:1.0\n" << ifs.rdbuf();
    const std::string content = buf.str();

    cv::FileStorage fs(content, cv::FileStorage::READ | cv::FileStorage::MEMORY);
    if (!fs.isOpened()) {
        throw std::runtime_error(
            "Calibration: failed to parse YAML file: " + yaml_path);
    }

    // ── Camera intrinsics ─────────────────────────────────────────────────────
    cv::FileNode cam = fs["camera"];
    if (cam.empty()) {
        throw std::runtime_error(
            "Calibration: missing 'camera' section in " + yaml_path);
    }

    cam["fx"] >> data_.fx;
    cam["fy"] >> data_.fy;
    cam["cx"] >> data_.cx;
    cam["cy"] >> data_.cy;
    cam["width"]  >> data_.width;
    cam["height"] >> data_.height;

    // ── 3×4 projection matrix P ───────────────────────────────────────────────
    std::vector<float> P_vec;
    cam["P"] >> P_vec;
    if (P_vec.size() != 12) {
        throw std::runtime_error(
            "Calibration: camera.P must have 12 elements, got "
            + std::to_string(P_vec.size()));
    }
    // Map the flat row-major vector into the Eigen matrix
    data_.P = Eigen::Map<Eigen::Matrix<float, 3, 4, Eigen::RowMajor>>(P_vec.data());

    // ── 4×4 extrinsic T (Velodyne → rectified camera) ────────────────────────
    cv::FileNode lidar_cam = fs["lidar_to_camera"];
    if (lidar_cam.empty()) {
        throw std::runtime_error(
            "Calibration: missing 'lidar_to_camera' section in " + yaml_path);
    }

    std::vector<float> T_vec;
    lidar_cam["T"] >> T_vec;
    if (T_vec.size() != 16) {
        throw std::runtime_error(
            "Calibration: lidar_to_camera.T must have 16 elements, got "
            + std::to_string(T_vec.size()));
    }
    // Map the flat row-major vector into the Eigen matrix
    data_.T = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(T_vec.data());

    fs.release();
}

}  // namespace perception_pipeline_cpp
