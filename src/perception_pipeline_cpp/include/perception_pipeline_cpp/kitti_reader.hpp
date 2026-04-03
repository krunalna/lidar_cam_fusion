#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception_pipeline_cpp {

/**
 * KittiReader
 * ===========
 * Pure I/O class — no ROS dependency. Reads KITTI raw sync sequences from disk.
 *
 * Directory layout expected:
 *   <sequence_path>/<camera_id>/data/          *.png frames
 *   <sequence_path>/velodyne_points/data/      *.bin scans
 *   <sequence_path>/<camera_id>/timestamps.txt (optional)
 */
class KittiReader {
public:
  /**
   * @param sequence_path  Absolute path to a KITTI raw sync sequence directory.
   * @param camera_id      Camera subfolder name (default: image_02).
   * @throws std::runtime_error if required directories are missing or empty.
   */
  KittiReader(const std::string & sequence_path,
              const std::string & camera_id = "image_02");

  size_t frame_count() const { return n_frames_; }

  /**
   * Read a camera frame as an RGB image.
   * @returns cv::Mat of type CV_8UC3 in RGB channel order.
   * @throws std::runtime_error if the image cannot be read.
   */
  cv::Mat read_image(size_t idx) const;

  /**
   * Read a Velodyne scan as a flat float32 array.
   * @returns Vector of N*4 floats in [x, y, z, intensity] order.
   * @throws std::runtime_error if the file cannot be read.
   */
  std::vector<float> read_lidar(size_t idx) const;

  /**
   * Return the KITTI timestamp for frame idx in nanoseconds since Unix epoch.
   * Returns -1 if no timestamps.txt was found.
   */
  int64_t timestamp_ns(size_t idx) const;

private:
  void validate_dirs() const;
  void load_file_lists();
  void load_timestamps();

  static int64_t parse_kitti_timestamp(const std::string & line);

  std::filesystem::path seq_path_;
  std::string camera_id_;
  std::vector<std::filesystem::path> image_files_;
  std::vector<std::filesystem::path> lidar_files_;
  std::vector<int64_t> timestamps_ns_;
  size_t n_frames_{0};
};

}  // namespace perception_pipeline_cpp
