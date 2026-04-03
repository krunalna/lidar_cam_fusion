#include "perception_pipeline_cpp/kitti_reader.hpp"

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace perception_pipeline_cpp {

// ── Constructor ───────────────────────────────────────────────────────────────

KittiReader::KittiReader(const std::string & sequence_path,
                         const std::string & camera_id)
: seq_path_(sequence_path), camera_id_(camera_id)
{
  validate_dirs();
  load_file_lists();
  load_timestamps();
}

// ── Public API ────────────────────────────────────────────────────────────────

cv::Mat KittiReader::read_image(size_t idx) const
{
  const auto & path = image_files_.at(idx);
  cv::Mat bgr = cv::imread(path.string());
  if (bgr.empty()) {
    throw std::runtime_error("Could not read image: " + path.string());
  }
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return rgb;
}

std::vector<float> KittiReader::read_lidar(size_t idx) const
{
  const auto & path = lidar_files_.at(idx);
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) {
    throw std::runtime_error("Could not open LiDAR file: " + path.string());
  }
  auto size_bytes = static_cast<size_t>(file.tellg());
  file.seekg(0);

  std::vector<float> points(size_bytes / sizeof(float));
  file.read(reinterpret_cast<char *>(points.data()), size_bytes);
  return points;
}

int64_t KittiReader::timestamp_ns(size_t idx) const
{
  if (timestamps_ns_.empty() || idx >= timestamps_ns_.size()) {
    return -1;
  }
  return timestamps_ns_[idx];
}

// ── Private helpers ───────────────────────────────────────────────────────────

void KittiReader::validate_dirs() const
{
  const std::vector<std::filesystem::path> required = {
    seq_path_ / camera_id_ / "data",
    seq_path_ / "velodyne_points" / "data",
  };
  for (const auto & dir : required) {
    if (!std::filesystem::is_directory(dir)) {
      throw std::runtime_error(
        "Expected directory not found: " + dir.string() +
        "\nEnsure sequence_path points to a KITTI raw sync sequence, e.g.:"
        "\n  2011_09_26/2011_09_26_drive_0001_sync/");
    }
  }
}

void KittiReader::load_file_lists()
{
  const auto img_dir = seq_path_ / camera_id_ / "data";
  const auto lid_dir = seq_path_ / "velodyne_points" / "data";

  for (const auto & entry : std::filesystem::directory_iterator(img_dir)) {
    if (entry.path().extension() == ".png") {
      image_files_.push_back(entry.path());
    }
  }
  for (const auto & entry : std::filesystem::directory_iterator(lid_dir)) {
    if (entry.path().extension() == ".bin") {
      lidar_files_.push_back(entry.path());
    }
  }

  std::sort(image_files_.begin(), image_files_.end());
  std::sort(lidar_files_.begin(), lidar_files_.end());

  if (image_files_.empty() || lidar_files_.empty()) {
    throw std::runtime_error(
      "No data found: images=" + std::to_string(image_files_.size()) +
      ", lidar=" + std::to_string(lidar_files_.size()));
  }

  n_frames_ = std::min(image_files_.size(), lidar_files_.size());
}

void KittiReader::load_timestamps()
{
  const auto ts_path = seq_path_ / camera_id_ / "timestamps.txt";
  if (!std::filesystem::exists(ts_path)) {
    return;  // caller will fall back to wall-clock stamps
  }

  std::ifstream file(ts_path);
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    int64_t ns = parse_kitti_timestamp(line);
    if (ns >= 0) {
      timestamps_ns_.push_back(ns);
    }
  }
}

// static
int64_t KittiReader::parse_kitti_timestamp(const std::string & line)
{
  // Format: 2011-09-26 13:02:25.820513000
  int year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;
  if (std::sscanf(line.c_str(), "%d-%d-%d %d:%d:%d",
                  &year, &month, &day, &hour, &min, &sec) != 6)
  {
    return -1;
  }

  struct tm tm = {};
  tm.tm_year  = year - 1900;
  tm.tm_mon   = month - 1;
  tm.tm_mday  = day;
  tm.tm_hour  = hour;
  tm.tm_min   = min;
  tm.tm_sec   = sec;
  tm.tm_isdst = 0;

  // timegm interprets tm as UTC (POSIX: Linux + macOS)
  time_t epoch = timegm(&tm);

  // Parse fractional seconds → nanoseconds
  int64_t frac_ns = 0;
  auto dot = line.find('.');
  if (dot != std::string::npos) {
    std::string frac = line.substr(dot + 1);
    while (frac.size() < 9) frac += "0";  // pad to nanosecond precision
    frac = frac.substr(0, 9);
    frac_ns = std::stoll(frac);
  }

  return static_cast<int64_t>(epoch) * 1'000'000'000LL + frac_ns;
}

}  // namespace perception_pipeline_cpp
