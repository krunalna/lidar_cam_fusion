#include <gtest/gtest.h>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "perception_pipeline_cpp/kitti_reader.hpp"

namespace fs = std::filesystem;
using perception_pipeline_cpp::KittiReader;

// ── Helpers ───────────────────────────────────────────────────────────────────

static fs::path make_kitti_sequence(const fs::path & root,
                                    const std::string & camera_id,
                                    int n_frames)
{
  const auto img_dir = root / camera_id / "data";
  const auto lid_dir = root / "velodyne_points" / "data";
  fs::create_directories(img_dir);
  fs::create_directories(lid_dir);

  for (int i = 0; i < n_frames; ++i) {
    char name[32];
    std::snprintf(name, sizeof(name), "%010d", i);

    // Minimal 1×1 PNG (valid PNG header for a 1×1 RGB image)
    // Using OpenCV to create a real PNG:
    cv::Mat img(1, 1, CV_8UC3, cv::Scalar(100, 150, 200));
    cv::imwrite((img_dir / (std::string(name) + ".png")).string(), img);

    // Minimal .bin: 2 points × 4 floats each
    const float pts[] = {
      1.0f, 2.0f, 3.0f, 0.5f,
      4.0f, 5.0f, 6.0f, 0.8f,
    };
    std::ofstream bin(lid_dir / (std::string(name) + ".bin"), std::ios::binary);
    bin.write(reinterpret_cast<const char *>(pts), sizeof(pts));
  }
  return root;
}

static fs::path make_temp_dir()
{
  auto tmp = fs::temp_directory_path() / ("kitti_test_" + std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(tmp);
  return tmp;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST(KittiReaderTest, FrameCount)
{
  auto root = make_temp_dir();
  make_kitti_sequence(root, "image_02", 3);
  KittiReader reader(root.string(), "image_02");
  EXPECT_EQ(reader.frame_count(), 3u);
  fs::remove_all(root);
}

TEST(KittiReaderTest, ReadImageRGB)
{
  auto root = make_temp_dir();
  make_kitti_sequence(root, "image_02", 1);
  KittiReader reader(root.string(), "image_02");
  cv::Mat img = reader.read_image(0);
  EXPECT_FALSE(img.empty());
  EXPECT_EQ(img.type(), CV_8UC3);
  EXPECT_GT(img.rows, 0);
  EXPECT_GT(img.cols, 0);
  fs::remove_all(root);
}

TEST(KittiReaderTest, ReadLidar)
{
  auto root = make_temp_dir();
  make_kitti_sequence(root, "image_02", 1);
  KittiReader reader(root.string(), "image_02");
  auto pts = reader.read_lidar(0);
  EXPECT_EQ(pts.size(), 8u);  // 2 points × 4 floats
  EXPECT_FLOAT_EQ(pts[0], 1.0f);  // x of first point
  EXPECT_FLOAT_EQ(pts[1], 2.0f);  // y
  EXPECT_FLOAT_EQ(pts[2], 3.0f);  // z
  EXPECT_FLOAT_EQ(pts[3], 0.5f);  // intensity
  fs::remove_all(root);
}

TEST(KittiReaderTest, MissingDirThrows)
{
  EXPECT_THROW(
    KittiReader("/nonexistent/path/that/does/not/exist"),
    std::runtime_error);
}

TEST(KittiReaderTest, NoTimestampsReturnsMinusOne)
{
  auto root = make_temp_dir();
  make_kitti_sequence(root, "image_02", 1);
  KittiReader reader(root.string(), "image_02");
  EXPECT_EQ(reader.timestamp_ns(0), -1);
  fs::remove_all(root);
}

TEST(KittiReaderTest, TimestampParsing)
{
  auto root = make_temp_dir();
  make_kitti_sequence(root, "image_02", 2);

  // Write a timestamps.txt with known values
  std::ofstream ts(root / "image_02" / "timestamps.txt");
  ts << "2011-09-26 13:02:25.820513000\n";
  ts << "2011-09-26 13:02:25.920513000\n";
  ts.close();

  KittiReader reader(root.string(), "image_02");
  int64_t t0 = reader.timestamp_ns(0);
  int64_t t1 = reader.timestamp_ns(1);
  EXPECT_GT(t0, 0);
  EXPECT_GT(t1, t0);
  // Difference should be 100 ms = 100,000,000 ns
  EXPECT_EQ(t1 - t0, 100'000'000LL);
  fs::remove_all(root);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
