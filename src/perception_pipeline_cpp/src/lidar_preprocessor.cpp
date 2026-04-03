#include "perception_pipeline_cpp/lidar_preprocessor.hpp"

#include <cmath>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace perception_pipeline_cpp {

using Cloud    = pcl::PointCloud<pcl::PointXYZI>;
using CloudPtr = Cloud::Ptr;

LidarPreprocessor::LidarPreprocessor(const LidarPreprocessorConfig & cfg)
: cfg_(cfg) {}

LidarPreprocessorResult LidarPreprocessor::process(
  const float * points_xyzi, uint32_t n_points) const
{
  LidarPreprocessorResult result;

  // ── Build PCL cloud from raw float buffer ─────────────────────────────────
  CloudPtr cloud(new Cloud());
  cloud->reserve(n_points);
  for (uint32_t i = 0; i < n_points; ++i) {
    const float * p = points_xyzi + i * 4;
    cloud->emplace_back(p[0], p[1], p[2], p[3]);
  }
  result.stats.n_input = n_points;

  // ── Stage 1: ROI crop ─────────────────────────────────────────────────────
  pcl::CropBox<pcl::PointXYZI> crop;
  crop.setInputCloud(cloud);
  crop.setMin(Eigen::Vector4f(cfg_.roi_x_min, cfg_.roi_y_min, cfg_.roi_z_min, 1.0f));
  crop.setMax(Eigen::Vector4f(cfg_.roi_x_max, cfg_.roi_y_max, cfg_.roi_z_max, 1.0f));
  CloudPtr roi_cloud(new Cloud());
  crop.filter(*roi_cloud);
  result.stats.n_roi = static_cast<uint32_t>(roi_cloud->size());

  if (roi_cloud->size() < 10) {
    return result;
  }

  // ── Stage 2: Voxel downsampling ───────────────────────────────────────────
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(roi_cloud);
  voxel.setLeafSize(cfg_.voxel_size, cfg_.voxel_size, cfg_.voxel_size);
  CloudPtr voxel_cloud(new Cloud());
  voxel.filter(*voxel_cloud);
  result.stats.n_voxel = static_cast<uint32_t>(voxel_cloud->size());

  if (voxel_cloud->size() < 10) {
    return result;
  }

  // ── Stage 3: Distance filter ──────────────────────────────────────────────
  CloudPtr dist_cloud(new Cloud());
  dist_cloud->reserve(voxel_cloud->size());
  for (const auto & pt : *voxel_cloud) {
    if (std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) <= cfg_.max_depth) {
      dist_cloud->push_back(pt);
    }
  }

  if (dist_cloud->size() < 10) {
    return result;
  }

  // ── Stage 4: RANSAC ground removal ────────────────────────────────────────
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(cfg_.ransac_dist);
  seg.setMaxIterations(cfg_.ransac_iter);
  seg.setInputCloud(dist_cloud);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  seg.segment(*inliers, *coefficients);

  // Extract ground (inliers) and non-ground (outliers)
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(dist_cloud);
  extract.setIndices(inliers);

  CloudPtr ground_cloud(new Cloud());
  extract.setNegative(false);
  extract.filter(*ground_cloud);

  CloudPtr filtered_cloud(new Cloud());
  extract.setNegative(true);
  extract.filter(*filtered_cloud);

  result.stats.n_ground = static_cast<uint32_t>(ground_cloud->size());
  result.stats.n_output = static_cast<uint32_t>(filtered_cloud->size());

  // ── Pack results into flat float buffers ──────────────────────────────────
  // Intensity: voxel grid averages it; zero it out to match Python behaviour
  result.filtered.reserve(filtered_cloud->size() * 4);
  for (const auto & pt : *filtered_cloud) {
    result.filtered.push_back(pt.x);
    result.filtered.push_back(pt.y);
    result.filtered.push_back(pt.z);
    result.filtered.push_back(0.0f);  // intensity zeroed (voxel grid loses it)
  }

  result.ground.reserve(ground_cloud->size() * 4);
  for (const auto & pt : *ground_cloud) {
    result.ground.push_back(pt.x);
    result.ground.push_back(pt.y);
    result.ground.push_back(pt.z);
    result.ground.push_back(0.0f);
  }

  return result;
}

}  // namespace perception_pipeline_cpp
