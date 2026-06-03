/**
 * test_fusion_engine.cpp
 * ======================
 * GTest unit tests for FusionEngine pure logic.
 *
 * Uses the hardcoded KITTI calibration (same values as test_projector.cpp)
 * to build a real Projector so that BBox2D pixels are computed consistently
 * with the projection math — no mock needed.
 *
 * Synthetic cloud:
 *   10 points at x=5, y in [-0.5..0.5], z=0  (Velodyne frame, 5 m straight ahead)
 *   These project to pixels near (612, 178) — well inside a 500–730 × 100–260 bbox.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "perception_pipeline_cpp/calibration.hpp"
#include "perception_pipeline_cpp/projector.hpp"
#include "perception_pipeline_cpp/fusion_engine.hpp"

// ── Helpers ───────────────────────────────────────────────────────────────────

static perception_pipeline_cpp::CalibrationData make_kitti_cal()
{
    perception_pipeline_cpp::CalibrationData cal;
    cal.fx = 721.5377f; cal.fy = 721.5377f;
    cal.cx = 609.5593f; cal.cy = 172.8540f;
    cal.width = 1242;   cal.height = 375;
    cal.P << 721.5377f, 0.0f,      609.5593f, 44.85728f,
             0.0f,      721.5377f, 172.8540f, 0.2163791f,
             0.0f,      0.0f,      1.0f,      0.002745884f;

    cal.T << 7.533745e-03f, -9.999714e-01f, -6.166020e-04f, -4.069766e-03f,
             1.480249e-02f,  7.280733e-04f, -9.998902e-01f, -7.631618e-02f,
             9.998621e-01f,  7.523790e-03f,  1.480755e-02f, -2.717806e-01f,
             0.0f,           0.0f,           0.0f,            1.0f;
    return cal;
}

// Build a flat XYZI cloud of n points at x=dist, y spread, z=0
static std::vector<float> make_cloud(float dist, int n)
{
    std::vector<float> pts(n * 4);
    for (int i = 0; i < n; ++i) {
        pts[i*4 + 0] = dist;
        pts[i*4 + 1] = -0.5f + (1.0f / (n - 1)) * i;  // y: -0.5 .. +0.5
        pts[i*4 + 2] = 0.f;
        pts[i*4 + 3] = 1.f;  // intensity
    }
    return pts;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST(FusionEngineTest, HappyPath_10Points)
{
    const auto cal  = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);
    perception_pipeline_cpp::FusionEngine engine;

    const auto pts = make_cloud(5.0f, 10);

    // BBox that covers the projected cluster (verified against Projector reference)
    // All 10 points project near u≈612, v≈178 — use a generous bbox
    std::vector<perception_pipeline_cpp::BBox2D> dets2d = {{
        500.f, 100.f, 730.f, 260.f, "car", 0.9f
    }};

    const auto result = engine.fuse(dets2d, pts.data(), 10, proj);

    ASSERT_EQ(result.detections.size(), 1u);
    const auto & d = result.detections[0];

    EXPECT_EQ(d.class_id, "car");
    EXPECT_NEAR(d.score, 0.9f, 1e-4f);
    EXPECT_EQ(d.n_points, 10u);

    // Centre x should be near 5.0 m (the cloud's x-distance)
    EXPECT_NEAR(d.cx, 5.0f, 0.1f);

    // size_y spans the y-spread of the cloud — must be positive
    EXPECT_GT(d.size_y, 0.f);
    // size_x and size_z may be 0: all points share the same x and z in this cloud
    EXPECT_GE(d.size_x, 0.f);
    EXPECT_GE(d.size_z, 0.f);
}

TEST(FusionEngineTest, BelowMinClusterPoints_Rejected)
{
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);

    // Require at least 10 points but only supply 5
    perception_pipeline_cpp::FusionConfig cfg;
    cfg.min_cluster_points = 10;
    perception_pipeline_cpp::FusionEngine engine(cfg);

    const auto pts = make_cloud(5.0f, 5);
    std::vector<perception_pipeline_cpp::BBox2D> dets2d = {{
        500.f, 100.f, 730.f, 260.f, "car", 0.9f
    }};

    const auto result = engine.fuse(dets2d, pts.data(), 5, proj);
    EXPECT_TRUE(result.detections.empty());
}

TEST(FusionEngineTest, EmptyCloud_ReturnsEmpty)
{
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);
    perception_pipeline_cpp::FusionEngine engine;

    std::vector<perception_pipeline_cpp::BBox2D> dets2d = {{
        500.f, 100.f, 730.f, 260.f, "car", 0.9f
    }};

    const auto result = engine.fuse(dets2d, nullptr, 0, proj);
    EXPECT_TRUE(result.detections.empty());
}

TEST(FusionEngineTest, EmptyDetections_ReturnsEmpty)
{
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);
    perception_pipeline_cpp::FusionEngine engine;

    const auto pts = make_cloud(5.0f, 10);
    const auto result = engine.fuse({}, pts.data(), 10, proj);
    EXPECT_TRUE(result.detections.empty());
}

TEST(FusionEngineTest, PointsBehindCamera_Rejected)
{
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);
    perception_pipeline_cpp::FusionEngine engine;

    // Points at x=-50 are behind the camera after extrinsic transform
    const auto pts = make_cloud(-50.0f, 10);
    std::vector<perception_pipeline_cpp::BBox2D> dets2d = {{
        0.f, 0.f, 1242.f, 375.f, "car", 0.9f  // full-image bbox
    }};

    const auto result = engine.fuse(dets2d, pts.data(), 10, proj);
    EXPECT_TRUE(result.detections.empty());
}

TEST(FusionEngineTest, BboxOutsideProjection_Rejected)
{
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);
    perception_pipeline_cpp::FusionEngine engine;

    // Points project near (612, 178) — bbox deliberately placed far away
    const auto pts = make_cloud(5.0f, 10);
    std::vector<perception_pipeline_cpp::BBox2D> dets2d = {{
        0.f, 0.f, 100.f, 50.f, "car", 0.9f   // top-left corner, far from cluster
    }};

    const auto result = engine.fuse(dets2d, pts.data(), 10, proj);
    EXPECT_TRUE(result.detections.empty());
}

TEST(FusionEngineTest, MultipleBboxes_IndependentClusters)
{
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);
    perception_pipeline_cpp::FusionEngine engine;

    // Cloud has 10 points all projecting near (612, 178)
    const auto pts = make_cloud(5.0f, 10);

    // Two bboxes: one covering the cluster, one not
    std::vector<perception_pipeline_cpp::BBox2D> dets2d = {
        { 500.f, 100.f, 730.f, 260.f, "car",    0.9f },
        {   0.f,   0.f, 100.f,  50.f, "person", 0.8f },
    };

    const auto result = engine.fuse(dets2d, pts.data(), 10, proj);
    ASSERT_EQ(result.detections.size(), 1u);
    EXPECT_EQ(result.detections[0].class_id, "car");
}

TEST(FusionEngineTest, BoxCentreAndSizeConsistent)
{
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);
    perception_pipeline_cpp::FusionEngine engine;

    const auto pts = make_cloud(5.0f, 10);
    std::vector<perception_pipeline_cpp::BBox2D> dets2d = {{
        500.f, 100.f, 730.f, 260.f, "car", 0.9f
    }};

    const auto result = engine.fuse(dets2d, pts.data(), 10, proj);
    ASSERT_EQ(result.detections.size(), 1u);

    const auto & d = result.detections[0];

    // size_y should span the full y-spread of the cloud: ~1.0 m
    EXPECT_NEAR(d.size_y, 1.0f, 0.05f);

    // centre_y should be near 0 (symmetric spread)
    // In Velodyne frame: y of first point ≈ -0.5, last ≈ +0.5 → centre ≈ 0
    // Note: cy in camera frame depends on transform, but Velodyne-frame y is direct
    EXPECT_NEAR(d.cy, 0.0f, 0.1f);
}

TEST(FusionEngineTest, NearestDepthSliceBeatsBackground)
{
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);
    perception_pipeline_cpp::FusionEngine engine;

    std::vector<float> pts;
    pts.reserve(20 * 4);

    // Foreground car-like cluster around 5 m.
    for (int i = 0; i < 10; ++i) {
        pts.insert(pts.end(), {
            5.0f + 0.1f * static_cast<float>(i % 3),
            -0.8f + 0.18f * static_cast<float>(i),
            -1.0f,
            1.0f,
        });
    }

    // Background points on the same viewing ray family, much farther away.
    for (int i = 0; i < 10; ++i) {
        pts.insert(pts.end(), {
            20.0f + 0.2f * static_cast<float>(i % 3),
            -0.8f + 0.18f * static_cast<float>(i),
            -1.0f,
            1.0f,
        });
    }

    std::vector<perception_pipeline_cpp::BBox2D> dets2d = {{
        0.f, 0.f, 1242.f, 375.f, "car", 0.9f
    }};

    const auto result = engine.fuse(dets2d, pts.data(), 20, proj);
    ASSERT_EQ(result.detections.size(), 1u);

    const auto & d = result.detections[0];
    EXPECT_NEAR(d.cx, 5.1f, 0.5f);
    EXPECT_LT(d.size_x, 2.0f);
    EXPECT_EQ(d.n_points, 10u);
}
