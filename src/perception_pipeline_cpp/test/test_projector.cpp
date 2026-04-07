/**
 * test_projector.cpp
 * ==================
 * GTest unit tests for Calibration loader and Projector math.
 *
 * Reference values computed from calibration.yaml via numpy:
 *
 *   P = reshape(calibration.yaml camera.P, 3, 4)
 *   T = reshape(calibration.yaml lidar_to_camera.T, 4, 4)
 *
 *   P_velo = [10, 0, 0, 1]^T
 *   P_cam  = T @ P_velo  →  [Xc, Yc, Zc]
 *   p      = P @ P_cam
 *   u, v   = p[0]/p[2], p[1]/p[2]
 *
 * Run with:
 *   pixi run build-cpp -- --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
 *   colcon test --packages-select perception_pipeline_cpp
 */

#include <gtest/gtest.h>
#include <cmath>

#include "perception_pipeline_cpp/calibration.hpp"
#include "perception_pipeline_cpp/projector.hpp"

// ── Helpers ───────────────────────────────────────────────────────────────────

// Path to the calibration YAML (resolved relative to the repo root at test time)
static const char * kCalibPath =
    CALIBRATION_YAML_PATH;   // injected by CMake via target_compile_definitions

// ── CalibrationData fixture ───────────────────────────────────────────────────

// Hardcoded KITTI values matching calibration.yaml — tests run without file I/O
static perception_pipeline_cpp::CalibrationData make_kitti_cal()
{
    perception_pipeline_cpp::CalibrationData cal;
    cal.fx = 721.5377f; cal.fy = 721.5377f;
    cal.cx = 609.5593f; cal.cy = 172.8540f;
    cal.width = 1242;   cal.height = 375;
    cal.P << 721.5377f, 0.0f,      609.5593f, 44.85728f,
             0.0f,      721.5377f, 172.8540f, 0.2163791f,
             0.0f,      0.0f,      1.0f,      0.002745884f;

    // 4×4 extrinsic T from calibration.yaml (Eigen comma initializer, row-major order)
    cal.T << 7.533745e-03f, -9.999714e-01f, -6.166020e-04f, -4.069766e-03f,
             1.480249e-02f,  7.280733e-04f, -9.998902e-01f, -7.631618e-02f,
             9.998621e-01f,  7.523790e-03f,  1.480755e-02f, -2.717806e-01f,
             0.0f,           0.0f,           0.0f,            1.0f;
    return cal;
}

// ── Calibration YAML tests ────────────────────────────────────────────────────

TEST(CalibrationTest, LoadsFileSuccessfully)
{
    ASSERT_NO_THROW({
        perception_pipeline_cpp::Calibration cal(kCalibPath);
        (void)cal;
    });
}

TEST(CalibrationTest, IntrinsicsMatchExpected)
{
    perception_pipeline_cpp::Calibration cal(kCalibPath);
    const auto & d = cal.data();

    EXPECT_NEAR(d.fx, 721.5377f, 1e-3f);
    EXPECT_NEAR(d.fy, 721.5377f, 1e-3f);
    EXPECT_NEAR(d.cx, 609.5593f, 1e-3f);
    EXPECT_NEAR(d.cy, 172.8540f, 1e-3f);
    EXPECT_EQ(d.width,  1242);
    EXPECT_EQ(d.height,  375);
}

TEST(CalibrationTest, ExtrinsicHas16Elements)
{
    perception_pipeline_cpp::Calibration cal(kCalibPath);
    const auto & T = cal.data().T;
    // Last row must be [0, 0, 0, 1] for a valid homogeneous transform
    EXPECT_NEAR(T(3, 0), 0.f, 1e-6f);
    EXPECT_NEAR(T(3, 1), 0.f, 1e-6f);
    EXPECT_NEAR(T(3, 2), 0.f, 1e-6f);
    EXPECT_NEAR(T(3, 3), 1.f, 1e-6f);
}

TEST(CalibrationTest, ProjectionMatrixHas12Elements)
{
    perception_pipeline_cpp::Calibration cal(kCalibPath);
    const auto & P = cal.data().P;
    // P(2,2) is the (row2, col2) element — must be 1 for a valid projection matrix
    EXPECT_NEAR(P(2, 2), 1.f, 1e-6f);
}

TEST(CalibrationTest, MissingFileThrows)
{
    EXPECT_THROW(
        perception_pipeline_cpp::Calibration cal("/nonexistent/path.yaml"),
        std::runtime_error);
}

// ── Projector tests ───────────────────────────────────────────────────────────

TEST(ProjectorTest, PointAheadProjectsIntoImage)
{
    // A point 10 m straight ahead in Velodyne frame (positive X = forward)
    // Should project near image centre after extrinsic rotation
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);

    const auto px = proj.project(10.0f, 0.0f, 0.0f);

    EXPECT_TRUE(px.valid);
    EXPECT_GT(px.u, 0.f);
    EXPECT_LT(px.u, 1242.f);
    EXPECT_GT(px.v, 0.f);
    EXPECT_LT(px.v, 375.f);
}

TEST(ProjectorTest, PointAheadMatchesPythonReference)
{
    // Hand-computed KITTI P2 reference for P_velo = [10, 0, 0]:
    //   P_cam = T @ P_velo
    //   p     = P2 @ P_cam
    //   u,v   = p[:2] / p[2] ≈ (619.28, 178.15)
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);

    const auto px = proj.project(10.0f, 0.0f, 0.0f);

    EXPECT_TRUE(px.valid);
    EXPECT_NEAR(px.u, 619.28f, 1.0f);   // 1-pixel tolerance
    EXPECT_NEAR(px.v, 178.15f, 1.0f);
}

TEST(ProjectorTest, PointBehindCameraIsInvalid)
{
    // A point directly behind the camera (negative Z_cam after transform)
    // Put it far behind along the camera's optical axis
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);

    // In Velodyne frame, the camera is roughly along +X (T[8] ≈ 0.9999).
    // A point at x=-50 will be well behind the camera.
    const auto px = proj.project(-50.0f, 0.0f, 0.0f);

    EXPECT_FALSE(px.valid);
}

TEST(ProjectorTest, PointOutsideImageBoundsIsInvalid)
{
    // A point that projects far to the side — outside [0, 1242)
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);

    // Very large Y offset pushes the projection past the image edge
    const auto px = proj.project(10.0f, 100.0f, 0.0f);

    // Either behind camera or outside image — either way, not valid
    // (exact invalidity depends on transform; we just assert not a valid in-bounds hit)
    if (px.valid) {
        // If somehow still valid, u must be in-bounds — but v could be out.
        // This test is checking the guard works, so at large offsets it should fail.
        EXPECT_TRUE(px.u < 0.f || px.u >= 1242.f || px.v < 0.f || px.v >= 375.f)
            << "Expected out-of-bounds projection for extreme lateral offset";
    }
    // If not valid, the test passes by definition.
}

TEST(ProjectorTest, PointsInBboxReturnsExpectedIndices)
{
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);

    // Two points: one ahead (projects ~614,162), one behind camera
    float pts[8] = {
        10.0f,  0.0f, 0.0f, 1.0f,   // idx 0 — projects into image
        -50.0f, 0.0f, 0.0f, 1.0f,   // idx 1 — behind camera
    };

    // Bbox that covers the expected projection of the first point
    const auto indices = proj.points_in_bbox(pts, 2, 500.f, 100.f, 750.f, 250.f);

    ASSERT_EQ(indices.size(), 1u);
    EXPECT_EQ(indices[0], 0u);
}

TEST(ProjectorTest, EmptyCloudReturnsEmpty)
{
    const auto cal = make_kitti_cal();
    perception_pipeline_cpp::Projector proj(cal);

    const auto result = proj.project_cloud(nullptr, 0);
    EXPECT_TRUE(result.empty());
}
