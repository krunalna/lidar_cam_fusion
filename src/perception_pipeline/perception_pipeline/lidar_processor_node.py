"""
LiDAR Processor Node
====================
Subscribes to raw Velodyne point clouds, runs an Open3D preprocessing pipeline,
and republishes a filtered cloud ready for the fusion node.

Preprocessing stages (all configurable via ROS parameters):
  1. ROI crop      — discard points outside a forward-facing box
  2. Voxel downsample — uniform density reduction
  3. Ground removal — RANSAC plane fitting to strip the road surface
  4. Distance filter — discard points beyond max_depth

Subscribed topics:
  /lidar/points          sensor_msgs/PointCloud2   raw Velodyne scan

Published topics:
  /lidar/filtered        sensor_msgs/PointCloud2   preprocessed cloud
  /lidar/ground_plane    sensor_msgs/PointCloud2   ground points (debug)

Parameters:
  roi_x_min   (float)  Forward ROI min  (default: 0.0 m — front half only)
  roi_x_max   (float)  Forward ROI max  (default: 50.0 m)
  roi_y_min   (float)  Lateral ROI min  (default: -10.0 m)
  roi_y_max   (float)  Lateral ROI max  (default:  10.0 m)
  roi_z_min   (float)  Vertical ROI min (default: -3.0 m)
  roi_z_max   (float)  Vertical ROI max (default:  2.0 m)
  voxel_size  (float)  Voxel grid leaf size in meters (default: 0.1 m)
  ransac_dist (float)  RANSAC inlier threshold for ground plane (default: 0.2 m)
  ransac_iter (int)    RANSAC max iterations (default: 100)
  max_depth   (float)  Max range to keep in meters (default: 50.0 m)
"""

from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


# ── PointCloud2 ↔ numpy helpers ──────────────────────────────────────────────

def _pc2_to_numpy(msg: PointCloud2) -> np.ndarray:
    """Convert sensor_msgs/PointCloud2 → (N, 4) float32 numpy array [x,y,z,intensity].

    Uses msg.fields to locate each channel by byte offset, so it works with
    any field layout — not just the KITTI 4-float32 packing.
    """
    if msg.point_step == 0:
        raise ValueError("PointCloud2 has point_step == 0")
    if msg.point_step % 4 != 0:
        raise ValueError(f"point_step {msg.point_step} is not a multiple of 4 bytes")

    field_offsets = {f.name: f.offset for f in msg.fields}
    required = {"x", "y", "z"}
    missing = required - set(field_offsets)
    if missing:
        raise ValueError(f"PointCloud2 is missing required fields: {missing}")

    n_points = msg.width * msg.height
    stride = msg.point_step // 4  # floats per point
    raw = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(n_points, stride)

    result = np.zeros((n_points, 4), dtype=np.float32)
    for col, name in enumerate(("x", "y", "z")):
        result[:, col] = raw[:, field_offsets[name] // 4]
    if "intensity" in field_offsets:
        result[:, 3] = raw[:, field_offsets["intensity"] // 4]
    return result


def _numpy_to_pc2(points: np.ndarray, header: Header) -> PointCloud2:
    """Convert (N, 3) or (N, 4) float32 numpy array → sensor_msgs/PointCloud2."""
    if points.ndim == 1:
        points = points.reshape(-1, 3)

    n_cols = points.shape[1]
    # Pad to 4 columns (add zero intensity if not present)
    if n_cols == 3:
        intensity = np.zeros((points.shape[0], 1), dtype=np.float32)
        points = np.hstack([points, intensity])

    points = points.astype(np.float32)
    itemsize = np.dtype(np.float32).itemsize

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = points.shape[0]
    msg.is_bigendian = False
    msg.is_dense = True
    msg.fields = [
        PointField(name="x",         offset=0 * itemsize, datatype=PointField.FLOAT32, count=1),
        PointField(name="y",         offset=1 * itemsize, datatype=PointField.FLOAT32, count=1),
        PointField(name="z",         offset=2 * itemsize, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=3 * itemsize, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 4 * itemsize
    msg.row_step = msg.point_step * msg.width
    msg.data = points.tobytes()
    return msg


# ── Voxel downsampling with intensity ─────────────────────────────────────────

def _voxel_downsample(
    xyz: np.ndarray,
    intensity: np.ndarray,
    voxel_size: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Voxel grid downsample that averages both xyz and intensity per voxel.

    Open3D's voxel_down_sample only stores xyz and silently drops intensity.
    This numpy implementation preserves it by averaging within each voxel cell.

    Args:
        xyz:        (N, 3) float32 point coordinates.
        intensity:  (N, 1) float32 per-point intensities.
        voxel_size: Leaf size in metres.

    Returns:
        Tuple of (xyz_down (M, 3), intensity_down (M, 1)) float32 arrays.
    """
    voxel_coords = np.floor(xyz / voxel_size).astype(np.int32)

    # Encode each 3-D voxel index into a single bytes key for np.unique
    keys = np.ascontiguousarray(voxel_coords).view(
        np.dtype((np.void, voxel_coords.dtype.itemsize * 3))
    ).ravel()

    _, inverse = np.unique(keys, return_inverse=True)
    n_voxels = int(_.shape[0])

    xyz_out = np.zeros((n_voxels, 3), dtype=np.float32)
    int_out = np.zeros((n_voxels, 1), dtype=np.float32)
    counts  = np.zeros(n_voxels,     dtype=np.float32)

    np.add.at(xyz_out,        inverse, xyz)
    np.add.at(int_out[:, 0],  inverse, intensity[:, 0])
    np.add.at(counts,         inverse, 1.0)

    xyz_out /= counts[:, np.newaxis]
    int_out /= counts[:, np.newaxis]
    return xyz_out, int_out


# ── Preprocessing pipeline ────────────────────────────────────────────────────

class LidarPreprocessor:
    """Pure-Python/Open3D preprocessing — no ROS dependency, fully testable."""

    def __init__(
        self,
        roi_x: tuple[float, float] = (0.0, 50.0),
        roi_y: tuple[float, float] = (-10.0, 10.0),
        roi_z: tuple[float, float] = (-3.0, 2.0),
        voxel_size: float = 0.1,
        ransac_dist: float = 0.2,
        ransac_iter: int = 100,
        max_depth: float = 50.0,
    ):
        self.roi_x = roi_x
        self.roi_y = roi_y
        self.roi_z = roi_z
        self.voxel_size = voxel_size
        self.ransac_dist = ransac_dist
        self.ransac_iter = ransac_iter
        self.max_depth = max_depth

    def process(self, points_xyzi: np.ndarray) -> dict:
        """
        Run the full preprocessing pipeline.

        Args:
            points_xyzi: (N, 4) float32 array [x, y, z, intensity]

        Returns:
            dict with keys:
              'filtered'    (M, 4) float32 — non-ground points after all stages
              'ground'      (K, 4) float32 — ground plane points
              'stats'       dict  — frame statistics for logging
        """
        xyz = points_xyzi[:, :3]
        intensity = points_xyzi[:, 3:4]

        # ── Stage 1: ROI crop ─────────────────────────────────────────────────
        mask = (
            (xyz[:, 0] >= self.roi_x[0]) & (xyz[:, 0] <= self.roi_x[1]) &
            (xyz[:, 1] >= self.roi_y[0]) & (xyz[:, 1] <= self.roi_y[1]) &
            (xyz[:, 2] >= self.roi_z[0]) & (xyz[:, 2] <= self.roi_z[1])
        )
        xyz_roi = xyz[mask]
        int_roi = intensity[mask]
        n_after_roi = len(xyz_roi)

        if n_after_roi < 10:
            empty = np.zeros((0, 4), dtype=np.float32)
            return {"filtered": empty, "ground": empty,
                    "stats": {"n_input": len(points_xyzi), "n_roi": 0,
                              "n_voxel": 0, "n_ground": 0, "n_output": 0}}

        # ── Stage 2: Voxel downsampling (numpy — preserves intensity) ────────
        xyz_down, intensity_down = _voxel_downsample(
            xyz_roi, int_roi, self.voxel_size)
        n_after_voxel = len(xyz_down)

        if n_after_voxel < 10:
            empty = np.zeros((0, 4), dtype=np.float32)
            return {"filtered": empty, "ground": empty,
                    "stats": {"n_input": len(points_xyzi), "n_roi": n_after_roi,
                              "n_voxel": 0, "n_ground": 0, "n_output": 0}}

        # ── Stage 3: Distance filter ──────────────────────────────────────────
        dist = np.linalg.norm(xyz_down, axis=1)
        close_mask = dist <= self.max_depth
        xyz_down = xyz_down[close_mask]
        intensity_down = intensity_down[close_mask]

        if len(xyz_down) < 10:
            empty = np.zeros((0, 4), dtype=np.float32)
            return {"filtered": empty, "ground": empty,
                    "stats": {"n_input": len(points_xyzi), "n_roi": n_after_roi,
                              "n_voxel": n_after_voxel, "n_ground": 0, "n_output": 0}}

        # ── Stage 4: Ground plane removal (RANSAC) ────────────────────────────
        pcd_filtered = o3d.geometry.PointCloud()
        pcd_filtered.points = o3d.utility.Vector3dVector(xyz_down)

        plane_model, inlier_indices = pcd_filtered.segment_plane(
            distance_threshold=self.ransac_dist,
            ransac_n=3,
            num_iterations=self.ransac_iter,
        )

        inlier_set = set(inlier_indices)
        all_indices = np.arange(len(xyz_down))
        outlier_mask = np.array([i not in inlier_set for i in all_indices], dtype=bool)

        xyz_ground = xyz_down[~outlier_mask]
        xyz_nonground = xyz_down[outlier_mask]
        int_nonground = intensity_down[outlier_mask]

        n_ground = len(xyz_ground)
        n_output = len(xyz_nonground)

        # Pack back to (N, 4)
        filtered = np.hstack([xyz_nonground, int_nonground])
        ground = np.hstack([xyz_ground,
                             np.zeros((len(xyz_ground), 1), dtype=np.float32)])

        return {
            "filtered": filtered,
            "ground": ground,
            "stats": {
                "n_input":  len(points_xyzi),
                "n_roi":    n_after_roi,
                "n_voxel":  n_after_voxel,
                "n_ground": n_ground,
                "n_output": n_output,
            },
        }


# ── ROS 2 Node ────────────────────────────────────────────────────────────────

class LidarProcessorNode(Node):
    def __init__(self):
        super().__init__("lidar_processor")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("roi_x_min",   0.0)
        self.declare_parameter("roi_x_max",  50.0)
        self.declare_parameter("roi_y_min", -10.0)
        self.declare_parameter("roi_y_max",  10.0)
        self.declare_parameter("roi_z_min",  -3.0)
        self.declare_parameter("roi_z_max",   2.0)
        self.declare_parameter("voxel_size",  0.1)
        self.declare_parameter("ransac_dist", 0.2)
        self.declare_parameter("ransac_iter", 100)
        self.declare_parameter("max_depth",  50.0)

        def p(name):
            return self.get_parameter(name).value

        self._preprocessor = LidarPreprocessor(
            roi_x=(p("roi_x_min"), p("roi_x_max")),
            roi_y=(p("roi_y_min"), p("roi_y_max")),
            roi_z=(p("roi_z_min"), p("roi_z_max")),
            voxel_size=p("voxel_size"),
            ransac_dist=p("ransac_dist"),
            ransac_iter=p("ransac_iter"),
            max_depth=p("max_depth"),
        )

        # ── QoS ───────────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Sub / Pub ─────────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            PointCloud2, "/lidar/points", self._on_pointcloud, qos
        )
        self._pub_filtered = self.create_publisher(PointCloud2, "/lidar/filtered", qos)
        self._pub_ground   = self.create_publisher(PointCloud2, "/lidar/ground_plane", qos)

        self._frame_count = 0
        self.get_logger().info("LidarProcessor ready — waiting for /lidar/points")

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        try:
            points = _pc2_to_numpy(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to parse PointCloud2: {e}")
            return

        try:
            result = self._preprocessor.process(points)
        except Exception as e:
            self.get_logger().error(f"Preprocessing failed on frame {self._frame_count}: {e}")
            return
        s = result["stats"]

        # Publish filtered cloud
        header = Header()
        header.stamp = msg.header.stamp   # preserve original timestamp
        header.frame_id = msg.header.frame_id

        self._pub_filtered.publish(_numpy_to_pc2(result["filtered"], header))
        self._pub_ground.publish(_numpy_to_pc2(result["ground"], header))

        self._frame_count += 1
        if self._frame_count % 20 == 0:
            self.get_logger().info(
                f"Frame {self._frame_count} | "
                f"in={s['n_input']:6d}  roi={s['n_roi']:6d}  "
                f"voxel={s['n_voxel']:6d}  ground={s['n_ground']:5d}  "
                f"out={s['n_output']:6d}"
            )


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    try:
        node = LidarProcessorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
