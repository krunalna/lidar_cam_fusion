"""
Step 2 verification — KITTI Publisher Node
Run with: pixi run verify-publisher

Tests three things:
  A. Code-level: imports, class structure, conversion utilities
  B. Data-level: KITTI sequence directory layout (if KITTI_SEQ is set)
  C. ROS-level: node can be instantiated and publishes correctly (dry-run)
"""

import sys
import os
import time
import struct
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image as PILImage

WORKSPACE_ROOT = Path(__file__).parent.parent
PASS = "\033[92m  PASS\033[0m"
FAIL = "\033[91m  FAIL\033[0m"
WARN = "\033[93m  WARN\033[0m"
BOLD = "\033[1m"
RESET = "\033[0m"
failures = []


def check(label, ok, detail="", warn_only=False):
    tag = (WARN if warn_only else FAIL) if not ok else PASS
    suffix = f"  ({detail})" if detail else ""
    print(f"{tag}  {label}{suffix}")
    if not ok and not warn_only:
        failures.append(label)


# ═══════════════════════════════════════════════════════════════════════════════
# A. Code-level checks
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{BOLD}A. Source files{RESET}")

node_file = WORKSPACE_ROOT / "src/perception_pipeline/perception_pipeline/kitti_publisher_node.py"
launch_file = WORKSPACE_ROOT / "src/perception_pipeline/launch/fusion_pipeline.launch.py"

check("kitti_publisher_node.py exists", node_file.exists())
check("fusion_pipeline.launch.py exists", launch_file.exists())

# Check entry point is registered in setup.py
setup_py = WORKSPACE_ROOT / "src/perception_pipeline/setup.py"
setup_text = setup_py.read_text()
check("kitti_publisher entry point in setup.py",
      "kitti_publisher = perception_pipeline.kitti_publisher_node:main" in setup_text)

print(f"\n{BOLD}B. Module imports{RESET}")

sys.path.insert(0, str(WORKSPACE_ROOT / "src/perception_pipeline"))

try:
    from perception_pipeline.kitti_publisher_node import (
        _bin_to_pointcloud2,
        _png_to_image,
        KittiPublisherNode,
    )
    check("kitti_publisher_node imports OK", True)
except ImportError as e:
    check("kitti_publisher_node imports OK", False, str(e))
    print(f"\n{FAIL}  Cannot continue without successful import.")
    sys.exit(1)

# ═══════════════════════════════════════════════════════════════════════════════
# B. Conversion utility unit tests (no ROS daemon needed)
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{BOLD}C. Conversion utilities (no ROS daemon){RESET}")

from std_msgs.msg import Header

# ── _bin_to_pointcloud2 ───────────────────────────────────────────────────────
with tempfile.TemporaryDirectory() as tmpdir:
    # Write synthetic .bin: 5 points, each [x, y, z, intensity]
    pts = np.array([
        [1.0, 2.0, 3.0, 0.5],
        [4.0, 5.0, 6.0, 0.8],
        [7.0, 8.0, 9.0, 0.2],
        [0.1, 0.2, 0.3, 1.0],
        [-1., -2., -3., 0.0],
    ], dtype=np.float32)
    bin_path = Path(tmpdir) / "test.bin"
    pts.tofile(bin_path)

    hdr = Header()
    hdr.frame_id = "velodyne"
    pc_msg = _bin_to_pointcloud2(bin_path, hdr)

    check("PointCloud2 width == n_points",     pc_msg.width == 5,       str(pc_msg.width))
    check("PointCloud2 height == 1",           pc_msg.height == 1)
    check("PointCloud2 point_step == 16 bytes", pc_msg.point_step == 16, str(pc_msg.point_step))
    check("PointCloud2 row_step == 80 bytes",  pc_msg.row_step == 80,   str(pc_msg.row_step))
    check("PointCloud2 fields == 4",           len(pc_msg.fields) == 4, str(len(pc_msg.fields)))
    check("PointCloud2 field names",
          [f.name for f in pc_msg.fields] == ["x", "y", "z", "intensity"])

    # Decode first point and verify round-trip
    x, y, z, i = struct.unpack_from("ffff", bytes(pc_msg.data), offset=0)
    check("Round-trip x coordinate", abs(x - 1.0) < 1e-6, f"{x:.4f}")
    check("Round-trip intensity",    abs(i - 0.5) < 1e-6, f"{i:.4f}")

with tempfile.TemporaryDirectory() as tmpdir:
    # Write a synthetic 10×20 RGB image
    fake_rgb = np.zeros((10, 20, 3), dtype=np.uint8)
    fake_rgb[:, :, 0] = 200
    png_path = Path(tmpdir) / "test.png"
    PILImage.fromarray(fake_rgb, mode="RGB").save(png_path)

    hdr = Header()
    hdr.frame_id = "camera_left"
    img_msg = _png_to_image(png_path, hdr)

    check("Image height == 10",       img_msg.height == 10,    str(img_msg.height))
    check("Image width == 20",        img_msg.width == 20,     str(img_msg.width))
    check("Image encoding == rgb8",   img_msg.encoding == "rgb8")
    check("Image step == 60 bytes",   img_msg.step == 60,      str(img_msg.step))
    check("Image data length correct", len(img_msg.data) == 10 * 20 * 3)

    # Verify the stored RGB data survives the round-trip
    pixel_r = img_msg.data[0]  # first pixel, R channel in rgb8
    check("RGB conversion correct", pixel_r == 200, f"R={pixel_r}")

# ═══════════════════════════════════════════════════════════════════════════════
# C. KITTI data directory check (optional — set KITTI_SEQ to run)
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{BOLD}D. KITTI data directory{RESET}")

kitti_seq = os.environ.get("KITTI_SEQ", "")
if not kitti_seq:
    check(
        "KITTI_SEQ env var set",
        False,
        "set KITTI_SEQ=/path/to/sequence to validate data layout",
        warn_only=True,
    )
else:
    seq_path = Path(kitti_seq)
    check("sequence_path exists",           seq_path.is_dir(), kitti_seq)
    check("image_02/data/ exists",          (seq_path / "image_02" / "data").is_dir())
    check("velodyne_points/data/ exists",   (seq_path / "velodyne_points" / "data").is_dir())

    images = sorted((seq_path / "image_02" / "data").glob("*.png"))
    lidars = sorted((seq_path / "velodyne_points" / "data").glob("*.bin"))
    check("PNG images found",   len(images) > 0, f"{len(images)} files")
    check(".bin scans found",   len(lidars) > 0, f"{len(lidars)} files")
    check("Frame counts match", len(images) == len(lidars),
          f"{len(images)} images vs {len(lidars)} scans", warn_only=(len(images) != len(lidars)))

    if images and lidars:
        # Validate first .bin is readable and has the right shape
        raw = np.fromfile(lidars[0], dtype=np.float32)
        ok = raw.size % 4 == 0 and raw.size > 0
        check("First .bin parseable (N×4 float32)", ok, f"{raw.size // 4} points")

# ═══════════════════════════════════════════════════════════════════════════════
# D. ROS node dry-run (synthetic sequence on disk)
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{BOLD}E. ROS node dry-run (synthetic sequence){RESET}")

import rclpy
from rclpy.executors import SingleThreadedExecutor

with tempfile.TemporaryDirectory() as tmpdir:
    # Build a minimal KITTI-like sequence (3 frames)
    img_dir = Path(tmpdir) / "image_02" / "data"
    lid_dir = Path(tmpdir) / "velodyne_points" / "data"
    img_dir.mkdir(parents=True)
    lid_dir.mkdir(parents=True)

    for i in range(3):
        # 640×480 image
        PILImage.fromarray(
            np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
            mode="RGB",
        ).save(img_dir / f"{i:010d}.png")
        # 100-point cloud
        pts = np.random.rand(100, 4).astype(np.float32)
        pts.tofile(lid_dir / f"{i:010d}.bin")

    rclpy.init(args=["--ros-args", "-p", f"sequence_path:={tmpdir}",
                     "-p", "frame_rate:=100.0", "-p", "loop:=false"])
    try:
        node = KittiPublisherNode()
        check("Node instantiated", True)
        check("3 image files detected",   node._n_frames == 3, str(node._n_frames))
        check("image publisher created",  node._img_pub is not None)
        check("pointcloud publisher created", node._pc_pub is not None)

        # Spin for 5 frames worth of time (should publish all 3 and stop)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        deadline = time.monotonic() + 5.0 / 100.0 * 5   # 5 frame periods
        while time.monotonic() < deadline and rclpy.ok():
            executor.spin_once(timeout_sec=0.01)

        check("Frame index advanced", node._frame_idx >= 3, str(node._frame_idx))
    except Exception as e:
        check("Node dry-run", False, str(e))
    finally:
        if rclpy.ok():
            rclpy.shutdown()

# ═══════════════════════════════════════════════════════════════════════════════
# Summary
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{'─' * 50}")
if failures:
    print(f"\033[91m{BOLD}FAILED{RESET} — {len(failures)} check(s) not passing:")
    for f in failures:
        print(f"  • {f}")
    sys.exit(1)
else:
    print(f"\033[92m{BOLD}ALL CHECKS PASSED{RESET} — Step 2 KITTI publisher is ready.")
    if not os.environ.get("KITTI_SEQ"):
        print(f"\n  {WARN[:-4]}NOTE{RESET}  To also validate your KITTI download, re-run with:")
        print("       KITTI_SEQ=/path/to/sequence pixi run verify-publisher")
print()
