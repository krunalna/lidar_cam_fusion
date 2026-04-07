"""
KITTI Publisher verification
Run with: pixi run verify-publisher

Tests three things:
  A. Code-level: imports, class structure, conversion utilities
  B. Data-level: KITTI sequence directory layout (if KITTI_SEQ is set)
  C. ROS-level: node can be instantiated and publishes correctly (dry-run)
"""

import os
import struct
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
from PIL import Image as PILImage

WORKSPACE_ROOT = Path(__file__).parent.parent
ROS_LOG_DIR = Path("/tmp/ros_logs")
ROS_LOG_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("ROS_LOG_DIR", str(ROS_LOG_DIR))
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


print(f"\n{BOLD}A. Source files{RESET}")

node_file = WORKSPACE_ROOT / "src/perception_pipeline_cpp" / "scripts" / "kitti_publisher.py"
launch_file = WORKSPACE_ROOT / "src/perception_pipeline_cpp" / "launch" / "fusion_pipeline_cpp.launch.py"
cmake_file = WORKSPACE_ROOT / "src/perception_pipeline_cpp" / "CMakeLists.txt"

check("kitti_publisher.py exists", node_file.exists())
check("fusion_pipeline_cpp.launch.py exists", launch_file.exists())
check("CMakeLists.txt installs kitti_publisher",
      "scripts/kitti_publisher.py" in cmake_file.read_text() and "RENAME kitti_publisher" in cmake_file.read_text())

print(f"\n{BOLD}B. Module imports{RESET}")

sys.path.insert(0, str(WORKSPACE_ROOT / "src/perception_pipeline_cpp" / "scripts"))

try:
    from kitti_publisher import _bin_to_pointcloud2, _png_to_image, KittiPublisherNode
    check("kitti_publisher imports OK", True)
except ImportError as exc:
    check("kitti_publisher imports OK", False, str(exc))
    print(f"\n{FAIL}  Cannot continue without successful import.")
    sys.exit(1)

print(f"\n{BOLD}C. Conversion utilities (no ROS daemon){RESET}")

from std_msgs.msg import Header

with tempfile.TemporaryDirectory() as tmpdir:
    pts = np.array([
        [1.0, 2.0, 3.0, 0.5],
        [4.0, 5.0, 6.0, 0.8],
        [7.0, 8.0, 9.0, 0.2],
        [0.1, 0.2, 0.3, 1.0],
        [-1.0, -2.0, -3.0, 0.0],
    ], dtype=np.float32)
    bin_path = Path(tmpdir) / "test.bin"
    pts.tofile(bin_path)

    hdr = Header()
    hdr.frame_id = "velodyne"
    pc_msg = _bin_to_pointcloud2(bin_path, hdr)

    check("PointCloud2 width == n_points", pc_msg.width == 5, str(pc_msg.width))
    check("PointCloud2 height == 1", pc_msg.height == 1)
    check("PointCloud2 point_step == 16 bytes", pc_msg.point_step == 16, str(pc_msg.point_step))
    check("PointCloud2 row_step == 80 bytes", pc_msg.row_step == 80, str(pc_msg.row_step))
    check("PointCloud2 fields == 4", len(pc_msg.fields) == 4, str(len(pc_msg.fields)))
    check("PointCloud2 field names",
          [field.name for field in pc_msg.fields] == ["x", "y", "z", "intensity"])

    x, y, z, intensity = struct.unpack_from("ffff", bytes(pc_msg.data), offset=0)
    check("Round-trip x coordinate", abs(x - 1.0) < 1e-6, f"{x:.4f}")
    check("Round-trip intensity", abs(intensity - 0.5) < 1e-6, f"{intensity:.4f}")

with tempfile.TemporaryDirectory() as tmpdir:
    fake_rgb = np.zeros((10, 20, 3), dtype=np.uint8)
    fake_rgb[:, :, 0] = 200
    png_path = Path(tmpdir) / "test.png"
    PILImage.fromarray(fake_rgb, mode="RGB").save(png_path)

    hdr = Header()
    hdr.frame_id = "camera_left"
    img_msg = _png_to_image(png_path, hdr)

    check("Image height == 10", img_msg.height == 10, str(img_msg.height))
    check("Image width == 20", img_msg.width == 20, str(img_msg.width))
    check("Image encoding == rgb8", img_msg.encoding == "rgb8")
    check("Image step == 60 bytes", img_msg.step == 60, str(img_msg.step))
    check("Image data length correct", len(img_msg.data) == 10 * 20 * 3)
    check("RGB conversion correct", img_msg.data[0] == 200, f"R={img_msg.data[0]}")

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
    check("sequence_path exists", seq_path.is_dir(), kitti_seq)
    check("image_02/data/ exists", (seq_path / "image_02" / "data").is_dir())
    check("velodyne_points/data/ exists", (seq_path / "velodyne_points" / "data").is_dir())

    images = sorted((seq_path / "image_02" / "data").glob("*.png"))
    lidars = sorted((seq_path / "velodyne_points" / "data").glob("*.bin"))
    check("PNG images found", len(images) > 0, f"{len(images)} files")
    check(".bin scans found", len(lidars) > 0, f"{len(lidars)} files")
    check("Frame counts match", len(images) == len(lidars),
          f"{len(images)} images vs {len(lidars)} scans", warn_only=(len(images) != len(lidars)))

    if images and lidars:
        raw = np.fromfile(lidars[0], dtype=np.float32)
        ok = raw.size % 4 == 0 and raw.size > 0
        check("First .bin parseable (N×4 float32)", ok, f"{raw.size // 4} points")

print(f"\n{BOLD}E. ROS node dry-run (synthetic sequence){RESET}")

import rclpy
from rclpy.executors import SingleThreadedExecutor

with tempfile.TemporaryDirectory() as tmpdir:
    img_dir = Path(tmpdir) / "image_02" / "data"
    lid_dir = Path(tmpdir) / "velodyne_points" / "data"
    img_dir.mkdir(parents=True)
    lid_dir.mkdir(parents=True)

    for idx in range(3):
        PILImage.fromarray(
            np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
            mode="RGB",
        ).save(img_dir / f"{idx:010d}.png")
        pts = np.random.rand(100, 4).astype(np.float32)
        pts.tofile(lid_dir / f"{idx:010d}.bin")

    rclpy.init(args=["--ros-args", "-p", f"sequence_path:={tmpdir}", "-p", "frame_rate:=100.0", "-p", "loop:=false"])
    try:
        node = KittiPublisherNode()
        check("Node instantiated", True)
        check("3 image files detected", node._n_frames == 3, str(node._n_frames))
        check("image publisher created", node._img_pub is not None)
        check("pointcloud publisher created", node._pc_pub is not None)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        deadline = time.monotonic() + 5.0 / 100.0 * 5
        while time.monotonic() < deadline and rclpy.ok():
            executor.spin_once(timeout_sec=0.01)

        check("Frame index advanced", node._frame_idx >= 3, str(node._frame_idx))
    except Exception as exc:
        check("Node dry-run", False, str(exc))
    finally:
        if rclpy.ok():
            rclpy.shutdown()

print(f"\n{'─' * 50}")
if failures:
    print(f"\033[91m{BOLD}FAILED{RESET} — {len(failures)} check(s) not passing:")
    for failure in failures:
        print(f"  • {failure}")
    sys.exit(1)
else:
    print(f"\033[92m{BOLD}ALL CHECKS PASSED{RESET} — KITTI publisher is ready.")
    if not os.environ.get("KITTI_SEQ"):
        print("Set KITTI_SEQ to additionally validate a real KITTI sequence layout.")
print()
