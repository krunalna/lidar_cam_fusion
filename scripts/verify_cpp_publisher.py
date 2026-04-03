"""
verify_cpp_publisher.py
=======================
Verification script for the C++ kitti_publisher_cpp node.

Sections:
  A. Source files — package skeleton, headers, sources, launch file
  B. Build artifacts — compiled executable present in install/
  C. ROS node dry-run — launch node against real KITTI data, verify topics
     (skipped automatically if KITTI data is not downloaded)

Usage:
  pixi run verify-cpp-publisher
"""

import os
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).parent.parent
FAILURES: list[str] = []


def check(condition: bool, label: str) -> bool:
    status = "PASS" if condition else "FAIL"
    print(f"  [{status}] {label}")
    if not condition:
        FAILURES.append(label)
    return condition


# ── A. Source files ───────────────────────────────────────────────────────────

print("\n── A. Source files ──────────────────────────────────────────────────────")

pkg = WORKSPACE / "src" / "perception_pipeline_cpp"
check((pkg / "CMakeLists.txt").exists(),           "CMakeLists.txt exists")
check((pkg / "package.xml").exists(),              "package.xml exists")
check((pkg / "include" / "perception_pipeline_cpp" / "kitti_reader.hpp").exists(),
      "kitti_reader.hpp exists")
check((pkg / "src" / "kitti_reader.cpp").exists(), "kitti_reader.cpp exists")
check((pkg / "src" / "kitti_publisher_node.cpp").exists(),
      "kitti_publisher_node.cpp exists")
check((pkg / "launch" / "fusion_pipeline_cpp.launch.py").exists(),
      "launch file exists")

# Spot-check key identifiers in source
kitti_reader_hpp = (pkg / "include" / "perception_pipeline_cpp" / "kitti_reader.hpp").read_text()
check("class KittiReader" in kitti_reader_hpp,     "KittiReader class declared")
check("read_image" in kitti_reader_hpp,            "read_image declared")
check("read_lidar" in kitti_reader_hpp,            "read_lidar declared")
check("timestamp_ns" in kitti_reader_hpp,          "timestamp_ns declared")

node_cpp = (pkg / "src" / "kitti_publisher_node.cpp").read_text()
check("KittiPublisherNode" in node_cpp,            "KittiPublisherNode defined")
check("/camera/image_raw" in node_cpp,             "publishes /camera/image_raw")
check("/lidar/points" in node_cpp,                 "publishes /lidar/points")
check("sequence_path" in node_cpp,                 "sequence_path parameter")
check("frame_rate" in node_cpp,                    "frame_rate parameter")
check("loop_" in node_cpp,                         "loop parameter")
check("camera_left" in node_cpp,                   "frame_id camera_left")
check("velodyne" in node_cpp,                      "frame_id velodyne")
check("rgb8" in node_cpp,                          "rgb8 encoding")
check("point_step   = 16" in node_cpp,             "PointCloud2 point_step = 16")

# ── B. Build artifacts ────────────────────────────────────────────────────────

print("\n── B. Build artifacts ───────────────────────────────────────────────────")

artifact = WORKSPACE / "install" / "perception_pipeline_cpp" / "lib" / \
           "perception_pipeline_cpp" / "kitti_publisher_cpp"
built = artifact.exists()
check(built, f"kitti_publisher_cpp executable exists at {artifact.relative_to(WORKSPACE)}")

if not built:
    print("\n  -> Run 'pixi run build-cpp' first, then re-run this script.")

# ── C. ROS node dry-run ───────────────────────────────────────────────────────

print("\n── C. ROS node dry-run ──────────────────────────────────────────────────")

kitti_seq = os.environ.get("KITTI_SEQ", "")
if not kitti_seq:
    # Try default location
    default = WORKSPACE / "data" / "kitti" / "2011_09_26" / "2011_09_26_drive_0001_sync"
    if default.is_dir():
        kitti_seq = str(default)

if not kitti_seq or not Path(kitti_seq).is_dir():
    print("  [SKIP] KITTI data not found. Set KITTI_SEQ or run 'pixi run download-kitti'.")
elif not built:
    print("  [SKIP] Build artifact missing — build first.")
else:
    print(f"  Using sequence: {kitti_seq}")
    try:
        import subprocess
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import Image, PointCloud2

        # Set domain BEFORE rclpy.init so spy and subprocess share the same domain
        os.environ["ROS_DOMAIN_ID"] = "99"

        rclpy.init(args=None)

        received: dict[str, list] = {"image": [], "lidar": []}

        class Spy(Node):
            def __init__(self):
                super().__init__("verify_spy")
                from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=5,
                )
                self.create_subscription(Image,        "/camera/image_raw", self._on_img, qos)
                self.create_subscription(PointCloud2,  "/lidar/points",     self._on_pc,  qos)

            def _on_img(self, msg):
                received["image"].append(msg)

            def _on_pc(self, msg):
                received["lidar"].append(msg)

        # Launch publisher node in a subprocess on the same isolated domain
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "99"
        proc = subprocess.Popen([
            str(artifact),
            "--ros-args",
            "-p", f"sequence_path:={kitti_seq}",
            "-p", "frame_rate:=10.0",
            "-p", "loop:=true",
        ], env=env)

        spy = Spy()
        executor = SingleThreadedExecutor()
        executor.add_node(spy)

        deadline = time.time() + 5.0
        while time.time() < deadline and (not received["image"] or not received["lidar"]):
            executor.spin_once(timeout_sec=0.1)

        proc.terminate()
        proc.wait(timeout=3)

        img_msgs  = received["image"]
        lidar_msgs = received["lidar"]

        check(len(img_msgs) > 0,  "Received Image messages on /camera/image_raw")
        check(len(lidar_msgs) > 0, "Received PointCloud2 messages on /lidar/points")

        if img_msgs:
            m = img_msgs[0]
            check(m.encoding == "rgb8",      f"Image encoding is rgb8 (got {m.encoding})")
            check(m.height > 0,              f"Image height > 0 (got {m.height})")
            check(m.width > 0,               f"Image width > 0 (got {m.width})")
            check(m.header.frame_id == "camera_left",
                  f"Image frame_id = camera_left (got {m.header.frame_id})")

        if lidar_msgs:
            m = lidar_msgs[0]
            check(m.point_step == 16,        f"PointCloud2 point_step = 16 (got {m.point_step})")
            check(m.width > 0,               f"PointCloud2 width > 0 (got {m.width})")
            check(m.header.frame_id == "velodyne",
                  f"PointCloud2 frame_id = velodyne (got {m.header.frame_id})")
            check(len(m.fields) == 4,        f"PointCloud2 has 4 fields (got {len(m.fields)})")

        if img_msgs and lidar_msgs:
            img_stamp  = (img_msgs[0].header.stamp.sec,  img_msgs[0].header.stamp.nanosec)
            lidar_stamp = (lidar_msgs[0].header.stamp.sec, lidar_msgs[0].header.stamp.nanosec)
            check(img_stamp == lidar_stamp,
                  f"Image and LiDAR share the same stamp ({img_stamp})")

        rclpy.shutdown()

    except Exception as e:
        print(f"  [FAIL] ROS dry-run error: {e}")
        FAILURES.append(f"ROS dry-run: {e}")

# ── Summary ───────────────────────────────────────────────────────────────────

print("\n" + "─" * 60)
if FAILURES:
    print(f"FAILED — {len(FAILURES)} check(s) failed:")
    for f in FAILURES:
        print(f"  • {f}")
    sys.exit(1)
else:
    print("PASSED — all checks passed.")
