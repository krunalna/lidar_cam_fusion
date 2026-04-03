"""
C++ LiDAR Processor verify script
===================================
Verifies the perception_pipeline_cpp package: source files, build artifact,
and a live ROS dry-run against the compiled binary.

Run with: pixi run verify-cpp-lidar
"""

import subprocess
import sys
import time
from pathlib import Path

import numpy as np

WORKSPACE_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(WORKSPACE_ROOT / "src/perception_pipeline"))

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
# A. Source files
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{BOLD}A. Source files{RESET}")

pkg_root = WORKSPACE_ROOT / "src/perception_pipeline_cpp"

check("lidar_preprocessor.hpp exists",
      (pkg_root / "include/perception_pipeline_cpp/lidar_preprocessor.hpp").exists())
check("lidar_preprocessor.cpp exists",
      (pkg_root / "src/lidar_preprocessor.cpp").exists())
check("lidar_processor_node.cpp exists",
      (pkg_root / "src/lidar_processor_node.cpp").exists())
check("CMakeLists.txt exists",
      (pkg_root / "CMakeLists.txt").exists())

launch_text = (pkg_root / "launch/fusion_pipeline_cpp.launch.py").read_text()
check("launch file references lidar_processor_cpp",
      "lidar_processor_cpp" in launch_text)
check("launch file reuses Python kitti_publisher",
      "perception_pipeline" in launch_text and "kitti_publisher" in launch_text)

# ═══════════════════════════════════════════════════════════════════════════════
# B. Build artifact
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{BOLD}B. Build artifact{RESET}")

binary = WORKSPACE_ROOT / "install/perception_pipeline_cpp/lib/perception_pipeline_cpp/lidar_processor_cpp"
check("lidar_processor_cpp binary exists", binary.exists(), str(binary))

if not binary.exists():
    print(f"{FAIL}  Binary not found — run: pixi run build-cpp")
    print(f"\n{'─' * 50}")
    print(f"\033[91m{BOLD}FAILED{RESET} — binary missing, cannot run dry-run.")
    sys.exit(1)

# ═══════════════════════════════════════════════════════════════════════════════
# C. ROS dry-run — launch the C++ node as a subprocess, publish synthetic data,
#    verify it publishes to /lidar/filtered and /lidar/ground_plane.
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{BOLD}C. ROS dry-run{RESET}")

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

# Import the Python helpers for building/reading PointCloud2 messages
try:
    from perception_pipeline.lidar_processor_node import _numpy_to_pc2, _pc2_to_numpy
    check("Python PC2 helpers importable", True)
except ImportError as e:
    check("Python PC2 helpers importable", False, str(e))
    sys.exit(1)

# Build a synthetic cloud: flat ground plane + object cluster
rng = np.random.default_rng(42)
n_ground = 3000
n_obj = 400

ground_pts = np.column_stack([
    rng.uniform(0, 40, n_ground),
    rng.uniform(-8, 8, n_ground),
    rng.normal(-1.7, 0.05, n_ground),
    rng.uniform(0.1, 0.9, n_ground),   # non-zero intensity
]).astype(np.float32)

obj_pts = np.column_stack([
    rng.normal(20, 0.5, n_obj),
    rng.normal(3, 0.3, n_obj),
    rng.uniform(-0.8, 1.5, n_obj),
    np.ones(n_obj, dtype=np.float32) * 0.6,
]).astype(np.float32)

cloud = np.vstack([ground_pts, obj_pts])

received = {"filtered": [], "ground": []}
cpp_proc = None

rclpy.init()
try:
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )

    spy = rclpy.create_node("verify_cpp_lidar_spy")

    def on_filtered(msg):
        received["filtered"].append(msg)

    def on_ground(msg):
        received["ground"].append(msg)

    spy.create_subscription(PointCloud2, "/lidar/filtered",     on_filtered, qos)
    spy.create_subscription(PointCloud2, "/lidar/ground_plane", on_ground,   qos)
    pub = spy.create_publisher(PointCloud2, "/lidar/points", qos)

    # Launch the C++ node in a subprocess
    import os
    env = os.environ.copy()
    # Ensure the install overlay is on the path
    setup_bash = WORKSPACE_ROOT / "install/setup.bash"
    if setup_bash.exists():
        # Source install/setup.bash by running through bash and capturing env
        result = subprocess.run(
            ["bash", "-c", f"source {setup_bash} && env"],
            capture_output=True, text=True
        )
        for line in result.stdout.splitlines():
            if "=" in line:
                k, _, v = line.partition("=")
                env[k] = v

    cpp_proc = subprocess.Popen(
        [str(binary)],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    check("C++ node process launched", cpp_proc.poll() is None)

    executor = SingleThreadedExecutor()
    executor.add_node(spy)

    # Give the C++ node a moment to start up and connect
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)

    # Build and publish the test message
    hdr = Header()
    hdr.frame_id = "velodyne"
    hdr.stamp = spy.get_clock().now().to_msg()
    test_msg = _numpy_to_pc2(cloud, hdr)

    deadline = time.monotonic() + 3.0
    published = False
    while time.monotonic() < deadline and rclpy.ok():
        if not published:
            pub.publish(test_msg)
            published = True
        executor.spin_once(timeout_sec=0.05)
        if received["filtered"] and received["ground"]:
            break

    check("Published /lidar/filtered",
          len(received["filtered"]) > 0,
          f"{len(received['filtered'])} msg(s)")
    check("Published /lidar/ground_plane",
          len(received["ground"]) > 0,
          f"{len(received['ground'])} msg(s)")

    # ── Validate filtered cloud structure ─────────────────────────────────────
    if received["filtered"]:
        fmsg = received["filtered"][0]
        check("filtered point_step == 16",
              fmsg.point_step == 16,
              f"got {fmsg.point_step}")
        check("filtered frame_id preserved",
              fmsg.header.frame_id == "velodyne",
              f"got '{fmsg.header.frame_id}'")
        check("filtered width > 0",
              fmsg.width > 0,
              f"{fmsg.width} points")

        # Check intensity channel is present and non-zero
        pts_back = _pc2_to_numpy(fmsg)
        has_intensity_field = any(f.name == "intensity" for f in fmsg.fields)
        check("filtered has intensity field", has_intensity_field)
        if has_intensity_field and pts_back.shape[0] > 0:
            max_intensity = float(pts_back[:, 3].max())
            check("filtered intensity non-zero (preserved)",
                  max_intensity > 0.0,
                  f"max intensity={max_intensity:.4f}")

    if received["ground"]:
        gmsg = received["ground"][0]
        check("ground point_step == 16",
              gmsg.point_step == 16,
              f"got {gmsg.point_step}")
        check("ground width > 0",
              gmsg.width > 0,
              f"{gmsg.width} points")

        # Ground z should be near planted plane (~-1.7)
        gpts = _pc2_to_numpy(gmsg)
        if gpts.shape[0] > 0:
            mean_z = float(gpts[:, 2].mean())
            check("ground mean z near planted plane (-1.7)",
                  abs(mean_z - (-1.7)) < 0.6,
                  f"mean z={mean_z:.3f}")

    # ── Pipeline reduction sanity check ───────────────────────────────────────
    if received["filtered"] and received["ground"]:
        n_filtered = received["filtered"][0].width
        n_ground_pts = received["ground"][0].width
        total_out = n_filtered + n_ground_pts
        check("C++ output count < input count (filtering applied)",
              total_out < len(cloud),
              f"in={len(cloud)}  out={total_out}")

    # ── C++ node still running? ────────────────────────────────────────────────
    check("C++ node still alive after dry-run",
          cpp_proc.poll() is None,
          f"exit code: {cpp_proc.poll()}")

except Exception as e:
    check("ROS dry-run", False, str(e))
    import traceback; traceback.print_exc()
finally:
    if cpp_proc and cpp_proc.poll() is None:
        cpp_proc.terminate()
        try:
            cpp_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            cpp_proc.kill()
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
    print(f"\033[92m{BOLD}ALL CHECKS PASSED{RESET} — C++ LiDAR processor (perception_pipeline_cpp) is ready.")
print()
