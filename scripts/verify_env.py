#!/usr/bin/env python3
"""
Environment and workspace verification
Run with: pixi run verify-env
"""

import os
import subprocess
import sys
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).parent.parent
PASS = "\033[92m  PASS\033[0m"
FAIL = "\033[91m  FAIL\033[0m"
WARN = "\033[93m  WARN\033[0m"
BOLD = "\033[1m"
RESET = "\033[0m"

failures = []


def check(label: str, ok: bool, detail: str = "", warn_only: bool = False):
    tag = (WARN if warn_only else FAIL) if not ok else PASS
    suffix = f"  ({detail})" if detail else ""
    print(f"{tag}  {label}{suffix}")
    if not ok and not warn_only:
        failures.append(label)


print(f"\n{BOLD}1. Python{RESET}")
ver = sys.version_info
check("Python >= 3.12", ver >= (3, 12), f"{ver.major}.{ver.minor}.{ver.micro}")

print(f"\n{BOLD}2. ROS 2 Jazzy{RESET}")
ros_distro = os.environ.get("ROS_DISTRO", "")
check("ROS_DISTRO=jazzy", ros_distro == "jazzy", ros_distro or "not set")

try:
    import rclpy  # noqa: F401
    check("rclpy importable", True)
except ImportError as exc:
    check("rclpy importable", False, str(exc))

print(f"\n{BOLD}3. ROS 2 message and launch packages{RESET}")
ros_imports = [
    ("sensor_msgs.msg", ["Image", "PointCloud2"]),
    ("vision_msgs.msg", ["Detection2DArray", "Detection3DArray"]),
    ("visualization_msgs.msg", ["MarkerArray", "Marker"]),
    ("message_filters", ["ApproximateTimeSynchronizer"]),
    ("launch", ["LaunchDescription"]),
    ("launch_ros.actions", ["Node"]),
    ("ament_index_python.packages", ["get_package_share_directory"]),
    ("rosbag2_py", []),
]
for module, attrs in ros_imports:
    try:
        mod = __import__(module, fromlist=attrs)
        for attr in attrs:
            getattr(mod, attr)
        detail = ", ".join(attrs) if attrs else "imported"
        check(module, True, detail)
    except (ImportError, AttributeError) as exc:
        check(module, False, str(exc))

print(f"\n{BOLD}4. ML / vision stack{RESET}")

try:
    import numpy as np
    check("numpy", True, np.__version__)
except ImportError as exc:
    check("numpy", False, str(exc))

try:
    import cv2
    check("cv2 (OpenCV)", True, cv2.__version__)
except ImportError as exc:
    check("cv2 (OpenCV)", False, str(exc))

try:
    import yaml
    check("pyyaml", True)
except ImportError as exc:
    check("pyyaml", False, str(exc))

try:
    import ultralytics
    check("ultralytics (YOLOv8)", True, ultralytics.__version__)
except ImportError as exc:
    check("ultralytics (YOLOv8)", False, str(exc))

print(f"\n{BOLD}5. Build tooling{RESET}")
result = subprocess.run(["colcon", "version-check"], capture_output=True, text=True)
check("colcon", result.returncode == 0, "colcon version-check passed")

result = subprocess.run(["ros2", "pkg", "list"], capture_output=True, text=True)
required_pkgs = [
    "sensor_msgs",
    "vision_msgs",
    "message_filters",
    "visualization_msgs",
    "rosbag2",
]
for pkg in required_pkgs:
    check(f"ros2 pkg: {pkg}", pkg in result.stdout)

print(f"\n{BOLD}6. Workspace scaffold{RESET}")
required_files = [
    "pixi.toml",
    "scripts/activate_ros.sh",
    "scripts/download_kitti.py",
    "scripts/export_yolo_onnx.py",
    "scripts/verify_publisher.py",
    "scripts/verify_lidar.py",
    "scripts/verify_camera.py",
    "scripts/verify_projections.py",
    "scripts/verify_fusion.py",
    "src/perception_pipeline_cpp/package.xml",
    "src/perception_pipeline_cpp/CMakeLists.txt",
    "src/perception_pipeline_cpp/config/calibration.yaml",
    "src/perception_pipeline_cpp/scripts/kitti_publisher.py",
    "src/perception_pipeline_cpp/launch/fusion_pipeline_cpp.launch.py",
    ".gitignore",
]
for rel in required_files:
    path = WORKSPACE_ROOT / rel
    check(rel, path.exists())

print(f"\n{BOLD}7. Calibration YAML{RESET}")
calib_path = WORKSPACE_ROOT / "src/perception_pipeline_cpp/config/calibration.yaml"
if calib_path.exists():
    import yaml
    with open(calib_path) as handle:
        calib = yaml.safe_load(handle)
    required_keys = [
        ("camera", "fx"),
        ("camera", "fy"),
        ("camera", "cx"),
        ("camera", "cy"),
        ("lidar_to_camera", "T"),
        ("fusion", "max_depth"),
    ]
    for section, key in required_keys:
        has_key = section in calib and key in calib[section]
        check(f"calibration.yaml [{section}.{key}]", has_key)
else:
    check("calibration.yaml exists", False)

print(f"\n{'─' * 50}")
if failures:
    print(f"\033[91m{BOLD}FAILED{RESET} — {len(failures)} check(s) not passing:")
    for failure in failures:
        print(f"  • {failure}")
    sys.exit(1)
else:
    print(f"\033[92m{BOLD}ALL CHECKS PASSED{RESET} — Environment is ready.")
print()
