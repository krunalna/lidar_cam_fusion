#!/usr/bin/env python3
"""
Step 1 verification — Environment & Workspace Setup
Run with: pixi run python scripts/verify_step1.py
"""

import sys
import os
import subprocess
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


# ── 1. Python ─────────────────────────────────────────────────────────────────
print(f"\n{BOLD}1. Python{RESET}")
ver = sys.version_info
check("Python >= 3.12", ver >= (3, 12), f"{ver.major}.{ver.minor}.{ver.micro}")

# ── 2. ROS 2 distro ───────────────────────────────────────────────────────────
print(f"\n{BOLD}2. ROS 2 Jazzy{RESET}")
ros_distro = os.environ.get("ROS_DISTRO", "")
check("ROS_DISTRO=jazzy", ros_distro == "jazzy", ros_distro or "not set")

try:
    import rclpy  # noqa: F401
    check("rclpy importable", True)
except ImportError as e:
    check("rclpy importable", False, str(e))

# ── 3. ROS 2 message packages ─────────────────────────────────────────────────
print(f"\n{BOLD}3. ROS 2 message packages{RESET}")
ros_imports = [
    ("sensor_msgs.msg", ["Image", "PointCloud2"]),
    ("vision_msgs.msg", ["Detection2DArray", "Detection3DArray"]),
    ("visualization_msgs.msg", ["MarkerArray", "Marker"]),
    ("message_filters", ["ApproximateTimeSynchronizer"]),
    ("tf2_ros", ["Buffer", "TransformListener"]),
    ("rosbag2_py", []),
]
for module, attrs in ros_imports:
    try:
        mod = __import__(module, fromlist=attrs)
        for attr in attrs:
            getattr(mod, attr)
        detail = ", ".join(attrs) if attrs else "imported"
        check(module, True, detail)
    except (ImportError, AttributeError) as e:
        check(module, False, str(e))

# ── 4. ML / vision stack ──────────────────────────────────────────────────────
print(f"\n{BOLD}4. ML / vision stack{RESET}")

try:
    import numpy as np
    check("numpy", True, np.__version__)
except ImportError as e:
    check("numpy", False, str(e))

try:
    import cv2
    check("cv2 (OpenCV)", True, cv2.__version__)
except ImportError as e:
    check("cv2 (OpenCV)", False, str(e))

try:
    import open3d as o3d
    check("open3d", True, o3d.__version__)
except ImportError as e:
    check("open3d", False, str(e))

try:
    import scipy
    check("scipy", True, scipy.__version__)
except ImportError as e:
    check("scipy", False, str(e))

try:
    import yaml
    check("pyyaml", True)
except ImportError as e:
    check("pyyaml", False, str(e))

try:
    import ultralytics
    check("ultralytics (YOLOv8)", True, ultralytics.__version__)
except ImportError as e:
    check("ultralytics (YOLOv8)", False, str(e))

# ── 5. Build tooling ──────────────────────────────────────────────────────────
print(f"\n{BOLD}5. Build tooling{RESET}")
result = subprocess.run(
    ["colcon", "version-check"],
    capture_output=True, text=True
)
check("colcon", result.returncode == 0, "colcon version-check passed")

result = subprocess.run(
    ["ros2", "pkg", "list"],
    capture_output=True, text=True
)
required_pkgs = ["sensor_msgs", "vision_msgs", "tf2_ros", "rosbag2"]
for pkg in required_pkgs:
    found = pkg in result.stdout
    check(f"ros2 pkg: {pkg}", found)

# ── 6. Workspace scaffold ─────────────────────────────────────────────────────
print(f"\n{BOLD}6. Workspace scaffold{RESET}")
required_files = [
    "pixi.toml",
    "scripts/activate_ros.sh",
    "src/perception_pipeline/package.xml",
    "src/perception_pipeline/setup.py",
    "src/perception_pipeline/setup.cfg",
    "src/perception_pipeline/config/calibration.yaml",
    "src/perception_pipeline/perception_pipeline/__init__.py",
    "src/perception_pipeline/perception_pipeline/utils/__init__.py",
    "src/perception_pipeline/resource/perception_pipeline",
    ".gitignore",
]
for rel in required_files:
    path = WORKSPACE_ROOT / rel
    check(rel, path.exists())

# ── 7. Calibration YAML sanity ────────────────────────────────────────────────
print(f"\n{BOLD}7. Calibration YAML{RESET}")
calib_path = WORKSPACE_ROOT / "src/perception_pipeline/config/calibration.yaml"
if calib_path.exists():
    import yaml
    with open(calib_path) as f:
        calib = yaml.safe_load(f)
    required_keys = [("camera", "fx"), ("camera", "fy"), ("camera", "cx"),
                     ("camera", "cy"), ("lidar_to_camera", "T"), ("fusion", "max_depth")]
    for section, key in required_keys:
        has_key = section in calib and key in calib[section]
        check(f"calibration.yaml [{section}.{key}]", has_key)
else:
    check("calibration.yaml exists", False)

# ── Summary ───────────────────────────────────────────────────────────────────
print(f"\n{'─' * 50}")
if failures:
    print(f"\033[91m{BOLD}FAILED{RESET} — {len(failures)} check(s) not passing:")
    for f in failures:
        print(f"  • {f}")
    sys.exit(1)
else:
    print(f"\033[92m{BOLD}ALL CHECKS PASSED{RESET} — Step 1 environment is ready.")
print()
