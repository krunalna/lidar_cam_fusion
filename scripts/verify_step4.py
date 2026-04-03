"""
Step 4 verification — Camera Detector Node
Run with: pixi run verify4
"""

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
# A. Source file checks
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{BOLD}A. Source files{RESET}")

node_file = WORKSPACE_ROOT / "src/perception_pipeline/perception_pipeline/camera_detector_node.py"
check("camera_detector_node.py exists", node_file.exists())

setup_text = (WORKSPACE_ROOT / "src/perception_pipeline/setup.py").read_text()
check("camera_detector entry point in setup.py",
      "camera_detector = perception_pipeline.camera_detector_node:main" in setup_text)

launch_text = (WORKSPACE_ROOT / "src/perception_pipeline/launch/fusion_pipeline.launch.py").read_text()
check("camera_detector in launch file", "camera_detector" in launch_text)

# ═══════════════════════════════════════════════════════════════════════════════
# B. Module imports
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{BOLD}B. Module imports{RESET}")

try:
    from perception_pipeline.camera_detector_node import (
        CameraDetector,
        CameraDetectorNode,
        _imgmsg_to_numpy,
    )
    check("camera_detector_node imports OK", True)
except ImportError as e:
    check("camera_detector_node imports OK", False, str(e))
    print(f"{FAIL}  Cannot continue — aborting.")
    sys.exit(1)

# ═══════════════════════════════════════════════════════════════════════════════
# C. Unit tests — core class (no ROS needed)
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{BOLD}C. CameraDetector unit tests{RESET}")

# C1. Image conversion helper
from sensor_msgs.msg import Image as RosImage

rng = np.random.default_rng(42)
h, w = 375, 1242  # KITTI image dimensions
fake_rgb = rng.integers(0, 256, (h, w, 3), dtype=np.uint8)

ros_img = RosImage()
ros_img.height = h
ros_img.width = w
ros_img.encoding = "rgb8"
ros_img.step = w * 3
ros_img.data = fake_rgb.tobytes()

converted = _imgmsg_to_numpy(ros_img)
check("_imgmsg_to_numpy shape correct", converted.shape == (h, w, 3), str(converted.shape))
check("_imgmsg_to_numpy dtype uint8", converted.dtype == np.uint8, str(converted.dtype))
check("_imgmsg_to_numpy values preserved", np.array_equal(converted, fake_rgb))

# C2. CameraDetector instantiation and inference
print(f"\n  Instantiating CameraDetector (yolov8n.pt — may download ~6 MB on first run)...")
try:
    detector = CameraDetector(model_name="yolov8n.pt", conf_threshold=0.25, device="cpu")
    check("CameraDetector instantiated", True)

    # Run on a blank image — expect zero or more detections, but no crash
    blank = np.zeros((375, 1242, 3), dtype=np.uint8)
    result = detector.detect(blank)
    check("detect() returns a list", isinstance(result, list))

    # Run on a realistic random image
    real_img = rng.integers(0, 256, (375, 1242, 3), dtype=np.uint8)
    result2 = detector.detect(real_img)
    check("detect() on random image returns list", isinstance(result2, list))

    # Validate detection dict schema if any detections present
    if result2:
        d = result2[0]
        required_keys = {"x1", "y1", "x2", "y2", "class_id", "class_name", "confidence"}
        check("detection dict has required keys", required_keys.issubset(d.keys()),
              str(set(d.keys()) - required_keys or "OK"))
        check("x2 > x1", d["x2"] > d["x1"])
        check("y2 > y1", d["y2"] > d["y1"])
        check("confidence in [0, 1]", 0.0 <= d["confidence"] <= 1.0, f"{d['confidence']:.3f}")
        check("class_id is int", isinstance(d["class_id"], int))
        check("class_name is str", isinstance(d["class_name"], str))
        print(f"\n  Sample detection: {d['class_name']} @ conf={d['confidence']:.2f} "
              f"bbox=({d['x1']:.0f},{d['y1']:.0f},{d['x2']:.0f},{d['y2']:.0f})")
    else:
        print(f"  (no detections on random image — OK, model may not fire on noise)")

except Exception as e:
    check("CameraDetector instantiation/inference", False, str(e))

# ═══════════════════════════════════════════════════════════════════════════════
# D. ROS node dry-run
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{BOLD}D. ROS node dry-run{RESET}")

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection2DArray

received = {"detections_2d": 0}

rclpy.init()
try:
    detector_node = CameraDetectorNode()
    check("Node instantiated", True)

    spy_node = rclpy.create_node("spy")
    spy_node.create_subscription(
        Detection2DArray,
        "/detections_2d",
        lambda m: received.__setitem__("detections_2d", received["detections_2d"] + 1),
        10,
    )

    # Publish a test image on /camera/image_raw with best-effort QoS to match subscriber
    pub_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    pub = spy_node.create_publisher(RosImage, "/camera/image_raw", pub_qos)

    test_img = RosImage()
    test_img.height = 375
    test_img.width = 1242
    test_img.encoding = "rgb8"
    test_img.step = 1242 * 3
    test_img.data = np.zeros((375, 1242, 3), dtype=np.uint8).tobytes()
    test_img.header.frame_id = "camera"

    executor = SingleThreadedExecutor()
    executor.add_node(detector_node)
    executor.add_node(spy_node)

    deadline = time.monotonic() + 3.0
    published = False
    while time.monotonic() < deadline and rclpy.ok():
        if not published:
            pub.publish(test_img)
            published = True
        executor.spin_once(timeout_sec=0.05)

    check("Published /detections_2d",
          received["detections_2d"] > 0,
          f"{received['detections_2d']} msgs")

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
    print(f"\033[92m{BOLD}ALL CHECKS PASSED{RESET} — Step 4 camera detector is ready.")
print()
