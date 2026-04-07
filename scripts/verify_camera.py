"""
Camera Detector verify script
=============================
Verifies the camera_detector_cpp node: source files, build artifact,
ONNX model presence, and a live ROS dry-run against the compiled binary.

Run with: pixi run verify-camera
"""

import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

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

pkg = WORKSPACE_ROOT / "src/perception_pipeline_cpp"

check("camera_detector.hpp exists",
      (pkg / "include/perception_pipeline_cpp/camera_detector.hpp").exists())
check("camera_detector.cpp exists", (pkg / "src/camera_detector.cpp").exists())
check("camera_detector_node.cpp exists", (pkg / "src/camera_detector_node.cpp").exists())

cmake_text = (pkg / "CMakeLists.txt").read_text()
check("CMakeLists.txt has camera_detector_cpp target", "camera_detector_cpp" in cmake_text)
check("CMakeLists.txt finds onnxruntime", "onnxruntime" in cmake_text)
check("CMakeLists.txt finds vision_msgs", "vision_msgs" in cmake_text)

launch_text = (pkg / "launch/fusion_pipeline_cpp.launch.py").read_text()
check("launch file references camera_detector_cpp", "camera_detector_cpp" in launch_text)
check("launch file passes model_path parameter", "model_path" in launch_text)

print(f"\n{BOLD}B. Build artifact{RESET}")

binary = (
    WORKSPACE_ROOT /
    "install/perception_pipeline_cpp/lib/perception_pipeline_cpp/camera_detector_cpp"
)
check("camera_detector_cpp binary exists", binary.exists(), str(binary))

if not binary.exists():
    print(f"{FAIL}  Binary not found — run: pixi run build")
    print(f"\n{'─' * 50}")
    print(f"\033[91m{BOLD}FAILED{RESET} — binary missing, cannot run dry-run.")
    sys.exit(1)

print(f"\n{BOLD}C. ONNX model{RESET}")

default_model = WORKSPACE_ROOT / "models" / "yolov8n.onnx"
model_path = Path(os.environ.get("YOLO_ONNX", str(default_model)))

model_ok = model_path.exists()
check("ONNX model exists", model_ok, str(model_path), warn_only=not model_ok)

if not model_ok:
    print(f"{WARN}  Model not found — export it first:")
    print("       pixi run export-onnx")
    print(f"\n{'─' * 50}")
    print(f"\033[93m{BOLD}SKIPPED dry-run{RESET} — ONNX model missing.")
    print("Run 'pixi run export-onnx' then re-run this script.")
    sys.exit(0)

print(f"\n{BOLD}D. ROS dry-run{RESET}")

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

IMG_W, IMG_H = 1242, 375
synthetic_rgb = np.ones((IMG_H, IMG_W, 3), dtype=np.uint8) * 128

received = {"detections": 0}
cpp_proc = None

rclpy.init()
try:
    qos_sub = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )
    qos_pub = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )

    spy = rclpy.create_node("verify_camera_spy")

    def on_detections(_msg):
        received["detections"] += 1

    spy.create_subscription(Detection2DArray, "/detections_2d", on_detections, qos_pub)
    pub = spy.create_publisher(Image, "/camera/image_raw", qos_sub)

    img_msg = Image()
    img_msg.height = IMG_H
    img_msg.width = IMG_W
    img_msg.encoding = "rgb8"
    img_msg.is_bigendian = False
    img_msg.step = IMG_W * 3
    img_msg.data = synthetic_rgb.tobytes()

    env = os.environ.copy()
    env["CUDA_VISIBLE_DEVICES"] = ""
    setup_bash = WORKSPACE_ROOT / "install/setup.bash"
    if setup_bash.exists():
        result = subprocess.run(
            ["bash", "-c", f"source {setup_bash} && env"],
            capture_output=True,
            text=True,
        )
        for line in result.stdout.splitlines():
            if "=" in line:
                key, _, value = line.partition("=")
                env[key] = value

    cpp_proc = subprocess.Popen(
        [
            str(binary),
            "--ros-args",
            "-p", f"model_path:={model_path}",
            "-p", "conf_threshold:=0.3",
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    check("C++ node process launched", cpp_proc.poll() is None)

    executor = SingleThreadedExecutor()
    executor.add_node(spy)

    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)

    deadline = time.monotonic() + 4.0
    n_published = 0
    while time.monotonic() < deadline and rclpy.ok():
        if n_published < 3:
            img_msg.header.stamp = spy.get_clock().now().to_msg()
            pub.publish(img_msg)
            n_published += 1
        executor.spin_once(timeout_sec=0.05)
        if received["detections"] > 0:
            break

    check("camera_detector_cpp published /detections_2d",
          received["detections"] > 0,
          f"{received['detections']} msg(s) after {n_published} frame(s) sent")

    check("C++ node still alive after inference",
          cpp_proc.poll() is None,
          f"exit code: {cpp_proc.poll()}")

    if cpp_proc.poll() is not None:
        stderr = cpp_proc.stderr.read().decode(errors="replace")
        if stderr.strip():
            print(f"\n  Node stderr:\n{stderr[:800]}")

except Exception as exc:
    check("ROS dry-run", False, str(exc))
    import traceback
    traceback.print_exc()
finally:
    if cpp_proc and cpp_proc.poll() is None:
        cpp_proc.terminate()
        try:
            cpp_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            cpp_proc.kill()
    if rclpy.ok():
        rclpy.shutdown()

print(f"\n{'─' * 50}")
if failures:
    print(f"\033[91m{BOLD}FAILED{RESET} — {len(failures)} check(s) not passing:")
    for failure in failures:
        print(f"  • {failure}")
    sys.exit(1)
else:
    print(f"\033[92m{BOLD}ALL CHECKS PASSED{RESET} — Camera detector is ready.")
print()
