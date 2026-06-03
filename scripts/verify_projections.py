#!/usr/bin/env python3
"""
verify_projections.py
=====================
Verification script for calibration and projection math.

Checks:
  1. calibration.yaml exists and has the required fields
  2. Python reference projection math (validates the same logic as C++)
  3. C++ build succeeds (calib_utils + test_projector binary present)
  4. GTests all pass (colcon test)

Run with:
  pixi run verify-projections
"""

import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CALIB_YAML = os.path.join(ROOT, "src", "perception_pipeline_cpp", "config", "calibration.yaml")
BUILD_DIR = os.path.join(ROOT, "build", "perception_pipeline_cpp")

PASS = "\033[32m✓\033[0m"
FAIL = "\033[31m✗\033[0m"

errors = 0


def check(label: str, ok: bool, detail: str = "") -> None:
    global errors
    if ok:
        print(f"  {PASS} {label}")
    else:
        print(f"  {FAIL} {label}" + (f": {detail}" if detail else ""))
        errors += 1


print("\n[1] calibration.yaml sanity")

check("file exists", os.path.isfile(CALIB_YAML), CALIB_YAML)

try:
    import yaml
    with open(CALIB_YAML) as handle:
        cal = yaml.safe_load(handle)

    cam = cal.get("camera", {})
    check("camera.fx present", "fx" in cam)
    check("camera.fy present", "fy" in cam)
    check("camera.cx present", "cx" in cam)
    check("camera.cy present", "cy" in cam)
    check("camera.width present", "width" in cam)
    check("camera.height present", "height" in cam)
    check("camera.P has 12 elements", len(cam.get("P", [])) == 12, f"got {len(cam.get('P', []))}")

    transform = cal.get("lidar_to_camera", {}).get("T", [])
    check("lidar_to_camera.T has 16 elements", len(transform) == 16, f"got {len(transform)}")
    check(
        "T last row is [0,0,0,1]",
        abs(transform[12]) < 1e-9 and abs(transform[13]) < 1e-9 and
        abs(transform[14]) < 1e-9 and abs(transform[15] - 1.0) < 1e-9,
    )
except ImportError:
    print("  (yaml not importable — skipping YAML parse checks)")
except Exception as exc:
    check("YAML parse", False, str(exc))

print("\n[2] Python reference projection math")

try:
    import yaml
    with open(CALIB_YAML) as handle:
        cal = yaml.safe_load(handle)

    cam = cal["camera"]
    transform = cal["lidar_to_camera"]["T"]

    fx, fy = cam["fx"], cam["fy"]
    cx, cy = cam["cx"], cam["cy"]
    width, height = cam["width"], cam["height"]

    x, y, z = 10.0, 0.0, 0.0
    xc = transform[0] * x + transform[1] * y + transform[2] * z + transform[3]
    yc = transform[4] * x + transform[5] * y + transform[6] * z + transform[7]
    zc = transform[8] * x + transform[9] * y + transform[10] * z + transform[11]

    check("Zc > 0 (point in front of camera)", zc > 0, f"Zc={zc:.4f}")

    u = fx * (xc / zc) + cx
    v = fy * (yc / zc) + cy
    in_bounds = 0 <= u < width and 0 <= v < height
    check(f"projects inside image bounds  u={u:.2f}, v={v:.2f}", in_bounds, f"image is {width}×{height}")
    check("u near 614.85 (±1px)", abs(u - 614.85) < 1.0, f"u={u:.4f}")
    check("v near 178.17 (±1px)", abs(v - 178.17) < 1.0, f"v={v:.4f}")
except Exception as exc:
    check("projection math", False, str(exc))

print("\n[3] C++ build artefacts")

calib_lib = os.path.join(BUILD_DIR, "libcalib_utils.a")
test_bin = os.path.join(BUILD_DIR, "test_projector")

check("libcalib_utils.a built", os.path.isfile(calib_lib), calib_lib)
check("test_projector binary built", os.path.isfile(test_bin), test_bin)

print("\n[4] GTests (colcon test)")

try:
    result = subprocess.run(
        ["colcon", "test", "--packages-select", "perception_pipeline_cpp", "--event-handlers", "console_direct+"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=180,
    )
    passed = "100% tests passed" in result.stdout or ("PASSED" in result.stdout and "FAILED" not in result.stdout)
    check("Projection-related GTests pass", passed, result.stdout[-500:] if not passed else "")
except FileNotFoundError:
    check("colcon available", False, "colcon not on PATH — run inside pixi shell")
except subprocess.TimeoutExpired:
    check("colcon test timeout", False, "timed out after 180s")

print()
if errors == 0:
    print("\033[32mAll checks passed — calibration and projection verified.\033[0m")
    sys.exit(0)
else:
    print(f"\033[31m{errors} check(s) failed.\033[0m")
    sys.exit(1)
