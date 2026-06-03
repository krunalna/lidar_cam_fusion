#!/usr/bin/env python3
"""
verify_fusion.py
================
Verification script for the fusion node.

Checks:
  1. Build artefacts present (fusion_node_cpp binary, calib_utils library)
  2. GTests pass (test_projector + test_fusion_engine)
  3. ROS dry-run: node starts cleanly with calibration_file param

Run with:
  pixi run verify-fusion
"""

import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD_DIR = os.path.join(ROOT, "build", "perception_pipeline_cpp")
CALIB = os.path.join(ROOT, "src", "perception_pipeline_cpp", "config", "calibration.yaml")
os.makedirs("/tmp/ros_logs", exist_ok=True)
os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")

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


print("\n[1] Build artefacts")

fusion_bin = os.path.join(BUILD_DIR, "fusion_node_cpp")
calib_lib = os.path.join(BUILD_DIR, "libcalib_utils.a")
test_bin = os.path.join(BUILD_DIR, "test_fusion_engine")

check("fusion_node_cpp binary", os.path.isfile(fusion_bin), fusion_bin)
check("libcalib_utils.a", os.path.isfile(calib_lib), calib_lib)
check("test_fusion_engine binary", os.path.isfile(test_bin), test_bin)

print("\n[2] GTests (colcon test — projector + fusion engine)")

try:
    result = subprocess.run(
        ["colcon", "test", "--packages-select", "perception_pipeline_cpp", "--event-handlers", "console_direct+"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=180,
    )
    all_passed = "100% tests passed" in result.stdout
    import re
    passed_match = re.search(r"(\d+)% tests passed", result.stdout)
    total_match = re.search(r"out of (\d+)", result.stdout)
    pct = passed_match.group(1) if passed_match else "?"
    total = total_match.group(1) if total_match else "?"

    check(f"{pct}% tests passed ({total} suites)", all_passed, result.stdout[-300:] if not all_passed else "")
except FileNotFoundError:
    check("colcon available", False, "colcon not on PATH — run inside pixi shell")
except subprocess.TimeoutExpired:
    check("colcon test", False, "timed out after 180 s")

print("\n[3] ROS dry-run (fusion_node_cpp)")

check("calibration.yaml exists", os.path.isfile(CALIB), CALIB)

try:
    proc = subprocess.Popen(
        [fusion_bin, "--ros-args", "-p", f"calibration_file:={CALIB}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    output = ""
    deadline = time.time() + 8.0
    while time.time() < deadline:
        line = proc.stdout.readline()
        if not line:
            break
        output += line
        if "FusionNode ready" in line:
            break

    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()

    ready = "FusionNode ready" in output
    check("node prints 'FusionNode ready'", ready, output[-400:] if not ready else "")
except FileNotFoundError:
    check("fusion_node_cpp runnable", False, "binary not found — build first")

print()
if errors == 0:
    print("\033[32mAll checks passed — fusion node verified.\033[0m")
    sys.exit(0)
else:
    print(f"\033[31m{errors} check(s) failed.\033[0m")
    sys.exit(1)
