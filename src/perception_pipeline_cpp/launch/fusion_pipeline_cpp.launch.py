"""
fusion_pipeline_cpp.launch.py
==============================
Launches the C++ LiDAR-Camera fusion pipeline nodes.

Environment variables:
  KITTI_SEQ   Path to the KITTI raw sync sequence directory (required)
  FRAME_RATE  Playback rate in Hz (default: 10.0)
  LOOP        Loop the sequence: true/false (default: true)
  YOLO_ONNX   Absolute path to yolov8n.onnx (required for camera_detector_cpp)

Example:
  # 1. Export the ONNX model once
  pixi run export-onnx

  # 2. Launch
  KITTI_SEQ=$(pwd)/data/kitti/2011_09_26/2011_09_26_drive_0001_sync \\
  YOLO_ONNX=$(pwd)/models/yolov8n.onnx \\
  pixi run launch
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    seq_arg = DeclareLaunchArgument(
        "sequence_path",
        default_value=os.environ.get("KITTI_SEQ", ""),
        description="Path to KITTI raw sync sequence directory",
    )
    rate_arg = DeclareLaunchArgument(
        "frame_rate",
        default_value=os.environ.get("FRAME_RATE", "10.0"),
        description="Playback rate in Hz",
    )
    loop_arg = DeclareLaunchArgument(
        "loop",
        default_value=os.environ.get("LOOP", "true"),
        description="Loop the sequence when it ends",
    )
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value=os.environ.get("YOLO_ONNX", ""),
        description="Absolute path to yolov8n.onnx (export with: pixi run export-onnx)",
    )

    _calib_default = os.environ.get(
        "CALIB_FILE",
        os.path.join(
            get_package_share_directory("perception_pipeline_cpp"),
            "config", "calibration.yaml"))
    calib_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=_calib_default,
        description="Path to KITTI calibration YAML",
    )

    # ── KITTI publisher (Phase 2) ─────────────────────────────────────────────
    kitti_publisher = Node(
        package="perception_pipeline_cpp",
        executable="kitti_publisher",
        name="kitti_publisher",
        parameters=[{
            "sequence_path": LaunchConfiguration("sequence_path"),
            "frame_rate":    LaunchConfiguration("frame_rate"),
            "loop":          LaunchConfiguration("loop"),
        }],
        output="screen",
        emulate_tty=True,
    )

    # ── LiDAR processor (Phase 3 — C++) ──────────────────────────────────────
    lidar_processor = Node(
        package="perception_pipeline_cpp",
        executable="lidar_processor_cpp",
        name="lidar_processor",
        parameters=[{
            "roi_x_min":   0.0,
            "roi_x_max":  50.0,
            "roi_y_min": -10.0,
            "roi_y_max":  10.0,
            "roi_z_min":  -3.0,
            "roi_z_max":   2.0,
            "voxel_size":  0.1,
            "ransac_dist": 0.2,
            "ransac_iter": 100,
            "max_depth":  50.0,
        }],
        output="screen",
        emulate_tty=True,
    )

    # ── Camera detector (Phase 4 — C++, ONNX Runtime) ────────────────────────
    camera_detector = Node(
        package="perception_pipeline_cpp",
        executable="camera_detector_cpp",
        name="camera_detector",
        parameters=[{
            "model_path":     LaunchConfiguration("model_path"),
            "conf_threshold": 0.5,
            "iou_threshold":  0.45,
        }],
        output="screen",
        emulate_tty=True,
    )

    # ── Fusion node (Phase 6 — C++) ──────────────────────────────────────────
    fusion_node = Node(
        package="perception_pipeline_cpp",
        executable="fusion_node_cpp",
        name="fusion_node",
        parameters=[{
            "calibration_file":    LaunchConfiguration("calibration_file"),
            "min_cluster_points":  5,
            "sync_slop":           0.5,
            "publish_markers":     True,
            "publish_debug_image": True,
        }],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        seq_arg,
        rate_arg,
        loop_arg,
        model_arg,
        calib_arg,
        LogInfo(msg="Starting LiDAR-Camera Fusion Pipeline — C++ (KITTI playback)"),
        kitti_publisher,
        lidar_processor,
        camera_detector,
        fusion_node,
    ])
