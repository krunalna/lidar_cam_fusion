"""
fusion_pipeline.launch.py
=========================
Launches all nodes in the LiDAR-Camera fusion pipeline.

Environment variables (override defaults):
  KITTI_SEQ   Path to the KITTI raw sync sequence directory (required)
  FRAME_RATE  Playback rate in Hz (default: 10.0)
  LOOP        Loop the sequence: true/false (default: true)

Example:
  KITTI_SEQ=/data/kitti/2011_09_26/2011_09_26_drive_0001_sync pixi run launch
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Arguments (can be overridden on CLI or via env vars) ─────────────────
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

    # ── KITTI publisher (Step 2) ───────────────────────────────────────────────
    kitti_publisher = Node(
        package="perception_pipeline",
        executable="kitti_publisher",
        name="kitti_publisher",
        parameters=[{
            "sequence_path": LaunchConfiguration("sequence_path"),
            "frame_rate": LaunchConfiguration("frame_rate"),
            "loop": LaunchConfiguration("loop"),
        }],
        output="screen",
        emulate_tty=True,
    )

    # ── LiDAR processor (Step 3) ──────────────────────────────────────────────
    lidar_processor = Node(
        package="perception_pipeline",
        executable="lidar_processor",
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

    # ── Camera detector (Step 4) ──────────────────────────────────────────────
    camera_detector = Node(
        package="perception_pipeline",
        executable="camera_detector",
        name="camera_detector",
        parameters=[{
            "model_name":     "yolov8n.pt",
            "conf_threshold":  0.5,
            "device":         "cpu",
        }],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        seq_arg,
        rate_arg,
        loop_arg,
        LogInfo(msg="Starting LiDAR-Camera Fusion Pipeline (KITTI playback)"),
        kitti_publisher,
        lidar_processor,
        camera_detector,
    ])
