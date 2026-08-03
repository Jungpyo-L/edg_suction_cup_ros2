#!/usr/bin/env python3

# Brings up the Intel RealSense camera, the camera-to-robot extrinsic, and the
# point cloud filter that publishes filtered_points and sphere_apex.

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_camera = LaunchConfiguration("launch_camera")
    camera_frame = LaunchConfiguration("camera_frame")
    base_frame = LaunchConfiguration("base_frame")

    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument("launch_camera", default_value="true"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),
        DeclareLaunchArgument("input_topic", default_value="/camera/camera/depth/color/points"),
        # Camera extrinsic: replace these with your calibrated hand-eye result.
        # Translation is meters, rotation is roll pitch yaw in radians, both
        # expressed as base_frame -> camera_frame.
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.6"),
        DeclareLaunchArgument("camera_roll", default_value="3.1416"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        # Workspace crop box in base_frame, meters. To pass every point through
        # for alignment debugging, see the passthrough defaults in the README.
        DeclareLaunchArgument("crop_min", default_value="[0.40, -0.35, -0.02]"),
        DeclareLaunchArgument("crop_max", default_value="[0.75, 0.10, 0.30]"),
        DeclareLaunchArgument("voxel_leaf", default_value="0.002"),
        # 0 disables outlier removal, which is the expensive stage on a full cloud.
        DeclareLaunchArgument("outlier_neighbors", default_value="12"),
        DeclareLaunchArgument("min_points", default_value="50"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            condition=IfCondition(launch_camera),
            launch_arguments={
                "pointcloud.enable": "true",
                "align_depth.enable": "true",
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_extrinsic",
            output="screen",
            arguments=[
                "--x", LaunchConfiguration("camera_x"),
                "--y", LaunchConfiguration("camera_y"),
                "--z", LaunchConfiguration("camera_z"),
                "--roll", LaunchConfiguration("camera_roll"),
                "--pitch", LaunchConfiguration("camera_pitch"),
                "--yaw", LaunchConfiguration("camera_yaw"),
                "--frame-id", base_frame,
                "--child-frame-id", camera_frame,
            ],
        ),
        Node(
            package="suction_cup",
            executable="realsense_sphere_detector.py",
            name="realsense_sphere_detector",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "target_frame": base_frame,
                "crop_min": ParameterValue(LaunchConfiguration("crop_min"), value_type=List[float]),
                "crop_max": ParameterValue(LaunchConfiguration("crop_max"), value_type=List[float]),
                "voxel_leaf": ParameterValue(LaunchConfiguration("voxel_leaf"), value_type=float),
                "outlier_neighbors": ParameterValue(
                    LaunchConfiguration("outlier_neighbors"), value_type=int),
                "min_points": ParameterValue(LaunchConfiguration("min_points"), value_type=int),
            }],
        ),
    ])
