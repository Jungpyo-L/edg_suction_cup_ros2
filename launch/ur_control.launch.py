#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")

    ur_driver_launch = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),
        "launch",
        "ur_control.launch.py",
    ])
    ur_moveit_launch = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"),
        "launch",
        "ur_moveit.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="10.0.0.1"),
        DeclareLaunchArgument("ur_type", default_value="ur10"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("headless_mode", default_value="true"),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(ur_driver_launch),
                    launch_arguments={
                        "ur_type": ur_type,
                        "robot_ip": robot_ip,
                        "launch_rviz": "false",
                        "headless_mode": headless_mode,
                    }.items(),
                ),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_moveit_launch),
            launch_arguments={
                "ur_type": ur_type,
                "launch_rviz": launch_rviz,
            }.items(),
        ),
    ])
