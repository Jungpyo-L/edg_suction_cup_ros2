#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_plotjuggler(context):
    launch_plot = LaunchConfiguration("launch_plot")
    plotjuggler_layout = LaunchConfiguration("plotjuggler_layout").perform(context)
    arguments = []

    if plotjuggler_layout and os.path.exists(plotjuggler_layout):
        arguments = ["--layout", plotjuggler_layout]

    return [
        Node(
            package="plotjuggler",
            executable="plotjuggler",
            name="plotjuggler",
            output="screen",
            condition=IfCondition(launch_plot),
            arguments=arguments,
        )
    ]


def generate_launch_description():
    ati_ip = LaunchConfiguration("ati_ip")
    robot_ip = LaunchConfiguration("robot_ip")
    default_plotjuggler_layout = os.path.join(
        get_package_share_directory("suction_cup"),
        "config",
        "PlotJuggler_default_layout.xml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("ati_ip", default_value="192.168.1.42"),
        DeclareLaunchArgument("launch_plot", default_value="true"),
        DeclareLaunchArgument("plotjuggler_layout", default_value=default_plotjuggler_layout),
        DeclareLaunchArgument("robot_ip", default_value="10.0.0.1"),
        Node(
            package="suction_cup",
            executable="robotStatePublisher.py",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_ip": robot_ip}],
        ),
        Node(
            package="edg_netft_ros2",
            executable="netft_node",
            name="netft_node",
            output="screen",
            arguments=[ati_ip],
        ),
        Node(
            package="suction_cup",
            executable="data_logger.py",
            name="data_logger",
            output="screen",
        ),
        OpaqueFunction(function=launch_plotjuggler),
    ])
