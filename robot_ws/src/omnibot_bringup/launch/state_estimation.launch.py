#!/usr/bin/env python3
"""
state_estimation.launch.py — robot_localization EKF for OmniBot.

Fuses /odom (wheel dead-reckoning) + /imu/data (Yahboom IMU) into:
  /odometry/filtered   (fused pose + twist)
  TF: odom → base_link (EKF owns this — driver runs with publish_tf:=false)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("omnibot_bringup")
    ekf_config = os.path.join(pkg_bringup, "config", "robot_localization.yaml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("odometry/filtered", "/odometry/filtered"),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_node,
    ])
