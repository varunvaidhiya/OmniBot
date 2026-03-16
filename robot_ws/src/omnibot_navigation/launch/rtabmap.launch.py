#!/usr/bin/env python3
"""
rtabmap.launch.py — 3-D RGB-D SLAM for OmniBot using Intel RealSense D435.

Starts:
  1. rtabmap (rtabmap_ros::CoreWrapper) — loop-closure + OctoMap
  2. octomap_server — converts RTAB-Map point cloud → projected 2-D map

Topic layout (after remapping):
  /camera/depth/image        ← Gazebo / real RealSense depth image
  /camera/depth/camera_info  ← intrinsics
  /camera/color/image_raw    ← (optional) RGB for richer loop closure
  /odom                      ← odometry from EKF / driver
  /rtabmap/cloud_map         → PointCloud2 (consumed by octomap_server)
  /rtabmap/octomap_binary    → OctoMap (binary)
  /rtabmap/octomap_full      → OctoMap (full probability)
  /projected_map             → OccupancyGrid (Nav2-compatible 2-D slice)

Usage:
  ros2 launch omnibot_navigation rtabmap.launch.py
  ros2 launch omnibot_navigation rtabmap.launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory('omnibot_navigation')
    rtabmap_params = os.path.join(nav_dir, 'config', 'rtabmap_params.yaml')
    octomap_params = os.path.join(nav_dir, 'config', 'octomap_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # ── RTAB-Map core node ────────────────────────────────────────────────────
    # rtabmap_ros provides a single executable that wraps the RTAB-Map library.
    # Remappings translate generic RTAB-Map topic names to OmniBot's camera topics.
    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            # Depth + camera info from RealSense (Gazebo RGBD sensor)
            ('rgb/image',       '/camera/depth/image'),
            ('rgb/camera_info', '/camera/depth/camera_info'),
            ('depth/image',     '/camera/depth/image'),
            # Odometry from EKF node
            ('odom',            '/odom'),
            # Output map
            ('grid_map',        '/rtabmap/grid_map'),
        ],
        arguments=['--delete_db_on_start'],   # remove to preserve map across restarts
    )

    # ── RTAB-Map visualisation node (optional, for RViz) ─────────────────────
    rtabmapviz_node = Node(
        package='rtabmap_ros',
        executable='rtabmapviz',
        name='rtabmapviz',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'subscribe_depth': True},
            {'frame_id': 'base_link'},
        ],
        remappings=[
            ('rgb/image',       '/camera/depth/image'),
            ('rgb/camera_info', '/camera/depth/camera_info'),
            ('depth/image',     '/camera/depth/image'),
        ],
        # Comment out the condition below to enable the RTAB-Map GUI
        # condition=IfCondition(LaunchConfiguration('use_rtabmapviz')),
    )

    # ── OctoMap server ────────────────────────────────────────────────────────
    # Subscribes to /rtabmap/cloud_map (PointCloud2 from RTAB-Map) and builds
    # a 3-D OctoMap + projected 2-D occupancy grid.
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            octomap_params,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cloud_in', '/rtabmap/cloud_map'),
        ],
    )

    # ── depth_image_proc: convert depth image → PointCloud2 for Nav2 ─────────
    # Nav2 VoxelLayer reads PointCloud2 directly.
    # RTAB-Map's cloud_map is a full accumulated map; for local obstacle
    # avoidance we also publish a per-frame point cloud from the sensor.
    depth_to_pointcloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='depth_to_pointcloud',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('image_rect',   '/camera/depth/image'),
            ('camera_info',  '/camera/depth/camera_info'),
            ('points',       '/camera/depth/points'),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(rtabmap_node)
    ld.add_action(octomap_server_node)
    ld.add_action(depth_to_pointcloud_node)
    # ld.add_action(rtabmapviz_node)  # uncomment to open RTAB-Map GUI

    return ld
