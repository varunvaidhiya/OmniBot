#!/usr/bin/env python3
"""
Hybrid Robot Launch — Nav2 + VLA with cmd_vel mux and mission planner.

┌──────────────────────────────────────────────────────────────────────┐
│  MISSION LAYER                                                        │
│  /mission/command  →  mission_planner  →  /control_mode + /vla/prompt│
└──────────────────────────────────────┬───────────────────────────────┘
                                       │
          ┌────────────────────────────▼────────────────────────────┐
          │                     cmd_vel_mux                          │
          │  /cmd_vel       ← Nav2 (native output)                  │
          │  /cmd_vel/vla   ← VLA inference node                    │
          │  /cmd_vel/teleop← keyboard / joystick                   │
          │                     selects by /control_mode            │
          │                   → /cmd_vel/out                        │
          └────────────────────────────┬────────────────────────────┘
                                       │
          ┌────────────────────────────▼────────────────────────────┐
          │  yahboom_driver  (remapped: /cmd_vel ← /cmd_vel/out)    │
          └─────────────────────────────────────────────────────────┘

Launching this file starts the complete hybrid system:
  • Robot driver        (RPi 5 — Yahboom board)
  • Robot state publisher + EKF localization
  • SLAM Toolbox        (live mapping)
  • Nav2 autonomy       (path planning + obstacle avoidance)
  • VLA inference node  (Desktop GPU — semantic tasks)
  • cmd_vel_mux         (mode-based multiplexer)
  • mission_planner     (high-level orchestrator)
  • RViz                (optional visualisation)

Quick-start after launch
────────────────────────
  # Full hybrid mission
  ros2 topic pub --once /mission/command std_msgs/msg/String \\
    "data: 'navigate:kitchen,vla:find the red cup'"

  # Force a specific mode
  ros2 topic pub --once /control_mode std_msgs/msg/String "data: 'vla'"
  ros2 topic pub --once /control_mode std_msgs/msg/String "data: 'nav2'"
  ros2 topic pub --once /control_mode std_msgs/msg/String "data: 'teleop'"

  # Cancel running mission
  ros2 topic pub --once /mission/cancel std_msgs/msg/String "data: 'cancel'"

  # Watch status
  ros2 topic echo /mission/status
  ros2 topic echo /control_mode/active
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_hybrid     = get_package_share_directory('omnibot_hybrid')
    pkg_navigation = get_package_share_directory('omnibot_navigation')
    pkg_description = get_package_share_directory('omnibot_description')

    xacro_file = os.path.join(pkg_description, 'urdf', 'omnibot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz     = LaunchConfiguration('use_rviz',     default='true')
    use_slam     = LaunchConfiguration('use_slam',     default='true')
    vla_device   = LaunchConfiguration('vla_device',   default='cuda')
    vla_4bit     = LaunchConfiguration('vla_4bit',     default='false')

    # ── Robot driver ──────────────────────────────────────────────────────────
    # Remapped: driver reads /cmd_vel/out (mux output) instead of /cmd_vel.
    # This keeps Nav2 writing to /cmd_vel natively, while the mux selects
    # the correct source and forwards it to /cmd_vel/out.
    driver_node = Node(
        package='omnibot_driver',
        executable='yahboom_controller_node.py',
        name='yahboom_driver',
        output='screen',
        parameters=[{
            'serial_port':             '/dev/ttyUSB0',
            'baud_rate':               115200,
            'wheel_separation_length': 0.165,
            'wheel_separation_width':  0.215,
            'wheel_radius':            0.04,
            'use_sim_time':            use_sim_time,
        }],
        remappings=[('/cmd_vel', '/cmd_vel/out')],
    )

    # ── Robot state publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time':      use_sim_time,
        }],
    )

    # ── EKF localisation ──────────────────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_navigation, 'config', 'robot_localization.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── SLAM Toolbox ──────────────────────────────────────────────────────────
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'slam_toolbox.launch.py')),
        condition=IfCondition(use_slam),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── Nav2 (outputs to /cmd_vel natively) ───────────────────────────────────
    # The mux subscribes to /cmd_vel as the "nav2" source.
    autonomous_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_navigation, 'launch', 'autonomous_navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  os.path.join(
                pkg_navigation, 'config', 'nav2_params.yaml'),
        }.items(),
    )

    # ── VLA node (publishes to /cmd_vel/vla) ──────────────────────────────────
    vla_node = Node(
        package='omnibot_vla',
        executable='vla_node',
        name='vla_node',
        output='screen',
        parameters=[{
            'model_path':   'openvla/openvla-7b',
            'device':       vla_device,
            'load_in_4bit': vla_4bit,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── cmd_vel Mux ───────────────────────────────────────────────────────────
    # Subscribes: /cmd_vel (nav2), /cmd_vel/vla, /cmd_vel/teleop
    # Publishes:  /cmd_vel/out  →  driver
    cmd_vel_mux_node = Node(
        package='omnibot_hybrid',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{
            'default_mode': 'nav2',
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Mission Planner ───────────────────────────────────────────────────────
    mission_planner_node = Node(
        package='omnibot_hybrid',
        executable='mission_planner',
        name='mission_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── RViz (optional) ───────────────────────────────────────────────────────
    rviz_config = os.path.join(
        pkg_description, 'config', 'omnibot_navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # ── Arguments ─────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use Gazebo simulation clock'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz for visualisation'),
        DeclareLaunchArgument(
            'use_slam', default_value='true',
            description='Launch SLAM Toolbox for live mapping'),
        DeclareLaunchArgument(
            'vla_device', default_value='cuda',
            description='PyTorch device for VLA inference (cuda / cpu)'),
        DeclareLaunchArgument(
            'vla_4bit', default_value='false',
            description='Load VLA model in 4-bit quantisation (needs <16 GB VRAM)'),

        # ── Nodes ─────────────────────────────────────────────────────────────
        driver_node,
        robot_state_publisher,
        ekf_node,
        slam_toolbox,
        autonomous_navigation,
        vla_node,
        cmd_vel_mux_node,
        mission_planner_node,
        rviz_node,
    ])
