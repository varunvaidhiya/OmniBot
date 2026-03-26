from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_ros_gz_sim          = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_omnibot_description = FindPackageShare(package='omnibot_description').find('omnibot_description')
    pkg_omnibot_bringup     = FindPackageShare(package='omnibot_bringup').find('omnibot_bringup')
    pkg_omnibot_arm         = FindPackageShare(package='omnibot_arm').find('omnibot_arm')

    xacro_file       = os.path.join(pkg_omnibot_description, 'urdf', 'omnibot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_omnibot_bringup, 'config', 'omnibot.rviz')
    bridge_config    = os.path.join(pkg_omnibot_bringup, 'config', 'ros_gz_bridge.yaml')
    world_file       = os.path.join(pkg_omnibot_bringup, 'worlds', 'omnibot_world.sdf')
    arm_params       = os.path.join(pkg_omnibot_arm, 'config', 'arm_params.yaml')

    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ── Launch arguments ──────────────────────────────────────────────────────
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 (set false for headless / CI)')

    bev_arg = DeclareLaunchArgument(
        'bev', default_value='true',
        description='Launch BEV stitcher (requires 4 base cameras in Gazebo)')

    rosbridge_arg = DeclareLaunchArgument(
        'rosbridge', default_value='true',
        description='Launch ROSBridge on port 9090 (needed for Android app)')

    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Launch Foxglove bridge on ws://localhost:8765')

    world_arg = DeclareLaunchArgument(
        'world', default_value=world_file,
        description='Path to Gazebo world SDF')

    # ── 1. Gazebo Sim (-r = run unpaused) ────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items(),
    )

    # ── 2. Robot State Publisher ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # ── 3. Spawn robot — wait 3 s for RSP to publish /robot_description ──────
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'mecanum_bot',
                    '-topic', '/robot_description',
                    '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )

    # ── 4. ROS ↔ GZ bridge (/cmd_vel, /odom, /tf, /imu, /scan, /camera/*) ───
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # ── 5. Arm driver — simulation passthrough mode ───────────────────────────
    arm_driver = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='omnibot_arm',
                executable='arm_driver_node.py',
                name='arm_driver_node',
                output='screen',
                parameters=[arm_params],
                remappings=[('/arm/joint_states', '/joint_states')],
            )
        ]
    )

    # ── 6. BEV stitcher — composites 4 base cameras into /camera/base/bev ────
    # Required by smolvla_node; safe to disable with bev:=false for quick tests.
    bev_stitcher = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros2_bev_stitcher',
                executable='bev_stitcher_node',
                name='bev_stitcher_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('bev')),
            )
        ]
    )

    # ── 7. ROSBridge — WebSocket on port 9090 (Android app + web clients) ────
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}],
        condition=IfCondition(LaunchConfiguration('rosbridge')),
    )

    # ── 8. Foxglove bridge — WebSocket on port 8765 (browser visualisation) ──
    # Open https://app.foxglove.dev → Connect → ws://localhost:8765
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765, 'address': '0.0.0.0'}],
        condition=IfCondition(LaunchConfiguration('foxglove')),
    )

    # ── 9. RViz ───────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        bev_arg,
        rosbridge_arg,
        foxglove_arg,
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        arm_driver,
        bev_stitcher,
        rosbridge,
        foxglove_bridge,
        rviz,
    ])
