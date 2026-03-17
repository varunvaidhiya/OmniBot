from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_ros_gz_sim         = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_omnibot_description = FindPackageShare(package='omnibot_description').find('omnibot_description')
    pkg_omnibot_bringup    = FindPackageShare(package='omnibot_bringup').find('omnibot_bringup')
    pkg_omnibot_arm        = FindPackageShare(package='omnibot_arm').find('omnibot_arm')

    xacro_file       = os.path.join(pkg_omnibot_description, 'urdf', 'omnibot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_omnibot_bringup, 'config', 'omnibot.rviz')
    bridge_config    = os.path.join(pkg_omnibot_bringup, 'config', 'ros_gz_bridge.yaml')
    world_file       = os.path.join(pkg_omnibot_bringup, 'worlds', 'omnibot_world.sdf')
    arm_params       = os.path.join(pkg_omnibot_arm, 'config', 'arm_params.yaml')

    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ── 1. Gazebo Sim (-r = run unpaused) ────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
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

    # ── 4. ROS ↔ GZ bridge — use the YAML config (was defined but unused) ────
    # Bridges:  /cmd_vel, /odom, /tf, /imu/data, /camera/*, /joint_states
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # ── 5. Arm driver — simulation passthrough mode ───────────────────────────
    # Receives /arm/joint_commands, stores commanded positions, publishes them.
    # Remapped onto /joint_states so robot_state_publisher gets arm joint angles
    # (wheel joint angles come from Gazebo JointStatePublisher via the bridge).
    # joint_names in arm_params.yaml MUST match the URDF joint names (arm_ prefix).
    arm_driver = TimerAction(
        period=4.0,   # wait for Gazebo + RSP to be fully up
        actions=[
            Node(
                package='omnibot_arm',
                executable='arm_driver_node.py',
                name='arm_driver_node',
                output='screen',
                parameters=[arm_params],
                remappings=[
                    # Feed arm joint angles into /joint_states so robot_state_publisher
                    # can animate the arm in RViz alongside the wheel states from Gazebo.
                    ('/arm/joint_states', '/joint_states'),
                ],
            )
        ]
    )

    # ── 6. RViz ───────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        arm_driver,
        rviz,
    ])
