from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_omnibot_description = FindPackageShare(package='omnibot_description').find('omnibot_description')
    pkg_omnibot_bringup = FindPackageShare(package='omnibot_bringup').find('omnibot_bringup')

    xacro_file = os.path.join(pkg_omnibot_description, 'urdf', 'omnibot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_omnibot_bringup, 'config', 'omnibot.rviz')
    bridge_config = os.path.join(pkg_omnibot_bringup, 'config', 'ros_gz_bridge.yaml')

    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 1. Gazebo Sim — -r means "run" (not paused)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. Robot State Publisher — must start before spawn
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. Spawn Entity — wait 3s for RSP to publish /robot_description
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

    # 4. ROS <-> GZ Bridge — use @ (bidirectional) format, confirmed working
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        output='screen'
    )

    # 5. Robot State Publisher
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
        rviz
    ])
