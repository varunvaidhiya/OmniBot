import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Package share directories
    pkg_omnibot_bringup = get_package_share_directory('omnibot_bringup')
    pkg_omnibot_arm = get_package_share_directory('omnibot_arm')

    # Config files
    arm_params = os.path.join(pkg_omnibot_arm, 'config', 'arm_params.yaml')

    # ------------------------------------------------------------------
    # 1. Base robot (yahboom driver + robot state publisher)
    # ------------------------------------------------------------------
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_omnibot_bringup, 'launch', 'robot.launch.py')
        )
    )

    # ------------------------------------------------------------------
    # 2. ARM driver (delayed 2s to let base come up first)
    # ------------------------------------------------------------------
    arm_driver_node = Node(
        package='omnibot_arm',
        executable='arm_driver_node.py',
        name='arm_driver_node',
        output='screen',
        parameters=[arm_params],
    )

    arm_driver_delayed = TimerAction(
        period=2.0,
        actions=[arm_driver_node],
    )

    # ------------------------------------------------------------------
    # 3. Front camera — /dev/video0
    # ------------------------------------------------------------------
    front_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='front_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'yuyv',
            'camera_name': 'front_camera',
            'camera_frame_id': 'front_camera_link',
        }],
        remappings=[
            ('image_raw', '/camera/front/image_raw'),
            ('camera_info', '/camera/front/camera_info'),
        ],
    )

    # ------------------------------------------------------------------
    # 4. Wrist camera — /dev/video2
    # ------------------------------------------------------------------
    wrist_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='wrist_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video2',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'yuyv',
            'camera_name': 'wrist_camera',
            'camera_frame_id': 'wrist_camera_link',
        }],
        remappings=[
            ('image_raw', '/camera/wrist/image_raw'),
            ('camera_info', '/camera/wrist/camera_info'),
        ],
    )

    return LaunchDescription([
        robot_launch,
        arm_driver_delayed,
        front_camera_node,
        wrist_camera_node,
    ])
