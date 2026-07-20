import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_omnibot_arm = get_package_share_directory("omnibot_arm")
    arm_params = os.path.join(pkg_omnibot_arm, "config", "arm_params.yaml")

    # In OmniBot Open there is no arm_cmd_mux (that lives in the closed
    # omnibot_rl package), so relay /arm/joint_commands → /arm/joint_commands/out
    # by default. Set direct_command:=false when running the full command mux.
    direct_command = LaunchConfiguration("direct_command")
    declare_direct_command = DeclareLaunchArgument(
        "direct_command",
        default_value="true",
        description="Relay /arm/joint_commands to the driver input "
        "(disable when using the full command mux).",
    )

    arm_driver_node = Node(
        package="omnibot_arm",
        executable="arm_driver_node.py",
        name="arm_driver_node",
        output="screen",
        parameters=[arm_params],
    )

    arm_command_relay = Node(
        package="omnibot_arm",
        executable="arm_command_relay.py",
        name="arm_command_relay",
        output="screen",
        condition=IfCondition(direct_command),
    )

    return LaunchDescription(
        [
            declare_direct_command,
            arm_driver_node,
            arm_command_relay,
        ]
    )
