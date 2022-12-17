from email.policy import default
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(name="localization", default_value="False"),
        DeclareLaunchArgument(name="robot_state", default_value="False"),
        DeclareLaunchArgument(name="joint_state", default_value="False")
    ]

    drive_control_config_path = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'DriveController.yaml'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    localization_config_path = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'localization.yaml'
    )

    ardak_nodes = Node(
        package="ardak",
        executable="main",
        parameters=[drive_control_config_path],
        remappings=[
            ("appout/drive/left_drive_command", "odrive0/motor0/input_vel"),
            ("appout/drive/right_drive_command", "odrive0/motor1/input_vel")
        ]
    )

    gamepad = Node(
        package="bfr_hal",
        executable="gamepad.py"
    )

    odrive = Node(
        package="odrive_ros2",
        executable="odrive"
    )

    return LaunchDescription(launch_args + [
        ardak_nodes,
        gamepad,
        odrive
    ])
