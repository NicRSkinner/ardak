import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription()

    drive_control_config_path = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'DriveController.yaml'
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

    ld.add_action(ardak_nodes)
    ld.add_action(gamepad)
    ld.add_action(odrive)

    return ld
