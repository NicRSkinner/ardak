import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription()

    ardak_nodes = Node(
        package="ardak",
        executable="main",
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
