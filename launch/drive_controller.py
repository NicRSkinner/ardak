import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription()

    container = ComposableNodeContainer(
        name='launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ardak',
                plugin='bfr::DriveControllerNode',
                name='DriveControllerNode',
                parameters=[
                    {"gamepadEquipped": True}
                ]
            )
        ],
        output="screen"
    )

    ld.add_action(container)

    return ld
