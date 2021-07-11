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
                    {"gamepadEquipped": True},
                    {"maxVelocity": 16093.0},
                    {"minVelocity": 0.0},
                    {"driveGearRatio": 0.1},
                    {"wheelCircumference": 57.026},
                    {"steeringGearRatio": 0.667},
                    {"maxSteeringAngle": 45.0},
                    {"minSteeringAngle": -45.0}
                ],
                remappings=[
                    ("appout/drive/output_command", "odrive0/motor0/input_vel"),
                    ("appout/steering/output_command", "odrive0/motor1/input_pos")
                ]
            )
        ],
        output="screen"
    )

    ld.add_action(container)

    return ld
