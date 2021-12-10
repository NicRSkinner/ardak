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
            #ComposableNode(
            #    package='ardak',
            #    plugin='bfr::InputCalibrationNode',
            #    name='InputCalibrationNode'
            #),
            #ComposableNode(
            #    package='ardak',
            #    plugin='bfr::OutputCalibrationNode',
            #    name="OutputCalibrationNode"
            #),
            ComposableNode(
                package='ardak',
                plugin='bfr::ArdakNode',
                name='ArdakNode'
            ),
            ComposableNode(
                package='ardak',
                plugin='bfr::DriveControllerNode',
                name='DriveControllerNode',
                parameters=[
                    {"manualControlAllowed": True},
                    {"maxVelocity": 16093.0},
                    {"minVelocity": 0.0},
                    {"driveGearRatio": 0.1},
                    {"wheelCircumference": 57.026},
                    {"steeringGearRatio": 0.0667},
                    {"maxSteeringAngle": 30.0},
                    {"minSteeringAngle": -30.0}
                ],
                remappings=[
                    ("appout/drive/output_command", "odrive0/motor0/input_vel"),
                    ("appout/steering/output_command", "odrive0/motor1/input_pos")
                ]
            )
        ],
        output="screen"
    )

    gamepad = Node(
        package="bfr_hal",
        executable="gamepad.py"
    )

    odrive = Node(
        package="odrive_ros2",
        executable="odrive"
    )

    ld.add_action(container)
    ld.add_action(gamepad)
    ld.add_action(odrive)

    return ld
