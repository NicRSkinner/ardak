# Launch tftree
# Launch realsense cameras
# Launch pointcloud to laserscan
# Launch map algo

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    tfnode1 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0.3', '0.2', '0.1', '0', '0', '0', '3', 'base_link', 'wheel_link']
        )

    tfnode2 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '0.1', 'base_link', 'map']
        )

    tfnode3 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '0.1', 'map', 'base_footprint']
        )

    ld.add_action(tfnode1, tfnode2, tfnode3)

    return ld