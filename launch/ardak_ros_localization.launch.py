import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, declare_launch_argument
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    localization_config_path = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'localization.yaml'
    )

    parameters = [{
        'queue_size': 20,
        'frame_id': 'camera_link_d435',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True}]

    return LaunchDescription([

    ])
