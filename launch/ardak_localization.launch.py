# Due to issues with the realsens wrapped, an intermediary package may be ncessary
# https://github.com/introlab/rtabmap_ros/issues/743

# https://github.com/introlab/rtabmap_ros/issues/345

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

    remappings = [
        ('odom', '/rs_t265/odom'),
        ('rgb/image', '/rs_d435/image_raw'),
        ('rgb/camera_info', '/rs_d435/image_raw/camera_info'),
        ('depth/image', '/rs_d435/aligned_depth/image_raw')]

    def generate_launch_description():
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')

        return LaunchDescription([

        ])