# Due to issues with the realsens wrapper, an intermediary package may be ncessary
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

    pointcloud_remappings = [
        {'depth/image', '/D400/depth/image_rect_raw'},
        {'depth/camera_info', '/D400/depth/camera_info'},
        {'cloud', '/D400/cloud_from_depth'}
    ]

    pointcloud_parameters = [{
        'approx_synd': False
    }]

    alignment_remappings = [
        {'camera_info' , '/D400/color/camera_info'},
        {'cloud', '/D400/cloud_from_depth'},
        {'image_raw', '/D400/aligned_depth_to_color/image_raw'},
    ]

    alignment_parameters = [{
        'decimation': 2,
        'fixed_frame_id': 'D400_link',
        'fill_holes_size': 1
    }]

    mapping_remappings = [
        {'odom', '/T265/pose/sample'}
    ]

    mapping_parameters = [{
        'frame_id': 'T265_link'
    }]

    def generate_launch_description():
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')

        return LaunchDescription([
            # START: Build TF Tree
            Node(
                # Configure the TF of the robot
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=['0.0', '0.0', '0.0', '0.0',
                            '0.0', '0.0', 'odom', 'map']
            ),
            Node(
                # Configure the TF of the robot
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=['0.0', '0.0', '0.0', '0.0',
                            '0.0', '0.0', 'base_link', 'base_footprint']
            ),
            Node(
                # Configure the TF of the robot
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=['0.0', '0.0', '0.0', '0.0',
                            '0.0', '0.0', 'T265_link', 'base_link']
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=['0.0', '0.025', '0.03', '-1.5708', '0.0',
                        '-1.5708', 'D435_link', 'T265_link']
            ),
            # END: Build TF Tree

            # START: Launch D435 and T265 cammeras
            # END: Launch D435 and T265 cammeras
            
            # START: Align depth image to color image
            Node(
                package='rtabmap_ros',
                executable='point_cloud_xyz',
                parameters=pointcloud_parameters,
                remmappings=pointcloud_remappings,
                output='screen'
            ),
            Node(
                package='rtabmap_ros',
                executable='pointcloud_to_depthimage',
                parameters=alignment_parameters,
                remappings=alignment_remappings
            )
            # END: Align depth image to color image
        ])