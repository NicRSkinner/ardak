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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    camera_dir = get_package_share_directory('realsense2_camera')

    cameras_config_path = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'Cameras.yaml'
    )

    pointcloud_remappings = [
        ('depth/image', '/D400/depth/image_rect_raw'),
        ('depth/camera_info', '/D400/depth/camera_info'),
        ('cloud', '/D400/cloud_from_depth')
    ]

    pointcloud_parameters = [{
        'approx_sync': True
    }]

    alignment_remappings = [
        ('camera_info' , '/D400/color/camera_info'),
        ('cloud', '/D400/cloud_from_depth'),
        ('image_raw', '/D400/aligned_depth_to_color/image_raw'),
    ]

    alignment_parameters = [{
        'decimation': 2,
        'fixed_frame_id': 'base_link',
        'fill_holes_size': 1
    }]

    mapping_remappings = [
        ('odom', '/T265/pose/sample'),
        ('rgb/image', '/D400/color/image_raw'),
        ('rgb/camera_info', '/D400/color/camera_info'),
        ('depth/image', '/D400/aligned_depth_to_color/image_raw')
    ]

    mapping_parameters = [{
        'queue_size': 200,
        'frame_id': 'base_link',
        'use_sim_time': False,
        'approx_sync': True,
        'wait_imu_to_init': True
    }]



    print ("returning launch description")
    return LaunchDescription([
        # START: Build TF Tree
        #Node(
        #    # Configure the TF of the robot
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    output='screen',
        #    arguments=['0.0', '0.0', '0.0', '0.0',
        #                '0.0', '0.0', 'base_link', 'T265_link']
        #),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.025', '0.03', '0.0', '0.0',
                    '0.0', 'base_link', 'D400_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.025', '0.03', '0.0', '0.0',
                    '0.0', 'odom_frame', 'base_link']
        ),
        Node(
            # Configure the TF of the robot
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0',
                        '0.0', '0.0', 'base_footprint', 'base_link']
        ),
        # END: Build TF Tree

        # START: Launch D435 and T265 cammeras
        Node(
            package='realsense2_camera',
            namespace="D400",
            name="D400",
            executable='realsense2_camera_node',
            parameters=[cameras_config_path],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            emulate_tty=True,
        ),

        Node(
            package='realsense2_camera',
            namespace="T265",
            name="T265",
            executable='realsense2_camera_node',
            parameters=[cameras_config_path],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            emulate_tty=True,
        ),
        # END: Launch D435 and T265 cammeras

        # START: Align depth image to color image
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyz',
            parameters=pointcloud_parameters,
            remappings=pointcloud_remappings,
            output='screen'
        ),
        Node(
            package='rtabmap_util',
            executable='pointcloud_to_depthimage',
            parameters=alignment_parameters,
            remappings=alignment_remappings
        ),
        # END: Align depth image to color image

        # START: Mapping
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=mapping_parameters,
            remappings=mapping_remappings,
            arguments=['-d']
        ),
        # END: Mapping

        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true')
    ])
