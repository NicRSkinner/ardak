import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='ardak').find('ardak')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    default_urdf_model_path = os.path.join(
        pkg_share, 'description/ardak_description.urdf')

    pointcloud_remappings = [
        ('depth/image', '/D400/depth/image_rect_raw'),
        ('depth/camera_info', '/D400/depth/camera_info'),
        ('cloud', '/D400/cloud_from_depth')
    ]

    pointcloud_parameters = [{
        'approx_sync': True
    }]

    alignment_remappings = [
        ('camera_info', '/D400/color/camera_info'),
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

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_pubisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_pubisher_node_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # Need node for simulated realsense D435 camera
    # Need node for simulated T265 camera

    pointcloud_node = Node(
        package='rtabmap_ros',
        executable='point_cloud_xyz',
        parameters=pointcloud_parameters,
        remappings=pointcloud_remappings,
        output='screen'
    )
    alignment_node = Node(
        package='rtabmap_ros',
        executable='pointcloud_to_depthimage',
        parameters=alignment_parameters,
        remappings=alignment_remappings
    )

    mapping_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        output='screen',
        parameters=mapping_parameters,
        remappings=mapping_remappings,
        arguments=['-d']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True',
                              description='flag to enable joint_state_publiser_gui'),
        DeclareLaunchArgument(name='rviz', default_value='False',
                              description='flag to use rviz instead of gazebo'),
        DeclareLaunchArgument(name='model', default_value=default_urdf_model_path,
                              description='Absolute path to the urdf model'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to the rviz config file.'),
        joint_state_pubisher_node,
        joint_state_pubisher_node_gui,
        robot_state_publisher_node,
        rviz_node
    ])
