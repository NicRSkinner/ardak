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
        pkg_share, 'description/ardak/ardak.urdf')

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
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True',
                              description='flag to enable joint_state_publiser_gui'),
        DeclareLaunchArgument(name='model', default_value=default_urdf_model_path,
                              description='Absolute path to the urdf model'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to the rviz config file.'),
        joint_state_pubisher_node,
        joint_state_pubisher_node_gui,
        robot_state_publisher_node,
        rviz_node
    ])
