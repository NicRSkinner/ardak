# Launch tftree
# Launch realsense cameras
# Launch pointcloud to laserscan  --  This may not happen as I intend to use rtabmap going forward if I can on the raspberry pi
# Launch map algo

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

    parameters=[{
          'queue_size':20,
          'frame_id':'camera_link_d435',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True}]

    remappings=[
          ('odom', 'rs_t265/odom'),
          ('rgb/image', '/rs_d435/image_raw'),
          ('rgb/camera_info', 'rs_d435/image_raw/camera_info'),
          ('depth/image', '/rs_d435/aligned_depth/image_raw')]

    return LaunchDescription([

    
    #Node(
    #    package='realsense_ros2',
    #    executable='rs_t265_node',
    #    name='rs_t265',
    #    output='screen'
    #),
    #Node(
    #    package='realsense_ros2',
    #    executable='rs_d435_node',
    #    name='rs_d435',
    #    output='screen',
    #    parameters=[
    #        {"publish_depth": True},
    #        {"publish_pointcloud": True},
    #        {"is_color": True},
    #        {"publish_image_raw_": True},
    #        {"fps": 6}      # Can only take values of 6,15,30 or 60
    #    ]
    #),
    Node(
        package='realsense2_camera',
        namespace="d435",
        name="d435",
        executable='realsense2_camera_node',
        parameters=[localization_config_path],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
    ),
    Node(
        package='realsense2_camera',
        namespace="t265",
        name="t265",
        executable='realsense2_camera_node',
        parameters=[localization_config_path],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
    ),
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.3', '0.1', '0.0', '0.0', '0.0', 'base_link', 'base_camera']
    ),
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.001', '0.0', '0.0', '0.0', 'base_camera', 'd435_link']
    ),
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '1.5708', '0.0', '1.5708', 'd435_link', 'd435_rotated_frame']
    ),
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '-0.001', '0.0', '0.0', '0.0', 'base_camera', 't265_link']
    ),
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 't265_odom_frame', 'base_link']
    ),
    #Node(
    #    package='rtabmap_ros', executable='rtabmap', output='screen',
    #    parameters=parameters,
    #    remappings=remappings,
    #    arguments=['-d']
    #),
    #Node(
    #    package='rtabmap_ros', executable='rtabmapviz', output='screen',
    #    parameters=parameters,
    #    remappings=remappings
    #),

    SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

    DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true')

    ])