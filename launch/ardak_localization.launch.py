# Launch tftree
# Launch realsense cameras
# Launch pointcloud to laserscan  --  This may not happen as I intend to use rtabmap going forward if I can on the raspberry pi
# Launch map algo

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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

    ld = LaunchDescription()

    t265_node = Node(
        package='realsense_ros2',
        node_executable='rs_t265_node',
        node_name='rs_t265',
        output='screen'
        )

    d400_node = Node(
        package='realsense_ros2',
        node_executable='rs_d435_node',
        node_name='rs_d435',
        output='screen',
        parameters=[
            {"publish_depth": True},
            {"publish_pointcloud": True},
            {"is_color": True},
            {"publish_image_raw_": True},
            {"fps": 30}      # Can only take values of 6,15,30 or 60
            ]
        )

    tf265_tf_node = Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 't265_frame', 'base_link']
        ),

    camera_link_tf_node = Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            # Check this to make sure the transform is correct.
            arguments=['0.0', '0.025', '0.03', '1.5708', '0.0', '1.5708', 'base_link', 'camera_link_d435']
        )

    rtabmap_node = Node(
            package='rtabmap_ros', node_executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'])

    # Camera nodes
    ld.add_action(t265_node, d400_node)

    # Transform nodes
    ld.add_action(tf265_tf_node, camera_link_tf_node)

    # Localization nodes
    ld.add_action(rtabmap_node)

    ld.SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

    ld.DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true')

    return ld