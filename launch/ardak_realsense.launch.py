import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription()

    localization_config_path = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'localization.yaml'
    )

    parameters = [{
        'queue_size': 30,
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'approx_sync': False,
        'odom_frame_id': 'odom',
        'odom_sensor_sync': True,
        'odom_tf_linear_variance': 0.001,
        'odom_tf_angular_variance': 0.001,
        'RGBD/NeighborLinkRefining': 'true'}]

    remappings = [
        ('odom', '/rs_t265/odom'),
        ('rgb/image', '/rs_d435/image_raw'),
        ('rgb/camera_info', '/rs_d435/image_raw/camera_info'),
        ('rgbd_image', '/rs_d435/image_raw'),
        ('depth/image', '/rs_d435/aligned_depth/image_raw')]

    base_link_transform = Node(
        # Configure the TF of the robot
        package='tf2_ros',
        node_executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0',
                   '0.0', '0.0', '0.0',
                   'base_link', 'd435_camera_link']
    )

    rs_t265_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d435',
        output='screen',
        parameters=[localization_config_path]
    )

    ld.add_action(base_link_transform)
    ld.add_action(rs_t265_node)

    return ld
