from email.policy import default
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(name="localization", default_value="False"),
        DeclareLaunchArgument(name="robot_state", default_value="False"),
        DeclareLaunchArgument(name="joint_state", default_value="False")
    ]

    drive_control_config_path = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'DriveController.yaml'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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

    rs_t265_node = Node(
        package='realsense_ros2',
        executable='rs_t265_node',
        name='rs_t265',
        condition=IfCondition(PythonExpression(
            [LaunchConfiguration("localization"), " == True"])),
        output='screen'
    )

    rs_d435_node = Node(
        package='realsense_ros2',
        executable='rs_d435_node',
        name='rs_d435',
        condition=IfCondition(PythonExpression(
            [LaunchConfiguration("localization"), " == True"])),
        parameters=[
            {"publish_depth": True},
            {"publish_pointcloud": False},
            {"is_color": True},
            {"publish_image_raw_": True},
            {"fps": 6}      # Can only take values of 6,15,30 or 60
        ],
        output='screen'
    )

    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        condition=IfCondition(PythonExpression(
            [LaunchConfiguration("localization"), " == True"])),
        parameters=parameters,
        remappings=remappings,
        arguments=['-d'],
        output='screen',
    )

    ardak_nodes = Node(
        package="ardak",
        executable="main",
        parameters=[drive_control_config_path],
        remappings=[
            ("appout/drive/left_drive_command", "odrive0/motor0/input_vel"),
            ("appout/drive/right_drive_command", "odrive0/motor1/input_vel")
        ]
    )

    gamepad = Node(
        package="bfr_hal",
        executable="gamepad.py"
    )

    odrive = Node(
        package="odrive_ros2",
        executable="odrive"
    )

    base_link_transform = Node(
        # Configure the TF of the robot
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0',
                   '0.0', '0.0', '0.0',
                   't265_frame', 'base_link'],
        output='screen'
    )

    camera_link_d435_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.025', '0.03',
                   '-1.5708', '0.0', '-1.5708',
                   'base_link', 'camera_link_d435'],
        output='screen'
    )

    camera_link_d435_offset_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.025', '0.03',
                   '-1.5708', '0.0', '-1.5708',
                   'base_link', 'camera_link_d435_pcl'],
        output='screen'
    )

    return LaunchDescription(launch_args + [
        rs_t265_node,
        rs_d435_node,
        rtabmap_node,
        camera_link_d435_node,
        camera_link_d435_offset_node,
        base_link_transform,
        ardak_nodes,
        gamepad,
        odrive
    ])
