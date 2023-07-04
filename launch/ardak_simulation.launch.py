import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='ardak').find('ardak')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav_config.rviz')
    default_sdf_model_path = os.path.join(
        pkg_share, 'description/ardak/model.sdf')
    default_urdf_model_path = os.path.join(
        pkg_share, 'description/ardak/ardak.urdf')
    default_world_path = os.path.join(
        pkg_share, 'world/smalltown.world'
    )
    robot_name_in_model = 'ardak'
    nav2_dir = get_package_share_directory('nav2_bringup')

    ekf_config_path = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'ekf.yaml'
    )

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.3'
    spawn_yaw_val = '0.0'

    drive_control_config_path = os.path.join(
        pkg_share,
        'config',
        'DriveController.yaml'
    )

    nav_control_config_path = os.path.join(
        pkg_share,
        'config',
        'Navigation.yaml'
    )

    # Set the path to different files and folders.
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    gazebo_models_path = os.path.join(pkg_share, 'description')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration('headless')
    namespace = LaunchConfiguration('namespace')
    sdf_model = LaunchConfiguration('sdf_model')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    use_manual_drive = LaunchConfiguration('use_manual_drive')
    urdf_model = LaunchConfiguration('urdf_model')

    mapping_remappings = [
        ('odom', '/odometry/local'),
        ('rgb/image', '/color/image_raw'),
        ('rgb/camera_info', '/color/camera_info'),
        ('depth/image', '/aligned_depth_to_color/image_raw'),
        ('map', '/unfenced_map')
    ]

    # Make this not publish TF and have a different program set it to 0,0,0?
    mapping_parameters = [{
        'queue_size': 200,
        'frame_id': 'base_link',
        'use_sim_time': use_sim_time,
        'approx_sync': True,
        'wait_imu_to_init': True,
        'wait_for_transform': 2.0,
        'odom_frame_id': "odom",
        #publish_tf: False
    }]

    ardak_parameters = [{
        'manualControlAllowed': use_manual_drive,
        'maxVelocity': 4828.0,  # 3mph
        'minVelocity': -4828.0,  # -3mph
        'driveGearRatio': 0.111,
        'wheelCircumference': 63.837,
        'maxSteeringVelocity': 0.0,
        'wheelbase': 0.433,
    }]

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[
            {
                'robot_description': Command(['xacro ', LaunchConfiguration('urdf_model')]),
                'use_sim_time': use_sim_time
            }],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')],
        arguments=[LaunchConfiguration('urdf_model')]
    )

    mapping_node = Node(
        package='rtabmap_slam',
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

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())

    simulation_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    # Launch the robot
    entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,
                   '-file', sdf_model,
                   '-x', spawn_x_val,
                   '-y', spawn_y_val,
                   '-z', spawn_z_val,
                   '-Y', spawn_yaw_val],
        output='screen')

    # All of the Ardak nodes used for simulation only
    ardak_nodes = Node(
        package="ardak",
        executable="main",
        parameters=ardak_parameters,
        remappings=[
            ("appout/drive/left_drive_command", "odrive0/motor0/input_vel"),
            ("appout/drive/right_drive_command", "odrive0/motor1/input_vel"),
            ("appout/drive/wheel_cmd", "wheel_cmd")
        ]
    )

    gamepad_node = Node(
        package="bfr_hal",
        executable="gamepad.py",
        condition=IfCondition(LaunchConfiguration('use_manual_drive'))
    )

    ekf_node_odom = Node(
        package="robot_localization",
        name="ekf_filter_node_odom",
        executable="ekf_node",
        parameters=[ekf_config_path,
                    {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local'),
                    ('/set_pose', '/initialpose')]
    )

    # Make this not give odometry filtered. Fix.
    ekf_node_map = Node(
        package="robot_localization",
        name="ekf_filter_node_map",
        executable="ekf_node",
        parameters=[ekf_config_path,
                    {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global'),
                     ('/set_pose', '/initialpose')]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config_path,
        {'use_sim_time': use_sim_time}],
        remappings=[('imu', 'imu/data'),
                    ('gps/fix', 'gps/fix'),
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps'),
                    ('odometry/filtered', 'odometry/global')]
    )

    geofencer_node = Node(
        package="zyg_ai",
        executable="geofencer_node"
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            nav2_dir + '/launch/navigation_launch.py'),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_control_config_path
        }.items()
    )

    nav2_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            nav2_dir + '/launch/rviz_launch.py')
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='rviz', default_value='True',
                              description='flag to use rviz instead of gazebo'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to the rviz config file.'),
        DeclareLaunchArgument(name='namespace', default_value='',
                              description='top-level namespace'),
        DeclareLaunchArgument(name='use_namespace', default_value='False',
                              description='Whether to apply a namespace to the navigation stack.'),
        DeclareLaunchArgument(name='sdf_model', default_value=default_sdf_model_path,
                              description='Absolute path to the sdf model.'),
        DeclareLaunchArgument(name='headless', default_value='False',
                              description='Whether to execute gzclient'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='use_simulator', default_value='True',
                              description='Whether to start the simulator'),
        DeclareLaunchArgument(name='world', default_value=default_world_path,
                              description='Full path to the world model file to load'),
        DeclareLaunchArgument(name='urdf_model', default_value=default_urdf_model_path,
                              description='Full path to the urdf model.'),
        DeclareLaunchArgument(name='use_manual_drive', default_value='False',
                              description='Use manual driving rather than nav2 input points.'),


        simulation_launch,
        simulation_client_launch,
        entity_node,
        robot_state_publisher_node,
        gamepad_node,
        ardak_nodes,
        mapping_node,
        geofencer_node,
        nav2_launch,
        rviz_node,
        ekf_node_odom,
        ekf_node_map,
        navsat_transform_node,
    ])
