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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , OpaqueFunction
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # -- DIRECTORIES/ARGUMENTS/CONFIGS --
    pkg_share = FindPackageShare(package='ardak').find('ardak')
    default_rviz_config_path = os.path.join(pkg_share,
                                            'rviz/nav_config.rviz')
    default_sdf_model_path = os.path.join(
        pkg_share,
        'description/ardak/model.sdf')
    default_urdf_model_path = os.path.join(
        pkg_share,
        'description/ardak/ardak.urdf')
    default_world_path = os.path.join(
        pkg_share,
        'world/Gazebo_Fortress/basic_world.sdf'
    )

    default_world_name = 'basic_world'
    robot_name_in_model = 'ardak'
    nav2_dir = get_package_share_directory('nav2_bringup')
    foxglove_dir = get_package_share_directory('foxglove_bridge')

    drive_control_config_path = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'DriveController.yaml'
    )
    localization_config_path = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'localization.yaml'
    )
    ekf_config_path = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'ekf.yaml'
    )
    nav_control_config_path = os.path.join(
        pkg_share,
        'config',
        'Navigation.yaml'
    )
    cameras_config_path = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'Cameras.yaml'
    )
    parambridge_config = os.path.join(
        get_package_share_directory('ardak'),
        'config',
        'Gazebo11ParameterBridge.yaml'
    )

    # Set the path to different files and folders.
    #pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    gazebo_models_path = os.path.join(pkg_share, 'description')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    os.environ["IGN_GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration('headless')
    gzweb = LaunchConfiguration('use_gzweb')
    namespace = LaunchConfiguration('namespace')
    sdf_model = LaunchConfiguration('sdf_model')
    use_namespace = LaunchConfiguration('use_namespace')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    use_manual_drive = LaunchConfiguration('use_manual_drive')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz = LaunchConfiguration('rviz')
    rvizconfig = LaunchConfiguration('rvizconfig')
    localization = LaunchConfiguration('localization')
    spawnX = LaunchConfiguration('SimSpawnX')
    spawnY = LaunchConfiguration('SimSpawnY')
    spawnZ = LaunchConfiguration('SimSpawnZ')
    spawnYaw = LaunchConfiguration('SimSpawnYaw')
    useMappingDriver = LaunchConfiguration('UseMappingDriver')
    useFoxgloveRosBridge = LaunchConfiguration('UseFoxgloveRosBridge')
    useRosbridgeServer = LaunchConfiguration('UseRosbridgeServer')
    rosbridgeCertDir = LaunchConfiguration('RosbridgeCertDirectory')
    worldName = 'basic_world'

    # -- DIRECTORIES/ARGUMENTS/CONFIGS --

    # -- LAUNCH ARGUMENTS
    launch_args = [
        DeclareLaunchArgument(name="localization", default_value="True",
                              description='Whether to start localization nodes.'),
        DeclareLaunchArgument(name='rviz', default_value='False',
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
        DeclareLaunchArgument(name='use_gzweb', default_value='False',
                              description='Spawn a gzweb server for remote viewing of simulation'),
        DeclareLaunchArgument(name='use_simulator', default_value='True',
                              description='Whether to start the simulator'),
        DeclareLaunchArgument(name='world', default_value=default_world_path,
                              description='Full path to the world model file to load'),
        DeclareLaunchArgument(name='urdf_model', default_value=default_urdf_model_path,
                              description='Full path to the urdf model.'),
        DeclareLaunchArgument(name='use_manual_drive', default_value='False',
                              description='Use manual driving rather than nav2 input points.'),
        DeclareLaunchArgument(name='SimSpawnX', default_value='0.0',
                              description='X position to spawn the robot at during simulation.'),
        DeclareLaunchArgument(name='SimSpawnY', default_value='0.0',
                              description='Y position to spawn the robot at during simulation.'),
        DeclareLaunchArgument(name='SimSpawnZ', default_value='0.2',
                              description='Z position to spawn the robot at during simulation.'),
        DeclareLaunchArgument(name='SimSpawnYaw', default_value='0.0',
                              description='Rotational Yaw to spawn the robot at during simulation.'),
        DeclareLaunchArgument(name="UseMappingDriver",
                              default_value='True', description="Use an autonomous mapping driver for robot movement"),
        DeclareLaunchArgument(name="UseFoxgloveRosBridge",default_value='True', description="Open a Websocket for use with foxglove-studio"),
        DeclareLaunchArgument(name="UseRosbridgeServer", default_value="False", description="Open a Rosbridge server with wss for foxglove-studio"),
        DeclareLaunchArgument(name="RosbridgeCertDirectory", default_value="/usr/share/rosbridge/certifications/", description="Directory for ssl certifications using rosbridge"),
    ]
    # -- LAUNCH ARGUMENTS

    # -- REMAPPINGS/PARAMETERS
    camera_remappings = [
        ('/T265/odom', '/camera/odometry'),
        ('/T265/imu', '/camera/imu/data'),
        ('/D400/color/camera_info', '/color/camera_info'),
        ('/D400/color/image_raw', '/color/image_raw'),
    ]

    mapping_remappings = [
        ('odom', '/odometry/local'),
        ('rgb/image', '/color/image_raw'),
        ('rgb/camera_info', '/color/camera_info'),
        ('depth/image', '/aligned_depth_to_color/image_raw'),
        ('map', '/unfenced_map')
    ]

    mapping_parameters = [{
        'queue_size': 200,
        'frame_id': 'base_link',
        'use_sim_time': use_simulator,
        'approx_sync': True,
        'wait_imu_to_init': True,
        'wait_for_transform': 2.0,
        'odom_frame_id': "odom",
        'GridGlobal/MinSize': '40.0', # Make the map a minimum of 40 meters
        #publish_tf: False
    }]

    ardak_parameters = [{
        'manualControlAllowed': use_manual_drive,
        'maxVelocity': 4828.0,  # 3mph
        'minVelocity': -4828.0,  # -3mph
        'driveGearRatio': 0.037037, # 1/27 gear ratio
        'wheelCircumference': 63.837,
        'maxSteeringVelocity': 0.0, # 0.0 rad/s, not currently used.
        'wheelbase': 0.433,
    }]

    pointcloud_remappings = [
        ('depth/image', '/D400/depth/image_rect_raw'),
        ('depth/camera_info', '/D400/depth/camera_info'),
        ('cloud', '/D400/cloud_from_depth')
    ]

    pointcloud_parameters = [{
        'approx_sync': True
    }]

    alignment_remappings = [
        ('camera_info' , '/color/camera_info'),
        ('cloud', '/D400/cloud_from_depth'),
        ('image_raw', '/aligned_depth_to_color/image_raw'),
    ]

    alignment_parameters = [{
        'decimation': 2,
        'fixed_frame_id': 'base_link',
        'fill_holes_size': 1
    }]

    # -- REMAPPINGS/PARAMETERS
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[
            {
                'robot_description': Command(['xacro ', urdf_model]),
                'use_sim_time': use_simulator
            }],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')],
        arguments=[urdf_model]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(
            PythonExpression(['not ', use_simulator])
        )
    )

    mapping_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=mapping_parameters,
        remappings=mapping_remappings,
        arguments=['-d'],
        condition=IfCondition(localization)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        parameters=[
                    {'use_sim_time': use_simulator}
                    ],
        condition=launch.conditions.IfCondition(rviz)
    )

    """simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo_ros,
                'launch',
                'gzserver.launch.py')
            ),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items()
    )

    simulation_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo_ros,
                'launch', 'gzclient.launch.py')
            ),
        condition=IfCondition(
            PythonExpression([use_simulator, ' and not ', headless])
        )
    )"""

    # Launch the robot
    # Sim Server Creator
    gazebo_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', world],
        output='screen',
        condition=IfCondition(use_simulator)
    )

    # Sim Entity Spawner
    entity_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
                   '-entity', robot_name_in_model,
                   '-file', sdf_model,
                   '-x', spawnX,
                   '-y', spawnY,
                   '-z', spawnZ,
                   '-Y', spawnYaw
                   ],
        condition=IfCondition(use_simulator),
        output='screen'
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '--log-level', 'info', '-p', 'config_file:=' + parambridge_config],
        condition=IfCondition(use_simulator)
    )
    
    # Sim Client Creator
    gazebo_client = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        output='screen',
        condition=IfCondition(
            PythonExpression([use_simulator, ' and not ', headless])
        )
    )

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
        condition=IfCondition(use_manual_drive)
    )

    odrive_node = Node(
        package="odrive_ros2",
        executable="odrive",
        condition=IfCondition(
            PythonExpression(['not ', use_simulator])
        )
    )

    ekf_node_odom = Node(
        package="robot_localization",
        name="ekf_filter_node_odom",
        executable="ekf_node",
        parameters=[
                    ekf_config_path,
                    {'use_sim_time': use_simulator}
                    ],
        remappings=[
                    ('odometry/filtered', 'odometry/local'),
                    ('/set_pose', '/initialpose')
                    ],
        condition=IfCondition(localization)
    )


    ekf_node_map = Node(
        package="robot_localization",
        name="ekf_filter_node_map",
        executable="ekf_node",
        parameters=[
                    ekf_config_path,
                    {'use_sim_time': use_simulator}
                    ],
        remappings=[
                    ('odometry/filtered', 'odometry/global'),
                     ('/set_pose', '/initialpose')
                     ],
        condition=IfCondition(localization)
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
                    ekf_config_path,
                    {'use_sim_time': use_simulator}
                    ],
        remappings=[
                    ('imu', 'imu/data'),
                    ('gps/fix', 'gps/fix'),
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps'),
                    ('odometry/filtered', 'odometry/global')
                    ],
        condition=IfCondition(localization)
    )

    geofencer_node = Node(
        package="zyg_ai",
        executable="geofencer_node",
        parameters=[
            {
                'use_sim_time': use_simulator
            }
        ],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            nav2_dir + '/launch/navigation_launch.py'),
        launch_arguments= {
            'use_sim_time': use_simulator,
            'params_file': nav_control_config_path
        }.items(),
        condition=IfCondition(
            PythonExpression(['not ', use_manual_drive])
        )
    )

    nav2_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            nav2_dir + '/launch/rviz_launch.py')
    )

    # HARDWARE NODES
    d400_node = Node(
        package='realsense2_camera',
        namespace="D400",
        name="D400",
        executable='realsense2_camera_node',
        parameters=[cameras_config_path],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
        remappings=camera_remappings,
        condition=IfCondition(
            PythonExpression(['not ', use_simulator])
        )
    )

    rtab_alignment_pointcloud = Node(
            package='rtabmap_util',
            executable='point_cloud_xyz',
            parameters=pointcloud_parameters,
            remappings=pointcloud_remappings,
            output='screen',
            condition=IfCondition(
                PythonExpression(['not ', use_simulator])
            )
        )

    rtab_alignment_aligner = Node(
        package='rtabmap_util',
        executable='pointcloud_to_depthimage',
        parameters=alignment_parameters,
        remappings=alignment_remappings,
        condition=IfCondition(
            PythonExpression(['not ', use_simulator])
        )
    )

    t265_node = Node(
        package='bfr_hal',
        name='T265',
        executable='t265_node',
        parameters=[
            {
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,
                'use_sim_time': use_simulator
            }
        ],
        remappings=camera_remappings,
        condition=IfCondition(
            PythonExpression(['not ', use_simulator])
        )
    )

    # AI Algorithms
    beanbagdetector_node = Node(
        package="zyg_ai",
        executable="beanbagdetector",
        parameters=[
            {
                'use_sim_time': use_simulator
            }
        ]
    )

    mappingdriver_node = Node(
        package="zyg_ai",
        executable="mappingdriver",
        parameters=[
            {
                'use_sim_time': use_simulator
            }
        ],
        condition=IfCondition(useMappingDriver)
    )

    foxglove_server = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[
            {
                'port': 10621,
                'address': '0.0.0.0',
                'tls': True,
                'certfile': PathJoinSubstitution([rosbridgeCertDir, 'server_cert.pem']),
                'keyfile': PathJoinSubstitution([rosbridgeCertDir, 'server_key.pem']),
                'num_threads': 8,
                'send_buffer_limit': 100000000, #100MB, lower and use whitelist if buffer cannot clear.
                'use_sim_time': use_simulator
            }
        ],
        condition=IfCondition(useFoxgloveRosBridge)
    )

    rosbridge_server = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        parameters=[
            {
                'port': 10621,
                'address': '',
                'ssl': True,
                'certfile': PathJoinSubstitution([rosbridgeCertDir, 'server_cert.pem']),
                'keyfile': PathJoinSubstitution([rosbridgeCertDir, 'server_key.pem']),
                'authenticate': False
            }
        ]
    )



    return LaunchDescription(launch_args + [
        SetParameter(name='use_sim_time', value=use_simulator),

        # ROBOT NODES
        ardak_nodes,
        geofencer_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        odrive_node,

        # HARDWARE NODES
        gamepad_node,
        t265_node,
        d400_node,
        rtab_alignment_pointcloud,
        rtab_alignment_aligner,

        # LOCALIZATION NODES
        ekf_node_odom,
        ekf_node_map,
        navsat_transform_node,
        mapping_node,

        # NAVIGATION NODES
        nav2_launch,

        # SIMULATION NODES
        #simulation_launch,
        #simulation_client_launch,
        gazebo_server,
        entity_spawner,
        gz_bridge,

        # VISUALIZATION NODES
        rviz_node,
        foxglove_server,
        gazebo_client,
        #rosbridge_server,

        # AI Algorithms
        #beanbagdetector_node,
        mappingdriver_node
    ])
