"""Launch realsense2_camera for ardak"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch

local_parameters = [{'name': 'camera_name1',                'default': 'camera', 'description': 'camera unique name'},
                    {'name': 'device_type1',                'default': 'd4.', 'description': 'choose device by type'},
                    {'name': 'depth_width1',                'default': '-1', 'description': 'depth image width'},
                    {'name': 'depth_height1',               'default': '-1', 'description': 'depth image height'},
                    {'name': 'enable_depth1',               'default': 'true', 'description': 'enable depth stream'},
                    {'name': 'color_width1',                'default': '-1', 'description': 'color image width'},
                    {'name': 'color_height1',               'default': '-1', 'description': 'color image height'},
                    {'name': 'enable_color1',               'default': 'true', 'description': 'enable color stream'},
                    {'name': 'depth_fps1',                  'default': '15.', 'description': ''},
                    {'name': 'color_fps1',                  'default': '15.', 'description': ''},
                    {'name': 'camera_name2',                'default': 'T265', 'description': 'camera unique name'},
                    {'name': 'device_type2',                'default': 't265', 'description': 'choose device by type'},
                    {'name': 'enable_fisheye12',            'default': 'true', 'description': 'topic for T265 wheel odometry'},
                    {'name': 'enable_fisheye22',            'default': 'true', 'description': 'topic for T265 wheel odometry'},
                   ]

def generate_launch_description():
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) + 
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_multi_camera_launch.py']),
            launch_arguments=rs_launch.set_configurable_parameters(local_parameters).items(),
        ),
    ])