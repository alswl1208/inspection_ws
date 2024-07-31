import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    param_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(get_package_share_directory('camera_capture_pkg'), 'config', 'contest.yaml'),
            description='/home/minji/contest.yaml'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={'params_file': param_file, 'map': map_file}.items(),
        ),
    ])
