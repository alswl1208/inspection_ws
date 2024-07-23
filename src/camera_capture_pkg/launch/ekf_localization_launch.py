from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/home/minji/inspection_ws/src/camera_capture_pkg/config/ekf_localization.yaml'],
        )
    ])
