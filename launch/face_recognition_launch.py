from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_fdr_cpp',
            executable='ros2_fdr_cpp',
            name='ros2_fdr_cpp',
            output='screen'
        )
    ])