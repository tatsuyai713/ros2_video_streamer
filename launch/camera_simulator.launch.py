import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node

config = os.path.join(
    get_package_prefix('camera_simulator'),
    '..',
    '..',
    'src',
    'ros2_video_streamer',
    'config',
    'config.yaml'
)

def generate_launch_description():
    return LaunchDescription([
        # 起動したいノードを記述
        Node(
            package="camera_simulator",
            executable="camera_simulator",
            output="screen",
            prefix=['stdbuf -o L'],
            parameters=[config]
        )
    ])

    