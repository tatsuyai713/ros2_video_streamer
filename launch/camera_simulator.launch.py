import os
from ament_index_python.packages import get_package_prefix
import launch
import launch_ros.actions

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
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package="camera_simulator",
            node_executable="camera_simulator",
            output="screen",
            prefix=['stdbuf -o L'],
            parameters=[config]
        )
    ])
    return ld


if __name__ == "__main__":
    generate_launch_description()
