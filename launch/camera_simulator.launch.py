import os
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package="camera_simulator",
            node_executable="camera_simulator",
            output="screen",
            parameters=[os.path.join(os.path.dirname(__file__), "..", "config", "config.yaml")]
        )
    ])
    return ld


if __name__ == "__main__":
    generate_launch_description()
