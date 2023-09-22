from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="acquire",
                namespace="acquire",
                executable="streamer",
                name="streamer",
            ),
            Node(
                package="acquire",
                namespace="acquire",
                executable="viewer",
                name="viewer",
            ),
        ]
    )
