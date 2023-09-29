from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("acquire"),
                                "launch",
                                "_view_video_streams.inner.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "camera0": ".*simulated: uniform random.*",
                    "camera1": ".*simulated: radial sin.*",
                }.items(),
            ),
        ]
    )
