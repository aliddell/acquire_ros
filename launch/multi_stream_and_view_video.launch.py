from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    keep_last_arg = DeclareLaunchArgument(
        "keep_last",
        default_value=TextSubstitution(text="-1"),
        description="Number of images to keep in the buffer.",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text=str("WARN")),
        description="Logging level",
    )

    camera00_arg = DeclareLaunchArgument(
        "camera00",
        default_value=TextSubstitution(text=".*simulated.*uniform random.*"),
        description="Camera descriptor for the first process, first stream.",
    )

    camera01_arg = DeclareLaunchArgument(
        "camera01",
        default_value=TextSubstitution(text=".*simulated.*radial sin.*"),
        description="Camera descriptor for the first process, second stream.",
    )

    camera10_arg = DeclareLaunchArgument(
        "camera10",
        default_value=TextSubstitution(text=".*bfly.*"),
        description="Camera descriptor for the second process, first stream.",
    )

    camera11_arg = DeclareLaunchArgument(
        "camera11",
        default_value=TextSubstitution(text=".*simulated.*radial sin.*"),
        description="Camera descriptor for the second process, second stream.",
    )

    return LaunchDescription(
        [
            log_level_arg,
            keep_last_arg,
            camera00_arg,
            camera01_arg,
            camera10_arg,
            camera11_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("acquire"),
                                "launch",
                                "stream_and_view_video.launch.py",
                            ]
                        ),
                    ]
                ),
                launch_arguments={
                    "namespace": "acquire/first",
                    "log_level": LaunchConfiguration("log_level"),
                    "keep_last": LaunchConfiguration("keep_last"),
                    "camera0": LaunchConfiguration("camera00"),
                    "camera1": LaunchConfiguration("camera01"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("acquire"),
                                "launch",
                                "stream_and_view_video.launch.py",
                            ]
                        ),
                    ]
                ),
                launch_arguments={
                    "namespace": "acquire/second",
                    "log_level": LaunchConfiguration("log_level"),
                    "keep_last": LaunchConfiguration("keep_last"),
                    "camera0": LaunchConfiguration("camera10"),
                    "camera1": LaunchConfiguration("camera11"),
                }.items(),
            ),
        ]
    )
