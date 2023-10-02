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
    n_processes_arg = DeclareLaunchArgument(
        "n_processes",
        default_value=TextSubstitution(text="2"),
        description="Number of Acquire processes to launch.",
    )

    keep_last_arg = DeclareLaunchArgument(
        "keep_last",
        default_value=TextSubstitution(text="-1"),
        description="Number of images to keep in the buffer.",
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=TextSubstitution(text="acquire"),
        description="Namespace for the nodes.",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text=str("WARN")),
        description="Logging level",
    )

    camera0_arg = DeclareLaunchArgument(
        "camera0",
        default_value=TextSubstitution(text=".*simulated.*uniform random.*"),
        description="Camera descriptor for the first stream.",
    )

    camera1_arg = DeclareLaunchArgument(
        "camera1",
        default_value=TextSubstitution(text=".*simulated.*radial sin.*"),
        description="Camera descriptor for the second stream.",
    )

    args = [
        n_processes_arg,
        namespace_arg,
        log_level_arg,
        keep_last_arg,
        camera0_arg,
        camera1_arg,
    ]

    return LaunchDescription(
        [
            n_processes_arg,
            namespace_arg,
            log_level_arg,
            keep_last_arg,
            camera0_arg,
            camera1_arg,
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
                    "namespace": LaunchConfiguration("namespace"),
                    "log_level": LaunchConfiguration("log_level"),
                    "keep_last": LaunchConfiguration("keep_last"),
                    "camera0": LaunchConfiguration("camera0"),
                    "camera1": LaunchConfiguration("camera1"),
                }.items(),
            ),
        ]
    )
