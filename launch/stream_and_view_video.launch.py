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

    keep_last_arg = DeclareLaunchArgument(
        "keep_last",
        default_value=TextSubstitution(text="-1"),
        description="Number of images to keep in the buffer.",
    )

    camera_args = {
        "camera0/identifier": DeclareLaunchArgument(
            "camera0/identifier",
            default_value=TextSubstitution(text=".*simulated.*uniform random.*"),
            description="Camera descriptor for the first stream.",
        ),
        "camera0/topic": DeclareLaunchArgument(
            "camera0/topic",
            default_value=TextSubstitution(text="stream0"),
            description="Topic to publish to for the first stream.",
        ),
        "camera0/binning": DeclareLaunchArgument(
            "camera0/binning",
            default_value=TextSubstitution(text="1"),
            description="Binning for the first stream.",
        ),
        "camera0/exposure_time_us": DeclareLaunchArgument(
            "camera0/exposure_time_us",
            default_value=TextSubstitution(text="10000"),
            description="Exposure time for the first stream.",
        ),
        "camera0/image_size": DeclareLaunchArgument(
            "camera0/image_size",
            default_value=TextSubstitution(text="[640,480]"),
            description="Image size for the first stream.",
        ),
        "camera1/identifier": DeclareLaunchArgument(
            "camera1/identifier",
            default_value=TextSubstitution(text=".*simulated.*radial sin.*"),
            description="Camera descriptor for the second stream.",
        ),
        "camera1/topic": DeclareLaunchArgument(
            "camera1/topic",
            default_value=TextSubstitution(text="stream1"),
            description="Topic to publish to for the second stream.",
        ),
        "camera1/binning": DeclareLaunchArgument(
            "camera1/binning",
            default_value=TextSubstitution(text="1"),
            description="Binning for the second stream.",
        ),
        "camera1/exposure_time_us": DeclareLaunchArgument(
            "camera1/exposure_time_us",
            default_value=TextSubstitution(text="10000"),
            description="Exposure time for the second stream.",
        ),
        "camera1/image_size": DeclareLaunchArgument(
            "camera1/image_size",
            default_value=TextSubstitution(text="[640,480]"),
            description="Image size for the second stream.",
        ),
    }

    streamer_launch_args = {
        "namespace": LaunchConfiguration("namespace"),
        "log_level": LaunchConfiguration("log_level"),
    }
    streamer_launch_args.update({k: LaunchConfiguration(k) for k in camera_args.keys()})

    return LaunchDescription(
        [
            namespace_arg,
            log_level_arg,
            keep_last_arg,
        ]
        + list(camera_args.values())
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("acquire"),
                                "launch",
                                "view_video.launch.py",
                            ]
                        ),
                    ]
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "log_level": LaunchConfiguration("log_level"),
                    "keep_last": LaunchConfiguration("keep_last"),
                    "topic": LaunchConfiguration("camera0/topic"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("acquire"),
                                "launch",
                                "view_video.launch.py",
                            ]
                        ),
                    ]
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "log_level": LaunchConfiguration("log_level"),
                    "keep_last": LaunchConfiguration("keep_last"),
                    "topic": LaunchConfiguration("camera1/topic"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("acquire"),
                                "launch",
                                "stream_video.launch.py",
                            ]
                        ),
                    ]
                ),
                launch_arguments=streamer_launch_args.items(),
            ),
        ]
    )
