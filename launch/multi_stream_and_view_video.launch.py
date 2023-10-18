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

    stream0_camera_args = {
        "camera0/identifier": DeclareLaunchArgument(
            "stream0/camera0/identifier",
            default_value=TextSubstitution(text=".*simulated.*uniform random.*"),
            description="Camera descriptor for the first stream.",
        ),
        "camera0/topic": DeclareLaunchArgument(
            "stream0/camera0/topic",
            default_value=TextSubstitution(text="stream0"),
            description="Topic to publish to for the first stream.",
        ),
        "camera0/binning": DeclareLaunchArgument(
            "stream0/camera0/binning",
            default_value=TextSubstitution(text="1"),
            description="Binning for the first stream.",
        ),
        "camera0/exposure_time_us": DeclareLaunchArgument(
            "stream0/camera0/exposure_time_us",
            default_value=TextSubstitution(text="10000"),
            description="Exposure time for the first stream.",
        ),
        "camera0/image_size": DeclareLaunchArgument(
            "stream0/camera0/image_size",
            default_value=TextSubstitution(text="[640,480]"),
            description="Image size for the first stream.",
        ),
        "camera1/identifier": DeclareLaunchArgument(
            "stream0/camera1/identifier",
            default_value=TextSubstitution(text=".*simulated.*radial sin.*"),
            description="Camera descriptor for the second stream.",
        ),
        "camera1/topic": DeclareLaunchArgument(
            "stream0/camera1/topic",
            default_value=TextSubstitution(text="stream1"),
            description="Topic to publish to for the second stream.",
        ),
        "camera1/binning": DeclareLaunchArgument(
            "stream0/camera1/binning",
            default_value=TextSubstitution(text="1"),
            description="Binning for the second stream.",
        ),
        "camera1/exposure_time_us": DeclareLaunchArgument(
            "stream0/camera1/exposure_time_us",
            default_value=TextSubstitution(text="10000"),
            description="Exposure time for the second stream.",
        ),
        "camera1/image_size": DeclareLaunchArgument(
            "stream0/camera1/image_size",
            default_value=TextSubstitution(text="[640,480]"),
            description="Image size for the second stream.",
        ),
    }

    stream1_camera_args = {
        "camera0/identifier": DeclareLaunchArgument(
            "stream1/camera0/identifier",
            default_value=TextSubstitution(text=".*simulated.*uniform random.*"),
            description="Camera descriptor for the first stream.",
        ),
        "camera0/topic": DeclareLaunchArgument(
            "stream1/camera0/topic",
            default_value=TextSubstitution(text="stream0"),
            description="Topic to publish to for the first stream.",
        ),
        "camera0/binning": DeclareLaunchArgument(
            "stream1/camera0/binning",
            default_value=TextSubstitution(text="1"),
            description="Binning for the first stream.",
        ),
        "camera0/exposure_time_us": DeclareLaunchArgument(
            "stream1/camera0/exposure_time_us",
            default_value=TextSubstitution(text="10000"),
            description="Exposure time for the first stream.",
        ),
        "camera0/image_size": DeclareLaunchArgument(
            "stream1/camera0/image_size",
            default_value=TextSubstitution(text="[640,480]"),
            description="Image size for the first stream.",
        ),
        "camera1/identifier": DeclareLaunchArgument(
            "stream1/camera1/identifier",
            default_value=TextSubstitution(text=".*simulated.*radial sin.*"),
            description="Camera descriptor for the second stream.",
        ),
        "camera1/topic": DeclareLaunchArgument(
            "stream1/camera1/topic",
            default_value=TextSubstitution(text="stream1"),
            description="Topic to publish to for the second stream.",
        ),
        "camera1/binning": DeclareLaunchArgument(
            "stream1/camera1/binning",
            default_value=TextSubstitution(text="1"),
            description="Binning for the second stream.",
        ),
        "camera1/exposure_time_us": DeclareLaunchArgument(
            "stream1/camera1/exposure_time_us",
            default_value=TextSubstitution(text="10000"),
            description="Exposure time for the second stream.",
        ),
        "camera1/image_size": DeclareLaunchArgument(
            "stream1/camera1/image_size",
            default_value=TextSubstitution(text="[640,480]"),
            description="Image size for the second stream.",
        ),
    }

    stream0_launch_args = {
        "namespace": "acquire/first",
        "log_level": LaunchConfiguration("log_level"),
    }
    stream0_launch_args.update(
        {k: LaunchConfiguration("stream0/" + k) for k in stream0_camera_args.keys()}
    )

    stream1_launch_args = {
        "namespace": "acquire/second",
        "log_level": LaunchConfiguration("log_level"),
    }
    stream1_launch_args.update(
        {k: LaunchConfiguration("stream1/" + k) for k in stream1_camera_args.keys()}
    )

    return LaunchDescription(
        [
            log_level_arg,
            keep_last_arg,
        ]
        + list(stream0_camera_args.values())
        + list(stream1_camera_args.values())
        + [
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
                    k.replace("stream0/", ""): v for k, v in stream0_launch_args.items()
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
                    k.replace("stream1/", ""): v for k, v in stream1_launch_args.items()
                }.items(),
            ),
        ]
    )
