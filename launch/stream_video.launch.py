from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
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

    streamer_node = Node(
        package="acquire",
        namespace=LaunchConfiguration("namespace"),
        executable="streamer",
        name="streamer",
        parameters=[
            {p: ParameterValue(LaunchConfiguration(p)) for p in camera_args.keys()}
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            namespace_arg,
            log_level_arg,
        ]
        + list(camera_args.values())
        + [
            streamer_node,
        ]
    )
