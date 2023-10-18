from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    camera0_identifier_arg = DeclareLaunchArgument(
        "camera0/identifier",
        default_value=TextSubstitution(text=".*simulated.*uniform random.*"),
        description="Camera descriptor for the first stream.",
    )
    camera0_topic_arg = DeclareLaunchArgument(
        "camera0/topic",
        default_value=TextSubstitution(text="stream0"),
        description="Topic to publish to for the first stream.",
    )
    camera0_binning_arg = DeclareLaunchArgument(
        "camera0/binning",
        default_value=TextSubstitution(text="1"),
        description="Binning for the first stream.",
    )
    camera0_exposure_time_us_arg = DeclareLaunchArgument(
        "camera0/exposure_time_us",
        default_value=TextSubstitution(text="10000"),
        description="Exposure time for the first stream.",
    )
    camera0_image_size_arg = DeclareLaunchArgument(
        "camera0/image_size",
        default_value=TextSubstitution(text="640,480"),
        description="Image size for the first stream.",
    )
    camera1_identifier_arg = DeclareLaunchArgument(
        "camera1/identifier",
        default_value=TextSubstitution(text=".*simulated.*radial sin.*"),
        description="Camera descriptor for the second stream.",
    )
    camera1_topic_arg = DeclareLaunchArgument(
        "camera1/topic",
        default_value=TextSubstitution(text="stream1"),
        description="Topic to publish to for the second stream.",
    )
    camera1_binning_arg = DeclareLaunchArgument(
        "camera1/binning",
        default_value=TextSubstitution(text="1"),
        description="Binning for the second stream.",
    )
    camera1_exposure_time_us_arg = DeclareLaunchArgument(
        "camera1/exposure_time_us",
        default_value=TextSubstitution(text="10000"),
        description="Exposure time for the second stream.",
    )
    camera1_image_size_arg = DeclareLaunchArgument(
        "camera1/image_size",
        default_value=TextSubstitution(text="640,480"),
        description="Image size for the second stream.",
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

    streamer_node = Node(
        package="acquire",
        namespace=LaunchConfiguration("namespace"),
        executable="streamer",
        name="streamer",
        parameters=[
            {
                "camera0/identifier": ParameterValue(
                    LaunchConfiguration("camera0/identifier")
                ),
                "camera0/topic": ParameterValue(LaunchConfiguration("camera0/topic")),
                "camera0/binning": ParameterValue(
                    LaunchConfiguration("camera0/binning")
                ),
                "camera0/exposure_time_us": ParameterValue(
                    LaunchConfiguration("camera0/exposure_time_us")
                ),
                "camera0/image_size": ParameterValue(
                    LaunchConfiguration("camera0/image_size")
                ),
                "camera1/identifier": ParameterValue(
                    LaunchConfiguration("camera1/identifier")
                ),
                "camera1/topic": ParameterValue(LaunchConfiguration("camera1/topic")),
                "camera1/binning": ParameterValue(
                    LaunchConfiguration("camera1/binning")
                ),
                "camera1/exposure_time_us": ParameterValue(
                    LaunchConfiguration("camera1/exposure_time_us")
                ),
                "camera1/image_size": ParameterValue(
                    LaunchConfiguration("camera1/image_size")
                ),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            namespace_arg,
            log_level_arg,
            camera0_identifier_arg,
            camera0_topic_arg,
            camera0_binning_arg,
            camera0_exposure_time_us_arg,
            camera0_image_size_arg,
            camera1_identifier_arg,
            camera1_topic_arg,
            camera1_binning_arg,
            camera1_exposure_time_us_arg,
            camera1_image_size_arg,
            streamer_node,
        ]
    )
