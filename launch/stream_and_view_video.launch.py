from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    camera0_launch_arg = DeclareLaunchArgument(
        "camera0",
        default_value=TextSubstitution(text=".*simulated.*uniform random.*"),
        description="Camera descriptor for the first stream.",
    )
    camera1_launch_arg = DeclareLaunchArgument(
        "camera1",
        default_value=TextSubstitution(text=".*simulated.*radial sin.*"),
        description="Camera descriptor for the second stream.",
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
                "camera0": ParameterValue(LaunchConfiguration("camera0")),
                "camera1": ParameterValue(LaunchConfiguration("camera1")),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    viewer_node = Node(
        package="acquire",
        namespace=LaunchConfiguration("namespace"),
        executable="viewer",
        name="viewer",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            camera0_launch_arg,
            camera1_launch_arg,
            namespace_arg,
            log_level_arg,
            streamer_node,
            viewer_node,
        ]
    )
