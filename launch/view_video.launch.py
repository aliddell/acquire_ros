from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


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

    topic_arg = DeclareLaunchArgument(
        "topic",
        default_value=TextSubstitution(text="stream0"),
        description="Topic to subscribe to.",
    )

    viewer_node = Node(
        package="acquire",
        namespace=LaunchConfiguration("namespace"),
        executable="viewer",
        name="viewer",
        parameters=[
            {
                "keep_last": ParameterValue(LaunchConfiguration("keep_last")),
                "topic": ParameterValue(LaunchConfiguration("topic")),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            namespace_arg,
            log_level_arg,
            keep_last_arg,
            topic_arg,
            viewer_node,
        ]
    )
