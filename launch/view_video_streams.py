from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution


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

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text=str("WARN")),
        description="Logging level",
    )

    streamer_node = Node(
        package="acquire",
        namespace="acquire",
        executable="streamer",
        name="streamer",
        parameters=[
            {
                "camera0": LaunchConfiguration("camera0"),
                "camera1": LaunchConfiguration("camera1"),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    viewer_node = Node(
        package="acquire",
        namespace="acquire",
        executable="viewer",
        name="viewer",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            camera0_launch_arg,
            camera1_launch_arg,
            log_level_arg,
            streamer_node,
            viewer_node,
        ]
    )
