"""Launch a circle-drawing node against an already running simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        ("radius", "0.07"), ("num_points", "72"), ("center_x", "0.30"), ("center_y", "0.0"),
        ("draw_height", "0.30"), ("lift_height", "0.36"), ("frame_id", "base_link"),
    ]
    node = Node(
        package="ur_drawing", executable="circle_drawer", output="screen",
        parameters=[{name: LaunchConfiguration(name) for name, _ in arguments}],
    )
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, default in arguments]
        + [SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"), node]
    )
