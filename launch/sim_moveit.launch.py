"""Compose UR Gazebo Sim and MoveIt launches for drawing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # These are the actual argument names accepted by the official Humble launches.
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world_file = LaunchConfiguration("world_file")
    prefix = LaunchConfiguration("prefix")

    rviz_value = SetLaunchConfiguration("ur_drawing_launch_rviz", LaunchConfiguration("launch_rviz"))
    gazebo_gui_value = SetLaunchConfiguration("ur_drawing_gazebo_gui", LaunchConfiguration("gazebo_gui"))

    sim_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_drawing"), "launch", "ur_sim_control_compat.launch.py"
        ])),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "runtime_config_package": "ur_simulation_gz",
            "controllers_file": "ur_controllers.yaml",
            "description_package": "ur_description",
            "description_file": "ur.urdf.xacro",
            "prefix": prefix,
            "start_joint_controller": "true",
            "initial_joint_controller": "joint_trajectory_controller",
            # MoveIt supplies the RViz instance; do not launch the description-only RViz.
            "launch_rviz": "false",
            "gazebo_gui": LaunchConfiguration("ur_drawing_gazebo_gui"),
            "world_file": world_file,
            "world_name": "empty",
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_drawing"), "launch", "ur_moveit_with_markers.launch.py"
        ])),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "description_package": "ur_description",
            "description_file": "ur.urdf.xacro",
            "moveit_config_package": "ur_moveit_config",
            "moveit_config_file": "ur.srdf.xacro",
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz": LaunchConfiguration("ur_drawing_launch_rviz"),
            "launch_servo": "false",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e", choices=["ur3", "ur3e"],
                              description="Simulated UR model."),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch MoveIt RViz."),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Launch Gazebo GUI (false keeps server running)."),
        DeclareLaunchArgument(
            "world_file", default_value="empty.sdf",
            description="Gazebo Sim world file; use a Gazebo Sim .sdf world.",
        ),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("prefix", default_value='""', description="Joint-name prefix; leave default for one robot."),
        rviz_value,
        gazebo_gui_value,
        sim_control,
        moveit,
    ])
