# UR Drawing

`ur_drawing` is an independent ROS 2 Humble assignment package that draws manually-defined English alphabet letters and a Cartesian circle with a simulated UR3 or UR3e. It does not modify or replace any Universal Robots package.

## Architecture

```text
ur_drawing launch
  -> local compatibility launch for ur_simulation_gz (Gazebo Sim, robot, controllers)
  -> official ur_moveit_config launch (move_group and optional RViz)
  -> letter_drawer or circle_drawer
  -> MoveIt compute_cartesian_path service and execute_trajectory action
  -> joint_trajectory_controller in Gazebo
```

The package uses a local compatibility wrapper around the official Gazebo Sim composition.  It fixes local-container discovery by binding Ignition transport and ROS 2 discovery to loopback; it does not modify upstream repositories.

## Robot and Cartesian convention

- Default robot: `ur3e`; use `ur_type:=ur3` for UR3.
- Planning group: `ur_manipulator`.
- Reference frame: `base_link`.
- End-effector link: `tool0`.
- Drawing plane: XY plane at `draw_height` in metres; Z is raised to `lift_height` between pen strokes.
- The default fixed tool orientation is quaternion `(0, 1, 0, 0)`, which points tool Z down toward the drawing plane.

Letters are hand-authored polylines in normalized `[-0.5, 0.5]` coordinates. `scale` converts those coordinates to metres and adds `center_x`, `center_y`. All A–Z glyphs are present; lowercase input is normalized to uppercase. A circle samples `num_points + 1` points using `x=r*cos(theta)`, `y=r*sin(theta)`, including the final closing point.

## Dependencies and build

The workspace must already provide ROS 2 Humble, MoveIt 2, and the official `ur_description`, `ur_moveit_config`, and `ur_simulation_gz` packages.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur_drawing --symlink-install
source install/setup.bash
```

## Launch

Start only one launch at a time.

```bash
ros2 launch ur_drawing sim_moveit.launch.py
ros2 launch ur_drawing sim_moveit.launch.py ur_type:=ur3 gazebo_gui:=false launch_rviz:=false
ros2 launch ur_drawing draw_letter.launch.py letter:=A
ros2 launch ur_drawing draw_letter.launch.py letter:=m
ros2 launch ur_drawing draw_circle.launch.py
```

Useful drawing parameters:

```bash
ros2 launch ur_drawing draw_letter.launch.py letter:=B scale:=0.10 draw_height:=0.30 lift_height:=0.36 frame_id:=base_link
ros2 launch ur_drawing draw_circle.launch.py radius:=0.06 center_x:=0.30 center_y:=0.0 draw_height:=0.30 num_points:=72
```

`sim_moveit.launch.py` accepts `ur_type` (`ur3` or `ur3e`), `launch_rviz`, `gazebo_gui`, `world_file`, and the official safety/prefix arguments.

## Planning, execution, and visualization

Each stroke is executed as lift/travel, lower, draw, and lift segments. The node requests every segment from MoveIt with collision avoidance enabled, a configurable `eef_step` (default 0.008 m), and conservative velocity/acceleration scaling (0.15). It rejects a non-success MoveIt response or a Cartesian fraction below `required_fraction` (default 1.0), then never executes that partial trajectory.

The local RViz configuration includes the `drawing_markers` MarkerArray display automatically. Cyan lines are planned strokes; the currently executing stroke is orange.

## Known limitations

- The default Cartesian plane is a conservative starting pose, not a substitute for validating a specific robot posture. If MoveIt reports IK, collision, or fraction failure, no incomplete trajectory is executed; adjust the placement only after inspecting RViz and the robot state.
- The package targets the official single-robot, unprefixed simulation configuration. A nonempty joint prefix requires matching upstream controller configuration and is not supported by the drawing node defaults.
- The nodes wait up to 60 seconds for MoveIt; they deliberately do not attempt to start or repair missing controllers.
