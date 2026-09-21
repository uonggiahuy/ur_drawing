"""Shared geometry, visualization, and MoveIt service/action helpers."""

from __future__ import annotations

from copy import deepcopy
import math
import time
from dataclasses import dataclass
from typing import Iterable, Sequence

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

Point2D = tuple[float, float]


@dataclass(frozen=True)
class DrawingParameters:
    """Frame, plane, orientation, and conservative Cartesian planning settings."""

    frame_id: str
    planning_group: str
    end_effector_link: str
    center_x: float
    center_y: float
    draw_height: float
    lift_height: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    eef_step: float
    jump_threshold: float
    velocity_scale: float
    acceleration_scale: float
    required_fraction: float
    joint_state_max_age: float
    joint_state_timeout: float
    lift_pose_tolerance: float


class CartesianDrawingNode(Node):
    """Plans Cartesian segments with MoveIt and executes only complete solutions.

    This intentionally uses ROS service/action interfaces rather than moveit_py:
    MoveIt Python bindings are not supplied by this Humble workspace.
    """

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self._declare_common_parameters()
        self.params = DrawingParameters(
            frame_id=str(self.get_parameter("frame_id").value),
            planning_group=str(self.get_parameter("planning_group").value),
            end_effector_link=str(self.get_parameter("end_effector_link").value),
            center_x=float(self.get_parameter("center_x").value),
            center_y=float(self.get_parameter("center_y").value),
            draw_height=float(self.get_parameter("draw_height").value),
            lift_height=float(self.get_parameter("lift_height").value),
            orientation_x=float(self.get_parameter("orientation_x").value),
            orientation_y=float(self.get_parameter("orientation_y").value),
            orientation_z=float(self.get_parameter("orientation_z").value),
            orientation_w=float(self.get_parameter("orientation_w").value),
            eef_step=float(self.get_parameter("eef_step").value),
            jump_threshold=float(self.get_parameter("jump_threshold").value),
            velocity_scale=float(self.get_parameter("velocity_scale").value),
            acceleration_scale=float(self.get_parameter("acceleration_scale").value),
            required_fraction=float(self.get_parameter("required_fraction").value),
            joint_state_max_age=float(self.get_parameter("joint_state_max_age").value),
            joint_state_timeout=float(self.get_parameter("joint_state_timeout").value),
            lift_pose_tolerance=float(self.get_parameter("lift_pose_tolerance").value),
        )
        self._cartesian_client = self.create_client(GetCartesianPath, "compute_cartesian_path")
        self._fk_client = self.create_client(GetPositionFK, "compute_fk")
        self._ik_client = self.create_client(GetPositionIK, "compute_ik")
        self._motion_plan_client = self.create_client(GetMotionPlan, "plan_kinematic_path")
        self._execute_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self._latest_joint_state: JointState | None = None
        self._joint_state_received_at: float | None = None
        self._joint_state_sequence = 0
        self._joint_state_sub = self.create_subscription(
            JointState, "joint_states", self._on_joint_state, qos_profile_sensor_data
        )
        self._marker_pub = self.create_publisher(
            MarkerArray,
            "drawing_markers",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE),
        )

    def _declare_common_parameters(self) -> None:
        # The approved UR MoveIt configuration defines ur_manipulator, base_link, and tool0.
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("planning_group", "ur_manipulator")
        self.declare_parameter("end_effector_link", "tool0")
        self.declare_parameter("center_x", 0.30)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("draw_height", 0.30)
        self.declare_parameter("lift_height", 0.36)
        # tool0 Z points down toward the XY drawing plane in base_link.
        self.declare_parameter("orientation_x", 0.0)
        self.declare_parameter("orientation_y", 1.0)
        self.declare_parameter("orientation_z", 0.0)
        self.declare_parameter("orientation_w", 0.0)
        self.declare_parameter("eef_step", 0.008)
        self.declare_parameter("jump_threshold", 2.0)
        self.declare_parameter("velocity_scale", 0.15)
        self.declare_parameter("acceleration_scale", 0.15)
        self.declare_parameter("required_fraction", 1.0)
        self.declare_parameter("joint_state_max_age", 1.0)
        self.declare_parameter("joint_state_timeout", 2.0)
        self.declare_parameter("lift_pose_tolerance", 0.02)
        self.declare_parameter("wait_timeout", 60.0)

    def validate_common(self) -> None:
        p = self.params
        if not p.frame_id or not p.planning_group or not p.end_effector_link:
            raise ValueError("frame_id, planning_group, and end_effector_link must be non-empty")
        if not 0.001 <= p.eef_step <= 0.02:
            raise ValueError("eef_step must be between 0.001 and 0.02 m")
        if p.jump_threshold <= 0.0:
            raise ValueError("jump_threshold must be positive")
        if not 0.0 < p.velocity_scale <= 1.0 or not 0.0 < p.acceleration_scale <= 1.0:
            raise ValueError("velocity_scale and acceleration_scale must be in (0, 1]")
        if not 0.99 <= p.required_fraction <= 1.0:
            raise ValueError("required_fraction must be in [0.99, 1.0]")
        if p.joint_state_max_age <= 0.0 or p.joint_state_timeout <= 0.0:
            raise ValueError("joint-state age and timeout must be positive")
        if p.lift_pose_tolerance <= 0.0:
            raise ValueError("lift_pose_tolerance must be positive")
        if p.lift_height - p.draw_height < 0.01:
            raise ValueError("lift_height must be at least 0.01 m above draw_height")
        for label, value in (("center_x", p.center_x), ("center_y", p.center_y),
                             ("draw_height", p.draw_height), ("lift_height", p.lift_height)):
            if not math.isfinite(value):
                raise ValueError(f"{label} must be finite")
        if not (0.05 <= p.center_x <= 0.50 and -0.35 <= p.center_y <= 0.35
                and 0.10 <= p.draw_height <= 0.50 and p.lift_height <= 0.55):
            raise ValueError("drawing center/heights are outside this conservative UR3/UR3e workspace")

    def pose(self, x: float, y: float, z: float) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.x = self.params.orientation_x
        pose.orientation.y = self.params.orientation_y
        pose.orientation.z = self.params.orientation_z
        pose.orientation.w = self.params.orientation_w
        return pose

    def poses_from_points(self, points: Iterable[Point2D], z: float) -> list[Pose]:
        return [self.pose(self.params.center_x + x, self.params.center_y + y, z) for x, y in points]

    def _on_joint_state(self, message: JointState) -> None:
        self._latest_joint_state = deepcopy(message)
        self._joint_state_received_at = time.monotonic()
        self._joint_state_sequence += 1

    def _current_robot_state(self, log_error: bool = True) -> RobotState | None:
        joint_state = self._latest_joint_state
        received_at = self._joint_state_received_at
        if joint_state is None or received_at is None:
            if log_error:
                self.get_logger().error("No /joint_states message has been received")
            return None
        if not joint_state.name or len(joint_state.name) != len(joint_state.position):
            if log_error:
                self.get_logger().error("Latest /joint_states message has invalid joint names or positions")
            return None
        age = time.monotonic() - received_at
        if age > self.params.joint_state_max_age:
            if log_error:
                self.get_logger().error(
                    f"Latest /joint_states is stale ({age:.3f} s > {self.params.joint_state_max_age:.3f} s)"
                )
            return None
        state = RobotState()
        state.joint_state = deepcopy(joint_state)
        state.is_diff = False
        return state

    def _wait_for_joint_state_after(self, sequence: int, label: str) -> bool:
        deadline = time.monotonic() + self.params.joint_state_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_state_sequence > sequence and self._current_robot_state(log_error=False) is not None:
                return True
        self.get_logger().error(
            f"{label}: no fresh valid /joint_states message after execution; refusing to plan from an unknown state"
        )
        return False

    def _current_eef_pose(self, state: RobotState, label: str) -> Pose | None:
        request = GetPositionFK.Request()
        request.header.frame_id = self.params.frame_id
        request.fk_link_names = [self.params.end_effector_link]
        request.robot_state = state
        future = self._fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if (
            response is None
            or response.error_code.val != MoveItErrorCodes.SUCCESS
            or not response.pose_stamped
        ):
            self.get_logger().error(f"{label}: could not obtain the current end-effector pose from MoveIt FK")
            return None
        return response.pose_stamped[0].pose

    @staticmethod
    def _format_pose(pose: Pose) -> str:
        return (
            f"position=({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}), "
            f"orientation=({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, "
            f"{pose.orientation.z:.4f}, {pose.orientation.w:.4f})"
        )

    def _log_cartesian_start(
        self, label: str, state: RobotState, actual_pose: Pose, target_pose: Pose, expected_lift: Pose | None
    ) -> None:
        stamp = state.joint_state.header.stamp
        self.get_logger().info(
            f"{label}: /joint_states stamp={stamp.sec}.{stamp.nanosec:09d}; frame={self.params.frame_id}; "
            f"actual EEF {self._format_pose(actual_pose)}; target {self._format_pose(target_pose)}"
        )
        if expected_lift is not None:
            distance = math.dist(
                (actual_pose.position.x, actual_pose.position.y, actual_pose.position.z),
                (expected_lift.position.x, expected_lift.position.y, expected_lift.position.z),
            )
            self.get_logger().info(
                f"{label}: expected lift {self._format_pose(expected_lift)}; lift-position error={distance:.4f} m"
            )
            if distance > self.params.lift_pose_tolerance:
                self.get_logger().warning(
                    f"{label}: actual EEF differs from expected lift pose by {distance:.4f} m "
                    f"(tolerance {self.params.lift_pose_tolerance:.4f} m)"
                )

    def wait_for_moveit(self) -> bool:
        deadline = time.monotonic() + float(self.get_parameter("wait_timeout").value)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            service_ready = (
                self._cartesian_client.wait_for_service(timeout_sec=1.0)
                and self._fk_client.wait_for_service(timeout_sec=1.0)
                and self._ik_client.wait_for_service(timeout_sec=1.0)
                and self._motion_plan_client.wait_for_service(timeout_sec=1.0)
            )
            action_ready = self._execute_client.wait_for_server(timeout_sec=1.0)
            if service_ready and action_ready and self._current_robot_state(log_error=False) is not None:
                return True
        self.get_logger().error("MoveIt services, execution action, or fresh /joint_states were unavailable")
        return False

    def plan_and_execute_cartesian(
        self, label: str, waypoints: Sequence[Pose], expected_lift: Pose | None = None
    ) -> bool:
        if not waypoints:
            self.get_logger().error(f"{label} has no waypoints")
            return False
        state = self._current_robot_state()
        if state is None:
            self.get_logger().error(f"{label}: refusing Cartesian planning without a current robot state")
            return False
        actual_pose = self._current_eef_pose(state, label)
        if actual_pose is None:
            return False
        self._log_cartesian_start(label, state, actual_pose, waypoints[0], expected_lift)
        request = GetCartesianPath.Request()
        request.header.frame_id = self.params.frame_id
        request.start_state = state
        request.group_name = self.params.planning_group
        request.link_name = self.params.end_effector_link
        request.waypoints = list(waypoints)
        request.max_step = self.params.eef_step
        request.jump_threshold = self.params.jump_threshold
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.params.velocity_scale
        request.max_acceleration_scaling_factor = self.params.acceleration_scale
        future = self._cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error(f"{label}: Cartesian-path request failed")
            return False
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"{label}: MoveIt error code {response.error_code.val}")
            return False
        self.get_logger().info(f"{label}: Cartesian fraction={response.fraction:.3f}")
        if response.fraction < self.params.required_fraction:
            self.get_logger().error(
                f"{label}: rejected incomplete Cartesian path "
                f"(fraction {response.fraction:.3f} < {self.params.required_fraction:.3f})"
            )
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        send_future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{label}: trajectory execution goal was rejected")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = result.result.error_code.val if result is not None else "no result"
            self.get_logger().error(f"{label}: execution failed ({code})")
            return False
        if not self._wait_for_joint_state_after(self._joint_state_sequence, label):
            return False
        self.get_logger().info(f"{label}: executed complete Cartesian trajectory")
        return True

    def plan_and_execute_joint_space_to_pose(self, label: str, target: Pose) -> bool:
        """Use collision-checked joint-space planning for pen-up repositioning."""
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = self.params.planning_group
        ik_request.ik_request.ik_link_name = self.params.end_effector_link
        ik_request.ik_request.pose_stamped = PoseStamped()
        ik_request.ik_request.pose_stamped.header.frame_id = self.params.frame_id
        ik_request.ik_request.pose_stamped.pose = target
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.timeout.sec = 2
        ik_future = self._ik_client.call_async(ik_request)
        rclpy.spin_until_future_complete(self, ik_future)
        ik_response = ik_future.result()
        if ik_response is None or ik_response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"{label}: collision-checked IK failed")
            return False
        constraints = Constraints()
        for name, position in zip(ik_response.solution.joint_state.name, ik_response.solution.joint_state.position):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = self.params.planning_group
        motion.goal_constraints = [constraints]
        motion.num_planning_attempts = 5
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = self.params.velocity_scale
        motion.max_acceleration_scaling_factor = self.params.acceleration_scale
        future = self._motion_plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"{label}: collision-checked joint-space plan failed")
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.motion_plan_response.trajectory
        send_future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{label}: trajectory execution goal was rejected")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = result.result.error_code.val if result is not None else "no result"
            self.get_logger().error(f"{label}: execution failed ({code})")
            return False
        if not self._wait_for_joint_state_after(self._joint_state_sequence, label):
            return False
        self.get_logger().info(f"{label}: executed collision-checked joint-space trajectory")
        return True

    def execute_strokes(self, strokes: Sequence[Sequence[Pose]]) -> bool:
        """Lift, travel, lower, draw, and lift after every disconnected stroke."""
        for index, stroke in enumerate(strokes, start=1):
            if len(stroke) < 2:
                self.get_logger().error(f"stroke {index} needs at least two points")
                return False
            start, end = stroke[0], stroke[-1]
            start_lift = self.pose(start.position.x, start.position.y, self.params.lift_height)
            end_lift = self.pose(end.position.x, end.position.y, self.params.lift_height)
            self.publish_markers(strokes, current_stroke=index - 1)
            if not self.plan_and_execute_joint_space_to_pose(f"stroke {index} lift/travel", start_lift):
                return False
            if not self.plan_and_execute_cartesian(f"stroke {index} lower", [start], expected_lift=start_lift):
                return False

            # Sharp corners (such as the apex of A) are planned as individual
            # straight segments. A single Cartesian request through a corner
            # can have incomplete IK even when both legs are valid.
            for segment_index in range(len(stroke) - 1):
                if not self.plan_and_execute_cartesian(
                    f"stroke {index} draw segment {segment_index + 1}",
                    stroke[segment_index : segment_index + 2],
                ):
                    return False

            if not self.plan_and_execute_cartesian(f"stroke {index} lift", [end_lift]):
                return False
        self.publish_markers(strokes, current_stroke=None)
        return True

    def publish_markers(self, strokes: Sequence[Sequence[Pose]], current_stroke: int | None) -> None:
        markers = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)
        for index, stroke in enumerate(strokes):
            marker = Marker()
            marker.header.frame_id = self.params.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "drawing_path"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.006
            marker.color.a = 1.0
            if index == current_stroke:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.15, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.7, 1.0
            marker.points = [Point(x=p.position.x, y=p.position.y, z=p.position.z) for p in stroke]
            markers.markers.append(marker)
        self._marker_pub.publish(markers)
