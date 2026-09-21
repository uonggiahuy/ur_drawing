"""Draw a sampled, closed Cartesian circle using MoveIt."""

from __future__ import annotations

import math

import rclpy

from ur_drawing.drawing_common import CartesianDrawingNode, Point2D


def generate_circle_points(radius: float, num_points: int) -> list[Point2D]:
    """Generate a closed circle centered at the origin in the drawing plane."""
    points = [
        (radius * math.cos(2.0 * math.pi * index / num_points),
         radius * math.sin(2.0 * math.pi * index / num_points))
        for index in range(num_points)
    ]
    points.append(points[0])
    return points


class CircleDrawer(CartesianDrawingNode):
    def __init__(self) -> None:
        super().__init__("circle_drawer")
        self.declare_parameter("radius", 0.07)
        self.declare_parameter("num_points", 72)

    def run(self) -> bool:
        try:
            self.validate_common()
            radius = float(self.get_parameter("radius").value)
            num_points = int(self.get_parameter("num_points").value)
            if not 0.02 <= radius <= 0.15:
                raise ValueError("radius must be between 0.02 and 0.15 m")
            if not 8 <= num_points <= 200:
                raise ValueError("num_points must be between 8 and 200")
            # Circle extrema must remain inside the conservative base_link workspace.
            if not (0.05 <= self.params.center_x - radius and self.params.center_x + radius <= 0.50
                    and -0.35 <= self.params.center_y - radius and self.params.center_y + radius <= 0.35):
                raise ValueError("circle extrema are outside this conservative UR3/UR3e workspace")
            stroke = self.poses_from_points(generate_circle_points(radius, num_points), self.params.draw_height)
            self.get_logger().info(f"Drawing closed circle: radius {radius:.3f} m, {num_points} segments")
            if not self.wait_for_moveit():
                return False
            return self.execute_strokes([stroke])
        except ValueError as error:
            self.get_logger().error(f"Invalid circle drawing request: {error}")
            return False


def main() -> None:
    rclpy.init()
    node = CircleDrawer()
    try:
        if not node.run():
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
