"""Draw manually defined normalized A-Z strokes using MoveIt Cartesian paths."""

from __future__ import annotations

from typing import Sequence

import rclpy

from ur_drawing.drawing_common import CartesianDrawingNode, Point2D

# Every glyph fits in [-0.5, 0.5] in both axes.  A glyph is a list of
# disconnected pen-down strokes; travel between them is explicitly lifted.
LETTER_STROKES: dict[str, list[list[Point2D]]] = {
    "A": [[(-.5, -.5), (0, .5), (.5, -.5)], [(-.25, 0), (.25, 0)]],
    "B": [[(-.5, -.5), (-.5, .5), (.18, .5), (.5, .28), (.18, 0), (-.5, 0), (.22, 0), (.5, -.28), (.18, -.5), (-.5, -.5)]],
    "C": [[(.5, .35), (.25, .5), (-.25, .5), (-.5, .25), (-.5, -.25), (-.25, -.5), (.25, -.5), (.5, -.35)]],
    "D": [[(-.5, -.5), (-.5, .5), (.12, .5), (.5, .2), (.5, -.2), (.12, -.5), (-.5, -.5)]],
    "E": [[(.5, .5), (-.5, .5), (-.5, -.5), (.5, -.5)], [(-.5, 0), (.22, 0)]],
    "F": [[(-.5, -.5), (-.5, .5), (.5, .5)], [(-.5, 0), (.22, 0)]],
    "G": [[(.5, .32), (.22, .5), (-.25, .5), (-.5, .25), (-.5, -.25), (-.25, -.5), (.3, -.5), (.5, -.3), (.5, 0), (.05, 0)]],
    "H": [[(-.5, -.5), (-.5, .5)], [(.5, -.5), (.5, .5)], [(-.5, 0), (.5, 0)]],
    "I": [[(-.5, .5), (.5, .5)], [(0, .5), (0, -.5)], [(-.5, -.5), (.5, -.5)]],
    "J": [[(-.5, .5), (.5, .5)], [(0, .5), (0, -.3), (-.2, -.5), (-.5, -.3)]],
    "K": [[(-.5, -.5), (-.5, .5)], [(.5, .5), (-.5, 0), (.5, -.5)]],
    "L": [[(-.5, .5), (-.5, -.5), (.5, -.5)]],
    "M": [[(-.5, -.5), (-.5, .5), (0, -.05), (.5, .5), (.5, -.5)]],
    "N": [[(-.5, -.5), (-.5, .5), (.5, -.5), (.5, .5)]],
    "O": [[(-.25, -.5), (-.5, -.25), (-.5, .25), (-.25, .5), (.25, .5), (.5, .25), (.5, -.25), (.25, -.5), (-.25, -.5)]],
    "P": [[(-.5, -.5), (-.5, .5), (.2, .5), (.5, .25), (.2, 0), (-.5, 0)]],
    "Q": [[(-.25, -.5), (-.5, -.25), (-.5, .25), (-.25, .5), (.25, .5), (.5, .25), (.5, -.25), (.25, -.5), (-.25, -.5)], [(.12, -.12), (.5, -.5)]],
    "R": [[(-.5, -.5), (-.5, .5), (.18, .5), (.5, .25), (.18, 0), (-.5, 0)], [(.05, 0), (.5, -.5)]],
    "S": [[(.5, .35), (.25, .5), (-.25, .5), (-.5, .25), (-.25, 0), (.25, 0), (.5, -.25), (.25, -.5), (-.25, -.5), (-.5, -.35)]],
    "T": [[(-.5, .5), (.5, .5)], [(0, .5), (0, -.5)]],
    "U": [[(-.5, .5), (-.5, -.25), (-.25, -.5), (.25, -.5), (.5, -.25), (.5, .5)]],
    "V": [[(-.5, .5), (0, -.5), (.5, .5)]],
    "W": [[(-.5, .5), (-.3, -.5), (0, .05), (.3, -.5), (.5, .5)]],
    "X": [[(-.5, .5), (.5, -.5)], [(.5, .5), (-.5, -.5)]],
    "Y": [[(-.5, .5), (0, 0), (.5, .5)], [(0, 0), (0, -.5)]],
    "Z": [[(-.5, .5), (.5, .5), (-.5, -.5), (.5, -.5)]],
}


def generate_letter_strokes(letter: str) -> list[list[Point2D]]:
    """Return normalized strokes for an ASCII alphabet letter, case-insensitively."""
    normalized = letter.strip().upper()
    if len(normalized) != 1 or normalized not in LETTER_STROKES:
        raise ValueError("letter must be one English alphabet character A-Z")
    return LETTER_STROKES[normalized]


class LetterDrawer(CartesianDrawingNode):
    def __init__(self) -> None:
        super().__init__("letter_drawer")
        self.declare_parameter("letter", "A")
        self.declare_parameter("scale", 0.13)

    def run(self) -> bool:
        try:
            self.validate_common()
            letter = str(self.get_parameter("letter").value)
            scale = float(self.get_parameter("scale").value)
            if not 0.02 <= scale <= 0.18:
                raise ValueError("scale must be between 0.02 and 0.18 m")
            normalized_strokes = generate_letter_strokes(letter)
            strokes: Sequence[Sequence] = [
                self.poses_from_points([(scale * u, scale * v) for u, v in stroke], self.params.draw_height)
                for stroke in normalized_strokes
            ]
            self.get_logger().info(f"Drawing letter {letter.strip().upper()} with {len(strokes)} stroke(s)")
            if not self.wait_for_moveit():
                return False
            return self.execute_strokes(strokes)
        except ValueError as error:
            self.get_logger().error(f"Invalid letter drawing request: {error}")
            return False


def main() -> None:
    rclpy.init()
    node = LetterDrawer()
    try:
        if not node.run():
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
