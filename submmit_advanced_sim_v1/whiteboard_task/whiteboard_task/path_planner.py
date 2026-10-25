"""Geometry-only boustrophedon path planner for a rectangular whiteboard.

This module produces tool-centre points only. A robot-specific motion planner
must turn them into collision-checked trajectories and enforce contact force.
"""
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


def raster_path(
    lower_left: Point3,
    upper_right: Point3,
    row_spacing_m: float,
    edge_margin_m: float = 0.02,
) -> list[Point3]:
    """Return alternating horizontal strokes inside an axis-aligned board.

    Coordinates must already be expressed in the selected board-aligned frame.
    The caller validates reachability, collision clearance and contact approach.
    """
    if row_spacing_m <= 0:
        raise ValueError("row_spacing_m must be positive")
    if upper_right.x <= lower_left.x or upper_right.y <= lower_left.y:
        raise ValueError("upper_right must be above/right of lower_left")
    left, right = lower_left.x + edge_margin_m, upper_right.x - edge_margin_m
    bottom, top = lower_left.y + edge_margin_m, upper_right.y - edge_margin_m
    if left >= right or bottom >= top:
        raise ValueError("edge margin leaves no drawable board area")
    rows = max(1, int((top - bottom) / row_spacing_m) + 1)
    y_values = [min(bottom + index * row_spacing_m, top) for index in range(rows)]
    # Include the upper edge even when the board height is not an integer
    # multiple of spacing; otherwise a thin strip can remain unvisited.
    if y_values[-1] < top:
        y_values.append(top)
    points: list[Point3] = []
    for index, y in enumerate(y_values):
        endpoints = (Point3(left, y, lower_left.z), Point3(right, y, lower_left.z))
        points.extend(endpoints if index % 2 == 0 else endpoints[::-1])
    return points


def format_csv(points: Iterable[Point3]) -> str:
    return "\n".join(f"{p.x:.6f},{p.y:.6f},{p.z:.6f}" for p in points)
