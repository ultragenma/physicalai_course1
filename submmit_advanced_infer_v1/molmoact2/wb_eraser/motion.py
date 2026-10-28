"""ROS-independent workspace checks and execution of board-plane strokes.

Both the real MoveL client and the offline simulator use this sequence. A
client supplies ``workspace`` and ``move_to(xyz, **kwargs) -> bool``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .geometry import BoardFrame
from .planner import Stroke

TOOL_DOWN = (0.0, 0.0, 0.0, 1.0)


class WorkspaceError(RuntimeError):
    pass


@dataclass
class WorkspaceBox:
    """Allowed end-effector positions in base_link, in meters."""

    x: tuple[float, float] = (0.10, 0.62)
    y: tuple[float, float] = (-0.65, 0.20)
    z: tuple[float, float] = (0.65, 1.60)

    def contains(self, p) -> bool:
        x, y, z = p
        return (self.x[0] <= x <= self.x[1] and self.y[0] <= y <= self.y[1]
                and self.z[0] <= z <= self.z[1])

    @classmethod
    def for_arm(cls, arm: str) -> "WorkspaceBox":
        if arm not in ("r", "l"):
            raise ValueError("arm must be 'r' or 'l'")
        return cls(y=(-0.20, 0.65)) if arm == "l" else cls()


@dataclass
class ExecutionReport:
    waypoints: int = 0
    reached: int = 0
    aborted: bool = False
    reason: str = ""
    per_target: dict[int, int] = field(default_factory=dict)


def execute_strokes(client, board: BoardFrame, strokes: list[Stroke],
                    quat=TOOL_DOWN, tool_offset: float = 0.0, dry_run: bool = False,
                    on_waypoint=None, max_press_depth: float = 0.015) -> ExecutionReport:
    """Check the entire plan before motion, then stop on the first failed move.

    Positive ``tool_offset`` pulls the EE back along the board normal. Press
    and wipe segments wait by time because a contact target may be unreachable.
    A timed completion is not a contact-force or tracking measurement.
    """
    rep = ExecutionReport(waypoints=sum(len(s.waypoints) for s in strokes))
    if (not math.isfinite(tool_offset) or tool_offset < 0
            or not math.isfinite(max_press_depth) or not 0 <= max_press_depth <= 0.015):
        rep.aborted, rep.reason = True, "invalid tool offset or maximum press depth"
        return rep
    poses = []
    for stroke in strokes:
        for wp in stroke.waypoints:
            if (not all(math.isfinite(v) for v in (wp.u, wp.v, wp.depth, wp.speed))
                    or wp.speed <= 0 or wp.depth > max_press_depth):
                rep.aborted, rep.reason = True, f"invalid {wp.kind} waypoint for target {stroke.target_id}"
                return rep
            p = board.to_base(wp.u, wp.v, wp.depth - tool_offset)
            if not client.workspace.contains(p):
                rep.aborted, rep.reason = True, f"{p.round(3).tolist()} outside workspace"
                return rep
            poses.append((stroke, wp, p))
    if dry_run:
        return rep
    for stroke, wp, p in poses:
        if on_waypoint:
            on_waypoint(stroke, wp, p)
        try:
            ok = client.move_to(p, quat=quat, speed=wp.speed,
                                wait="time" if wp.kind in ("press", "wipe") else "pose",
                                kind=wp.kind)
        except WorkspaceError as exc:
            rep.aborted, rep.reason = True, str(exc)
            return rep
        if not ok:
            rep.aborted, rep.reason = True, f"{wp.kind} failed for target {stroke.target_id}"
            return rep
        rep.reached += 1
        if wp.kind == "retreat":
            rep.per_target[stroke.target_id] = rep.per_target.get(stroke.target_id, 0) + 1
    return rep
