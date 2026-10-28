"""Wipe stroke planning on the board plane.

Input: erase targets as rectangles in board coordinates (meters).
Output: a list of strokes, each a sequence of waypoints in board coordinates
``(u, v, depth)`` with a motion kind and a speed. ``control.execute_strokes``
converts them to base_link poses with a :class:`geometry.BoardFrame`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class EraseTarget:
    """Axis-aligned rectangle on the board to erase (board coords, meters)."""

    id: int
    u0: float
    v0: float
    u1: float
    v1: float
    area_px: int = 0
    label: str = ""

    @property
    def width(self) -> float:
        return self.u1 - self.u0

    @property
    def height(self) -> float:
        return self.v1 - self.v0

    def expanded(self, margin: float) -> "EraseTarget":
        return EraseTarget(self.id, self.u0 - margin, self.v0 - margin,
                           self.u1 + margin, self.v1 + margin, self.area_px, self.label)


@dataclass
class Waypoint:
    u: float
    v: float
    depth: float
    kind: str   # approach | press | wipe | retreat
    speed: float


@dataclass
class Stroke:
    target_id: int
    waypoints: list[Waypoint] = field(default_factory=list)


@dataclass
class PlannerConfig:
    cleaner_u: float = 0.05       # cleaner footprint along the stroke (m)
    cleaner_v: float = 0.03       # cleaner footprint across the stroke (m)
    overlap: float = 0.35        # band overlap ratio (0..0.9)
    margin: float = 0.015        # grow each target by this much (m)
    approach_dist: float = 0.03  # hover distance in front of the board (m)
    press_depth: float = 0.004   # commanded penetration into the board (m)
    max_press_depth: float = 0.012
    wipe_speed: float = 0.12     # m/s
    travel_speed: float = 0.25   # m/s
    passes: int = 2              # back-and-forth passes per band
    min_wipe_len: float = 0.02   # jiggle length for very narrow targets (m)

    def __post_init__(self) -> None:
        for name in ("cleaner_u", "cleaner_v", "approach_dist", "wipe_speed", "travel_speed", "min_wipe_len"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not math.isfinite(self.overlap) or not 0 <= self.overlap <= 0.9:
            raise ValueError("overlap must be between 0 and 0.9")
        if not math.isfinite(self.margin) or self.margin < 0:
            raise ValueError("margin must be finite and non-negative")
        if (not math.isfinite(self.press_depth) or self.press_depth < 0
                or not math.isfinite(self.max_press_depth) or not 0 <= self.max_press_depth <= 0.015):
            raise ValueError("press depth must be non-negative; maximum must be <= 0.015 m")
        if not isinstance(self.passes, int) or self.passes < 1:
            raise ValueError("passes must be a positive integer")


def merge_overlapping(targets: list[EraseTarget], gap: float = 0.0) -> list[EraseTarget]:
    """Merge rectangles that overlap (or are within ``gap``) into one."""
    rects = [t.expanded(gap / 2.0) for t in targets]
    merged = True
    while merged:
        merged = False
        out: list[EraseTarget] = []
        while rects:
            a = rects.pop(0)
            i = 0
            while i < len(rects):
                b = rects[i]
                if a.u0 <= b.u1 and b.u0 <= a.u1 and a.v0 <= b.v1 and b.v0 <= a.v1:
                    a = EraseTarget(min(a.id, b.id), min(a.u0, b.u0), min(a.v0, b.v0),
                                    max(a.u1, b.u1), max(a.v1, b.v1),
                                    a.area_px + b.area_px, a.label or b.label)
                    rects.pop(i)
                    merged = True
                else:
                    i += 1
            out.append(a)
        rects = out
    return [t.expanded(-gap / 2.0) for t in rects]


def _band_centers(v0: float, v1: float, cleaner_v: float, overlap: float) -> list[float]:
    height = v1 - v0
    if height <= cleaner_v:
        return [(v0 + v1) / 2.0]
    pitch = max(cleaner_v * (1.0 - overlap), 1e-3)
    top = v1 - cleaner_v / 2.0
    bottom = v0 + cleaner_v / 2.0
    n = int(math.ceil((top - bottom) / pitch)) + 1
    if n <= 1:
        return [(v0 + v1) / 2.0]
    step = (top - bottom) / (n - 1)
    return [top - i * step for i in range(n)]


def plan_strokes(targets: list[EraseTarget], cfg: PlannerConfig | None = None) -> list[Stroke]:
    """Generate zig-zag wipe strokes covering every target.

    Targets are visited top-to-bottom, then left-to-right. Within a target the
    cleaner stays pressed between bands; it retreats between targets.
    """
    cfg = cfg or PlannerConfig()
    press = min(cfg.press_depth, cfg.max_press_depth)
    ordered = sorted(targets, key=lambda t: (-round(t.v1, 2), t.u0))
    strokes: list[Stroke] = []
    for t in ordered:
        r = t.expanded(cfg.margin)
        bands = _band_centers(r.v0, r.v1, cfg.cleaner_v, cfg.overlap)
        # Cleaner center travel range along u so the footprint covers [u0, u1].
        u_start = r.u0 + cfg.cleaner_u / 2.0
        u_end = r.u1 - cfg.cleaner_u / 2.0
        if u_end - u_start < cfg.min_wipe_len:
            mid = (r.u0 + r.u1) / 2.0
            u_start, u_end = mid - cfg.min_wipe_len / 2.0, mid + cfg.min_wipe_len / 2.0
        stroke = Stroke(target_id=t.id)
        direction = 1
        first = True
        for v in bands:
            a, b = (u_start, u_end) if direction > 0 else (u_end, u_start)
            if first:
                stroke.waypoints.append(Waypoint(a, v, -cfg.approach_dist, "approach", cfg.travel_speed))
                stroke.waypoints.append(Waypoint(a, v, press, "press", cfg.wipe_speed))
                first = False
            else:
                # slide down to the next band while pressed
                stroke.waypoints.append(Waypoint(a, v, press, "wipe", cfg.wipe_speed))
            for p in range(cfg.passes):
                end = b if p % 2 == 0 else a
                stroke.waypoints.append(Waypoint(end, v, press, "wipe", cfg.wipe_speed))
            # next band starts where this one ended
            if cfg.passes % 2 == 1:
                direction = -direction
        last = stroke.waypoints[-1]
        stroke.waypoints.append(Waypoint(last.u, last.v, -cfg.approach_dist, "retreat", cfg.travel_speed))
        strokes.append(stroke)
    return strokes


def stroke_length(strokes: list[Stroke]) -> float:
    total = 0.0
    for s in strokes:
        for a, b in zip(s.waypoints[:-1], s.waypoints[1:]):
            total += math.dist((a.u, a.v, a.depth), (b.u, b.v, b.depth))
    return total


def estimate_duration(strokes: list[Stroke], min_segment_time: float = 0.5) -> float:
    total = 0.0
    for s in strokes:
        for a, b in zip(s.waypoints[:-1], s.waypoints[1:]):
            d = math.dist((a.u, a.v, a.depth), (b.u, b.v, b.depth))
            total += max(min_segment_time, d / max(b.speed, 1e-3))
    return total


if __name__ == "__main__":
    # Smoke test: two targets, print the plan.
    demo = [EraseTarget(0, 0.00, 0.20, 0.18, 0.28), EraseTarget(1, 0.25, 0.05, 0.30, 0.12)]
    plan = plan_strokes(demo)
    for s in plan:
        print(f"target {s.target_id}: {len(s.waypoints)} waypoints")
        for w in s.waypoints:
            print(f"  {w.kind:9s} u={w.u:+.3f} v={w.v:+.3f} depth={w.depth:+.3f} speed={w.speed:.2f}")
    print(f"path length {stroke_length(plan):.2f} m, est. {estimate_duration(plan):.1f} s")
