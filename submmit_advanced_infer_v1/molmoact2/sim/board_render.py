"""Synthetic whiteboard renderer (pure numpy / OpenCV, no ROS).

The board is a vertical plane ``x = board_x`` in base_link, facing the robot.
A texture (1 px = 1 mm) holds the ink. A pinhole camera with pose
``T_base_cam`` (optical frame: z forward, x right, y down) renders the colour
image and a z-depth image. ``erase_at`` clears a rectangle of the texture when
the cleaner (felt) face is pressed against the board.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from wb_eraser.geometry import transform_matrix


@dataclass
class BoardSpec:
    x: float = 0.45          # plane position in base_link (m)
    cy: float = -0.20        # board centre y (m)
    cz: float = 1.15         # board centre z (m)
    w: float = 0.50          # width (m)
    h: float = 0.35          # height (m)
    frame_mm: int = 15       # dark frame width (mm)
    wall_gray: int = 120


class BoardRenderer:
    def __init__(self, spec: BoardSpec | None = None, layout: str = "demo"):
        self.spec = spec or BoardSpec()
        self.W = int(round(self.spec.w * 1000))
        self.H = int(round(self.spec.h * 1000))
        self.texture = np.full((self.H, self.W, 3), 245, np.uint8)
        self.ink_px0 = 0
        self.draw_layout(layout)

    # ---------------------------------------------------------------- texture
    def draw_layout(self, layout: str) -> None:
        tex = self.texture
        tex[:] = 245
        f = self.spec.frame_mm
        black = (20, 20, 30)
        if layout == "demo":
            cv2.putText(tex, "ROBOT 2026", (40, 110), cv2.FONT_HERSHEY_SIMPLEX, 2.2, black, 8, cv2.LINE_AA)
            cv2.putText(tex, "AI x PHYSICAL", (40, 210), cv2.FONT_HERSHEY_SIMPLEX, 1.8, (30, 30, 160), 7, cv2.LINE_AA)
            cv2.circle(tex, (400, 280), 45, (160, 40, 40), 7, cv2.LINE_AA)
            cv2.polylines(tex, [np.array([[60, 320], [140, 250], [220, 320]], np.int32)], True, black, 7, cv2.LINE_AA)
        elif layout == "numbers":
            cv2.putText(tex, "TODO", (40, 110), cv2.FONT_HERSHEY_SIMPLEX, 2.4, black, 9, cv2.LINE_AA)
            cv2.putText(tex, "12 34 56", (40, 230), cv2.FONT_HERSHEY_SIMPLEX, 2.4, (30, 30, 160), 9, cv2.LINE_AA)
            cv2.putText(tex, "done", (320, 320), cv2.FONT_HERSHEY_SIMPLEX, 1.6, black, 6, cv2.LINE_AA)
        elif layout == "blank":
            pass
        else:
            raise ValueError(f"unknown layout {layout}")
        # frame
        cv2.rectangle(tex, (0, 0), (self.W - 1, self.H - 1), (40, 40, 40), f)
        self.ink_px0 = self.ink_pixels()

    def ink_pixels(self) -> int:
        f = self.spec.frame_mm + 2
        inner = self.texture[f:-f, f:-f]
        return int((inner.min(axis=2) < 150).sum())

    def ink_remaining_ratio(self) -> float:
        return self.ink_pixels() / max(self.ink_px0, 1)

    # ---------------------------------------------------------------- board <-> texture
    def base_to_tex(self, y: float, z: float) -> tuple[float, float]:
        """base_link (y, z) on the plane -> texture (col, row) in px."""
        col = (self.spec.cy + self.spec.w / 2.0 - y) * 1000.0
        row = (self.spec.cz + self.spec.h / 2.0 - z) * 1000.0
        return col, row

    def on_board(self, y: float, z: float) -> bool:
        return (abs(y - self.spec.cy) <= self.spec.w / 2.0 and abs(z - self.spec.cz) <= self.spec.h / 2.0)

    def erase_at(self, y: float, z: float, cleaner_u: float, cleaner_v: float) -> int:
        """Clear the cleaner footprint centred at base (y, z). Returns cleared px."""
        col, row = self.base_to_tex(y, z)
        hw, hh = cleaner_u * 500.0, cleaner_v * 500.0
        f = self.spec.frame_mm
        c0, c1 = int(max(f, col - hw)), int(min(self.W - f, col + hw))
        r0, r1 = int(max(f, row - hh)), int(min(self.H - f, row + hh))
        if c1 <= c0 or r1 <= r0:
            return 0
        region = self.texture[r0:r1, c0:c1]
        n = int((region.min(axis=2) < 150).sum())
        region[:] = 245
        return n

    # ---------------------------------------------------------------- camera
    @staticmethod
    def pixel_rays(K: np.ndarray, width: int, height: int) -> np.ndarray:
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        xs = (np.arange(width) - cx) / fx
        ys = (np.arange(height) - cy) / fy
        gx, gy = np.meshgrid(xs, ys)
        return np.stack([gx, gy, np.ones_like(gx)], axis=-1)

    def render(self, T_base_cam: np.ndarray, K: np.ndarray, width: int, height: int,
               depth_noise: float = 0.0, rays: np.ndarray | None = None,
               rng: np.random.Generator | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Return (bgr image, z-depth in meters) for the camera pose."""
        rays = rays if rays is not None else self.pixel_rays(K, width, height)
        R, t = T_base_cam[:3, :3], T_base_cam[:3, 3]
        d = rays @ R.T                      # ray directions in base_link
        dx = d[..., 0]
        valid = dx > 1e-6
        s = np.where(valid, (self.spec.x - t[0]) / np.where(valid, dx, 1.0), np.nan)
        py = t[1] + s * d[..., 1]
        pz = t[2] + s * d[..., 2]
        img = np.full((height, width, 3), self.spec.wall_gray, np.uint8)
        col = (self.spec.cy + self.spec.w / 2.0 - py) * 1000.0
        row = (self.spec.cz + self.spec.h / 2.0 - pz) * 1000.0
        on = valid & (col >= 0) & (col < self.W) & (row >= 0) & (row < self.H)
        ci = np.clip(col[on].astype(np.int32), 0, self.W - 1)
        ri = np.clip(row[on].astype(np.int32), 0, self.H - 1)
        img[on] = self.texture[ri, ci]
        depth = np.where(valid & (s > 0), s, np.nan).astype(np.float32)  # z-depth: rays have z=1
        if depth_noise > 0:
            noise = (rng.normal if rng is not None else np.random.normal)(0.0, depth_noise, depth.shape)
            depth = depth + noise.astype(np.float32)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        return img, depth


def default_intrinsics(width: int = 672, height: int = 376, f: float = 340.0) -> np.ndarray:
    return np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])


def head_camera_pose(pitch: float = 0.60, yaw: float = -0.30) -> np.ndarray:
    """Approximate SG2 base <- optical pose; this is not a URDF/IK model."""
    camera = transform_matrix((0.093, 0.025, 1.577), (-0.5, 0.5, -0.5, 0.5))
    c, s = np.cos(pitch), np.sin(pitch)
    rp = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    c, s = np.cos(yaw), np.sin(yaw)
    ry = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    camera[:3, :3] = ry @ rp @ camera[:3, :3]
    return camera
