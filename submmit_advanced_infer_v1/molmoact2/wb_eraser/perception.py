"""Board and ink detection from an RGB-D frame (no ROS dependency).

Pipeline:
    1. ``detect_board``  - the large bright, low-saturation region is the board.
    2. ``fit_plane``     - plane through the board pixels using the depth image.
    3. ``detect_ink``    - dark strokes inside the board (adaptive threshold).
    4. ``find_components`` - group ink into word/figure-sized components.
    5. ``components_to_targets`` - back-project component boxes onto the plane
       and express them in board coordinates (see :mod:`geometry`).
    6. ``annotate_som``  - "set-of-mark" image for a vision-language model.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .geometry import BoardFrame, apply_transform, rotate_vectors
from .planner import EraseTarget


@dataclass
class Component:
    id: int
    x: int
    y: int
    w: int
    h: int
    area: int
    cx: float
    cy: float


@dataclass
class PlaneCam:
    """Plane in the camera optical frame: n . p = d."""

    point: np.ndarray
    normal: np.ndarray
    inlier_ratio: float


# --------------------------------------------------------------------------- board

def detect_board(bgr: np.ndarray, sat_max: int = 70, val_min: int = 140,
                 min_area_ratio: float = 0.03) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Return (board_mask, quad) or (None, None).

    board_mask is uint8 {0,255}; quad is a (4, 2) float32 array of corners
    (approximate polygon of the largest bright region) or the bounding box.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    bright = ((hsv[..., 1] < sat_max) & (hsv[..., 2] > val_min)).astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    # Close small gaps (ink strokes) so the board becomes one blob.
    bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, kernel, iterations=3)
    bright = cv2.morphologyEx(bright, cv2.MORPH_OPEN, kernel, iterations=1)
    contours, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None
    best = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(best)
    if area < min_area_ratio * bgr.shape[0] * bgr.shape[1]:
        return None, None
    peri = cv2.arcLength(best, True)
    approx = cv2.approxPolyDP(best, 0.02 * peri, True)
    if len(approx) == 4:
        quad = approx.reshape(4, 2).astype(np.float32)
    else:
        x, y, w, h = cv2.boundingRect(best)
        quad = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], np.float32)
    mask = np.zeros(bgr.shape[:2], np.uint8)
    cv2.fillPoly(mask, [quad.astype(np.int32)], 255)
    return mask, quad


# --------------------------------------------------------------------------- plane

def backproject(px: np.ndarray, py: np.ndarray, depth: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Pixels + z-depth -> (N, 3) points in the camera optical frame."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x = (px - cx) / fx * depth
    y = (py - cy) / fy * depth
    return np.stack([x, y, depth], axis=-1)


def fit_plane(depth: np.ndarray, mask: np.ndarray, K: np.ndarray, step: int = 4,
              max_dist: float = 0.02, min_points: int = 200) -> PlaneCam | None:
    """Least-squares plane through masked depth pixels with one outlier-rejection pass."""
    ys, xs = np.nonzero(mask[::step, ::step])
    ys = ys * step
    xs = xs * step
    z = depth[ys, xs].astype(np.float64)
    ok = np.isfinite(z) & (z > 0.05) & (z < 5.0)
    if ok.sum() < min_points:
        return None
    pts = backproject(xs[ok].astype(np.float64), ys[ok].astype(np.float64), z[ok], K)

    def _fit(p):
        c = p.mean(axis=0)
        _, _, vt = np.linalg.svd(p - c, full_matrices=False)
        n = vt[-1]
        return c, n / np.linalg.norm(n)

    c, n = _fit(pts)
    dist = np.abs((pts - c) @ n)
    inl = dist < max_dist
    if inl.sum() >= min_points:
        c, n = _fit(pts[inl])
        dist = np.abs((pts - c) @ n)
        inl = dist < max_dist
    # Camera looks along +z; make the normal face the camera (negative z component).
    if n[2] > 0:
        n = -n
    return PlaneCam(c, n, float(inl.mean()))


def pixel_to_plane(px, py, K: np.ndarray, plane: PlaneCam) -> np.ndarray:
    """Intersect pixel rays with the plane. Returns (N, 3) camera-frame points."""
    px = np.atleast_1d(np.asarray(px, dtype=np.float64))
    py = np.atleast_1d(np.asarray(py, dtype=np.float64))
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    rays = np.stack([(px - cx) / fx, (py - cy) / fy, np.ones_like(px)], axis=-1)
    denom = rays @ plane.normal
    denom = np.where(np.abs(denom) < 1e-9, 1e-9, denom)
    s = (plane.point @ plane.normal) / denom
    return rays * s[:, None]


# --------------------------------------------------------------------------- ink

def board_width_px(board_mask: np.ndarray) -> int:
    xs = np.nonzero(board_mask.any(axis=0))[0]
    return int(xs.max() - xs.min() + 1) if xs.size else 0


def touches_border(quad: np.ndarray, shape: tuple[int, ...], margin: int = 2) -> bool:
    h, w = shape[:2]
    return bool((quad[:, 0] <= margin).any() or (quad[:, 1] <= margin).any()
                or (quad[:, 0] >= w - 1 - margin).any() or (quad[:, 1] >= h - 1 - margin).any())


def detect_ink(bgr: np.ndarray, board_mask: np.ndarray, block: int = 41, c: int = 12,
               erode_frac: float = 0.03, min_dark_contrast: int = 25) -> np.ndarray:
    """Binary mask of ink strokes inside the board.

    ``erode_frac`` shrinks the board mask by that fraction of the board width so
    the board frame / tray is never treated as ink.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    block = block if block % 2 == 1 else block + 1
    ink = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, block, c)
    # Also require real darkness relative to the board median to suppress texture noise.
    board_pixels = gray[board_mask > 0]
    if board_pixels.size:
        med = int(np.median(board_pixels))
        ink[gray > med - min_dark_contrast] = 0
    erode_px = max(3, int(erode_frac * board_width_px(board_mask)))
    inner = cv2.erode(board_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=erode_px)
    ink = cv2.bitwise_and(ink, inner)
    ink = cv2.morphologyEx(ink, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    ink = cv2.morphologyEx(ink, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return ink


def find_components(ink: np.ndarray, min_area: int = 40, group_dilate: int = 12) -> list[Component]:
    """Connected components of the ink mask, grouped by a dilation so that
    letters of one word become one component. Boxes are reported on the
    original (undilated) ink. Pass ``group_dilate`` relative to the board size
    (see :func:`analyse`)."""
    if group_dilate > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * group_dilate + 1, 2 * group_dilate + 1))
        grouped = cv2.dilate(ink, k)
    else:
        grouped = ink
    n, labels, stats, _ = cv2.connectedComponentsWithStats(grouped, connectivity=8)
    comps: list[Component] = []
    for lab in range(1, n):
        region = (labels == lab) & (ink > 0)
        area = int(region.sum())
        if area < min_area:
            continue
        ys, xs = np.nonzero(region)
        x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
        comps.append(Component(len(comps), int(x0), int(y0), int(x1 - x0 + 1), int(y1 - y0 + 1),
                               area, float(xs.mean()), float(ys.mean())))
    # Reading order: top-to-bottom rows, then left-to-right.
    comps.sort(key=lambda cmp: (round(cmp.cy / 40.0), cmp.cx))
    for i, cmp in enumerate(comps):
        cmp.id = i
    return comps


def ink_area_ratio(ink: np.ndarray, board_mask: np.ndarray) -> float:
    b = int((board_mask > 0).sum())
    return float((ink > 0).sum()) / b if b else 0.0


# --------------------------------------------------------------------------- 3D

def components_to_targets(comps: list[Component], K: np.ndarray, plane: PlaneCam,
                          T_base_cam: np.ndarray, board: BoardFrame) -> list[EraseTarget]:
    """Back-project each component's box corners onto the board plane and
    return axis-aligned rectangles in board coordinates."""
    targets: list[EraseTarget] = []
    for cmp in comps:
        xs = np.array([cmp.x, cmp.x + cmp.w, cmp.x + cmp.w, cmp.x], float)
        ys = np.array([cmp.y, cmp.y, cmp.y + cmp.h, cmp.y + cmp.h], float)
        pts_cam = pixel_to_plane(xs, ys, K, plane)
        pts_base = apply_transform(T_base_cam, pts_cam)
        uv = np.array([board.to_board(p)[:2] for p in pts_base])
        targets.append(EraseTarget(cmp.id, float(uv[:, 0].min()), float(uv[:, 1].min()),
                                   float(uv[:, 0].max()), float(uv[:, 1].max()), cmp.area))
    return targets


def board_frame_from_plane(plane: PlaneCam, T_base_cam: np.ndarray, anchor_px: tuple[float, float],
                           K: np.ndarray) -> BoardFrame:
    """BoardFrame in base_link with the origin at the plane point under ``anchor_px``."""
    origin_cam = pixel_to_plane([anchor_px[0]], [anchor_px[1]], K, plane)[0]
    origin = apply_transform(T_base_cam, origin_cam)
    normal = rotate_vectors(T_base_cam, plane.normal)
    return BoardFrame.from_plane(origin, normal)


# --------------------------------------------------------------------------- viz

def annotate_som(bgr: np.ndarray, comps: list[Component], selected: set[int] | None = None) -> np.ndarray:
    """Draw numbered boxes (set-of-mark) for a VLM or for the report."""
    out = bgr.copy()
    for cmp in comps:
        color = (0, 200, 0) if (selected is None or cmp.id in selected) else (0, 0, 220)
        cv2.rectangle(out, (cmp.x, cmp.y), (cmp.x + cmp.w, cmp.y + cmp.h), color, 2)
        label = str(cmp.id)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        tx, ty = cmp.x, max(cmp.y - 4, th + 2)
        cv2.rectangle(out, (tx, ty - th - 2), (tx + tw + 4, ty + 2), color, -1)
        cv2.putText(out, label, (tx + 2, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def overlay_mask(bgr: np.ndarray, mask: np.ndarray, color=(0, 0, 255), alpha: float = 0.5) -> np.ndarray:
    out = bgr.copy()
    sel = mask > 0
    out[sel] = (out[sel] * (1 - alpha) + np.array(color) * alpha).astype(np.uint8)
    return out


@dataclass
class Perception:
    """One analysed frame."""

    board_mask: np.ndarray
    quad: np.ndarray
    ink: np.ndarray
    comps: list[Component]
    plane: PlaneCam | None
    ink_ratio: float
    board_px: int = 0
    clipped: bool = False


def analyse(bgr: np.ndarray, depth: np.ndarray | None, K: np.ndarray | None,
            min_area_frac: float = 2e-4, group_frac: float = 0.012) -> Perception | None:
    """Full 2D analysis. Grouping and minimum area scale with the board size in
    pixels so the same settings work at VGA and HD720 and at different ranges."""
    board_mask, quad = detect_board(bgr)
    if board_mask is None:
        return None
    bw = board_width_px(board_mask)
    ink = detect_ink(bgr, board_mask)
    comps = find_components(ink, min_area=max(10, int(min_area_frac * bw * bw)),
                            group_dilate=max(2, int(group_frac * bw)))
    plane = fit_plane(depth, board_mask, K) if depth is not None and K is not None else None
    return Perception(board_mask, quad, ink, comps, plane, ink_area_ratio(ink, board_mask),
                      bw, touches_border(quad, bgr.shape))


def plane_from_base(T_base_cam: np.ndarray, point_base, normal_base) -> PlaneCam:
    """PlaneCam for a plane known in base_link (e.g. a fixed board at x = X)."""
    T_cam_base = np.linalg.inv(T_base_cam)
    p = apply_transform(T_cam_base, np.asarray(point_base, float))
    n = rotate_vectors(T_cam_base, np.asarray(normal_base, float))
    n = n / np.linalg.norm(n)
    if n[2] > 0:
        n = -n
    return PlaneCam(p, n, 1.0)
