"""Board plane frame and coordinate conversions.

All vectors are expressed in the robot ``base_link`` frame unless noted.
Board coordinates are ``(u, v, depth)``:

* ``u``     horizontal position on the board (robot's right is positive)
* ``v``     vertical position on the board (up is positive)
* ``depth`` penetration into the board (positive = into the board,
            negative = in front of the board on the robot side)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError("zero-length vector")
    return v / n


@dataclass
class BoardFrame:
    """A plane in base_link with an orthonormal in-plane basis."""

    origin: np.ndarray  # a point on the board surface
    normal: np.ndarray  # unit normal pointing out of the board toward the robot
    u: np.ndarray       # unit vector, board horizontal (robot's right)
    v: np.ndarray       # unit vector, board vertical (up)

    @classmethod
    def from_plane(cls, point, normal, up=(0.0, 0.0, 1.0)) -> "BoardFrame":
        point = np.asarray(point, dtype=float)
        n = _unit(normal)
        # The robot stands at the base_link origin. Make the normal point toward it.
        if np.dot(n, point) > 0.0:
            n = -n
        up = _unit(up)
        if abs(np.dot(up, n)) > 0.99:
            raise ValueError("board normal is parallel to the up vector")
        u = _unit(np.cross(up, n))
        v = _unit(np.cross(n, u))
        return cls(point, n, u, v)

    @classmethod
    def vertical_facing_robot(cls, x: float, y: float = 0.0, z: float = 1.0) -> "BoardFrame":
        """A vertical board in front of the robot at distance ``x``."""
        return cls.from_plane((x, y, z), (-1.0, 0.0, 0.0))

    def to_base(self, u: float, v: float, depth: float = 0.0) -> np.ndarray:
        return self.origin + u * self.u + v * self.v - depth * self.normal

    def to_board(self, p) -> tuple[float, float, float]:
        d = np.asarray(p, dtype=float) - self.origin
        return float(d @ self.u), float(d @ self.v), float(-(d @ self.normal))

    def as_dict(self) -> dict:
        return {
            "origin": self.origin.tolist(),
            "normal": self.normal.tolist(),
            "u": self.u.tolist(),
            "v": self.v.tolist(),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "BoardFrame":
        return cls(
            np.asarray(d["origin"], float),
            np.asarray(d["normal"], float),
            np.asarray(d["u"], float),
            np.asarray(d["v"], float),
        )


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix from a unit quaternion (x, y, z, w)."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def transform_matrix(translation, quaternion_xyzw) -> np.ndarray:
    """4x4 homogeneous transform from translation and quaternion."""
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(*quaternion_xyzw)
    T[:3, 3] = np.asarray(translation, dtype=float)
    return T


def apply_transform(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to an (N, 3) or (3,) array of points."""
    pts = np.asarray(points, dtype=float)
    single = pts.ndim == 1
    pts = np.atleast_2d(pts)
    out = pts @ T[:3, :3].T + T[:3, 3]
    return out[0] if single else out


def rotate_vectors(T: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    vec = np.asarray(vectors, dtype=float)
    single = vec.ndim == 1
    vec = np.atleast_2d(vec)
    out = vec @ T[:3, :3].T
    return out[0] if single else out
