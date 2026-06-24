"""Local mapping utilities for hemispherical LiDAR (Robosense Airy A00).

Provides a simple 2D occupancy grid local mapper using log-odds updates
with raytracing (Bresenham) from the sensor origin. Intended for
hemispherical lidar where points cover roughly the upper hemisphere.

API:
- `HemisphericalLidarLocalMap` - maintain and update an occupancy grid.

Assumptions:
- Input `points` are an (N,3) numpy array in the robot (base) frame.
- `pose` is optional; if provided it should be (x, y, yaw) to transform
  points from sensor frame into the map/robot frame before updating.

This module has no external dependencies besides numpy.
"""

from typing import Optional, Tuple

import numpy as np


def _bresenham(x0: int, y0: int, x1: int, y1: int):
    """Integer Bresenham line algorithm between two grid cells.
    Returns list of (x,y) tuples from (x0,y0) to (x1,y1) inclusive.
    """
    x0 = int(x0); y0 = int(y0); x1 = int(x1); y1 = int(y1)
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    cells = []
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return cells


class HemisphericalLidarLocalMap:
    """Simple local occupancy grid mapper for hemispherical LiDAR.

    Parameters
    - size_m: physical width/height of the square local map in meters
    - resolution: meters per grid cell
    - min_height, max_height: vertical filtering of points (meters)
    - hit_logodds, miss_logodds: increments applied for hits/misses
    """

    def __init__(
        self,
        size_m: float = 10.0,
        resolution: float = 0.05,
        min_height: float = -1.0,
        max_height: float = 2.0,
        hit_logodds: float = 0.85,
        miss_logodds: float = -0.4,
    ): 
        self.size_m = float(size_m)
        self.resolution = float(resolution)
        self.min_h = float(min_height)
        self.max_h = float(max_height)
        self.hit = float(hit_logodds)
        self.miss = float(miss_logodds)

        self.grid_size = int(np.ceil(self.size_m / self.resolution))
        if self.grid_size % 2 == 0:
            self.grid_size += 1
        self.center = self.grid_size // 2

        # log-odds grid initialized to 0 (unknown)
        self.logodds = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)

        # clamp range for numerical stability
        self._lo_min = -20.0
        self._lo_max = 20.0

    def reset(self):
        """Reset the map to unknown state."""
        self.logodds.fill(0.0)

    def _world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        ix = int(np.floor((x + self.size_m / 2.0) / self.resolution))
        iy = int(np.floor((y + self.size_m / 2.0) / self.resolution))
        if 0 <= ix < self.grid_size and 0 <= iy < self.grid_size:
            return ix, iy
        return None

    def transform_points(self, points: np.ndarray, pose: Optional[Tuple[float, float, float]] = None) -> np.ndarray:
        """Transform points by pose=(x,y,yaw) if provided.
        Points are expected shape (N,3).
        """
        if pose is None:
            return points
        x, y, yaw = pose
        c = np.cos(yaw); s = np.sin(yaw)
        R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        pts = points @ R.T
        pts[:, 0] += x
        pts[:, 1] += y
        return pts

    def update(self, points: np.ndarray, pose: Optional[Tuple[float, float, float]] = None) -> None:
        """Update occupancy grid with a single point cloud batch.

        - `points`: (N,3) numpy array in sensor frame (or map frame if `pose` is None)
        - `pose`: optional (x,y,yaw) transform of the sensor into the map/robot frame
        """
        if points is None or len(points) == 0:
            return
        pts = self.transform_points(points, pose)

        # filter by height
        mask = (pts[:, 2] >= self.min_h) & (pts[:, 2] <= self.max_h)
        pts = pts[mask]
        if pts.shape[0] == 0:
            return

        # sensor origin in grid coordinates (center cell)
        ox, oy = self.center, self.center

        for p in pts:
            x, y = float(p[0]), float(p[1])
            cell = self._world_to_cell(x, y)
            if cell is None:
                continue
            ix, iy = cell

            # raytrace from origin to target cell
            ray = _bresenham(ox, oy, ix, iy)
            if len(ray) == 0:
                continue
            # all cells along ray except last are misses
            for (cx, cy) in ray[:-1]:
                self.logodds[cy, cx] += self.miss
            # last cell is a hit
            lx, ly = ray[-1]
            self.logodds[ly, lx] += self.hit

        # clamp
        np.clip(self.logodds, self._lo_min, self._lo_max, out=self.logodds)

    def get_probability_grid(self) -> np.ndarray:
        """Return occupancy probability grid in [0,1] computed from log-odds."""
        odds = np.exp(self.logodds)
        prob = odds / (1.0 + odds)
        return prob

    def get_occupancy_grid(self, threshold: float = 0.5) -> np.ndarray:
        """Boolean occupancy grid where True = occupied.

        Threshold is applied to probability (default 0.5).
        """
        prob = self.get_probability_grid()
        return prob >= threshold


__all__ = ["HemisphericalLidarLocalMap"]
