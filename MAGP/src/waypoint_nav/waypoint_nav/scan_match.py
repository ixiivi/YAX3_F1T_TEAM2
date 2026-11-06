"""Scan-to-map matching on a saved occupancy map (likelihood field).

Used to check how well the current pose explains the LiDAR scan, and to find the pose
again (around a guess, or anywhere on the map) when localization is lost.
"""
import math
import os

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt


def scan_rays(scan, max_range=8.0, step=1):
    """Valid ranges and beam angles of a LaserScan, every `step`-th beam."""
    r = np.asarray(scan.ranges)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    ok = np.isfinite(r) & (r > max(scan.range_min, 0.05)) & (r < max_range)
    return r[ok][::step], a[ok][::step]


class MapMatcher:
    def __init__(self, map_yaml, laser_x=0.267):
        with open(map_yaml) as f:
            meta = yaml.safe_load(f)
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), img_path)
        self.img = np.array(Image.open(img_path).convert('L'))
        self.h, self.w = self.img.shape
        self.res = float(meta['resolution'])
        self.ox, self.oy = meta['origin'][:2]
        self.laser_x = laser_x
        # distance from every cell to the nearest wall
        self.dist = distance_transform_edt(~(self.img < 50)) * self.res

    def _cells(self, px, py):
        u = ((np.asarray(px) - self.ox) / self.res).astype(int)
        v = self.h - 1 - ((np.asarray(py) - self.oy) / self.res).astype(int)
        return u, v, (u >= 0) & (u < self.w) & (v >= 0) & (v < self.h)

    def inside(self, px, py):
        """True for world points inside the map image."""
        return self._cells(px, py)[2]

    def wall_dist(self, px, py):
        """Distance to the nearest map wall for world points (1.0 outside the map)."""
        u, v, ins = self._cells(px, py)
        d = np.full(np.shape(px), 1.0)
        d[ins] = self.dist[v[ins], u[ins]]
        return d

    def project(self, x, y, yaw, r, a):
        """Scan endpoints in the map frame for base_link pose(s) x, y (scalars or arrays)."""
        cx = np.asarray(x, dtype=float)[..., None] + self.laser_x * math.cos(yaw)
        cy = np.asarray(y, dtype=float)[..., None] + self.laser_x * math.sin(yaw)
        return cx + r * np.cos(yaw + a), cy + r * np.sin(yaw + a)

    def fit(self, x, y, yaw, r, a, tol=0.10):
        """Fraction of scan points within tol of a map wall at this pose (0..1)."""
        if len(r) == 0:
            return 0.0
        px, py = self.project(x, y, yaw, r, a)
        return float((self.wall_dist(px, py) < tol).mean())

    def _score(self, xs, ys, yaw, r, a):
        px, py = self.project(xs, ys, yaw, r, a)
        d = np.minimum(self.wall_dist(px, py), 1.0)
        return np.exp(-(d / 0.1) ** 2).mean(-1)

    def _search(self, xs, ys, yaws, r, a):
        best = (-1.0, 0.0, 0.0, 0.0)
        for yaw in yaws:
            sc = self._score(xs, ys, yaw, r, a)
            i = int(sc.argmax())
            if sc[i] > best[0]:
                best = (float(sc[i]), float(xs[i]), float(ys[i]), float(yaw))
        sc0, x0, y0, yaw0 = best
        for step, dang in ((0.05, 1.0), (0.02, 0.3)):    # refine around the best cell
            for _ in range(3):
                g = np.arange(-3, 4) * step
                gx, gy = np.meshgrid(g, g)
                gx, gy = gx.ravel() + x0, gy.ravel() + y0
                for yaw in yaw0 + np.radians(np.arange(-3, 4) * dang):
                    sc = self._score(gx, gy, yaw, r, a)
                    i = int(sc.argmax())
                    if sc[i] > sc0:
                        sc0, x0, y0, yaw0 = float(sc[i]), float(gx[i]), float(gy[i]), float(yaw)
        return sc0, x0, y0, math.atan2(math.sin(yaw0), math.cos(yaw0))

    def local_search(self, x, y, yaw, r, a, radius=1.0, yaw_range_deg=30.0):
        """Best pose near a guess. Returns (score, x, y, yaw); score is 0..1."""
        g = np.arange(-radius, radius + 1e-9, 0.1)
        gx, gy = np.meshgrid(g, g)
        yaws = yaw + np.radians(np.arange(-yaw_range_deg, yaw_range_deg + 1e-9, 2.0))
        return self._search(gx.ravel() + x, gy.ravel() + y, yaws, r, a)

    def global_search(self, r, a):
        """Best pose anywhere in the known free space (a few seconds on the Jetson)."""
        free = self.img > 250
        free &= distance_transform_edt(free) * self.res > 0.15
        vv, uu = np.nonzero(free[::2, ::2])
        xs = self.ox + (uu * 2 + 0.5) * self.res
        ys = self.oy + (self.h - vv * 2 - 0.5) * self.res
        return self._search(xs, ys, np.radians(np.arange(0, 360, 3)), r, a)
