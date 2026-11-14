"""Scan-to-map matching on a saved occupancy map (likelihood field).

Used to check how well the current pose explains the LiDAR scan, and to find the pose
again (around a guess, or anywhere on the map) when localization is lost.

device='cuda' scores the candidate poses on the Jetson GPU with PyTorch (the searches are
otherwise the same loops). Without torch or CUDA it falls back to the CPU and says so in
self.device_note. The CPU path is unchanged and stays the default.
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
    def __init__(self, map_yaml, laser_x=0.267, device='cpu'):
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
        self.top_k = 5                  # coarse places refined in every search (see _search)
        self.torch = None
        self.device_note = 'cpu'
        if device != 'cpu':
            self._init_gpu(device)

    def _init_gpu(self, device):
        """Put the distance field on the GPU and warm CUDA up, so the first search is not slow."""
        try:
            import torch
            if not torch.cuda.is_available():
                raise RuntimeError('CUDA not available')
            self.torch = torch
            self.tdev = torch.device(device)
            # with the 1.0 cap of _score folded in, and 1.0 outside the map like wall_dist
            self.tdist = torch.as_tensor(self.dist, dtype=torch.float32, device=self.tdev)
            self._score_gpu(np.zeros(4), np.zeros(4), 0.0, np.ones(8), np.zeros(8))
            torch.cuda.synchronize()
            self.device_note = f'gpu ({torch.cuda.get_device_name(0)})'
        except Exception as e:  # noqa: BLE001 - any failure: CPU as before
            self.torch = None
            self.device_note = f'cpu (gpu unavailable: {e})'

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

    def _lookup_gpu(self, u, v):
        """wall_dist for cell indices on the GPU: 1.0 outside the map. A gather with clamped
        indices and a where, no boolean indexing (that copied the whole batch and ran out of
        the Jetson's shared memory)."""
        t = self.torch
        ins = (u >= 0) & (u < self.w) & (v >= 0) & (v < self.h)
        flat = (v.clamp(0, self.h - 1) * self.w + u.clamp(0, self.w - 1))
        d = self.tdist.reshape(-1).take(flat)
        return t.where(ins, d, t.ones((), dtype=d.dtype, device=d.device))

    def _score_gpu(self, xs, ys, yaw, r, a):
        """_score on the GPU: same cells (truncating like _cells), same field, same formula."""
        t = self.torch
        dev = self.tdev
        xs_t = t.as_tensor(np.asarray(xs, dtype=np.float64), device=dev)
        ys_t = t.as_tensor(np.asarray(ys, dtype=np.float64), device=dev)
        r_t = t.as_tensor(np.asarray(r, dtype=np.float64), device=dev)
        a_t = t.as_tensor(np.asarray(a, dtype=np.float64), device=dev)
        cx = xs_t[:, None] + self.laser_x * math.cos(yaw)
        cy = ys_t[:, None] + self.laser_x * math.sin(yaw)
        px = cx + r_t * t.cos(yaw + a_t)
        py = cy + r_t * t.sin(yaw + a_t)
        u = ((px - self.ox) / self.res).to(t.int64)
        v = self.h - 1 - ((py - self.oy) / self.res).to(t.int64)
        d = t.clamp(self._lookup_gpu(u, v), max=1.0)
        return t.exp(-(d / 0.1) ** 2).mean(-1).double().cpu().numpy()

    def _score(self, xs, ys, yaw, r, a):
        if self.torch is not None and len(r) > 0 and np.size(xs) * len(r) > 200_000:
            return self._score_gpu(xs, ys, yaw, r, a)
        px, py = self.project(xs, ys, yaw, r, a)
        d = np.minimum(self.wall_dist(px, py), 1.0)
        return np.exp(-(d / 0.1) ** 2).mean(-1)

    def _coarse_gpu(self, xs, ys, yaws, r, a, heading=None, tol=None, chunk_elems=2_000_000):
        """The coarse loop of _search, all yaws at once on the GPU (in chunks): the best cell of
        every yaw, in yaw order, as the CPU loop collects them."""
        t = self.torch
        dev = self.tdev
        X = t.as_tensor(np.asarray(xs, dtype=np.float64), device=dev)
        Y = t.as_tensor(np.asarray(ys, dtype=np.float64), device=dev)
        R = t.as_tensor(np.asarray(r, dtype=np.float64), device=dev)
        A = t.as_tensor(np.asarray(a, dtype=np.float64), device=dev)
        H = None if heading is None else t.as_tensor(np.asarray(heading, dtype=np.float64), device=dev)
        yaws = np.asarray(yaws, dtype=np.float64)
        per = max(1, int(chunk_elems // max(1, len(xs) * len(r))))
        per_yaw = []
        for c0 in range(0, len(yaws), per):
            yw = t.as_tensor(yaws[c0:c0 + per], device=dev)               # (k,)
            cx = X[None, :, None] + self.laser_x * t.cos(yw)[:, None, None]
            cy = Y[None, :, None] + self.laser_x * t.sin(yw)[:, None, None]
            ang = yw[:, None, None] + A[None, None, :]
            px = cx + R[None, None, :] * t.cos(ang)
            py = cy + R[None, None, :] * t.sin(ang)
            u = ((px - self.ox) / self.res).to(t.int64)
            v = self.h - 1 - ((py - self.oy) / self.res).to(t.int64)
            del px, py, ang
            d = self._lookup_gpu(u, v)
            del u, v
            sc = t.exp(-(t.clamp(d, max=1.0) / 0.1) ** 2).mean(-1)          # (k, N)
            del d
            if H is not None:
                diff = t.remainder(H[None, :] - yw[:, None] + math.pi, 2 * math.pi) - math.pi
                sc = t.where(diff.abs() < tol, sc, t.full_like(sc, -1.0))
            vals, idx = sc.max(dim=1)                                      # best cell per yaw
            for k, (val, i) in enumerate(zip(vals.tolist(), idx.tolist())):
                if val >= 0.0:
                    per_yaw.append((float(val), float(xs[i]), float(ys[i]), float(yaws[c0 + k])))
        return per_yaw

    def _search(self, xs, ys, yaws, r, a, heading=None, tol=None):
        """heading/tol: per-cell heading [rad]; only yaws within tol of it are tried there."""
        per_yaw = []                    # best cell of every yaw tried: (score, x, y, yaw)
        # many candidates: the GPU scores all of them at once; few (refinement, small local
        # searches): the CPU loop, which is faster than the transfers then
        if self.torch is not None and len(r) > 0 and len(xs) * len(yaws) > 20000:
            per_yaw = self._coarse_gpu(xs, ys, yaws, r, a, heading, tol)
            yaws = []
        for yaw in yaws:
            cx, cy = xs, ys
            if heading is not None:
                k = np.abs(np.angle(np.exp(1j * (heading - yaw)))) < tol
                if not k.any():
                    continue
                cx, cy = xs[k], ys[k]
            sc = self._score(cx, cy, yaw, r, a)
            i = int(sc.argmax())
            per_yaw.append((float(sc[i]), float(cx[i]), float(cy[i]), float(yaw)))
        if not per_yaw:
            return self._refine(-1.0, 0.0, 0.0, 0.0, r, a)
        # refine the top_k best distinct places, not only the best coarse cell: on left1043 the
        # coarse grid just missed the true pose (fit 1.00 there) and a look-alike place 2.5-8 m
        # away won at 5 of 30 spots (2026-11-15). The first is the old answer; a later one wins
        # only with a strictly higher refined score, so the result never scores lower than before
        order = sorted(range(len(per_yaw)), key=lambda j: (-per_yaw[j][0], j))
        picks = []
        for j in order:
            c = per_yaw[j]
            if all(math.hypot(c[1] - p[1], c[2] - p[2]) > 0.5
                   or abs(math.remainder(c[3] - p[3], 2 * math.pi)) > math.radians(15) for p in picks):
                picks.append(c)
            if len(picks) == self.top_k:
                break
        best = None
        for c in picks:
            res = self._refine(*c, r, a)
            if best is None or res[0] > best[0]:
                best = res
        return best

    def _refine(self, sc0, x0, y0, yaw0, r, a):
        """Polish one coarse pose on finer grids (5 cm / 1 deg, then 2 cm / 0.3 deg)."""
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
        # a disc, not the square around it: with several places refined (_search) a corner
        # 1.4 radius out could win and break a reach limit (2026-11-15)
        k = np.hypot(gx, gy) <= radius + 1e-9
        gx, gy = gx[k], gy[k]
        yaws = yaw + np.radians(np.arange(-yaw_range_deg, yaw_range_deg + 1e-9, 2.0))
        return self._search(gx + x, gy + y, yaws, r, a)

    def free_cells(self):
        """Search grid of global_search: free cells 0.15 m off the walls, every 2nd cell."""
        free = self.img > 250
        free &= distance_transform_edt(free) * self.res > 0.15
        vv, uu = np.nonzero(free[::2, ::2])
        return self.ox + (uu * 2 + 0.5) * self.res, self.oy + (self.h - vv * 2 - 0.5) * self.res

    def global_search(self, r, a, heading_of=None, tol_deg=45.0):
        """Best pose anywhere in the known free space (14 s on the Jetson on a 19 x 17 m map).
        heading_of(xs, ys) -> heading [rad] per cell: only headings within tol_deg of it are
        tried, e.g. the route direction (about 4x faster with 45 deg)."""
        xs, ys = self.free_cells()
        heading = None if heading_of is None else heading_of(xs, ys)
        return self._search(xs, ys, np.radians(np.arange(0, 360, 3)), r, a,
                            heading=heading, tol=math.radians(tol_deg))
