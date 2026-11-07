"""Shared path math: spline smoothing, curvature and speed profile, CSV output."""
import csv

import numpy as np
from scipy.interpolate import splev, splprep

SPACING = 0.10      # m between output waypoints
HANDLE_EVERY = 10   # saved waypoints per editable point when a path is reopened (about 1 m)
HANDLE_TOLERANCE = 0.02  # m: a reopened path may differ from the saved one by this much


def smooth_path(pts, closed):
    """Fit a spline through the clicked points and resample it every SPACING metres."""
    pts = np.asarray(pts, dtype=float)
    if closed:
        pts = np.vstack([pts, pts[:1]])
    k = min(3, len(pts) - 1)
    tck, _ = splprep(pts.T, s=0.0, k=k, per=closed)
    dense = np.array(splev(np.linspace(0, 1, 2000), tck)).T
    seg = np.linalg.norm(np.diff(dense, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(2, int(s[-1] / SPACING))
    s_new = np.linspace(0, s[-1], n, endpoint=not closed)
    xy = np.column_stack([np.interp(s_new, s, dense[:, 0]), np.interp(s_new, s, dense[:, 1])])
    return xy, s[-1]


def handles_from_path(xy):
    """Editable points for a saved path, which stores only the smoothed result: (points, closed)."""
    if len(xy) < 2:
        return xy, True
    # a closed path has no repeated end point, so its ends are one spacing apart
    closed = bool(np.linalg.norm(xy[0] - xy[-1]) < 3 * SPACING)
    n = max(3 if closed else 2, round(len(xy) / HANDLE_EVERY))
    if closed:
        idx = np.linspace(0, len(xy), n, endpoint=False).astype(int)
    else:
        idx = np.linspace(0, len(xy) - 1, n).round().astype(int)
    idx = set(idx.tolist())
    # tight corners need more points, or saving would move parts of the path nobody touched
    for _ in range(len(xy)):
        handles = xy[sorted(idx)]
        curve, _length = smooth_path(handles, closed)
        if closed:
            curve = np.vstack([curve, curve[:1]])
        gaps = [distance_to_polyline(p, curve) for p in xy]
        if max(gaps) < HANDLE_TOLERANCE:
            break
        idx.add(int(np.argmax(gaps)))
    return xy[sorted(idx)], closed


def distance_to_polyline(p, line):
    """Distance from point p to the segments of an Nx2 polyline."""
    a, d = line[:-1], np.diff(line, axis=0)
    t = np.clip(((p - a) * d).sum(1) / np.maximum((d * d).sum(1), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(a + t[:, None] * d - p, axis=1).min())


def curvature(xy, closed):
    if closed:
        prev, nxt = np.roll(xy, 1, axis=0), np.roll(xy, -1, axis=0)
    else:
        prev = np.vstack([xy[:1], xy[:-1]])
        nxt = np.vstack([xy[1:], xy[-1:]])
    a = np.linalg.norm(xy - prev, axis=1)
    b = np.linalg.norm(nxt - xy, axis=1)
    c = np.linalg.norm(nxt - prev, axis=1)
    cross = np.abs((xy[:, 0] - prev[:, 0]) * (nxt[:, 1] - prev[:, 1])
                   - (xy[:, 1] - prev[:, 1]) * (nxt[:, 0] - prev[:, 0]))
    with np.errstate(divide='ignore', invalid='ignore'):
        k = np.where(a * b * c > 1e-9, 2.0 * cross / (a * b * c), 0.0)
    return k


def speed_profile(xy, closed, vmax, vmin, a_lat, a_lon):
    """Curvature-limited speed, then forward/backward passes for accel/brake limits."""
    k = curvature(xy, closed)
    with np.errstate(divide='ignore'):
        v = np.minimum(vmax, np.sqrt(a_lat / np.maximum(k, 1e-6)))
    if not closed:
        # start from rest and brake down to the slowest speed before the end
        v[0] = v[-1] = vmin
    ds = np.linalg.norm(np.diff(xy, axis=0, append=xy[:1] if closed else xy[-1:]), axis=1)
    passes = 2 if closed else 1
    for _ in range(passes):
        for i in range(1, len(v) if not closed else len(v) + 1):
            j, p = i % len(v), (i - 1) % len(v)
            v[j] = min(v[j], np.sqrt(v[p] ** 2 + 2 * a_lon * ds[p]))
        for i in range(len(v) - 2, -1 if not closed else -2, -1):
            j, n = i % len(v), (i + 1) % len(v)
            v[j] = min(v[j], np.sqrt(v[n] ** 2 + 2 * a_lon * ds[j]))
    return np.maximum(v, vmin)


def read_csv(path):
    """The x, y columns of a file from write_csv, as an Nx2 array (a file may have no rows)."""
    with open(path, newline='') as f:
        rows = [(float(r['x']), float(r['y'])) for r in csv.DictReader(f)]
    return np.array(rows).reshape(-1, 2)


def write_csv(path, xy, v):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'speed'])
        for (x, y), s in zip(xy, v):
            w.writerow([f'{x:.4f}', f'{y:.4f}', f'{s:.3f}'])
