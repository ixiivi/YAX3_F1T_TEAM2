"""Shared path math: spline smoothing, curvature and speed profile, CSV output."""
import csv

import numpy as np
from scipy.interpolate import CubicSpline, splev, splprep
from scipy.optimize import lsq_linear

SPACING = 0.10      # m between output waypoints
HANDLE_EVERY = 10   # saved waypoints per editable point when a path is reopened (about 1 m)
HANDLE_TOLERANCE = 0.02  # m: a reopened path may differ from the saved one by this much
BOOST, BRAKE = 'boost', 'brake'   # full-throttle / full-brake marks and the zones they make


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


def resample(dense, closed):
    """Points every SPACING along a dense polyline (for a closed one, without the repeated end)."""
    seg = np.linalg.norm(np.diff(dense, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(2, int(s[-1] / SPACING))
    s_new = np.linspace(0, s[-1], n, endpoint=not closed)
    return np.column_stack([np.interp(s_new, s, dense[:, 0]), np.interp(s_new, s, dense[:, 1])])


def _hermite_piece(pts, d0, d1):
    """Spline through pts (by arc length) leaving along unit direction d0 and arriving along d1
    (None: free end). Dense points from the first to the last."""
    pts = np.asarray(pts, dtype=float)
    t = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    if t[-1] < 1e-9 or (np.diff(t) < 1e-9).any():
        raise ValueError('two points on the same spot')
    if len(pts) == 2 and d0 is None and d1 is None:
        return np.linspace(pts[0], pts[1], 50)
    out = []
    for dim in range(2):
        bc0 = (1, d0[dim]) if d0 is not None else (2, 0.0)
        bc1 = (1, d1[dim]) if d1 is not None else (2, 0.0)
        out.append(CubicSpline(t, pts[:, dim], bc_type=(bc0, bc1))(
            np.linspace(0, t[-1], max(50, int(t[-1] / SPACING) * 5))))
    return np.column_stack(out)


def build_path(handles, closed, straight=()):
    """smooth_path where gap i (handle i to handle i+1) in `straight` is a straight line, and the
    curve on either side leaves and joins that line along it (no kink at its ends).

    Returns (xy, length, gap): gap[j] is the handle gap output point j lies on."""
    handles = np.asarray(handles, dtype=float)
    n = len(handles)
    gaps = n if closed else n - 1
    straight = sorted(g for g in set(straight) if 0 <= g < gaps)
    if not straight:
        xy, length = smooth_path(handles, closed)
        return xy, length, _gap_of(xy, handles, closed)
    # walk the gaps from just after a straight one: runs of curve gaps between the lines
    start = straight[0] if closed else 0
    order = [(start + k) % n for k in range(gaps)] if closed else list(range(gaps))
    pieces, run = [], []
    for g in order:
        if g in straight:
            if run:
                pieces.append(('curve', run))
                run = []
            pieces.append(('line', [g]))
        else:
            run.append(g)
    if run:
        pieces.append(('curve', run))

    def direction(g):
        a, b = handles[g], handles[(g + 1) % n]
        d = b - a
        return d / max(np.linalg.norm(d), 1e-9)
    dense, dense_gap = [], []
    for k, (kind, gs) in enumerate(pieces):
        if kind == 'line':
            g = gs[0]
            seg = np.linspace(handles[g], handles[(g + 1) % n], 50)
            ids = [g] * len(seg)
        else:
            idx = [gs[0]] + [(g + 1) % n for g in gs]
            prev = pieces[k - 1] if k > 0 else (pieces[-1] if closed else None)
            nxt = pieces[k + 1] if k + 1 < len(pieces) else (pieces[0] if closed else None)
            d0 = direction(prev[1][0]) if prev and prev[0] == 'line' else None
            d1 = direction(nxt[1][0]) if nxt and nxt[0] == 'line' else None
            seg = _hermite_piece(handles[idx], d0, d1)
            # which gap each dense point is on: by arc length between the handles
            knots = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(handles[idx], axis=0), axis=1))])
            s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))])
            s *= knots[-1] / max(s[-1], 1e-9)
            ids = [gs[min(int(np.searchsorted(knots, x, side='right')) - 1, len(gs) - 1)] for x in s]
        if dense:   # pieces meet at a handle: do not repeat it
            seg, ids = seg[1:], ids[1:]
        dense.append(seg)
        dense_gap.extend(ids)
    dense = np.vstack(dense)
    dense_gap = np.array(dense_gap)
    if closed and np.linalg.norm(dense[0] - dense[-1]) > 1e-6:
        dense = np.vstack([dense, dense[:1]])
        dense_gap = np.append(dense_gap, dense_gap[0])
    seg = np.linalg.norm(np.diff(dense, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    xy = resample(dense, closed)
    s_out = np.linspace(0, s[-1], len(xy), endpoint=not closed)
    gap = dense_gap[np.clip(np.searchsorted(s, s_out, side='right') - 1, 0, len(dense_gap) - 1)]
    return xy, s[-1], gap


def _gap_of(xy, handles, closed):
    """Handle gap of every point of a path through the handles (by order along it)."""
    at = np.array([int(np.argmin(np.linalg.norm(xy - h, axis=1))) for h in handles])
    gap = np.zeros(len(xy), dtype=int)
    for g in range(len(handles) - (0 if closed else 1)):
        a = at[g]
        b = at[(g + 1) % len(handles)] if g + 1 < len(handles) else len(xy)
        if b > a:
            gap[a:b] = g
        elif closed:      # the last gap wraps past the end
            gap[a:] = g
            gap[:b] = g
    if not closed and len(handles) > 1:
        gap[at[-1]:] = len(handles) - 2
    return gap


def handles_from_path(xy, closed=None):
    """Editable points for a saved path, which stores only the smoothed result: (points, closed).

    closed=None tells it from the ends of xy (as read from a CSV)."""
    if len(xy) < 2:
        return xy, True if closed is None else closed
    if closed is None:
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


def smoothing_fit(xy, closed, deviation, pinned=0):
    """A smoothing spline through a path, allowed about `deviation` [m] off it, resampled every
    SPACING. Unlike moving points one by one this has no kinks: the whole curve eases at once.

    pinned: an open path keeps this many points at each end in place (so a piece still meets the
    rest of the path where and along the direction it did)."""
    pts = np.vstack([xy, xy[:1]]) if closed else np.asarray(xy, dtype=float)
    w = np.ones(len(pts))
    if pinned and not closed:
        w[:pinned] = w[-pinned:] = 1e3
    tck, _ = splprep(pts.T, w=w, s=len(pts) * deviation ** 2, k=3, per=closed)
    dense = np.array(splev(np.linspace(0, 1, len(pts) * 3), tck)).T
    seg = np.linalg.norm(np.diff(dense, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(2, int(s[-1] / SPACING))
    s_new = np.linspace(0, s[-1], n, endpoint=not closed)
    out = np.column_stack([np.interp(s_new, s, dense[:, 0]), np.interp(s_new, s, dense[:, 1])])
    if pinned and not closed:
        out[0], out[-1] = pts[0], pts[-1]
    return out


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


def speed_profile(xy, closed, vmax, vmin, a_lat, a_lon, a_brake=None):
    """Curvature-limited speed, then forward/backward passes for accel/brake limits.

    a_lon is the speed-up (how soon the car gets going out of a corner), a_brake the slow-down
    before one (defaults to a_lon). Keep a_brake at or below pure_pursuit's stop_decel, or the
    car cannot slow down as fast as the CSV assumes and reaches corners too fast."""
    a_brake = a_lon if a_brake is None else a_brake
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
            v[j] = min(v[j], np.sqrt(v[n] ** 2 + 2 * a_brake * ds[j]))
    return np.maximum(v, vmin)


def race_profile(xy, closed, vmax, vmin, a_lat, a_lon, a_brake, marks=(), a_boost=5.0, a_full=4.0,
                 minor_dip=0.5):
    """speed_profile with full-throttle and full-brake marks: (speeds, zones).

    marks: (waypoint index, BOOST or BRAKE). From a BOOST mark the car speeds up at a_boost (up to
    vmax and what the curves allow) until it has to brake at a_full for the next corner, then
    brakes at a_full into it (that part is zone BRAKE). A BRAKE mark starts that full braking
    right there, also cutting a boost short, down to the speed of the next corner. Every run ends
    at the next mark at the latest. A BOOST mark placed before a corner's slowest point starts
    there. A dip in curve speed less than minor_dip [m/s] below the top speed of the run is not a
    corner (curvature noise, a kink inside one corner): the boost carries on after it. Zones are '' where the normal profile applies."""
    v = speed_profile(xy, closed, vmax, vmin, a_lat, a_lon, a_brake)
    n = len(xy)
    zone = np.array([''] * n, dtype=object)
    if not len(marks) or n < 3:
        return v, zone
    k = curvature(xy, closed)
    with np.errstate(divide='ignore'):
        vc = np.maximum(np.minimum(vmax, np.sqrt(a_lat / np.maximum(k, 1e-6))), vmin)
    ds = np.linalg.norm(np.diff(xy, axis=0, append=xy[:1] if closed else xy[-1:]), axis=1)
    # the latest the car can brake at a_full for every corner ahead
    vb = vc.copy()
    for _ in range(2 if closed else 1):
        for i in range(n - 2, -1 if not closed else -2, -1):
            j, nx = i % n, (i + 1) % n
            vb[j] = min(vb[j], np.sqrt(vb[nx] ** 2 + 2 * a_full * ds[j]))
    marks = sorted({int(i) % n: kind for i, kind in marks}.items())
    out = v.copy()
    for m, (i, kind) in enumerate(marks):
        if m + 1 < len(marks):
            stop = marks[m + 1][0]
        else:
            stop = marks[0][0] + n if closed else n - 1
        if kind == BOOST:    # placed in a corner: flat out from its slowest point on
            while i < stop and vb[(i + 1) % n] <= vb[i % n] + 1e-9:
                i += 1
            i %= n if closed else n + 1
        cur, braking, target = out[i], kind == BRAKE, None
        zone[i], top, slowing = kind, cur, []
        if braking:   # down to the next corner: the first dip in curve speed below the speed here
            ahead = [s % n for s in range(i + 1, stop + 1)] if closed else range(i + 1, stop + 1)
            dips = [q for q in ahead if vc[q] < cur and vc[q] <= vc[(q + 1) % n] if q + 1 < n or closed]
            target = vc[dips[0]] if dips else cur
        for step in range(i, stop):
            j, q = step % n, (step + 1) % n
            if not braking:
                up = np.sqrt(cur ** 2 + 2 * a_boost * ds[j])
                # through a curve the curve speed caps it; only braking for one ahead ends it
                if up <= vb[q] or vb[q] >= vc[q] - 1e-6:
                    cur, zone[q] = min(up, vb[q]), BOOST
                    out[q], top, slowing = cur, max(top, cur), []
                    continue
                braking = True
            if target is None:      # braking out of a boost: just as hard as the corner needs
                if vb[q] <= v[q] + 1e-6:
                    if top - vb[q] < minor_dip:     # not a real corner: keep going flat out
                        braking, cur, zone[q] = False, vb[q], BOOST
                        out[q] = cur
                        zone[slowing] = BOOST       # one zone, shown as one
                        continue
                    break
                cur, zone[q] = vb[q], BRAKE
                slowing.append(q)
            else:
                down = np.sqrt(max(cur ** 2 - 2 * a_full * ds[j], 0.0))
                if v[q] <= target + 1e-6:
                    break          # reached the corner: the normal profile is slower from here
                cur = min(max(down, target), vb[q])
                if down > target:
                    zone[q] = BRAKE
            out[q] = min(cur, vb[q]) if zone[q] else min(cur, v[q])
    # after a run the normal profile picks up again with its own speed-up
    for _ in range(2 if closed else 1):
        for i in range(1, n if not closed else n + 1):
            j, p = i % n, (i - 1) % n
            if not zone[j]:
                out[j] = min(out[j], np.sqrt(out[p] ** 2 + 2 * a_lon * ds[p]))
    return np.maximum(out, vmin), zone


def left_normals(xy, closed):
    nxt = np.roll(xy, -1, 0) if closed else np.vstack([xy[1:], xy[-1:]])
    prv = np.roll(xy, 1, 0) if closed else np.vstack([xy[:1], xy[:-1]])
    t = nxt - prv
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
    return np.column_stack([-t[:, 1], t[:, 0]])


def signed_curvature(xy, closed, window=9):
    """Curvature [1/m], positive turning left, averaged over `window` points (about 1 m)."""
    nxt = np.roll(xy, -1, 0) if closed else np.vstack([xy[1:], xy[-1:]])
    prv = np.roll(xy, 1, 0) if closed else np.vstack([xy[:1], xy[:-1]])
    turn = np.sign((xy[:, 0] - prv[:, 0]) * (nxt[:, 1] - xy[:, 1])
                   - (xy[:, 1] - prv[:, 1]) * (nxt[:, 0] - xy[:, 0]))
    k = curvature(xy, closed) * turn
    if window > 1 and len(k) > window:
        kern = np.ones(window) / window
        k = (np.convolve(np.concatenate([k[-window:], k, k[:window]]), kern, 'same')[window:-window]
             if closed else np.convolve(k, kern, 'same'))
    return k


def wall_margins(xy, closed, v, base, ld_min=0.6, ld_gain=0.4, slip=0.03, cap=0.2):
    """Wall distance to keep on the left and right of every point [m], (left, right).

    base everywhere, plus on the inside of a curve the corner pure pursuit cuts (about L^2 k / 2
    for lookahead L = ld_min + ld_gain v) and on the outside slip * lateral acceleration v^2 k.
    Each extra is capped at `cap`; cap 0 gives base on both sides."""
    if cap <= 0:
        return np.full(len(xy), base), np.full(len(xy), base)
    k = signed_curvature(xy, closed)
    look = ld_min + ld_gain * np.asarray(v)
    inner = np.minimum(cap, look ** 2 * np.abs(k) / 2)
    outer = np.minimum(cap, slip * np.asarray(v) ** 2 * np.abs(k))
    left = base + np.where(k > 0, inner, outer)
    right = base + np.where(k > 0, outer, inner)
    return left, right


def side_room(xy, normals, clear_fn, margin_left, margin_right, reach=2.0, step=0.02):
    """How far every point can move left (+) and right (-) along its normal while clear_fn
    (wall distance of Nx2 points) stays at least the margin on that side: (lo <= 0, hi >= 0)."""
    t = np.arange(step, reach + step / 2, step)
    hi, lo = np.zeros(len(xy)), np.zeros(len(xy))
    for sign, margin, out in ((1, margin_left, hi), (-1, margin_right, lo)):
        pts = xy[:, None, :] + sign * t[None, :, None] * normals[:, None, :]
        ok = clear_fn(pts.reshape(-1, 2)).reshape(len(xy), len(t)) >= np.asarray(margin)[:, None]
        first_bad = np.where(ok.all(1), len(t), np.argmin(ok, axis=1))
        out[:] = sign * np.where(first_bad > 0, t[np.maximum(first_bad - 1, 0)], 0.0)
    return lo, hi


def racing_line(xy, closed, clear_fn, margins_fn, fixed=3, iters=5):
    """Minimum-curvature line (out-in-in-out) between the walls.

    Every point moves sideways along its normal so the sum of squared second differences (the
    curvature) is smallest, while staying inside the room side_room() gives with the margins
    from margins_fn(xy) -> (left, right). An open piece keeps its first and last `fixed` points,
    so it still meets the rest of the path along the same direction. Redone `iters` times on the
    result, as the normals and margins change with the line."""
    xy = np.asarray(xy, dtype=float).copy()
    for _ in range(iters):
        n = len(xy)
        if n < (3 if closed else 2 * fixed + 1):
            return xy
        nrm = left_normals(xy, closed)
        ml, mr = margins_fn(xy)
        lo, hi = side_room(xy, nrm, clear_fn, ml, mr)
        if not closed:
            lo[:fixed] = hi[:fixed] = lo[-fixed:] = hi[-fixed:] = 0.0
        hi = np.maximum(hi, lo + 1e-9)
        rows = range(n) if closed else range(1, n - 1)
        d2 = np.zeros((len(rows), n))
        for r, i in enumerate(rows):
            d2[r, (i - 1) % n] += 1.0
            d2[r, i] -= 2.0
            d2[r, (i + 1) % n] += 1.0
        a = np.vstack([d2 * nrm[:, 0], d2 * nrm[:, 1]])
        b = -np.concatenate([d2 @ xy[:, 0], d2 @ xy[:, 1]])
        off = lsq_linear(a, b, bounds=(lo, hi)).x
        moved = xy + off[:, None] * nrm
        if closed:
            moved = np.vstack([moved, moved[:1]])
        xy = resample(moved, closed)
        if not closed:   # resampling keeps the ends; keep the exact end points too
            xy[0], xy[-1] = moved[0], moved[-1]
    return xy


def read_csv(path):
    """The x, y columns of a file from write_csv, as an Nx2 array (a file may have no rows)."""
    with open(path, newline='') as f:
        rows = [(float(r['x']), float(r['y'])) for r in csv.DictReader(f)]
    return np.array(rows).reshape(-1, 2)


def read_marks(path):
    """(waypoint index, BOOST or BRAKE) for every row with a mark in a file from write_csv."""
    with open(path, newline='') as f:
        return [(i, r['mark']) for i, r in enumerate(csv.DictReader(f)) if r.get('mark')]


def write_csv(path, xy, v, zone=None, mark=None):
    """x,y,speed; plus zone (BOOST / BRAKE where pure_pursuit drives flat out) and mark (where the
    editor's marks sit) only when there are any, so plain routes keep the legacy format."""
    extra = zone is not None and any(zone) or mark is not None and any(mark)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'speed'] + (['zone', 'mark'] if extra else []))
        for i, ((x, y), s) in enumerate(zip(xy, v)):
            row = [f'{x:.4f}', f'{y:.4f}', f'{s:.3f}']
            if extra:
                row += [zone[i] if zone is not None else '', mark[i] if mark is not None else '']
            w.writerow(row)
