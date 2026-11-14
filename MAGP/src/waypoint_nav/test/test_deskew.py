"""Scan deskew (2026-11-14): one UST-10LX sweep takes 18.75 ms, at 5 m/s the car moves 9 cm and
the far side of the scan no longer sits on the walls of the stamp pose. rays() moves every beam
back to the stamp pose with the odom speed and yaw rate."""
import math
from types import SimpleNamespace

import numpy as np
from test_obstacle_filter import LASER_X, lobby, poses  # noqa: F401

N, T_INC = 1081, 1.736111e-05
ANG0, DANG = -2.356194496, 0.004363323


def moving_scan(pp, x, y, yaw, v, w):
    """LaserScan of a car at (x, y, yaw) at t=0 that drives at v and turns at w during the sweep:
    beam i fires at i*T_INC from the pose the car has reached by then."""
    m = pp.matcher
    r = np.arange(0.06, 5.0, 0.01)
    out = np.full(N, np.inf)
    for i0 in range(0, N, 12):
        idx = np.arange(i0, min(N, i0 + 12))
        t = (i0 + 6) * T_INC
        th = yaw + w * t
        cx = x + v * t * math.cos(yaw + w * t / 2)
        cy = y + v * t * math.sin(yaw + w * t / 2)
        lx, ly = cx + LASER_X * math.cos(th), cy + LASER_X * math.sin(th)
        ang = th + ANG0 + idx * DANG
        wx = lx + r[None, :] * np.cos(ang)[:, None]
        wy = ly + r[None, :] * np.sin(ang)[:, None]
        u, vv, ins = m._cells(wx, wy)
        hit = np.zeros(wx.shape, bool)
        hit[ins] = m.img[vv[ins], u[ins]] < 50
        ok = hit.any(1)
        out[idx[ok]] = r[hit.argmax(1)[ok]]
    return SimpleNamespace(ranges=out.tolist(), angle_min=ANG0, angle_increment=DANG,
                           time_increment=T_INC, range_min=0.06)


def setup_pp(pp, v, w, on):
    from waypoint_nav.pure_pursuit import PurePursuit
    pp.laser_x = LASER_X
    pp.car_front, pp.car_rear, pp.car_half_width = 0.42, 0.08, 0.135
    pp.odom_v, pp.odom_w = v, w
    pp.params['scan_deskew'] = on
    pp.params['scan_first_beam'] = 0.0   # moving_scan fires beam 0 at the stamp
    return lambda scan: PurePursuit.rays(pp, scan, 1)


def test_deskew_puts_a_fast_scan_back_on_the_walls(lobby):
    pp = lobby
    v, w = 5.0, 2.0
    better = n = 0
    gain = []
    for i, x, y, yaw in poses(pp, every=60):
        scan = moving_scan(pp, x, y, yaw, v, w)
        raw = pp.matcher.fit(x, y, yaw, *setup_pp(pp, v, w, False)(scan), tol=0.03)
        fixed = pp.matcher.fit(x, y, yaw, *setup_pp(pp, v, w, True)(scan), tol=0.03)
        n += 1
        better += fixed >= raw - 0.01
        gain.append(fixed - raw)
    assert n > 0 and better == n
    assert np.mean(gain) > 0.05


def test_first_beam_offset_shifts_the_reference(lobby):
    """Stamp at mid-sweep: with the offset the scan fits at the stamp pose, without it not."""
    pp = lobby
    v, w = 5.0, 0.0
    i, x, y, yaw = next(poses(pp))
    mid = N // 2 * T_INC
    # the car is at (x, y) at mid-sweep: beam 0 fired from mid * v behind
    scan = moving_scan(pp, x - v * mid * math.cos(yaw), y - v * mid * math.sin(yaw), yaw, v, w)
    rays = setup_pp(pp, v, w, True)
    zero = pp.matcher.fit(x, y, yaw, *rays(scan), tol=0.03)
    pp.params['scan_first_beam'] = -mid
    shifted = pp.matcher.fit(x, y, yaw, *PurePursuit_rays(pp, scan), tol=0.03)
    assert shifted > zero


def PurePursuit_rays(pp, scan):
    from waypoint_nav.pure_pursuit import PurePursuit
    return PurePursuit.rays(pp, scan, 1)


def test_deskew_changes_nothing_when_standing(lobby):
    pp = lobby
    i, x, y, yaw = next(poses(pp))
    scan = moving_scan(pp, x, y, yaw, 0.0, 0.0)
    r0, a0 = setup_pp(pp, 0.0, 0.0, False)(scan)
    r1, a1 = setup_pp(pp, 0.0, 0.0, True)(scan)
    assert np.allclose(r0, r1) and np.allclose(a0, a1)
