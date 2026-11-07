"""Scan matching on the real saved maps, with scans ray-cast from a known pose."""
import math
import time

import numpy as np
import pytest
from conftest import map_path, waypoint_path

from waypoint_nav.path_tools import curvature, speed_profile
from waypoint_nav.pure_pursuit import load_waypoints
from waypoint_nav.scan_match import MapMatcher

LASER_X = 0.267


def raycast(m, x, y, yaw, n=1081, fov=math.radians(270), max_range=8.0):
    """Synthetic LaserScan ranges by sphere tracing the map's wall distance field."""
    a = -fov / 2 + np.arange(n) * fov / (n - 1)
    cx, cy = x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw)
    r = np.full(n, 0.05)
    hit = np.zeros(n, bool)
    for _ in range(400):
        px, py = cx + r * np.cos(yaw + a), cy + r * np.sin(yaw + a)
        d = m.wall_dist(px, py)
        hit |= d < m.res
        r = np.where(hit, r, r + np.maximum(d, m.res * 0.5))
        if (hit | (r > max_range)).all():
            break
    return np.where(hit & (r < max_range), r, np.inf), a


def valid(r, a):
    ok = np.isfinite(r)
    return r[ok], a[ok]


@pytest.fixture(scope='module')
def lobby():
    m = MapMatcher(map_path('lobby'), LASER_X)
    xy, _ = load_waypoints(waypoint_path('lobby'))
    return m, xy


def pose_on_path(xy, i):
    d = xy[(i + 3) % len(xy)] - xy[i]
    return float(xy[i, 0]), float(xy[i, 1]), math.atan2(d[1], d[0])


@pytest.mark.parametrize('i', [0, 70, 140, 210])
def test_true_pose_fits(lobby, i):
    m, xy = lobby
    x, y, yaw = pose_on_path(xy, i)
    r, a = valid(*raycast(m, x, y, yaw))
    assert m.fit(x, y, yaw, r, a) > 0.95


@pytest.mark.parametrize('i', [0, 70, 140, 210])
def test_local_search_recovers_a_slipped_pose(lobby, i):
    m, xy = lobby
    x, y, yaw = pose_on_path(xy, i)
    r, a = valid(*raycast(m, x, y, yaw))
    r, a = r[::3], a[::3]
    sc, bx, by, byaw = m.local_search(x + 0.25, y - 0.15, yaw + math.radians(6), r, a)
    assert math.hypot(bx - x, by - y) < 0.06
    assert abs(math.degrees(math.atan2(math.sin(byaw - yaw), math.cos(byaw - yaw)))) < 2.0


def test_drift_search_is_fast_enough_to_run_while_driving(lobby):
    """_track runs local_search(radius 0.5, +-15 deg) in a thread while the 40 Hz loop drives.
    Records the time so a slower change shows up; the Jetson is ~3-5x slower than a laptop."""
    m, xy = lobby
    x, y, yaw = pose_on_path(xy, 100)
    r, a = valid(*raycast(m, x, y, yaw))
    t0 = time.perf_counter()
    m.local_search(x + 0.1, y, yaw, r[::3], a[::3], radius=0.5, yaw_range_deg=15.0)
    dt = time.perf_counter() - t0
    print(f'\nlocal_search(0.5 m, 15 deg): {dt * 1000:.0f} ms on this machine')
    assert dt < 2.0


@pytest.mark.parametrize('name', ['lobby', 'DLAB', 'map_1101_2135', 'map_1101_1950'])
def test_saved_waypoints_respect_their_own_limits(name):
    xy, v = load_waypoints(waypoint_path(name))
    closed = np.linalg.norm(xy[0] - xy[-1]) < 0.5
    k = curvature(xy, closed)
    a_lat = v ** 2 * k
    print(f'\n{name}: {len(xy)} pts, v {v.min():.2f}..{v.max():.2f}, '
          f'max lateral accel {a_lat.max():.2f} m/s^2, {"closed" if closed else "open"}')
    assert np.isfinite(v).all() and (v > 0).all()
    m = MapMatcher(map_path(name), LASER_X)
    assert m.inside(xy[:, 0], xy[:, 1]).mean() > 0.9


def test_speed_profile_obeys_lateral_and_longitudinal_limits():
    th = np.linspace(0, 2 * np.pi, 200, endpoint=False)
    xy = np.column_stack([2.0 * np.cos(th), 1.0 * np.sin(th)])
    v = speed_profile(xy, True, vmax=3.0, vmin=0.5, a_lat=2.0, a_lon=1.5)
    k = curvature(xy, True)
    assert (v ** 2 * k <= 2.0 + 1e-6).all()
    ds = np.linalg.norm(np.roll(xy, -1, 0) - xy, axis=1)
    assert (np.abs(np.diff(np.append(v, v[0]) ** 2)) <= 2 * 1.5 * ds + 1e-6).all()
