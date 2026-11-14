"""Trail of recent scans for relocalizing and the wrong-way check (2026-11-14)."""
import math
import time
from collections import deque
from types import SimpleNamespace

import numpy as np
from test_jump_limit import prepare
from test_match_obstacles import rays_of
from test_obstacle_filter import LASER_X, lobby, poses, scan_from  # noqa: F401

from waypoint_nav.pure_pursuit import PurePursuit


def stamp_scan():
    from builtin_interfaces.msg import Time
    return SimpleNamespace(header=SimpleNamespace(stamp=Time()))


def test_trail_points_come_back_where_they_were():
    pp = object.__new__(PurePursuit)
    pp.params = {'trail_len': 5.0, 'trail_step': 0.25, 'trail_points': 1000}
    pp.gp = pp.params.get
    pp.laser_x, pp.scan_trail, pp.odom_path = LASER_X, deque(), 0.0
    pp.scan = stamp_scan()
    wall = np.array([[3.0, 1.0], [3.0, -1.0]])            # odom-frame points
    od = [(0.0, 0.0, 0.0)]
    pp.lookup = lambda frame, stamp=None: od[0]
    body = wall - (0.0, 0.0)
    pp.rays = lambda scan, step: rays_of(body)
    PurePursuit.store_trail(pp)
    pp.odom_path, od[0] = 0.1, (1.0, 0.0, math.pi / 2)    # too close: not kept
    PurePursuit.store_trail(pp)
    assert len(pp.scan_trail) == 1
    # seen from (1, 0) facing +y the wall at x=3 lies to the right, 2 m away
    r, a = PurePursuit.trail_rays(pp, stamp_scan()) if len(pp.scan_trail) > 1 else (None, None)
    assert r is None                                       # one scan is no trail yet
    pp.odom_path = 0.3
    PurePursuit.store_trail(pp)
    r, a = PurePursuit.trail_rays(pp, stamp_scan())
    bx, by = LASER_X + r * np.cos(a), r * np.sin(a)
    # body frame at (1, 0, 90 deg): odom point (3, 1) -> (1, -2), (3, -1) -> (-1, -2)
    first = np.column_stack([bx, by])[:2]
    assert np.allclose(first, [[1.0, -2.0], [-1.0, -2.0]], atol=1e-9)
    pp.odom_path = 9.0
    PurePursuit.store_trail(pp)
    assert all(9.0 - p <= 5.0 for p, _ in pp.scan_trail)


def narrow(body, deg=25.0):
    """Only the beams within +-deg ahead: the car nose-on to a wall sees about this."""
    a = np.arctan2(body[:, 1], body[:, 0] - LASER_X)
    return body[np.abs(a) < math.radians(deg)]


def test_the_trail_finds_the_place_a_narrow_scan_cannot(lobby):
    pp = lobby
    prepare(pp)
    pp.laser_x = LASER_X
    pp.car_front, pp.car_rear, pp.car_half_width = 0.42, 0.08, 0.135
    pp.params.update({'trail_len': 5.0, 'trail_points': 150})
    right = {'scan': 0, 'trail': 0}
    tried = 0
    for i, x, y, yaw in list(poses(pp, every=90))[:3]:
        body = narrow(scan_from(pp, x, y, yaw))
        if len(body) < 20:
            continue
        pp.last_obs = None
        pp.rays = lambda scan, step, b=body: rays_of(b)
        # trail: full scans from the 3 m of route before, odom frame = map frame
        pts = []
        for k in range(i - 12, i, 2):
            k %= len(pp.xy)
            j = (k + 3) % len(pp.xy)
            kyaw = math.atan2(*(pp.xy[j] - pp.xy[k])[::-1])
            b = scan_from(pp, pp.xy[k, 0], pp.xy[k, 1], kyaw)[::6]
            c, s = math.cos(kyaw), math.sin(kyaw)
            pts.append(np.column_stack([pp.xy[k, 0] + c * b[:, 0] - s * b[:, 1],
                                        pp.xy[k, 1] + s * b[:, 0] + c * b[:, 1]]))
        pp.scan_trail = deque((0.0, p) for p in pts)
        pp.lookup = lambda frame, stamp=None, q=(x, y, yaw): q
        trail = PurePursuit.trail_rays(pp, stamp_scan())
        tried += 1
        for key, tr in (('scan', None), ('trail', trail)):
            PurePursuit._search(pp, None, None, None, None, tr)
            res = pp.search_result
            right[key] += math.hypot(res[1] - x, res[2] - y) < 0.3
    assert tried > 0
    assert right['trail'] == tried and right['trail'] >= right['scan']


def test_wrong_way_fires_after_its_time_and_then_holds():
    pp = object.__new__(PurePursuit)
    pp.params = {'wrong_way_deg': 120.0, 'wrong_way_time': 0.05}
    pp.gp = pp.params.get
    pp.xy = np.column_stack([np.arange(10.0), np.zeros(10)])   # route along +x
    pp.mode, pp.odom_v, pp.odom_path = 'drive', 2.0, 0.0
    pp.wrong_since, pp.wrong_hold = None, 0.0
    assert not PurePursuit.wrong_way(pp, 5, 0.3)
    assert not PurePursuit.wrong_way(pp, 5, math.pi)           # not for long yet
    time.sleep(0.08)
    assert PurePursuit.wrong_way(pp, 5, math.pi)
    time.sleep(0.08)
    assert not PurePursuit.wrong_way(pp, 5, math.pi)           # holds for the next 3 m ...
    pp.odom_path = 3.5
    PurePursuit.wrong_way(pp, 5, math.pi)
    time.sleep(0.08)
    assert PurePursuit.wrong_way(pp, 5, math.pi)               # ... then judges again
    pp.odom_v = 0.2
    assert not PurePursuit.wrong_way(pp, 5, math.pi)           # standing / creeping: never


def test_scan_trail_and_reverse_trail_live_side_by_side():
    """2026-11-14 on the car: the scan trail was named self.trail, which the reverse path already
    used, and the node crashed seconds after starting. Both run on one node here."""
    pp = object.__new__(PurePursuit)
    pp.params = {'trail_len': 5.0, 'trail_step': 0.25, 'trail_points': 150}
    pp.gp = pp.params.get
    pp.laser_x, pp.odom_path = LASER_X, 0.0
    pp.trail, pp.scan_trail = [], deque()
    pp.scan = stamp_scan()
    od = [(0.0, 0.0, 0.0)]
    pp.lookup = lambda frame, stamp=None: od[0]
    pp.rays = lambda scan, step: rays_of(np.array([[3.0, 1.0], [3.0, -1.0]]))
    for k in range(6):
        od[0] = (0.3 * k, 0.0, 0.0)
        pp.odom_path = 0.3 * k
        PurePursuit.record_trail(pp)
        PurePursuit.store_trail(pp)
    assert len(pp.trail) >= 3 and len(pp.scan_trail) >= 3
    assert PurePursuit.trail_rays(pp, stamp_scan()) is not None
