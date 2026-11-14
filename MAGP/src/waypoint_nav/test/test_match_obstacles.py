"""Drift fix with a car ahead in the lane (2026-11-14): its points are not on the map, and a
pose shifted sideways could lay them on the next wall. match_rays leaves them out."""
import math

import numpy as np
from test_obstacle_filter import LASER_X, lobby, poses, scan_from  # noqa: F401


def rays_of(body):
    return np.hypot(body[:, 0] - LASER_X, body[:, 1]), np.arctan2(body[:, 1], body[:, 0] - LASER_X)


def place(pp, pose, disk):
    pp.laser_x = LASER_X
    pp.params['match_obstacle_margin'] = 0.20
    body = scan_from(pp, *pose, disk=disk)
    pp.scan_xy = body
    pp.last_obs = pp.obstacles_map(*pose)
    pp.rays = lambda scan, step: rays_of(body)
    return rays_of(body)


def test_obstacle_rays_are_dropped_and_the_search_stays(lobby):
    from waypoint_nav.pure_pursuit import PurePursuit
    pp = lobby
    worse = checked = 0
    for i, x, y, yaw in poses(pp, every=40):
        disk = (x + 1.2 * math.cos(yaw), y + 1.2 * math.sin(yaw), 0.15)
        if pp.matcher.wall_dist(np.array([disk[0]]), np.array([disk[1]]))[0] < 0.4:
            continue
        r, a = place(pp, (x, y, yaw), disk)
        if pp.last_obs is None:
            continue
        fr, fa = PurePursuit.match_rays(pp, None, 3, (x, y, yaw))
        assert len(fr) < len(r)                      # the car ahead is left out ...
        assert len(fr) > len(r) // 2                 # ... and nothing else
        start = (x, y + 0.0, yaw)
        e_all = math.hypot(*(np.array(pp.matcher.local_search(*start, r, a, radius=0.5,
                                                              yaw_range_deg=15)[1:3]) - (x, y)))
        e_cut = math.hypot(*(np.array(pp.matcher.local_search(*start, fr, fa, radius=0.5,
                                                              yaw_range_deg=15)[1:3]) - (x, y)))
        checked += 1
        worse += e_cut > max(e_all, 0.10) + 0.02
    assert checked > 0
    assert worse == 0


def test_off_when_margin_is_zero(lobby):
    from waypoint_nav.pure_pursuit import PurePursuit
    pp = lobby
    i, x, y, yaw = next(poses(pp))
    r, a = place(pp, (x, y, yaw), (x + 1.2 * math.cos(yaw), y + 1.2 * math.sin(yaw), 0.15))
    pp.params['match_obstacle_margin'] = 0.0
    fr, _ = PurePursuit.match_rays(pp, None, 3, (x, y, yaw))
    assert len(fr) == len(r)
