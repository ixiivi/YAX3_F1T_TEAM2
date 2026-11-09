"""Track obstacle filter (ForzaETH style) on a real map and route, with a ray-cast LiDAR.

Walls must not become obstacles, also with the pose a little off; a small object on the
route must. The old rule (points far from the mapped walls) is checked for comparison.
"""
import math

import numpy as np
import pytest
from conftest import map_path, waypoint_path
from scipy.spatial import cKDTree

from waypoint_nav.pure_pursuit import PurePursuit, load_waypoints
from waypoint_nav.scan_match import MapMatcher

PARAMS = {'obstacle_filter': 'track', 'track_inflation': 0.10, 'track_width_max': 2.0,
          'obs_cluster_gap': 0.15, 'obs_min_points': 6, 'obs_max_size': 0.5,
          'obstacle_min_map_dist': 0.25, 'skew_gain': 1.0, 'skew_max': 0.35}
LASER_X = 0.267


@pytest.fixture(scope='module', params=['lobby', '11085'])
def lobby(request):
    """A planner on a real map and its route: a cluttered room and a walled lab track."""
    pp = object.__new__(PurePursuit)
    pp.name = request.param
    params = dict(PARAMS)
    pp.gp = params.get
    pp.params = params
    pp.xy, _ = load_waypoints(waypoint_path(pp.name))
    t = np.roll(pp.xy, -1, 0) - np.roll(pp.xy, 1, 0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    pp.normal = np.column_stack([-t[:, 1], t[:, 0]])
    pp.matcher = MapMatcher(map_path(pp.name), LASER_X)
    pp.d_left, pp.d_right = pp.track_widths()
    pp.route_tree = cKDTree(pp.xy)
    pp.skew, pp.n_clusters = 0.0, 0
    return pp


def scan_from(pp, x, y, yaw, disk=None):
    """Body-frame points a LiDAR at (x, y, yaw) sees on the map, plus an optional disk."""
    m = pp.matcher
    ang = np.radians(np.arange(-135, 135, 0.25))
    r = np.arange(0.06, 5.0, 0.01)
    lx, ly = x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw)
    wx = lx + r[None, :] * np.cos(yaw + ang)[:, None]
    wy = ly + r[None, :] * np.sin(yaw + ang)[:, None]
    u, v, ins = m._cells(wx, wy)
    hit = np.zeros(wx.shape, bool)
    hit[ins] = m.img[v[ins], u[ins]] < 50
    if disk is not None:
        hit |= np.hypot(wx - disk[0], wy - disk[1]) < disk[2]
    first = hit.argmax(1)
    ok = hit.any(1)
    rr = r[first][ok]
    return np.column_stack([LASER_X + rr * np.cos(ang[ok]), rr * np.sin(ang[ok])])


def poses(pp, every=12):
    for i in range(0, len(pp.xy), every):
        j = (i + 3) % len(pp.xy)
        yield i, pp.xy[i, 0], pp.xy[i, 1], math.atan2(*(pp.xy[j] - pp.xy[i])[::-1])


def count(pp, scan_pose, belief_pose, disk=None, rule='track'):
    pp.params['obstacle_filter'] = rule
    pp.scan_xy = scan_from(pp, *scan_pose, disk=disk)
    obs = pp.obstacles_map(*belief_pose)
    pp.params['obstacle_filter'] = 'track'
    return 0 if obs is None else len(obs)


def test_track_widths_are_measured_on_both_sides(lobby):
    # the lobby route crosses an open room (widths reach the cap), 11085 runs between walls
    for side in (lobby.d_left, lobby.d_right):
        assert np.median(side) > 0.3 and (side >= 0).all()


def test_walls_are_no_obstacles_with_the_right_pose(lobby):
    found = [count(lobby, (x, y, a), (x, y, a)) for _, x, y, a in poses(lobby)]
    assert max(found) == 0, found


@pytest.mark.parametrize('dx, dyaw', [(0.10, 0.0), (0.0, math.radians(5)), (0.10, math.radians(5))])
def test_walls_stay_no_obstacles_with_the_pose_off(lobby, dx, dyaw):
    track, old = [], []
    for _, x, y, a in poses(lobby):
        off = (x - dx * math.sin(a), y + dx * math.cos(a), a + dyaw)   # belief beside the truth
        track.append(count(lobby, (x, y, a), off))
        old.append(count(lobby, (x, y, a), off, rule='map'))
    print(f'\n{lobby.name}: pose off {dx:.2f} m {math.degrees(dyaw):.0f} deg: wall points as obstacles, '
          f'track filter {sum(track)} vs old rule {sum(old)} over {len(track)} poses')
    assert sum(track) <= sum(old)                       # never worse than the old rule
    if lobby.name == '11085':                           # on a walled track: a fifth or less
        assert sum(track) <= 0.2 * sum(old)


def test_a_small_object_on_the_route_is_found(lobby):
    found = 0
    tried = 0
    for i, x, y, a in poses(lobby, every=24):
        j = (i + 20) % len(lobby.xy)                    # about 2 m further along the route
        disk = (lobby.xy[j, 0], lobby.xy[j, 1], 0.10)
        if math.dist(disk[:2], (x, y)) > 3.5 or lobby.matcher.wall_dist(*disk[:2]) < 0.45:
            continue
        tried += 1
        if count(lobby, (x, y, a), (x, y, a), disk=disk) > 0:
            found += 1
    assert tried >= 3 and found == tried, (found, tried)
