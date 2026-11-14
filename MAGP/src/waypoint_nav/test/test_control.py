"""Properties of the pure pursuit control math that must hold for left and right alike."""
import math

import numpy as np
import pytest


def arc_point(steer, s, wheelbase=0.33, lateral=0.0):
    """Point s metres along the arc driven with this steering (base_link frame), shifted
    `lateral` metres towards the turn centre."""
    k = math.tan(steer) / wheelbase
    if abs(k) < 1e-9:
        return s, lateral
    R = 1.0 / k
    r = R - lateral
    return r * math.sin(s / R), R - r * math.cos(s / R)


@pytest.mark.parametrize('steer', [0.0, 0.05, 0.2, 0.34, -0.05, -0.2, -0.34])
@pytest.mark.parametrize('s', [0.6, 1.0, 1.5])
def test_aeb_sees_a_point_on_the_arc(bare_pp, steer, s):
    bare_pp.scan_xy = np.array([arc_point(steer, s)])
    hit = bare_pp.arc_obstacle(steer, reach=2.0)
    assert hit is not None, f'AEB blind at steer {steer:+.2f}'
    # the body reaches a point on the arc a little before base_link's arc length says
    assert s - 0.04 <= hit <= s + 0.005


@pytest.mark.parametrize('steer', [0.1, 0.3])
def test_aeb_is_mirror_symmetric(bare_pp, steer):
    pts = np.array([arc_point(steer, s, lateral=d) for s in (0.7, 1.2) for d in (-0.2, 0.1)])
    bare_pp.scan_xy = pts
    left = bare_pp.arc_obstacle(steer, 2.0)
    bare_pp.scan_xy = pts * [1.0, -1.0]
    right = bare_pp.arc_obstacle(-steer, 2.0)
    assert left == pytest.approx(right)


@pytest.mark.parametrize('steer', [0.0, 0.2, -0.2])
def test_aeb_ignores_points_behind_and_beside(bare_pp, steer):
    bare_pp.scan_xy = np.array([[-0.5, 0.0], arc_point(steer, 1.0, lateral=0.5)])
    assert bare_pp.arc_obstacle(steer, 2.0) is None


@pytest.mark.parametrize('side', [1.0, -1.0])
def test_pursuit_steers_towards_the_target(bare_pp, side):
    steer = bare_pp.pursuit_steer(0.0, 0.0, 0.0, 1.0, side * 0.3)
    assert math.copysign(1.0, steer) == side
    # the arc through the target has curvature 2*ly / d^2
    assert math.tan(steer) / 0.33 == pytest.approx(side * 2 * 0.3 / (1.0 + 0.09), rel=1e-3)


def test_ramp_never_commands_the_cogging_band(bare_pp):
    seen = []
    for target in [1.5] * 80 + [0.0] * 80 + [0.8] * 40 + [0.0] * 40:
        bare_pp.speed_cmd = bare_pp.ramp(target)
        seen.append(bare_pp.speed_cmd)
    seen = np.array(seen)
    assert not ((seen > 0.0) & (seen < bare_pp.min_speed - 1e-9)).any()
    assert seen.max() == pytest.approx(1.5)
    assert seen[-1] == 0.0


def test_ramp_respects_accel(bare_pp):
    bare_pp.speed_cmd = 0.5
    nxt = bare_pp.ramp(2.0)
    assert nxt - 0.5 == pytest.approx(bare_pp.accel * bare_pp.dt)


def test_aeb_ignores_a_wall_beside_the_front_corner(bare_pp):
    # a wall 3-8 cm outside the front corner, running along the car: turning away from it is free
    wall = np.array([[x, -(0.135 + 0.03)] for x in np.arange(0.0, 0.8, 0.01)])
    bare_pp.scan_xy = wall
    assert bare_pp.arc_obstacle(0.34, 0.42 + 0.5) is None
    # turning into it is not
    assert bare_pp.arc_obstacle(-0.34, 0.42 + 0.5) is not None


def test_escape_picks_the_free_steering_closest_to_the_wanted_one(bare_pp):
    # a wall corner 20 cm ahead reaching just under the right side of the car: left gets past it
    bare_pp.scan_xy = np.array([[0.62, y] for y in np.arange(-0.5, -0.12, 0.01)])
    assert bare_pp.arc_obstacle(0.0, 1.0) is not None
    alt = bare_pp.escape_steer(0.0, 0.42 + 0.3)
    assert alt is not None and alt > 0.0
    assert bare_pp.arc_obstacle(alt, 0.42 + 0.3) is None


def test_turn_speed_caps_full_lock_but_not_straight(bare_pp):
    bare_pp.gp = {'steer_alat': 4.0}.get
    full = bare_pp.turn_speed(0.34)          # radius about 0.93 m
    assert 1.8 < full < 2.1
    assert bare_pp.turn_speed(0.0) == float('inf')
    assert bare_pp.turn_speed(0.05) > 4.0


def test_full_throttle_and_full_brake_zones_change_only_the_ramp_rates(bare_pp, tmp_path):
    from waypoint_nav.path_tools import write_csv
    from waypoint_nav.pure_pursuit import load_zones
    pp = bare_pp
    params = {'boost_accel': 5.0, 'brake_decel': 4.0}
    pp.gp = params.get
    pp.speed_cmd = 2.0
    assert pp.ramp(5.0) == pytest.approx(2.0 + 1.5 * pp.dt)
    assert pp.ramp(5.0, 'boost') == pytest.approx(2.0 + 5.0 * pp.dt)
    assert pp.ramp(1.0, 'brake') == pytest.approx(2.0 - 4.0 * pp.dt)
    assert pp.ramp(1.0, 'boost') == pytest.approx(2.0 - 2.0 * pp.dt)
    # full throttle grows smoothly from accel at full lock to boost_accel at the centre
    steps = [(pp.ramp(5.0, 'boost', s) - 2.0) / pp.dt for s in np.linspace(pp.max_steer, 0.0, 9)]
    assert steps[0] == pytest.approx(1.5) and steps[-1] == pytest.approx(5.0)
    assert (np.diff(steps) > 0).all()
    assert steps[-1] - steps[-2] < steps[4] - steps[3]      # flat as it reaches the centre
    # a plain route has no zone column; a route with marks has one
    xy = np.column_stack([np.arange(5.0), np.zeros(5)])
    write_csv(tmp_path / 'a.csv', xy, np.ones(5))
    assert list(load_zones(tmp_path / 'a.csv')) == [''] * 5
    write_csv(tmp_path / 'b.csv', xy, np.ones(5), ['', 'boost', 'boost', 'brake', ''],
              ['', 'boost', '', '', ''])
    assert list(load_zones(tmp_path / 'b.csv')) == ['', 'boost', 'boost', 'brake', '']


def test_global_pose_is_carried_forward_by_odometry():
    from waypoint_nav.global_localize import carry_forward
    # found at (1, 2) facing +y; odometry says the car then drove 2 m straight on and turned 90 deg
    x, y, yaw = carry_forward((1.0, 2.0, math.pi / 2), (0.0, 0.0, 0.0), (2.0, 0.0, math.pi / 2))
    assert (x, y) == pytest.approx((1.0, 4.0)) and yaw == pytest.approx(math.pi)
    # odom frame rotated against the map: same motion in the car frame, same answer
    x, y, yaw = carry_forward((1.0, 2.0, math.pi / 2), (5.0, 5.0, math.pi), (3.0, 5.0, -math.pi / 2))
    assert (x, y) == pytest.approx((1.0, 4.0)) and yaw == pytest.approx(math.pi)
