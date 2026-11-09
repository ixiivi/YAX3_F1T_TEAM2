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
