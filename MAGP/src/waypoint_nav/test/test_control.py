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
    assert hit == pytest.approx(s, abs=0.01)


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
