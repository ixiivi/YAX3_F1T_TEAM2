"""Relocalization reach (2026-11-14): stopped at a wall the search jumped 9-13 m to a wrong place
at fit 91-100%. The answer has to lie within what the car can have driven since its last sure
pose; twice cut back, the whole map is allowed again."""
import math
from types import SimpleNamespace

import numpy as np
from test_match_obstacles import place
from test_obstacle_filter import lobby, poses  # noqa: F401


def prepare(pp):
    pp.params.update({'accept_fit': 0.75, 'route_prior_dist': 0.0, 'global_heading_tol_deg': 0.0,
                      'resume_fit': 0.6, 'global_min_gain': 0.10, 'jump_free_after': 2,
                      'jump_base': 1.0, 'jump_gain': 0.5, 'match_obstacle_margin': 0.0})
    pp.far_cuts = 0
    pp.get_logger = lambda: SimpleNamespace(info=lambda *_: None)


def test_reach_moves_the_trusted_pose_by_odom():
    from waypoint_nav.pure_pursuit import PurePursuit
    pp = object.__new__(PurePursuit)
    pp.params = {'jump_base': 1.0, 'jump_gain': 0.5, 'jump_free_after': 2}
    pp.gp = pp.params.get
    pp.far_cuts = 0
    # trusted at map (10, 0) facing +y while odom read (0, 0) facing +x; then 2 m forward
    pp.trust = ((10.0, 0.0, math.pi / 2), (0.0, 0.0, 0.0), 5.0)
    pp.odom_path = 7.0
    pp.lookup = lambda frame, stamp=None: (2.0, 0.0, 0.1)
    x, y, yaw, rad = PurePursuit.reach(pp)
    assert abs(x - 10.0) < 1e-9 and abs(y - 2.0) < 1e-9 and abs(yaw - (math.pi / 2 + 0.1)) < 1e-9
    assert abs(rad - 2.0) < 1e-9
    pp.far_cuts = 2
    assert PurePursuit.reach(pp) is None
    pp.far_cuts, pp.trust = 0, None
    assert PurePursuit.reach(pp) is None


def test_a_far_answer_is_cut_back_to_the_reach(lobby):
    from waypoint_nav.pure_pursuit import PurePursuit
    pp = lobby
    prepare(pp)
    i, x, y, yaw = next(poses(pp))
    place(pp, (x, y, yaw), None)
    scan = None
    # the estimate is 1.5 m off (so the near search fails), the reach is around a place 4 m away
    guess = (x + 1.5, y, yaw)
    PurePursuit._search(pp, scan, guess)
    free = pp.search_result
    assert math.hypot(free[1] - x, free[2] - y) < 0.3          # unlimited: finds the car
    cx, cy = x + 4.0 * math.cos(yaw + 1.2), y + 4.0 * math.sin(yaw + 1.2)
    PurePursuit._search(pp, scan, guess, limit=(cx, cy, yaw, 1.0))
    cut = pp.search_result
    assert math.hypot(cut[1] - cx, cut[2] - cy) <= 1.0 + 1e-6
    assert pp.far_cuts == 1
    # a reach that holds the car changes nothing
    pp.far_cuts = 0
    PurePursuit._search(pp, scan, guess, limit=(x + 0.5, y, yaw, 1.5))
    assert math.hypot(pp.search_result[1] - x, pp.search_result[2] - y) < 0.3
    assert pp.far_cuts == 0
