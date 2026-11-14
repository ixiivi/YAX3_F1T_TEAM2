"""Relocalizing on the move (2026-11-14): standing 2-24 s per search cost whole laps. The car
rolls into the widest gap at search_drive_speed, the pose found for the search scan is carried
on by the odom since."""
import math

from waypoint_nav.pure_pursuit import PurePursuit


def bare(speed, gap=0.1, blocked=False):
    pp = object.__new__(PurePursuit)
    pp.params = {'search_drive_speed': speed}
    pp.gp = pp.params.get
    pp.sent, pp.trail_calls = [], 0
    pp.gap_heading = lambda: gap
    pp.ramp = lambda v, zone='', steer=0.0: v
    pp.floor = lambda v, steer: v
    pp.aeb_reach = lambda v: 0.5
    pp.arc_obstacle = lambda steer, reach: (1.0, 0.0) if blocked else None
    pp.drive = lambda v, st: pp.sent.append((v, st))
    pp.record_trail = lambda: setattr(pp, 'trail_calls', pp.trail_calls + 1)
    return pp


def test_creep_rolls_into_the_gap():
    pp = bare(1.0)
    PurePursuit.creep(pp)
    assert pp.sent[-1] == (1.0, 0.1) and pp.trail_calls == 1


def test_creep_stops_when_blocked_or_no_gap_or_off():
    for pp in (bare(1.0, blocked=True), bare(1.0, gap=None), bare(0.0)):
        PurePursuit.creep(pp)
        assert pp.sent[-1] == (0.0, 0.0) and pp.speed_cmd == 0.0


def test_carry_on_moves_the_found_pose_by_the_odom_since():
    pp = object.__new__(PurePursuit)
    # odom: the car went 1 m forward and turned 0.2 rad since the search scan
    pp.lookup = lambda frame, stamp=None: (1.0, 0.0, 0.2)
    x, y, yaw = PurePursuit.carry_on(pp, 5.0, 5.0, math.pi / 2, (0.0, 0.0, 0.0))
    assert abs(x - 5.0) < 1e-9 and abs(y - 6.0) < 1e-9 and abs(yaw - (math.pi / 2 + 0.2)) < 1e-9
    assert PurePursuit.carry_on(pp, 1.0, 2.0, 0.3, None) == (1.0, 2.0, 0.3)
