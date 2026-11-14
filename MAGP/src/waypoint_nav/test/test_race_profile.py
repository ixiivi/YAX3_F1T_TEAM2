"""Full-throttle / full-brake / back-to-track marks on the speed profile (path_tools)."""
import numpy as np
import pytest

from waypoint_nav import path_tools as pt

LIMITS = dict(vmax=6.0, vmin=0.5, a_lat=2.0, a_lon=3.0, a_brake=1.5)


def oval(r=3.0, straight=12.0):
    """A closed oval: two straights joined by half circles, waypoints SPACING apart."""
    t = np.linspace(-np.pi / 2, np.pi / 2, 200)
    right = np.column_stack([straight / 2 + r * np.cos(t), r * np.sin(t)])
    left = np.column_stack([-straight / 2 - r * np.cos(t), -r * np.sin(t)])
    top = np.column_stack([np.linspace(straight / 2, -straight / 2, 50), np.full(50, r)])
    bottom = np.column_stack([np.linspace(-straight / 2, straight / 2, 50), np.full(50, -r)])
    return pt.resample(np.vstack([bottom, right, top, left, bottom[:1]]), True)


def profile(xy, marks, vmax=LIMITS['vmax']):
    return pt.race_profile(xy, True, vmax, LIMITS['vmin'], LIMITS['a_lat'],
                           LIMITS['a_lon'], LIMITS['a_brake'], marks)


def test_full_throttle_runs_past_curve_speeds_until_the_next_mark():
    xy = oval()
    track = profile(xy, [], vmax=20.0)[0]
    i, j = 0, len(xy) // 3                    # through the right-hand curve
    # a high cap, so the speed-up itself is what is checked (the cap has its own test)
    v, zone = profile(xy, [(i, pt.BOOST), (j, pt.RESUME)], vmax=20.0)
    assert (zone[i:j + 1] == pt.BOOST).all()
    assert v[j] > track[i:j + 1].max() + 0.5  # it did not slow down for the curve
    ds = np.linalg.norm(np.diff(xy[i:j + 1], axis=0), axis=1).sum()
    assert v[j] == pytest.approx(np.sqrt(v[i] ** 2 + 2 * pt.BOOST_ACCEL * ds), rel=0.02)
    # after back-to-track it slows down at stop_decel to the track speed, without a zone
    after = slice(j + 1, j + 40)
    assert not zone[after].any()
    assert (np.diff(v[j:j + 40]) <= 1e-9).all()


def test_full_brake_stops_at_the_track_speed_and_never_holds_a_slow_speed():
    xy = oval()
    track = profile(xy, [])[0]
    i, j = 0, 30
    v, zone = profile(xy, [(i, pt.BOOST), (j, pt.BRAKE)])
    brake = np.where(zone == pt.BRAKE)[0]
    assert brake[0] == j and len(brake) > 2
    end = brake[-1] + 1
    assert v[end] == pytest.approx(track[end])          # handed back at the track speed
    assert np.allclose(v[end:end + 50], track[end:end + 50])
    assert (v[brake] >= track[brake] - 1e-9).all()      # never below the track speed


def test_a_boost_near_the_end_of_a_loop_covers_only_up_to_its_next_mark():
    xy = oval()
    n = len(xy)
    track = profile(xy, [])[0]
    v, zone = profile(xy, [(n - 10, pt.BOOST), (15, pt.RESUME)])
    on = np.where(zone == pt.BOOST)[0]
    assert set(on) == set(range(n - 10, n)) | set(range(0, 16))
    far = slice(60, n - 60)
    assert np.allclose(v[far], track[far])


def test_marks_without_a_boost_change_nothing_above_the_track_speed():
    xy = oval()
    track = profile(xy, [])[0]
    v, zone = profile(xy, [(10, pt.BRAKE), (80, pt.RESUME)])
    assert np.allclose(v, track) and not zone.any()


def test_full_throttle_never_goes_above_the_top_speed():
    """The car's max_speed caps a full-throttle zone, so the plan (and the brake after it)
    must start from that speed, not from the speed it would reach without a cap."""
    xy = oval()
    i, j = 0, len(xy) // 3
    for vmax in (2.0, 4.0, 6.0):
        v, zone = profile(xy, [(i, pt.BOOST), (j, pt.BRAKE)], vmax=vmax)
        assert v.max() <= vmax + 1e-9
        assert (zone[i:j] == pt.BOOST).all()                # still a full-throttle zone up to the brake
        assert v[j] == pytest.approx(vmax)                   # it reached the cap and held it


def test_full_brake_after_a_zone_that_wraps_past_the_lap_start():
    """A full-throttle zone that runs over the end of the lap into the first points: the brake
    mark after it has a lower index, but must still start from the full-throttle speed (it was
    skipped, and the planned speed dropped to the track speed in one step)."""
    xy = oval()
    n = len(xy)
    track = profile(xy, [])[0]
    boost, brake = n - n // 6, n // 8              # the zone runs from boost past index 0 to brake
    v, zone = profile(xy, [(boost, pt.BOOST), (brake, pt.BRAKE)])
    assert v[brake] > track[brake] + 0.5           # it arrives faster than the track speed ...
    assert zone[brake] == pt.BRAKE and zone[(brake + 1) % n] == pt.BRAKE   # ... so it brakes
    ds = np.linalg.norm(np.diff(xy, axis=0, append=xy[:1]), axis=1)
    drop = v[brake:brake + 60] ** 2 - v[brake + 1:brake + 61] ** 2
    # never faster than full braking from one point to the next (2 a ds), no step down
    assert (drop <= 2 * pt.BRAKE_DECEL * ds[brake:brake + 60] + 1e-6).all()
    # same answer as the same marks on a path that starts at the full-throttle mark
    v2, zone2 = profile(np.roll(xy, -boost, axis=0), [(0, pt.BOOST), ((brake - boost) % n, pt.BRAKE)])
    assert np.allclose(np.roll(v2, boost), v) and (np.roll(zone2, boost) == zone).all()
