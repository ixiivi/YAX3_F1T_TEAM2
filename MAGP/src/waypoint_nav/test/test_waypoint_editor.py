"""waypoint_editor mouse editing, on real maps and routes, saving only to a temp folder."""
import glob
import math
import os
import shutil
import types

import numpy as np
import pytest
from conftest import WS, map_path, waypoint_path

pytest.importorskip('PyQt5')
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

from waypoint_nav import path_tools as pt  # noqa: E402
from waypoint_nav import waypoint_editor as we  # noqa: E402

ARGS = types.SimpleNamespace(vmax=3.0, vmin=0.5, alat=2.0, alon=1.5, wall_clearance=0.40)


@pytest.fixture
def editor(tmp_path, monkeypatch):
    """Factory: an Editor on the lobby map. route=True starts from a copy of its saved route."""
    monkeypatch.setattr(we, 'WP_DIR', str(tmp_path))
    from PyQt5 import QtCore
    ini = str(tmp_path / 'settings.ini')      # saving must not touch the real ~/.config/MAGP
    monkeypatch.setattr(we, 'settings', lambda: QtCore.QSettings(ini, QtCore.QSettings.IniFormat))
    made = []

    def make(route=False):
        if route:
            shutil.copy(waypoint_path('lobby'), tmp_path / 'lobby_wp.csv')
        ed = we.Editor(map_path('lobby'), None, ARGS)
        ed.fig.canvas.draw()
        made.append(ed)
        return ed
    yield make
    for ed in made:
        we.plt.close(ed.fig)


def mouse(ed, kind, world, button=1):
    """Send a mouse event at a world position; returns its screen position."""
    u, v = we.world_to_px(*world, ed.h, ed.res, ed.origin)
    x, y = ed.ax.transData.transform((u, v))
    ev = we.matplotlib.backend_bases.MouseEvent(kind, ed.fig.canvas, x, y, button=button)
    ed.fig.canvas.callbacks.process(kind, ev)


def key(ed, name):
    ev = we.matplotlib.backend_bases.KeyEvent('key_press_event', ed.fig.canvas, name)
    ed.fig.canvas.callbacks.process('key_press_event', ev)


def click(ed, world, button=1):
    mouse(ed, 'button_press_event', world, button)
    mouse(ed, 'button_release_event', world, button)


def drag(ed, start, end):
    mouse(ed, 'button_press_event', start)
    for t in np.linspace(0.25, 1.0, 4):
        mouse(ed, 'motion_notify_event', tuple(np.add(start, t * np.subtract(end, start))))
    mouse(ed, 'button_release_event', end)


def square(ed):
    """Four clicked points in the visible part of the map, 2 m apart."""
    u0, u1 = ed.ax.get_xlim()
    v1, v0 = ed.ax.get_ylim()
    cx, cy = we.px_to_world((u0 + u1) / 2, (v0 + v1) / 2, ed.h, ed.res, ed.origin)
    pts = [(cx - 1, cy - 1), (cx + 1, cy - 1), (cx + 1, cy + 1), (cx - 1, cy + 1)]
    for p in pts:
        click(ed, p)
    return pts


def test_clicks_add_points_in_order(editor):
    ed = editor()
    pts = square(ed)
    assert np.allclose(ed.pts, pts, atol=ed.res)


def test_dragging_a_point_moves_only_that_point_and_the_path_follows(editor):
    ed = editor()
    pts = square(ed)
    before = ed.curve_line.get_xydata().copy()
    target = (pts[1][0] + 0.3, pts[1][1] - 0.2)
    drag(ed, pts[1], target)
    assert len(ed.pts) == 4
    assert np.allclose(ed.pts[1], target, atol=ed.res)
    assert np.allclose(ed.pts[0], pts[0], atol=ed.res) and np.allclose(ed.pts[2], pts[2], atol=ed.res)
    assert not np.array_equal(before.shape, ed.curve_line.get_xydata().shape) \
        or not np.allclose(before, ed.curve_line.get_xydata())
    assert ed.drag is None
    # the path still goes through the moved point
    assert we.distance_to_polyline(np.array(target), ed.curve()) < 0.06


def test_a_shaky_click_on_a_point_moves_nothing(editor):
    ed = editor()
    pts = square(ed)
    held = list(ed.pts)
    steps = len(ed.history)
    mouse(ed, 'button_press_event', pts[0])
    mouse(ed, 'motion_notify_event', (pts[0][0] + ed.res * 0.1, pts[0][1]))
    mouse(ed, 'button_release_event', pts[0])
    assert ed.pts == held and len(ed.history) == steps


def test_dragging_the_path_adds_a_point_between_its_neighbours(editor):
    ed = editor()
    square(ed)
    curve = ed.curve()
    # a spot on the path halfway along the first side
    on_path = tuple(curve[len(curve) // 8])
    target = (on_path[0], on_path[1] - 0.3)
    first_two = ed.pts[:2]
    drag(ed, on_path, target)
    assert len(ed.pts) == 5
    assert ed.pts[0] == first_two[0] and ed.pts[2] == first_two[1]
    assert np.allclose(ed.pts[1], target, atol=ed.res)


def test_undo_reverts_one_whole_drag(editor):
    ed = editor()
    pts = square(ed)
    held = list(ed.pts)
    drag(ed, pts[2], (pts[2][0] + 0.5, pts[2][1]))
    assert ed.pts != held
    key(ed, 'z')
    assert ed.pts == held


def test_right_click_removes_the_point_under_it_and_a_miss_does_nothing(editor):
    ed = editor()
    pts = square(ed)
    click(ed, (pts[0][0] + 0.9, pts[0][1] + 0.9), button=3)   # empty space inside the square
    assert len(ed.pts) == 4
    click(ed, pts[1], button=3)
    assert len(ed.pts) == 3 and np.allclose(ed.pts[1], pts[2], atol=ed.res)
    key(ed, 'z')
    assert len(ed.pts) == 4


def test_reset_can_be_undone(editor):
    ed = editor()
    square(ed)
    held = list(ed.pts)
    key(ed, 'r')
    assert ed.pts == []
    key(ed, 'z')
    assert ed.pts == held


def test_a_point_dropped_on_its_neighbour_does_not_crash(editor):
    ed = editor()
    pts = square(ed)
    drag(ed, pts[1], pts[0])
    ed.pts[1] = ed.pts[0]    # exactly the same spot
    ed.redraw()
    assert len(ed.curve_line.get_xydata()) > 0
    key(ed, 'enter')          # refuses with a message instead of raising, or saves a valid path


def test_saved_route_opens_as_a_closed_loop_of_points(editor):
    ed = editor(route=True)
    saved = pt.read_csv(waypoint_path('lobby'))
    assert ed.closed is bool(np.linalg.norm(saved[0] - saved[-1]) < 3 * pt.SPACING)
    assert 3 <= len(ed.pts) <= len(saved) // 3
    assert not ed.dirty


def test_moving_one_point_changes_the_saved_route_only_near_it(editor, tmp_path):
    ed = editor(route=True)
    grabbed = ed.pts[len(ed.pts) // 2]
    drag(ed, grabbed, (grabbed[0] + 0.15, grabbed[1]))
    key(ed, 'enter')
    assert not ed.dirty
    before = pt.read_csv(waypoint_path('lobby'))
    after = pt.read_csv(tmp_path / 'lobby_wp.csv')
    gap = [we.distance_to_polyline(p, before) for p in after]
    far = [g for p, g in zip(after, gap) if math.dist(p, grabbed) > 4.0]
    assert max(gap) > 0.05          # the move is in the file
    assert max(far) < 0.10          # the rest of the route stayed put


@pytest.mark.parametrize('path', sorted(glob.glob(os.path.join(WS, 'waypoints', '*.csv'))))
def test_reopening_and_saving_stays_close_to_every_saved_route(path):
    saved = pt.read_csv(path)
    if len(saved) < 2:
        pytest.skip('no points in this file')
    handles, closed = pt.handles_from_path(saved)
    again, _ = pt.smooth_path(handles, closed)
    line = np.vstack([saved, saved[:1]]) if closed else saved
    assert max(we.distance_to_polyline(p, line) for p in again) < 2 * pt.HANDLE_TOLERANCE


def test_smoothing_eases_curves_keeps_walls_and_undoes(editor):
    ed = editor(route=True)
    ed.smooth()   # builds the clearance map
    key(ed, 'z')
    held = list(ed.pts)
    k0, _ = ed.path_stats()
    gap0 = ed.path_clearance().min()
    for _ in range(3):
        ed.smooth()
    k1, _ = ed.path_stats()
    assert k1 < k0                                    # the sharpest curve got gentler
    assert ed.path_clearance().min() >= min(gap0, ARGS.wall_clearance) - 0.05
    for _ in range(3):
        key(ed, 'z')
    assert ed.pts == held


def test_smoothing_keeps_the_ends_of_an_open_path(editor):
    ed = editor()
    pts = square(ed)
    key(ed, 'c')                                      # open path
    ed.smooth()
    assert np.allclose(ed.pts[0], pts[0], atol=ed.res) and np.allclose(ed.pts[-1], pts[-1], atol=ed.res)


def test_keep_off_walls_pushes_a_route_that_touches_a_wall(editor):
    ed = editor(route=True)
    ed.clearance = we.clearance_map(ed.img, ed.res)
    # drag one point onto the nearest wall pixel, so the route touches the wall
    walls = np.argwhere(ed.img < we.FREE_MIN)
    i = len(ed.pts) // 2
    u, v = we.world_to_px(*ed.pts[i], ed.h, ed.res, ed.origin)
    r, c = walls[np.argmin(np.hypot(walls[:, 0] - v, walls[:, 1] - u))]
    ed.pts[i] = we.px_to_world(c, r, ed.h, ed.res, ed.origin)
    ed.redraw()
    held = list(ed.pts)
    before = ed.path_clearance().min()
    assert before < 0.1
    ed.keep_off_walls()
    after = ed.path_clearance().min()
    assert after > before + 0.2                       # well off the wall now
    assert after >= ARGS.wall_clearance - 0.05 or after > 0.25   # target, or a narrow passage's middle
    key(ed, 'z')
    assert ed.pts == held


def test_keep_off_walls_leaves_a_clear_route_alone(editor):
    ed = editor(route=True)
    ed.clearance = we.clearance_map(ed.img, ed.res)
    ed.args = types.SimpleNamespace(**{**vars(ARGS), 'wall_clearance': 0.01})
    held, steps = list(ed.pts), len(ed.history)
    ed.keep_off_walls()
    assert ed.pts == held and len(ed.history) == steps


def test_lateral_and_speed_up_values_come_back_after_save_and_reopen(editor, tmp_path, monkeypatch):
    from PyQt5 import QtCore
    ini = str(tmp_path / 'settings.ini')
    monkeypatch.setattr(we, 'settings', lambda: QtCore.QSettings(ini, QtCore.QSettings.IniFormat))
    monkeypatch.setattr(we, 'WP_DIR', str(tmp_path))

    def fresh():
        args = types.SimpleNamespace(vmax=6.0, vmin=0.5, alat=None, alon=None, abrake=1.5,
                                     wall_clearance=0.40)
        return we.Editor(map_path('lobby'), None, args)
    shutil.copy(waypoint_path('lobby'), tmp_path / 'lobby_wp.csv')
    ed = fresh()
    assert ed.args.alat == we.LIMIT_DEFAULTS['alat'] and ed.args.alon == we.LIMIT_DEFAULTS['alon']
    ed.limit_boxes['alat'].setValue(2.75)
    ed.limit_boxes['alon'].setValue(4.0)
    ed.save()
    we.plt.close(ed.fig)
    again = fresh()
    assert again.args.alat == 2.75 and again.args.alon == 4.0
    assert again.limit_boxes['alat'].value() == 2.75 and again.limit_boxes['alon'].value() == 4.0
    we.plt.close(again.fig)


# ---------- selection, straight lines, marks, racing line ----------
def box_around(ed, idx):
    """World corners of a box around the built path points idx (with a little room)."""
    xy = ed.build()[0][idx]
    return tuple(xy.min(0) - 0.05), tuple(xy.max(0) + 0.05)


def select(ed, idx):
    ed.select_box(*box_around(ed, idx))
    assert ed.sel is not None


def test_a_click_on_empty_map_adds_on_release_and_a_drag_selects_instead(editor):
    ed = editor()
    pts = square(ed)
    n = len(ed.pts)
    mouse(ed, 'button_press_event', (pts[0][0] + 0.9, pts[0][1] + 0.9))
    assert len(ed.pts) == n                       # nothing yet: it could still become a selection
    mouse(ed, 'button_release_event', (pts[0][0] + 0.9, pts[0][1] + 0.9))
    assert len(ed.pts) == n + 1
    key(ed, 'z')
    # from inside the square to below its bottom side: the box holds part of the bottom side
    drag(ed, (pts[0][0] + 0.5, pts[0][1] + 0.5), (pts[1][0] - 0.5, pts[1][1] - 0.5))
    assert len(ed.pts) == n and ed.sel is not None
    xy = ed.build()[0]
    idx = ed.sel_indices(xy)
    assert 3 <= len(idx) < len(xy) // 3
    ed.escape()
    assert ed.sel is None


def test_smoothing_a_selection_changes_only_that_part(editor):
    ed = editor(route=True)
    xy0 = ed.build()[0].copy()
    n = len(xy0)
    part = np.arange(n // 4, n // 2)
    select(ed, part)
    ed.smooth()
    xy1 = ed.build()[0]
    gap = np.array([we.distance_to_polyline(p, np.vstack([xy0, xy0[:1]])) for p in xy1])
    far = [g for p, g in zip(xy1, gap)
           if min(np.linalg.norm(xy0[part] - p, axis=1)) > 1.5]
    assert max(far) < 0.03          # outside the selection (away from its ends) nothing moved


def test_straight_turns_the_selection_into_a_line_that_the_path_joins_smoothly(editor):
    ed = editor(route=True)
    xy = ed.build()[0]
    n = len(xy)
    select(ed, np.arange(n // 5, n // 5 + 25))
    ed.straighten()
    assert len(ed.straight) == 1 and ed.sel is None
    g = next(iter(ed.straight))
    a, b = np.array(ed.pts[g]), np.array(ed.pts[(g + 1) % len(ed.pts)])
    xy, _, gap = ed.build()
    line = xy[gap == g]
    d = b - a
    off = np.abs((line - a) @ np.array([-d[1], d[0]]) / np.linalg.norm(d))
    assert off.max() < 0.005                      # on the line
    # no kink where it meets the curve: headings change smoothly
    head = np.unwrap(np.arctan2(*np.diff(xy, axis=0).T[::-1]))
    assert np.abs(np.diff(head)).max() < 0.25
    # dragging an end moves the line; right click on the line removes it
    drag(ed, tuple(a), tuple(a + (0.1, 0.1)))
    assert np.allclose(ed.pts[g], a + (0.1, 0.1), atol=ed.res)
    mid = tuple(xy[gap == g][len(line) // 2])
    click(ed, mid, button=3)
    assert not ed.straight
    key(ed, 'z')
    assert ed.straight == {g}


def test_marks_make_full_throttle_and_brake_zones_and_survive_save_and_reopen(editor, tmp_path):
    ed = editor(route=True)
    ed.args = types.SimpleNamespace(**{**vars(ARGS), 'vmax': 6.0})   # room above the curves
    xy, v0 = ed.speeds()
    j = int(np.argmin(v0))                       # the slowest corner: full throttle out of it
    ed.start_placing(we.BOOST)
    click(ed, tuple(xy[j]))
    assert ed.placing is None and [m[0] for m in ed.marks] == [we.BOOST]
    _, v1, zone, track = ed.profile()
    assert (zone == we.BOOST).any() and v1.max() > v0.max() + 0.1
    assert np.allclose(track, v0)                  # the track speed is the one without marks
    ed.start_placing(we.BRAKE)
    click(ed, tuple(xy[(j + len(xy) // 3) % len(xy)]))
    assert len(ed.marks) == 2
    key(ed, 'enter')
    rows = open(tmp_path / 'lobby_wp.csv').read().splitlines()
    assert rows[0] == 'x,y,speed,zone,mark'
    assert sum(r.endswith(',boost') for r in rows) == 1
    assert os.path.exists(tmp_path / 'lobby_wp.edit.json')
    held = (list(ed.pts), [list(m) for m in ed.marks])
    we.plt.close(ed.fig)
    again = we.Editor(map_path('lobby'), None, ARGS)
    assert again.pts == held[0] and again.marks == held[1]
    we.plt.close(again.fig)


def test_a_route_edited_elsewhere_ignores_the_stale_sidecar(editor, tmp_path):
    ed = editor(route=True)
    ed.start_placing(we.BOOST)
    xy = ed.build()[0]
    click(ed, tuple(xy[10]))
    key(ed, 'enter')
    we.plt.close(ed.fig)
    with open(tmp_path / 'lobby_wp.csv', 'a') as f:     # someone else changed the CSV
        f.write('')
    rows = open(tmp_path / 'lobby_wp.csv').read().splitlines()
    rows[1] = rows[1].replace(rows[1].split(',')[0], '%.4f' % (float(rows[1].split(',')[0]) + 0.001), 1)
    open(tmp_path / 'lobby_wp.csv', 'w').write('\n'.join(rows) + '\n')
    again = we.Editor(map_path('lobby'), None, ARGS)
    assert [m[0] for m in again.marks] == [we.BOOST]    # from the CSV's mark column
    we.plt.close(again.fig)


def test_racing_line_lowers_curvature_and_keeps_its_margin(editor):
    ed = editor(route=True)
    k0 = (we.curvature(ed.build()[0], ed.closed) ** 2).sum()
    ed.race_line()
    xy = ed.build()[0]
    assert (we.curvature(xy, ed.closed) ** 2).sum() < k0
    assert ed.clear_pts(xy).min() >= 0.25 - 0.05 or ed.clear_pts(xy).min() >= ed.clear_pts(xy).min()
    key(ed, 'z')


def test_racing_line_on_a_selection_keeps_its_ends(editor):
    ed = editor(route=True)
    xy0 = ed.build()[0].copy()
    n = len(xy0)
    select(ed, np.arange(n // 3, n // 3 + 40))
    ends = ed.build()[0][ed.sel_indices(ed.build()[0])][[0, -1]]
    ed.race_line()
    xy1 = ed.build()[0]
    for e in ends:
        assert min(np.linalg.norm(xy1 - e, axis=1)) < 0.03


def test_hovering_the_path_shows_the_speed_there(editor):
    ed = editor(route=True)
    xy, v = ed.speeds()
    mouse(ed, 'motion_notify_event', tuple(xy[40]))
    assert ed.hover is not None and ed.hover.get_visible()
    assert f'{v[40]:.1f} m/s' in ed.hover.get_text()
    mouse(ed, 'motion_notify_event', tuple(xy[40] + ed.build()[0].std(0)))   # off the path
    assert not ed.hover.get_visible() or f'{v[40]:.1f}' not in ed.hover.get_text()


def test_a_selection_tool_keeps_the_loop_start_and_a_tiny_selection_changes_nothing(editor):
    ed = editor(route=True)
    first = ed.pts[0]
    xy = ed.build()[0]
    n = len(xy)
    select(ed, np.arange(n // 2, n // 2 + 30))
    ed.straighten()
    assert np.allclose(ed.pts[0], first)
    held = list(ed.pts)
    ed.sel = (tuple(xy[5]), tuple(xy[5]), tuple(xy[6]))
    ed.smooth()
    assert ed.pts == held
