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

ARGS = types.SimpleNamespace(vmax=3.0, vmin=0.5, alat=2.0, alon=1.5)


@pytest.fixture
def editor(tmp_path, monkeypatch):
    """Factory: an Editor on the lobby map. route=True starts from a copy of its saved route."""
    monkeypatch.setattr(we, 'WP_DIR', str(tmp_path))
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
