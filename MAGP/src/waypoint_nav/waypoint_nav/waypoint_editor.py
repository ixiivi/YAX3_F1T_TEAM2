#!/usr/bin/env python3
"""Click waypoints on a saved map (.yaml + .pgm) and save a smoothed path as CSV.

Usage:
  ros2 run waypoint_nav waypoint_editor <map.yaml> [--out name.csv] [--vmax 3.0] [--alat 2.0]

An existing CSV for this map opens ready to edit, with a point about every metre and more
in tight corners.

The icon bar on the left has every action; hover a button for its name and key.
Mouse on the map:
  left click     add a point at the end
  drag a point   move it; the path follows while dragging
  drag the path  add a point there and move it
  drag elsewhere select the part of the path inside the box (Esc clears it); smoothing,
                 keep-off-walls and the racing line then change only that part
  right click    remove the point, mark or straight line under the cursor
  right drag     move the view
  wheel          zoom in / out at the cursor

Full throttle (F), full brake (B) and back-to-track (E) marks sit on the path, each acting up
to the next mark: from full throttle the car speeds up as hard as it can (past the curve
speeds; only pure_pursuit's max_speed caps it), from full brake it brakes as hard as it can
until it is down to the track speed, from back-to-track it slows down normally to the track
speed. Hover over the path to see the speed there. Straight (L) turns the selection into one straight line; the path
leaves and joins it along it. Drag its end points to move it.

Keys: a add at the cursor (for remote desktops), z undo, c closed loop / open path,
r clear all (z brings them back), + / - zoom, 0 whole map, enter save, q quit.

CSV format (same as the legacy race_stack): x,y,speed  in the map frame [m, m, m/s], plus
zone,mark columns when the route has full-throttle / full-brake marks. The editable points,
straight lines and marks are kept beside it in <route>.edit.json.
"""
import argparse
import hashlib
import json
import os
import signal
import sys

import matplotlib
import matplotlib.pyplot as plt  # noqa: F401  (tests close figures through it)
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

from waypoint_nav.map_editor import (BUTTON, KOREAN_FONTS, STYLE, ZOOM_STEP, _icon_lines,
                                     make_icon, zoomed)
from scipy.spatial import cKDTree

from waypoint_nav.path_tools import (BOOST, BOOST_ACCEL, BRAKE, BRAKE_DECEL, RESUME, SPACING,
                                     STOP_DECEL, build_path, curvature, distance_to_polyline,
                                     handles_from_path, left_normals, race_profile, racing_line,
                                     read_csv, read_marks, smooth_path, smoothing_fit,
                                     speed_profile, wall_margins, write_csv)

WP_DIR = os.path.expanduser('~/MAGP/MAGP/waypoints')
LIMIT_DEFAULTS = {'alat': 2.0, 'alon': 3.0}   # used until a route is saved with other values


def settings():
    """Where the editor remembers each route's lateral / speed-up values (~/.config/MAGP)."""
    from PyQt5 import QtCore
    return QtCore.QSettings('MAGP', 'waypoint_editor')


GRAB_POINT = 10     # screen pixels: a press this close grabs a point
GRAB_PATH = 6       # screen pixels: a press this close to the path adds a point there
DRAG_START = 3      # screen pixels: less movement than this is a click, not a drag
SELECT_START = 8    # screen pixels: a press on empty map moved this far selects instead of adding
GRAB_MARK = 30      # screen pixels: a full-throttle / full-brake mark lands on the path this close
BOOST_COLOR, BRAKE_COLOR = '#2e7d32', '#c62828'   # the panel's go / E-STOP colours
RESUME_COLOR = '#757575'
MARKS = (BOOST, BRAKE, RESUME)
SEL_COLOR = '#1565c0'
FREE_MIN = 250      # map pixels at least this bright are free space (254); walls 0, unknown 205
SMOOTH_DEVIATION = 0.05  # [m] how far one smoothing click may move the path, halved until it fits
SMOOTH_TRIES = 5         # halvings before giving up (0.05 m down to about 3 mm)
WALL_PASSES = 12    # wall clearance: rounds of pushing points out and adding points where needed
TEXT = {
    'ko': {'undo': '되돌리기', 'loop': '닫힌 경로 / 열린 경로',
           'loop_tip': '닫힌 경로: 마지막 점이 첫 점으로 이어지는 한 바퀴',
           'clear': '모든 점 지우기', 'clear_tip': '되돌리기로 복구 가능',
           'zoom_in': '확대', 'zoom_out': '축소', 'fit': '전체 보기',
           'smooth': '곡률 줄이기', 'smooth_tip': '누를 때마다 커브를 조금씩 완만하게 (벽 여유는 줄이지 않음). 되돌리기 가능',
           'walls': '벽에서 띄우기', 'walls_tip': '경로가 벽에서 {d:.2f} m 이상 떨어지게 점을 옮기고 필요하면 점 추가. 되돌리기 가능',
           'smoothed': '곡률: 최대 {k0:.2f} → {k1:.2f} 1/m, 커브 최저 속도 {v0:.2f} → {v1:.2f} m/s',
           'smooth_none': '더 완만하게 할 수 없어요 (벽 여유 때문에 막히거나 이미 충분히 완만함)',
           'walled': '벽 여유: 최소 {d0:.2f} → {d1:.2f} m (점 {m}개 이동, {a}개 추가)',
           'walled_none': '이미 벽에서 최소 {d:.2f} m 떨어져 있어요',
           'alat': '횡가속도', 'alat_tip': '커브 속도: 높이면 커브를 빨리 돌아요 (타이어가 미끄러지지 않을 만큼만)',
           'alon': '가속', 'alon_tip': '커브를 빠져나오며 속도를 올리는 정도 (자율주행 accel 이하로)',
           'speed': '곡률별 속도 색', 'speed_tip': '파랑 = 느림, 빨강 = 빠름. 점을 옮기면 바로 다시 계산',
           'speed_range': '속도 {v0:.2f}~{v1:.2f} m/s',
           'save': '저장', 'save_tip': '경로를 부드럽게 만들고 속도를 계산해 CSV와 미리보기 PNG로 저장',
           'quit': '종료', 'quit_tip': '저장 안 한 변경은 사라짐',
           'help': '클릭: 점 추가   점 드래그: 이동   경로 드래그: 점 끼우기   빈 곳 드래그: 구간 선택   '
                   '오른쪽 클릭: 삭제   오른쪽 드래그: 이동   휠: 확대/축소',
           'closed': '닫힌 경로', 'open': '열린 경로', 'pts': '점 {n}개',
           'unsaved': '저장 안 된 변경 있음', 'need': '점이 {n}개 이상 필요해요',
           'same_spot': '두 점이 같은 자리에 있어 부드럽게 만들 수 없어요',
           'saved': '저장됨: 점 {n}개, {m:.1f} m, 속도 {v0:.2f}~{v1:.2f} m/s',
           'boost': '풀악셀 점', 'boost_tip': '누른 뒤 경로를 클릭. 여기부터 다음 마크까지 최대 가속 (곡선 속도 무시)',
           'brake': '풀브레이크 점', 'brake_tip': '누른 뒤 경로를 클릭. 여기부터 트랙 속도가 될 때까지 최대 감속',
           'resume': '트랙 속도 복귀 점', 'resume_tip': '누른 뒤 경로를 클릭. 풀악셀을 끝내고 트랙 속도로 (느리면 평소 감속)',
           'no_end': '풀악셀 뒤에 풀브레이크나 복귀 점이 없어요: 다음 마크까지 계속 가속해요',
           'sel_short': '선택 구간이 너무 짧아요',
           'hover': '{v:.1f} m/s (트랙 {vt:.1f})', 'zone_boost': '풀악셀', 'zone_brake': '풀브레이크',
           'straight': '직선 구간', 'straight_tip': '드래그로 선택한 구간을 직선 하나로. 양 끝 점을 끌어 옮기고, 선을 오른쪽 클릭하면 직선 해제',
           'race': '레이싱 라인', 'race_tip': '아웃-인-아웃: 벽 여유 안에서 곡률이 가장 작은 선으로. 선택 구간이 있으면 그 구간만, 없으면 전체',
           'place': '경로를 클릭해서 {what} 놓기 (Esc 취소)', 'placed': '{what}: 최고 {v:.1f} m/s',
           'sel': '선택: {m:.1f} m (Esc로 해제)', 'need_sel': '먼저 빈 곳을 드래그해서 구간을 선택하세요',
           'straightened': '직선 {m:.1f} m', 'raced': '레이싱 라인: 최대 곡률 {k0:.2f} → {k1:.2f} 1/m, 커브 최저 속도 {v0:.2f} → {v1:.2f} m/s',
           'race_none': '더 펼 수 없어요 (벽 여유 안에서 이미 최소 곡률)',
           'scope_sel': '선택 구간에만 적용', 'scope_all': '경로 전체에 적용'},
    'en': {'undo': 'Undo', 'loop': 'Closed loop / open path',
           'loop_tip': 'closed: the last point joins the first, one lap',
           'clear': 'Clear all points', 'clear_tip': 'undo brings them back',
           'zoom_in': 'Zoom in', 'zoom_out': 'Zoom out', 'fit': 'Whole map',
           'smooth': 'Smooth curves', 'smooth_tip': 'each click eases the curves a little (never closer to walls); undo works',
           'walls': 'Keep off walls',
           'walls_tip': 'move points so the path is at least {d:.2f} m from walls, '
                        'adding points where needed; undo works',
           'smoothed': 'curvature: max {k0:.2f} -> {k1:.2f} 1/m, slowest corner {v0:.2f} -> {v1:.2f} m/s',
           'smooth_none': 'cannot ease further (blocked by wall clearance)',
           'walled': 'wall clearance: min {d0:.2f} -> {d1:.2f} m ({m} moved, {a} added)',
           'walled_none': 'already at least {d:.2f} m from walls',
           'alat': 'lateral', 'alat_tip': 'corner speed: higher turns faster (as long as the tyres grip)',
           'alon': 'speed-up', 'alon_tip': 'how fast speed rises out of corners (at most pure_pursuit accel)',
           'speed': 'Speed colours', 'speed_tip': 'blue = slow, red = fast; follows every edit',
           'speed_range': 'speed {v0:.2f}..{v1:.2f} m/s',
           'save': 'Save', 'save_tip': 'smooth the path, compute speeds, write CSV and preview PNG',
           'quit': 'Quit', 'quit_tip': 'unsaved changes are lost',
           'help': 'click: add   drag point: move   drag path: insert   drag empty map: select   '
                   'right click: remove   right drag: move view   wheel: zoom',
           'closed': 'closed loop', 'open': 'open path', 'pts': '{n} points',
           'unsaved': 'unsaved changes', 'need': 'need at least {n} points',
           'same_spot': 'cannot smooth: two points are on the same spot',
           'saved': 'saved {n} pts, {m:.1f} m, v {v0:.2f}..{v1:.2f} m/s',
           'boost': 'Full throttle mark', 'boost_tip': 'then click the path: flat out from here to the next mark (past curve speeds)',
           'brake': 'Full brake mark', 'brake_tip': 'then click the path: brake as hard as possible until down to the track speed',
           'resume': 'Back-to-track mark', 'resume_tip': 'then click the path: end full throttle, back to the track speed (normal slow-down)',
           'no_end': 'a full throttle mark has no full brake or back-to-track mark after it: it speeds up until the next mark',
           'sel_short': 'the selection is too short',
           'hover': '{v:.1f} m/s (track {vt:.1f})', 'zone_boost': 'full throttle', 'zone_brake': 'full brake',
           'straight': 'Straight', 'straight_tip': 'turn the dragged selection into one straight line; drag its ends to move it, right click the line to undo it',
           'race': 'Racing line', 'race_tip': 'out-in-out: least curvature within the wall margins; the selection only if there is one',
           'place': 'click the path to place the {what} (Esc cancels)', 'placed': '{what}: top {v:.1f} m/s',
           'sel': 'selected {m:.1f} m (Esc clears)', 'need_sel': 'drag on empty map first to select a part of the path',
           'straightened': 'straight {m:.1f} m', 'raced': 'racing line: max curvature {k0:.2f} -> {k1:.2f} 1/m, slowest corner {v0:.2f} -> {v1:.2f} m/s',
           'race_none': 'cannot straighten further (already least curvature within the margins)',
           'scope_sel': 'applies to the selection', 'scope_all': 'applies to the whole path'},
}


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
    img = np.array(Image.open(img_path).convert('L'))
    return img, float(meta['resolution']), meta['origin']


def px_to_world(u, v, h, res, origin):
    # image row 0 is the top; map origin is the bottom-left pixel corner
    return origin[0] + (u + 0.5) * res, origin[1] + (h - v - 0.5) * res


def world_to_px(x, y, h, res, origin):
    return (x - origin[0]) / res - 0.5, h - (y - origin[1]) / res - 0.5


def insert_index(pts, p, closed):
    """Where a new point p fits in the list: the place that adds the least path length."""
    pts = np.asarray(pts, dtype=float)
    a = pts if closed else pts[:-1]
    b = np.roll(pts, -1, axis=0) if closed else pts[1:]
    detour = (np.linalg.norm(p - a, axis=1) + np.linalg.norm(p - b, axis=1)
              - np.linalg.norm(b - a, axis=1))
    return int(detour.argmin()) + 1


def clearance_map(img, res):
    """Distance [m] from every map pixel to the nearest wall or unknown pixel (0 on them)."""
    return ndimage.distance_transform_edt(img >= FREE_MIN) * res


def sample(field, u, v):
    """Bilinear value of a map-sized field at pixel (u = column, v = row)."""
    return float(ndimage.map_coordinates(field, [[v], [u]], order=1, mode='nearest')[0])


def climb(field, u, v, target, step=0.5, max_steps=400):
    """Walk uphill on the clearance field from (u, v) until it reaches target [m] or a ridge."""
    d = sample(field, u, v)
    for _ in range(max_steps):
        if d >= target:
            break
        gu = (sample(field, u + 1, v) - sample(field, u - 1, v)) / 2
        gv = (sample(field, u, v + 1) - sample(field, u, v - 1)) / 2
        g = np.hypot(gu, gv)
        if g < 1e-6:
            break
        nu, nv = u + step * gu / g, v + step * gv / g
        nd = sample(field, nu, nv)
        if nd <= d:             # the middle of a narrow passage: as far from walls as it gets
            break
        u, v, d = nu, nv, nd
    return u, v


def _icon_lines_wp(name, p, QtCore, QtGui):
    """The map editor's icons plus the ones only this editor has, in the same outline style."""
    P, R = QtCore.QPointF, QtCore.QRectF
    if name == 'loop':          # a loop with an arrow head
        p.drawArc(R(4, 4, 16, 16), 100 * 16, 300 * 16)
        p.drawLine(P(12, 4), P(15.5, 1.5))
        p.drawLine(P(12, 4), P(15.5, 7))
    elif name == 'clear':       # bin
        p.drawLine(P(4, 6), P(20, 6))
        p.drawLine(P(9.5, 6), P(9.5, 3.5))
        p.drawLine(P(9.5, 3.5), P(14.5, 3.5))
        p.drawLine(P(14.5, 3.5), P(14.5, 6))
        p.drawPolygon(QtGui.QPolygonF([P(6, 6), P(7.5, 21), P(16.5, 21), P(18, 6)]))
        for x in (10, 14):
            p.drawLine(P(x, 10), P(x, 17))
    elif name == 'smooth':      # a sharp corner beside the eased curve
        p.drawLine(P(3, 20), P(9, 6))
        p.drawLine(P(9, 6), P(13, 20))
        path = QtGui.QPainterPath(P(11, 20))
        path.cubicTo(P(14, 4), P(18, 4), P(21, 20))
        p.drawPath(path)
    elif name == 'walls':       # two walls, the path in the middle, arrows pushing it off them
        p.drawLine(P(3, 3), P(3, 21))
        p.drawLine(P(21, 3), P(21, 21))
        p.drawLine(P(12, 4), P(12, 20))
        for x0, x1 in ((5, 9), (19, 15)):
            p.drawLine(P(x0, 12), P(x1, 12))
            p.drawLine(P(x1, 12), P(x1 - (x1 - x0) * 0.4, 9.5))
            p.drawLine(P(x1, 12), P(x1 - (x1 - x0) * 0.4, 14.5))
    elif name == 'speed':       # speedometer: arc, ticks and a needle
        p.drawArc(R(3, 5, 18, 18), 0, 180 * 16)
        p.drawLine(P(3, 14), P(21, 14))
        for x0, y0, x1, y1 in ((5.5, 9.5, 7, 10.5), (12, 5, 12, 7), (18.5, 9.5, 17, 10.5)):
            p.drawLine(P(x0, y0), P(x1, y1))
        p.drawLine(P(12, 14), P(17, 8))
    elif name == BOOST:         # flat green double chevron with a black edge
        p.setBrush(QtGui.QColor(BOOST_COLOR))
        for x in (2, 10):
            p.drawPolygon(QtGui.QPolygonF([P(x, 4), P(x + 6, 4), P(x + 12, 12), P(x + 6, 20),
                                           P(x, 20), P(x + 6, 12)]))
    elif name == BRAKE:         # flat red octagon with a black edge and a white stop bar
        p.setBrush(QtGui.QColor(BRAKE_COLOR))
        p.drawPolygon(QtGui.QPolygonF([P(8, 2), P(16, 2), P(22, 8), P(22, 16), P(16, 22),
                                       P(8, 22), P(2, 16), P(2, 8)]))
        p.setBrush(QtGui.QColor('white'))
        p.drawRect(R(6, 10, 12, 4))
    elif name == RESUME:        # flat grey disc with a black edge and a white "=" (track speed)
        p.setBrush(QtGui.QColor(RESUME_COLOR))
        p.drawEllipse(P(12, 12), 10, 10)
        p.setBrush(QtGui.QColor('white'))
        p.drawRect(R(6, 7.5, 12, 3))
        p.drawRect(R(6, 13.5, 12, 3))
    elif name == 'straight':    # curve, straight line between two points, curve
        path = QtGui.QPainterPath(P(2, 20))
        path.cubicTo(P(3, 14), P(5, 12), P(7, 12))
        p.drawPath(path)
        p.drawLine(P(7, 12), P(17, 12))
        path = QtGui.QPainterPath(P(17, 12))
        path.cubicTo(P(19, 12), P(21, 10), P(22, 4))
        p.drawPath(path)
        p.setBrush(QtGui.QColor('black'))
        for x in (7, 17):
            p.drawEllipse(P(x, 12), 1.8, 1.8)
    elif name == 'race':        # a corner's walls and the wide-in-wide line through it
        p.drawLine(P(2, 22), P(2, 2))
        p.drawLine(P(2, 2), P(22, 2))
        p.drawLine(P(10, 22), P(10, 10))
        p.drawLine(P(10, 10), P(22, 10))
        path = QtGui.QPainterPath(P(4, 22))           # wide in, the inner corner, wide out
        path.cubicTo(P(6, 9), P(9, 6), P(22, 4))
        pen = QtGui.QPen(QtGui.QColor(BOOST_COLOR), 2.2)
        p.setPen(pen)
        p.drawPath(path)
    else:
        _icon_lines(name, p, QtCore, QtGui)


class Editor:
    """Map view (matplotlib) beside the map editor's icon bar (Qt)."""

    def __init__(self, yaml_path, out_name, args):
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
        from matplotlib.figure import Figure
        from PyQt5 import QtCore, QtGui, QtWidgets
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
        self.img, self.res, self.origin = load_map(yaml_path)
        self.h = self.img.shape[0]
        self.args = args
        self.map_name = os.path.splitext(os.path.basename(yaml_path))[0]
        self.out_name = out_name or f'{self.map_name}_wp.csv'
        # values not given on the command line: the ones this route was last saved with
        store = settings()
        for name, default in LIMIT_DEFAULTS.items():
            if getattr(args, name, None) is None:
                setattr(args, name, store.value(f'{self.out_name}/{name}', default, type=float))
        self.pts = []           # world coordinates
        self.closed = True
        self.history = []       # point lists to go back to with undo
        self.drag = None        # index of the point held by the mouse
        self.drag_from = None   # screen position of that press, until the drag really starts
        self.pan = None         # (mouse x, y in pixels, xlim, ylim, moved) while right-dragging
        self.dirty = False
        self.show_speed = False  # colour the path by the speed the CSV would store
        self.clearance = None    # [m] per map pixel, built on first use (see keep_off_walls)
        self.straight = set()    # gaps i (point i to point i+1) that are straight lines
        self.marks = []          # [BOOST, BRAKE or RESUME, (x, y) on the path]
        self.sel = None          # selected part of the path: its (start, middle, end) points
        self.press = None        # (screen x, y, world point, moved) while left-dragging empty map
        self.placing = None      # BOOST / BRAKE while waiting for a click on the path
        self.mark_drag = None    # index of the mark held by the mouse
        self._built = (None, None)   # (key, build_path result) for the current points
        self._prof = (None, None)    # (key, profile result) for the current path, marks and limits
        self.hover = None            # speed label beside the mouse on the path
        families = set(QtGui.QFontDatabase().families())
        self.t = TEXT['ko' if families & set(KOREAN_FONTS) else 'en']
        # labels drawn on the map (hover speed) use matplotlib's fonts: one with Hangul first
        from matplotlib import font_manager
        known = {f.name for f in font_manager.fontManager.ttflist}
        self.map_font = [f for f in KOREAN_FONTS + ('Noto Sans CJK KR',) if f in known] + ['DejaVu Sans']
        out = os.path.join(WP_DIR, self.out_name)
        if os.path.exists(out):
            self.load(out)
            print(f'opened {out}: {len(self.pts)} points', flush=True)

        # ---- map ----
        self.fig = Figure(figsize=(10, 9))
        FigureCanvasQTAgg(self.fig)
        self.ax = self.fig.add_axes((0.06, 0.04, 0.92, 0.94))
        self.ax.imshow(self.img, cmap='gray', origin='upper', vmin=0, vmax=255,
                       interpolation='nearest')
        self.curve_line, = self.ax.plot([], [], '-', color='tab:orange', lw=1.5)
        # zorder: the points stay on top of the speed colours, so they can still be grabbed
        self.click_line, = self.ax.plot([], [], 'o', color='tab:blue', ms=7, mec='white', mew=0.8,
                                        zorder=4)
        self.path_line = None
        self.sel_line, = self.ax.plot([], [], '-', color=SEL_COLOR, lw=10, alpha=0.3,
                                      solid_capstyle='round', zorder=2)
        self.box = None           # rectangle while dragging a selection
        self.overlays = []        # zone lines, marks and their labels, redrawn every time
        for ev, cb in (('button_press_event', self.on_click), ('motion_notify_event', self.on_motion),
                       ('button_release_event', self.on_release), ('key_press_event', self.on_key),
                       ('scroll_event', self.on_scroll)):
            self.fig.canvas.mpl_connect(ev, cb)

        # ---- icon bar, top to bottom (same layout code as the map editor) ----
        t = self.t
        bar = QtWidgets.QWidget(objectName='bar')
        lay = QtWidgets.QVBoxLayout(bar)
        lay.setContentsMargins(6, 8, 6, 8)
        lay.setSpacing(4)
        self.buttons = {}

        def button(key, shortcut, slot, tip=''):
            b = QtWidgets.QToolButton()
            b.setIcon(make_icon(key, QtCore, QtGui, draw=_icon_lines_wp))
            b.setIconSize(QtCore.QSize(BUTTON - 14, BUTTON - 14))
            b.setFixedSize(BUTTON, BUTTON)
            b.setToolTip(f'<b>{t[key]}</b> ({shortcut})' + (f'<br>{tip}' if tip else ''))
            b.clicked.connect(slot)
            lay.addWidget(b, 0, QtCore.Qt.AlignHCenter)
            self.buttons[key] = b
            return b

        def line():
            f = QtWidgets.QFrame(objectName='line')
            f.setFrameShape(QtWidgets.QFrame.HLine)
            lay.addWidget(f)

        button('undo', 'Z', self.act(self.undo))
        button('loop', 'C', self.act(self.toggle_closed), t['loop_tip'])
        button('clear', 'R', self.act(self.clear), t['clear_tip'])
        line()
        button('smooth', 'S', self.smooth, t['smooth_tip'])
        button('walls', 'W', self.keep_off_walls, t['walls_tip'].format(d=args.wall_clearance))
        button('race', 'O', self.race_line, t['race_tip'])
        button('straight', 'L', self.straighten, t['straight_tip'])
        line()
        button(BOOST, 'F', lambda: self.start_placing(BOOST), t['boost_tip']).setCheckable(True)
        button(BRAKE, 'B', lambda: self.start_placing(BRAKE), t['brake_tip']).setCheckable(True)
        button(RESUME, 'E', lambda: self.start_placing(RESUME), t['resume_tip']).setCheckable(True)
        line()
        button('zoom_in', '+', lambda: self.zoom(1 / ZOOM_STEP))
        button('zoom_out', '-', lambda: self.zoom(ZOOM_STEP))
        button('fit', '0', self.fit_view)
        button('speed', 'V', self.toggle_speed, t['speed_tip']).setCheckable(True)
        line()
        button('save', 'Enter', self.save, t['save_tip'])
        lay.addStretch(1)
        button('quit', 'Q', lambda: self.win.close(), t['quit_tip'])

        self.win = QtWidgets.QMainWindow()
        self.win.setStyleSheet(STYLE)
        body = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(body)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)
        row.addWidget(bar)
        row.addWidget(self.fig.canvas, 1)
        self.win.setCentralWidget(body)
        self.win.resize(1200, 850)   # wide enough for the status, the two limit boxes and help
        self.status = QtWidgets.QLabel()
        self.win.statusBar().addWidget(self.status, 1)
        # lateral grip and speed-up: change them and watch the speed colours, then save
        self.limit_boxes = {}
        for name, label, lo, hi in (('alat', t['alat'], 1.0, 8.0), ('alon', t['alon'], 0.5, 8.0)):
            self.win.statusBar().addPermanentWidget(QtWidgets.QLabel(label))
            box = QtWidgets.QDoubleSpinBox(minimum=lo, maximum=hi, singleStep=0.25, decimals=2,
                                           value=getattr(args, name))
            box.setFocusPolicy(QtCore.Qt.NoFocus)   # the editor's keys keep working
            box.setToolTip(t[name + '_tip'])
            box.valueChanged.connect(lambda v, n=name: self.set_limit(n, v))
            self.win.statusBar().addPermanentWidget(box)
            self.limit_boxes[name] = box
        self.win.statusBar().addPermanentWidget(QtWidgets.QLabel('m/s²'))
        # when the bar is short the help text gives way, not the status (speed range, saved ...)
        help_label = QtWidgets.QLabel(t['help'])
        help_label.setMinimumWidth(1)
        help_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        self.win.statusBar().addPermanentWidget(help_label, 1)
        # 'a' stays a matplotlib key: it needs the cursor position on the map
        keys = {'Z': self.act(self.undo), 'C': self.act(self.toggle_closed),
                'R': self.act(self.clear), '+': lambda: self.zoom(1 / ZOOM_STEP),
                '=': lambda: self.zoom(1 / ZOOM_STEP), '-': lambda: self.zoom(ZOOM_STEP),
                '0': self.fit_view, 'V': self.toggle_speed, 'S': self.smooth,
                'W': self.keep_off_walls, 'O': self.race_line, 'L': self.straighten,
                'F': lambda: self.start_placing(BOOST), 'B': lambda: self.start_placing(BRAKE),
                'E': lambda: self.start_placing(RESUME),
                'Return': self.save, 'Enter': self.save,
                'Q': self.win.close, 'Escape': self.escape}
        self.shortcuts = []
        for seq, slot in keys.items():
            sc = QtWidgets.QShortcut(QtGui.QKeySequence(seq), self.win)
            sc.activated.connect(slot)
            self.shortcuts.append(sc)
        self.fit_view()
        self.redraw()

    # ---------- actions shared by buttons, Qt shortcuts and matplotlib keys ----------
    def act(self, change):
        def run():
            change()
            self.redraw()
        return run

    def toggle_closed(self):
        self.remember()
        self.closed = not self.closed

    def clear(self):
        self.remember()
        self.pts, self.straight, self.marks, self.sel = [], set(), [], None

    def set_limit(self, name, value):
        setattr(self.args, name, value)
        if not self.show_speed:
            self.toggle_speed()     # the point of changing it is to see the speeds
        else:
            self.redraw()

    def toggle_speed(self):
        self.show_speed = not self.show_speed
        self.buttons['speed'].setChecked(self.show_speed)
        self.redraw()

    def build(self):
        """build_path of the current points (xy, length, gap), or None while there is none."""
        if len(self.pts) < (3 if self.closed else 2):
            return None
        key = (np.asarray(self.pts, dtype=float).tobytes(), self.closed, frozenset(self.straight))
        if self._built[0] != key:
            try:
                built = build_path(self.pts, self.closed, self.straight)
                if not np.isfinite(built[0]).all():
                    built = None
            except (ValueError, TypeError):   # e.g. two points on the same spot during a drag
                built = None
            self._built = (key, built)
        return self._built[1]

    def mark_indices(self, xy):
        """(waypoint index, kind) of every full-throttle / full-brake mark on the path xy."""
        return [(int(np.argmin(np.linalg.norm(xy - p, axis=1))), kind) for kind, p in self.marks]

    def profile(self):
        """(xy, speeds, zones, track speeds) as save() would write them (the track speed is the
        one without marks), or None while there is no smooth path."""
        built = self.build()
        if built is None:
            return None
        xy, a = built[0], self.args
        limits = (a.vmax, a.vmin, a.alat, a.alon, getattr(a, 'abrake', a.alon),
                  getattr(a, 'aboost', BOOST_ACCEL), getattr(a, 'afull', BRAKE_DECEL))
        marks = self.mark_indices(xy)
        key = (self._built[0], tuple(marks), limits)
        if self._prof[0] != key:     # every redraw asks; recompute only on a change
            vmax, vmin, alat, alon, abrake, aboost, afull = limits
            v, zone = race_profile(xy, self.closed, vmax, vmin, alat, alon, abrake, marks,
                                   aboost, afull, STOP_DECEL)
            track = speed_profile(xy, self.closed, vmax, vmin, alat, alon, abrake)
            self._prof = (key, (xy, v, zone, track))
        return self._prof[1]

    def open_boost(self):
        """True when full throttle has nothing after it that ends it (it then speeds up to the
        next mark, or for ever)."""
        kinds = [k for _, k in sorted(self.mark_indices(self.build()[0]))] if self.build() else []
        if BOOST not in kinds:
            return False
        if self.closed:
            return not (set(kinds) & {BRAKE, RESUME})
        return kinds[-1] == BOOST

    def speeds(self):
        """(xy, speeds) as save() would write them, or None while there is no smooth path."""
        prof = self.profile()
        return None if prof is None else prof[:2]

    def zoom(self, factor, center=None):
        xl, yl = self.ax.get_xlim(), self.ax.get_ylim()
        cx, cy = center or (sum(xl) / 2, sum(yl) / 2)
        self.ax.set_xlim(zoomed(xl, cx, factor))
        self.ax.set_ylim(zoomed(yl, cy, factor))
        self.fig.canvas.draw_idle()

    def fit_view(self):
        known = np.argwhere(self.img != 205)
        if len(known):
            (r0, c0), (r1, c1) = known.min(0), known.max(0)
            pad = 20
            self.ax.set_xlim(c0 - pad, c1 + pad)
            self.ax.set_ylim(r1 + pad, r0 - pad)
        self.fig.canvas.draw_idle()

    # ---------- path ----------
    def curve(self):
        """The path as it would be saved, or straight lines while it cannot be smoothed yet."""
        if not self.pts:
            return np.empty((0, 2))
        straight = np.array(self.pts + self.pts[:1] if self.closed else self.pts)
        built = self.build()
        if built is None:
            return straight
        xy = built[0]
        return np.vstack([xy, xy[:1]]) if self.closed else xy

    def to_px(self, xy):
        return np.array([world_to_px(x, y, self.h, self.res, self.origin) for x, y in xy]).reshape(-1, 2)

    def sel_indices(self, xy):
        """Indices of the path xy inside the selection, in driving order, or None."""
        if self.sel is None or len(xy) < 3:
            return None
        ia, im, ib = (int(np.argmin(np.linalg.norm(xy - np.asarray(p), axis=1))) for p in self.sel)
        n = len(xy)
        if not self.closed:
            a, b = sorted((ia, ib))
            return np.arange(a, b + 1)
        fwd = np.arange(ia, ia + (ib - ia) % n + 1) % n
        return fwd if np.isin(im, fwd) else np.arange(ib, ib + (ia - ib) % n + 1) % n

    def select_box(self, w0, w1):
        """Select the longest stretch of the path inside the box with corners w0, w1 (world)."""
        built = self.build()
        if built is None:
            return self.redraw()
        xy = built[0]
        (x0, x1), (y0, y1) = sorted((w0[0], w1[0])), sorted((w0[1], w1[1]))
        inside = (xy[:, 0] >= x0) & (xy[:, 0] <= x1) & (xy[:, 1] >= y0) & (xy[:, 1] <= y1)
        runs = _runs_of(inside, self.closed)
        run = max(runs, key=len) if runs else []
        if len(run) < 3 or len(run) == len(xy):   # nothing, or all of it: the tools take it all
            self.sel = None
            return self.redraw()
        self.sel = (tuple(xy[run[0]]), tuple(xy[run[len(run) // 2]]), tuple(xy[run[-1]]))
        self.redraw()

    def show_box(self, w0, w1):
        from matplotlib.patches import Rectangle
        (u0, v0), (u1, v1) = self.to_px([w0, w1])
        if self.box is None:
            self.box = Rectangle((0, 0), 0, 0, fill=False, ec=SEL_COLOR, ls='--', lw=1.5, zorder=6)
            self.ax.add_patch(self.box)
        self.box.set_bounds(min(u0, u1), min(v0, v1), abs(u1 - u0), abs(v1 - v0))
        self.fig.canvas.draw_idle()

    def draw_overlays(self, prof):
        """Full-throttle zones as a thick rainbow line with a black edge, full-brake zones in red
        and black stripes, the marks on top (a green arrow along the path, a red stop sign)."""
        from matplotlib.collections import LineCollection
        for o in self.overlays:
            o.remove()
        self.overlays = []
        if prof is None:
            return
        xy, v, zone, _ = prof
        px = self.to_px(xy)
        for kind in (BOOST, BRAKE):
            for run in _runs_of(zone == kind, self.closed):
                if len(run) < 2:
                    continue
                seg = px[run]
                if kind == BOOST:
                    edge, = self.ax.plot(seg[:, 0], seg[:, 1], '-', color='black', lw=9,
                                         solid_capstyle='round', zorder=3.2)
                    pieces = np.stack([seg[:-1], seg[1:]], axis=1)
                    colors = matplotlib.colormaps['hsv'](np.linspace(0.0, 0.85, len(pieces)))
                    rainbow = LineCollection(pieces, colors=colors, linewidths=6, zorder=3.3,
                                             capstyle='round')
                    self.ax.add_collection(rainbow)
                    top = run[int(np.argmax(v[run]))]
                    label = self.ax.annotate(f'{v[top]:.1f} m/s', px[top], xytext=(8, 8),
                                             textcoords='offset points', fontsize=9,
                                             fontweight='bold', zorder=7,
                                             bbox={'boxstyle': 'round', 'fc': 'white', 'ec': 'black'})
                    self.overlays += [edge, rainbow, label]
                else:
                    for k in range(0, len(run) - 1, 3):
                        part = px[run[k:k + 4]]
                        line, = self.ax.plot(part[:, 0], part[:, 1], '-', lw=7,
                                             color=BRAKE_COLOR if k // 3 % 2 == 0 else 'black',
                                             solid_capstyle='butt', zorder=3.2)
                        self.overlays.append(line)
        for j, kind in self.mark_indices(xy):
            nxt = xy[(j + 1) % len(xy)] if self.closed or j + 1 < len(xy) else xy[j]
            prv = xy[j - 1] if j > 0 or self.closed else xy[j]
            d = nxt - prv
            angle = np.degrees(np.arctan2(d[1], d[0]))
            marker = {BOOST: (3, 0, angle - 90), BRAKE: '8', RESUME: 'o'}[kind]
            color = {BOOST: BOOST_COLOR, BRAKE: BRAKE_COLOR, RESUME: RESUME_COLOR}[kind]
            dot, = self.ax.plot([px[j, 0]], [px[j, 1]], marker=marker, ms=16 if kind != RESUME else 13,
                                mfc=color, mec='black', mew=1.5, zorder=5)
            self.overlays.append(dot)

    def redraw(self, msg=''):
        px = self.to_px(self.pts)
        self.click_line.set_data(px[:, 0], px[:, 1])
        curve = self.to_px(self.curve())
        self.curve_line.set_data(curve[:, 0], curve[:, 1])
        t = self.t
        if self.path_line:
            self.path_line.remove()
            self.path_line = None
        prof = self.profile() if self.show_speed or self.marks else None
        self.draw_overlays(prof)
        if self.show_speed and prof is not None:
            xy, v, _, _ = prof
            px = self.to_px(xy)
            a = self.args
            self.path_line = self.ax.scatter(px[:, 0], px[:, 1], c=v, cmap='turbo', s=8,
                                             vmin=a.vmin, vmax=max(a.vmax, v.max()), zorder=3)
            if not msg:
                msg = t['speed_range'].format(v0=v.min(), v1=v.max(), vmax=a.vmax)
        built = self.build()
        idx = self.sel_indices(built[0]) if built is not None else None
        if idx is not None:
            sel = self.to_px(built[0][idx])
            self.sel_line.set_data(sel[:, 0], sel[:, 1])
            msg = msg or t['sel'].format(m=len(idx) * SPACING)
        else:
            self.sel_line.set_data([], [])
        if not msg and self.marks and self.open_boost():
            msg = t['no_end']
        parts = [f'{self.map_name}{" *" if self.dirty else ""}', t['pts'].format(n=len(self.pts)),
                 t['closed'] if self.closed else t['open'],
                 msg or (t['unsaved'] if self.dirty else '')]
        self.status.setText('  ·  '.join(p for p in parts if p))
        a = self.args
        self.status.setToolTip(f'vmax {a.vmax:.1f} m/s, vmin {a.vmin:.1f} m/s, '
                               f'lateral {a.alat:.2f}, speed-up {a.alon:.2f}, '
                               f'slow-down {getattr(a, "abrake", a.alon):.2f}, '
                               f'full throttle {getattr(a, "aboost", BOOST_ACCEL):.2f}, '
                               f'full brake {getattr(a, "afull", BRAKE_DECEL):.2f} m/s²')
        self.win.setWindowTitle(f'waypoint editor — {self.out_name}{" *" if self.dirty else ""}')
        self.fig.canvas.draw_idle()

    def snapshot(self):
        return (list(self.pts), set(self.straight), [list(m) for m in self.marks], self.closed)

    def restore(self, state):
        pts, straight, marks, closed = state
        self.pts, self.straight, self.closed = list(pts), set(straight), closed
        self.marks = [list(m) for m in marks]

    def remember(self):
        """Call before any change to the path: makes it undoable and clears the saved view."""
        self.history.append(self.snapshot())
        self.dirty = True

    def undo(self):
        if self.history:
            self.restore(self.history.pop())
            self.dirty = True
            self.drag = self.mark_drag = None

    def insert_point(self, i, p, keep_line=False):
        """Insert point p at index i; a straight line it splits stays straight with keep_line."""
        split = i - 1 in self.straight
        self.straight = _shift_insert(self.straight, i)
        if split and keep_line:
            self.straight |= {i - 1, i}
        self.pts.insert(i, tuple(p))

    def delete_point(self, i):
        self.straight = _shift_delete(self.straight, i, len(self.pts), self.closed)
        del self.pts[i]

    def keep_start(self, p0):
        """Turn a closed loop back so it starts at the point nearest p0, where it started before
        a tool turned it (the CSV keeps its first row)."""
        if self.closed and self.pts and p0 is not None:
            self.rotate(self.handle_near(p0, np.inf))

    def rotate(self, k):
        """Start a closed loop at point k (the path stays the same)."""
        n = len(self.pts)
        self.pts = self.pts[k:] + self.pts[:k]
        self.straight = {(g - k) % n for g in self.straight}

    # ---------- mouse ----------
    def event_world(self, ev):
        return px_to_world(ev.xdata, ev.ydata, self.h, self.res, self.origin)

    def point_at(self, ev):
        """Index of the point under the mouse, or None."""
        if not self.pts:
            return None
        screen = self.ax.transData.transform(self.to_px(self.pts))
        d = np.linalg.norm(screen - (ev.x, ev.y), axis=1)
        return int(d.argmin()) if d.min() <= GRAB_POINT else None

    def mark_at(self, ev):
        """Index of the full-throttle / full-brake mark under the mouse, or None."""
        if not self.marks:
            return None
        screen = self.ax.transData.transform(self.to_px([p for _, p in self.marks]))
        d = np.linalg.norm(screen - (ev.x, ev.y), axis=1)
        return int(d.argmin()) if d.min() <= GRAB_POINT + 4 else None

    def path_index_at(self, ev, reach):
        """(index on the built path, its gap) nearest the mouse within reach pixels, or None."""
        built = self.build()
        if built is None:
            return None
        screen = self.ax.transData.transform(self.to_px(built[0]))
        d = np.linalg.norm(screen - (ev.x, ev.y), axis=1)
        j = int(d.argmin())
        return (j, int(built[2][j])) if d[j] <= reach else None

    def straight_at(self, ev):
        """The straight gap under the mouse, or None."""
        hit = self.path_index_at(ev, GRAB_PATH)
        return hit[1] if hit and hit[1] in self.straight else None

    def on_path(self, ev):
        if len(self.pts) < 2:
            return False
        screen = self.ax.transData.transform(self.to_px(self.curve()))
        return distance_to_polyline(np.array((ev.x, ev.y)), screen) <= GRAB_PATH

    def add_point(self, ev):
        if ev.inaxes != self.ax or ev.xdata is None:
            return
        self.append_point(self.event_world(ev))

    def append_point(self, p):
        self.remember()
        self.pts.append(tuple(p))
        x, y = self.pts[-1]
        print(f'point {len(self.pts)}: ({x:.2f}, {y:.2f}) m', flush=True)

    def on_click(self, ev):
        if ev.inaxes != self.ax:
            return
        if ev.button == 3:
            # removal happens on release, unless the right button dragged the view
            self.pan = (ev.x, ev.y, self.ax.get_xlim(), self.ax.get_ylim(), False)
            return
        if ev.button != 1:
            return
        if self.placing:
            return self.place_mark(ev)
        mark = self.mark_at(ev)
        hit = self.point_at(ev) if mark is None else None
        if mark is not None:
            self.mark_drag, self.drag_from = mark, (ev.x, ev.y)
        elif hit is not None:
            self.drag, self.drag_from = hit, (ev.x, ev.y)
        elif self.on_path(ev) and self.straight_at(ev) is None:
            # the new point is held right away: it starts on the path and follows the mouse
            self.remember()
            self.drag = insert_index(self.pts, self.event_world(ev), self.closed)
            self.insert_point(self.drag, self.event_world(ev))
            self.redraw()
        else:
            # empty map (or a straight line): a point on release, or a selection box if it moves
            self.press = (ev.x, ev.y, self.event_world(ev), False)

    def on_motion(self, ev):
        if self.pan is not None:
            x0, y0, xl, yl, moved = self.pan
            if not moved and np.hypot(ev.x - x0, ev.y - y0) < DRAG_START:
                return
            self.pan = (x0, y0, xl, yl, True)
            bb = self.ax.bbox
            dx = (ev.x - x0) * (xl[1] - xl[0]) / bb.width
            dy = (ev.y - y0) * (yl[1] - yl[0]) / bb.height
            self.ax.set_xlim(xl[0] - dx, xl[1] - dx)
            self.ax.set_ylim(yl[0] - dy, yl[1] - dy)
            self.fig.canvas.draw_idle()
            return
        if self.press is not None:
            x0, y0, w0, moved = self.press
            if not moved and np.hypot(ev.x - x0, ev.y - y0) < SELECT_START:
                return
            self.press = (x0, y0, w0, True)
            if ev.inaxes == self.ax and ev.xdata is not None:
                self.show_box(w0, self.event_world(ev))
            return
        if self.drag is None and self.mark_drag is None:
            return self.show_hover(ev)
        if ev.inaxes != self.ax or ev.xdata is None:
            return
        if self.drag_from is not None:
            if np.hypot(ev.x - self.drag_from[0], ev.y - self.drag_from[1]) < DRAG_START:
                return
            self.drag_from = None
            self.remember()
        if self.mark_drag is not None:   # marks stay on the path: the nearest point of it
            built = self.build()
            if built is not None:
                xy, w = built[0], np.asarray(self.event_world(ev))
                self.marks[self.mark_drag][1] = tuple(xy[np.argmin(np.linalg.norm(xy - w, axis=1))])
        else:
            self.pts[self.drag] = self.event_world(ev)
        self.redraw()

    def show_hover(self, ev):
        """Small label beside the mouse while it is over the path: the planned speed there, the
        track speed (without marks) and the zone, to place marks by."""
        hit = self.path_index_at(ev, GRAB_PATH + 4) if ev.inaxes == self.ax else None
        prof = self.profile() if hit else None
        if prof is None:
            if self.hover is not None and self.hover.get_visible():
                self.hover.set_visible(False)
                self.fig.canvas.draw_idle()
            return
        _, v, zone, track = prof
        j = hit[0]
        text = self.t['hover'].format(v=v[j], vt=track[j])
        if zone[j]:
            text += ' · ' + self.t['zone_' + zone[j]]
        if self.hover is None:
            self.hover = self.ax.annotate('', (0, 0), xytext=(12, 12), textcoords='offset points',
                                          fontsize=8, zorder=8, fontfamily=self.map_font,
                                          bbox={'boxstyle': 'round', 'fc': '#fffde7', 'ec': '#444'})
        self.hover.xy = (ev.xdata, ev.ydata)
        self.hover.set_text(text)
        self.hover.set_visible(True)
        self.fig.canvas.draw_idle()

    def on_release(self, ev):
        if self.pan is not None:
            moved = self.pan[4]
            self.pan = None
            if not moved and ev.inaxes == self.ax:
                self.remove_at(ev)
            return
        if self.press is not None:
            _, _, w0, moved = self.press
            self.press = None
            if not moved:
                self.append_point(w0)
                return self.redraw()
            if self.box is not None:
                self.box.remove()
                self.box = None
            if ev.inaxes == self.ax and ev.xdata is not None:
                return self.select_box(w0, self.event_world(ev))
            return self.redraw()
        if self.drag is not None and self.drag_from is None:
            x, y = self.pts[self.drag]
            print(f'point {self.drag + 1}: moved to ({x:.2f}, {y:.2f}) m', flush=True)
        self.drag = self.drag_from = self.mark_drag = None

    def remove_at(self, ev):
        """Right click: the point under the mouse, else the mark, else the straight line."""
        hit, mark = self.point_at(ev), self.mark_at(ev)
        line = self.straight_at(ev) if hit is None and mark is None else None
        if hit is None and mark is None and line is None:
            return
        self.remember()
        if hit is not None:
            self.delete_point(hit)
        elif mark is not None:
            del self.marks[mark]
        else:
            self.straight.discard(line)
        self.redraw()

    def on_scroll(self, ev):
        if ev.inaxes == self.ax and ev.xdata is not None:
            self.zoom(1 / ZOOM_STEP if ev.button == 'up' else ZOOM_STEP, (ev.xdata, ev.ydata))

    def on_key(self, ev):
        """Keys that reach matplotlib: 'a' (needs the cursor), and the rest when no Qt shortcut
        took them first (tests send key events straight to the canvas)."""
        if ev.key == 'a':
            self.add_point(ev)
        elif ev.key == 'z':
            self.undo()
        elif ev.key == 'c':
            self.toggle_closed()
        elif ev.key == 'r':
            self.clear()
        elif ev.key == 'enter':
            self.save()
            return
        elif ev.key == 'escape':
            return self.escape()
        elif ev.key == 'q':
            self.win.close()
            return
        self.redraw()

    def escape(self):
        """Esc: stop placing a mark, else drop the selection, else close."""
        if self.placing:
            self.set_placing(None)
        elif self.sel is not None:
            self.sel = None
        else:
            return self.win.close()
        self.redraw()

    # ---------- marks ----------
    def set_placing(self, kind):
        self.placing = kind
        for k in MARKS:
            self.buttons[k].setChecked(k == kind)

    def start_placing(self, kind):
        self.set_placing(None if self.placing == kind else kind)
        self.redraw(self.t['place'].format(what=self.t[kind]) if self.placing else '')

    def place_mark(self, ev):
        t, kind = self.t, self.placing
        hit = self.path_index_at(ev, GRAB_MARK)
        if hit is None:
            return self.redraw(t['place'].format(what=t[kind]))
        self.remember()
        xy = self.build()[0]
        self.marks.append([kind, tuple(xy[hit[0]])])
        self.set_placing(None)
        _, v, zone, _ = self.profile()
        j, top = hit[0], 0.0
        for k in range(len(xy)):     # the zone this mark starts
            q = (j + k) % len(xy)
            if k and (not zone[q] or q < j and not self.closed):
                break
            top = max(top, v[q])
        self.redraw(t['no_end'] if self.open_boost() else t['placed'].format(what=t[kind], v=top))

    # ---------- path tools (each click is one undoable step) ----------
    def path_stats(self):
        """(max curvature [1/m], slowest speed [m/s]) of the smoothed path, or None."""
        sp = self.speeds()
        if sp is None:
            return None
        xy, v = sp
        return float(curvature(xy, self.closed).max()), float(v.min())

    def clearance_at(self, p):
        u, v = world_to_px(*p, self.h, self.res, self.origin)
        return sample(self.clearance, u, v)

    def clear_pts(self, pts):
        """Wall distance [m] of Nx2 world points."""
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        u = (pts[:, 0] - self.origin[0]) / self.res - 0.5
        v = self.h - (pts[:, 1] - self.origin[1]) / self.res - 0.5
        return ndimage.map_coordinates(self.clearance, [v, u], order=1, mode='nearest')

    def path_clearance(self):
        """Clearance [m] of every point of the smoothed path."""
        return np.array([self.clearance_at(p) for p in self.curve()])

    def need_clearance(self):
        if self.clearance is None:
            self.clearance = clearance_map(self.img, self.res)

    def handle_near(self, p, tol):
        if not self.pts:
            return None
        d = np.linalg.norm(np.asarray(self.pts, dtype=float) - np.asarray(p), axis=1)
        return int(d.argmin()) if d.min() <= tol else None

    def anchor_selection(self):
        """Make both ends of the selection points of their own: (a, b) with the selection running
        from point a forward to point b. A closed loop is turned so that a = 0. None: no part."""
        built = self.build()
        idx = self.sel_indices(built[0]) if built is not None else None
        if idx is None or len(idx) < 3:
            return None
        ends = [built[0][idx[0]].copy(), built[0][idx[-1]].copy()]
        for p in ends:
            if self.handle_near(p, 0.15) is None:
                built = self.build()
                if built is None:
                    return None
                j = int(np.argmin(np.linalg.norm(built[0] - p, axis=1)))
                self.insert_point(int(built[2][j]) + 1, p, keep_line=True)
        a, b = (self.handle_near(p, 0.15) for p in ends)
        if a is None or b is None or a == b:
            return None
        if self.closed:
            self.rotate(a)
            a, b = 0, (b - a) % len(self.pts)
        elif a > b:
            a, b = b, a
        return a, b

    def scope(self):
        """Gap range [a, b) the tools change: the selection, else the whole path (a closed loop
        turned so the range does not wrap). None: a whole closed loop without straight lines.
        'short': there is a selection, but too short to work on."""
        if self.sel is not None:
            ab = self.anchor_selection()
            return ab if ab is not None else 'short'     # never the whole path instead
        n = len(self.pts)
        if not self.closed:
            return 0, n - 1
        if not self.straight:
            return None
        self.rotate((min(self.straight) + 1) % n)
        return 0, n

    def curve_runs(self, a, b):
        """(s, e): stretches from point s to point e inside [a, b) without a straight line."""
        runs, s, n = [], a, len(self.pts)
        for g in range(a, b):
            if g % n in self.straight:
                if g > s:
                    runs.append((s, g))
                s = g + 1
        if b > s:
            runs.append((s, b))
        return runs

    def sub_path(self, s, e):
        """The built path from point s to point e (an open piece), or None if too short."""
        built = self.build()
        if built is None:
            return None
        xy, _, gap = built
        n = len(self.pts)
        ids = np.where((gap >= s) & (gap < e))[0]
        if len(ids) > 1:
            jump = np.where(np.diff(ids) > 1)[0]
            if len(jump):        # the stretch runs past the end of the array: put it in order
                ids = np.concatenate([ids[jump[0] + 1:], ids[:jump[0] + 1]])
        sub = np.vstack([self.pts[s], xy[ids], self.pts[e % n]])
        keep = np.concatenate([[True], np.linalg.norm(np.diff(sub, axis=0), axis=1) > 0.3 * SPACING])
        keep[-1] = True
        sub = sub[keep]
        if len(sub) > 2 and np.linalg.norm(sub[-1] - sub[-2]) < 0.3 * SPACING:
            sub = np.delete(sub, -2, axis=0)
        return sub if len(sub) >= 9 else None

    def splice(self, s, e, inner):
        """Replace the points between s and e with inner."""
        shift = len(inner) - (e - s - 1)
        self.straight = {g + shift if g >= e else g for g in self.straight if not s <= g < e}
        self.pts = self.pts[:s + 1] + [tuple(p) for p in inner] + self.pts[e:]

    def apply_runs(self, propose, accept):
        """For every stretch of curve in scope(): propose(open piece) -> new piece with the same
        ends, or None; the points then change and accept(old piece, rebuilt piece) checks the
        path as it is really built from them (it can differ from the proposal near the ends);
        a stretch it refuses goes back. One undo step. Returns how many stretches changed,
        -1 when the selection is too short."""
        state, dirty = self.snapshot(), self.dirty
        p0 = self.pts[0] if self.pts else None
        self.history.append(state)
        changed = 0
        sc = self.scope()
        if sc == 'short':
            changed = -1
        elif sc is not None:
            for s, e in reversed(self.curve_runs(*sc)):
                sub = self.sub_path(s, e)
                new = propose(sub) if sub is not None else None
                if new is None:
                    continue
                before = self.snapshot()
                inner = handles_from_path(np.asarray(new), closed=False)[0][1:-1]
                self.splice(s, e, inner)
                rebuilt = self.sub_path(s, s + len(inner) + 1)
                if rebuilt is None or not accept(sub, rebuilt):
                    self.restore(before)
                    continue
                changed += 1
        if changed <= 0:
            self.history.pop()
            self.restore(state)
            self.dirty = dirty
        else:
            self.keep_start(p0)
            self.dirty = True
        return changed

    def smooth(self):
        """One click: refit the path with a smoothing spline, as much as walls and curvature allow.

        Accepted only if the sharpest curve gets gentler and no part of the path ends up closer to
        a wall than before (when it is inside the clearance target); otherwise the allowed change
        is halved and tried again. With a selection or straight lines only the stretches of curve
        in the selection (or between the lines) change, and their ends stay put."""
        t = self.t
        before = self.path_stats()
        if before is None:
            return self.redraw(t['need'].format(n=3 if self.closed else 2))
        self.need_clearance()
        target = self.args.wall_clearance
        if self.sel is None and not self.straight:
            return self.smooth_whole(before)
        ks = []

        def fits(old, new):
            """Gentler and no closer to a wall than before (inside the clearance target)."""
            if len(new) < 5 or not np.isfinite(new).all():
                return None
            k0 = float(curvature(old, False)[1:-1].max())
            k1 = float(curvature(new, False)[1:-1].max())
            gap_old = self.clear_pts(old)[cKDTree(old).query(new)[1]]
            ok = k1 < k0 - 1e-4 and (self.clear_pts(new) >= np.minimum(target, gap_old) - 0.01).all()
            return (k0, k1) if ok else None

        def ease(sub):
            deviation = SMOOTH_DEVIATION
            for _ in range(SMOOTH_TRIES):
                try:
                    new = smoothing_fit(sub, False, deviation, pinned=3)
                except (ValueError, TypeError):
                    new = None
                if new is not None and fits(sub, new):
                    return new
                deviation /= 2
            return None

        def accept(old, rebuilt):
            k = fits(old, rebuilt)
            if k:
                ks.append(k)
            return bool(k)
        done = self.apply_runs(ease, accept)
        if done < 0:
            return self.redraw(t['sel_short'])
        if not done:
            return self.redraw(t['smooth_none'])
        after = self.path_stats() or before
        self.redraw(t['smoothed'].format(k0=max(k for k, _ in ks), k1=max(k for _, k in ks),
                                         v0=before[1], v1=after[1]))

    def smooth_whole(self, before):
        t = self.t
        target = self.args.wall_clearance
        cur = self.curve()
        cur = cur[:-1] if self.closed else cur
        gap_old = np.array([self.clearance_at(p) for p in cur])
        deviation = SMOOTH_DEVIATION
        for _ in range(SMOOTH_TRIES):
            try:
                handles, _ = handles_from_path(smoothing_fit(cur, self.closed, deviation), self.closed)
                xy, _ = smooth_path(handles, self.closed)
            except (ValueError, TypeError):
                xy = None
            if xy is not None and np.isfinite(xy).all():
                k = float(curvature(xy, self.closed).max())
                near = np.array([np.argmin(np.linalg.norm(cur - p, axis=1)) for p in xy])
                gap_new = np.array([self.clearance_at(p) for p in xy])
                if k < before[0] and (gap_new >= np.minimum(target, gap_old[near]) - 0.01).all():
                    self.remember()
                    self.pts = [tuple(p) for p in handles]
                    after = self.path_stats() or before
                    return self.redraw(t['smoothed'].format(k0=before[0], k1=after[0],
                                                            v0=before[1], v1=after[1]))
            deviation /= 2
        self.redraw(t['smooth_none'])

    def margins_fn(self, closed, base, xy0, v0):
        """margins(xy) -> (left, right) wall distances for racing_line, from the speeds v0 the
        current path xy0 is driven at (each new point takes the speed of the nearest old one)."""
        a = self.args

        tree = cKDTree(xy0)

        def margins(xy):
            near = tree.query(xy)[1]
            return wall_margins(xy, closed, v0[near], base, getattr(a, 'ld_min', 0.6),
                                getattr(a, 'ld_gain', 0.4), getattr(a, 'slip', 0.03),
                                getattr(a, 'margin_cap', 0.0))
        return margins

    def race_line(self):
        """Out-in-out: the least-curvature line within the wall margins, for the selection if
        there is one (its ends stay put), else the whole path."""
        t = self.t
        prof = self.profile()
        if prof is None:
            return self.redraw(t['need'].format(n=3 if self.closed else 2))
        self.need_clearance()
        before = self.path_stats()
        xy0, v0, _, _ = prof
        base = getattr(self.args, 'race_clearance', 0.25)
        ks = []
        if self.sel is None and self.closed and not self.straight:
            new = racing_line(xy0, True, self.clear_pts, self.margins_fn(True, base, xy0, v0))
            k0, k1 = curvature(xy0, True), curvature(new, True)
            if not np.isfinite(new).all() or (k1 ** 2).sum() > 0.99 * (k0 ** 2).sum():
                return self.redraw(t['race_none'])
            self.remember()
            self.pts = [tuple(p) for p in handles_from_path(new, True)[0]]
            ks.append((k0.max(), k1.max()))
        else:
            def gentler(old, new):
                if len(new) < 5 or not np.isfinite(new).all():
                    return None
                k0, k1 = curvature(old, False)[1:-1], curvature(new, False)[1:-1]
                return (k0.max(), k1.max()) if (k1 ** 2).sum() <= 0.99 * (k0 ** 2).sum() else None

            def straighten_out(sub):
                new = racing_line(sub, False, self.clear_pts, self.margins_fn(False, base, xy0, v0))
                return new if gentler(sub, new) else None

            def accept(old, rebuilt):
                k = gentler(old, rebuilt)
                if k:
                    ks.append(k)
                return bool(k)
            done = self.apply_runs(straighten_out, accept)
            if done < 0:
                return self.redraw(t['sel_short'])
            if not done:
                return self.redraw(t['race_none'])
        after = self.path_stats() or before
        self.redraw(t['raced'].format(k0=max(k for k, _ in ks), k1=max(k for _, k in ks),
                                      v0=before[1], v1=after[1]))

    def straighten(self):
        """The selection becomes one straight line between its end points."""
        t = self.t
        if self.sel is None:
            return self.redraw(t['need_sel'])
        state, dirty = self.snapshot(), self.dirty
        p0 = self.pts[0] if self.pts else None
        self.history.append(state)
        ab = self.anchor_selection()
        if ab is None or self.closed and len(self.pts) - (ab[1] - ab[0] - 1) < 3:
            self.history.pop()
            self.restore(state)
            self.dirty = dirty
            return self.redraw(t['sel_short'])
        a, b = ab
        length = float(np.linalg.norm(np.subtract(self.pts[a], self.pts[b % len(self.pts)])))
        self.splice(a, b, [])
        self.straight.add(a)
        self.keep_start(p0)
        self.sel = None
        self.dirty = True
        self.redraw(t['straightened'].format(m=length))

    def wall_targets(self):
        """Wall distance each point should keep: wall_clearance, plus with margin_cap the extra
        a fast or tight stretch needs on the side of the nearest wall (see wall_margins)."""
        a, base = self.args, self.args.wall_clearance
        cap = getattr(a, 'margin_cap', 0.0)
        prof = self.profile() if cap > 0 else None
        if prof is None:
            return [base] * len(self.pts)
        xy, v, _, _ = prof
        left, right = wall_margins(xy, self.closed, v, base, getattr(a, 'ld_min', 0.6),
                                   getattr(a, 'ld_gain', 0.4), getattr(a, 'slip', 0.03), cap)
        nrm = left_normals(xy, self.closed)
        out = []
        for p in self.pts:
            j = int(np.argmin(np.linalg.norm(xy - np.asarray(p), axis=1)))
            u, w = world_to_px(*p, self.h, self.res, self.origin)
            # clearance grows away from the wall; image rows grow downwards
            gx = sample(self.clearance, u + 1, w) - sample(self.clearance, u - 1, w)
            gy = sample(self.clearance, u, w - 1) - sample(self.clearance, u, w + 1)
            wall_left = -(gx * nrm[j, 0] + gy * nrm[j, 1]) > 0
            out.append(float(left[j] if wall_left else right[j]))
        return out

    def keep_off_walls(self):
        t = self.t
        if len(self.pts) < 2:
            return self.redraw(t['need'].format(n=2))
        self.need_clearance()
        base = self.args.wall_clearance
        targets = self.wall_targets()
        built = self.build()
        idx = self.sel_indices(built[0]) if built is not None else None
        if idx is not None:          # only the points on the selection
            on = set(idx.tolist())
            near = [int(np.argmin(np.linalg.norm(built[0] - np.asarray(p), axis=1))) for p in self.pts]
            inside = {i for i, j in enumerate(near) if j in on}
        else:
            inside = set(range(len(self.pts)))

        def checked():
            b = self.build()
            if b is None:
                return self.curve()
            i = self.sel_indices(b[0])
            return b[0] if i is None else b[0][i]
        before = float(self.clear_pts(checked()).min())
        low = [i for i in inside if self.clearance_at(self.pts[i]) < targets[i] - 0.01]
        if before >= base - 0.01 and not low:
            return self.redraw(t['walled_none'].format(d=before))
        self.remember()
        moved, added = set(), 0
        for _ in range(WALL_PASSES):
            for i in sorted(inside):
                p = self.pts[i]
                if self.clearance_at(p) < targets[i] - 0.01:
                    u, v = climb(self.clearance, *world_to_px(*p, self.h, self.res, self.origin),
                                 targets[i])
                    q = px_to_world(u, v, self.h, self.res, self.origin)
                    if np.hypot(q[0] - p[0], q[1] - p[1]) > 1e-6:
                        self.pts[i] = q
                        moved.add(i)
            # between the points the curve can still cut a corner: add a point at the worst spot
            curve = checked()
            gaps = self.clear_pts(curve)
            worst = int(gaps.argmin())
            if gaps[worst] >= base - 0.02:
                break
            w = curve[worst]
            if min(np.hypot(*(np.array(p) - w)) for p in self.pts) < 0.3:
                break                       # the nearest point is already as far out as it gets
            i = insert_index(self.pts, w, self.closed)
            self.insert_point(i, w)
            moved = {j + (j >= i) for j in moved}
            inside = {j + (j >= i) for j in inside} | {i}
            targets.insert(i, base)
            added += 1
        after = float(self.clear_pts(checked()).min())
        self.redraw(t['walled'].format(d0=before, d1=after, m=len(moved), a=added))

    # ---------- files ----------
    def load(self, out):
        """Open a saved route: the points, straight lines and marks from <route>.edit.json when it
        belongs to this CSV, else points rebuilt from the CSV and its marks."""
        xy = read_csv(out)
        try:
            with open(_edit_path(out)) as f:
                kept = json.load(f)
            if kept.get('csv_sha1') == _sha1(out):
                self.pts = [tuple(map(float, p)) for p in kept['points']]
                self.closed = bool(kept['closed'])
                self.straight = {int(g) for g in kept['straight']}
                self.marks = [[k, tuple(map(float, p))] for k, p in kept['marks'] if k in MARKS]
                return
        except (OSError, ValueError, KeyError, TypeError):
            pass
        if len(xy) >= 2:
            handles, self.closed = handles_from_path(xy)
            self.pts = [tuple(p) for p in handles]
        self.marks = [[k, tuple(xy[i])] for i, k in read_marks(out) if k in MARKS and i < len(xy)]

    def save(self):
        t = self.t
        need = 3 if self.closed else 2
        if len(self.pts) < need:
            self.redraw(t['need'].format(n=need))
            return
        a = self.args
        built = self.build()
        if built is None:
            self.redraw(t['same_spot'])
            return
        xy, length, _ = built
        _, v, zone, _ = self.profile()
        mark = [''] * len(xy)
        for j, kind in self.mark_indices(xy):
            mark[j] = kind
        os.makedirs(WP_DIR, exist_ok=True)
        out = os.path.join(WP_DIR, self.out_name)
        write_csv(out, xy, v, zone, mark)
        n = len(self.pts)
        with open(_edit_path(out), 'w') as f:
            json.dump({'csv_sha1': _sha1(out), 'closed': self.closed,
                       'points': [list(map(float, p)) for p in self.pts],
                       'straight': sorted(g for g in self.straight if g < (n if self.closed else n - 1)),
                       'marks': [[k, list(map(float, p))] for k, p in self.marks]}, f, indent=1)
        store = settings()
        for name in LIMIT_DEFAULTS:
            store.setValue(f'{self.out_name}/{name}', float(getattr(a, name)))
        store.sync()
        self.dirty = False
        msg = t['saved'].format(n=len(xy), m=length, v0=v.min(), v1=v.max())
        if self.open_boost():
            msg += '  ·  ' + t['no_end']
        # the preview PNG always shows the speeds, and they stay on after saving
        self.show_speed = True
        self.buttons['speed'].setChecked(True)
        self.redraw(msg)
        self.fig.canvas.draw()
        self.fig.savefig(out.replace('.csv', '.png'), dpi=120, bbox_inches='tight')
        print(f'saved {len(xy)} pts, {length:.1f} m -> {out}', flush=True)
        self.redraw(msg)


def _edit_path(csv_path):
    """<route>.edit.json beside <route>.csv: what the editor needs that the CSV does not keep."""
    return (csv_path[:-4] if csv_path.endswith('.csv') else csv_path) + '.edit.json'


def _sha1(path):
    with open(path, 'rb') as f:
        return hashlib.sha1(f.read()).hexdigest()


def _runs_of(mask, closed):
    """Index arrays of the stretches where mask is True, in order (a closed path wraps)."""
    n = len(mask)
    if not mask.any():
        return []
    if mask.all():
        return [np.arange(n)]
    start = int(np.argmin(mask)) if closed else 0    # a False index: no stretch wraps past it
    runs, cur = [], []
    for i in (np.arange(n) + start) % n:
        if mask[i]:
            cur.append(i)
        elif cur:
            runs.append(np.array(cur))
            cur = []
    if cur:
        runs.append(np.array(cur))
    return runs


def _shift_insert(straight, i):
    """Straight gaps after a point is inserted at index i (the gap it splits is no longer one)."""
    return {g if g < i - 1 else g + 1 for g in straight if g != i - 1}


def _shift_delete(straight, i, n, closed):
    """Straight gaps after point i of n is removed (the two gaps beside it merge into a curve)."""
    gone = {i, (i - 1) % n if closed else i - 1}
    return {g - (g > i) for g in straight if g not in gone}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('map_yaml')
    p.add_argument('--out', default=None, help='CSV file name in ~/MAGP/MAGP/waypoints')
    p.add_argument('--vmax', type=float, default=6.0,
                   help='speed cap stored in the CSV [m/s]; drive speed is set by pure_pursuit max_speed')
    p.add_argument('--vmin', type=float, default=0.5,
                   help='min speed [m/s]; below ~0.5 the sensorless motor cogs')
    p.add_argument('--alat', type=float, default=None,
                   help=f'max lateral accel [m/s^2]; default: as last saved for this route, '
                        f'else {LIMIT_DEFAULTS["alat"]}')
    p.add_argument('--alon', type=float, default=None,
                   help='speed-up out of corners [m/s^2], at most pure_pursuit accel; default: as '
                        f'last saved for this route, else {LIMIT_DEFAULTS["alon"]}')
    p.add_argument('--abrake', type=float, default=1.5,
                   help='slow-down before corners [m/s^2]; keep below pure_pursuit stop_decel')
    p.add_argument('--wall_clearance', type=float, default=0.40,
                   help='keep-off-walls tool: path to wall distance [m] (half car width 0.21 + margin)')
    p.add_argument('--aboost', type=float, default=BOOST_ACCEL,
                   help='speed-up from a full-throttle mark [m/s^2]; keep it equal to pure_pursuit '
                        'boost_accel (both default to path_tools.BOOST_ACCEL)')
    p.add_argument('--afull', type=float, default=BRAKE_DECEL,
                   help='braking from a full-brake mark [m/s^2]; keep it equal to pure_pursuit '
                        'brake_decel (both default to path_tools.BRAKE_DECEL)')
    p.add_argument('--race_clearance', type=float, default=0.25,
                   help='racing line: path to wall distance [m] (half car width 0.135 + margin)')
    p.add_argument('--margin_cap', type=float, default=0.2,
                   help='extra wall distance [m] at most for speed and curvature '
                        '(corner cutting inside, sliding outside); 0 = off')
    p.add_argument('--slip', type=float, default=0.03,
                   help='outside extra per lateral acceleration [m per m/s^2]')
    p.add_argument('--ld_min', type=float, default=0.6, help='pure_pursuit lookahead_min [m]')
    p.add_argument('--ld_gain', type=float, default=0.4, help='pure_pursuit lookahead_gain [s]')
    args = p.parse_args(sys.argv[1:] if '--ros-args' not in sys.argv
                        else sys.argv[1:sys.argv.index('--ros-args')])
    editor = Editor(args.map_yaml, args.out, args)
    editor.win.show()
    # the Qt event loop never returns to Python, so let Ctrl+C kill the process directly
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(editor.app.exec_())


if __name__ == '__main__':
    matplotlib.use('Qt5Agg')
    main()
