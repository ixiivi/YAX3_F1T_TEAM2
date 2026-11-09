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
  right click    remove the point under the cursor
  right drag     move the view
  wheel          zoom in / out at the cursor

Keys: a add at the cursor (for remote desktops), z undo, c closed loop / open path,
r clear all (z brings them back), + / - zoom, 0 whole map, enter save, q quit.

CSV format (same as the legacy race_stack): x,y,speed  in the map frame [m, m, m/s].
"""
import argparse
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
from waypoint_nav.path_tools import (curvature, distance_to_polyline, handles_from_path,
                                     read_csv, smooth_path, smoothing_fit, speed_profile,
                                     write_csv)

WP_DIR = os.path.expanduser('~/MAGP/MAGP/waypoints')
LIMIT_DEFAULTS = {'alat': 2.0, 'alon': 3.0}   # used until a route is saved with other values


def settings():
    """Where the editor remembers each route's lateral / speed-up values (~/.config/MAGP)."""
    from PyQt5 import QtCore
    return QtCore.QSettings('MAGP', 'waypoint_editor')


GRAB_POINT = 10     # screen pixels: a press this close grabs a point
GRAB_PATH = 6       # screen pixels: a press this close to the path adds a point there
DRAG_START = 3      # screen pixels: less movement than this is a click, not a drag
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
           'help': '클릭: 점 추가   점 드래그: 이동   경로 드래그: 점 끼우기   '
                   '오른쪽 클릭: 점 삭제   오른쪽 드래그: 이동   휠: 확대/축소',
           'closed': '닫힌 경로', 'open': '열린 경로', 'pts': '점 {n}개',
           'unsaved': '저장 안 된 변경 있음', 'need': '점이 {n}개 이상 필요해요',
           'same_spot': '두 점이 같은 자리에 있어 부드럽게 만들 수 없어요',
           'saved': '저장됨: 점 {n}개, {m:.1f} m, 속도 {v0:.2f}~{v1:.2f} m/s'},
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
           'help': 'click: add   drag point: move   drag path: insert   right click: remove   '
                   'right drag: move view   wheel: zoom',
           'closed': 'closed loop', 'open': 'open path', 'pts': '{n} points',
           'unsaved': 'unsaved changes', 'need': 'need at least {n} points',
           'same_spot': 'cannot smooth: two points are on the same spot',
           'saved': 'saved {n} pts, {m:.1f} m, v {v0:.2f}..{v1:.2f} m/s'},
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
        families = set(QtGui.QFontDatabase().families())
        self.t = TEXT['ko' if families & set(KOREAN_FONTS) else 'en']
        out = os.path.join(WP_DIR, self.out_name)
        if os.path.exists(out):
            handles, self.closed = handles_from_path(read_csv(out))
            self.pts = [tuple(p) for p in handles]
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
                'W': self.keep_off_walls, 'Return': self.save, 'Enter': self.save,
                'Q': self.win.close, 'Escape': self.win.close}
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
        self.pts = []

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

    def speeds(self):
        """(xy, speeds) as save() would write them, or None while there is no smooth path."""
        if len(self.pts) < (3 if self.closed else 2):
            return None
        try:
            xy, _ = smooth_path(self.pts, self.closed)
        except (ValueError, TypeError):
            return None
        if not np.isfinite(xy).all():
            return None
        a = self.args
        return xy, speed_profile(xy, self.closed, a.vmax, a.vmin, a.alat, a.alon,
                                 getattr(a, 'abrake', a.alon))

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
        if len(self.pts) < (3 if self.closed else 2):
            return straight
        try:
            xy, _ = smooth_path(self.pts, self.closed)
        except (ValueError, TypeError):   # e.g. two points on the same spot during a drag
            return straight
        if not np.isfinite(xy).all():
            return straight
        return np.vstack([xy, xy[:1]]) if self.closed else xy

    def to_px(self, xy):
        return np.array([world_to_px(x, y, self.h, self.res, self.origin) for x, y in xy]).reshape(-1, 2)

    def redraw(self, msg=''):
        px = self.to_px(self.pts)
        self.click_line.set_data(px[:, 0], px[:, 1])
        curve = self.to_px(self.curve())
        self.curve_line.set_data(curve[:, 0], curve[:, 1])
        t = self.t
        if self.path_line:
            self.path_line.remove()
            self.path_line = None
        sp = self.speeds() if self.show_speed else None
        if sp is not None:
            xy, v = sp
            px = self.to_px(xy)
            a = self.args
            self.path_line = self.ax.scatter(px[:, 0], px[:, 1], c=v, cmap='turbo', s=8,
                                             vmin=a.vmin, vmax=a.vmax, zorder=3)
            if not msg:
                msg = t['speed_range'].format(v0=v.min(), v1=v.max(), vmax=a.vmax)
        parts = [f'{self.map_name}{" *" if self.dirty else ""}', t['pts'].format(n=len(self.pts)),
                 t['closed'] if self.closed else t['open'],
                 msg or (t['unsaved'] if self.dirty else '')]
        self.status.setText('  ·  '.join(p for p in parts if p))
        a = self.args
        self.status.setToolTip(f'vmax {a.vmax:.1f} m/s, vmin {a.vmin:.1f} m/s, '
                               f'lateral {a.alat:.2f}, speed-up {a.alon:.2f}, '
                               f'slow-down {getattr(a, "abrake", a.alon):.2f} m/s²')
        self.win.setWindowTitle(f'waypoint editor — {self.out_name}{" *" if self.dirty else ""}')
        self.fig.canvas.draw_idle()

    def remember(self):
        """Call before any change to the points: makes it undoable and clears the saved view."""
        self.history.append(list(self.pts))
        self.dirty = True

    def undo(self):
        if self.history:
            self.pts = self.history.pop()
            self.dirty = True
            self.drag = None

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

    def on_path(self, ev):
        if len(self.pts) < 2:
            return False
        screen = self.ax.transData.transform(self.to_px(self.curve()))
        return distance_to_polyline(np.array((ev.x, ev.y)), screen) <= GRAB_PATH

    def add_point(self, ev):
        if ev.inaxes != self.ax or ev.xdata is None:
            return
        self.remember()
        self.pts.append(self.event_world(ev))
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
        hit = self.point_at(ev)
        if hit is not None:
            self.drag, self.drag_from = hit, (ev.x, ev.y)
        elif self.on_path(ev):
            # the new point is held right away: it starts on the path and follows the mouse
            self.remember()
            self.drag = insert_index(self.pts, self.event_world(ev), self.closed)
            self.pts.insert(self.drag, self.event_world(ev))
        else:
            self.add_point(ev)
        self.redraw()

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
        if self.drag is None or ev.inaxes != self.ax or ev.xdata is None:
            return
        if self.drag_from is not None:
            if np.hypot(ev.x - self.drag_from[0], ev.y - self.drag_from[1]) < DRAG_START:
                return
            self.drag_from = None
            self.remember()
        self.pts[self.drag] = self.event_world(ev)
        self.redraw()

    def on_release(self, ev):
        if self.pan is not None:
            moved = self.pan[4]
            self.pan = None
            if not moved and ev.inaxes == self.ax:
                hit = self.point_at(ev)
                if hit is not None:
                    self.remember()
                    del self.pts[hit]
                    self.redraw()
            return
        if self.drag is not None and self.drag_from is None:
            x, y = self.pts[self.drag]
            print(f'point {self.drag + 1}: moved to ({x:.2f}, {y:.2f}) m', flush=True)
        self.drag = self.drag_from = None

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
        elif ev.key in ('q', 'escape'):
            self.win.close()
            return
        self.redraw()

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

    def path_clearance(self):
        """Clearance [m] of every point of the smoothed path."""
        return np.array([self.clearance_at(p) for p in self.curve()])

    def smooth(self):
        """One click: refit the path with a smoothing spline, as much as walls and curvature allow.

        Accepted only if the sharpest curve gets gentler and no part of the path ends up closer to
        a wall than before (when it is inside the clearance target); otherwise the allowed change
        is halved and tried again."""
        t = self.t
        before = self.path_stats()
        if before is None:
            return self.redraw(t['need'].format(n=3 if self.closed else 2))
        if self.clearance is None:
            self.clearance = clearance_map(self.img, self.res)
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

    def keep_off_walls(self):
        t = self.t
        if len(self.pts) < 2:
            return self.redraw(t['need'].format(n=2))
        if self.clearance is None:
            self.clearance = clearance_map(self.img, self.res)
        target = self.args.wall_clearance
        before = self.path_clearance().min()
        if before >= target - 0.01:
            return self.redraw(t['walled_none'].format(d=before))
        self.remember()
        moved, added = set(), 0
        for _ in range(WALL_PASSES):
            for i, p in enumerate(self.pts):
                if self.clearance_at(p) < target - 0.01:
                    u, v = climb(self.clearance, *world_to_px(*p, self.h, self.res, self.origin),
                                 target)
                    q = px_to_world(u, v, self.h, self.res, self.origin)
                    if np.hypot(q[0] - p[0], q[1] - p[1]) > 1e-6:
                        self.pts[i] = q
                        moved.add(i)
            # between the points the curve can still cut a corner: add a point at the worst spot
            curve = self.curve()
            gaps = np.array([self.clearance_at(p) for p in curve])
            worst = int(gaps.argmin())
            if gaps[worst] >= target - 0.02:
                break
            w = curve[worst]
            if min(np.hypot(*(np.array(p) - w)) for p in self.pts) < 0.3:
                break                       # the nearest point is already as far out as it gets
            i = insert_index(self.pts, w, self.closed)
            self.pts.insert(i, tuple(w))
            moved = {j + (j >= i) for j in moved}
            added += 1
        after = self.path_clearance().min()
        self.redraw(t['walled'].format(d0=before, d1=after, m=len(moved), a=added))

    def save(self):
        t = self.t
        need = 3 if self.closed else 2
        if len(self.pts) < need:
            self.redraw(t['need'].format(n=need))
            return
        a = self.args
        try:
            xy, length = smooth_path(self.pts, self.closed)
        except (ValueError, TypeError):
            self.redraw(t['same_spot'])
            return
        if not np.isfinite(xy).all():
            self.redraw(t['same_spot'])
            return
        v = speed_profile(xy, self.closed, a.vmax, a.vmin, a.alat, a.alon,
                          getattr(a, 'abrake', a.alon))
        os.makedirs(WP_DIR, exist_ok=True)
        out = os.path.join(WP_DIR, self.out_name)
        write_csv(out, xy, v)
        store = settings()
        for name in LIMIT_DEFAULTS:
            store.setValue(f'{self.out_name}/{name}', float(getattr(a, name)))
        store.sync()
        self.dirty = False
        msg = t['saved'].format(n=len(xy), m=length, v0=v.min(), v1=v.max())
        # the preview PNG always shows the speeds, and they stay on after saving
        self.show_speed = True
        self.buttons['speed'].setChecked(True)
        self.redraw(msg)
        self.fig.canvas.draw()
        self.fig.savefig(out.replace('.csv', '.png'), dpi=120, bbox_inches='tight')
        print(f'saved {len(xy)} pts, {length:.1f} m -> {out}', flush=True)
        self.redraw(msg)


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
