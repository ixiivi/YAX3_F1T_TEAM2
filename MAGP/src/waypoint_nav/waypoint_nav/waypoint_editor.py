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

from waypoint_nav.map_editor import (BUTTON, KOREAN_FONTS, STYLE, ZOOM_STEP, _icon_lines,
                                     make_icon, zoomed)
from waypoint_nav.path_tools import (distance_to_polyline, handles_from_path,
                                     read_csv, smooth_path, speed_profile, write_csv)

WP_DIR = os.path.expanduser('~/MAGP/MAGP/waypoints')
GRAB_POINT = 10     # screen pixels: a press this close grabs a point
GRAB_PATH = 6       # screen pixels: a press this close to the path adds a point there
DRAG_START = 3      # screen pixels: less movement than this is a click, not a drag
TEXT = {
    'ko': {'undo': '되돌리기', 'loop': '닫힌 경로 / 열린 경로',
           'loop_tip': '닫힌 경로: 마지막 점이 첫 점으로 이어지는 한 바퀴',
           'clear': '모든 점 지우기', 'clear_tip': '되돌리기로 복구 가능',
           'zoom_in': '확대', 'zoom_out': '축소', 'fit': '전체 보기',
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
        self.pts = []           # world coordinates
        self.closed = True
        self.history = []       # point lists to go back to with undo
        self.drag = None        # index of the point held by the mouse
        self.drag_from = None   # screen position of that press, until the drag really starts
        self.pan = None         # (mouse x, y in pixels, xlim, ylim, moved) while right-dragging
        self.dirty = False
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
        self.click_line, = self.ax.plot([], [], 'o', color='tab:blue', ms=7, mec='white', mew=0.8)
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
        button('zoom_in', '+', lambda: self.zoom(1 / ZOOM_STEP))
        button('zoom_out', '-', lambda: self.zoom(ZOOM_STEP))
        button('fit', '0', self.fit_view)
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
        self.win.resize(1100, 850)
        self.status = QtWidgets.QLabel()
        self.win.statusBar().addWidget(self.status, 1)
        self.win.statusBar().addPermanentWidget(QtWidgets.QLabel(t['help']))
        # 'a' stays a matplotlib key: it needs the cursor position on the map
        keys = {'Z': self.act(self.undo), 'C': self.act(self.toggle_closed),
                'R': self.act(self.clear), '+': lambda: self.zoom(1 / ZOOM_STEP),
                '=': lambda: self.zoom(1 / ZOOM_STEP), '-': lambda: self.zoom(ZOOM_STEP),
                '0': self.fit_view, 'Return': self.save, 'Enter': self.save,
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
        parts = [f'{self.map_name}{" *" if self.dirty else ""}', t['pts'].format(n=len(self.pts)),
                 t['closed'] if self.closed else t['open'],
                 msg or (t['unsaved'] if self.dirty else '')]
        self.status.setText('   |   '.join(p for p in parts if p))
        self.win.setWindowTitle(f'waypoint editor — {self.out_name}{" *" if self.dirty else ""}')
        self.fig.canvas.draw_idle()

    def remember(self):
        """Call before any change to the points: makes it undoable and clears the saved view."""
        self.history.append(list(self.pts))
        self.dirty = True
        if self.path_line:
            self.path_line.remove()   # the speed colours belong to the path as it was saved
            self.path_line = None

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
        v = speed_profile(xy, self.closed, a.vmax, a.vmin, a.alat, a.alon)
        os.makedirs(WP_DIR, exist_ok=True)
        out = os.path.join(WP_DIR, self.out_name)
        write_csv(out, xy, v)
        if self.path_line:
            self.path_line.remove()
        px = np.array([world_to_px(x, y, self.h, self.res, self.origin) for x, y in xy])
        self.path_line = self.ax.scatter(px[:, 0], px[:, 1], c=v, cmap='turbo', s=6,
                                         vmin=a.vmin, vmax=a.vmax)
        self.fig.savefig(out.replace('.csv', '.png'), dpi=120, bbox_inches='tight')
        self.dirty = False
        msg = t['saved'].format(n=len(xy), m=length, v0=v.min(), v1=v.max())
        print(f'saved {len(xy)} pts, {length:.1f} m -> {out}', flush=True)
        self.redraw(msg)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('map_yaml')
    p.add_argument('--out', default=None, help='CSV file name in ~/MAGP/MAGP/waypoints')
    p.add_argument('--vmax', type=float, default=3.0,
                   help='speed cap stored in the CSV [m/s]; drive speed is set by pure_pursuit max_speed')
    p.add_argument('--vmin', type=float, default=0.5,
                   help='min speed [m/s]; below ~0.5 the sensorless motor cogs')
    p.add_argument('--alat', type=float, default=2.0, help='max lateral accel [m/s^2]')
    p.add_argument('--alon', type=float, default=1.5, help='max accel/brake [m/s^2]')
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
