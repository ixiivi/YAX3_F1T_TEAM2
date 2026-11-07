#!/usr/bin/env python3
"""Clean up a saved map (.yaml + .pgm): erase walls that were recorded by mistake (people,
chairs, glass reflections), draw missing walls, mark areas unknown.

Usage:
  ros2 run waypoint_nav map_editor <map.yaml> [--waypoints name_wp.csv]
  (laptop without ROS:  python3 -m waypoint_nav.map_editor <map.yaml> --waypoints <csv path>)

The icon bar on the left has every action; hover a button for its name and key.
Mouse on the map:
  left drag      paint with the current tool (brush, or rectangle)
  right drag     move the view
  wheel          zoom in / out at the cursor

Keys: e / w / u tool, b brush/rectangle, [ ] brush size, x specks, + / - zoom, 0 whole map,
z undo, enter save, q quit.

A speck is a wall blob of at most "speck size" pixels that touches no other wall: one pixel
is resolution x resolution (5 cm x 5 cm on our maps), so 4 px is about 10 x 10 cm.
Only the .pgm changes: AMCL, pure_pursuit and the waypoint editor read it, a slam_toolbox
pose graph of the same map does not.
"""
import argparse
import os
import shutil
import signal
import sys

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

# no waypoint_nav.pure_pursuit import: it pulls in rclpy, and this tool also runs on a laptop
WP_DIR = os.path.expanduser('~/MAGP/MAGP/waypoints')
# trinary map values written by map_saver_cli
WALL, UNKNOWN, FREE = 0, 205, 254
TOOL_KEYS = ('e', 'w', 'u')
TOOL_VALUE = {'e': FREE, 'w': WALL, 'u': UNKNOWN}
UNDO_DEPTH = 30
ZOOM_STEP = 1.5
BUTTON = 40          # [px] square tool buttons
KOREAN_FONTS = ('Apple SD Gothic Neo', 'AppleGothic', 'NanumGothic', 'Nanum Gothic',
                'Noto Sans CJK KR', 'Noto Sans KR', 'Malgun Gothic', 'UnDotum')
# tooltip and status texts; English when no Korean font is installed (Hangul would be boxes)
TEXT = {
    'ko': {'e': '지우기', 'e_tip': '빈 공간으로 칠하기', 'w': '벽 그리기', 'w_tip': '벽으로 칠하기',
           'u': '미확인', 'u_tip': '모르는 영역으로 칠하기',
           'brush': '브러시', 'brush_tip': '드래그한 곳을 동그랗게 칠하기',
           'rect': '사각형', 'rect_tip': '드래그한 사각형을 칠하기',
           'size': '브러시 크기', 'size_tip': '1 px = {cm:.0f} cm',
           'speck': '작은 점 기준 크기', 'speck_tip': '다른 벽과 떨어진 이 크기 이하 덩어리를 점으로 봄',
           'specks': '작은 점 지우기',
           'specks_tip': '켜면 작은 점을 지우고 지운 곳을 빨갛게 표시, 다시 누르면 되돌림<br>'
                         '켠 채로 저장하거나 칠하면 확정 (되돌리기로 복구 가능)',
           'zoom_in': '확대', 'zoom_out': '축소', 'fit': '전체 보기',
           'undo': '되돌리기', 'save': '저장', 'save_tip': '처음 저장할 때 원본을 _orig.pgm으로 보관',
           'quit': '종료', 'quit_tip': '저장 안 한 변경은 사라짐',
           'help': '왼쪽 드래그: 칠하기   오른쪽 드래그: 이동   휠: 확대/축소',
           'found': '작은 점 {n}개 지움 (빨간색) — 버튼을 끄면 되돌림', 'none': '해당하는 점 없음',
           'restored': '작은 점 되돌림', 'kept': '작은 점 삭제 확정',
           'undone': '되돌림', 'empty': '되돌릴 것 없음',
           'saved': '저장됨: {name} (원본: {orig})', 'unsaved': '저장 안 된 변경 있음'},
    'en': {'e': 'Erase', 'e_tip': 'paint free space', 'w': 'Wall', 'w_tip': 'paint walls',
           'u': 'Unknown', 'u_tip': 'paint unknown area',
           'brush': 'Brush', 'brush_tip': 'paint a round brush where you drag',
           'rect': 'Rectangle', 'rect_tip': 'paint the dragged rectangle',
           'size': 'Brush size', 'size_tip': '1 px = {cm:.0f} cm',
           'speck': 'Speck size limit', 'speck_tip': 'wall blobs up to this size touching no other wall',
           'specks': 'Remove specks',
           'specks_tip': 'on: small specks are removed and shown in red; off: they come back<br>'
                         'saving or painting while on keeps the removal (undo brings it back)',
           'zoom_in': 'Zoom in', 'zoom_out': 'Zoom out', 'fit': 'Whole map',
           'undo': 'Undo', 'save': 'Save', 'save_tip': 'the first save keeps the original as _orig.pgm',
           'quit': 'Quit', 'quit_tip': 'unsaved changes are lost',
           'help': 'left drag: paint   right drag: move   wheel: zoom',
           'found': '{n} specks removed (red) — switch the button off to bring them back',
           'none': 'no specks', 'restored': 'specks back', 'kept': 'speck removal kept',
           'undone': 'undone', 'empty': 'nothing to undo',
           'saved': 'saved: {name} (original: {orig})', 'unsaved': 'unsaved changes'},
}


def image_path(yaml_path):
    with open(yaml_path) as f:
        img = yaml.safe_load(f)['image']
    return img if os.path.isabs(img) else os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img)


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(image_path(yaml_path)).convert('L'))
    return img, float(meta['resolution']), meta['origin']


def world_to_px(x, y, h, res, origin):
    # same convention as waypoint_editor: image row 0 is the top
    return (x - origin[0]) / res - 0.5, h - (y - origin[1]) / res - 0.5


def paint_disc(img, u, v, radius, value):
    """Set every pixel within radius of column u, row v."""
    h, w = img.shape
    r = int(np.ceil(radius))
    v0, v1, u0, u1 = max(0, v - r), min(h, v + r + 1), max(0, u - r), min(w, u + r + 1)
    if v0 >= v1 or u0 >= u1:
        return
    vv, uu = np.mgrid[v0:v1, u0:u1]
    img[v0:v1, u0:u1][(uu - u) ** 2 + (vv - v) ** 2 <= radius ** 2] = value


def paint_rect(img, u0, v0, u1, v1, value):
    h, w = img.shape
    ua, ub = sorted((int(u0), int(u1)))
    va, vb = sorted((int(v0), int(v1)))
    img[max(0, va):min(h, vb + 1), max(0, ua):min(w, ub + 1)] = value


def wall_specks(img, max_px):
    """Mask of wall blobs of at most max_px pixels (8-connected) that touch no other wall."""
    lab, n = ndimage.label(img < 50, structure=np.ones((3, 3)))
    if n == 0:
        return np.zeros(img.shape, bool)
    size = np.bincount(lab.ravel())
    small = size <= max_px
    small[0] = False
    return small[lab]


def save_map(img, yaml_path):
    """Write the .pgm in place; the first save copies the original to <name>_orig.pgm."""
    path = image_path(yaml_path)
    root, ext = os.path.splitext(path)
    backup = f'{root}_orig{ext}'
    if not os.path.exists(backup):
        shutil.copy2(path, backup)
    Image.fromarray(img.astype(np.uint8)).save(path)   # 2-D uint8 is greyscale ('L')
    return path, backup


def zoomed(lim, center, factor):
    """Axis limits scaled by factor around center (works for inverted image axes too)."""
    return tuple(center + (x - center) * factor for x in lim)


# ---------- icons: black outlines on a 24 x 24 grid, drawn with QPainter ----------
def _icon_lines(name, p, QtCore, QtGui):
    """Draw one icon; p is a QPainter scaled to 24 x 24 with the outline pen set."""
    P, R = QtCore.QPointF, QtCore.QRectF

    def magnifier():
        p.drawEllipse(R(3, 3, 13, 13))
        p.drawLine(P(14, 14), P(21, 21))

    def dots(*xy):
        # filled, so a speck reads as a speck at icon size (a pen point is too faint)
        p.save()
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor('black'))
        for x, y in xy:
            p.drawEllipse(P(x, y), 1.3, 1.3)
        p.restore()

    if name == 'e':            # eraser
        poly = QtGui.QPolygonF([P(9, 20), P(3, 14), P(13, 4), P(21, 12), P(13, 20)])
        p.drawPolygon(poly)
        p.drawLine(P(7, 10), P(15, 18))
        p.drawLine(P(9, 20), P(21, 20))
    elif name == 'w':          # brick wall
        p.drawRect(R(3, 5, 18, 14))
        for y in (9.7, 14.3):
            p.drawLine(P(3, y), P(21, y))
        for x, y0, y1 in ((12, 5, 9.7), (7.5, 9.7, 14.3), (16.5, 9.7, 14.3), (12, 14.3, 19)):
            p.drawLine(P(x, y0), P(x, y1))
    elif name == 'u':          # dashed square with a question mark
        pen = p.pen()
        dashed = QtGui.QPen(pen)
        dashed.setStyle(QtCore.Qt.DashLine)
        p.setPen(dashed)
        p.drawRect(R(3, 3, 18, 18))
        p.setPen(pen)
        path = QtGui.QPainterPath(P(9.5, 9.5))
        path.cubicTo(P(9.5, 6), P(14.5, 6), P(14.5, 9.5))
        path.cubicTo(P(14.5, 12), P(12, 12), P(12, 14.5))
        p.drawPath(path)
        p.drawPoint(P(12, 17.5))
    elif name == 'brush':      # round brush tip
        p.drawEllipse(R(5, 5, 14, 14))
        dots((12, 12))
    elif name == 'rect':
        p.drawRect(R(4, 6, 16, 12))
    elif name == 'specks':     # specks crossed out
        dots((5, 6), (10, 4), (6, 11))
        p.drawLine(P(12, 12), P(20, 20))
        p.drawLine(P(20, 12), P(12, 20))
    elif name == 'zoom_in':
        magnifier()
        p.drawLine(P(6.5, 9.5), P(12.5, 9.5))
        p.drawLine(P(9.5, 6.5), P(9.5, 12.5))
    elif name == 'zoom_out':
        magnifier()
        p.drawLine(P(6.5, 9.5), P(12.5, 9.5))
    elif name == 'fit':        # four corners
        for sx, sy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
            cx, cy = 12 - sx * 8, 12 - sy * 8
            p.drawLine(P(cx, cy), P(cx + sx * 5, cy))
            p.drawLine(P(cx, cy), P(cx, cy + sy * 5))
    elif name == 'undo':       # arrow curving back to the left
        path = QtGui.QPainterPath(P(5, 10))
        path.lineTo(P(15, 10))
        path.cubicTo(P(21, 10), P(21, 19), P(15, 19))
        path.lineTo(P(9, 19))
        p.drawPath(path)
        p.drawLine(P(5, 10), P(9, 6))
        p.drawLine(P(5, 10), P(9, 14))
    elif name == 'save':       # floppy disk
        poly = QtGui.QPolygonF([P(4, 4), P(17, 4), P(20, 7), P(20, 20), P(4, 20)])
        p.drawPolygon(poly)
        p.drawRect(R(8, 4, 7, 5))
        p.drawRect(R(7, 13, 10, 7))
    elif name == 'quit':       # door with an arrow out
        p.drawPolyline(QtGui.QPolygonF([P(13, 7), P(13, 4), P(4, 4), P(4, 20), P(13, 20), P(13, 17)]))
        p.drawLine(P(9, 12), P(21, 12))
        p.drawLine(P(21, 12), P(17.5, 8.5))
        p.drawLine(P(21, 12), P(17.5, 15.5))


def make_icon(name, QtCore, QtGui, size=64):
    pix = QtGui.QPixmap(size, size)
    pix.fill(QtCore.Qt.transparent)
    p = QtGui.QPainter(pix)
    p.setRenderHint(QtGui.QPainter.Antialiasing)
    p.scale(size / 24.0, size / 24.0)
    pen = QtGui.QPen(QtGui.QColor('black'), 1.6)
    pen.setCapStyle(QtCore.Qt.RoundCap)
    pen.setJoinStyle(QtCore.Qt.RoundJoin)
    p.setPen(pen)
    p.setBrush(QtCore.Qt.NoBrush)
    _icon_lines(name, p, QtCore, QtGui)
    p.end()
    return QtGui.QIcon(pix)


STYLE = """
QMainWindow, QWidget#bar { background: #fafafa; }
QToolButton { border: 1px solid transparent; border-radius: 6px; background: transparent; }
QToolButton:hover { background: #e6e6e6; }
QToolButton:pressed, QToolButton:checked { background: #d2d2d2; border-color: #9e9e9e; }
QSpinBox { border: 1px solid #b0b0b0; border-radius: 4px; padding: 1px 2px; background: white;
           color: black; }
QFrame#line { color: #d0d0d0; }
QLabel { color: black; }
QStatusBar { color: #333; }
"""


class Editor:
    """Map view (matplotlib) beside a narrow icon bar (Qt). Needs a running QApplication."""

    def __init__(self, yaml_path, waypoints):
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
        from matplotlib.figure import Figure
        from PyQt5 import QtCore, QtGui, QtWidgets
        self.yaml_path = yaml_path
        self.img, self.res, self.origin = load_map(yaml_path)
        self.h = self.img.shape[0]
        self.name = os.path.splitext(os.path.basename(yaml_path))[0]
        self.undo = []
        self.dirty = False
        self.tool = 'e'
        self.rect_mode = False
        self.radius = 3.0          # [px]
        self.speck_px = 4
        self.specks = None         # pixels the speck toggle removed, shown in red
        self.speck_base = None     # the map before that removal, while the toggle is on
        self.dirty_before = False
        self.drag = None           # rectangle start, or True while brushing
        self.pan = None            # (mouse x, y in pixels, xlim, ylim) while right-dragging
        families = set(QtGui.QFontDatabase().families())
        self.t = TEXT['ko' if families & set(KOREAN_FONTS) else 'en']

        # ---- map ----
        self.fig = Figure(figsize=(10, 9))
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.ax = self.fig.add_axes((0.06, 0.04, 0.92, 0.94))
        self.view = self.ax.imshow(self.img, cmap='gray', origin='upper', vmin=0, vmax=255,
                                   interpolation='nearest')
        self.overlay = self.ax.imshow(np.zeros((*self.img.shape, 4)), origin='upper',
                                      interpolation='nearest')
        self.rect = self.ax.plot([], [], '-', color='tab:orange', lw=1)[0]
        if waypoints:
            # a path that exists as given (absolute or relative) wins over the Jetson folder
            path = os.path.expanduser(waypoints)
            if not os.path.exists(path):
                path = os.path.join(WP_DIR, waypoints)
            wp = np.genfromtxt(path, delimiter=',', names=True)
            u, v = world_to_px(wp['x'], wp['y'], self.h, self.res, self.origin)
            self.ax.plot(u, v, '.', color='tab:blue', ms=2)
        for ev, cb in (('button_press_event', self.on_press), ('motion_notify_event', self.on_move),
                       ('button_release_event', self.on_release), ('scroll_event', self.on_scroll)):
            self.canvas.mpl_connect(ev, cb)

        # ---- icon bar, top to bottom ----
        t = self.t
        bar = QtWidgets.QWidget(objectName='bar')
        lay = QtWidgets.QVBoxLayout(bar)
        lay.setContentsMargins(6, 8, 6, 8)
        lay.setSpacing(4)
        self.buttons = {}

        def button(key, name, shortcut, slot, checkable=False, tip=''):
            b = QtWidgets.QToolButton()
            b.setIcon(make_icon(key, QtCore, QtGui))
            b.setIconSize(QtCore.QSize(BUTTON - 14, BUTTON - 14))
            b.setFixedSize(BUTTON, BUTTON)
            b.setCheckable(checkable)
            b.setToolTip(f'<b>{name}</b> ({shortcut})' + (f'<br>{tip}' if tip else ''))
            if slot is not None:
                b.clicked.connect(slot)
            lay.addWidget(b, 0, QtCore.Qt.AlignHCenter)
            self.buttons[key] = b
            return b

        def spin(lo, hi, value, tip, slot):
            s = QtWidgets.QSpinBox()
            s.setRange(lo, hi)
            s.setValue(value)
            s.setFixedWidth(BUTTON + 4)
            s.setButtonSymbols(QtWidgets.QAbstractSpinBox.PlusMinus)
            s.setToolTip(tip)
            s.valueChanged.connect(slot)
            lay.addWidget(s, 0, QtCore.Qt.AlignHCenter)
            return s

        def line():
            f = QtWidgets.QFrame(objectName='line')
            f.setFrameShape(QtWidgets.QFrame.HLine)
            lay.addWidget(f)

        self.tool_group = QtWidgets.QButtonGroup(bar)
        for k in TOOL_KEYS:
            self.tool_group.addButton(button(k, t[k], k.upper(), lambda _c, k=k: self.set_tool(k),
                                             checkable=True, tip=t[f'{k}_tip']))
        self.buttons['e'].setChecked(True)
        line()
        self.shape_group = QtWidgets.QButtonGroup(bar)
        self.shape_group.addButton(button('brush', t['brush'], 'B', lambda: self.set_rect(False),
                                          checkable=True, tip=t['brush_tip']))
        self.shape_group.addButton(button('rect', t['rect'], 'B', lambda: self.set_rect(True),
                                          checkable=True, tip=t['rect_tip']))
        self.buttons['brush'].setChecked(True)
        self.size_spin = spin(1, 15, int(self.radius),
                              f'<b>{t["size"]}</b> ([ ])<br>{t["size_tip"].format(cm=self.res * 100)}',
                              self.set_radius)
        line()
        # toggled (not clicked): undo switches it off from code and must restore the specks too
        button('specks', t['specks'], 'X', None, checkable=True,
               tip=t['specks_tip']).toggled.connect(self.toggle_specks)
        self.speck_spin = spin(1, 30, self.speck_px, f'<b>{t["speck"]}</b><br>{t["speck_tip"]}',
                               self.set_speck_px)
        line()
        button('zoom_in', t['zoom_in'], '+', lambda: self.zoom(1 / ZOOM_STEP))
        button('zoom_out', t['zoom_out'], '-', lambda: self.zoom(ZOOM_STEP))
        button('fit', t['fit'], '0', self.fit_view)
        line()
        button('undo', t['undo'], 'Z', self.undo_last)
        button('save', t['save'], 'Enter', self.save, tip=t['save_tip'])
        lay.addStretch(1)
        button('quit', t['quit'], 'Q', lambda: self.win.close(), tip=t['quit_tip'])

        self.win = QtWidgets.QMainWindow()
        self.win.setStyleSheet(STYLE)
        body = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(body)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)
        row.addWidget(bar)
        row.addWidget(self.canvas, 1)
        self.win.setCentralWidget(body)
        self.win.resize(1100, 850)
        self.status = QtWidgets.QLabel()
        self.win.statusBar().addWidget(self.status, 1)
        self.win.statusBar().addPermanentWidget(QtWidgets.QLabel(t['help']))

        keys = {'E': lambda: self.buttons['e'].click(), 'W': lambda: self.buttons['w'].click(),
                'U': lambda: self.buttons['u'].click(),
                'B': lambda: self.buttons['brush' if self.rect_mode else 'rect'].click(),
                '[': lambda: self.size_spin.stepBy(-1), ']': lambda: self.size_spin.stepBy(1),
                'X': lambda: self.buttons['specks'].click(),
                '+': lambda: self.zoom(1 / ZOOM_STEP), '=': lambda: self.zoom(1 / ZOOM_STEP),
                '-': lambda: self.zoom(ZOOM_STEP), '0': self.fit_view, 'Z': self.undo_last,
                'Return': self.save, 'Enter': self.save, 'Q': self.win.close,
                'Escape': self.win.close}
        self.shortcuts = []
        for seq, slot in keys.items():
            sc = QtWidgets.QShortcut(QtGui.QKeySequence(seq), self.win)
            sc.activated.connect(slot)
            self.shortcuts.append(sc)
        self.fit_view()
        self.redraw()

    # ---------- state changes (buttons and keys share these) ----------
    def set_tool(self, key):
        self.tool = key
        self.redraw()

    def set_rect(self, on):
        self.rect_mode = on
        self.redraw()

    def set_radius(self, value):
        self.radius = float(value)
        self.redraw()

    def set_speck_px(self, value):
        self.speck_px = int(value)
        if self.speck_base is not None:
            self.apply_specks()     # the removal follows the size while the toggle is on

    def toggle_specks(self, on):
        if on:
            self.speck_base, self.dirty_before = self.img.copy(), self.dirty
            self.apply_specks()
        elif self.speck_base is not None:
            self.img, self.speck_base = self.speck_base, None
            self.dirty = self.dirty_before
            self.clear_specks()
            self.redraw(self.t['restored'])

    def apply_specks(self):
        mask = wall_specks(self.speck_base, self.speck_px)
        self.img = self.speck_base.copy()
        self.img[mask] = FREE
        self.specks = mask
        self.show_specks()
        self.dirty = self.dirty_before or bool(mask.any())
        n = ndimage.label(mask, structure=np.ones((3, 3)))[1]
        self.redraw(self.t['found'].format(n=n) if n else self.t['none'])

    def commit_specks(self):
        """Keep the removal and switch the toggle off; undo still brings the specks back."""
        if self.speck_base is None:
            return False
        self.undo.append(self.speck_base)
        del self.undo[:-UNDO_DEPTH]
        self.speck_base = None
        self.clear_specks()
        b = self.buttons['specks']
        b.blockSignals(True)       # off without toggle_specks putting the specks back
        b.setChecked(False)
        b.blockSignals(False)
        return True

    def undo_last(self):
        if self.speck_base is not None:
            self.buttons['specks'].setChecked(False)
            return
        if not self.undo:
            return self.redraw(self.t['empty'])
        self.img = self.undo.pop()
        self.dirty = bool(self.undo)
        self.clear_specks()
        self.redraw(self.t['undone'])

    def save(self):
        kept = self.commit_specks()
        path, backup = save_map(self.img, self.yaml_path)
        self.dirty = False
        print(f'saved {path} (original: {backup})', flush=True)
        msg = self.t['saved'].format(name=os.path.basename(path), orig=os.path.basename(backup))
        self.redraw(f'{self.t["kept"]}, {msg}' if kept else msg)

    def zoom(self, factor, center=None):
        xl, yl = self.ax.get_xlim(), self.ax.get_ylim()
        cx, cy = center or (sum(xl) / 2, sum(yl) / 2)
        self.ax.set_xlim(zoomed(xl, cx, factor))
        self.ax.set_ylim(zoomed(yl, cy, factor))
        self.canvas.draw_idle()

    def fit_view(self):
        known = np.argwhere(self.img != UNKNOWN)
        if len(known):
            (r0, c0), (r1, c1) = known.min(0), known.max(0)
            self.ax.set_xlim(c0 - 20, c1 + 20)
            self.ax.set_ylim(r1 + 20, r0 - 20)
        self.canvas.draw_idle()

    # ---------- drawing ----------
    def remember(self):
        self.commit_specks()       # painting on the cleaned map keeps the removal
        self.undo.append(self.img.copy())
        del self.undo[:-UNDO_DEPTH]
        self.dirty = True
        self.clear_specks()

    def show_specks(self):
        rgba = np.zeros((*self.img.shape, 4))
        if self.specks is not None:
            rgba[self.specks] = (1.0, 0.0, 0.0, 1.0)
        self.overlay.set_data(rgba)

    def clear_specks(self):
        if self.specks is not None:
            self.specks = None
            self.show_specks()

    def redraw(self, msg=''):
        self.view.set_data(self.img)
        t = self.t
        shape = t['rect'] if self.rect_mode else f'{t["brush"]} {self.radius * self.res * 100:.0f} cm'
        parts = [f'{self.name}{" *" if self.dirty else ""}', f'{t[self.tool]}, {shape}']
        parts.append(msg or (t['unsaved'] if self.dirty else ''))
        self.status.setText('   |   '.join(p for p in parts if p))
        self.win.setWindowTitle(f'map editor — {self.name}{" *" if self.dirty else ""}')
        self.canvas.draw_idle()

    # ---------- mouse ----------
    def _px(self, ev):
        if ev.inaxes != self.ax or ev.xdata is None:
            return None
        return int(round(ev.xdata)), int(round(ev.ydata))

    def on_press(self, ev):
        if ev.inaxes != self.ax:
            return
        if ev.button == 3:
            self.pan = (ev.x, ev.y, self.ax.get_xlim(), self.ax.get_ylim())
            return
        p = self._px(ev)
        if p is None or ev.button != 1:
            return
        self.remember()
        if self.rect_mode:
            self.drag = p
        else:
            self.drag = True
            paint_disc(self.img, *p, self.radius, TOOL_VALUE[self.tool])
            self.redraw()

    def on_move(self, ev):
        if self.pan is not None:
            x0, y0, xl, yl = self.pan
            bb = self.ax.bbox
            dx = (ev.x - x0) * (xl[1] - xl[0]) / bb.width
            dy = (ev.y - y0) * (yl[1] - yl[0]) / bb.height
            self.ax.set_xlim(xl[0] - dx, xl[1] - dx)
            self.ax.set_ylim(yl[0] - dy, yl[1] - dy)
            self.canvas.draw_idle()
            return
        p = self._px(ev)
        if p is None or self.drag is None:
            return
        if self.rect_mode:
            (u0, v0), (u1, v1) = self.drag, p
            self.rect.set_data([u0, u1, u1, u0, u0], [v0, v0, v1, v1, v0])
            self.canvas.draw_idle()
        else:
            paint_disc(self.img, *p, self.radius, TOOL_VALUE[self.tool])
            self.redraw()

    def on_release(self, ev):
        if self.pan is not None:
            self.pan = None
            return
        if self.rect_mode and isinstance(self.drag, tuple):
            p = self._px(ev)
            if p is not None:
                paint_rect(self.img, *self.drag, *p, TOOL_VALUE[self.tool])
            self.rect.set_data([], [])
        if self.drag is not None:
            self.drag = None
            self.redraw()

    def on_scroll(self, ev):
        if ev.inaxes == self.ax and ev.xdata is not None:
            self.zoom(1 / ZOOM_STEP if ev.button == 'up' else ZOOM_STEP, (ev.xdata, ev.ydata))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('map_yaml')
    p.add_argument('--waypoints', default=None,
                   help='CSV in ~/MAGP/MAGP/waypoints (or a path) drawn on top, to see the line')
    args = p.parse_args(sys.argv[1:] if '--ros-args' not in sys.argv
                        else sys.argv[1:sys.argv.index('--ros-args')])
    from PyQt5 import QtWidgets
    app = QtWidgets.QApplication(sys.argv[:1])
    editor = Editor(args.map_yaml, args.waypoints)
    editor.win.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
