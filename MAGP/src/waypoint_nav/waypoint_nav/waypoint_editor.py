#!/usr/bin/env python3
"""Click waypoints on a saved map (.yaml + .pgm) and save a smoothed path as CSV.

Usage:
  ros2 run waypoint_nav waypoint_editor <map.yaml> [--out name.csv] [--vmax 3.0] [--alat 2.0]

An existing CSV for this map opens ready to edit, with a point about every metre and more
in tight corners.

Mouse / keys (in the map window):
  left click, a  add point at the end (a = at the mouse cursor, for remote desktops)
  drag a point   move it; the path follows while dragging
  drag the path  add a point there and move it
  right click    remove the point under the cursor
  z              undo
  c              toggle closed loop (lap) / open path
  enter          smooth, compute speeds, save CSV + preview PNG
  r              clear all points (z brings them back)
  q / esc        quit without saving

CSV format (same as the legacy race_stack): x,y,speed  in the map frame [m, m, m/s].
"""
import argparse
import os
import signal
import sys

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from PIL import Image  # noqa: E402

from waypoint_nav.path_tools import (distance_to_polyline, handles_from_path,  # noqa: E402
                                     read_csv, smooth_path, speed_profile, write_csv)

WP_DIR = os.path.expanduser('~/MAGP/MAGP/waypoints')
GRAB_POINT = 10     # screen pixels: a press this close grabs a point
GRAB_PATH = 6       # screen pixels: a press this close to the path adds a point there
DRAG_START = 3      # screen pixels: less movement than this is a click, not a drag


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


class Editor:
    def __init__(self, yaml_path, out_name, args):
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
        self.dirty = False
        out = os.path.join(WP_DIR, self.out_name)
        if os.path.exists(out):
            handles, self.closed = handles_from_path(read_csv(out))
            self.pts = [tuple(p) for p in handles]
            print(f'opened {out}: {len(self.pts)} points', flush=True)
        # free our keys from matplotlib's defaults (c = back, r = home, a/z/s ...)
        for name in [k for k in plt.rcParams if k.startswith('keymap.')]:
            plt.rcParams[name] = [k for k in plt.rcParams[name]
                                  if k not in ('a', 'c', 'r', 'z', 's', 'enter')]
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.imshow(self.img, cmap='gray', origin='upper')
        self._crop_to_known_area()
        self.curve_line, = self.ax.plot([], [], '-', color='tab:orange', lw=1.5)
        self.click_line, = self.ax.plot([], [], 'o', color='tab:blue', ms=7, mec='white', mew=0.8)
        self.path_line = None
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.redraw()

    def _crop_to_known_area(self):
        known = np.argwhere(self.img != 205)
        if len(known):
            (r0, c0), (r1, c1) = known.min(0), known.max(0)
            pad = 20
            self.ax.set_xlim(c0 - pad, c1 + pad)
            self.ax.set_ylim(r1 + pad, r0 - pad)

    def title(self, msg=''):
        mode = 'closed loop' if self.closed else 'open path'
        changed = '  |  * not saved' if self.dirty else ''
        self.ax.set_title(f'{self.map_name}  |  {len(self.pts)} pts  |  {mode}  (c: toggle){changed}\n'
                          'click/a: add  drag: move  right: remove  z: undo  '
                          f'enter: save  r: reset  q: quit   {msg}')

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
        self.title(msg)
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
        print(f'[mouse] button={ev.button} in_map={ev.inaxes == self.ax} '
              f'tool={self.fig.canvas.toolbar.mode or "none"}', flush=True)
        if self.fig.canvas.toolbar.mode or ev.inaxes != self.ax:
            return  # ignore clicks while zoom/pan tool is active
        hit = self.point_at(ev)
        if ev.button == 3:
            if hit is not None:
                self.remember()
                del self.pts[hit]
        elif ev.button == 1:
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
        if self.drag is not None and self.drag_from is None:
            x, y = self.pts[self.drag]
            print(f'point {self.drag + 1}: moved to ({x:.2f}, {y:.2f}) m', flush=True)
        self.drag = self.drag_from = None

    def on_key(self, ev):
        if ev.key == 'a':
            self.add_point(ev)
        elif ev.key == 'z':
            self.undo()
        elif ev.key == 'c':
            self.remember()
            self.closed = not self.closed
        elif ev.key == 'r':
            self.remember()
            self.pts = []
        elif ev.key == 'enter':
            self.save()
            return
        elif ev.key in ('q', 'escape'):
            plt.close(self.fig)
            return
        self.redraw()

    def save(self):
        need = 3 if self.closed else 2
        if len(self.pts) < need:
            self.redraw(f'need at least {need} points')
            return
        a = self.args
        try:
            xy, length = smooth_path(self.pts, self.closed)
        except (ValueError, TypeError):
            self.redraw('cannot smooth: two points are on the same spot')
            return
        if not np.isfinite(xy).all():
            self.redraw('cannot smooth: two points are on the same spot')
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
        msg = f'saved {len(xy)} pts, {length:.1f} m, v {v.min():.2f}..{v.max():.2f} m/s'
        print(f'{msg} -> {out}', flush=True)
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
    # keep a reference: mpl_connect only holds weak refs, so an unreferenced Editor gets
    # garbage collected and every click/key callback silently disconnects
    editor = Editor(args.map_yaml, args.out, args)  # noqa: F841
    # the Qt event loop never returns to Python, so let Ctrl+C kill the process directly
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    plt.show()


if __name__ == '__main__':
    main()
