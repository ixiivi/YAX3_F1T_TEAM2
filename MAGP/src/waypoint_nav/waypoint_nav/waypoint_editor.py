#!/usr/bin/env python3
"""Click waypoints on a saved map (.yaml + .pgm) and save a smoothed path as CSV.

Usage:
  ros2 run waypoint_nav waypoint_editor <map.yaml> [--out name.csv] [--vmax 3.0] [--alat 2.0]

Mouse / keys (in the map window):
  left click, a  add point (a = at the mouse cursor, for remote desktops)
  right click/z  remove last point
  c              toggle closed loop (lap) / open path
  enter          smooth, compute speeds, save CSV + preview PNG
  r              clear all points
  q / esc        quit

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

from waypoint_nav.path_tools import smooth_path, speed_profile, write_csv  # noqa: E402

WP_DIR = os.path.expanduser('~/MAGP/MAGP/waypoints')


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


class Editor:
    def __init__(self, yaml_path, out_name, args):
        self.img, self.res, self.origin = load_map(yaml_path)
        self.h = self.img.shape[0]
        self.args = args
        self.map_name = os.path.splitext(os.path.basename(yaml_path))[0]
        self.out_name = out_name or f'{self.map_name}_wp.csv'
        self.pts = []           # world coordinates
        self.closed = True
        # free our keys from matplotlib's defaults (c = back, r = home, a/z/s ...)
        for name in [k for k in plt.rcParams if k.startswith('keymap.')]:
            plt.rcParams[name] = [k for k in plt.rcParams[name]
                                  if k not in ('a', 'c', 'r', 'z', 's', 'enter')]
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.imshow(self.img, cmap='gray', origin='upper')
        self._crop_to_known_area()
        self.click_line, = self.ax.plot([], [], 'o-', color='tab:blue', ms=5, lw=1)
        self.path_line = None
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
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
        self.ax.set_title(f'{self.map_name}  |  {len(self.pts)} pts  |  {mode}  (c: toggle)\n'
                          f'left click or a: add  right/z: undo  enter: save  r: reset  q: quit   {msg}')

    def redraw(self, msg=''):
        if self.pts:
            px = [world_to_px(x, y, self.h, self.res, self.origin) for x, y in self.pts]
            u, v = zip(*px)
            self.click_line.set_data(u, v)
        else:
            self.click_line.set_data([], [])
        self.title(msg)
        self.fig.canvas.draw_idle()

    def add_point(self, ev):
        if ev.inaxes != self.ax or ev.xdata is None:
            return
        self.pts.append(px_to_world(ev.xdata, ev.ydata, self.h, self.res, self.origin))
        x, y = self.pts[-1]
        print(f'point {len(self.pts)}: ({x:.2f}, {y:.2f}) m', flush=True)

    def on_click(self, ev):
        print(f'[mouse] button={ev.button} in_map={ev.inaxes == self.ax} '
              f'tool={self.fig.canvas.toolbar.mode or "none"}', flush=True)
        if self.fig.canvas.toolbar.mode:
            return  # ignore clicks while zoom/pan tool is active
        if ev.button == 3:
            if self.pts:
                self.pts.pop()
        else:
            self.add_point(ev)
        self.redraw()

    def on_key(self, ev):
        if ev.key == 'a':
            self.add_point(ev)
        elif ev.key == 'z' and self.pts:
            self.pts.pop()
        elif ev.key == 'c':
            self.closed = not self.closed
        elif ev.key == 'r':
            self.pts = []
            if self.path_line:
                self.path_line.remove()
                self.path_line = None
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
        xy, length = smooth_path(self.pts, self.closed)
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
