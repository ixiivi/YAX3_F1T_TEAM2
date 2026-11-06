#!/usr/bin/env python3
"""Make waypoints by clicking in RViz with the "Publish Point" tool (/clicked_point).

Usage:
  ros2 run waypoint_nav rviz_waypoints --ros-args -p name:=room_loop [-p vmax:=3.0]

In RViz: pick "Publish Point" in the toolbar (or press its >> menu), click on the map.
Each click adds one point (the tool deselects after a click, pick it again).
Keys in this terminal:
  enter  smooth, compute speeds, save ~/MAGP/MAGP/waypoints/<name>.csv
  z      remove last point      c  toggle closed loop / open path
  r      clear all points       q  quit
The clicked points (orange) and the smoothed path (blue) show on /waypoints_preview.
"""
import os
import select
import sys
import termios
import time
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from waypoint_nav.path_tools import smooth_path, speed_profile, write_csv

WP_DIR = os.path.expanduser('~/MAGP/MAGP/waypoints')


class RvizWaypoints(Node):
    def __init__(self):
        super().__init__('rviz_waypoints')
        self.name = self.declare_parameter('name', 'waypoints').value
        self.vmax = self.declare_parameter('vmax', 3.0).value
        self.vmin = self.declare_parameter('vmin', 0.5).value
        self.alat = self.declare_parameter('alat', 2.0).value
        self.alon = self.declare_parameter('alon', 1.5).value
        self.pts = []
        self.closed = True
        self.path = None
        self.msg = ''
        self.pub = self.create_publisher(MarkerArray, '/waypoints_preview', 10)
        self.create_subscription(PointStamped, '/clicked_point', self.on_point, 10)
        self.create_timer(0.5, self.publish)

    def on_point(self, m):
        if m.header.frame_id not in ('map', '/map'):
            self.msg = f'ignored point in frame {m.header.frame_id}, set Fixed Frame to map'
            return
        self.pts.append((m.point.x, m.point.y))
        self.path = self.preview()
        self.msg = f'point {len(self.pts)}: ({m.point.x:.2f}, {m.point.y:.2f})'

    def preview(self):
        need = 3 if self.closed else 2
        if len(self.pts) < need:
            return None
        return smooth_path(self.pts, self.closed)[0]

    def save(self):
        need = 3 if self.closed else 2
        if len(self.pts) < need:
            self.msg = f'need at least {need} points'
            return
        xy, length = smooth_path(self.pts, self.closed)
        v = speed_profile(xy, self.closed, self.vmax, self.vmin, self.alat, self.alon)
        os.makedirs(WP_DIR, exist_ok=True)
        out = os.path.join(WP_DIR, f'{self.name}.csv')
        write_csv(out, xy, v)
        self.msg = f'saved {len(xy)} pts {length:.1f} m -> {out}'

    def publish(self):
        arr = MarkerArray()
        clicks = Marker(type=Marker.SPHERE_LIST, id=0, ns='clicks')
        clicks.scale.x = clicks.scale.y = clicks.scale.z = 0.12
        clicks.color.r, clicks.color.g, clicks.color.a = 1.0, 0.6, 1.0
        clicks.points = [Point(x=x, y=y) for x, y in self.pts]
        line = Marker(type=Marker.LINE_STRIP, id=1, ns='path')
        line.scale.x = 0.03
        line.color.b, line.color.g, line.color.a = 1.0, 0.6, 1.0
        if self.path is not None:
            pts = np.vstack([self.path, self.path[:1]]) if self.closed else self.path
            line.points = [Point(x=float(x), y=float(y)) for x, y in pts]
        for m in (clicks, line):
            m.header.frame_id = 'map'
            m.pose.orientation.w = 1.0
            if not m.points:
                m.action = Marker.DELETE
            arr.markers.append(m)
        self.pub.publish(arr)

    def key(self, k):
        if k in ('\n', '\r'):
            self.save()
        elif k == 'z' and self.pts:
            self.pts.pop()
        elif k == 'c':
            self.closed = not self.closed
        elif k == 'r':
            self.pts = []
        else:
            return
        if k != '\n' and k != '\r':
            self.path = self.preview()
            self.msg = f'{len(self.pts)} pts, {"closed loop" if self.closed else "open path"}'


def main():
    rclpy.init()
    node = RvizWaypoints()
    tty_ok = sys.stdin.isatty()
    old = termios.tcgetattr(sys.stdin) if tty_ok else None
    if tty_ok:
        tty.setcbreak(sys.stdin.fileno())
    print(__doc__)
    shown = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if tty_ok and select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1)
                if k == 'q':
                    break
                node.key(k)
            if node.msg != shown:
                print(node.msg, flush=True)
                shown = node.msg
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        if tty_ok:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
