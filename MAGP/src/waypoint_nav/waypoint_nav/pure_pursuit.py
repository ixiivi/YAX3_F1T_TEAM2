#!/usr/bin/env python3
"""Pure pursuit waypoint follower with obstacle bypass and localization recovery.

Pose comes from TF map -> base_link (AMCL), the path from a CSV (x,y,speed) made by
waypoint_editor. Publishes AckermannDriveStamped on /drive.

Starts PAUSED. Keys in this terminal:  enter: go   space: smooth stop   q: quit (hard stop)
Same from other programs: std_msgs/String 'go' / 'stop' / 'estop' on /pure_pursuit/command;
the status line is published on /pure_pursuit/status, notable events on /pure_pursuit/event.

With map_yaml set:
  * obstacle bypass: scan points that are not on the map are obstacles; the path is shifted
    sideways (never against the driving direction) just enough to pass them with clearance,
    and comes back to the line afterwards.
  * recovery: when the LiDAR says the car hit something (points right at the body next to a map
    wall, or the scan stopped changing although it should drive) or the scan stops fitting the
    map (localization lost), the car brakes to a stop, backs up along the track it just drove
    (that space was free a moment ago; the rear is a LiDAR blind spot) while turning parallel to
    the nearby wall, finds its pose by scan matching, hands it to AMCL and drives on. Each
    attempt without progress backs up further; after max_attempts it stops backing up: it waits
    until the way is free, or keeps searching for its pose. It never gives up by itself.
It also stops by itself when the pose is stale, something is on the arc it is about to drive
(AEB), no bypass is free, or the end of an open path is reached.

Every run is recorded as a CSV in run_log_dir (see run_log.py).
"""
import csv
import math
import os
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
import rclpy.duration
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from waypoint_nav.global_localize import initial_pose_msg
from waypoint_nav.run_log import RunLog
from waypoint_nav.scan_match import MapMatcher, scan_rays

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _num(value):
    return '' if value is None else f'{value:.3f}'


def load_waypoints(path):
    with open(os.path.expanduser(path)) as f:
        rows = [r for r in csv.DictReader(f)]
    xy = np.array([[float(r['x']), float(r['y'])] for r in rows])
    v = np.array([float(r.get('speed') or 1.0) for r in rows])
    return xy, v


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        p = self.declare_parameter
        p('waypoint_file', '')
        p('map_yaml', '')            # enables obstacle bypass and localization recovery
        p('max_speed', 1.0)          # hard cap on top of the CSV speeds [m/s]
        p('speed_scale', 1.0)        # multiplies CSV speeds
        p('min_speed', 0.5)          # below this the sensorless motor cogs (driving straight)
        p('min_speed_per_rad', 1.0)  # added per rad of steering: turning loads the motor more
        p('lookahead_min', 0.6)      # [m]
        p('lookahead_gain', 0.4)     # lookahead = min + gain * speed
        p('wheelbase', 0.33)
        p('max_steer', 0.34)
        # AEB: stop if a scan point lies on the arc the car is about to drive (current steering),
        # within base + v * aeb_time metres and aeb_half_width of the arc
        # car body around base_link (rear axle centre), measured 2026-11-03
        p('car_front', 0.42)         # [m] front bumper ahead of base_link (wheelbase 0.33 + 0.09)
        p('car_rear', 0.08)          # [m] rear bumper behind base_link
        p('car_half_width', 0.21)    # [m] (width 0.42)
        p('aeb_base', 0.35)          # [m] gap kept ahead of the front bumper
        p('aeb_time', 0.4)           # [s]
        p('aeb_half_width', 0.26)    # [m] half car width 0.21 + margin
        p('laser_x', 0.267)          # laser position ahead of base_link [m] (6.3 cm behind front axle)
        p('pose_timeout', 0.5)       # [s]
        p('accel', 1.5)              # [m/s^2] speed-up limit
        p('stop_decel', 2.0)         # [m/s^2] motor-controlled slow-down for space / slower corners
        p('rate', 40.0)
        p('autostart', False)        # start driving without pressing enter (simulation tests)
        # obstacle bypass
        p('avoid_enabled', True)
        p('avoid_max_offset', 0.6)   # [m] furthest sideways shift from the line
        p('avoid_clearance', 0.36)   # [m] path to obstacle / wall: half width 0.21 + margin
        p('avoid_slope', 0.5)        # sideways metres per metre driven while shifting
        p('avoid_horizon', 3.0)      # [m] how far ahead a shifted path must be free
        p('avoid_speed', 1.0)        # [m/s] speed cap while shifted
        # scan points further than this from any map wall are obstacles; generous, because a few
        # cm of AMCL error already moves wall points off the mapped walls
        p('obstacle_min_map_dist', 0.25)
        # localization recovery
        p('recovery_enabled', True)
        # below this fit while driving, search near the pose and hand the better one to AMCL
        # without stopping (AMCL drifts while moving; standing still it fits ~98%)
        p('track_fit', 0.7)
        p('track_radius', 0.5)       # [m] search window around the pose at scan time
        p('track_yaw_deg', 15.0)     # [deg]
        p('lost_fit', 0.45)          # scan fraction on map walls below this ...
        p('lost_time', 0.3)          # ... for this long [s] = localization lost
        p('resume_fit', 0.6)         # needed after relocalizing to drive on
        p('accept_fit', 0.75)        # a local re-search result is trusted from this fit on
        p('reverse_enabled', True)
        p('reverse_dist', 0.4)       # [m] back along the driven track, times the attempt number
        p('reverse_speed', 1.0)      # [m/s]
        p('reverse_timeout', 3.0)    # [s]
        p('reverse_settle', 0.1)     # [s] brake at least this long before reversing
        p('reverse_settle_max', 2.0) # [s] reverse after this even if odom still reports motion
        p('search_settle', 0.1)      # [s] the pose search uses a scan at least this much newer
        p('verify_time', 0.3)        # [s] AMCL gets this long with the new pose before the check
        p('align_gain', 0.6)         # steering per rad of angle to the side wall while reversing
        p('max_attempts', 3)         # recoveries in a row without progress before no more reverse
        p('progress_dist', 1.0)      # [m] driven past the trouble spot for it to count as progress
        p('retry_wait', 5.0)         # [s] blocked this long after max_attempts: back up again
        # hit detection, LiDAR only: wheels can spin freely against a wall, so odometry cannot tell
        p('hit_start_grace', 1.5)    # [s] after a start: the motor may take a moment to get going
        p('contact_band', 0.05)      # [m] points this close to the body count as contact ...
        p('contact_points', 3)       # ... at least this many, on two scans in a row ...
        p('contact_wall_dist', 0.25) # ... and this close to a map wall (a car is not a wall)
        p('frozen_change', 0.08)     # [m] a beam changed if its range moved more than this
        p('frozen_fraction', 0.04)   # below this fraction of changed beams the scan is frozen
        p('frozen_time', 0.5)        # [s] frozen while commanding speed = hit
        p('frozen_stop_time', 1.0)   # [s] frozen while AEB or BLOCKED holds the car = hit
        p('run_log_dir', '~/MAGP/MAGP/runs')  # CSV record of each run; empty = off
        p('run_log_rate', 10.0)      # [Hz]
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.gp = gp

        wp_file = gp('waypoint_file')
        if not wp_file:
            raise SystemExit('waypoint_file parameter is required')
        self.xy, self.v_ref = load_waypoints(wp_file)
        self.closed = np.linalg.norm(self.xy[0] - self.xy[-1]) < 0.5
        csv_max = float(self.v_ref.max() * gp('speed_scale'))
        if gp('max_speed') > csv_max + 1e-6:
            self.get_logger().warn(
                f'max_speed {gp("max_speed"):.2f} is above the CSV speeds (max {csv_max:.2f}); '
                'the CSV wins. Regenerate the waypoints with a higher vmax or raise speed_scale.')
        self.v_ref = np.minimum(self.v_ref * gp('speed_scale'), gp('max_speed'))
        self.get_logger().info(
            f'{len(self.xy)} waypoints from {wp_file}, '
            f'{"closed loop" if self.closed else "open path"}, '
            f'speed {self.v_ref.min():.2f}..{self.v_ref.max():.2f} m/s')
        # left normals and segment lengths of the path, for sideways shifts
        nxt = np.roll(self.xy, -1, 0) if self.closed else np.vstack([self.xy[1:], self.xy[-1:]])
        prv = np.roll(self.xy, 1, 0) if self.closed else np.vstack([self.xy[:1], self.xy[:-1]])
        t = nxt - prv
        t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
        self.normal = np.column_stack([-t[:, 1], t[:, 0]])
        self.seg = np.linalg.norm(np.roll(self.xy, -1, 0) - self.xy, axis=1)

        self.min_speed = gp('min_speed')
        self.min_speed_per_rad = gp('min_speed_per_rad')
        self.ld_min, self.ld_gain = gp('lookahead_min'), gp('lookahead_gain')
        self.wheelbase, self.max_steer = gp('wheelbase'), gp('max_steer')
        self.aeb_base, self.aeb_time = gp('aeb_base'), gp('aeb_time')
        self.aeb_half_width, self.laser_x = gp('aeb_half_width'), gp('laser_x')
        self.car_front, self.car_rear = gp('car_front'), gp('car_rear')
        self.car_half_width = gp('car_half_width')
        self.pose_timeout = gp('pose_timeout')
        self.accel, self.stop_decel = gp('accel'), gp('stop_decel')
        self.dt = 1.0 / gp('rate')

        self.matcher = None
        if gp('map_yaml'):
            self.matcher = MapMatcher(gp('map_yaml'), self.laser_x)
            inside = float(self.matcher.inside(self.xy[:, 0], self.xy[:, 1]).mean())
            if inside < 0.9:
                raise SystemExit(f'웨이포인트의 {(1 - inside) * 100:.0f}%가 맵 밖에 있어요. '
                                 f'{wp_file} 은(는) {gp("map_yaml")} 맵용이 아닌 것 같아요.')
        else:
            self.get_logger().warn('no map_yaml: obstacle bypass and recovery are off')
        self.avoid_on = bool(self.matcher) and gp('avoid_enabled')
        self.recovery_on = bool(self.matcher) and gp('recovery_enabled')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.path_pub = self.create_publisher(Path, '/waypoints', LATCHED)
        self.target_pub = self.create_publisher(Marker, '/pure_pursuit/target', 10)
        self.avoid_pub = self.create_publisher(Path, '/pure_pursuit/avoid_path', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        # remote control (control panel): 'go', 'stop' (smooth), 'estop' (brake now)
        self.create_subscription(String, '/pure_pursuit/command', self.on_command, 10)
        self.status_pub = self.create_publisher(String, '/pure_pursuit/status', 10)
        self.event_pub = self.create_publisher(String, '/pure_pursuit/event', 10)

        self.scan = None
        self.scan_xy = np.empty((0, 2))   # scan points in the base_link frame
        self.idx = None
        self.running = gp('autostart')
        self.speed_cmd = 0.0
        self.status = 'paused'
        self.offset = 0.0                 # current sideways shift of the followed line [m]
        self.last_line, self.blocked_since = None, 0.0
        self.fit = None
        self.low_since = None
        self.last_fit_t = 0.0
        self.mode = 'drive'               # drive | reverse | search | verify | wait
        self.attempts = 0
        self.trail = []                   # recent forward track in the odom frame
        self.search_result = None
        self.searching = False
        self.search_wait = None
        self.tracking = False             # a nearby search for drift correction is running
        self.track_result = None
        self.track_gen = 0
        self.corrections = 0
        self.contact_xy = np.empty((0, 2))  # scan points right at the body, base_link frame
        self.contact_scans = 0            # scans in a row with contact
        self.frozen_ref, self.frozen_since = None, time.monotonic()
        self.scan_change = 0.0            # fraction of beams changed since frozen_ref
        self.go_t = time.monotonic()
        self.wait_since = None            # blocked with no reverse attempts left, since when
        self.wait_t0 = 0.0
        self.verify_at = 0.0
        self.recovered_at = None          # odom position where the last recovery drove off
        self.trouble_at = None            # odom position where the last recovery started
        self.rev_start, self.rev_t0, self.rev_last = None, 0.0, None
        self.cmd = (0.0, 0.0)             # last speed and steering sent
        self.v_want = 0.0
        self.plan = {}                    # last choose_offset result, for the run log
        self.pose_seen = None
        self.err_seen = None
        self.events = []
        self.run_log = RunLog(gp('run_log_dir'), self.get_logger().warn)
        self.publish_path()
        self.create_timer(1.0 / gp('rate'), self.step)
        self.create_timer(0.2, lambda: self.status_pub.publish(String(data=self.status)))
        self.create_timer(1.0 / gp('run_log_rate'), self.log_row)

    # ---------- commands / io ----------
    def go(self):
        self.running = True
        self.idx = None  # re-acquire the nearest waypoint
        self.mode = 'drive'
        self.low_since = None
        self.go_t = time.monotonic()
        self.frozen_ref = None
        self.contact_scans = 0
        self.track_gen += 1

    def on_command(self, msg):
        if msg.data == 'go':
            self.attempts = 0
            self.go()
        elif msg.data == 'stop':
            self.running = False
            self.mode = 'drive'
        elif msg.data == 'estop':
            self.running = False
            self.mode = 'drive'
            self.stop('E-STOP')

    def event(self, text):
        self.get_logger().info(text)
        self.event_pub.publish(String(data=text))
        self.events.append(text)

    def log_row(self):
        """One row of the run record, only while something is happening."""
        if not (self.running or self.mode != 'drive' or self.speed_cmd > 0.0):
            return
        pose = self.pose_seen or (None, None, None)
        path = self.run_log.write({
            'x': _num(pose[0]), 'y': _num(pose[1]), 'yaw': _num(pose[2]),
            'i': self.idx, 'off_path': _num(self.err_seen), 'mode': self.mode,
            'running': int(self.running), 'status': self.status,
            'v_want': _num(self.v_want), 'cmd_v': _num(self.cmd[0]),
            'cmd_steer': _num(self.cmd[1]), 'offset': _num(self.offset),
            'goal': _num(self.plan.get('goal')), 'n_obs': self.plan.get('n_obs', ''),
            'n_free': self.plan.get('n_free', ''), 'n_cand': self.plan.get('n_cand', ''),
            'fit': _num(self.fit), 'corrections': self.corrections,
            'attempts': self.attempts, 'n_contact': len(self.contact_xy),
            'scan_change': _num(self.scan_change),
            'frozen_s': _num(time.monotonic() - self.frozen_since),
            'event': ' / '.join(self.events)})
        self.events = []
        if path:
            self.get_logger().info(f'run log: {path}')

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = 'map'
        for x, y in self.xy:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def on_scan(self, scan):
        self.scan = scan
        r = np.asarray(scan.ranges)
        ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > scan.range_min) & (r < 5.0)
        xy = np.column_stack([self.laser_x + r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])])
        # drop returns from the car itself (wheels, body) so they never count as obstacles
        m = 0.03
        body = ((xy[:, 0] > -self.car_rear - m) & (xy[:, 0] < self.car_front + m)
                & (np.abs(xy[:, 1]) < self.car_half_width + m))
        self.scan_xy = xy[~body]
        # hit signal 1: points in a thin band around the body
        m += self.gp('contact_band')
        near = ((xy[:, 0] > -self.car_rear - m) & (xy[:, 0] < self.car_front + m)
                & (np.abs(xy[:, 1]) < self.car_half_width + m))
        self.contact_xy = xy[near & ~body]
        enough = len(self.contact_xy) >= self.gp('contact_points')
        self.contact_scans = self.contact_scans + 1 if enough else 0
        # hit signal 2: the scan stays the same as a reference taken when it last changed
        now = time.monotonic()
        if self.frozen_ref is None or len(self.frozen_ref) != len(r):
            self.frozen_ref, self.frozen_since, self.scan_change = r, now, 0.0
            return
        both = np.isfinite(r) & np.isfinite(self.frozen_ref)
        moved = np.abs(r[both] - self.frozen_ref[both]) > self.gp('frozen_change')
        self.scan_change = float(moved.mean()) if moved.size else 0.0
        if self.scan_change > self.gp('frozen_fraction'):
            self.frozen_ref, self.frozen_since = r, now

    def rays(self, scan, step):
        """scan_rays without the returns from the car body (they are not on the map)."""
        r, a = scan_rays(scan, step=step)
        x, y = self.laser_x + r * np.cos(a), r * np.sin(a)
        m = 0.03
        body = ((x > -self.car_rear - m) & (x < self.car_front + m)
                & (np.abs(y) < self.car_half_width + m))
        return r[~body], a[~body]

    def lookup(self, parent, stamp=None):
        """Pose of base_link in `parent`: latest, or at `stamp` (e.g. the scan time)."""
        try:
            t = self.tf_buffer.lookup_transform(parent, 'base_link', stamp or Time())
        except TransformException:
            return None
        if stamp is None:
            age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
            if age > self.pose_timeout:
                return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_from_quat(tr.rotation)

    def drive(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.cmd = (float(speed), float(steer))
        self.drive_pub.publish(msg)

    def stop(self, why):
        """Hard stop now: speed 0, which ackermann_to_vesc turns into brake current."""
        self.speed_cmd = 0.0
        self.status = why
        self.drive(0.0, 0.0)

    # ---------- geometry ----------
    def arc_obstacle(self, steer, reach):
        """Distance along the arc driven with this steering to the first scan point on it."""
        x, y = self.scan_xy[:, 0], self.scan_xy[:, 1]
        k = math.tan(steer) / self.wheelbase
        if abs(k) < 1e-3:
            s, lat = x, np.abs(y)
        else:
            R = 1.0 / k  # turn centre at (0, R)
            lat = np.abs(np.hypot(x, y - R) - abs(R))
            s = abs(R) * np.arctan2(x / R, (R - y) / R)
        hit = (lat < self.aeb_half_width) & (s > 0.0) & (s < reach)
        return float(s[hit].min()) if hit.any() else None

    def pursuit_steer(self, x, y, yaw, tx, ty):
        """Pure pursuit steering towards (tx, ty); also right for a target behind the car."""
        dx, dy = tx - x, ty - y
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
        dist = max(math.hypot(lx, ly), 1e-3)
        return math.atan2(2.0 * self.wheelbase * ly, dist * dist)

    def clamp_steer(self, s):
        return max(-self.max_steer, min(self.max_steer, s))

    def nearest(self, x, y):
        n = len(self.xy)
        if self.idx is None:
            cand = np.arange(n)  # first fix: search everywhere
        else:
            # search a window ahead of the last index so loops that pass close to
            # themselves do not make the index jump
            cand = (self.idx + np.arange(-10, 60)) % n if self.closed else \
                np.arange(max(0, self.idx - 10), min(n, self.idx + 60))
        d = np.hypot(self.xy[cand, 0] - x, self.xy[cand, 1] - y)
        self.idx = int(cand[np.argmin(d)])
        return self.idx, float(d.min())

    def ahead(self, i, length):
        """Waypoint indices from i forward over `length` metres, with distance along the path."""
        n = len(self.xy)
        count = min(n, int(length / max(np.median(self.seg), 1e-3)) + 2)
        idx = (i + np.arange(count)) % n if self.closed else np.arange(i, min(n, i + count))
        s = np.concatenate([[0.0], np.cumsum(self.seg[idx[:-1]])])
        return idx, s

    def shifted(self, idx, s, goal, release):
        """Line points for a bypass: shift from self.offset to goal, hold it up to `release`
        metres ahead (just past the obstacle), then come back to the line. Sideways change is
        limited to avoid_slope per metre along the path."""
        slope = self.gp('avoid_slope')
        off = np.empty(len(s))
        cur = self.offset
        for k in range(len(s)):
            want = goal if s[k] <= release else 0.0
            ds = s[k] - s[k - 1] if k else 0.0
            cur += max(-slope * ds, min(slope * ds, want - cur))
            off[k] = cur
        return self.xy[idx] + self.normal[idx] * off[:, None]

    def obstacles_map(self, x, y, yaw):
        """Scan points (map frame) that the map does not explain: new obstacles."""
        if len(self.scan_xy) == 0:
            return None
        c, s = math.cos(yaw), math.sin(yaw)
        px = x + c * self.scan_xy[:, 0] - s * self.scan_xy[:, 1]
        py = y + s * self.scan_xy[:, 0] + c * self.scan_xy[:, 1]
        new = self.matcher.wall_dist(px, py) > self.gp('obstacle_min_map_dist')
        if not new.any():
            return None
        return np.column_stack([px[new], py[new]])

    def choose_offset(self, i, x, y, yaw):
        """Smallest sideways shift whose path ahead is free; None if every shift is blocked."""
        idx, s = self.ahead(i, self.gp('avoid_horizon'))
        clear, m = self.gp('avoid_clearance'), self.gp('avoid_max_offset')
        obs = self.obstacles_map(x, y, yaw)
        tree, release = None, -1.0
        if obs is not None:
            tree = cKDTree(obs)
            # obstacles that can touch a shifted line: release the shift just past the last one
            line_tree = cKDTree(self.xy[idx])
            d, k = line_tree.query(obs)
            near = d < m + clear
            if near.any():
                release = float(s[k[near]].max()) + 0.5
        best, best_cost, best_line = None, float('inf'), None
        goals = np.round(np.arange(-m, m + 1e-9, 0.1), 2)
        n_free = 0
        # a shift must keep the clearance, but where the line itself runs closer to a wall it
        # only must not get closer than the line: the line is what the car is meant to drive,
        # and rejecting it would stop the car on an empty track
        line_wall = self.matcher.wall_dist(self.xy[idx, 0], self.xy[idx, 1])
        wall_limit = np.minimum(clear, line_wall) - 1e-6
        for goal in goals:
            pts = self.shifted(idx, s, goal, release)
            if (self.matcher.wall_dist(pts[:, 0], pts[:, 1]) < wall_limit).any():
                continue
            if tree is not None and np.isfinite(tree.query(pts, distance_upper_bound=clear)[0]).any():
                continue
            n_free += 1
            cost = abs(goal) + 0.5 * abs(goal - self.offset)
            if cost < best_cost:
                best, best_cost, best_line = float(goal), cost, pts
        self.plan = {'goal': best, 'n_obs': 0 if obs is None else len(obs),
                     'n_free': n_free, 'n_cand': len(goals)}
        # what the shift should be right here: the goal while the obstacle is still ahead
        here = best if best is not None and release > 0.0 else 0.0
        return best, here, best_line

    def wall_angle(self):
        """Angle of the nearest side wall relative to the car heading, in (-pi/2, pi/2]."""
        p = self.scan_xy
        side = p[(np.abs(p[:, 0]) < 1.5) & (np.abs(p[:, 1]) > self.car_half_width + 0.05)
                 & (np.abs(p[:, 1]) < 1.5)]
        if len(side) < 15:
            return 0.0
        left, right = side[side[:, 1] > 0], side[side[:, 1] < 0]
        pts = left if len(left) >= len(right) else right
        if len(pts) < 15:
            return 0.0
        q = pts - pts.mean(0)
        w, v = np.linalg.eigh(q.T @ q)
        th = math.atan2(v[1, -1], v[0, -1])
        while th > math.pi / 2:
            th -= math.pi
        while th <= -math.pi / 2:
            th += math.pi
        return th

    def rear_blocked(self):
        """Something within 0.3 m behind the rear bumper, in the car's width (what the LiDAR
        can see of it: the area right behind is a blind spot)."""
        p = self.scan_xy
        if len(p) == 0:
            return False
        behind = (p[:, 0] < -self.car_rear) & (p[:, 0] > -self.car_rear - 0.3)
        return bool((behind & (np.abs(p[:, 1]) < self.car_half_width + 0.1)).any())

    # ---------- speed ----------
    def ramp(self, target):
        """Next speed command. Never commands the cogging band below min_speed: starting
        jumps to min_speed, stopping slows down by motor speed control to min_speed and
        then drops to 0 (brake)."""
        cur = self.speed_cmd
        if target == 0.0:
            if cur <= self.min_speed + 1e-6:
                return 0.0
            return max(self.min_speed, cur - self.stop_decel * self.dt)
        if cur < self.min_speed:
            return self.min_speed
        if target > cur:
            return min(target, cur + self.accel * self.dt)
        return max(target, cur - self.stop_decel * self.dt)

    def floor(self, v, steer):
        return math.copysign(max(abs(v), self.min_speed + self.min_speed_per_rad * abs(steer)), v)

    # ---------- recovery ----------
    def update_fit(self, x, y, yaw):
        now = time.monotonic()
        if self.matcher is None or self.scan is None or now - self.last_fit_t < 0.2:
            return
        self.last_fit_t = now
        # the pose when the scan was taken: at 1 m/s a pose a scan-period newer is cm off
        at_scan = self.lookup('map', Time.from_msg(self.scan.header.stamp)) or (x, y, yaw)
        self.fit = self.matcher.fit(*at_scan, *self.rays(self.scan, 3))
        if self.fit < self.gp('lost_fit'):
            self.low_since = self.low_since or now
        else:
            self.low_since = None
        if self.fit < self.gp('track_fit') and not self.tracking:
            self.tracking = True
            threading.Thread(target=self._track, args=(self.scan, at_scan, self.track_gen),
                             daemon=True).start()

    def _track(self, scan, at_scan, gen):
        r, a = self.rays(scan, 3)
        res = self.matcher.local_search(*at_scan, r, a, radius=self.gp('track_radius'),
                                        yaw_range_deg=self.gp('track_yaw_deg'))
        self.track_result = (gen, self.matcher.fit(*res[1:], r, a), at_scan, res[1:])

    def apply_track(self, x, y, yaw):
        """Correct AMCL on the fly with the result of _track; the car keeps driving."""
        if self.track_result is None:
            return
        gen, fit, (sx, sy, syaw), (bx, by, byaw) = self.track_result
        self.track_result, self.tracking = None, False
        # a result from before a stop or a relocalization belongs to a pose AMCL no longer has
        if gen != self.track_gen or fit < self.gp('accept_fit') or self.fit is None \
                or fit <= self.fit:
            return
        # the search used an older scan: move the pose now by the same correction
        dyaw = math.atan2(math.sin(byaw - syaw), math.cos(byaw - syaw))
        c, s = math.cos(dyaw), math.sin(dyaw)
        nx = bx + c * (x - sx) - s * (y - sy)
        ny = by + s * (x - sx) + c * (y - sy)
        msg = initial_pose_msg(self, nx, ny, yaw + dyaw)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.005
        self.pose_pub.publish(msg)
        self.corrections += 1
        self.low_since = None
        self.last_fit_t = time.monotonic() + 0.3   # give AMCL a moment before judging again
        self.get_logger().info(
            f'drift fix #{self.corrections}: {math.hypot(bx - sx, by - sy) * 100:.0f} cm '
            f'{math.degrees(dyaw):+.1f} deg, fit {self.fit * 100:.0f}% -> {fit * 100:.0f}%')

    def hit_reason(self, x, y, yaw):
        """Why the LiDAR says the car hit something while it should drive, or None."""
        now = time.monotonic()
        if self.contact_scans >= 2:
            c, s = math.cos(yaw), math.sin(yaw)
            px = x + c * self.contact_xy[:, 0] - s * self.contact_xy[:, 1]
            py = y + s * self.contact_xy[:, 0] + c * self.contact_xy[:, 1]
            on_wall = self.matcher.wall_dist(px, py) < self.gp('contact_wall_dist')
            if on_wall.sum() >= self.gp('contact_points'):
                return f'벽 접촉: 차체 옆 라이다 점 {int(on_wall.sum())}개'
        if now - self.go_t < self.gp('hit_start_grace'):
            return None
        frozen = now - self.frozen_since
        limit = self.gp('frozen_stop_time') if self.cmd[0] == 0.0 else self.gp('frozen_time')
        if frozen > limit:
            return f'{frozen:.1f}초 동안 스캔 변화 없음: {self.status}'
        return None

    def way_free(self, pose):
        """Nothing on the line ahead or right at the body: driving on is worth a try."""
        x, y, yaw = pose
        i, _ = self.nearest(x, y)
        goal, _, _ = self.choose_offset(i, x, y, yaw)
        reach = self.car_front + self.aeb_base + max(self.v_ref[i], self.min_speed) * self.aeb_time
        return goal is not None and self.contact_scans == 0 and self.arc_obstacle(0.0, reach) is None

    def note_progress(self):
        """Attempts only start over once the car has driven back past where it got stuck, plus
        progress_dist; otherwise a car stuck at the same spot would back up and retry forever."""
        if self.recovered_at is None or self.trouble_at is None:
            return
        odom = self.lookup('odom')
        if odom is None:
            return
        rx, ry = self.recovered_at[:2]
        backed = math.hypot(self.trouble_at[0] - rx, self.trouble_at[1] - ry)
        if math.hypot(odom[0] - rx, odom[1] - ry) > backed + self.gp('progress_dist'):
            self.attempts = 0
            self.recovered_at = None
            self.wait_since = None

    def start_recovery(self, reason, hit=False):
        self.running = False
        self.track_gen += 1
        self.recovered_at = None
        self.stop(f'LOST: {reason}')
        if self.attempts >= self.gp('max_attempts'):
            # backing up has not helped: no more blind reversing, but do not give up either
            if hit:
                self.mode = 'wait'
                self.wait_t0 = time.monotonic()
                self.wait_since = self.wait_since or self.wait_t0
                self.event(f'복구 {self.attempts}번 실패 ({reason}) → 후진 없이 길이 열릴 때까지 대기')
            else:
                self.mode = 'search'
                self.event(f'복구 {self.attempts}번 실패 ({reason}) → 후진 없이 위치 찾기 반복')
            return
        self.attempts += 1
        odom = self.lookup('odom')
        self.trouble_at = odom
        if self.gp('reverse_enabled') and odom and len(self.trail) >= 3:
            self.mode = 'reverse'
            self.rev_start, self.rev_t0, self.rev_last = None, time.monotonic(), odom
            self.event(f'복구 시작 ({reason}) → 지나온 길로 후진 후 위치 재설정 '
                       f'(시도 {self.attempts}/{self.gp("max_attempts")})')
        else:
            self.mode = 'search'
            self.event(f'복구 시작 ({reason}) → 위치 재설정 '
                       f'(시도 {self.attempts}/{self.gp("max_attempts")})')

    def step_wait(self, pose):
        self.drive(0.0, 0.0)
        now = time.monotonic()
        self.status = f'waiting for a free way {now - self.wait_since:.1f} s'
        if now - self.wait_since > self.gp('retry_wait'):
            self.event('대기 시간 초과 → 복구를 처음부터 다시 시도')
            self.attempts = 0
            self.wait_since = None
            self.go()
        elif now - self.wait_t0 > 0.5 and pose is not None and self.way_free(pose):
            self.go()

    def record_trail(self):
        odom = self.lookup('odom')
        if odom is None:
            return
        if not self.trail or math.hypot(odom[0] - self.trail[-1][0], odom[1] - self.trail[-1][1]) > 0.05:
            self.trail.append(odom[:2])
            del self.trail[:-200]

    def step_reverse(self):
        odom = self.lookup('odom')
        if odom is None:
            return self.stop('reverse: no odom')
        ox, oy, oyaw = odom
        if self.rev_start is None:
            # come to rest first: reversing into a car still rolling forward is hard on the
            # drivetrain, and the distance backed up must not count the roll-out
            now = time.monotonic()
            rolling = math.hypot(ox - self.rev_last[0], oy - self.rev_last[1]) > 0.005
            self.rev_last = odom
            waited = now - self.rev_t0
            if (waited < self.gp('reverse_settle') or rolling) and waited < self.gp('reverse_settle_max'):
                return self.stop('braking before reverse')
            self.rev_start, self.rev_t0 = odom, now
        moved = math.hypot(ox - self.rev_start[0], oy - self.rev_start[1])
        while self.trail and math.hypot(ox - self.trail[-1][0], oy - self.trail[-1][1]) < 0.3:
            self.trail.pop()
        why = None
        if moved >= self.gp('reverse_dist') * self.attempts:
            why = f'{moved:.2f} m'
        elif time.monotonic() - self.rev_t0 > self.gp('reverse_timeout'):
            why = 'timeout'
        elif not self.trail:
            why = 'end of track'
        elif self.rear_blocked():
            why = 'something behind'
        if why:
            self.stop(f'reverse done ({why})')
            self.mode = 'search'
            return
        tx, ty = self.trail[-1]
        # retrace the track, turning towards parallel with the side wall
        steer = self.pursuit_steer(ox, oy, oyaw, tx, ty) - self.gp('align_gain') * self.wall_angle()
        steer = self.clamp_steer(steer)
        v = self.floor(-self.gp('reverse_speed'), steer)
        self.status = f'reverse {moved:.2f} m st {steer:+.2f}'
        self.drive(v, steer)

    def step_search(self, pose):
        self.drive(0.0, 0.0)
        if self.search_result is None and not self.searching:
            # use a scan taken after the car has come to rest
            if self.search_wait is None:
                self.search_wait = self.get_clock().now() + rclpy.duration.Duration(
                    seconds=self.gp('search_settle'))
                self.status = 'stopping to look around...'
                return
            if self.scan is None or Time.from_msg(self.scan.header.stamp) < self.search_wait:
                return
            self.search_wait = None
            self.searching = True
            scan = self.scan
            guess = pose
            threading.Thread(target=self._search, args=(scan, guess), daemon=True).start()
            self.status = 'relocalizing...'
            return
        if self.search_result is None:
            return
        fit, x, y, yaw = self.search_result
        self.search_result, self.searching = None, False
        if fit < self.gp('resume_fit'):
            return self.start_recovery(f'맞는 위치를 못 찾았어요 (맞음 {fit * 100:.0f}%)')
        for _ in range(2):
            self.pose_pub.publish(initial_pose_msg(self, x, y, yaw))
        self.mode, self.verify_at = 'verify', time.monotonic() + self.gp('verify_time')
        self.status = f'relocalized ({x:.2f}, {y:.2f}), checking...'

    def _search(self, scan, guess):
        r, a = self.rays(scan, 3)
        ok = lambda res: self.matcher.fit(*res[1:], r, a) >= self.gp('accept_fit')  # noqa: E731
        res = None
        if guess is not None:
            # near the (wrong) estimate first: quick, and right when AMCL only slipped a bit.
            # Judge it by the strict fit, the soft score also likes wrong places nearby
            res = self.matcher.local_search(*guess, r, a)
        if res is None or not ok(res):
            res = self.matcher.global_search(*self.rays(scan, 6))
            # the global grid is coarse: polish around its answer with the denser scan
            res = max(res, self.matcher.local_search(*res[1:], r, a, radius=0.3, yaw_range_deg=6))
        fit = self.matcher.fit(*res[1:], r, a)
        if guess is not None:
            dyaw = math.atan2(math.sin(res[3] - guess[2]), math.cos(res[3] - guess[2]))
            self.get_logger().info(
                f'relocalize: {math.hypot(res[1] - guess[0], res[2] - guess[1]) * 100:.0f} cm '
                f'{math.degrees(dyaw):+.1f} deg from the estimate, fit {fit * 100:.0f}%')
        self.search_result = (fit,) + tuple(res[1:])

    def step_verify(self, pose):
        self.drive(0.0, 0.0)
        if time.monotonic() < self.verify_at or pose is None:
            return
        fit = self.matcher.fit(*pose, *self.rays(self.scan, 3))
        if fit >= self.gp('resume_fit'):
            self.event(f'위치 재설정 완료 (맞음 {fit * 100:.0f}%) → 재출발')
            # the trail stays: if the car gets stuck again right away it must still be able to
            # back up. It is in the odom frame, so the pose correction does not affect it.
            self.recovered_at = self.lookup('odom')
            self.go()
        else:
            self.start_recovery(f'재설정 후에도 안 맞아요 ({fit * 100:.0f}%)')

    # ---------- main loop ----------
    def step(self):
        pose = self.lookup('map')
        self.pose_seen = pose
        if self.mode == 'reverse':
            return self.step_reverse()
        if self.mode == 'search':
            return self.step_search(pose)
        if self.mode == 'verify':
            return self.step_verify(pose)
        if self.mode == 'wait':
            return self.step_wait(pose)
        if pose is None:
            return self.stop('NO POSE (localization?)')
        x, y, yaw = pose
        i, err = self.nearest(x, y)
        self.err_seen = err
        if self.running and self.recovery_on:
            self.note_progress()
            self.apply_track(x, y, yaw)
            self.update_fit(x, y, yaw)
            if self.low_since and time.monotonic() - self.low_since > self.gp('lost_time'):
                return self.start_recovery(f'맵과 {self.fit * 100:.0f}%만 맞음')
            hit = self.hit_reason(x, y, yaw)
            if hit:
                return self.start_recovery(hit, hit=True)
        if self.running and err > 1.0:
            if self.recovery_on:
                return self.start_recovery(f'경로에서 {err:.2f} m 벗어남')
            self.running = False
            return self.stop(f'STOP off path {err:.2f} m')
        if self.running and not self.closed and i >= len(self.xy) - 2:
            self.running = False
            return self.stop('goal reached')
        if not self.running and self.speed_cmd == 0.0:
            return self.stop(f'paused i {i}/{len(self.xy)} off-path {err:.2f} m')

        v_ref = max(self.v_ref[i], self.min_speed)
        self.v_want = v_ref
        ld = self.ld_min + self.ld_gain * v_ref
        if self.avoid_on:
            goal, here, line = self.choose_offset(i, x, y, yaw)
            now = time.monotonic()
            if goal is None and self.last_line is not None and now - self.blocked_since < 0.3:
                # a single blocked plan right beside the obstacle: keep the last line slowly
                # for a moment instead of braking (AEB below still guards the arc)
                goal, here, line = self.offset, self.offset, self.last_line
                v_ref = self.min_speed
            elif goal is None:
                # nothing free: wait in place (running stays on, it drives on once clear)
                self.last_line = None
                self.publish_avoid(None)
                return self.stop('BLOCKED: no free way around')
            else:
                self.last_line, self.blocked_since = line, now
            # move the shift along with the distance driven
            span = self.gp('avoid_slope') * max(self.speed_cmd, self.min_speed) * self.dt
            self.offset += max(-span, min(span, here - self.offset))
            if abs(self.offset) > 0.05 or abs(here) > 0.05:
                v_ref = min(v_ref, max(self.gp('avoid_speed'), self.min_speed))
            self.publish_avoid(line if abs(self.offset) > 0.01 or abs(here) > 0.01 else None)
        else:
            idx, _ = self.ahead(i, ld + 1.0)
            line = self.xy[idx]
        # first point of the (possibly shifted) line at least the lookahead away
        d = np.hypot(line[:, 0] - x, line[:, 1] - y)
        k = int(np.argmax(d >= ld)) if (d >= ld).any() else len(line) - 1
        tx, ty = line[k]
        steer = self.clamp_steer(self.pursuit_steer(x, y, yaw, tx, ty))

        v = self.ramp(v_ref if self.running else 0.0)
        # reach from the speed it wants to drive, not the current one: otherwise the reach
        # shrinks once AEB has stopped the car and it creeps on towards the obstacle
        hit = self.arc_obstacle(steer, self.car_front + self.aeb_base + v_ref * self.aeb_time)
        if hit is not None:
            return self.stop(f'AEB obstacle {hit - self.car_front:.2f} m ahead of bumper')

        self.speed_cmd = v
        if v > 0.0:
            # the ramp state stays as is; only the command gets the steering-dependent floor
            v = self.floor(v, steer)
            self.record_trail()
        mode = 'run' if self.running else 'stopping'
        extra = f' off {self.offset:+.2f}' if abs(self.offset) > 0.01 else ''
        fit = f' fit {self.fit * 100:.0f}%' if self.fit is not None else ''
        fit += f' fix {self.corrections}' if self.corrections else ''
        self.status = f'{mode} v {v:.2f} st {steer:+.2f} i {i}{extra}{fit}'
        self.drive(v, steer)
        self.publish_target(tx, ty)

    def publish_target(self, tx, ty):
        m = Marker()
        m.header.frame_id = 'map'
        m.type = Marker.SPHERE
        m.pose.position.x, m.pose.position.y = float(tx), float(ty)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.g, m.color.a = 1.0, 1.0
        self.target_pub.publish(m)

    def publish_avoid(self, line):
        msg = Path()
        msg.header.frame_id = 'map'
        if line is not None:
            for x, y in line:
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.pose.position.x, ps.pose.position.y = float(x), float(y)
                ps.pose.orientation.w = 1.0
                msg.poses.append(ps)
        self.avoid_pub.publish(msg)


def main():
    rclpy.init()
    node = PurePursuit()
    tty_ok = sys.stdin.isatty()
    old = termios.tcgetattr(sys.stdin) if tty_ok else None
    if tty_ok:
        tty.setcbreak(sys.stdin.fileno())
    print(__doc__)
    last_print = 0.0
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.01)
            except Exception:
                if not rclpy.ok():   # a callback hit the context a signal just shut down
                    break
                raise
            if tty_ok and select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1)
                if k in ('\n', '\r'):
                    node.attempts = 0
                    node.go()
                elif k == ' ':
                    node.running = False
                    node.mode = 'drive'
                elif k == 'q':
                    break
            now = time.monotonic()
            if now - last_print > 0.1:
                # short enough not to wrap in a narrow terminal
                print(f'\r{node.status[:48]:<48}', end='', flush=True)
                last_print = now
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.run_log.close()
        try:
            node.drive(0.0, 0.0)
        except Exception:  # context already shut down by a signal
            pass
        if tty_ok:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()
        print('\nstopped')


if __name__ == '__main__':
    main()
