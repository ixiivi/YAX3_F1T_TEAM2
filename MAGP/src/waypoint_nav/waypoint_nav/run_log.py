"""CSV record of one driving run, so a failed run can be explained from the file.

Which column answers what:
  n_obs = 0                       the obstacle was not detected
  n_obs > 0, n_free = 0           detected, but every sideways shift was blocked
  n_free > 0, goal = 0            a way was free, but the line was kept
  goal != 0, offset does not move the shift is not being followed
  fit low                         localization is off (walls can then look like obstacles)
  n_contact, scan_change, frozen_s  the hit signals; check them first after a false recovery
  skew                            pose offset added to the obstacle threshold (high = pose off)
  v_plan, zone, v_odom            the route's speed (and full-throttle / full-brake zone) at the car
                                  against the measured speed: where a mark is early or late
  ax, ay, wz                      VESC IMU (sensors/imu/raw) forward / sideways acceleration and
                                  yaw rate, as the VESC sends them (g and deg/s, unconverted):
                                  grip used in corners, how hard it really brakes
"""
import csv
import os
import time

FIELDS = ('t', 'x', 'y', 'yaw', 'i', 'off_path', 'mode', 'running', 'status',
          'v_want', 'cmd_v', 'v_plan', 'zone', 'v_odom', 'ax', 'ay', 'wz', 'cmd_steer', 'offset', 'goal', 'n_obs', 'n_clusters', 'skew', 'n_free', 'n_cand',
          'fit', 'corrections', 'attempts', 'n_contact', 'n_contact_front', 'scan_change',
          'frozen_s', 'event')


class RunLog:
    """Opens its file on the first row. A disk problem disables the log, never the car."""

    def __init__(self, directory, warn):
        self.directory = os.path.expanduser(directory) if directory else ''
        self.warn = warn
        self.file = None
        self.writer = None
        self.t0 = None
        self.last_flush = 0.0

    def _open(self):
        os.makedirs(self.directory, exist_ok=True)
        path = os.path.join(self.directory, time.strftime('%Y%m%d_%H%M%S') + '.csv')
        self.file = open(path, 'w', newline='')
        self.writer = csv.DictWriter(self.file, FIELDS, restval='', extrasaction='ignore')
        self.writer.writeheader()
        self.t0 = time.monotonic()
        return path

    def write(self, row):
        if not self.directory:
            return None
        try:
            path = self._open() if self.file is None else None
            now = time.monotonic()
            self.writer.writerow({**row, 't': f'{now - self.t0:.2f}'})
            if now - self.last_flush > 1.0:
                self.file.flush()
                self.last_flush = now
            return path
        except OSError as e:
            self.warn(f'run log disabled: {e}')
            self.directory = ''
            return None

    def close(self):
        if self.file is not None:
            try:
                self.file.close()
            except OSError:
                pass
            self.file = None
