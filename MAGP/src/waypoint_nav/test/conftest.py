"""Lets the tests import waypoint_nav on a laptop without ROS 2.

On the Jetson (rclpy installed) nothing is stubbed and the real modules are used. Elsewhere
the ROS modules pure_pursuit imports are replaced by empty stand-ins, so its geometry, speed
and recovery methods can be tested directly on an object built without Node.__init__
(see bare_pp below).
"""
import os
import sys
import types

import numpy as np
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WS = os.path.dirname(os.path.dirname(PKG))          # .../MAGP
sys.path.insert(0, PKG)

ROS_MODULES = ['rclpy', 'rclpy.duration', 'rclpy.executors', 'rclpy.node', 'rclpy.qos',
               'rclpy.time', 'ackermann_msgs', 'ackermann_msgs.msg', 'geometry_msgs',
               'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg',
               'std_msgs', 'std_msgs.msg', 'tf2_ros', 'visualization_msgs',
               'visualization_msgs.msg']


class _AnyAttr(type):
    """Class attributes such as DurabilityPolicy.TRANSIENT_LOCAL read as None."""

    def __getattr__(cls, name):
        if name.startswith('__'):
            raise AttributeError(name)
        return None


class _Stub(types.ModuleType):
    """Any attribute is a do-nothing class, so `from x import Y` always works."""

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        cls = _AnyAttr(name, (), {'__init__': lambda self, *a, **k: None})
        setattr(self, name, cls)
        return cls


try:
    import rclpy  # noqa: F401
except ImportError:
    for name in ROS_MODULES:
        sys.modules[name] = _Stub(name)


def map_path(name):
    return os.path.join(WS, 'maps', f'{name}.yaml')


def waypoint_path(name):
    return os.path.join(WS, 'waypoints', f'{name}_wp.csv')


@pytest.fixture
def bare_pp():
    """A PurePursuit with the default car geometry but no ROS node behind it."""
    from waypoint_nav.pure_pursuit import PurePursuit
    pp = object.__new__(PurePursuit)
    pp.wheelbase, pp.max_steer = 0.33, 0.34
    pp.aeb_half_width, pp.aeb_base, pp.aeb_time = 0.26, 0.35, 0.4
    pp.car_front, pp.car_rear, pp.car_half_width = 0.42, 0.08, 0.21
    pp.laser_x = 0.267
    pp.min_speed, pp.min_speed_per_rad = 0.5, 1.0
    pp.accel, pp.stop_decel, pp.dt = 1.5, 2.0, 1.0 / 40
    pp.speed_cmd = 0.0
    pp.scan_xy = np.empty((0, 2))
    return pp
