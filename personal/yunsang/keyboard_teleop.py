#!/usr/bin/env python3
"""Keyboard teleop for the F1TENTH stack: publishes AckermannDriveStamped on /drive at 20 Hz.

w/s: speed +/-   a/d: steer left/right   x: center steering
space: smooth stop   b: hard stop   q: quit
"""
import math
import select
import sys
import termios
import time
import tty

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

MIN_SPEED = 0.5     # m/s, slower than this the sensorless motor cogs (driving straight)
MIN_SPEED_PER_RAD = 1.0  # m/s added per rad of steering: turning loads the motor more
SPEED_STEP = 0.25   # m/s per key press above MIN_SPEED
STEER_STEP = 0.05   # rad per key press
MAX_SPEED = 1.5     # m/s
MAX_STEER = 0.34    # rad
ACCEL = 1.0         # m/s^2 when speeding up
DECEL = 1.5         # m/s^2 when slowing down above MIN_SPEED
REVERSE_HOLD = 0.5  # s to hold the brake after stopping before driving off again
RATE = 20.0         # Hz


def clamp(v, lim):
    return max(-lim, min(lim, v))


def floor_speed(speed, steer):
    """Commanded speed: while moving, never below the cogging limit, which rises with steering."""
    if speed == 0.0:
        return 0.0
    return math.copysign(max(abs(speed), MIN_SPEED + MIN_SPEED_PER_RAD * abs(steer)), speed)


def step_speed(target, direction):
    """Next target speed for a w (+1) / s (-1) press, skipping the cogging band below MIN_SPEED."""
    if target == 0.0:
        return direction * MIN_SPEED
    nxt = target + direction * SPEED_STEP
    if abs(nxt) < MIN_SPEED - 1e-6:
        return 0.0
    return clamp(nxt, MAX_SPEED)


def ramp(current, target, dt):
    """Move current toward target without ever commanding speeds inside the cogging band.

    Starting jumps straight to MIN_SPEED; stopping ramps down to MIN_SPEED and then drops to 0,
    where ackermann_to_vesc switches to brake current.
    """
    if current * target < 0:
        # reverse requested: stop first
        target = 0.0
    if target == 0.0:
        if abs(current) <= MIN_SPEED + 1e-6:
            return 0.0
        return math.copysign(max(MIN_SPEED, abs(current) - DECEL * dt), current)
    if current == 0.0:
        return math.copysign(MIN_SPEED, target)
    lim = (DECEL if abs(target) < abs(current) else ACCEL) * dt
    return current + clamp(target - current, lim)


def main():
    rclpy.init()
    node = Node('keyboard_teleop')
    pub = node.create_publisher(AckermannDriveStamped, '/drive', 10)
    target, speed, steer = 0.0, 0.0, 0.0
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print(__doc__)
    last = stopped_at = time.monotonic()
    try:
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 1.0 / RATE)[0]:
                k = sys.stdin.read(1).lower()
                if k == 'w':
                    target = step_speed(target, 1)
                elif k == 's':
                    target = step_speed(target, -1)
                elif k == 'a':
                    steer = clamp(steer + STEER_STEP, MAX_STEER)
                elif k == 'd':
                    steer = clamp(steer - STEER_STEP, MAX_STEER)
                elif k == 'x':
                    steer = 0.0
                elif k == ' ':
                    target = 0.0
                elif k == 'b':
                    target, speed = 0.0, 0.0
                elif k == 'q':
                    break
            now = time.monotonic()
            if speed != 0.0:
                stopped_at = now
            if speed == 0.0 and target != 0.0 and now - stopped_at < REVERSE_HOLD:
                pass  # keep braking until the wheels have stopped before driving off again
            else:
                speed = ramp(speed, target, now - last)
            last = now
            # short enough not to wrap in a narrow terminal, which would scroll on every update
            cmd = floor_speed(speed, steer)
            print(f'\rv {cmd:+.2f}/{target:+.2f} st {steer:+.2f} ', end='', flush=True)
            msg = AckermannDriveStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.drive.speed = cmd
            msg.drive.steering_angle = steer
            pub.publish(msg)
    finally:
        pub.publish(AckermannDriveStamped())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
        print('\nstopped')


if __name__ == '__main__':
    main()
