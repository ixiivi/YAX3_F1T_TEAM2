#!/usr/bin/env python3
"""Publish /joy from an F710 in X mode by reading its USB reports directly.

Only X mode reports how far RT is pressed, and this kernel has no xpad driver, so
joy_node cannot see the pad in that mode. Set the switch on the back of the pad to X.

/joy layout:
  axes:    0 steering (+left), 1 left stick Y (+up), 2 right stick X (+left),
           3 right stick Y (+up), 4 RT (0..1), 5 LT (0..1), 6 throttle RT - LT (-1..1, see shape_throttle)
  buttons: 0 X, 1 A, 2 B, 3 Y, 4 LB, 5 RB, 8 back, 9 start, 10 L3, 11 R3
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import usb.core
import usb.util

VENDOR_ID = 0x046d
PRODUCT_ID = 0xc21f
ENDPOINT_IN = 0x81
PUBLISH_PERIOD = 0.02
READ_TIMEOUT_MS = 10
NEUTRAL_AXES = [0.0] * 7
NEUTRAL_BUTTONS = [0] * 12


def shape_steering(x, deadzone, full):
    """Map stick X to -1..1 using X alone, so vertical drift cannot change it.

    Reaching full lock before the edge keeps it reachable when the stick is also
    pushed up or down, where the round gate limits how far X can go.
    """
    size = (abs(x) - deadzone) / (full - deadzone)
    size = max(0.0, min(size, 1.0))
    return size if x > 0.0 else -size


def shape_throttle(t, deadzone, minimum):
    """Map RT - LT to -1..1 with a floor: any press past the deadzone asks for at least minimum.

    Below about 0.5 m/s the sensorless motor cogs: it stalls, the speed loop winds up, and the
    car then jumps off. A light press now starts at the floor instead.
    """
    size = (abs(t) - deadzone) / (1.0 - deadzone)
    if size <= 0.0:
        return 0.0
    size = minimum + (1.0 - minimum) * min(size, 1.0)
    return size if t > 0.0 else -size


def parse_report(data, deadzone, full, throttle_deadzone=0.0, throttle_min=0.0):
    """Return (axes, buttons) from an input report, or None for any other message."""
    if len(data) < 14 or data[0] != 0x00 or data[1] != 0x14:
        return None

    def stick(index):
        return int.from_bytes(bytes(data[index:index + 2]), 'little', signed=True) / 32767.0

    low, high = data[2], data[3]
    axes = [
        shape_steering(-stick(6), deadzone, full),
        max(-1.0, stick(8)),
        -max(-1.0, stick(10)),
        max(-1.0, stick(12)),
        data[5] / 255.0,
        data[4] / 255.0,
    ]
    # joy_teleop maps one axis to the speed: RT drives forward, LT backward, both cancel
    axes.append(shape_throttle(axes[4] - axes[5], throttle_deadzone, throttle_min))
    buttons = [
        int(bool(high & 0x40)),     # X
        int(bool(high & 0x10)),     # A
        int(bool(high & 0x20)),     # B
        int(bool(high & 0x80)),     # Y
        int(bool(high & 0x01)),     # LB
        int(bool(high & 0x02)),     # RB
        0,
        0,
        int(bool(low & 0x20)),      # back
        int(bool(low & 0x10)),      # start
        int(bool(low & 0x40)),      # L3
        int(bool(low & 0x80)),      # R3
    ]
    return axes, buttons


class F710XInput(Node):

    def __init__(self):
        super().__init__('joy')
        self.declare_parameter('steer_deadzone', 0.08)
        self.declare_parameter('steer_full', 0.92)
        self.declare_parameter('silence_timeout', 1.0)
        self.declare_parameter('throttle_deadzone', 0.05)
        self.declare_parameter('throttle_min', 0.4)
        self.pub = self.create_publisher(Joy, 'joy', 10)
        self.device = None
        self.was_connected = False
        self.axes = list(NEUTRAL_AXES)
        self.buttons = list(NEUTRAL_BUTTONS)
        self.last_report = 0.0
        self.last_publish = 0.0

    def set_neutral(self):
        self.axes = list(NEUTRAL_AXES)
        self.buttons = list(NEUTRAL_BUTTONS)

    def connect(self):
        device = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        if device is None:
            self.get_logger().warn('F710 not found in X mode: set the back switch to X',
                                   throttle_duration_sec=5.0)
            return False
        device.set_configuration()
        usb.util.claim_interface(device, 0)
        self.device = device
        self.was_connected = True
        self.last_report = time.monotonic()
        self.get_logger().info('F710 connected in X mode')
        return True

    def disconnect(self):
        if self.device is not None:
            usb.util.dispose_resources(self.device)
        self.device = None
        self.set_neutral()

    def read_report(self):
        deadzone = self.get_parameter('steer_deadzone').value
        full = self.get_parameter('steer_full').value
        try:
            data = self.device.read(ENDPOINT_IN, 64, timeout=READ_TIMEOUT_MS)
        except usb.core.USBTimeoutError:
            return
        report = parse_report(data, deadzone, full,
                              self.get_parameter('throttle_deadzone').value,
                              self.get_parameter('throttle_min').value)
        if report is not None:
            self.axes, self.buttons = report
            self.last_report = time.monotonic()

    def step(self):
        now = time.monotonic()
        if self.device is None and not self.connect():
            time.sleep(READ_TIMEOUT_MS / 1000.0)
        elif self.device is not None:
            try:
                self.read_report()
            except usb.core.USBError as e:
                self.get_logger().error(f'F710 USB error, publishing neutral: {e}')
                self.disconnect()

        # The pad only reports changes, so a pad that dies with RB held looks like RB
        # still being held. Treat a long silence as a release.
        silent = now - self.last_report > self.get_parameter('silence_timeout').value
        if self.device is not None and silent and self.buttons != NEUTRAL_BUTTONS:
            self.get_logger().warn('no F710 report while a button is held, releasing it')
            self.set_neutral()

        # Keep publishing neutral after losing the pad so joy_teleop still holds the car.
        # Before the first connection nothing is published, which shows as "no input".
        if self.was_connected and now - self.last_publish >= PUBLISH_PERIOD:
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = [float(a) for a in self.axes]
            msg.buttons = list(self.buttons)
            self.pub.publish(msg)
            self.last_publish = now


def main():
    rclpy.init()
    node = F710XInput()
    try:
        while rclpy.ok():
            node.step()
    except KeyboardInterrupt:
        pass
    finally:
        node.disconnect()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
