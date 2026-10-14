#!/usr/bin/env python3
"""
Keyboard Teleop — 맵핑/디버깅용 키보드 조종.

/teleop (mux priority 100)으로 발행 → 자율주행 명령보다 항상 우선.
w/s 가감속, a/d 조향, space 정지, q 조향중립, Ctrl+C 종료.
"""
import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('speed_step', 0.05)
        self.declare_parameter('steer_step', 0.05)
        self.declare_parameter('max_steer', 0.36)
        self.pub = self.create_publisher(AckermannDriveStamped, '/teleop', 10)
        self.speed = 0.0
        self.steer = 0.0
        self.create_timer(0.1, self.publish_cmd)
        print('\n=== 키보드 조종 (mux 최우선) ===')
        print('w 가속 | s 감속/후진 | a 좌 | d 우')
        print('space 정지 | q 조향중립 | Ctrl+C 종료')
        print(f"최고속도 {self.get_parameter('max_speed').value} m/s")
        print('==============================\n')

    def publish_cmd(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steer
        self.pub.publish(msg)

    def update(self, key):
        vmax = self.get_parameter('max_speed').value
        smax = self.get_parameter('max_steer').value
        dv = self.get_parameter('speed_step').value
        ds = self.get_parameter('steer_step').value
        if key == 'w':
            self.speed = min(self.speed + dv, vmax)
        elif key == 's':
            self.speed = max(self.speed - dv, -vmax)
        elif key == 'a':
            self.steer = min(self.steer + ds, smax)
        elif key == 'd':
            self.steer = max(self.steer - ds, -smax)
        elif key == ' ':
            self.speed = 0.0
            self.steer = 0.0
        elif key == 'q':
            self.steer = 0.0
        print(f'\r속도 {self.speed:+.2f} m/s | 조향 {self.steer:+.2f} rad   ',
              end='', flush=True)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = KeyboardTeleop()
    settings = termios.tcgetattr(sys.stdin)
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03':
                break
            node.update(key)
    finally:
        stop = AckermannDriveStamped()
        stop.drive.speed = 0.0
        node.pub.publish(stop)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print()


if __name__ == '__main__':
    main()
