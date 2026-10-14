#!/usr/bin/env python3
"""Kernel /dev/input/js* → /joy 브리지 — ROS joy_node 의 드롭인 대체.

■ 왜 있나 (2026-08-09)
  bringup 의 joy_node(SDL) + joy_teleop 이 /teleop 을 같이 쏘면서 손을 안 대도
  차가 나갔다. joy_teleop.yaml 이 drive-speed 를 axis1 × scale 5.0 으로 잡는데,
  SDL 리매핑 때문에 그 "axis 1" 이 **놓은 상태 −1 인 트리거**에 걸려
  −1 × 5.0 = −5㎧ 가 됐다. mux 우선순위가 teleop:100 vs drive:10 이라 그대로 나간다.
  SDL 이 축을 어떻게 재배열할지는 패드·드라이버·모드에 따라 달라 예측이 안 된다.
  → 커널 이벤트를 직접 읽어 **축 번호가 패드 물리 배치와 일치**하게 만든다.

■ 부호 규약 (2026-08-10 — 처음엔 틀렸다)
  처음엔 "커널 값을 그대로" 내보냈는데, ROS `joy` 패키지는 커널 값을 **negate**
  해서 발행한다 (실측: 커널 axis2 −32767 → /joy +1.0000).
  그래서 브리지를 쓰는 mapping-joy 만 **모든 축이 반대**가 됐고, 트리거는
  joystick_teleop 의 자동 캘리가 부호를 흡수해 티가 안 나서 **조향에서만**
  증상이 드러났다. 지금은 여기서 negate 해 joy_node 와 규약을 맞춘다.

  ★ 이 파일은 joy_node 의 **드롭인 대체**여야 한다. 쓰는 쪽(race_script /
    vehicle_car*.yaml)이 드라이버마다 다른 invert 를 관리하게 만들면 안 된다.

  즉 얻는 것은 "SDL 축 재배열 회피" 하나뿐이고, 부호·스케일은 ROS 규약 그대로다.
  휴지값 부호는 joystick_teleop 이 기동 시 자동 측정해 맞춘다.

  python3 js_joy_bridge.py [/dev/input/js1]
"""
import os
import struct
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

JS_EVENT_FMT = 'IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


class JsJoyBridge(Node):
    def __init__(self, device: str):
        super().__init__('js_joy_bridge')
        self.device = device
        self.pub = self.create_publisher(Joy, '/joy', 10)
        self.axes = {}
        self.buttons = {}
        self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
        self.get_logger().info(f'열린 장치: {device} → /joy (raw 커널 스케일)')
        self.create_timer(0.02, self._tick)

    def _publish(self):
        n_ax = max(list(self.axes.keys()) + [7], default=7) + 1
        n_btn = max(list(self.buttons.keys()) + [11], default=11) + 1
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.device
        msg.axes = [float(self.axes.get(i, 0.0)) for i in range(n_ax)]
        msg.buttons = [int(self.buttons.get(i, 0)) for i in range(n_btn)]
        self.pub.publish(msg)

    def _drain(self):
        while True:
            try:
                raw = os.read(self.fd, JS_EVENT_SIZE)
            except BlockingIOError:
                break
            if len(raw) < JS_EVENT_SIZE:
                break
            _t, value, etype, number = struct.unpack(JS_EVENT_FMT, raw)
            is_init = bool(etype & JS_EVENT_INIT)
            etype &= ~JS_EVENT_INIT
            if etype == JS_EVENT_AXIS:
                # ★ 부호를 뒤집는다 — ROS `joy` 패키지와 같은 규약으로 맞추기 위해서다.
                #   joy_node 는 커널 값을 negate 해서 발행한다 (2026-08-10 실측:
                #   커널 axis2 = -32767 인데 /joy 에는 +1.0000 으로 나온다).
                #   여기서 raw 를 그대로 내보내면 **모든 축이 반대**가 되어,
                #   같은 race_script/vehicle 설정인데도 브리지를 쓰는 mapping-joy 만
                #   조향이 좌우 반대로 동작한다 (실제로 그랬다).
                #   트리거는 teleop 의 기동 시 자동 캘리가 부호를 흡수해서 티가 안 나고,
                #   자동 캘리가 없는 조향에서만 증상이 드러나 원인을 찾기 어렵다.
                #   → 브리지는 joy_node 의 **드롭인 대체**여야 한다. 설정은 그대로 두고
                #     여기서 규약을 맞춘다.
                self.axes[number] = max(-1.0, min(1.0, -value / 32767.0))
            elif etype == JS_EVENT_BUTTON:
                self.buttons[number] = 1 if value else 0
            if is_init:
                pass  # 초기값도 그대로 반영

    def _tick(self):
        self._drain()
        # 이벤트 전에도 0축 발행 → teleop 자동 캘리 샘플이 쌓이게
        self._publish()


def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else '/dev/input/js1'
    if not os.path.exists(dev):
        for cand in ('/dev/input/js0', '/dev/input/js1'):
            if os.path.exists(cand):
                dev = cand
                break
        else:
            print(f'조이 장치 없음: {dev}', file=sys.stderr)
            sys.exit(1)

    rclpy.init()
    node = JsJoyBridge(dev)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            os.close(node.fd)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
