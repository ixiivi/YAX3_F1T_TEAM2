#!/usr/bin/env python3
"""
scan_deskew.py — 주행 중 찌그러진 라이다 스캔을 IMU 각속도로 펴준다 (motion unwarping).

왜 필요한가
───────────
2D 라이다도 한 바퀴 도는 데 시간이 걸린다. Hokuyo 40Hz 면 **한 스캔에 25ms**.
그 사이에 차가 움직이면 스캔의 첫 빔과 마지막 빔이 **서로 다른 위치에서** 찍힌
것인데, LaserScan 메시지는 전부 같은 순간에 찍힌 것처럼 담긴다. 그래서 점군이
찌그러진다 (motion blur / skew).

    코너에서 yaw rate 2 rad/s → 스캔 도중 2.9° 회전
    → 5m 거리의 점이 25cm 어긋난다

AMCL 의 likelihood field 는 이 찌그러진 점을 맵에 억지로 맞추려 하므로 가중치가
잘못 계산되고, 급코너에서 위치가 튄다. **속도를 올릴수록 정확히 비례해 나빠진다.**

무엇을 하나
───────────
빔 i 의 실제 촬영 시각 t_i = t0 + i·time_increment 를 기준으로,
그 순간의 차량 자세(Δθ, Δx, Δy)를 각속도·선속도로 보간해 되돌린다.

    p_i(기준시각) = R(Δθ_i)·(r_i·cos a_i, r_i·sin a_i) + (Δx_i, Δy_i)

되돌린 점을 다시 극좌표로 바꿔 **원래 각도 격자에 재배치(re-bin)** 한다.
같은 칸에 여러 점이 오면 가까운 쪽을 남기고(보수적), 빈 칸은 inf 로 둔다
(AMCL 은 range_max 밖을 무시한다).

★ 정지 상태나 각속도가 아주 작을 때는 계산을 건너뛰고 원본을 그대로 넘긴다.
  보정량이 노이즈보다 작으면 재배치가 오히려 점을 흩뜨린다.

입력/출력
─────────
  구독: /scan (LaserScan), /odom_imu (Odometry — 자이로 기반 yaw rate + 속도)
  발행: /scan_deskewed (LaserScan)

`race_script.yaml` 의 `localization.scan_deskew: true` 면 AMCL 이 이 토픽을 본다.
"""
import math
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanDeskew(Node):
    def __init__(self):
        super().__init__('scan_deskew')

        self.declare_parameter('scan_in',   '/scan')
        self.declare_parameter('scan_out',  '/scan_deskewed')
        self.declare_parameter('odom_topic', '/odom_imu')
        # header.stamp 가 스캔의 어느 시점인가. urg_node 는 첫 빔 기준(start).
        # 여기가 틀리면 보정이 반대 방향으로 걸려 오히려 나빠진다.
        self.declare_parameter('stamp_at',  'start')      # start | end | mid
        # 이보다 각속도가 작으면 보정하지 않고 원본을 그대로 넘긴다 (rad/s)
        self.declare_parameter('min_yaw_rate', 0.05)
        # 이보다 속도가 작으면 병진 보정을 생략한다 (m/s)
        self.declare_parameter('min_speed',    0.10)
        # time_increment 가 0 으로 오는 드라이버 대비 — scan_time/빔수로 만든다
        self.declare_parameter('fallback_scan_time', 0.025)   # 40Hz
        self.declare_parameter('report_sec', 10.0)            # 통계 로그 주기

        gp = lambda n: self.get_parameter(n).value
        self.stamp_at = str(gp('stamp_at')).lower()
        self.min_w    = float(gp('min_yaw_rate'))
        self.min_v    = float(gp('min_speed'))
        self.fb_scan  = float(gp('fallback_scan_time'))
        self.report_s = float(gp('report_sec'))

        self.w = 0.0          # yaw rate (rad/s)
        self.v = 0.0          # 전진 속도 (m/s)
        self._odom_seen = False
        self._n_in = 0
        self._n_fix = 0
        self._max_corr = 0.0
        self._last_report = None
        self._warned_ti = False

        self.pub = self.create_publisher(LaserScan, gp('scan_out'),
                                         qos_profile_sensor_data)
        self.create_subscription(LaserScan, gp('scan_in'), self.scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, gp('odom_topic'), self.odom_cb, 20)

        self.get_logger().info(
            f'ScanDeskew 시작 — {gp("scan_in")} → {gp("scan_out")}, '
            f'stamp_at={self.stamp_at}, odom={gp("odom_topic")}')

    def odom_cb(self, msg: Odometry):
        self.w = msg.twist.twist.angular.z
        self.v = msg.twist.twist.linear.x
        self._odom_seen = True

    def scan_cb(self, msg: LaserScan):
        self._n_in += 1
        n = len(msg.ranges)
        if n < 8 or not self._odom_seen:
            self.pub.publish(msg)
            return

        w, v = float(self.w), float(self.v)
        if abs(w) < self.min_w and abs(v) < self.min_v:
            self.pub.publish(msg)       # 거의 정지 — 보정할 게 없다
            return

        # ── 빔별 시각 오프셋 ───────────────────────────────────────────
        ti = float(msg.time_increment)
        if ti <= 0.0:
            scan_t = float(msg.scan_time) if msg.scan_time > 0 else self.fb_scan
            ti = scan_t / float(n)
            if not self._warned_ti:
                self._warned_ti = True
                self.get_logger().warn(
                    f'time_increment 가 0 이라 scan_time/{n} = {ti*1e6:.1f}us 로 '
                    '대체합니다. 드라이버가 빔 시각을 안 채워주는 경우입니다.')
        dt = np.arange(n, dtype=float) * ti
        if self.stamp_at == 'end':
            dt -= dt[-1]                # 마지막 빔이 기준
        elif self.stamp_at == 'mid':
            dt -= dt[n // 2]

        # ── 그 순간의 자세 차이 (등속·등각속도 가정) ────────────────────
        dth = w * dt
        dx = v * dt * np.cos(0.5 * dth)      # 원호를 직선으로 근사 (dt<25ms)
        dy = v * dt * np.sin(0.5 * dth)

        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(n) * msg.angle_increment
        valid = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        if not valid.any():
            self.pub.publish(msg)
            return

        # ── 기준 시각의 센서 좌표계로 되돌리기 ──────────────────────────
        x = r * np.cos(a)
        y = r * np.sin(a)
        c, s = np.cos(dth), np.sin(dth)
        xr = c * x - s * y + dx
        yr = s * x + c * y + dy

        r2 = np.hypot(xr, yr)
        a2 = np.arctan2(yr, xr)

        # ── 원래 각도 격자에 재배치 ────────────────────────────────────
        idx = np.rint((a2 - msg.angle_min) / msg.angle_increment).astype(int)
        keep = valid & (idx >= 0) & (idx < n)
        out = np.full(n, np.inf)
        # 같은 칸에 여러 점이면 가까운 쪽을 남긴다 — 장애물을 놓치지 않는 방향
        np.minimum.at(out, idx[keep], r2[keep])

        new = LaserScan()
        new.header = msg.header
        new.angle_min = msg.angle_min
        new.angle_max = msg.angle_max
        new.angle_increment = msg.angle_increment
        new.time_increment = msg.time_increment
        new.scan_time = msg.scan_time
        new.range_min = msg.range_min
        new.range_max = msg.range_max
        new.ranges = out.astype(np.float32).tolist()
        new.intensities = []        # 재배치로 대응이 깨져 그대로 넘기면 거짓말이 된다
        self.pub.publish(new)

        self._n_fix += 1
        corr = float(np.abs(r2[keep] - r[keep]).max()) if keep.any() else 0.0
        self._max_corr = max(self._max_corr, corr)
        self._report(msg)

    def _report(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_report is None:
            self._last_report = t
            return
        if t - self._last_report < self.report_s:
            return
        self._last_report = t
        self.get_logger().info(
            f'[deskew] {self._n_fix}/{self._n_in} 프레임 보정, '
            f'이 구간 최대 보정량 {self._max_corr*100:.1f}cm, '
            f'현재 ω={self.w:+.2f}rad/s v={self.v:+.2f}m/s')
        self._max_corr = 0.0


def main():
    rclpy.init()
    node = ScanDeskew()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
