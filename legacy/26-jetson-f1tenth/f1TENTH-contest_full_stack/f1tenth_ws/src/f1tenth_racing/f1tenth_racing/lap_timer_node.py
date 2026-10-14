#!/usr/bin/env python3
"""
랩타임 측정 + 자동 기록 노드
- 스폰 지점 반경 내 진입 시 랩 완료 감지
- 랩타임 + 파라미터 세팅 → CSV 자동 저장
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
import csv
import os
import re
from datetime import datetime

# ── 결승선 설정 ────────────────────────────────────────────────────────────
SPAWN_X      =  0.0
SPAWN_Y      = -11.0
FINISH_RADIUS  = 3.0   # 결승선 감지 반경 (m)
MIN_DIST_AWAY  = 8.0   # 한 바퀴로 인정하려면 스폰에서 최소 이만큼 멀어져야 함 (m)
MIN_LAP_TIME   = 4.0   # 최소 랩타임 (초)

# ── 파일 경로 ──────────────────────────────────────────────────────────────
PKG_DIR         = os.path.dirname(os.path.abspath(__file__))
WP_PATH         = os.path.join(PKG_DIR, 'waypoints.csv')
RESULTS_PATH    = os.path.join(os.path.expanduser('~'),
                                'f1TENTH', 'lap_results.csv')
CONTROLLER_FILE = os.path.expanduser('~/.f1tenth_active_controller')

_CONTROLLER_TO_FILE = {
    'pure_pursuit': 'pure_pursuit_node.py',
    'hybrid':       'hybrid_controller_node.py',
}


def read_controller_name():
    """racing_node 가 저장한 컨트롤러 이름을 읽음. 없으면 'pure_pursuit'."""
    try:
        with open(CONTROLLER_FILE) as f:
            name = f.read().strip()
        return name if name in _CONTROLLER_TO_FILE else 'pure_pursuit'
    except Exception:
        return 'pure_pursuit'


def read_pursuit_params(pp_path):
    params = {'SPEED_SCALE': None, 'LA_BASE': None,
              'LA_K': None, 'PREVIEW_N': None, 'A_BRAKE': None}
    try:
        with open(pp_path) as f:
            for line in f:
                for key in params:
                    m = re.search(rf'self\.{key}\s*=\s*([0-9.]+)', line)
                    if m:
                        params[key] = float(m.group(1))
    except Exception:
        pass
    return params


def read_map_name(pp_path):
    """컨트롤러 소스파일의 WAYPOINTS_MAP 에서 맵 이름 추출."""
    try:
        with open(pp_path) as f:
            for line in f:
                m = re.search(r"WAYPOINTS_MAP\s*=\s*['\"](.+?)['\"]", line)
                if m:
                    return m.group(1)
    except Exception:
        pass
    return 'real_track'


def read_waypoint_speeds():
    speeds = []
    try:
        with open(WP_PATH) as f:
            for row in csv.DictReader(f):
                speeds.append(float(row['speed']))
    except Exception:
        pass
    if speeds:
        return round(max(speeds), 2), round(min(speeds), 2)
    return None, None


def ensure_csv_header():
    if not os.path.exists(RESULTS_PATH):
        with open(RESULTS_PATH, 'w', newline='') as f:
            csv.writer(f).writerow([
                'datetime', 'controller', 'map', 'lap', 'lap_time', 'best_lap',
                'SPEED_SCALE', 'LA_BASE', 'LA_K', 'PREVIEW_N', 'A_BRAKE',
                'vmax', 'vmin'
            ])


class LapTimer(Node):
    def __init__(self):
        super().__init__('lap_timer')

        self.lap_count      = 0
        self.best_lap       = float('inf')
        self.lap_start      = time.time()
        self.last_cross     = 0.0
        self._odom_received = False
        self._in_zone       = True   # 스폰 시 결승선 위 → True로 시작
        self._max_dist      = 0.0    # 이번 랩에서 스폰에서 최대로 멀어진 거리

        self.controller = read_controller_name()
        pp_path = os.path.join(PKG_DIR, _CONTROLLER_TO_FILE[self.controller])
        self.params   = read_pursuit_params(pp_path)
        self.map_name = read_map_name(pp_path)
        self.vmax, self.vmin = read_waypoint_speeds()

        ensure_csv_header()

        self.create_subscription(
            Odometry, '/ego_racecar/odom', self._odom_cb, 10)
        self.create_timer(3.0, self._status_cb)

        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info(' 랩타임 측정 + 기록 시작')
        self.get_logger().info(
            f' 결승선: ({SPAWN_X}, {SPAWN_Y}) 반경 {FINISH_RADIUS}m')
        self.get_logger().info(f' 컨트롤러: {self.controller}')
        self.get_logger().info(f' 맵: {self.map_name}')
        self.get_logger().info(f' 파라미터: {self.params}')
        self.get_logger().info(f' 속도: vmax={self.vmax}, vmin={self.vmin}')
        self.get_logger().info(f' 저장: {RESULTS_PATH}')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')

    def _status_cb(self):
        # 이 함수는 3초마다 상태만 출력 (랩 감지는 odom 50Hz에서 수행)
        if not self._odom_received:
            self.get_logger().warn(
                '⚠ odom 수신 없음 → 터미널 1 시뮬레이터 확인!')
        else:
            elapsed = time.time() - self.lap_start
            best_str = f'{self.best_lap:.3f}s' if self.lap_count > 0 else '-'
            self.get_logger().info(
                f' [상태] 경과: {elapsed:.1f}s | 완료: {self.lap_count}랩 | '
                f'베스트: {best_str} | 최대이탈: {self._max_dist:.1f}m'
                f' (기준: {MIN_DIST_AWAY}m)')

    def _odom_cb(self, msg):
        self._odom_received = True
        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        now   = time.time()

        dist = ((car_x - SPAWN_X)**2 + (car_y - SPAWN_Y)**2) ** 0.5

        # 이번 랩에서 최대 거리 갱신
        self._max_dist = max(self._max_dist, dist)

        if dist < FINISH_RADIUS:
            if (not self._in_zone
                    and self._max_dist > MIN_DIST_AWAY
                    and (now - self.last_cross) > MIN_LAP_TIME):
                # 결승선 진입 + 충분히 멀리 다녀옴 → 랩 완료
                lap_time = now - self.lap_start
                self.lap_count  += 1
                self.lap_start   = now
                self.last_cross  = now
                self._max_dist   = 0.0   # 다음 랩 초기화

                is_best = lap_time < self.best_lap
                if is_best:
                    self.best_lap = lap_time

                flag = '  ★ 베스트!' if is_best else ''
                self.get_logger().info(
                    '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
                self.get_logger().info(
                    f' LAP {self.lap_count:3d} │ {lap_time:.3f}s{flag}')
                self.get_logger().info(
                    f' 베스트: {self.best_lap:.3f}s')
                self.get_logger().info(
                    '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
                self._save_row(lap_time)

            self._in_zone = True
        else:
            self._in_zone = False

    def _save_row(self, lap_time):
        p = self.params
        try:
            with open(RESULTS_PATH, 'a', newline='') as f:
                csv.writer(f).writerow([
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    self.controller,
                    self.map_name,
                    self.lap_count,
                    round(lap_time, 3),
                    round(self.best_lap, 3),
                    p.get('SPEED_SCALE'), p.get('LA_BASE'),
                    p.get('LA_K'), p.get('PREVIEW_N'), p.get('A_BRAKE'),
                    self.vmax, self.vmin
                ])
            self.get_logger().info(f' CSV 저장 완료: {RESULTS_PATH}')
        except Exception as e:
            self.get_logger().error(f' CSV 저장 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LapTimer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
