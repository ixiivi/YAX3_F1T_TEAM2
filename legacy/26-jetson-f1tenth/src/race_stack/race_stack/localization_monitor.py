#!/usr/bin/env python3
"""localization_monitor — 위치추정 품질을 실시간 숫자로 만든다.

★ 왜 필요한가
  지금까지 AMCL 품질은 주행이 끝난 뒤 궤적을 보고 **추측**했다. "벽에 박았으니
  위치가 틀렸겠지" 수준이다. 그래서 08/02 부터 미해결인 "맵 스케일이 맞는가" 도
  판정할 수단이 없었다.

  이 노드는 매 스캔마다 이렇게 묻는다:

      지금 AMCL 이 말하는 자세가 맞다면, 라이다는 무엇을 봐야 하는가?
      실제로 본 것과 얼마나 다른가?

  차이(잔차)가 곧 위치추정 오차다. 맵을 레이캐스팅해 기대거리를 구하는 방식은
  stanley_controller 의 장애물 판정(2026-08-05)과 같은 기계다.

★ 맵 스케일 판정은 **여기서 하지 않는다** (2026-08-05 검증 결과)
  처음엔 "잔차가 거리에 비례해 자라면 맵 스케일 오류" 로 판정하려 했다.
  세 가지 추정기를 실측 자세로 검증한 결과 전부 실패했다.

    · |잔차|~거리 회귀      : 정지 상태인데도 창마다 -16% ~ +7% 로 부호가 뒤집힘
    · 원점통과 비율 회귀    : 부호는 맞히나 자세 오차 20cm 만으로 ±2~5% 가짜 신호
    · 자세+스케일 동시 최적화: +3% 주입 → -5% 추정. **잔차 0.1cm 로 완벽히 맞음**

  마지막 결과가 원인을 말해 준다. 출발 지점은 긴 복도라, 스캔을 확대·축소한 뒤
  옆으로 밀면 같은 벽에 맞는다 — 스케일과 자세가 **기하학적으로 분리되지 않는다**
  (aperture problem 의 스케일 판). 알고리즘 문제가 아니라 관측 기하의 한계다.

  틀린 값을 확신에 차서 출력하는 것은 아무것도 안 하느니 못하므로 뺐다.
  **맵 스케일은 줄자로 트랙 실측 길이를 재서 맵 치수와 비교하는 것이 정답이다.**
  (이 맵: 33.1 × 9.3 m 라고 주장하고 있다)

★ 무엇을 하지 않는가
  이 노드는 **읽기 전용**이다. TF 도 /drive 도 건드리지 않는다. 그래서 켜 두어도
  주행에 영향이 없고, 어떤 위치추정 방식(AMCL·SynPF·Cartographer)을 쓰든
  같은 잣대로 비교할 수 있다. 새 필터를 도입할 때 "좋아졌는가" 를 판단할
  유일한 근거가 된다.
"""
import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from PIL import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


def _load_occ(map_yaml: str):
    """맵 이미지를 (점유 bool [H,W], (h,w,res,ox,oy)) 로 읽는다.

    stanley_controller._load_map 과 같은 규약이다 — 행 0 이 위(최대 y).
    map_server 와 동일하게 yaml 의 image: 를 그대로 따른다 (같은 이름의 미리보기
    PNG 를 집으면 해상도가 통째로 어긋난다).
    """
    cfg = yaml.safe_load(open(map_yaml))
    d = os.path.dirname(os.path.abspath(map_yaml))
    img_p = os.path.join(d, cfg['image'])
    if not os.path.exists(img_p):
        stem = os.path.splitext(map_yaml)[0]
        for ext in ('.pgm', '.png'):
            if os.path.exists(stem + ext):
                img_p = stem + ext
                break
    arr = np.asarray(Image.open(img_p).convert('L'), dtype=np.uint8)
    # 행 0 이 위(최대 y) 인 배열. negate/occupied_thresh 는 map_server 규약.
    occ = (arr < 128) if not cfg.get('negate', 0) else (arr > 128)
    h, w = occ.shape
    return occ, (h, w, float(cfg['resolution']),
                 float(cfg['origin'][0]), float(cfg['origin'][1]))


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('odom_topic', '/odom_imu')
        self.declare_parameter('laser_offset_x', 0.27)
        self.declare_parameter('laser_offset_y', 0.0)
        self.declare_parameter('fov_deg', 180.0)
        self.declare_parameter('beam_deg', 2.0)
        # 이 거리 안의 빔만 쓴다 — 멀수록 입사각이 눕고 맵 양자화 오차가 커진다
        self.declare_parameter('max_range', 6.0)
        # 이보다 큰 잔차는 장애물·사람 등 맵에 없는 물체로 보고 버린다
        self.declare_parameter('reject_m', 1.0)
        self.declare_parameter('report_sec', 2.0)
        # 잔차 공간 분포의 격자 크기(m) 와 보고 주기(s)
        self.declare_parameter('spatial_cell', 0.4)
        self.declare_parameter('spatial_report_sec', 20.0)

        p = lambda n: self.get_parameter(n).value
        map_yaml = p('map_yaml')
        if not map_yaml or not os.path.exists(map_yaml):
            self.get_logger().error(f'map_yaml 을 읽을 수 없다: {map_yaml!r}')
            raise SystemExit(1)
        self.occ, self.meta = _load_occ(map_yaml)
        h, w, res, ox, oy = self.meta
        self.get_logger().info(
            f'맵 {w}x{h}px  해상도 {res}m  원점 ({ox:.2f},{oy:.2f})  '
            f'점유 {int(self.occ.sum())}px')

        self.laser_dx = float(p('laser_offset_x'))
        self.laser_dy = float(p('laser_offset_y'))
        self.fov      = math.radians(float(p('fov_deg')))
        self.beam     = math.radians(float(p('beam_deg')))
        self.rmax     = float(p('max_range'))
        self.reject   = float(p('reject_m'))

        self.pose = None          # (x, y, yaw)
        self.speed = 0.0
        self._acc = []            # (잔차, 실측거리, x, y) 누적 — 짧은 보고용
        self._n_scan = 0
        # ── 잔차의 공간 분포 (긴 누적) ────────────────────────────────
        # ★ 왜 필요한가 (2026-08-05)
        #   맵을 만든 뒤 주행 테스트 중 차가 벽 구조물을 들이받아 실제 벽이
        #   조금씩 옮겨졌다. 그러면 맵과 지형이 다르므로, 필터를 아무리 좋게
        #   만들어도 틀린 답에 정확히 수렴할 뿐이다.
        #   자세 오차라면 모든 빔이 고르게 틀리지만, 벽이 옮겨진 것이라면
        #   **그 벽을 보는 빔만** 크게 틀린다. 잔차를 위치별로 모으면 어느
        #   구조물이 옮겨졌는지 좌표로 찍어낼 수 있다.
        self.cell = float(p('spatial_cell'))
        self._sp_sum = {}         # (i,j) → 잔차 합
        self._sp_n = {}           # (i,j) → 표본 수
        self._sp_report = 0.0

        self.create_subscription(PoseWithCovarianceStamped,
                                 p('pose_topic'), self._pose_cb, 10)
        self.create_subscription(Odometry, p('odom_topic'), self._odom_cb, 10)
        self.create_subscription(LaserScan, p('scan_topic'), self._scan_cb, 10)
        self.res_pub = self.create_publisher(Float32, '/loc/residual', 10)
        self.create_timer(float(p('report_sec')), self._report)

    # ── 입력 ───────────────────────────────────────────────────────────
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _odom_cb(self, msg: Odometry):
        self.speed = abs(msg.twist.twist.linear.x)

    # ── 기대거리 (맵 레이캐스팅) ───────────────────────────────────────
    def _expected(self, lx0, ly0, angs):
        h, w, res, ox, oy = self.meta
        step = res * 0.5
        t = np.arange(0.0, self.rmax + step, step)
        xs = lx0 + np.cos(angs)[:, None] * t[None, :]
        ys = ly0 + np.sin(angs)[:, None] * t[None, :]
        ix = ((xs - ox) / res).astype(np.int32)
        iy = (h - (ys - oy) / res).astype(np.int32)
        inb = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        hit = np.zeros(ix.shape, bool)
        hit[inb] = self.occ[iy[inb], ix[inb]]
        hit |= ~inb
        # 출발점이 벽 안이면(자세가 크게 틀렸거나 벽에 붙음) 그 빔은 버린다
        free = ~hit
        escaped = free.any(axis=1)
        first_free = np.argmax(free, axis=1)
        hit = hit & (np.arange(hit.shape[1])[None, :] > first_free[:, None])
        any_hit = hit.any(axis=1)
        exp = np.where(any_hit, t[np.argmax(hit, axis=1)], np.nan)
        return np.where(escaped, exp, np.nan)

    def _scan_cb(self, msg: LaserScan):
        if self.pose is None:
            return
        n = len(msg.ranges)
        stride = max(1, int(round(self.beam / max(msg.angle_increment, 1e-6))))
        idx = np.arange(0, n, stride)
        a_car = msg.angle_min + idx * msg.angle_increment
        sel = np.abs(a_car) <= self.fov * 0.5
        idx, a_car = idx[sel], a_car[sel]
        if len(idx) == 0:
            return

        meas = np.asarray(msg.ranges, dtype=float)[idx]
        px, py, yaw = self.pose
        cy, sy = math.cos(yaw), math.sin(yaw)
        lx0 = px + cy * self.laser_dx - sy * self.laser_dy
        ly0 = py + sy * self.laser_dx + cy * self.laser_dy
        exp = self._expected(lx0, ly0, yaw + a_car)

        ok = (np.isfinite(meas) & np.isfinite(exp)
              & (meas > msg.range_min) & (meas < min(msg.range_max, self.rmax)))
        if ok.sum() < 10:
            return
        r, e = meas[ok], exp[ok]
        d = np.abs(r - e)
        # 맵에 없는 물체(장애물·사람)는 잔차가 크게 나온다 — 위치 오차와 섞이지
        # 않도록 버린다. 위치 오차만 남기는 게 이 노드의 목적이다.
        good = d < self.reject
        if good.sum() < 10:
            return
        self._n_scan += 1
        for dv, rv in zip(d[good], r[good]):
            self._acc.append((dv, rv, px, py))

        # 빔이 실제로 맞은 지점에 잔차를 쌓는다 — "이 자리의 벽이 맵과 다르다"
        ang = (yaw + a_car)[ok][good]
        ex = lx0 + r[good] * np.cos(ang)
        ey = ly0 + r[good] * np.sin(ang)
        for gx, gy, dv in zip(ex, ey, d[good]):
            key = (int(math.floor(gx / self.cell)), int(math.floor(gy / self.cell)))
            self._sp_sum[key] = self._sp_sum.get(key, 0.0) + float(dv)
            self._sp_n[key] = self._sp_n.get(key, 0) + 1
        self.res_pub.publish(Float32(data=float(d[good].mean())))

    # ── 보고 ───────────────────────────────────────────────────────────
    def _report(self):
        if len(self._acc) < 50:
            return
        a = np.asarray(self._acc)
        d, r = a[:, 0], a[:, 1]
        # 정지 상태의 잔차가 가장 순수한 지표다 — 스캔 왜곡도, 자세 지연도 없다.
        # 그런데도 잔차가 크면 그건 위치추정이 그만큼 틀렸다는 뜻이다.
        tag = '정지' if self.speed < 0.05 else f'v{self.speed:.1f}'
        near = d[r < 2.0]
        far = d[r >= 2.0]
        self.get_logger().info(
            f'[loc:{tag}] 잔차 평균 {d.mean()*100:5.1f}㎝  중앙 {np.median(d)*100:5.1f}㎝  '
            f'90% {np.percentile(d,90)*100:5.1f}㎝  '
            f'근거리<2m {near.mean()*100 if len(near) else float("nan"):4.1f}㎝ / '
            f'원거리 {far.mean()*100 if len(far) else float("nan"):4.1f}㎝  '
            f'빔 {len(d)}/{self._n_scan}스캔')

        # ※ 맵 스케일 판정은 뺐다 — 복도에서 스케일과 자세가 분리되지 않는다.
        #   자세한 검증 결과는 파일 상단 주석 참조. 줄자로 재는 것이 정답이다.
        self._acc.clear()
        self._n_scan = 0

        self._sp_report += float(self.get_parameter('report_sec').value)
        if self._sp_report >= float(self.get_parameter('spatial_report_sec').value):
            self._sp_report = 0.0
            self._report_spatial()

    def _report_spatial(self):
        """잔차가 어디에 몰려 있는지 좌표로 보고한다.

        고르게 퍼져 있으면 자세 오차, 특정 자리에만 몰려 있으면 **그 자리의 벽이
        맵과 다르다** — 구조물이 옮겨졌다는 뜻이다. 어느 쪽인지 숫자로 가른다.

        ★ 검출 한계 (2026-08-05 오프라인 교정, 자세 오차 15cm 를 함께 준 상태)
            벽 정상    비 1.6배  → 자세 오차     (정답)
            벽 10cm 이동 비 1.6배  → 자세 오차     ← **못 잡는다**
            벽 20cm 이동 비 2.2배  → 국소 불일치   좌표도 정확히 지목
            벽 40cm 이동 비 3.6배  → 국소 불일치
          즉 자세 오차와 같은 크기의 이동은 묻힌다. 문턱 2.0 배는 그 경계에서
          거짓양성을 내지 않도록 잡은 값이다.
        """
        keys = [k for k in self._sp_n if self._sp_n[k] >= 20]
        if len(keys) < 8:
            return
        vals = np.array([self._sp_sum[k] / self._sp_n[k] for k in keys])
        order = np.argsort(-vals)

        med = float(np.median(vals))
        top = vals[order[:5]]
        # 상위 자리가 중앙값보다 뚜렷하게 높으면 국소 불일치다.
        ratio = float(top.mean() / max(med, 1e-3))
        self.get_logger().warn(
            f'[map] 잔차 공간분포 {len(keys)}칸: 중앙 {med*100:.1f}㎝ / '
            f'상위5 평균 {top.mean()*100:.1f}㎝ (비 {ratio:.1f}배) — '
            + ('**국소 불일치**: 아래 좌표의 벽이 맵과 다르다 (구조물 이동 의심)'
               if ratio > 2.0 else
               '고르게 퍼져 있다 → 자세 오차 쪽이다'))
        for i in order[:5]:
            k = keys[i]
            self.get_logger().warn(
                f'  ({k[0]*self.cell + self.cell/2:+6.2f},'
                f'{k[1]*self.cell + self.cell/2:+6.2f})  '
                f'잔차 {vals[i]*100:5.1f}㎝  표본 {self._sp_n[k]}')


def main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
