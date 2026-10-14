#!/usr/bin/env python3
"""odom_scale — 줄자로 잰 실제 거리와 /odom 거리를 비교해 속도 스케일을 잡는다.

왜 필요한가
──────────
2026-08-07 실측에서 정상 선회 중 a_y 가 v_wheel·ω 보다 28% 컸다. ω 는 IMU
자이로와 LiDAR ICP 가 서로 일치했고(기울기 1.01~1.06), 가속도계는 정지 시
중력이 1.036g 로 나와 4% 이내였다. 소거법으로 v_wheel(VESC 오도메트리)이
약 24% 낮게 읽힌다는 결론이 나왔다.

이 도구는 그 결론을 **IMU·LiDAR 와 완전히 무관하게** 검증한다 — 줄자와
/odom 만 쓴다. 추론 사슬이 틀렸다면 여기서 1.0 이 나올 것이다.

사용
────
  터미널 A:  ./race.sh joy          (차량 브링업 — /odom 이 나와야 한다)
  터미널 B:  ros2 run race_stack odom_scale

  ① 출발 지점에서 ENTER (바닥에 기준점 표시)
  ② 직진 주행 후 **완전히 정지**
  ③ ENTER  →  그 자리에서 줄자로 실제 거리를 재서 입력

  거리를 미리 정하지 않는다. 도착선을 정확히 통과시킬 필요가 없고, 차가
  멈춘 뒤에 재므로 ENTER 타이밍 오차가 안 들어간다.
  여러 번 반복하면 중앙값과 산포를 같이 낸다. q + ENTER 로 종료.

  ※ 손으로 밀어도 된다(구동 슬립이 없어 오히려 깨끗하다). 다만 VESC 는
    센서리스라 아주 느리면 erpm 을 놓친다 — 걷는 속도 정도는 유지할 것.
  ※ 줄자는 차체의 **같은 지점**을 기준으로 잰다(예: 앞바퀴 접지점).
    출발 때 바닥에 표시해 두면 편하다.
"""
import argparse, math, os, sys, threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def _read_gain():
    """vesc.yaml 의 현재 speed_to_erpm_gain. 못 읽으면 None."""
    import yaml
    repo = os.path.expanduser(os.environ.get('RACE_REPO', '~/yax'))
    for p in (os.path.join(repo, 'config', 'vesc.yaml'),
              os.path.expanduser('~/f1tenth_ws/src/f1tenth_stack/config/vesc.yaml')):
        try:
            v = yaml.safe_load(open(p))['/**']['ros__parameters']
            return float(v['speed_to_erpm_gain']), p
        except Exception:
            continue
    return None, None


class OdomScale(Node):
    def __init__(self):
        super().__init__('odom_scale')
        self.reset()
        self.runs = []
        self.create_subscription(Odometry, '/odom', self._cb, 20)

    def reset(self):
        self.active = False
        self.s_int = 0.0        # ∫|v| dt  — 속도 적분 거리
        self.x0 = self.y0 = None
        self.x = self.y = 0.0
        self.t_prev = None
        self.t0 = None
        self.vmax = 0.0

    def _cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        if not self.active:
            self.t_prev = t
            return
        v = float(msg.twist.twist.linear.x)
        if self.t_prev is not None:
            dt = t - self.t_prev
            if 1e-4 < dt < 0.5:
                self.s_int += abs(v) * dt
        self.t_prev = t
        self.vmax = max(self.vmax, abs(v))
        if self.x0 is None:
            self.x0, self.y0, self.t0 = p.x, p.y, t

    def start(self):
        self.reset()
        self.active = True
        self.x0 = self.y0 = None

    def stop(self):
        """측정을 끝내고 결과를 돌려준다. 실제 거리는 나중에 채운다."""
        self.active = False
        if self.x0 is None:
            return None
        s_pose = math.hypot(self.x - self.x0, self.y - self.y0)
        dur = (self.t_prev - self.t0) if self.t0 else 0.0
        return dict(s_int=self.s_int, s_pose=s_pose, dur=dur, vmax=self.vmax)


def main(argv=None):
    ap = argparse.ArgumentParser()
    a = ap.parse_args([x for x in (argv or sys.argv[1:]) if x != '--'])

    gain, gpath = _read_gain()
    rclpy.init()
    node = OdomScale()
    th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    th.start()

    print()
    if gain:
        print(f'현재 speed_to_erpm_gain = {gain:.1f}   ({gpath})')
    print('출발점에서 ENTER → 주행 후 정지 → ENTER → 줄자로 잰 거리 입력.')
    print('q + ENTER 로 종료.\n')

    try:
        while True:
            if input(f'  [{len(node.runs)+1}회] 출발점에서 ENTER: ').strip().lower() == 'q':
                break
            node.start()
            print('    ... 측정 중 — 주행 후 완전히 정지하고 ENTER')
            if input('  [정지 완료] ENTER: ').strip().lower() == 'q':
                break
            r = node.stop()
            if not r or r['s_int'] < 0.2:
                print('    ★ 이동이 감지되지 않았다. /odom 이 나오는지 확인할 것.\n')
                continue
            print(f'    odom:  ∫|v|dt = {r["s_int"]:6.2f} m   pose 변위 = {r["s_pose"]:6.2f} m'
                  f'   {r["dur"]:.1f}초  최고 {r["vmax"]:.2f}㎧')

            # 이제 줄자로 잰다 — 차는 멈춰 있으므로 서두를 필요 없다
            while True:
                s = input('    줄자 실제 거리 [m] (x=이번 회차 버림): ').strip().lower()
                if s == 'x':
                    r = None;  break
                if s == 'q':
                    r = None;  break
                try:
                    td = float(s)
                    if td <= 0.05:
                        print('      ★ 값이 너무 작다. 다시 입력.');  continue
                    r['true'] = td;  break
                except ValueError:
                    print('      ★ 숫자로 입력할 것 (예: 9.85)')
            if r is None:
                print('    (버림)\n');  continue

            node.runs.append(r)
            k_int, k_pose = r['s_int'] / r['true'], r['s_pose'] / r['true']
            print(f'    odom/실제 = {k_int:.3f} (속도적분)  {k_pose:.3f} (pose)')
            if gain:
                print(f'    → speed_to_erpm_gain 제안 {gain*k_int:.0f}  (현재 {gain:.0f})\n')
            else:
                print()
    except (EOFError, KeyboardInterrupt):
        pass

    rs = node.runs
    if rs:
        import statistics as st
        ki = [r['s_int'] / r['true'] for r in rs]
        kp = [r['s_pose'] / r['true'] for r in rs]
        print(f'\n{"="*58}\n{len(rs)}회 측정 요약')
        for i, r in enumerate(rs, 1):
            print(f'   {i}회: 실제 {r["true"]:6.2f}m  odom {r["s_int"]:6.2f}m  '
                  f'비율 {r["s_int"]/r["true"]:.3f}')
        print(f'  odom/실제  속도적분 {st.median(ki):.3f}'
              + (f' (표준편차 {st.stdev(ki):.3f})' if len(ki) > 1 else ''))
        print(f'             pose    {st.median(kp):.3f}'
              + (f' (표준편차 {st.stdev(kp):.3f})' if len(kp) > 1 else ''))
        k = st.median(ki)
        # 두 표현이 헷갈리기 쉬워 둘 다 적는다 (19.5% 낮음 == 1.242배 높음)
        print(f'\n  표시 속도는 실제의 {k*100:.1f}%'
              f'  ⇔  실제 속도 = 표시 × {1/k:.3f}')
        if gain:
            print(f'  speed_to_erpm_gain:  {gain:.0f}  →  {gain*k:.0f}')
            print(f'    {gpath} 를 고치고 f1tenth_stack 재빌드/재링크할 것.')
        print(f'\n  ※ 1.0 에 가깝게 나왔다면 오도메트리는 정상이고, a_y 대 v·ω')
        print(f'    불일치의 원인은 다른 곳에 있다는 뜻이다.')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
