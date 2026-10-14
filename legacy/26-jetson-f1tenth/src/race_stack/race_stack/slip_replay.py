#!/usr/bin/env python3
"""slip_replay — 녹화한 bag 으로 slip_friction 추정을 오프라인 재생한다.

왜 필요한가
──────────
ICP 파라미터(max_scan_m, icp_hz, icp_points, lidar_x)를 CSV 로는 못 고친다.
CSV 에는 이미 추정이 끝난 결과만 남기 때문이다. 원시 /scan 을 다시 돌려야
하고, 그건 차를 굴리지 않고 노트북에서 할 수 있어야 한다.

사용
────
  ros2 run race_stack slip_replay -- <bag_경로> [--icp-hz 10] [--max-scan 10]
                                     [--points 400] [--lidar-x 0.27]

출력: 요약 통계 + <bag>_replay.csv (slip_friction 과 같은 컬럼)
"""
import argparse, csv, math, os, sys
import numpy as np

from race_stack.slip_friction import _icp, _scan_xy, G


def read_bag(path):
    """bag 에서 (t, LaserScan) · (t, odom v) · (t, servo) · (t, imu) 를 뽑는다."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    rdr = rosbag2_py.SequentialReader()
    rdr.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
             rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in rdr.get_all_topics_and_types()}
    out = {k: [] for k in ('scan', 'odom', 'servo', 'imu')}
    key = {'/scan': 'scan', '/odom': 'odom',
           '/commands/servo/position': 'servo', '/sensors/imu/raw': 'imu'}
    while rdr.has_next():
        topic, data, ts = rdr.read_next()
        k = key.get(topic)
        if k is None:
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        out[k].append((ts * 1e-9, msg))
    return out


G = 9.80665


def voser_mu(bag, speed_scale, min_g_turn=0.15, radius=None,
             wheelbase=0.32, servo_gain=-0.761, servo_offset=0.425,
             impact_g=1.5, impact_pad=0.5):
    """정속 원형 주행 bag 에서 노면 마찰계수 μ 를 두 가지로 뽑는다.

    왜 두 가지인가 (2026-08-09)
    ──────────────────────────
    μ = max(횡가속)/g 인데, "횡가속"을 무엇으로 재느냐가 갈린다.

      μ_a = max|a_y,imu| / g        ← IMU 가속도계
      μ_k = max(v_true·ω) / g       ← 오도메트리 × 자이로

    a_y 는 **차체 롤에 오염된다**. 롤각 φ 만큼 기울면 가속도계는
    a_lat·cos φ + g·sin φ 를 읽는다 — 6축 IMU 는 횡가속과 중력의 기울기
    성분을 원리적으로 구분할 수 없다(랭크 부족). 실측에서 a_y 는 v·ω 보다
    12.6% 컸고(R²=0.94~0.97, 절편 0 — 노이즈가 아니라 순수 스케일 편향),
    선회 중 롤레이트 자이로가 9~10°/s 를 찍었다. 롤 기울기로 환산하면
    7~10°/g 로, 오일 쇼크가 살아있는 RC 카에서 정상 범위다.

    반면 μ_k 는 롤과 무관하다. ω 는 자이로와 LiDAR ICP 두 독립 센서가
    기울기 1.01~1.06 으로 일치해 검증됐고, v 는 줄자 6회로 캘리브레이션했다.
    **μ_k 를 기준으로 쓰고 μ_a 는 상한으로 본다.** 두 값의 차이가 곧 롤의
    크기이므로, 쇼크를 묶는 별도 실험 없이 여기서 진단이 같이 나온다.

    가속도계 스케일은 정지 구간의 중력 크기로 보정한다 (실측 1.036g).
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    rdr = rosbag2_py.SequentialReader()
    rdr.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
             rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in rdr.get_all_topics_and_types()}
    I, O, S = [], [], []
    while rdr.has_next():
        tp, data, ts = rdr.read_next()
        if tp == '/sensors/imu/raw':
            m = deserialize_message(data, get_message(types[tp]))
            I.append([ts * 1e-9, m.linear_acceleration.x, m.linear_acceleration.y,
                      m.linear_acceleration.z, m.angular_velocity.z])
        elif tp == '/odom':
            m = deserialize_message(data, get_message(types[tp]))
            O.append([ts * 1e-9, m.twist.twist.linear.x])
        elif tp == '/commands/servo/position':
            m = deserialize_message(data, get_message(types[tp]))
            S.append([ts * 1e-9, m.data])
    if len(I) < 200 or len(O) < 50:
        print('★ IMU/odom 표본이 부족하다');  return
    S = np.array(S) if S else None
    I = np.array(I);  O = np.array(O)
    ti = I[:, 0]

    # ── 정지 구간으로 스케일·바이어스 (첫 3초) ──────────────────────
    hz = 1.0 / max(np.median(np.diff(ti)), 1e-6)
    n0 = int(min(3.0 * hz, len(I) // 4))
    scale = float(np.median(np.linalg.norm(I[:n0, 1:4], axis=1)))
    bx = float(np.median(I[:n0, 1]));  by = float(np.median(I[:n0, 2]))

    # 축 매핑은 slip_friction 과 같다: a_y = +imu.x, a_x = −imu.y
    ay = (I[:, 1] - bx) / scale          # [g]
    ax = -(I[:, 2] - by) / scale         # [g]
    wz = np.radians(I[:, 4])
    v_true = np.interp(ti, O[:, 0], O[:, 1]) / speed_scale

    print(f'\n{"="*64}\nVoser μ 측정')
    print(f'  IMU {len(I)}샘플 @ {hz:.0f}Hz   가속도계 정지 스케일 {scale:.4f}g '
          f'({100*(scale-1):+.1f}% 보정)')

    # ── 준정상상태 필터 ─────────────────────────────────────────────
    # ★ 이게 없으면 안 된다 (2026-08-09). μ 는 "지속 가능한 최대 횡가속"이라
    #   과도구간을 빼야 한다. 안 거르고 max|a_y| 와 max(v·ω) 를 각각 구해
    #   비를 내면 **서로 다른 시점의 값**을 비교하게 되어, 실제 1.126 인
    #   비율이 1.6~1.8 로 부풀고 롤각이 17° 같은 불가능한 값으로 나온다.
    # ★ 미분은 반드시 **구간 차분**으로 한다 (2026-08-09). 50Hz 신호를 1샘플
    #   차분하면 자이로 노이즈 1°/s 가 50°/s² 로 증폭돼 필터가 전부를 걸러낸다
    #   (실제로 원 주행 bag 이 2행만 남았다). 0.1초 창으로 넓히면 노이즈가
    #   1/5 로 줄고 우리가 보려는 과도(수백 ms)는 그대로 잡힌다.
    win = max(int(round(0.1 * hz)), 1)

    def rate(x):
        r = np.full_like(x, np.inf)
        if len(x) > win:
            dtv = ti[win:] - ti[:-win]
            ok = dtv > 1e-4
            r[win:][ok] = np.abs((x[win:] - x[:-win])[ok] / dtv[ok])
        return r
    qss = (rate(v_true) < 0.5) & (rate(wz) < math.radians(60))

    # ── 충격 구간 제외 ────────────────────────────────────────────────
    # 벽·연석 충돌은 IMU 에 수 g~십수 g 스파이크를 남긴다. 그 하나가 μ 의
    # 최대값·백분위를 통째로 오염시킨다 (2026-08-08 bag: 17.2g 관측).
    # 충격 전후로 차가 튕기므로 앞뒤 여유까지 함께 버린다.
    mag = np.linalg.norm(I[:, 1:4], axis=1) / scale
    hit = mag > impact_g
    if hit.any():
        pad = max(int(impact_pad * hz), 1)
        widened = np.convolve(hit.astype(float), np.ones(2 * pad + 1), 'same') > 0
    else:
        widened = np.zeros(len(I), bool)

    # ── 오버스티어(스핀) 구간 제외 ────────────────────────────────────
    # 실제 요레이트가 기구학값을 크게 웃돌면 뒤가 흐른 것이다. 과도현상이라
    # 정상상태 가정이 깨지고, 스핀 중 요레이트가 커서 μ 를 부풀린다.
    over = np.zeros(len(I), bool)
    if S is not None and len(S) > 10:
        sv = np.clip(np.interp(ti, S[:, 0], S[:, 1]),
                     min(0.15, 0.85), max(0.15, 0.85))
        delta = (sv - servo_offset) / servo_gain
        w_kin = v_true * np.tan(delta) / wheelbase
        over = np.abs(wz) > 1.5 * np.abs(w_kin) + math.radians(20)

    clean = ~widened & ~over
    turn = qss & clean & (np.abs(v_true * wz) / G > min_g_turn) & (v_true > 0.5)
    print(f'  제외: 충격 {int(widened.sum())}행 (최대 {mag.max():.1f}g)  '
          f'오버스티어 {int(over.sum())}행')
    if turn.sum() < 50:
        print(f'  ★ 준정상 선회 구간이 {turn.sum()}행뿐 — 정속 원형 주행이 맞는지,'
              f'\n    또는 속도를 더 올려야 하는지 확인할 것')
        return

    # ── 휠 스핀 진단 ─────────────────────────────────────────────────
    # 구동륜이 헛돌면 v_wheel 이 부풀고 μ_k = v·ω/g 가 같이 부푼다. 한계
    # 근처에서 정확히 문제가 되므로 반드시 확인해야 한다. IMU 종가속과
    # 휠속도 미분을 비교하면 잡힌다 — 스핀이 없으면 기울기가 1 이어야 한다.
    ax_ms2 = ax * G
    dv = np.full_like(v_true, np.nan)
    dv[win:] = (v_true[win:] - v_true[:-win]) / (ti[win:] - ti[:-win])
    ms = np.isfinite(dv) & (np.abs(dv) > 0.5) & (v_true > 0.5) & clean
    if ms.sum() > 100:
        sl_a = float(np.polyfit(dv[ms], ax_ms2[ms], 1)[0])
        print(f'  휠 스핀 검사: a_x = {sl_a:.3f}·(dv_wheel/dt)  n={ms.sum()}')
        if sl_a < 0.85:
            print(f'    ⚠ 휠이 차보다 {1/max(sl_a,1e-3):.2f}배 빨리 돈다 = 스핀.')
            print(f'      v_wheel 이 부풀어 있으므로 μ_k 도 그만큼 **과대평가**다.')
            print(f'      스로틀을 더 부드럽게 주고 정속을 유지할 것.')

    mu_a = np.abs(ay)
    mu_k = np.abs(v_true * wz) / G
    rho  = np.hypot(ax, ay)              # 마찰원 (종+횡)

    # ── 휠·롤 둘 다 무관한 μ (반경을 실측했을 때) ─────────────────────
    # 정상 원운동이면 a_lat = R·ω² 다. 자이로만 쓰므로
    #   · 휠 스핀 무관 (v_wheel 을 안 씀)
    #   · 롤 무관 (가속도계를 안 씀)
    # 우리 센서 조합에서 얻을 수 있는 가장 깨끗한 횡가속이다.
    # 전제: 차가 그 반경의 원 위에 실제로 있어야 한다 — 언더스티어로 밀려
    # 나가면 R 은 표시한 원이 아니라 **실제로 그린 원**이어야 한다.
    mu_r = None
    if radius:
        mu_r = radius * wz ** 2 / G
        v_r  = radius * np.abs(wz)      # 속도도 휠 없이 나온다

    print(f'\n  속도 구간별 (준정상 선회 {turn.sum()}행 / 전체 {len(I)})')
    print(f'  {"v_true[㎧]":>11} | {"n":>5} | {"μ_k 최대":>9} | {"μ_a 최대":>9} | {"ρ 최대":>8}')
    print('  ' + '-' * 56)
    edges = np.arange(0.5, max(v_true[turn].max(), 1.0) + 0.5, 0.5)
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = turn & (v_true >= lo) & (v_true < hi)
        if m.sum() < 20:
            continue
        print(f'  {lo:4.1f}~{hi:4.1f}   | {m.sum():5d} | {mu_k[m].max():9.3f} | '
              f'{mu_a[m].max():9.3f} | {rho[m].max():8.3f}')

    ka, kk = mu_a[turn], mu_k[turn]
    p99a, p99k = np.percentile(ka, 99), np.percentile(kk, 99)
    print(f'\n  μ_k (롤 무관, 휠 의존)  최대 {kk.max():.3f}   99%p {p99k:.3f}')
    print(f'  μ_a (롤 오염, 상한)     최대 {ka.max():.3f}   99%p {p99a:.3f}')
    if mu_r is not None:
        kr = mu_r[turn]
        print(f'  ★ μ_r = R·ω²/g (R={radius:.2f}m, 휠·롤 모두 무관)'
              f'   최대 {kr.max():.3f}   99%p {np.percentile(kr,99):.3f}')
        print(f'    실제로 그린 원의 반경을 넣었다면 이 값이 가장 믿을 만하다.')
    print(f'  마찰원 ρ 최대 {rho[turn].max():.3f}   99%p {np.percentile(rho[turn],99):.3f}')
    # ── a_y 대 v·ω 는 **회귀 기울기**로 본다 (최대값 비가 아니라) ─────
    # 둘 다 **g 단위**로 맞춘다 — ay 는 이미 g, v·ω 는 ㎧² 라 나눠야 한다.
    # (안 맞추면 기울기가 1/9.81 배로 나와 1.13 이 0.115 로 보인다)
    cx, cy = (v_true * wz)[turn] / G, ay[turn]
    slope = float(np.polyfit(cx, cy, 1)[0])
    r2 = float(np.corrcoef(cx, cy)[0, 1] ** 2)
    print(f'\n  a_y = {slope:.3f}·(v·ω)   R²={r2:.3f}')
    if r2 > 0.5 and slope > 1.02:
        # 롤이면 a_y = a_lat(1 + g·k_roll) 꼴의 순수 스케일로 나타난다
        print(f'    → 초과분 {100*(slope-1):+.1f}%. 롤이라면 롤 기울기 '
              f'{math.degrees(slope-1):.1f}°/g 에 해당한다 (RC 카 정상범위 7~10°/g).')
    elif r2 <= 0.5:
        print(f'    → R² 가 낮다. 준정상 구간이 부족하거나 데이터가 흔들린다.')

    print(f'\n  ★ vehicle_sim.py / map_controller 에는 **μ_k 99%p = {p99k:.4f}** 를 쓸 것.')
    print(f'    최대값 대신 99%p 를 쓰는 이유: 단일 스파이크(충격·이상치)로')
    print(f'    μ 가 통째로 오염되는 것을 막기 위해서다.')

    # ── 한계에 실제로 닿았는가 ────────────────────────────────────────
    # μ 는 "미끄러지기 직전"의 값이라야 의미가 있다. 속도를 올리는데도
    # μ_k 가 계속 커지고 있으면 아직 포화 전이고, 그 값은 **하한**일 뿐이다.
    lastbins = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = turn & (v_true >= lo) & (v_true < hi)
        if m.sum() >= 20:
            lastbins.append(np.percentile(mu_k[m], 99))
    if len(lastbins) >= 3 and lastbins[-1] > lastbins[-3] * 1.05:
        print(f'\n  ⚠ 속도를 올릴수록 μ_k 가 계속 증가하고 있다 '
              f'({lastbins[-3]:.3f} → {lastbins[-1]:.3f}).')
        print(f'    아직 마찰한계에 안 닿았다는 뜻이므로 이 값은 **하한**이다.')
        print(f'    실제로 미끄러질 때까지 속도를 더 올려야 μ 가 나온다.')


def _calib_lever(vw, vy, wz, max_g):
    """저속 정상원 주행에서 lidar_x 를 역산한다.

    원리: 횡가속이 충분히 작으면 타이어 슬립이 무시할 만해서 후축의 횡속도가
    0 이다. 그러면 LiDAR 에서 재는 횡속도는 순수 레버암 성분뿐이다:
        v_y_laser = ω · x_laser      →  x_laser = v_y_laser / ω

    ★ 반드시 저속이어야 한다. 미끄러지기 시작하면 횡속도 0 인 점이 후축 뒤로
      밀려 레버암이 실제보다 크게 나온다 — 2026-08-07 일반 주행 로그에서
      0.39m 로 나온 게 그 경우다(TF 선언값은 0.27m).
    """
    g = 9.80665
    alat = np.abs(vw * wz) / g                    # 구심가속도 [g]
    m = (alat < max_g) & (np.abs(wz) > math.radians(15)) & (vw > 0.5)
    print(f'\n── 레버암 캘리브레이션 (횡가속 < {max_g}g, |ω| > 15°/s) ──')
    print(f'  해당 구간 {m.sum()}행 / {len(vw)}'
          f'   횡가속 중앙 {np.median(alat[m]) if m.sum() else float("nan"):.3f}g')
    if m.sum() < 50:
        print('  ★ 표본 부족 — 더 크고 느린 원으로, 더 오래 돌아야 한다.')
        print('    (속도 1.0~1.5㎧ / 반경 2~3m 로 2~3바퀴, 좌·우 각각)')
        return

    for nm, mm in (('좌회전 ω>0', m & (wz > 0)),
                   ('우회전 ω<0', m & (wz < 0)),
                   ('전체',       m)):
        if mm.sum() < 20:
            print(f'  {nm:12s} 표본 부족 ({mm.sum()}행)')
            continue
        ratio = vy[mm] / wz[mm]
        slope = np.polyfit(wz[mm], vy[mm], 1)[0]
        print(f'  {nm:12s} n={mm.sum():4d}  비율중앙 {np.median(ratio):+.3f}m  '
              f'회귀기울기 {slope:+.3f}m  (표준편차 {np.std(ratio):.3f})')
    print('  → 좌/우 값이 비슷하면 신뢰할 만하다. 크게 갈리면 편향이 남은 것.')
    print(f'  → race_script.yaml 의 slip.lidar_x 에 넣을 것 (TF 선언값 0.27m)')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--icp-hz',   type=float, default=10.0)
    ap.add_argument('--max-scan', type=float, default=10.0)
    ap.add_argument('--points',   type=int,   default=400)
    ap.add_argument('--lidar-x',  type=float, default=0.27)
    ap.add_argument('--lpf-hz',   type=float, default=2.0)
    ap.add_argument('--wheelbase',type=float, default=0.33)
    ap.add_argument('--servo-gain',   type=float, default=-0.761)
    ap.add_argument('--servo-offset', type=float, default=0.425)
    ap.add_argument('--servo-min',type=float, default=0.15)
    ap.add_argument('--servo-max',type=float, default=0.85)
    # 2026-08-09 vesc.yaml 수정(4614→4203) 이후 표시=실제 이므로 1.0.
    # ★ 그 이전에 녹화한 bag 은 --speed-scale 0.911 을 줘야 한다.
    ap.add_argument('--speed-scale', type=float, default=1.0,
                    help='표시속도/실제속도. odom_scale 로 잰 값')
    # ★ 디스큐 기본 OFF (2026-08-09 실측). 물리적으로는 옳지만 이 데이터에서
    #   측정 가능한 이득이 없었다: |β| 중앙 2.5°→2.4°, 저요레이트 95%p 는
    #   14.8°→16.4° 로 오히려 악화. 이유는 두 스캔이 같은 방식으로 찌그러지면
    #   ICP 가 찾는 **상대** 변환에서 공통 성분이 상쇄되기 때문이다. 절대
    #   왜곡(2.5㎧×25ms=6.3cm)이 아니라 스캔 간 왜곡 **차이**만 오차가 되는데,
    #   등속 주행에서는 그게 작다. 급가감속·급조향이 많은 데이터에서는
    #   달라질 수 있으므로 옵션은 남긴다.
    ap.add_argument('--deskew', dest='deskew', action='store_true', default=False,
                    help='모션 디스큐 (기본 꺼짐 — 위 주석 참고)')
    ap.add_argument('--no-deskew', dest='deskew', action='store_false')
    ap.add_argument('--voser-mu', action='store_true',
                    help='정속 원형 주행 bag 에서 μ 를 뽑는다 (ICP 재생 없이 즉시)')
    ap.add_argument('--radius', type=float, default=None,
                    help='실제로 그린 원의 반경 [m]. 주면 μ = R·ω²/g 로도 계산한다 '
                         '— 자이로만 쓰므로 휠 스핀·차체 롤 둘 다 무관하다.')
    ap.add_argument('--impact-g', type=float, default=1.5,
                    help='이 값을 넘는 |a| 는 충돌로 보고 전후 구간까지 버린다 [g]')
    ap.add_argument('--calib-lever', action='store_true',
                    help='저속 원 주행 bag 에서 lidar_x 를 역산한다')
    ap.add_argument('--calib-max-g', type=float, default=0.15,
                    help='캘리브레이션에 쓸 최대 횡가속 [g]. 이보다 크면 슬립이 '
                         '섞여 레버암이 부풀어 보인다')
    a = ap.parse_args([x for x in (argv or sys.argv[1:]) if x != '--'])

    # μ 측정은 IMU·odom 만 있으면 되므로 ICP 재생(느림) 없이 바로 끝낸다.
    if a.voser_mu:
        voser_mu(a.bag, a.speed_scale, radius=a.radius,
                 wheelbase=a.wheelbase, servo_gain=a.servo_gain,
                 servo_offset=a.servo_offset, impact_g=a.impact_g)
        return 0

    b = read_bag(a.bag)
    if not b['scan']:
        print('★ /scan 이 bag 에 없다'); return 1
    print(f'스캔 {len(b["scan"])}개  odom {len(b["odom"])}  servo {len(b["servo"])}')

    # 최근접 시각으로 보간할 준비
    def sampler(seq, fn):
        ts = np.array([t for t, _ in seq]);  vs = np.array([fn(m) for _, m in seq])
        return lambda q: float(vs[np.searchsorted(ts, q).clip(0, len(vs) - 1)])
    v_of = sampler(b['odom'],  lambda m: m.twist.twist.linear.x)
    s_of = sampler(b['servo'], lambda m: m.data)
    g_of = sampler(b['imu'],   lambda m: m.angular_velocity.z)

    dt_min = 1.0 / max(a.icp_hz, 1.0)
    prev = prev_t = None
    vx = vy = wz = 0.0
    icp_ok = False
    rows = []

    for t, msg in b['scan']:
        # 디스큐는 **점군을 만들 때** 걸어야 한다 — ω 는 IMU 자이로, v 는 휠.
        # /odom 의 각속도는 서보 지령 기반 기구학값이라 쓰면 안 된다.
        gy_here = math.radians(g_of(t)) if a.deskew else 0.0
        pts = _scan_xy(msg, a.max_scan,
                       w=gy_here, v=v_of(t) / a.speed_scale if a.deskew else 0.0)
        if prev is None:
            prev, prev_t = pts, t;  continue
        dt = t - prev_t
        if dt < dt_min:
            continue
        if not (0.005 < dt < 0.5 and len(pts) >= 20 and len(prev) >= 20):
            prev, prev_t = pts, t;  continue

        vw = v_of(t)
        sv = min(max(s_of(t), a.servo_min), a.servo_max)
        dl = (sv - a.servo_offset) / a.servo_gain
        gy = math.radians(g_of(t))          # IMU 자이로 (deg/s 가정)

        cands = [(0.0, 0.0, 0.0), (vw * dt, 0.0, gy * dt)]
        if icp_ok:
            cands.append((vx * dt, vy * dt, wz * dt))
        best = None
        for c in cands:
            r = _icp(prev, pts, init=c, n_src=a.points)
            if r[3] and (best is None or r[4] < best[4]):
                best = r
        if best:
            icp_ok = True
            al = dt / (dt + 1.0 / (2 * math.pi * max(a.lpf_hz, 0.1)))
            vx += al * (best[0] / dt - vx)
            vy += al * (best[1] / dt - vy)
            wz += al * (best[2] / dt - wz)

        vyb = vy - a.lidar_x * wz
        rows.append((t, vw, dl, gy, vx, vy, vyb, wz,
                     vw * math.tan(dl) / a.wheelbase))
        prev, prev_t = pts, t

    d = np.array(rows)
    if len(d) < 10:
        print('★ 유효 행이 너무 적다'); return 1
    t, vw, dl, gy, vx, vy, vyb, wz, wk = d.T
    m = vw > 1.0
    beta_raw = np.degrees(np.arctan2(vy[m],  vw[m]))
    beta     = np.degrees(np.arctan2(vyb[m], vw[m]))
    lo = m & (np.abs(wz) < math.radians(10))

    print(f'\n재생 {len(d)}행  ({a.icp_hz}Hz, max_scan {a.max_scan}m, '
          f'{a.points}pt, lidar_x {a.lidar_x}m)')
    print(f'  ω_lidar vs ω_imu   corr={np.corrcoef(wz, gy)[0,1]:+.3f}  '
          f'기울기={np.polyfit(wz, gy, 1)[0]:+.3f}')
    print(f'  |β| 중앙값  레버암 보정 전 {np.median(abs(beta_raw)):5.1f}°  '
          f'→ 보정 후 {np.median(abs(beta)):5.1f}°')
    if lo.sum() > 30:
        bl = np.degrees(np.arctan2(vyb[lo], vw[lo]))
        print(f'  저요레이트(|ω|<10°/s) n={lo.sum()}  |β| 중앙값 {np.median(abs(bl)):.1f}°  '
              f'95%p {np.percentile(abs(bl),95):.1f}°   ← 0 에 가까워야')

    if a.calib_lever:
        _calib_lever(vw, vy, wz, a.calib_max_g)

    out = os.path.join(os.path.dirname(os.path.abspath(a.bag)),
                       os.path.basename(a.bag.rstrip('/')) + '_replay.csv')
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'v_wheel', 'delta', 'wz_imu', 'vx_lidar',
                    'vy_lidar', 'vy_base', 'wz_lidar', 'wz_kin'])
        w.writerows(np.round(d, 5))
    print(f'  저장: {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
