#!/usr/bin/env python3
"""
set_speed_kappa.py — 기존 웨이포인트 경로(x,y)는 그대로 두고 speed 컬럼만
곡률 기반으로 다시 계산한다.

곡선 진입부 실측(curvature_probe, 08/07)에 따르면 이 차량은 풀락 조향에서
v=2.0~2.25㎧대에 실제 횡가속도가 약 3.7 m/s²에서 포화된다(그 전까지는
슬립보다 조향 기하 자체가 병목). 기존 gen_waypoints_kappa.py 의
A_LAT_MAX=0.9g(8.83)는 이 실측보다 훨씬 낙관적이라 곡선 감속이 부족했다.

속도 = clip(sqrt(A_LAT_MAX / kappa), V_MIN, V_MAX) 로 곡률별 상한을 잡고,
직선→곡선 전환에서 급감속이 불가능하니 전진(가속 한계)·후진(제동 한계)
2-패스로 스무딩한다 — 곡선 시작점 앞에서부터 미리 감속.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
"""

import argparse
import csv
import os

import numpy as np

# ── 실측 기반 물리 파라미터 ──────────────────────────────────────────────
# A_LAT_MAX: curvature_probe 08/07 스윕 상위 구간(v=2.00~2.25㎧)에서
#   a_lat = v^2 * kappa_achieved 로 역산한 평균 (3.66, 3.75 → 3.70).
#   0.9g(8.83) 대비 훨씬 보수적 — 실차가 실제로 버티는 값.
A_LAT_MAX = 3.7
A_LON_ACC = 5.0
A_LON_BRK = 8.0
V_MAX     = 6.0   # 직선 구간 상한
V_MIN     = 0.5   # race_script.yaml speed.min_speed 와 통일


def _curvature(x, y):
    dx, dy = np.gradient(x), np.gradient(y)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    return (dx * ddy - dy * ddx) / np.clip((dx**2 + dy**2)**1.5, 1e-9, None)


def load_kappa_table(path):
    """curvature_probe 실측(0.05㎧ 보간) v,kappa_max_interp 표를 읽어
    속도 오름차순 (v_arr, k_arr) 반환. 앞부분(v<=peak)의 노이즈성 비단조
    구간은 뒤에서부터 running-max 로 감싸 '속도 오르면 achievable κ는
    내려간다'는 단조 감소 곡선으로 정리한다 — 역산(κ→v)에 필요."""
    with open(path) as f:
        lines = [l for l in f if not l.startswith('#')]
    rows = list(csv.DictReader(lines))
    v_arr = np.array([float(r['v']) for r in rows])
    k_arr = np.array([float(r['kappa_max_interp']) for r in rows])
    order = np.argsort(v_arr)
    v_arr, k_arr = v_arr[order], k_arr[order]
    k_mono = k_arr.copy()
    for i in range(len(k_mono) - 2, -1, -1):
        k_mono[i] = max(k_mono[i], k_mono[i + 1])
    return v_arr, k_mono


def v_lim_from_table(kappa, v_arr, k_mono, v_straight_max):
    """완만한 구간(κ <= 표에서 가장 빠른 속도로도 achievable 한 κ)은
    최고속도로 두고, 그보다 급한 코너만 표를 역산해서 속도를 잡는다."""
    kappa_gentle_thresh = k_mono[-1]  # 표의 가장 빠른 속도(2.25)에서 achieved κ
    v_lim = np.where(
        kappa <= kappa_gentle_thresh,
        v_straight_max,
        np.interp(kappa, k_mono[::-1], v_arr[::-1]),  # np.interp 은 xp 오름차순 필요
    )
    return v_lim


def speed_profile(x, y, v_max=V_MAX, v_min=V_MIN, a_lat_max=A_LAT_MAX,
                   a_lon_acc=A_LON_ACC, a_lon_brk=A_LON_BRK, kappa_table=None):
    n = len(x)
    dx = np.diff(np.append(x, x[0]))
    dy = np.diff(np.append(y, y[0]))
    ds = np.clip(np.hypot(dx, dy), 1e-6, None)
    kappa = np.abs(_curvature(x, y))

    if kappa_table is not None:
        v_arr, k_mono = kappa_table
        v_lim = np.clip(v_lim_from_table(kappa, v_arr, k_mono, v_max), v_min, v_max)
        print(f'  [테이블 기반] 완만(κ<={k_mono[-1]:.3f}) → {v_max}㎧, '
              f'급코너는 실측 보간표 역산')
    else:
        v_lim = np.clip(np.sqrt(a_lat_max / np.clip(kappa, 0.01, None)), v_min, v_max)
    v = v_lim.copy()
    for _ in range(3):
        for i in range(n):
            v[i] = min(v[i], np.sqrt(v[(i - 1) % n]**2 + 2 * a_lon_acc * ds[(i - 1) % n]))
    for _ in range(3):
        for i in range(n - 1, -1, -1):
            v[i] = min(v[i], np.sqrt(v[(i + 1) % n]**2 + 2 * a_lon_brk * ds[i]))
    v = np.clip(v, v_min, v_max)

    total = ds.sum()
    avg = total / (ds / np.clip(v, 1e-3, None)).sum()
    print(f'속도: {v.min():.2f}~{v.max():.2f} m/s  랩타임 ~{total / avg:.1f}s  (트랙 {total:.1f}m)')
    print(f'κ: {kappa.min():.3f}~{kappa.max():.3f} 1/m  (A_LAT_MAX={a_lat_max})')
    return v, kappa


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('input', help='x,y[,speed] 웨이포인트 csv')
    p.add_argument('-o', '--output', help='출력 경로 (기본: 입력 덮어씀)')
    p.add_argument('--v-max', type=float, default=V_MAX)
    p.add_argument('--v-min', type=float, default=V_MIN)
    p.add_argument('--a-lat-max', type=float, default=A_LAT_MAX)
    p.add_argument('--table', help='v,kappa_max_interp 보간표 csv — 주면 공식 대신 이걸로 급코너 속도 역산')
    args = p.parse_args()

    with open(args.input) as f:
        rows = list(csv.reader(f))
    header, data = rows[0], rows[1:]
    x = np.array([float(r[0]) for r in data])
    y = np.array([float(r[1]) for r in data])

    kappa_table = load_kappa_table(args.table) if args.table else None
    v, kappa = speed_profile(x, y, v_max=args.v_max, v_min=args.v_min,
                              a_lat_max=args.a_lat_max, kappa_table=kappa_table)

    out_path = args.output or args.input
    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'speed'])
        for xi, yi, vi in zip(x, y, v):
            w.writerow([f'{xi:.4f}', f'{yi:.4f}', f'{vi:.4f}'])
    print(f'저장: {out_path}')


if __name__ == '__main__':
    main()
