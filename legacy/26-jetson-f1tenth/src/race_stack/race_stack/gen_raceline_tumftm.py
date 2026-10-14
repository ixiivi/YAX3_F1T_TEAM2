#!/usr/bin/env python3
"""
gen_raceline_tumftm.py — TUMFTM 방식 최소 곡률 레이싱라인 생성기 (IQP)

TUMFTM/global_racetrajectory_optimization 의 minimum-curvature 옵션을
포팅. gen_raceline.py 의 단발 QP(opt_min_curv)를 그대로 재사용하되,
경로를 새 참조선으로 삼아 재선형화·재적용하는 IQP(iterative QP) 루프를
추가해 큰 오프셋 구간에서의 선형근사 오차를 줄인다.

곡률은 "평균적으로 매끈함"보다 "차가 못 도는 스파이크가 없음"을 우선해서,
QP 자체에 gen_waypoints_kappa.py 와 동일한 물리적 최대곡률
(κ_max = tan(max_steer)/wheelbase) 하드 제약을 건다.

속도 프로파일은 상수 마찰계수(mu) 원 대신, 실측 그립 LUT
(docs/lut_grip040/lut_grip0.40.csv — v-δ 스윕에서 뽑은 속도별 최대 횡가속)를
사용해 타이어 포화 비선형성을 반영한다.

알고리즘: Heilmeier et al. "Minimum Curvature Trajectory Planning and Control
for an Autonomous Racecar" (2019), DOI: 10.1080/00423114.2019.1631455

사용법:
  python3 gen_raceline_tumftm.py                          # 대화형 맵 선택
  python3 gen_raceline_tumftm.py --map sohot_clean
  python3 gen_raceline_tumftm.py --map sohot_clean --iqp-iters 5
"""

import csv
import math
import os
import argparse
import numpy as np

from race_stack.gen_raceline import (
    MAPS_DIR, WP_DIR,
    extract_reftrack, interp_track, calc_splines, opt_min_curv,
    apply_accel_brake_sweep, save_overlay,
)
from race_stack.paths import repo_root

DEFAULT_LUT = os.path.join(repo_root(), 'docs', 'lut_grip040', 'lut_grip0.40.csv')


# ══════════════════════════════════════════════════════════════════════════════
# 최소 곡률 IQP (댐핑 반복 재선형화 + 물리적 곡률 하드캡)
# ══════════════════════════════════════════════════════════════════════════════
def opt_min_curv_iqp(reftrack_u: np.ndarray,
                     w_veh:     float,
                     kappa_max: float = None,
                     iters_max: int = 8,
                     alpha_conv_thresh: float = 0.005,
                     damping:   float = 0.5) -> np.ndarray:
    """
    opt_min_curv 를 반복 적용하는 IQP.

    QP가 예측하는 곡률(kappa_ref + E·α)은 **작은 α에서만 유효한 1차 선형근사**다.
    실측 결과 한 라운드에 큰 α(최대 0.81m)를 통째로 적용하면, 예측 곡률은
    1.14인데 새 노드로 스플라인을 다시 피팅한 실제 곡률은 21.9까지 벌어졌다 —
    선형근사가 완전히 깨진 것. 그래서 매 라운드 QP가 계산한 α 중 damping
    비율만 실제로 적용해 작은 스텝으로 여러 번(iters_max) 나눠 수렴시킨다.

    kappa_max 를 주면 매 라운드 QP에 하드 곡률 제약이 걸려, 차량이 물리적으로
    돌 수 없는 곡률의 지점이 애초에 나오지 않는다.

    반환: 최종 레이싱라인 노드 [N, 2]
    """
    path    = reftrack_u[:, :2].copy()
    w_right = reftrack_u[:, 2].copy()
    w_left  = reftrack_u[:, 3].copy()

    for it in range(1, iters_max + 1):
        path_cl = np.vstack([path, path[0]])
        _, _, M_spl, normvec = calc_splines(path_cl)

        cur_reftrack = np.column_stack([path, w_right, w_left])
        alpha_full = opt_min_curv(cur_reftrack, normvec, M_spl, w_veh=w_veh,
                                  kappa_max=kappa_max)
        alpha = damping * alpha_full

        path    = path + alpha[:, None] * normvec
        # 이번 라운드에 소비한 폭만큼 다음 라운드의 여유 폭에서 차감
        w_right = np.maximum(w_right - alpha, 0.01)
        w_left  = np.maximum(w_left  + alpha, 0.01)

        max_alpha = float(np.max(np.abs(alpha)))
        print(f'  [IQP {it}/{iters_max}] max|α_full|={np.max(np.abs(alpha_full)):.4f}m'
             f'  적용(×{damping})={max_alpha:.4f}m')
        if max_alpha < alpha_conv_thresh:
            print(f'  IQP 수렴 (max|α| < {alpha_conv_thresh} m)')
            break

    return path


# ══════════════════════════════════════════════════════════════════════════════
# QP가 실제로 푼 스플라인을 그대로 샘플링 (재적합 없음 → 곡률 불일치 없음)
# ══════════════════════════════════════════════════════════════════════════════
def sample_analytic_spline(coeffs_x: np.ndarray, coeffs_y: np.ndarray,
                           stepsize: float) -> tuple:
    """
    calc_splines 가 만든 조각별 3차 다항식을 scipy로 다시 스무딩-재적합하지
    않고 그대로 균일 호길이로 직접 샘플링한다.

    이전엔 build_raceline(splprep 재적합)을 썼는데, QP가 최적화한 스플라인과
    다른 스플라인을 다시 피팅하다 보니 둘의 곡률이 어긋났다(같은 노드를
    지나도 스플라인 종류가 다르면 그 사이 형태가 달라짐). 이 함수는 QP가
    실제로 풀었던 그 곡선을 그대로 내보내므로 그런 불일치가 없다 — 다만
    QP 자체가 세그먼트 시작점(0.5m 격자)에서만 곡률을 체크했으므로, 세그먼트
    "내부"의 곡률까지 안전하다는 보장은 아니다(그건 별도 최종 검사로 확인).

    반환: (raceline [M,2], el [M], kappa [M])
    """
    N = coeffs_x.shape[0]
    tt0 = np.linspace(0, 1, 21)

    seg_len = np.empty(N)
    for i in range(N):
        ax, bx, cx, dx = coeffs_x[i]
        ay, by, cy, dy = coeffs_y[i]
        xs = ax + bx * tt0 + cx * tt0**2 + dx * tt0**3
        ys = ay + by * tt0 + cy * tt0**2 + dy * tt0**3
        seg_len[i] = np.sum(np.hypot(np.diff(xs), np.diff(ys)))
    total = seg_len.sum()
    M = max(3 * N, int(total / stepsize))

    xs_all, ys_all, k_all = [], [], []
    for i in range(N):
        n_i = max(1, int(round(seg_len[i] / total * M)))
        t = np.linspace(0, 1, n_i, endpoint=False)
        ax, bx, cx, dx = coeffs_x[i]
        ay, by, cy, dy = coeffs_y[i]
        x   = ax + bx * t + cx * t**2 + dx * t**3
        y   = ay + by * t + cy * t**2 + dy * t**3
        xp  = bx + 2 * cx * t + 3 * dx * t**2
        yp  = by + 2 * cy * t + 3 * dy * t**2
        xpp = 2 * cx + 6 * dx * t
        ypp = 2 * cy + 6 * dy * t
        denom = np.maximum((xp**2 + yp**2) ** 1.5, 1e-9)
        k = np.abs(xp * ypp - yp * xpp) / denom
        xs_all.append(x); ys_all.append(y); k_all.append(k)

    raceline = np.column_stack([np.concatenate(xs_all), np.concatenate(ys_all)])
    kappa = np.concatenate(k_all)
    el = np.linalg.norm(np.diff(np.vstack([raceline, raceline[0]]), axis=0), axis=1)
    return raceline, el, kappa


# ══════════════════════════════════════════════════════════════════════════════
# 레이싱라인 생성 + 세그먼트 내부 곡률 최종 검사
# ══════════════════════════════════════════════════════════════════════════════
def generate_raceline_verified(reftrack_u: np.ndarray,
                               w_veh:     float,
                               kappa_max: float,
                               step:      float,
                               iqp_iters: int = 8,
                               iqp_thresh: float = 0.005,
                               damping:   float = 0.5) -> tuple:
    """
    댐핑 IQP로 경로를 풀고, QP가 실제로 최적화한 스플라인을 그대로(재적합 없이)
    샘플링해 최종 레이싱라인을 만든다. 세그먼트 내부(QP가 직접 체크 안 한
    지점)에서도 한계를 지키는지 최종 검사해서 로그로 남긴다.

    반환: (raceline [M,2], el [M], kappa [M])
    """
    path = opt_min_curv_iqp(reftrack_u, w_veh=w_veh, kappa_max=kappa_max,
                            iters_max=iqp_iters, alpha_conv_thresh=iqp_thresh,
                            damping=damping)

    path_cl = np.vstack([path, path[0]])
    coeffs_x, coeffs_y, _, _ = calc_splines(path_cl)
    raceline, el, kappa = sample_analytic_spline(coeffs_x, coeffs_y, step)

    kappa_act = float(np.abs(kappa).max())
    over = int((np.abs(kappa) > kappa_max).sum())
    print(f'  [최종검사] 세그먼트 내부 포함 |κ|_max={kappa_act:.4f} 1/m'
         f'  (목표 {kappa_max:.4f}, 초과 {over}/{len(kappa)}pt)')
    if kappa_act > kappa_max * 1.005:
        print(f'  ⚠ 물리 한계 초과 — QP가 세그먼트 시작점만 체크해 내부 피크를 놓침')

    return raceline, el, kappa


# ══════════════════════════════════════════════════════════════════════════════
# 그립 LUT 기반 속도 프로파일
# ══════════════════════════════════════════════════════════════════════════════
def load_grip_lut(path: str) -> tuple:
    """
    vehicle_sim 의 v-δ 스윕 CSV(v, ay, ...)에서 속도별 최대 횡가속 →
    최대 곡률 κ_max(v) = max|ay(v)| / v² 곡선을 뽑는다.

    반환: (v_grid 오름차순 [M], kappa_max 그 v에서의 물리적 최대곡률 [M])
    """
    v_col, ay_col = [], []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            v_col.append(float(row['v']))
            ay_col.append(float(row['ay']))
    v_col  = np.asarray(v_col)
    ay_col = np.abs(np.asarray(ay_col))

    # v 는 스윕 스크립트가 각 속도마다 동일 float 값을 반복 기록하므로
    # 정확히 일치하는 값으로 묶는다 (반올림 후 isclose 는 근접 속도값을
    # 갈라놓아 일부 버킷이 빈 배열이 되는 문제가 있었다).
    v_grid = np.array(sorted(set(v_col)))
    kappa_max = np.array([
        ay_col[v_col == v].max() / max(v, 1e-3) ** 2
        for v in v_grid
    ])
    return v_grid, kappa_max


def calc_vel_profile_lut(kappa:    np.ndarray,
                         el:       np.ndarray,
                         v_grid:   np.ndarray,
                         kappa_lut: np.ndarray,
                         v_max:    float,
                         v_min:    float,
                         a_lon_acc: float,
                         a_lon_brk: float) -> np.ndarray:
    """
    calc_vel_profile 과 동일한 종방향 가감속 스윕을 쓰되, 횡방향 한계를 상수
    mu 원이 아니라 실측 LUT(v_grid, kappa_lut — v 오름차순, kappa_lut 내림차순)
    에서 역으로 조회해 구한다.

    LUT가 테스트한 속도 범위([v_grid[0], v_grid[-1]]) 밖은 신뢰하지 않는다:
      - 곡률이 v_grid[-1]에서의 κ_max보다 작으면(완만한 직선 등) LUT가 커버
        못하는 고속 영역이니 v_max 그대로 허용
      - 곡률이 v_grid[0]에서의 κ_max보다 크면(LUT 최저속에서도 못 도는 급코너)
        가장 보수적인 v_grid[0]로 바닥을 깐다
    """
    n = len(kappa)
    kappa_asc = kappa_lut[::-1]   # v 내림차순 → 오름차순으로 뒤집어 κ 오름차순 맞춤
    v_desc    = v_grid[::-1]

    v_lat = np.where(
        kappa <= kappa_asc[0], v_max,
        np.where(kappa >= kappa_asc[-1], v_grid[0],
                 np.interp(kappa, kappa_asc, v_desc)))
    v = np.minimum(np.full(n, v_max), v_lat)

    v = apply_accel_brake_sweep(v, el, a_lon_acc, a_lon_brk)
    v = np.clip(v, v_min, v_max)

    lap_t = float(np.sum(el / np.maximum(v, 0.01)))
    floor_pct = (kappa >= kappa_asc[-1]).mean() * 100
    print(f'  속도(LUT): {v.min():.2f}~{v.max():.2f} m/s  예상 랩타임={lap_t:.1f}s')
    if floor_pct > 0:
        print(f'  LUT 최저속({v_grid[0]}m/s) 그립도 못 버티는 구간: {floor_pct:.1f}%'
              f' — 곡률 하드캡을 더 낮추는 걸 고려')
    return v


# ══════════════════════════════════════════════════════════════════════════════
# main
# ══════════════════════════════════════════════════════════════════════════════
def main():
    p = argparse.ArgumentParser(description='TUMFTM 최소 곡률(IQP) 레이싱라인 생성기')
    p.add_argument('--map',        default=None,  help='맵 이름 (생략 시 대화형 선택)')
    p.add_argument('--vmax',       type=float, default=8.0,  help='최대 속도 (m/s)')
    p.add_argument('--vmin',       type=float, default=1.0,  help='최소 속도 (m/s)')
    p.add_argument('--wveh',       type=float, default=0.31, help='차량 폭 (m)')
    p.add_argument('--safety',     type=float, default=0.15, help='벽 안전 마진 (m)')
    p.add_argument('--acc',        type=float, default=5.0,  help='가속도 (m/s²)')
    p.add_argument('--brk',        type=float, default=8.0,  help='제동 (m/s²)')
    p.add_argument('--step',       type=float, default=0.1,  help='웨이포인트 간격 (m)')
    p.add_argument('--iqp-iters',  type=int,   default=8,    help='IQP 최대 반복 횟수')
    p.add_argument('--iqp-thresh', type=float, default=0.005, help='IQP 수렴 판정 max|α| (m)')
    p.add_argument('--damping',    type=float, default=0.5,  help='IQP 라운드당 α 적용 비율 (선형근사 유효범위 유지용)')
    p.add_argument('--wheelbase',  type=float, default=0.33, help='축간거리 (m) — race_script.yaml pure_pursuit.wheelbase 와 맞출 것')
    p.add_argument('--max-steer',  type=float, default=0.36, help='최대 조향각 (rad) — race_script.yaml pure_pursuit.max_steer 와 맞출 것')
    p.add_argument('--lut',        default=DEFAULT_LUT, help='그립 LUT csv 경로')
    p.add_argument('--no-lut',     action='store_true', help='LUT 대신 상수 mu 속도 프로파일 사용')
    p.add_argument('--mu',         type=float, default=0.40, help='--no-lut 일 때 쓸 마찰 계수')
    args = p.parse_args()

    kappa_max = math.tan(args.max_steer) / args.wheelbase
    print(f'물리적 최대곡률 κ_max={kappa_max:.3f} 1/m '
         f'(wheelbase={args.wheelbase}m, max_steer={math.degrees(args.max_steer):.1f}°, '
         f'최소회전반경 {1/kappa_max:.2f}m)')

    # 맵 선택
    if args.map:
        map_name = args.map
    else:
        maps = sorted(set(
            f.rsplit('.', 1)[0]
            for f in os.listdir(MAPS_DIR)
            if f.endswith(('.pgm', '.png'))
        ))
        print('\n사용 가능한 맵:')
        for i, m in enumerate(maps, 1):
            print(f'  {i}. {m}')
        val = input('맵 이름 또는 번호: ').strip()
        map_name = maps[int(val) - 1] if val.isdigit() else val

    wp_path = os.path.join(WP_DIR, f'{map_name}_wp_tumftm.csv')
    os.makedirs(WP_DIR, exist_ok=True)

    print(f'\n맵: {map_name}  vmax={args.vmax}  wveh={args.wveh}  safety={args.safety}')

    print('\n[1] 트랙 경계 추출...')
    reftrack = extract_reftrack(map_name, safety_m=args.safety)

    print('\n[2] 균일 리샘플 (0.5m 간격)...')
    reftrack_u = interp_track(reftrack, stepsize=0.5)
    print(f'  {len(reftrack_u)}pts')

    print('\n[3] 최소 곡률 IQP (댐핑 반복, 곡률 하드캡 포함)...')
    raceline, el, kappa = generate_raceline_verified(
        reftrack_u, w_veh=args.wveh, kappa_max=kappa_max, step=args.step,
        iqp_iters=args.iqp_iters, iqp_thresh=args.iqp_thresh,
        damping=args.damping)

    print('\n[4] 속도 프로파일...')
    if args.no_lut:
        from race_stack.gen_raceline import calc_vel_profile
        v = calc_vel_profile(kappa, el, mu=args.mu, v_max=args.vmax, v_min=args.vmin,
                             a_lon_acc=args.acc, a_lon_brk=args.brk)
    else:
        v_grid, kappa_lut = load_grip_lut(args.lut)
        print(f'  LUT: {args.lut} ({v_grid[0]}~{v_grid[-1]} m/s, {len(v_grid)}단계)')
        v = calc_vel_profile_lut(kappa, el, v_grid, kappa_lut,
                                 v_max=args.vmax, v_min=args.vmin,
                                 a_lon_acc=args.acc, a_lon_brk=args.brk)

    print('\n[5] 저장...')
    with open(wp_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'speed'])
        for (x, y), vi in zip(raceline, v):
            w.writerow([f'{x:.4f}', f'{y:.4f}', f'{vi:.4f}'])
    print(f'  웨이포인트: {len(raceline)}pts → {wp_path}')

    save_overlay(map_name, raceline, v, wp_path)

    print('\n완료! 다음 단계:')
    print(f'  race_script.yaml: map_name: {map_name}, waypoints: {map_name}_wp_tumftm')


if __name__ == '__main__':
    main()
