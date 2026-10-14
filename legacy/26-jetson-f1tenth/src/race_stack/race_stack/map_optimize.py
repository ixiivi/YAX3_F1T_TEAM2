#!/usr/bin/env python3
"""MAP 컨트롤러 파라미터 최적화 — Pacejka 자전거 모델 기반.

Optuna TPE + vehicle_sim.py 직접 적분 (ROS / gym 불필요).
race_script.yaml 의 map: / limits: 섹션을 읽어 고정 파라미터로 사용하고,
최적화 결과를 같은 파일에 반영한다.

실행:
  python3 src/race_stack/race_stack/map_optimize.py [--trials N] [--timeout S]

최적화 대상:
  l1_min      최소 L1 lookahead [m]    — 직선 진동의 주 원인
  l1_k        속도 비례 lookahead 게인  — L1 = max(l1_min, l1_k × v)
  steer_alpha 조향 저역통과 필터 계수   — 낮을수록 빠르지만 진동
  grip_util   마찰 예산 사용률 (η)     — 속도의 유일한 노브 (v ∝ √η)

속도 프로파일: speed_profile.solve_profile() 공용 모듈 사용
  (컨트롤러와 동일한 물리 — 마찰원 결합 forward-backward)

목표 함수 (낮을수록 좋음):
  score = mean_CTE × 10
        + steer_rate_std × 20    (직선 진동 패널티)
        + lap_time × 0.5         (완주 시)
        + PENALTY                (미완주 시 — 진행 거리로 부분 점수)
        + cte_wall_excess × 300  (벽 충돌 위험 패널티)
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys

import numpy as np
import optuna
import yaml

optuna.logging.set_verbosity(optuna.logging.WARNING)

# ── 경로 설정 ──────────────────────────────────────────────────────────────
_HERE      = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
_RACE_SCRIPT = os.path.join(_REPO_ROOT, 'race_script.yaml')

sys.path.insert(0, _HERE)
from vehicle_sim import VehicleParams, bicycle_ode
from speed_profile import solve_profile

# ── 시뮬 상수 ──────────────────────────────────────────────────────────────
G        = 9.81
DT       = 0.005          # 적분 스텝 [s]
CTRL_HZ  = 50             # 제어 주파수 [Hz]
CTRL_DT  = 1.0 / CTRL_HZ
STEPS_PER_CTRL = max(1, round(CTRL_DT / DT))

TIMEOUT    = 40.0
PENALTY    = 5000.0
CTE_WALL_M = 0.24   # 안전여유 0.35m - 차 반폭 0.11m

DEPART_DIST = 3.0
ARRIVE_DIST = 1.5
MAP_OUT_DIST = 5.0


# ── 유틸 ──────────────────────────────────────────────────────────────────

def _rk4_step(state: np.ndarray, delta: float, Fx: float,
              p: VehicleParams) -> np.ndarray:
    def f(s):
        return bicycle_ode(0.0, s, delta, Fx, p)
    k1 = np.array(f(state))
    k2 = np.array(f(state + DT / 2 * k1))
    k3 = np.array(f(state + DT / 2 * k2))
    k4 = np.array(f(state + DT * k3))
    return state + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def _load_waypoints(path: str) -> np.ndarray:
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append([float(r['x']), float(r['y']), float(r['speed'])])
    wp = np.array(rows)
    keep = np.concatenate([[True], np.linalg.norm(np.diff(wp[:, :2], axis=0), axis=1) > 1e-6])
    return wp[keep]


def _precompute_kappa(wp: np.ndarray) -> np.ndarray:
    from scipy.interpolate import CubicSpline
    pts = wp[:, :2]
    if np.linalg.norm(pts[-1] - pts[0]) < 1e-6:
        pts = pts[:-1]
    n = len(pts)
    ds = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(ds)])
    gap = np.linalg.norm(pts[0] - pts[-1])
    s_ext = np.append(s, s[-1] + max(gap, 1e-6))
    x_ext = np.append(pts[:, 0], pts[0, 0])
    y_ext = np.append(pts[:, 1], pts[0, 1])
    cs_x = CubicSpline(s_ext, x_ext, bc_type='periodic')
    cs_y = CubicSpline(s_ext, y_ext, bc_type='periodic')
    dx, dy   = cs_x(s, 1), cs_y(s, 1)
    ddx, ddy = cs_x(s, 2), cs_y(s, 2)
    denom = np.maximum((dx**2 + dy**2) ** 1.5, 1e-6)
    return np.abs(dx * ddy - dy * ddx) / denom


def _build_speed_profile(wp: np.ndarray, grip_util: float, p: VehicleParams,
                         v_min: float) -> np.ndarray:
    """컨트롤러와 동일한 speed_profile.solve_profile() 사용."""
    mu_eff = p.mu * p.pac_D
    csv_v = wp[:, 2] if wp.shape[1] > 2 else None
    v, _ = solve_profile(
        wp[:, 0], wp[:, 1], csv_v,
        mu_eff=mu_eff, grip_util=grip_util,
        lf=p.lf, lr=p.lr, v_min=v_min,
    )
    return v


def _find_nearest(wp: np.ndarray, pos: np.ndarray, hint: int,
                  search: int = 60) -> int:
    n = len(wp)
    idxs = [(hint + i) % n for i in range(-5, search)]
    dists = [np.linalg.norm(wp[i, :2] - pos) for i in idxs]
    return idxs[int(np.argmin(dists))]


def _find_lookahead(wp: np.ndarray, pos: np.ndarray, L1: float,
                    near: int) -> np.ndarray | None:
    n = len(wp)
    arc = 0.0
    for i in range(n):
        i0 = (near + i) % n
        i1 = (near + i + 1) % n
        seg = float(np.linalg.norm(wp[i1, :2] - wp[i0, :2]))
        if arc + seg >= L1:
            t = float(np.clip((L1 - arc) / max(seg, 1e-6), 0.0, 1.0))
            return (1.0 - t) * wp[i0, :2] + t * wp[i1, :2]
        arc += seg
    return None


# ── Pacejka 역함수 LUT ────────────────────────────────────────────────────

def _build_pac_lut(p: VehicleParams):
    alphas = np.linspace(-0.5, 0.5, 1000)
    B, C, D, E = p.pac_B, p.pac_C, p.pac_D, p.pac_E
    x  = B * alphas
    Fy = D * np.sin(C * np.arctan(x - E * (x - np.arctan(x))))
    return alphas, Fy


def _pac_inv(Fy_req: float, alpha_lut: np.ndarray, Fy_lut: np.ndarray) -> float:
    clamped = float(np.clip(Fy_req, Fy_lut[0] * 0.99, Fy_lut[-1] * 0.99))
    return float(np.interp(clamped, Fy_lut, alpha_lut))


# ── 한 에피소드 시뮬레이션 ─────────────────────────────────────────────────

def run_episode(wp: np.ndarray, v_profile: np.ndarray, kappa: np.ndarray,
                p: VehicleParams, params: dict,
                max_steer: float = 0.36) -> float:
    l1_min      = params['l1_min']
    l1_k        = params['l1_k']
    steer_alpha = params['steer_alpha']
    grip_util   = params['grip_util']

    mu_eff = p.mu * p.pac_D
    a_g    = grip_util * mu_eff * G
    a_max  = 9.51          # 최대 가속 (모터 한계)
    a_brk  = a_g           # 제동: 마찰원 전체 예산

    alpha_lut, Fy_lut = _build_pac_lut(p)

    n   = len(wp)
    wb  = p.lf + p.lr
    tau = 0.25   # 속도 응답 시정수 — 실측 250ms (2026-08-09)

    yaw0  = math.atan2(wp[1, 1] - wp[0, 1], wp[1, 0] - wp[0, 0])
    state = np.array([0.0, 0.0, 0.0, float(wp[0, 0]), float(wp[0, 1]), yaw0])
    start_xy = wp[0, :2].copy()

    steer_cmd = 0.0
    v_cmd     = 0.0
    near_idx  = 0
    elapsed   = 0.0
    departed  = False

    cte_list        = []
    steer_rate_list = []
    prev_steer_cmd  = 0.0
    ctrl_tick       = 0

    while elapsed < TIMEOUT:
        if ctrl_tick % STEPS_PER_CTRL == 0:
            vx, vy, r, x, y, psi = state
            v   = max(float(np.hypot(vx, vy)), 0.3)
            pos = np.array([x, y])

            near_idx = _find_nearest(wp, pos, near_idx)
            L1       = max(l1_min, l1_k * v)
            goal     = _find_lookahead(wp, pos, L1, near_idx)

            if goal is None:
                steer_cmd = 0.0
                v_cmd     = v_profile[near_idx]
            else:
                dx  = goal[0] - x
                dy  = goal[1] - y
                bearing = math.atan2(dy, dx)
                eta = math.atan2(math.sin(bearing - psi), math.cos(bearing - psi))

                a_lat = 2.0 * v * v * math.sin(eta) / max(L1, 1e-3)
                Fzf   = p.m * G * p.lr / wb
                F_yf  = p.m * a_lat * p.lr / wb
                alpha_f = _pac_inv(F_yf / (p.mu * Fzf), alpha_lut, Fy_lut)

                delta_kin = math.atan(wb * 2.0 * math.sin(eta) / max(L1, 1e-3))
                raw = float(np.clip(delta_kin + alpha_f, -max_steer, max_steer))
                steer_cmd = steer_alpha * steer_cmd + (1.0 - steer_alpha) * raw
                v_cmd     = float(v_profile[near_idx])

            cte = float(np.linalg.norm(wp[near_idx, :2] - np.array([x, y])))
            cte_list.append(cte)
            steer_rate_list.append(abs(steer_cmd - prev_steer_cmd) / CTRL_DT)
            prev_steer_cmd = steer_cmd

            if cte > MAP_OUT_DIST:
                progress = min(near_idx / n, 0.99)
                return PENALTY * (1.0 - progress) + PENALTY

        vx = state[0]
        dv = (v_cmd - vx) / tau
        Fx = p.m * max(-a_brk, min(a_max, dv))
        state = _rk4_step(state, steer_cmd, Fx, p)
        state[0] = max(0.0, state[0])

        elapsed   += DT
        ctrl_tick += 1

        pos = np.array([state[3], state[4]])
        dist_from_start = float(np.linalg.norm(pos - start_xy))
        if dist_from_start > DEPART_DIST:
            departed = True
        if departed and dist_from_start < ARRIVE_DIST:
            mean_cte        = float(np.mean(cte_list))  if cte_list else 99.0
            max_cte         = float(np.max(cte_list))   if cte_list else 99.0
            steer_rate_std  = float(np.std(steer_rate_list)) if steer_rate_list else 99.0
            cte_wall_excess = max(0.0, max_cte - CTE_WALL_M)
            return (elapsed * 0.5
                    + mean_cte * 10.0
                    + steer_rate_std * 20.0
                    + cte_wall_excess * 300.0)

    progress = min(near_idx / n, 0.99)
    return PENALTY * (1.0 - progress) + PENALTY


# ── Optuna 최적화 ──────────────────────────────────────────────────────────

def optimize(wp: np.ndarray, kappa: np.ndarray, p: VehicleParams,
             baseline: dict, n_trials: int, timeout_s: float,
             v_min: float, max_steer: float = 0.36):
    best_scores = [999999.0]

    def objective(trial: optuna.Trial) -> float:
        params = {
            'l1_min':      trial.suggest_float('l1_min',      0.40, 1.50),
            'l1_k':        trial.suggest_float('l1_k',        0.20, 0.60),
            'steer_alpha': trial.suggest_float('steer_alpha', 0.10, 0.95),
            'grip_util':   trial.suggest_float('grip_util',   0.30, 1.00),
        }
        v_profile = _build_speed_profile(wp, params['grip_util'], p, v_min)
        score = run_episode(wp, v_profile, kappa, p, params, max_steer=max_steer)

        tag = '★' if score < best_scores[0] else ' '
        if score < best_scores[0]:
            best_scores[0] = score
        completed = score < PENALTY
        status = f'{score:.1f}' if completed else f'미완주({score:.0f})'
        print(f'  [{trial.number:3d}]{tag} score={status:12s}  '
              f'l1={params["l1_min"]:.2f}/{params["l1_k"]:.2f}  '
              f'α={params["steer_alpha"]:.2f}  '
              f'η={params["grip_util"]:.2f}')
        return score

    sampler = optuna.samplers.TPESampler(seed=42)
    study   = optuna.create_study(direction='minimize', sampler=sampler)
    study.enqueue_trial(baseline)

    study.optimize(objective, n_trials=n_trials, timeout=timeout_s)
    return study


# ── race_script.yaml 업데이트 ──────────────────────────────────────────────

def update_race_script(best: dict, mode: str):
    with open(_RACE_SCRIPT) as f:
        text = f.read()

    # map: 섹션 파라미터 (2-space indent)
    for key in ('l1_min', 'l1_k', 'steer_alpha'):
        if key not in best:
            continue
        text = re.sub(
            rf'^(  {key}:)\s+\S+',
            rf'\g<1> {round(best[key], 4)}',
            text, flags=re.MULTILINE,
        )

    # limits.grip_util — 현재 mode 키에 기록 (default는 컨트롤러가 읽지 않음)
    if 'grip_util' in best:
        text = re.sub(
            rf'^(    {mode}:)\s+\S+',
            rf'\g<1> {round(best["grip_util"], 4)}',
            text, flags=re.MULTILINE,
        )

    with open(_RACE_SCRIPT, 'w') as f:
        f.write(text)
    print(f'  → {_RACE_SCRIPT} 업데이트 완료  (grip_util → limits.grip_util.{mode})')


# ── 메인 ──────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description='MAP 파라미터 최적화 (Pacejka 자전거 모델)')
    ap.add_argument('--trials',   type=int,   default=200,  help='최대 시도 횟수')
    ap.add_argument('--timeout',  type=float, default=600,  help='최대 시간 [s]')
    ap.add_argument('--no-write', action='store_true',      help='race_script.yaml 미기록')
    ap.add_argument('--wp',       type=str,   default=None, help='웨이포인트 CSV 경로')
    args = ap.parse_args()

    with open(_RACE_SCRIPT) as f:
        cfg = yaml.safe_load(f)

    mc  = cfg.get('map', {})
    lim = cfg.get('limits', {})

    p = VehicleParams(
        m     = float(mc.get('mass',  3.354)),
        lf    = float(mc.get('lf',    0.1757)),
        lr    = float(mc.get('lr',    0.1441)),
        Iz    = 0.050,
        mu    = float(mc.get('mu',    0.348)),
        pac_B = float(mc.get('pac_b', 3.8)),
        pac_C = float(mc.get('pac_c', 1.3)),
        pac_D = float(mc.get('pac_d', 1.15)),
        pac_E = float(mc.get('pac_e', -0.5)),
    )
    mu_eff = p.mu * p.pac_D

    # grip_util 기준값: 현재 mode 키 → default → 0.6 순으로 폴백
    mode   = cfg.get('mode', 'q1')
    gu_cfg = lim.get('grip_util', 0.6)
    if isinstance(gu_cfg, dict):
        grip_util_base = float(gu_cfg.get(mode, gu_cfg.get('default', 0.6)))
    else:
        grip_util_base = float(gu_cfg)

    v_min      = float(cfg.get('speed', {}).get('min_speed', 0.3))
    max_steer  = float(cfg.get('pure_pursuit', {}).get('max_steer', 0.36))

    baseline = {
        'l1_min':      float(mc.get('l1_min',      0.90)),
        'l1_k':        float(mc.get('l1_k',        0.35)),
        'steer_alpha': float(mc.get('steer_alpha',  0.20)),
        'grip_util':   grip_util_base,
    }

    # ── 웨이포인트 로드 ───────────────────────────────────────────────────
    if args.wp:
        wp_path = args.wp
    else:
        _ws  = os.environ.get('RACE_WS', os.path.expanduser('~/f1tenth_ws'))
        _src = os.path.join(_ws, 'src', 'race_stack')
        if not os.path.isdir(os.path.join(_src, 'waypoints')):
            _src = os.path.join(_REPO_ROOT, 'src', 'race_stack')
        wp_path = os.path.join(_src, 'waypoints', cfg['waypoints'] + '.csv')

    if not os.path.exists(wp_path):
        print(f'웨이포인트 없음: {wp_path}')
        sys.exit(1)

    wp    = _load_waypoints(wp_path)
    kappa = _precompute_kappa(wp)

    print('=' * 70)
    print(' MAP 파라미터 최적화 (Pacejka 자전거 모델)')
    print('=' * 70)
    print(f'  웨이포인트 : {wp_path}  ({len(wp)}개)')
    print(f'  μ={p.mu}  B={p.pac_B} C={p.pac_C} D={p.pac_D} E={p.pac_E}')
    print(f'  μ_eff={mu_eff:.4f}  (= μ × pac_D, 실효 그립)')
    print(f'  m={p.m}kg  lf={p.lf}  lr={p.lr}  max_steer={max_steer}rad')
    print(f'  mode={mode}  v_min={v_min}  trials={args.trials}  timeout={args.timeout}s')
    print()
    print('  기준값:')
    for k, v in baseline.items():
        print(f'    {k:12s} = {v}')
    print()

    v_profile_base = _build_speed_profile(wp, baseline['grip_util'], p, v_min)
    score_base = run_episode(wp, v_profile_base, kappa, p, baseline, max_steer=max_steer)
    print(f'  기준값 score: {score_base:.2f}  '
          f'({"완주" if score_base < PENALTY else "미완주"})')
    print()
    print(f'  {"#":>3}  {"score":>12}  {"l1_min/k":>10}  {"α":>5}  {"η":>5}')
    print('-' * 70)

    study = optimize(wp, kappa, p, baseline, args.trials, args.timeout,
                     v_min=v_min, max_steer=max_steer)

    best   = study.best_params
    best_t = study.best_value
    n_done = len(study.trials)

    print('\n' + '=' * 70)
    print(f' 완료: {n_done}회 시도')
    print('=' * 70)
    print(f'  기준 score : {score_base:.2f}')
    print(f'  최적 score : {best_t:.2f}  ({"완주" if best_t < PENALTY else "미완주"})')
    print()
    print('  최적 파라미터:')
    for k, v in best.items():
        old = baseline.get(k, '?')
        print(f'    {k:12s}: {old} → {v:.4f}')

    if not args.no_write:
        print()
        update_race_script(best, mode)
    else:
        print('\n  (--no-write 지정: race_script.yaml 미기록)')


if __name__ == '__main__':
    main()
