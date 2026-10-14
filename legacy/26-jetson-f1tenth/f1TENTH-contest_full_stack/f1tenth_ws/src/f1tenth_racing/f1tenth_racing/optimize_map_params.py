#!/usr/bin/env python3
"""
MAP 컨트롤러 파라미터 자동 최적화
Optuna (TPE Bayesian) + f1tenth_gym 직접 구동

최적화 대상 (6개):
  l1_min, l1_k, tau_steer, slip_gain, speed_lag, grip_util

제거된 항목:
  a_brake   — 실차 폐기됨 (a_lon_brk 로 대체, 물리에서 자동 유도)
  speed_scale — 실차 폐기됨 (grip_util 로 대체)
  pac_d     — 고정 (실차 캘리브레이션 값 1.15)

tau_v = 0.40:
  실측 속도 지연 406~440ms. gym step에 1차 IIR 필터를 씌워
  낙관적 파라미터 선택을 방지한다.

steer_alpha → tau_steer [s]:
  CTRL_DT = 30ms (시뮬), REAL_CAR_DT = 20ms (실차 50Hz) 로 루프 주기가 달라
  같은 alpha 가 다른 시정수를 낸다. 시정수로 탐색해 환산한다.
    alpha_sim  = exp(-0.030 / tau_steer)
    alpha_real = exp(-0.020 / tau_steer)  ← race_script.yaml steer_alpha 에 넣을 값
"""

import math
import numpy as np
import gym
import optuna
import csv
import os
from scipy.interpolate import CubicSpline
from f110_gym.envs.base_classes import Integrator

optuna.logging.set_verbosity(optuna.logging.WARNING)

_THIS_DIR  = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
MAP_PATH   = os.path.join(_REPO_ROOT, '..', 'src', 'race_stack', 'maps', 'fun_clean')
WP_PATH    = os.path.join(_REPO_ROOT, '..', 'src', 'race_stack', 'waypoints', 'fun_clean_map_wp.csv')
MAP_NODE_PATH = os.path.join(_THIS_DIR, 'map_controller_node.py')

# ── 시뮬레이션 설정 ───────────────────────────────────────────────────────
TIMEOUT    = 60.0
PENALTY    = 10000.0
N_WP       = 446        # fun_clean 웨이포인트 수
GYM_DT     = 0.01       # gym 물리 스텝 [s]
CTRL_STEPS = 3          # 제어 주기 = CTRL_STEPS × GYM_DT = 30ms
CTRL_DT    = CTRL_STEPS * GYM_DT   # 0.030 s
REAL_CAR_DT = 0.020     # 실차 map_controller 50Hz 타이머 [s]

# ── 속도 지연 (tau_v) ────────────────────────────────────────────────────
# 실측 406~440ms. gym step 마다 1차 IIR 로 명령 속도를 필터링해
# 옵티마이저가 실차보다 늦게 제동해도 되는 것처럼 보는 것을 막는다.
TAU_V   = 0.40
ALPHA_V = math.exp(-GYM_DT / TAU_V)   # ≈ 0.9753

# ── 고정 차량 물리값 (실차 race_script.yaml map: 절과 동일) ───────────────
# ★ 같은 값을 써야 옵티마이저 결과가 이전될 때 물리 맥락이 유지된다.
LF        = 0.1757
LR        = 0.1441
WHEELBASE = LF + LR       # 0.3198 m
MAX_STEER = 0.4189        # rad
MASS      = 3.354         # kg
MU        = 0.348         # 실효 마찰계수 기반값 (map.mu)
PAC_B     = 3.8
PAC_C     = 1.3
PAC_D     = 1.15          # 캘리브레이션 고정 — 옵티마이저 대상 아님
PAC_E     = -0.5
GRAVITY   = 9.81

MU_EFF        = MU * PAC_D                # ≈ 0.400
A_BRAKE_FIXED = MU_EFF * GRAVITY          # ≈ 3.92 m/s² (물리 최대 제동)


def load_waypoints(path):
    pts = []
    with open(path) as f:
        for row in csv.DictReader(f):
            pts.append([float(row['x']), float(row['y']), float(row['speed'])])
    return np.array(pts)


def precompute_ds(wp):
    """웨이포인트 간 호 길이 배열."""
    n = len(wp)
    return np.array([
        np.linalg.norm(wp[(i + 1) % n, :2] - wp[i, :2])
        for i in range(n)
    ])


def precompute_curvature(wp):
    pts = wp[:, :2]
    n   = len(pts)
    ds  = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s   = np.concatenate([[0.0], np.cumsum(ds)])
    s_ext = np.append(s, s[-1] + np.linalg.norm(pts[0] - pts[-1]))
    x_ext = np.append(pts[:, 0], pts[0, 0])
    y_ext = np.append(pts[:, 1], pts[0, 1])
    cs_x  = CubicSpline(s_ext, x_ext, bc_type='periodic')
    cs_y  = CubicSpline(s_ext, y_ext, bc_type='periodic')
    dx    = cs_x(s, 1);  dy  = cs_y(s, 1)
    ddx   = cs_x(s, 2);  ddy = cs_y(s, 2)
    denom = np.maximum((dx ** 2 + dy ** 2) ** 1.5, 1e-6)
    return np.abs(dx * ddy - dy * ddx) / denom


def precompute_speed_profile(wp, ds, curvature, grip_util):
    """마찰원 캡 + 역방향 스윕. grip_util 이 마찰 예산 전체를 조절한다."""
    n = len(wp)
    v_profile = wp[:, 2].copy()

    # 마찰원 속도 한계: v ≤ sqrt(grip_util × μ_eff × g / κ)
    a_lat_max = grip_util * MU_EFF * GRAVITY
    v_cap = np.sqrt(a_lat_max / np.maximum(curvature, 1e-4))
    v_profile = np.minimum(v_profile, v_cap)

    # 역방향 스윕 (제동 가속도도 grip_util 비례)
    a_brk = grip_util * A_BRAKE_FIXED
    for _ in range(3):
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v_lim = np.sqrt(v_profile[j] ** 2 + 2.0 * a_brk * ds[i])
            v_profile[i] = min(v_profile[i], v_lim)
    return v_profile


def build_pacejka_lut():
    alphas  = np.linspace(-0.30, 0.30, 500)
    Fy_norm = PAC_D * np.sin(
        PAC_C * np.arctan(
            PAC_B * alphas
            - PAC_E * (PAC_B * alphas - np.arctan(PAC_B * alphas))
        )
    )
    return alphas, Fy_norm


_PAC_ALPHA_LUT, _PAC_FY_LUT = build_pacejka_lut()


def pacejka_inv(Fy_req):
    clamped = float(np.clip(Fy_req,
                            _PAC_FY_LUT[0]  * 0.99,
                            _PAC_FY_LUT[-1] * 0.99))
    return float(np.interp(clamped, _PAC_FY_LUT, _PAC_ALPHA_LUT))


def find_lookahead(wp, pos, lookahead, near_idx):
    n = len(wp)
    dists = np.linalg.norm(wp[:, :2] - pos, axis=1)
    candidates = [(near_idx + i) % n for i in range(50)]
    near = candidates[int(np.argmin([dists[i] for i in candidates]))]

    arc = 0.0
    for i in range(n):
        i0 = (near + i)     % n
        i1 = (near + i + 1) % n
        seg = float(np.linalg.norm(wp[i1, :2] - wp[i0, :2]))
        if arc + seg >= lookahead:
            alpha   = float(np.clip((lookahead - arc) / max(seg, 1e-6), 0.0, 1.0))
            goal_xy = (1.0 - alpha) * wp[i0, :2] + alpha * wp[i1, :2]
            return goal_xy, near
        arc += seg
    return None, near


def compute_action(wp, v_profile, ds, median_ds, pos, yaw, speed, near_idx,
                   steer_prev, params):
    v  = max(speed, 0.3)
    L1 = max(params['L1_MIN'], params['L1_K'] * v)

    goal, near = find_lookahead(wp, pos, L1, near_idx)
    if goal is None:
        return 0.0, v_profile[near_idx], near_idx

    dx      = goal[0] - pos[0]
    dy      = goal[1] - pos[1]
    bearing = np.arctan2(dy, dx)
    eta     = float(np.arctan2(np.sin(bearing - yaw), np.cos(bearing - yaw)))

    a_lat   = 2.0 * v ** 2 * np.sin(eta) / L1
    Fzf     = MASS * GRAVITY * LR / WHEELBASE
    F_yf    = MASS * a_lat * LR / WHEELBASE
    alpha_f = pacejka_inv(F_yf / (MU * Fzf))
    alpha_f *= params['SLIP_GAIN']      # 슬립 보정 이득

    delta_kin = np.arctan(WHEELBASE * 2.0 * np.sin(eta) / max(L1, 1e-3))
    raw_steer = float(np.clip(delta_kin + alpha_f, -MAX_STEER, MAX_STEER))

    # steer_alpha: 시정수 tau_steer 를 CTRL_DT 기준으로 변환
    tau_steer = params['TAU_STEER']
    alpha     = math.exp(-CTRL_DT / tau_steer)
    steer     = alpha * steer_prev + (1.0 - alpha) * raw_steer

    # speed_lag: 지연만큼 앞의 프로파일 최솟값으로 미리 감속
    lag  = params['SPEED_LAG']
    n    = len(wp)
    k    = int(round(v * lag / max(median_ds, 1e-6)))
    if k > 0:
        idx          = np.arange(near, near + k + 1) % n
        target_speed = float(v_profile[idx].min())
    else:
        target_speed = float(v_profile[near])

    return steer, target_speed, near


def run_trial(env, wp, ds, curvature, median_ds, spawn, params):
    v_profile = precompute_speed_profile(wp, ds, curvature, params['GRIP_UTIL'])

    obs, _, _, _ = env.reset(spawn)
    elapsed     = 0.0
    departed    = False
    steer       = 0.0
    tgt_speed   = 1.0
    v_effective = 0.0   # tau_v IIR 필터 상태 (속도 지연 모델)
    near_idx    = 0
    step_count  = 0

    yaw0  = float(obs['poses_theta'][0])
    fwd   = np.array([np.cos(yaw0), np.sin(yaw0)])
    wp_dir = wp[1, :2] - wp[0, :2]
    if np.dot(fwd, wp_dir) < 0:
        wp        = wp[::-1]
        v_profile = v_profile[::-1]

    start_xy = spawn[0, :2]

    while elapsed < TIMEOUT:
        pos   = np.array([obs['poses_x'][0], obs['poses_y'][0]])
        yaw   = float(obs['poses_theta'][0])
        speed = float(np.hypot(obs['linear_vels_x'][0], obs['linear_vels_y'][0]))

        if step_count % CTRL_STEPS == 0:
            steer, tgt_speed, near_idx = compute_action(
                wp, v_profile, ds, median_ds, pos, yaw, speed, near_idx,
                steer, params)

        # 속도 1차 지연 필터 — 실차 VESC 응답 τ=TAU_V 모델링
        v_effective = ALPHA_V * v_effective + (1.0 - ALPHA_V) * tgt_speed

        obs, _, _, _ = env.step(np.array([[steer, v_effective]]))
        elapsed    += GYM_DT
        step_count += 1

        if obs['collisions'][0]:
            dists    = np.linalg.norm(wp[:, :2] - pos, axis=1)
            progress = int(np.argmin(dists))
            return PENALTY + (N_WP - progress)

        dist_from_start = float(np.linalg.norm(pos - start_xy))
        if dist_from_start > 5.0:
            departed = True
        if departed and dist_from_start < 1.5:
            return elapsed

    dists    = np.linalg.norm(wp[:, :2] - np.array([obs['poses_x'][0],
                                                      obs['poses_y'][0]]), axis=1)
    progress = int(np.argmin(dists))
    return PENALTY + (N_WP - progress)


def update_map_controller(best_params):
    """시뮬 map_controller_node.py 의 기본값을 갱신하고 실차 환산값을 출력한다."""
    alpha_sim  = math.exp(-CTRL_DT    / best_params['TAU_STEER'])
    alpha_real = math.exp(-REAL_CAR_DT / best_params['TAU_STEER'])

    with open(MAP_NODE_PATH) as f:
        code = f.read()

    import re
    replacements = {
        'L1_MIN':     f"self.L1_MIN = {best_params['L1_MIN']:.4f}",
        'L1_K':       f"self.L1_K   = {best_params['L1_K']:.4f}",
        'STEER_ALPHA': f"self.STEER_ALPHA = {alpha_sim:.4f}",
    }
    for key, new_line in replacements.items():
        code = re.sub(rf'self\.{key}\s*=\s*[^\n]+', new_line, code)

    with open(MAP_NODE_PATH, 'w') as f:
        f.write(code)

    print('  → map_controller_node.py 업데이트 완료')
    print()
    print('  ── race_script.yaml map: 절에 넣을 값 ─────────────────────────')
    print(f'    l1_min:     {best_params["L1_MIN"]:.4f}')
    print(f'    l1_k:       {best_params["L1_K"]:.4f}')
    print(f'    steer_alpha: {alpha_real:.4f}   # tau={best_params["TAU_STEER"]*1000:.1f}ms → '
          f'alpha(20ms)={alpha_real:.4f}')
    print(f'    slip_gain:  {best_params["SLIP_GAIN"]:.4f}')
    print(f'    speed_lag:  {best_params["SPEED_LAG"]:.4f}')
    print()
    print('  ── race_script.yaml limits: 절에 넣을 값 ──────────────────────')
    print(f'    grip_util: {best_params["GRIP_UTIL"]:.4f}')


def main():
    print('=' * 66)
    print(' MAP 컨트롤러 파라미터 최적화 (fun_clean / Optuna TPE)')
    print(f' tau_v={TAU_V}s  CTRL_DT={CTRL_DT*1000:.0f}ms  REAL_CAR_DT={REAL_CAR_DT*1000:.0f}ms')
    print('=' * 66)

    wp        = load_waypoints(WP_PATH)
    ds        = precompute_ds(wp)
    curvature = precompute_curvature(wp)
    median_ds = float(np.median(ds))
    print(f'  웨이포인트: {len(wp)}개  median_ds={median_ds*100:.1f}cm')
    print(f'  속도 범위: {wp[:,2].min():.2f}~{wp[:,2].max():.2f} m/s')
    print(f'  곡률 범위: {curvature.min():.3f}~{curvature.max():.3f} 1/m')

    yaw0  = np.arctan2(wp[1, 1] - wp[0, 1], wp[1, 0] - wp[0, 0])
    spawn = np.array([[wp[0, 0], wp[0, 1], yaw0]])

    env = gym.make(
        'f110_gym:f110-v0',
        num_agents=1,
        map=MAP_PATH,
        map_ext='.pgm',
        integrator=Integrator.RK4,
        timestep=GYM_DT,
    )
    print('  시뮬레이터 준비 완료\n')

    # 기준값: 실차 현재 적용 중인 값
    baseline = {
        'L1_MIN':    0.90,
        'L1_K':      0.35,
        'TAU_STEER': 0.034,   # 실차 steer_alpha=0.555 → tau=34ms (dt=20ms)
        'SLIP_GAIN': 0.5,
        'SPEED_LAG': 0.25,
        'GRIP_UTIL': 0.6,
    }
    print('기준값 평가 중...')
    t0 = run_trial(env, wp, ds, curvature, median_ds, spawn, baseline)
    lapped = t0 < PENALTY
    print(f'  기준: {"완주 " + str(round(t0, 2)) + "s" if lapped else "미완주 (score=" + str(round(t0, 0)) + ")"}')
    alpha_sim0  = math.exp(-CTRL_DT    / baseline['TAU_STEER'])
    alpha_real0 = math.exp(-REAL_CAR_DT / baseline['TAU_STEER'])
    print(f'  기준 steer_alpha: sim={alpha_sim0:.4f}  real={alpha_real0:.4f}  '
          f'tau={baseline["TAU_STEER"]*1000:.1f}ms\n')

    best_so_far = [t0]

    def objective(trial):
        params = {
            'L1_MIN':    trial.suggest_float('L1_MIN',    0.30, 1.50),
            'L1_K':      trial.suggest_float('L1_K',      0.05, 0.50),
            'TAU_STEER': trial.suggest_float('TAU_STEER', 0.005, 0.200),
            'SLIP_GAIN': trial.suggest_float('SLIP_GAIN', 0.0,  1.0),
            'SPEED_LAG': trial.suggest_float('SPEED_LAG', 0.0,  0.45),
            'GRIP_UTIL': trial.suggest_float('GRIP_UTIL', 0.3,  1.0),
        }
        t = run_trial(env, wp, ds, curvature, median_ds, spawn, params)

        marker = ''
        if t < best_so_far[0]:
            best_so_far[0] = t
            marker = '  ★'

        a_sim  = math.exp(-CTRL_DT    / params['TAU_STEER'])
        a_real = math.exp(-REAL_CAR_DT / params['TAU_STEER'])
        lapped = t < PENALTY
        status = f'{t:.2f}s' if lapped else f'충돌({int(N_WP - (t - PENALTY))}wp)'
        print(f'  [{trial.number:3d}] {status:10s}  '
              f'L1={params["L1_MIN"]:.2f}+{params["L1_K"]:.3f}v  '
              f'τ={params["TAU_STEER"]*1000:.1f}ms(α={a_sim:.3f})  '
              f'slip={params["SLIP_GAIN"]:.2f}  '
              f'lag={params["SPEED_LAG"]:.2f}  '
              f'η={params["GRIP_UTIL"]:.2f}{marker}')
        return t

    sampler = optuna.samplers.TPESampler(seed=42)
    study   = optuna.create_study(direction='minimize', sampler=sampler)
    study.enqueue_trial(baseline)

    print('최적화 시작 (10분 제한, 최대 300 trials)...')
    print(f'  {"#":>3}  {"결과":>10}  {"L1_MIN+K·v":>14}  '
          f'{"τ(ms)":>10}  {"slip":>5}  {"lag":>5}  {"η":>5}')
    print('-' * 66)

    study.optimize(objective, timeout=600, n_trials=300)
    env.close()

    best   = study.best_params
    best_t = study.best_value
    n_done = len(study.trials)

    alpha_sim_best  = math.exp(-CTRL_DT    / best['TAU_STEER'])
    alpha_real_best = math.exp(-REAL_CAR_DT / best['TAU_STEER'])

    print('\n' + '=' * 66)
    print(f' 최적화 완료: {n_done}회 시도')
    print('=' * 66)
    if lapped:
        print(f'  기준 랩타임  : {t0:.3f}s')
    else:
        print(f'  기준        : 미완주')
    if best_t < PENALTY:
        print(f'  최적 랩타임 : {best_t:.3f}s')
    else:
        progress = int(N_WP - (best_t - PENALTY))
        print(f'  최적        : 미완주 (최대 {progress}개 통과)')

    print('\n  최적 파라미터:')
    for k, v in best.items():
        old = baseline[k]
        if k == 'TAU_STEER':
            print(f'    {k:12s}: {old*1000:.1f}ms → {v*1000:.1f}ms  '
                  f'(alpha_sim={alpha_sim_best:.4f}, alpha_real={alpha_real_best:.4f})')
        else:
            print(f'    {k:12s}: {old} → {v:.4f}')

    print('\n  map_controller_node.py 업데이트 중...')
    update_map_controller(best)
    print('\n  완료. sim_race.launch.py 또는 ros2 run 으로 재시작하면 적용됩니다.')


if __name__ == '__main__':
    main()
