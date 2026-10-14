#!/usr/bin/env python3
"""
Pure Pursuit 파라미터 자동 최적화
Optuna (TPE Bayesian) + f1tenth_gym 고속 시뮬레이터
LA_BASE, LA_K, PREVIEW_N, A_BRAKE, SPEED_SCALE 자동 탐색
"""

import numpy as np
import gym
import optuna
import csv
import re
import os
from f110_gym.envs.base_classes import Integrator

optuna.logging.set_verbosity(optuna.logging.WARNING)

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
MAP_PATH  = os.path.join(_REPO_ROOT, 'sim_ws', 'src', 'f1tenth_gym_ros', 'maps', 'real_track')
WP_PATH   = os.path.join(_THIS_DIR, 'waypoints.csv')
PP_PATH   = os.path.join(_THIS_DIR, 'pure_pursuit_node.py')
# 랩타이머 지점(6.1, 0) 위 레이싱 라인 웨이포인트 (idx=127)
# yaw=-1.5204 rad = 남쪽 방향 (레이싱 라인 접선 방향)
SPAWN     = np.array([[6.248, 0.022, -1.5204]])
TIMEOUT   = 30.0
PENALTY   = 100.0
WHEELBASE = 0.33
MAX_STEER = 0.4189
START_XY  = SPAWN[0, :2]

# ROS2 odom 콜백 주기 시뮬레이션
# gym dt=0.01s, ROS2 제어 주기 ≈ 0.03s(33Hz) → 3 스텝마다 제어 재계산
CTRL_STEPS = 3


def load_waypoints(path):
    pts = []
    with open(path) as f:
        for row in csv.DictReader(f):
            pts.append([float(row['x']), float(row['y']), float(row['speed'])])
    return np.array(pts)


def find_lookahead(wp, pos, lookahead):
    dists = np.linalg.norm(wp[:, :2] - pos, axis=1)
    near = int(np.argmin(dists))
    n = len(wp)
    arc = 0.0
    for i in range(n):
        i0 = (near + i) % n
        i1 = (near + i + 1) % n
        seg = float(np.linalg.norm(wp[i1, :2] - wp[i0, :2]))
        if arc + seg >= lookahead:
            alpha = float(np.clip((lookahead - arc) / max(seg, 1e-6), 0.0, 1.0))
            goal_xy  = (1.0 - alpha) * wp[i0, :2] + alpha * wp[i1, :2]
            goal_spd = (1.0 - alpha) * wp[i0, 2]  + alpha * wp[i1, 2]
            return goal_xy, goal_spd, near
        arc += seg
    return None, None, near


def compute_action(wp, pos, yaw, speed, params):
    lookahead = max(0.5, params['LA_BASE'] + params['LA_K'] * speed)
    goal, wp_speed, near_idx = find_lookahead(wp, pos, lookahead)
    if goal is None:
        return 0.0, 1.0

    n = len(wp)
    dist_accum = float(np.linalg.norm(wp[near_idx, :2] - pos))
    v_brake = wp_speed
    for i in range(int(params['PREVIEW_N'])):
        cur = wp[(near_idx + i) % n, :2]
        nxt = wp[(near_idx + i + 1) % n, :2]
        dist_accum += float(np.linalg.norm(nxt - cur))
        vc = wp[(near_idx + i + 1) % n, 2]
        v_brake = min(v_brake, float(np.sqrt(vc**2 + 2.0 * params['A_BRAKE'] * dist_accum)))

    target_speed = min(wp_speed, v_brake) * params['SPEED_SCALE']

    dx = goal[0] - pos[0]
    dy = goal[1] - pos[1]
    lx =  np.cos(yaw) * dx + np.sin(yaw) * dy
    ly = -np.sin(yaw) * dx + np.cos(yaw) * dy
    L = np.hypot(lx, ly)
    if L < 1e-4:
        return 0.0, target_speed
    steer = float(np.clip(np.arctan2(2.0 * WHEELBASE * ly, L * L), -MAX_STEER, MAX_STEER))
    return steer, target_speed


def run_trial(env, wp, params):
    obs, _, _, _ = env.reset(SPAWN)
    elapsed = 0.0
    departed = False
    steer, tgt_speed = 0.0, 1.0   # 초기 명령값

    step_count = 0
    while elapsed < TIMEOUT:
        pos   = np.array([obs['poses_x'][0], obs['poses_y'][0]])
        yaw   = float(obs['poses_theta'][0])
        speed = float(obs['linear_vels_x'][0])

        # CTRL_STEPS 마다 제어 재계산 (ROS2 odom 주기 시뮬레이션)
        if step_count % CTRL_STEPS == 0:
            steer, tgt_speed = compute_action(wp, pos, yaw, speed, params)

        obs, _, _, _ = env.step(np.array([[steer, tgt_speed]]))
        elapsed += 0.01
        step_count += 1

        if obs['collisions'][0]:
            return PENALTY

        dist_from_start = float(np.linalg.norm(pos - START_XY))
        if dist_from_start > 5.0:
            departed = True
        if departed and dist_from_start < 1.5:
            return elapsed

    return PENALTY


def update_pure_pursuit(best_params):
    with open(PP_PATH) as f:
        code = f.read()

    replacements = {
        'LA_BASE':     f"self.LA_BASE      = {best_params['LA_BASE']:.4f}",
        'LA_K':        f"self.LA_K         = {best_params['LA_K']:.4f}",
        'PREVIEW_N':   f"self.PREVIEW_N    = {int(best_params['PREVIEW_N'])}",
        'A_BRAKE':     f"self.A_BRAKE      = {best_params['A_BRAKE']:.4f}",
        'SPEED_SCALE': f"self.SPEED_SCALE  = {best_params['SPEED_SCALE']:.4f}",
    }
    for key, new_line in replacements.items():
        code = re.sub(rf'(self\.{key}\s*=)[^\n]*', new_line, code)

    with open(PP_PATH, 'w') as f:
        f.write(code)
    print(f'  → pure_pursuit_node.py 업데이트 완료')


def main():
    print('=' * 58)
    print(' Pure Pursuit 파라미터 최적화 (Optuna + f1tenth_gym)')
    print('=' * 58)

    wp = load_waypoints(WP_PATH)
    print(f'  웨이포인트: {len(wp)}개 로드')

    env = gym.make(
        'f110_gym:f110-v0',
        num_agents=1,
        map=MAP_PATH,
        map_ext='.png',
        integrator=Integrator.RK4,
        timestep=0.01,
    )
    print(f'  시뮬레이터 준비 완료\n')

    # 기준값(현재 파라미터) 먼저 측정
    baseline = {
        'LA_BASE': 0.5, 'LA_K': 0.20,
        'PREVIEW_N': 40, 'A_BRAKE': 9.0, 'SPEED_SCALE': 1.0,
        # CTRL_STEPS=3으로 시뮬: 실제 gym 33Hz 기준 기준 랩타임
    }
    print('기준 랩타임 측정 중...')
    t0 = run_trial(env, wp, baseline)
    print(f'  기준 랩타임: {t0:.3f}s\n')

    best_so_far = [t0]

    def objective(trial):
        params = {
            'LA_BASE':     trial.suggest_float('LA_BASE',     0.3,  1.2),
            'LA_K':        trial.suggest_float('LA_K',        0.12, 0.45),  # 최소 0.12: ROS2 안정성
            'PREVIEW_N':   trial.suggest_int  ('PREVIEW_N',   15,   80),    # 최소 15: 충분한 제동 예견
            'A_BRAKE':     trial.suggest_float('A_BRAKE',     5.0,  15.0),
            'SPEED_SCALE': trial.suggest_float('SPEED_SCALE', 0.85, 1.0),   # 최대 1.0: ROS2 안전성
        }
        t = run_trial(env, wp, params)

        marker = ''
        if t < best_so_far[0]:
            best_so_far[0] = t
            marker = '  ★ NEW BEST'
        collision = ' [충돌]' if t >= PENALTY else ''
        print(f'  [{trial.number:3d}] {t:.3f}s  '
              f'LA={params["LA_BASE"]:.2f}+{params["LA_K"]:.3f}v  '
              f'PRV={params["PREVIEW_N"]}  BRK={params["A_BRAKE"]:.1f}  '
              f'SCL={params["SPEED_SCALE"]:.3f}{collision}{marker}')
        return t

    sampler = optuna.samplers.TPESampler(seed=42)
    study = optuna.create_study(direction='minimize', sampler=sampler)
    study.enqueue_trial(baseline)

    print('최적화 시작 (10분 제한, 최대 500 trials)...')
    print(f'  {"#":>3}  {"time":>7}  {"LA_BASE+K*v":>12}  {"PRV":>4}  {"BRK":>5}  {"SCL":>6}')
    print('-' * 58)

    study.optimize(objective, timeout=600, n_trials=500)

    env.close()

    best = study.best_params
    best_t = study.best_value
    n_trials = len(study.trials)

    print('\n' + '=' * 58)
    print(f' 최적화 완료: {n_trials}회 시도')
    print('=' * 58)
    print(f'  기준 랩타임 : {t0:.3f}s')
    print(f'  최적 랩타임 : {best_t:.3f}s')
    improvement = t0 - best_t
    print(f'  개선        : {improvement:+.3f}s ({improvement/t0*100:+.1f}%)')
    print('\n  최적 파라미터:')
    for k, v in best.items():
        old = baseline[k]
        print(f'    {k:12s}: {old} → {v:.4f}')

    if best_t < t0:
        print('\n  pure_pursuit_node.py에 최적값 반영 중...')
        update_pure_pursuit(best)
        print('\n  시뮬레이터 재시작 후 실제 랩타임을 확인하세요.')
    else:
        print('\n  기준값이 이미 최적입니다. 파라미터 변경 없음.')


if __name__ == '__main__':
    main()
