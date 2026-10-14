#!/usr/bin/env python3
"""
시뮬 실행 전 설정 스크립트
  1. 맵 선택
  2. 맵 이미지에서 출발점 클릭
  3. sim.yaml (map_path, sx, sy) 자동 업데이트
  4. safe_pure_pursuit_driver.py (WAYPOINTS_PATH) 자동 업데이트
"""

import os, sys, re, yaml
import numpy as np
from PIL import Image

_THIS_DIR  = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
MAPS_DIR   = os.path.join(_REPO_ROOT, 'sim_ws', 'src', 'f1tenth_gym_ros', 'maps')
WP_DIR     = _THIS_DIR
SIM_YAMLS  = [
    os.path.join(_REPO_ROOT, 'sim_ws', 'src', 'f1tenth_gym_ros', 'config', 'sim.yaml'),
    os.path.join(_REPO_ROOT, 'sim_ws', 'install', 'f1tenth_gym_ros', 'share', 'f1tenth_gym_ros', 'config', 'sim.yaml'),
]
PP_NODE = os.path.join(WP_DIR, 'pure_pursuit_node.py')


def list_maps():
    files = os.listdir(MAPS_DIR)
    pngs  = {f[:-4] for f in files if f.endswith('.png')}
    yamls = {f[:-5] for f in files if f.endswith('.yaml')}
    return sorted(pngs & yamls)


def click_spawn(name, origin, res, w, h):
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg

    fig, ax = plt.subplots(figsize=(8, 10))
    ax.imshow(mpimg.imread(os.path.join(MAPS_DIR, f'{name}.png')), cmap='gray', origin='upper')
    ax.set_title(f'{name}  -  1: spawn  2: direction', fontsize=12, fontweight='bold')
    ax.set_xlabel('pixel x'); ax.set_ylabel('pixel y')
    plt.tight_layout()

    pts = plt.ginput(2, timeout=120)
    plt.close()

    if len(pts) < 2:
        return None, None, None

    # 첫 번째 클릭 → 스폰 위치
    px1, py1 = pts[0]
    sx = round(px1 * res + origin[0], 2)
    sy = round((h - py1) * res + origin[1], 2)

    # 두 번째 클릭 → 방향 벡터 (첫 번째 점 기준)
    px2, py2 = pts[1]
    dx = px2 - px1
    dy = -(py2 - py1)  # 이미지 y축 반전
    stheta = round(np.arctan2(dy, dx), 4)

    print(f'  스폰: ({sx}, {sy})')
    print(f'  방향: ({dx:.1f}, {dy:.1f})  ->  stheta={stheta} rad ({np.degrees(stheta):.1f}°)')
    return sx, sy, stheta


def update_sim_yaml(name, sx, sy,stheta):
    map_path = os.path.join(MAPS_DIR, name)
    for path in SIM_YAMLS:
        if not os.path.exists(path):
            continue
        with open(path) as f:
            text = f.read()
        text = re.sub(r"map_path:.*",  f"map_path: '{map_path}'", text)
        text = re.sub(r"\bsx:.*",      f"sx: {sx}",               text)
        text = re.sub(r"\bsy:.*",      f"sy: {sy}",               text)
        text = re.sub(r"\bstheta:.*",  f"stheta: {stheta}",       text)
        with open(path, 'w') as f:
            f.write(text)
        print(f'  sim.yaml 업데이트: {path}')


def update_pure_pursuit(name):
    wp_path = os.path.join(MAPS_DIR, f'{name}_waypoints.csv')
    if not os.path.exists(wp_path):
        print(f'  ⚠ 웨이포인트 없음: {wp_path}')
        print(f'    먼저 make_waypoints.py 로 경로를 생성하세요.')
        return False
    with open(PP_NODE) as f:
        text = f.read()
    text = re.sub(
        r"WAYPOINTS_MAP\s*=\s*['\"].*['\"]",
        f"WAYPOINTS_MAP = '{name}'",
        text
    )
    with open(PP_NODE, 'w') as f:
        f.write(text)
    print(f'  pure_pursuit_node.py WAYPOINTS_MAP 업데이트: {name}')
    return True


# ── 메인 ──────────────────────────────────────────────────────────────────
maps = list_maps()

print('\n=== 시뮬 설정 ===')
print('사용 가능한 맵:')
for i, m in enumerate(maps, 1):
    wp     = os.path.join(MAPS_DIR, f'{m}_waypoints.csv')
    has_wp = '✓' if os.path.exists(wp) else ' '
    print(f'  {i}. [{has_wp}] {m}')

print()
sel = input('맵 이름 입력 (번호 또는 이름): ').strip()
if sel.isdigit() and 1 <= int(sel) <= len(maps):
    name = maps[int(sel) - 1]
else:
    name = sel
if name not in maps:
    print(f'오류: "{name}" 맵을 찾을 수 없습니다.')
    sys.exit(1)

# 맵 정보 읽기
with open(os.path.join(MAPS_DIR, f'{name}.yaml')) as f:
    cfg = yaml.safe_load(f)
origin = cfg.get('origin', [-10, -10, 0])
res    = float(cfg.get('resolution', 0.05))
w, h   = Image.open(os.path.join(MAPS_DIR, f'{name}.png')).size

# 출발점 클릭
print('\n맵 이미지가 열립니다. 출발선(스폰) 위치를 클릭하세요.')
try:
    sx, sy, stheta = click_spawn(name, origin, res, w, h)
    if sx is None:
        print('클릭 없음 → 취소')
        sys.exit(0)
except Exception as e:
    print(f'이미지 클릭 실패 ({e})')
    sys.exit(1)

# sim.yaml 업데이트
print('\nsim.yaml 업데이트 중...')
update_sim_yaml(name, sx, sy,stheta)

# safe_pure_pursuit_driver.py 업데이트
print('safe_pure_pursuit_driver.py 업데이트 중...')
ok = update_pure_pursuit(name)

print(f'\n=== 설정 완료 ===')
print(f'  맵      : {name}')
print(f'  스폰    : ({sx}, {sy}, {stheta})')
if ok:
    print(f'  웨이포인트: {name}_waypoints.csv')
print(f'\ncolcon build 중...')
import subprocess
result = subprocess.run(
    ['colcon', 'build', '--packages-select', 'f1tenth_racing'],
    cwd=os.path.join(_REPO_ROOT, 'f1tenth_ws'),
)
if result.returncode == 0:
    print('  빌드 완료 — source install/setup.bash 는 터미널에서 직접 실행하세요.')
else:
    print('  ⚠ 빌드 실패 — 수동으로 colcon build 를 실행하세요.')

print(f'\n이제 시뮬을 실행하세요:')
print(f'  터미널 1: cd ~/f1tenth/f1TENTH/sim_ws && source install/setup.bash && ros2 launch f1tenth_gym_ros gym_bridge_launch.py')
print(f'  터미널 2: cd ~/f1tenth/f1TENTH/f1tenth_ws && source install/setup.bash && ros2 run f1tenth_racing pure_pursuit_node')
