#!/usr/bin/env python3
"""맵 이름 입력 → yaml 자동 생성 + 웨이포인트 생성 + sim.yaml 스폰 위치 자동 업데이트"""

import os, subprocess, sys, yaml
from PIL import Image

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
MAPS_DIR = os.path.join(_REPO_ROOT, 'sim_ws', 'src', 'f1tenth_gym_ros', 'maps')
WP_DIR   = _THIS_DIR
GEN_WP   = os.path.join(WP_DIR, 'generate_waypoints.py')


def ask(prompt, default):
    val = input(f'{prompt} [기본값: {default}]: ').strip()
    return val if val else str(default)


def detect_resolution(name, ready):
    target_size = Image.open(os.path.join(MAPS_DIR, f'{name}.png')).size
    for m in ready:
        try:
            ref_size = Image.open(os.path.join(MAPS_DIR, f'{m}.png')).size
            if ref_size == target_size:
                with open(os.path.join(MAPS_DIR, f'{m}.yaml')) as f:
                    cfg = yaml.safe_load(f)
                res = float(cfg['resolution'])
                print(f'  "{m}"과 이미지 크기 동일({target_size[0]}x{target_size[1]}px) → 해상도 {res} m/px 자동 적용')
                return res
        except Exception:
            continue
    return None


def make_yaml(name, resolution):
    png_path = os.path.join(MAPS_DIR, f'{name}.png')
    w, h = Image.open(png_path).size
    origin_x = round(-(w / 2) * resolution, 3)
    origin_y = round(-(h / 2) * resolution, 3)
    cfg = {
        'image':           f'{name}.png',
        'resolution':      resolution,
        'origin':          [origin_x, origin_y, 0.0],
        'negate':          0,
        'occupied_thresh': 0.65,
        'free_thresh':     0.196,
    }
    yaml_path = os.path.join(MAPS_DIR, f'{name}.yaml')
    with open(yaml_path, 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False)
    print(f'  yaml 생성: resolution={resolution}  origin=({origin_x}, {origin_y})')
    print(f'  → {yaml_path}')
    return origin_x, origin_y


def list_maps():
    files = os.listdir(MAPS_DIR)
    pngs  = {f[:-4] for f in files if f.endswith('.png')}
    yamls = {f[:-5] for f in files if f.endswith('.yaml')}
    return sorted(pngs & yamls), sorted(pngs - yamls)


def click_spawn(name, origin, res, w, h):
    """맵 이미지를 띄우고 클릭한 픽셀을 ROS 좌표로 변환."""
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg

    img_path = os.path.join(MAPS_DIR, f'{name}.png')
    fig, ax = plt.subplots(figsize=(8, 10))
    ax.imshow(mpimg.imread(img_path), cmap='gray', origin='upper')
    ax.set_title(
        f'{name}  —  출발선(스폰) 위치를 클릭하세요',
        fontsize=12, fontweight='bold'
    )
    ax.set_xlabel('pixel x (→)'); ax.set_ylabel('pixel y (↓)')
    plt.tight_layout()

    pts = plt.ginput(1, timeout=120)
    plt.close()

    if not pts:
        return None, None
    px_click, py_click = pts[0]
    sx = round(px_click * res + origin[0], 2)
    sy = round((h - py_click) * res + origin[1], 2)
    print(f'  클릭 픽셀 ({px_click:.0f}, {py_click:.0f})  →  ROS ({sx}, {sy})')
    return sx, sy


# ── 메인 ──────────────────────────────────────────────────────────────────
ready, no_yaml = list_maps()
all_maps = ready + no_yaml

print('\n=== 웨이포인트 생성기 ===')
print('사용 가능한 맵:')
for i, m in enumerate(all_maps, 1):
    wp     = os.path.join(MAPS_DIR, f'{m}_waypoints.csv')
    has_wp = '✓' if os.path.exists(wp) else ' '
    tag    = '' if m in ready else '  ← yaml 없음 (자동 생성됨)'
    print(f'  {i}. [{has_wp}] {m}{tag}')

print()
name = input('맵 이름 입력 (번호 또는 이름): ').strip()
if name.isdigit() and 1 <= int(name) <= len(all_maps):
    name = all_maps[int(name) - 1]
if name not in all_maps:
    print(f'오류: "{name}" 맵을 찾을 수 없습니다.')
    sys.exit(1)

# ── yaml 없으면 자동 생성 ──────────────────────────────────────────────────
if name in no_yaml:
    w, h = Image.open(os.path.join(MAPS_DIR, f'{name}.png')).size
    print(f'\n"{name}.yaml" 없음 → 자동 생성합니다.')
    print(f'  이미지 크기: {w} x {h} px')
    res = detect_resolution(name, ready)
    if res is None:
        res = float(ask('  해상도를 찾지 못했습니다. 직접 입력 (m/px)', 0.05))
    make_yaml(name, res)

# ── 맵 정보 읽기 ──────────────────────────────────────────────────────────
with open(os.path.join(MAPS_DIR, f'{name}.yaml')) as f:
    cfg = yaml.safe_load(f)
origin = cfg.get('origin', [-10, -10, 0])
res    = float(cfg.get('resolution', 0.05))
w, h   = Image.open(os.path.join(MAPS_DIR, f'{name}.png')).size

# ── 출발선 위치: 맵 이미지 클릭 ───────────────────────────────────────────
print('\n맵 이미지가 열립니다. 출발선(스폰) 위치를 클릭하세요.')
try:
    seed_x, seed_y = click_spawn(name, origin, res, w, h)
    if seed_x is None:
        print('클릭 없음 → 맵 중앙 사용')
        seed_x = round(origin[0] + (w / 2) * res, 2)
        seed_y = round(origin[1] + (h / 2) * res, 2)
except Exception as e:
    print(f'이미지 클릭 실패 ({e})')
    print('직접 입력으로 전환합니다.')
    cx = round(origin[0] + (w / 2) * res, 2)
    cy = round(origin[1] + (h / 2) * res, 2)
    seed_x = float(ask('  seed-x', cx))
    seed_y = float(ask('  seed-y', cy))

# ── 안전 여유 ─────────────────────────────────────────────────────────────
print(f'\n벽 안전 여유 (코너 충돌 시 0.6~0.8m 추천):')
safety = float(ask('  safety (m)', 0.4))

# ── 웨이포인트 생성 ────────────────────────────────────────────────────────
out = os.path.join(MAPS_DIR, f'{name}_waypoints.csv')
print(f'\n맵     : {name}')
print(f'스폰   : ({seed_x}, {seed_y})')
print(f'안전여유: {safety}m')
print(f'출력   : {out}')
print('생성 시작...\n')

result = subprocess.run([
    sys.executable, GEN_WP,
    '--map',    os.path.join(MAPS_DIR, name),
    '--out',    out,
    '--seed-x', str(seed_x),
    '--seed-y', str(seed_y),
    '--safety', str(safety),
    '--vmax',   '8.0',
    '--vmin',   '1.5',
    '--acc',    '5.0',
    '--brk',    '8.0',
    '--mu',     '0.9',
    '--opt-mode', 'laptime',
])

if result.returncode == 0:
    print(f'\n완료! → {out}')
else:
    print('\n생성 실패.')
