#!/usr/bin/env python3
"""
제어기 분포 맵 생성기 (독립 스크립트)
- 웨이포인트 CSV 로드 → 쿠빅 스플라인 곡률 계산
- 곡률 기준으로 APP / Stanley / LQR 구간 분류
- {맵}_controller.jpg 저장

실행:
  python3 generate_controller_map.py
  python3 generate_controller_map.py --map real_track
  python3 generate_controller_map.py --map real_track --high-curv 0.25 --low-curv 0.06
"""

import argparse
import csv
import os
import sys

import numpy as np

_THIS_DIR  = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
_MAPS_DIR  = os.path.join(_REPO_ROOT, 'sim_ws', 'src', 'f1tenth_gym_ros', 'maps')


# ── 맵 선택 ──────────────────────────────────────────────────────────────────
def select_map():
    files = os.listdir(_MAPS_DIR)
    maps  = sorted({f.replace('_waypoints.csv', '')
                    for f in files if f.endswith('_waypoints.csv')})
    if not maps:
        print('웨이포인트 CSV가 없습니다.')
        sys.exit(1)

    print('\n사용 가능한 맵:')
    for i, m in enumerate(maps, 1):
        print(f'  {i}. {m}')
    print()
    sel = input('맵 선택 (번호 또는 이름): ').strip()
    if sel.isdigit() and 1 <= int(sel) <= len(maps):
        return maps[int(sel) - 1]
    if sel in maps:
        return sel
    print(f'오류: "{sel}" 을 찾을 수 없습니다.')
    sys.exit(1)


# ── 웨이포인트 로드 ───────────────────────────────────────────────────────────
def load_waypoints(map_name):
    path = os.path.join(_MAPS_DIR, f'{map_name}_waypoints.csv')
    pts  = []
    with open(path) as f:
        for row in csv.DictReader(f):
            pts.append([float(row['x']), float(row['y']), float(row['speed'])])
    return np.array(pts)


# ── 쿠빅 스플라인 곡률 계산 ──────────────────────────────────────────────────
def compute_curvature(wp):
    from scipy.interpolate import CubicSpline

    pts = wp[:, :2]
    diffs = np.diff(pts, axis=0)
    ds    = np.linalg.norm(diffs, axis=1)
    s     = np.concatenate([[0.0], np.cumsum(ds)])

    s_ext = np.append(s, s[-1] + np.linalg.norm(pts[0] - pts[-1]))
    x_ext = np.append(pts[:, 0], pts[0, 0])
    y_ext = np.append(pts[:, 1], pts[0, 1])

    cs_x = CubicSpline(s_ext, x_ext, bc_type='periodic')
    cs_y = CubicSpline(s_ext, y_ext, bc_type='periodic')

    dx  = cs_x(s, 1);  dy  = cs_y(s, 1)
    ddx = cs_x(s, 2);  ddy = cs_y(s, 2)

    denom = np.maximum((dx**2 + dy**2) ** 1.5, 1e-6)
    return np.abs(dx * ddy - dy * ddx) / denom


# ── 이미지 저장 ───────────────────────────────────────────────────────────────
def save_image(map_name, wp, curvature, high_curv, low_curv):
    import yaml
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image as PILImage

    # 맵 YAML
    yaml_path = os.path.join(_MAPS_DIR, f'{map_name}.yaml')
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    resolution = float(cfg.get('resolution', 0.05))
    origin_raw = cfg.get('origin', [-10.0, -10.0, 0.0])
    ox, oy     = float(origin_raw[0]), float(origin_raw[1])

    # 맵 이미지
    img_path = None
    for ext in ['.png', '.pgm']:
        p = os.path.join(_MAPS_DIR, f'{map_name}{ext}')
        if os.path.exists(p):
            img_path = p; break
    if img_path is None:
        print('오류: 맵 이미지(.png/.pgm)를 찾을 수 없습니다.')
        sys.exit(1)

    img  = np.array(PILImage.open(img_path).convert('L'))
    h_px = img.shape[0]

    # 웨이포인트 → 픽셀 좌표
    px = (wp[:, 0] - ox) / resolution
    py = h_px - (wp[:, 1] - oy) / resolution

    # 제어기 분류
    ctrl      = np.where(curvature >= high_curv, 0,
                np.where(curvature <= low_curv,  2, 1))
    n_app     = int((ctrl == 0).sum())
    n_stanley = int((ctrl == 1).sum())
    n_lqr     = int((ctrl == 2).sum())
    total     = len(wp)

    print(f'  APP     : {n_app:4d}pts ({n_app/total*100:.1f}%)')
    print(f'  Stanley : {n_stanley:4d}pts ({n_stanley/total*100:.1f}%)')
    print(f'  LQR     : {n_lqr:4d}pts ({n_lqr/total*100:.1f}%)')

    COLORS = {0: ('red',        'APP'),
              1: ('limegreen',  'Stanley'),
              2: ('dodgerblue', 'LQR')}

    fig, (ax_map, ax_curv) = plt.subplots(
        2, 1, figsize=(12, 14),
        gridspec_kw={'height_ratios': [4, 1]})

    # ── 위: 맵 + 제어기 색상 ─────────────────────────────────────────────
    ax_map.imshow(img, cmap='gray', origin='upper', alpha=0.7)
    for cid, (color, label) in COLORS.items():
        mask = ctrl == cid
        if mask.any():
            ax_map.scatter(px[mask], py[mask],
                           c=color, s=8, label=label,
                           linewidths=0, zorder=3)
    ax_map.legend(loc='lower right', fontsize=12, framealpha=0.9)
    ax_map.set_title(
        f'{map_name}  Controller Distribution\n'
        f'APP {n_app}pts  Stanley {n_stanley}pts  LQR {n_lqr}pts  '
        f'(HIGH={high_curv}  LOW={low_curv})',
        fontsize=13)
    ax_map.axis('off')

    # ── 아래: 곡률 프로파일 ───────────────────────────────────────────────
    idx = np.arange(len(wp))
    ax_curv.plot(idx, curvature, color='gray', linewidth=1.0, label='curvature')
    ax_curv.axhline(high_curv, color='red',        linestyle='--',
                    linewidth=1.2, label=f'HIGH_CURV={high_curv}')
    ax_curv.axhline(low_curv,  color='dodgerblue', linestyle='--',
                    linewidth=1.2, label=f'LOW_CURV={low_curv}')

    ax_curv.fill_between(idx, high_curv, curvature,
                         where=curvature >= high_curv,
                         color='red', alpha=0.2)
    ax_curv.fill_between(idx, 0, curvature,
                         where=curvature <= low_curv,
                         color='dodgerblue', alpha=0.2)
    ax_curv.fill_between(idx, low_curv, curvature,
                         where=(curvature > low_curv) & (curvature < high_curv),
                         color='limegreen', alpha=0.2)

    ax_curv.set_xlim(0, len(wp))
    ax_curv.set_ylim(0, curvature.max() * 1.15)
    ax_curv.set_xlabel('Waypoint Index', fontsize=10)
    ax_curv.set_ylabel('Curvature (1/m)', fontsize=10)
    ax_curv.legend(loc='upper right', fontsize=9, framealpha=0.9)
    ax_curv.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(_MAPS_DIR, f'{map_name}_controller.jpg')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  저장: {out_path}')


# ── 메인 ─────────────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(description='제어기 분포 맵 생성기')
    p.add_argument('--map',       default=None,  help='맵 이름 (미입력 시 선택)')
    p.add_argument('--high-curv', type=float, default=0.3,
                   help='APP 임계 곡률 (기본 0.3 1/m)')
    p.add_argument('--low-curv',  type=float, default=0.05,
                   help='LQR 임계 곡률 (기본 0.05 1/m)')
    args = p.parse_args()

    map_name = args.map if args.map else select_map()

    print(f'\n맵: {map_name}  HIGH={args.high_curv}  LOW={args.low_curv}')
    print('웨이포인트 로드...')
    wp = load_waypoints(map_name)
    print(f'  {len(wp)}개 로드 완료')

    print('스플라인 곡률 계산...')
    curvature = compute_curvature(wp)
    print(f'  max={curvature.max():.3f}  mean={curvature.mean():.3f}  1/m')

    print('이미지 저장...')
    save_image(map_name, wp, curvature, args.high_curv, args.low_curv)


if __name__ == '__main__':
    main()
