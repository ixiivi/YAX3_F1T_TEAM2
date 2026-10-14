#!/usr/bin/env python3
"""
plot_waypoints.py — 웨이포인트 CSV를 맵 이미지 위에 그려 시각적으로 확인.

인자 없이 실행하면 race_script.yaml의 map_name / waypoints 를 그대로 사용:

    python3 plot_waypoints.py                     # yaml 기본값
    python3 plot_waypoints.py --map track_2nd_clean --wp track_2nd_clean_wp
    python3 plot_waypoints.py --out /tmp/check.png

출력: src/race_stack/waypoints/<wp>_on_<map>.png
  - 점 색상 = CSV speed 열 (컬러바 참조, speed 없으면 단색)
  - ★ = 시작점(0번), 화살표 = 진행 방향, 30점마다 인덱스 표시
"""
import argparse
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')          # 디스플레이 없이(SSH) 파일로만 저장
import matplotlib.pyplot as plt
import yaml
from PIL import Image

REPO_DIR = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR = os.path.join(REPO_DIR, 'src', 'race_stack', 'maps')
WPS_DIR  = os.path.join(REPO_DIR, 'src', 'race_stack', 'waypoints')


def load_defaults():
    """race_script.yaml에서 map_name / waypoints 기본값을 읽는다."""
    try:
        cfg = yaml.safe_load(open(os.path.join(REPO_DIR, 'race_script.yaml')))
        return cfg.get('map_name', ''), cfg.get('waypoints', '')
    except OSError:
        return '', ''


def load_waypoints(path):
    """CSV(x, y[, speed]) → (N,3) 배열. speed 없으면 NaN."""
    pts = []
    with open(path) as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith('#'):
                continue
            try:
                x, y = float(row[0]), float(row[1])
                v = float(row[2]) if len(row) > 2 else float('nan')
                pts.append((x, y, v))
            except (ValueError, IndexError):
                continue
    return np.array(pts)


def main():
    def_map, def_wp = load_defaults()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default=def_map, help=f'맵 이름 (기본: {def_map or "없음"})')
    ap.add_argument('--wp',  default=def_wp,  help=f'웨이포인트 이름 (기본: {def_wp or "없음"})')
    ap.add_argument('--out', default='',      help='출력 이미지 경로 (기본: waypoints/<wp>_on_<map>.png)')
    args = ap.parse_args()

    if not args.map or not args.wp:
        sys.exit('맵/웨이포인트 이름이 없습니다. --map, --wp 를 지정하세요.')

    # ── 맵 로드 ────────────────────────────────────────────────
    map_yaml = os.path.join(MAPS_DIR, args.map + '.yaml')
    if not os.path.exists(map_yaml):
        sys.exit(f'맵 yaml 없음: {map_yaml}')
    mcfg = yaml.safe_load(open(map_yaml))
    res      = float(mcfg['resolution'])
    ox, oy   = float(mcfg['origin'][0]), float(mcfg['origin'][1])

    # 이미지는 yaml의 image: 필드가 가리키는 파일 (map_server와 동일)
    # — 같은 이름의 png가 있어도 해상도가 다를 수 있으므로 추측하지 않는다
    img_path = os.path.join(os.path.dirname(map_yaml), mcfg['image'])
    if not os.path.exists(img_path):
        sys.exit(f'맵 이미지 없음: {img_path} (yaml image: 필드 확인)')
    img = np.asarray(Image.open(img_path).convert('L'))
    H, W = img.shape

    # ── 웨이포인트 로드 ─────────────────────────────────────────
    wp_csv = os.path.join(WPS_DIR, args.wp + '.csv')
    if not os.path.exists(wp_csv):
        sys.exit(f'웨이포인트 CSV 없음: {wp_csv}')
    wps = load_waypoints(wp_csv)
    if len(wps) == 0:
        sys.exit(f'웨이포인트가 비어 있음: {wp_csv}')

    # ── world(m) → pixel 변환 (stanley_controller._world_to_img 과 동일 규약) ──
    px = (wps[:, 0] - ox) / res
    py = H - (wps[:, 1] - oy) / res

    out_of_map = np.sum((px < 0) | (px >= W) | (py < 0) | (py >= H))

    # ── 그리기 ─────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 10 * H / W))
    ax.imshow(img, cmap='gray', origin='upper',
              extent=[0, W, H, 0], interpolation='nearest')

    has_speed = not np.isnan(wps[:, 2]).all()
    if has_speed:
        sc = ax.scatter(px, py, c=wps[:, 2], cmap='RdYlGn_r', s=14, zorder=3)
        cb = fig.colorbar(sc, ax=ax, fraction=0.04, pad=0.02)
        cb.set_label('speed (m/s)')
    else:
        ax.scatter(px, py, color='deepskyblue', s=14, zorder=3)

    # 경로 연결선 (닫힌 루프)
    ax.plot(np.append(px, px[0]), np.append(py, py[0]),
            color='deepskyblue', lw=0.8, alpha=0.6, zorder=2)

    # 시작점 + 진행 방향 화살표
    ax.scatter([px[0]], [py[0]], marker='*', s=250, color='magenta',
               edgecolor='black', zorder=5, label='start (idx 0)')
    if len(wps) > 3:
        ax.annotate('', xy=(px[3], py[3]), xytext=(px[0], py[0]),
                    arrowprops=dict(arrowstyle='-|>', color='magenta', lw=2),
                    zorder=5)

    # 30점마다 인덱스 표시
    for i in range(0, len(wps), 30):
        ax.text(px[i] + 2, py[i] - 2, str(i), fontsize=7,
                color='blue', zorder=4)

    v_info = (f'  speed {np.nanmin(wps[:, 2]):.1f}~{np.nanmax(wps[:, 2]):.1f} m/s'
              if has_speed else '')
    ax.set_title(f'{args.wp}.csv on {args.map}  ({len(wps)} pts{v_info})')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_xlim(0, W); ax.set_ylim(H, 0)

    out = args.out or os.path.join(WPS_DIR, f'{args.wp}_on_{args.map}.png')
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f'저장: {out}')
    print(f'웨이포인트 {len(wps)}개, 맵 {W}x{H}px @ {res}m/px, origin=({ox}, {oy})')
    if out_of_map:
        print(f'⚠ 맵 범위를 벗어난 점 {out_of_map}개 — 맵과 웨이포인트 짝이 맞는지 확인!')


if __name__ == '__main__':
    main()
