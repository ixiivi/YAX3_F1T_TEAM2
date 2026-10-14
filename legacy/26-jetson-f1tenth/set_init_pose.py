#!/usr/bin/env python3
"""
set_init_pose.py — 맵 이미지 위에서 AMCL 초기 포즈(위치+yaw)를 직접 골라
                   race_script.yaml 의 localization.init_x/init_y/init_yaw 에 써넣는다.

초기 포즈는 맵 yaml 이 아니라 race_script.yaml 의 [localization] 에 있다.
(race.launch.py 가 이 값을 AMCL 의 initial_pose.x/y/yaw 로 주입한다)

──────────────────────────────────────────────────────────────
사용법 3가지 — 상황에 맞게 하나만 쓰면 된다
──────────────────────────────────────────────────────────────
1) 미리보기 이미지로 좌표 읽기 (디스플레이 없이, SSH 에서)

       python3 set_init_pose.py                    # 격자 미리보기 PNG 저장
       → /tmp/init_pose_<map>.png 를 열어 축(미터)에서 원하는 지점을 읽는다
       → 현재 설정된 초기 포즈가 빨간 화살표로 같이 그려진다

2) 좌표를 알았으면 바로 지정 (--write 없으면 미리보기만, 파일 안 건드림)

       python3 set_init_pose.py --x 12.2 --y 3.5 --yaw -152 --deg
       python3 set_init_pose.py --x 12.2 --y 3.5 --yaw -152 --deg --write

   yaw 를 각도로 재기 귀찮으면 "바라볼 지점"으로 줘도 된다:

       python3 set_init_pose.py --x 12.2 --y 3.5 --to-x 10.0 --to-y 2.4 --write

   픽셀 좌표로 주고 싶으면 (--px/--py, 이미지 왼쪽위가 0,0):

       python3 set_init_pose.py --px 340 --py 40 --to-px 300 --to-py 60 --write

3) 웨이포인트에 그냥 붙이기 (가장 안전한 기본값)

       python3 set_init_pose.py --wp 0 --write     # 0번 웨이포인트 + 진행방향 yaw

──────────────────────────────────────────────────────────────
4) 클릭으로 찍기 (젯슨에 모니터가 붙어 있거나 X11 forwarding 될 때만)

       python3 set_init_pose.py --gui
       왼쪽 클릭 1회 = 위치,  2회째 = 바라볼 방향
       w = race_script.yaml 에 쓰기,  r = 다시,  q = 취소

좌표 규약: map 프레임 (m). yaw 는 +x 축 기준 반시계(rad).
        pixel → world :  x = ox + px*res,  y = oy + (H - py)*res
"""
import argparse
import csv
import math
import os
import re
import sys

import numpy as np
import yaml
from PIL import Image

REPO_DIR   = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR   = os.path.join(REPO_DIR, 'src', 'race_stack', 'maps')
WPS_DIR    = os.path.join(REPO_DIR, 'src', 'race_stack', 'waypoints')
SCRIPT_YML = os.path.join(REPO_DIR, 'race_script.yaml')


# ── 로드 ────────────────────────────────────────────────────────
def load_cfg():
    try:
        return yaml.safe_load(open(SCRIPT_YML)) or {}
    except OSError:
        return {}


def load_map(map_name):
    """맵 yaml + 이미지 → (img, res, ox, oy, H, W)"""
    map_yaml = os.path.join(MAPS_DIR, map_name + '.yaml')
    if not os.path.exists(map_yaml):
        sys.exit(f'맵 yaml 없음: {map_yaml}')
    mcfg = yaml.safe_load(open(map_yaml))
    res = float(mcfg['resolution'])
    ox, oy = float(mcfg['origin'][0]), float(mcfg['origin'][1])
    # map_server 와 같은 파일을 본다 — 같은 이름의 png 가 있어도 추측하지 않는다
    img_path = os.path.join(os.path.dirname(map_yaml), mcfg['image'])
    if not os.path.exists(img_path):
        sys.exit(f'맵 이미지 없음: {img_path} (yaml image: 필드 확인)')
    img = np.asarray(Image.open(img_path).convert('L'))
    H, W = img.shape
    return img, res, ox, oy, H, W


def load_waypoints(name):
    """CSV(x,y[,speed]) → (N,2). 없으면 None."""
    path = os.path.join(WPS_DIR, name + '.csv')
    if not name or not os.path.exists(path):
        return None
    pts = []
    with open(path) as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith('#'):
                continue
            try:
                pts.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                continue
    return np.array(pts) if pts else None


# ── 좌표 변환 ───────────────────────────────────────────────────
def px_to_world(px, py, res, ox, oy, H):
    return ox + px * res, oy + (H - py) * res


def world_to_px(x, y, res, ox, oy, H):
    return (x - ox) / res, H - (y - oy) / res


def cell_value(img, x, y, res, ox, oy, H, W):
    """해당 world 좌표의 픽셀 밝기 (없으면 None). 밝을수록 자유공간."""
    px, py = world_to_px(x, y, res, ox, oy, H)
    ix, iy = int(round(px)), int(round(py))
    if 0 <= ix < W and 0 <= iy < H:
        return int(img[iy, ix])
    return None


# ── race_script.yaml 쓰기 (주석 보존을 위해 줄 단위 치환) ────────
def write_values(pairs, dry=False):
    """pairs: [(key, value_str), ...] — 'key:' 로 시작하는 첫 줄의 값만 바꾼다."""
    with open(SCRIPT_YML) as f:
        lines = f.readlines()

    done = {}
    for key, val in pairs:
        pat = re.compile(rf'^(\s*{re.escape(key)}:\s*)([^\s#]+)(.*)$')
        for i, ln in enumerate(lines):
            m = pat.match(ln)
            if m:
                lines[i] = f'{m.group(1)}{val}{m.group(3)}\n'
                done[key] = (m.group(2), val, i + 1)
                break
        else:
            print(f'⚠ race_script.yaml 에서 "{key}:" 를 못 찾음 — 건너뜀')

    if not dry:
        with open(SCRIPT_YML, 'w') as f:
            f.writelines(lines)
    for key, (old, new, ln) in done.items():
        print(f'  {SCRIPT_YML}:{ln}  {key}: {old} → {new}')
    return done


# ── 그리기 ──────────────────────────────────────────────────────
def draw(img, res, ox, oy, H, W, wps, poses, title, out, show=False):
    """poses: [(x, y, yaw, color, label), ...] — extent 를 미터로 줘서 축에서 좌표를 바로 읽는다."""
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    x0, x1 = ox, ox + W * res
    y0, y1 = oy, oy + H * res

    fig, ax = plt.subplots(figsize=(12, max(4.0, 12 * (y1 - y0) / (x1 - x0))))
    ax.imshow(np.flipud(img), cmap='gray', origin='lower',
              extent=[x0, x1, y0, y1], interpolation='nearest')

    if wps is not None:
        ax.plot(np.append(wps[:, 0], wps[0, 0]), np.append(wps[:, 1], wps[0, 1]),
                color='deepskyblue', lw=1.0, alpha=0.7, zorder=2, label='waypoints')
        ax.scatter([wps[0, 0]], [wps[0, 1]], marker='*', s=180, color='deepskyblue',
                   edgecolor='black', zorder=3)

    arrow_len = 0.9
    for (x, y, yaw, color, label) in poses:
        ax.scatter([x], [y], s=90, color=color, edgecolor='black', zorder=6, label=label)
        ax.annotate('', xy=(x + arrow_len * math.cos(yaw), y + arrow_len * math.sin(yaw)),
                    xytext=(x, y), zorder=6,
                    arrowprops=dict(arrowstyle='-|>', color=color, lw=2.5))

    # 1m 격자 — 이 눈금에서 좌표를 읽는다
    ax.set_xticks(np.arange(math.ceil(x0), x1, 1.0))
    ax.set_yticks(np.arange(math.ceil(y0), y1, 1.0))
    ax.grid(True, color='orange', lw=0.4, alpha=0.5)
    ax.tick_params(labelsize=7)
    ax.set_xlabel('map x (m)'); ax.set_ylabel('map y (m)')
    ax.set_title(title, fontsize=10)
    ax.legend(loc='upper right', fontsize=8)
    ax.set_aspect('equal')

    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f'미리보기 저장: {out}')
    return fig, ax, plt


# ── GUI 클릭 픽커 ───────────────────────────────────────────────
def run_gui(img, res, ox, oy, H, W, wps, cur, map_name, out, auto_write):
    state = {'pos': None, 'yaw': None, 'artists': []}

    poses = [cur + ('red', 'current init_pose')] if cur else []
    fig, ax, plt = draw(img, res, ox, oy, H, W, wps, poses,
                        f'{map_name} - click1=position, click2=heading | w=write  r=reset  q=quit',
                        out, show=True)

    def clear():
        for a in state['artists']:
            a.remove()
        state['artists'] = []
        state['pos'] = state['yaw'] = None
        fig.canvas.draw_idle()

    def on_click(ev):
        if ev.inaxes is not ax or ev.xdata is None:
            return
        if state['pos'] is None:
            state['pos'] = (ev.xdata, ev.ydata)
            state['artists'] += ax.plot([ev.xdata], [ev.ydata], 'o', ms=9,
                                        color='lime', mec='black', zorder=7)
            print(f'pos: x={ev.xdata:.3f} y={ev.ydata:.3f}  → now click the heading')
        else:
            x, y = state['pos']
            state['yaw'] = math.atan2(ev.ydata - y, ev.xdata - x)
            ann = ax.annotate('', xy=(x + 0.9 * math.cos(state['yaw']),
                                      y + 0.9 * math.sin(state['yaw'])),
                              xytext=(x, y), zorder=7,
                              arrowprops=dict(arrowstyle='-|>', color='lime', lw=2.5))
            state['artists'].append(ann)
            print(f'yaw = {state["yaw"]:.4f} rad ({math.degrees(state["yaw"]):.1f}°)'
                  f'   → press w to write race_script.yaml')
            if auto_write:
                apply_pose(x, y, state['yaw'], img, res, ox, oy, H, W, True, {})
        fig.canvas.draw_idle()

    def on_key(ev):
        if ev.key == 'r':
            clear(); print('reset — click again')
        elif ev.key == 'q':
            plt.close(fig)
        elif ev.key == 'w':
            if state['pos'] is None or state['yaw'] is None:
                print('position and heading are not both set yet')
                return
            x, y = state['pos']
            apply_pose(x, y, state['yaw'], img, res, ox, oy, H, W, True, {})
            plt.close(fig)

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()


# ── 적용 ────────────────────────────────────────────────────────
def apply_pose(x, y, yaw, img, res, ox, oy, H, W, do_write, extra):
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))     # -π~π 로 정규화
    v = cell_value(img, x, y, res, ox, oy, H, W)
    print(f'\n초기 포즈:  init_x={x:.4f}  init_y={y:.4f}  init_yaw={yaw:.4f} rad '
          f'({math.degrees(yaw):.1f}°)')
    if v is None:
        print('⚠ 이 좌표는 맵 밖입니다 — AMCL 이 수렴 못 합니다')
    elif v < 128:
        print(f'⚠ 이 좌표의 픽셀값이 {v} (어두움 = 벽/미탐색). 트랙 위인지 확인하세요')

    pairs = [('init_x', f'{x:.4f}'), ('init_y', f'{y:.4f}'), ('init_yaw', f'{yaw:.4f}')]
    if extra.get('start'):
        pairs += [('start_x', f'{x:.4f}'), ('start_y', f'{y:.4f}')]
    if extra.get('sim'):
        pairs += [('sx', f'{x:.4f}'), ('sy', f'{y:.4f}'), ('stheta', f'{yaw:.4f}')]

    if do_write:
        print('race_script.yaml 갱신:')
        write_values(pairs)
    else:
        print('(--write 를 붙이면 race_script.yaml 에 씁니다. 지금은 미리보기만)')
    return yaw


def main():
    cfg = load_cfg()
    def_map = cfg.get('map_name', '')
    def_wp = cfg.get('waypoints', '')

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default=def_map, help=f'맵 이름 (기본: {def_map or "없음"})')
    ap.add_argument('--wp-file', default=def_wp, help=f'웨이포인트 이름 (기본: {def_wp or "없음"})')
    ap.add_argument('--x', type=float); ap.add_argument('--y', type=float)
    ap.add_argument('--px', type=float, help='이미지 픽셀 x (왼쪽위 0,0)')
    ap.add_argument('--py', type=float, help='이미지 픽셀 y')
    ap.add_argument('--yaw', type=float, help='방향 (기본 rad, --deg 면 도)')
    ap.add_argument('--to-x', type=float); ap.add_argument('--to-y', type=float)
    ap.add_argument('--to-px', type=float); ap.add_argument('--to-py', type=float)
    ap.add_argument('--deg', action='store_true', help='--yaw 를 도(°) 로 해석')
    ap.add_argument('--wp', type=int, help='N번 웨이포인트 위치 + 진행방향 yaw 를 사용')
    ap.add_argument('--gui', action='store_true', help='클릭으로 찍기 (디스플레이 필요)')
    ap.add_argument('--write', action='store_true', help='race_script.yaml 에 실제로 쓰기')
    ap.add_argument('--also-start', action='store_true',
                    help='mission.start_x/start_y 도 같은 좌표로 갱신')
    ap.add_argument('--also-sim', action='store_true',
                    help='sim.sx/sy/stheta 도 같은 포즈로 갱신')
    ap.add_argument('--out', default='', help='미리보기 PNG 경로')
    args = ap.parse_args()

    if not args.map:
        sys.exit('맵 이름이 없습니다. --map 을 지정하세요.')

    img, res, ox, oy, H, W = load_map(args.map)
    wps = load_waypoints(args.wp_file)
    out = args.out or f'/tmp/init_pose_{args.map}.png'
    extra = {'start': args.also_start, 'sim': args.also_sim}

    loc = cfg.get('localization') or {}
    cur = None
    if {'init_x', 'init_y'} <= set(loc):
        cur = (float(loc['init_x']), float(loc['init_y']), float(loc.get('init_yaw', 0.0)))

    print(f'맵 {args.map}: {W}x{H}px @ {res}m/px, origin=({ox}, {oy})')
    print(f'  map 좌표 범위: x {ox:.2f}~{ox + W * res:.2f} m,  y {oy:.2f}~{oy + H * res:.2f} m')
    if cur:
        v = cell_value(img, cur[0], cur[1], res, ox, oy, H, W)
        tag = '맵 밖' if v is None else ('벽/미탐색' if v < 128 else '자유공간')
        print(f'  현재 init_pose: ({cur[0]:.3f}, {cur[1]:.3f}, {cur[2]:.3f} rad) — {tag}')

    if args.gui:
        run_gui(img, res, ox, oy, H, W, wps, cur, args.map, out, args.write)
        return

    # ── 위치 결정 ──
    x = y = yaw = None
    if args.wp is not None:
        if wps is None:
            sys.exit(f'웨이포인트를 못 읽었습니다: {args.wp_file}')
        i = args.wp % len(wps)
        j = (i + 3) % len(wps)              # 3점 앞을 보고 진행방향(접선) 계산
        x, y = wps[i]
        yaw = math.atan2(wps[j][1] - y, wps[j][0] - x)
        print(f'  웨이포인트 {i}번 사용 (진행방향은 {j}번 기준)')
    else:
        if args.px is not None and args.py is not None:
            x, y = px_to_world(args.px, args.py, res, ox, oy, H)
        elif args.x is not None and args.y is not None:
            x, y = args.x, args.y

        if args.to_px is not None and args.to_py is not None:
            tx, ty = px_to_world(args.to_px, args.to_py, res, ox, oy, H)
            if x is None:
                sys.exit('--to-px/--to-py 는 위치(--x/--y 또는 --px/--py)와 함께 쓰세요')
            yaw = math.atan2(ty - y, tx - x)
        elif args.to_x is not None and args.to_y is not None:
            if x is None:
                sys.exit('--to-x/--to-y 는 위치(--x/--y 또는 --px/--py)와 함께 쓰세요')
            yaw = math.atan2(args.to_y - y, args.to_x - x)
        elif args.yaw is not None:
            yaw = math.radians(args.yaw) if args.deg else args.yaw

    # ── 좌표가 안 주어지면 미리보기만 ──
    if x is None or yaw is None:
        poses = [cur + ('red', 'current init_pose')] if cur else []
        draw(img, res, ox, oy, H, W, wps, poses,
             f'{args.map} - grid 1m : read (x, y) of the spot you want from the axes', out)
        print('\n이미지에서 좌표를 읽은 뒤:')
        print(f'  python3 set_init_pose.py --x <X> --y <Y> --yaw <각도> --deg --write')
        print(f'  python3 set_init_pose.py --x <X> --y <Y> --to-x <앞쪽X> --to-y <앞쪽Y> --write')
        if wps is not None:
            print(f'  python3 set_init_pose.py --wp 0 --write     # 0번 웨이포인트에 붙이기')
        if x is not None and yaw is None:
            print('\n※ 위치는 받았지만 방향이 없습니다 (--yaw 또는 --to-x/--to-y 필요)')
        return

    yaw = apply_pose(x, y, yaw, img, res, ox, oy, H, W, args.write, extra)

    poses = []
    if cur:
        poses.append(cur + ('red', 'old init_pose'))
    poses.append((x, y, yaw, 'lime', 'new init_pose'))
    draw(img, res, ox, oy, H, W, wps, poses,
         f'{args.map} - init_pose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg)', out)
    if not args.write:
        print('확인 후 같은 명령에 --write 를 붙여 실행하세요.')


if __name__ == '__main__':
    main()
