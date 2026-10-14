#!/usr/bin/env python3
"""
gen_raceline_2.py — 클릭 경유점 → 장애물 회피 스무딩 → 웨이포인트 생성

열린 야외 공간(비닫힌 트랙)용. inner/outer 경계 불필요.
SLAM 맵 위에 클릭으로 경유점을 찍으면, 맵 장애물에서 밀어내며 부드럽게 정제한다.

의존 패키지: numpy, scipy, opencv-python, Pillow, matplotlib
  pip install numpy scipy opencv-python Pillow matplotlib

사용법:
  python3 gen_raceline_2.py                       # 대화형 맵 선택
  python3 gen_raceline_2.py --map outdoor_map     # 맵 지정
  python3 gen_raceline_2.py --map outdoor_map --closed   # 닫힌 루프
  python3 gen_raceline_2.py --map outdoor_map --out my_wp --vmax 8.0

조작 (GUI):
  좌클릭      경유점 추가
  우클릭 / u  마지막 점 취소
  d           전체 초기화
  c           닫힌/열린 경로 토글
  Enter       확정 → 최적화 → 저장
  q           취소
"""

import os, sys, csv, argparse, math
import numpy as np
import cv2
from PIL import Image
from scipy.interpolate import splprep, splev
import scipy.ndimage as ndimage

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.environ.get('RACE_WS'):
    _PKG_DIR = os.path.join(
        os.path.expanduser(os.environ['RACE_WS']), 'src', 'race_stack')
MAPS_DIR = os.path.join(_PKG_DIR, 'maps')
WP_DIR   = os.path.join(_PKG_DIR, 'waypoints')


def _car_min_radius(fallback=0.872):
    for name in ('.vehicle_overlay.yaml',):
        p = os.path.abspath(os.path.join(_PKG_DIR, '..', '..', 'config', name))
        if not os.path.exists(p):
            continue
        try:
            import yaml as _y
            o = _y.safe_load(open(p))
            ms = float(o['pure_pursuit']['max_steer'])
            wb = float(o['pure_pursuit']['wheelbase'])
            return wb / math.tan(ms)
        except Exception:
            pass
    return fallback

R_CAR_MIN = _car_min_radius()


# ══════════════════════════════════════════════════════════════════════════════
# 1. 맵 로드
# ══════════════════════════════════════════════════════════════════════════════
def load_map(map_name):
    """pgm/yaml → (dist_map_m, origin [2], res, img_gray [H,W], h, w)"""
    import yaml
    yaml_path = os.path.join(MAPS_DIR, map_name + '.yaml')
    if not os.path.exists(yaml_path):
        sys.exit(f'맵 없음: {yaml_path}')
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    res    = float(cfg['resolution'])
    origin = np.array(cfg['origin'][:2], dtype=float)

    img_path = None
    for ext in ('.pgm', '.png'):
        p = os.path.join(MAPS_DIR, map_name + ext)
        if os.path.exists(p):
            img_path = p; break
    if img_path is None:
        sys.exit(f'맵 이미지 없음: {MAPS_DIR}/{map_name}.pgm')

    img = np.array(Image.open(img_path).convert('L'))
    h, w = img.shape
    free  = (img > 128).astype(np.uint8)
    # 라이다 틈 메우기 — 건물 틈새로 경로가 새는 것 방지
    r_px = max(1, int(round(0.05 / res)))
    yy, xx = np.ogrid[-r_px:r_px+1, -r_px:r_px+1]
    kern = (yy*yy + xx*xx) <= r_px*r_px
    free = (~ndimage.binary_closing(~free.astype(bool), structure=kern,
                                     border_value=1)).astype(np.uint8)
    dist_map = cv2.distanceTransform(free, cv2.DIST_L2, 5) * res
    return dist_map, origin, res, img, h, w


def _px(pts, origin, res, h):
    """world [N,2] → pixel (col float, row float) [N,2]"""
    col = (pts[:, 0] - origin[0]) / res
    row = h - (pts[:, 1] - origin[1]) / res
    return np.column_stack([col, row])


def _dist_at(pts, dist_map, origin, res, h):
    px = _px(pts, origin, res, h)
    col = np.clip(np.round(px[:, 0]).astype(int), 0, dist_map.shape[1]-1)
    row = np.clip(np.round(px[:, 1]).astype(int), 0, dist_map.shape[0]-1)
    return dist_map[row, col]


# ══════════════════════════════════════════════════════════════════════════════
# 2. 스플라인 피팅 + 균일 리샘플
# ══════════════════════════════════════════════════════════════════════════════
def fit_dense(ctrl_pts, closed, step=0.02):
    """제어점 → 밀집 균일 간격 [N,2]"""
    arr = np.array(ctrl_pts, dtype=float)
    k = min(3, len(arr) - 1)
    tck, _ = splprep([arr[:, 0], arr[:, 1]], s=0, per=closed, k=k)

    u_est = np.linspace(0, 1, max(600, len(arr) * 30))
    xe, ye = splev(u_est, tck)
    pts_est = np.column_stack([xe, ye])
    if closed:
        pts_est = np.vstack([pts_est, pts_est[0]])
    total = float(np.linalg.norm(np.diff(pts_est, axis=0), axis=1).sum())

    n = max(6, int(total / step))
    u_u = np.linspace(0, 1, n, endpoint=not closed)
    xi, yi = splev(u_u, tck)
    return np.column_stack([xi, yi]), total


def resample_uniform(pts, step, closed):
    """호 길이 기준 균일 리샘플 → [N,2]"""
    seg = np.vstack([pts, pts[0]]) if closed else pts
    ds = np.linalg.norm(np.diff(seg, axis=0), axis=1)
    s  = np.concatenate([[0], np.cumsum(ds)])
    n  = max(3, int(s[-1] / step))
    s_new = np.linspace(0, s[-1], n, endpoint=not closed)
    xi = np.interp(s_new, s, seg[:, 0])
    yi = np.interp(s_new, s, seg[:, 1])
    return np.column_stack([xi, yi])


# ══════════════════════════════════════════════════════════════════════════════
# 3. 장애물 회피 스무딩
# ══════════════════════════════════════════════════════════════════════════════
def push_from_obstacles(pts, dist_map, origin, res, h,
                        safety_m, closed,
                        max_iter=80, alpha=0.25, sigma=4.0):
    """
    dist_map 그라디언트 방향으로 장애물에 가까운 점들을 밀고, 가우시안 스무딩.
    열린 경로의 양 끝점은 고정한다.
    """
    # dist_map gradient in world coordinates
    gy_px, gx_px = np.gradient(dist_map)   # [m/pixel]
    gx_w =  gx_px / res                    # ∂dist/∂world_x
    gy_w = -gy_px / res                    # ∂dist/∂world_y  (row↑ = y↓)

    def _grad(q):
        col = np.clip(np.round((q[:, 0] - origin[0]) / res).astype(int), 0, dist_map.shape[1]-1)
        row = np.clip(np.round(h - (q[:, 1] - origin[1]) / res).astype(int), 0, dist_map.shape[0]-1)
        return gx_w[row, col], gy_w[row, col]

    result = pts.copy()
    if not closed:
        p0, p1 = result[0].copy(), result[-1].copy()

    converged_at = max_iter
    for it in range(max_iter):
        d = _dist_at(result, dist_map, origin, res, h)
        mask = d < safety_m
        if not mask.any():
            converged_at = it; break

        gx, gy = _grad(result)
        gn = np.maximum(np.hypot(gx, gy), 1e-9)
        deficit = np.where(mask, safety_m - d, 0.0)
        result[:, 0] += alpha * deficit * gx / gn
        result[:, 1] += alpha * deficit * gy / gn

        # 가우시안 스무딩 (닫힌 경로: wrap-around)
        pad = int(sigma * 3)
        if closed and pad > 0:
            ex = np.r_[result[-pad:, 0], result[:, 0], result[:pad, 0]]
            ey = np.r_[result[-pad:, 1], result[:, 1], result[:pad, 1]]
            sx = ndimage.gaussian_filter1d(ex, sigma)[pad:-pad]
            sy = ndimage.gaussian_filter1d(ey, sigma)[pad:-pad]
        else:
            sx = ndimage.gaussian_filter1d(result[:, 0], sigma)
            sy = ndimage.gaussian_filter1d(result[:, 1], sigma)
        result = np.column_stack([sx, sy])

        if not closed:
            result[0], result[-1] = p0, p1

    n_fail = int((_dist_at(result, dist_map, origin, res, h) < safety_m).sum())
    if n_fail:
        print(f'  ⚠ 안전 마진({safety_m:.2f}m) 미달 점 {n_fail}/{len(result)}개 — '
              f'경유점을 장애물에서 더 멀리 그리거나 --safety 를 낮추세요')
    else:
        print(f'  장애물 회피 완료 ({converged_at+1}회 반복, 최소 여유 '
              f'{_dist_at(result, dist_map, origin, res, h).min():.2f}m)')
    return result


# ══════════════════════════════════════════════════════════════════════════════
# 4. 곡률 + 속도 프로파일
# ══════════════════════════════════════════════════════════════════════════════
def compute_kappa(pts, closed):
    if closed:
        dx  = np.roll(pts[:, 0], -1) - np.roll(pts[:, 0], 1)
        dy  = np.roll(pts[:, 1], -1) - np.roll(pts[:, 1], 1)
        ddx = np.roll(pts[:, 0], -1) - 2*pts[:, 0] + np.roll(pts[:, 0], 1)
        ddy = np.roll(pts[:, 1], -1) - 2*pts[:, 1] + np.roll(pts[:, 1], 1)
    else:
        dx  = np.gradient(pts[:, 0])
        dy  = np.gradient(pts[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
    return np.abs(dx*ddy - dy*ddx) / np.maximum((dx**2 + dy**2)**1.5, 1e-9)


def calc_vel(kappa, el, mu, v_max, v_min, a_acc, a_brk, closed):
    g = 9.81
    v_lat = np.where(kappa > 1e-4, np.sqrt(mu * g / kappa), v_max)
    v = np.minimum(np.full(len(kappa), v_max), v_lat)
    n = len(v)

    if closed:
        for _ in range(3):
            for i in range(n-1, -1, -1):
                j = (i+1) % n
                v[i] = min(v[i], math.sqrt(v[j]**2 + 2*a_brk*el[i]))
        for _ in range(3):
            for i in range(n):
                j = (i+1) % n
                v[j] = min(v[j], math.sqrt(v[i]**2 + 2*a_acc*el[i]))
    else:
        # 열린 경로: 시작·끝 감속
        v[0] = v[-1] = v_min
        for i in range(n-2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i+1]**2 + 2*a_brk*el[i]))
        for i in range(n-1):
            v[i+1] = min(v[i+1], math.sqrt(v[i]**2 + 2*a_acc*el[i]))
        for i in range(n-2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i+1]**2 + 2*a_brk*el[i]))

    v = np.clip(v, v_min, v_max)
    # 열린 경로: el은 n-1개, v는 n개 → 구간 평균속도로 주행시간 계산
    v_seg = v if closed else (v[:-1] + v[1:]) / 2
    t = float(np.sum(el / np.maximum(v_seg, 0.01)))
    print(f'  속도: {v.min():.2f}~{v.max():.2f} m/s  주행시간≈{t:.1f}s  '
          f'거리={el.sum():.1f}m')
    return v


# ══════════════════════════════════════════════════════════════════════════════
# 5. 결과 저장
# ══════════════════════════════════════════════════════════════════════════════
def save_csv(wp_path, raceline, v):
    with open(wp_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'speed'])
        for (x, y), vi in zip(raceline, v):
            w.writerow([f'{x:.4f}', f'{y:.4f}', f'{vi:.4f}'])
    print(f'  저장: {len(raceline)}pts → {wp_path}')


def save_overlay(map_name, raceline, v, wp_path, dist_map, origin, res, h, w):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection

        img_path = None
        for ext in ('.pgm', '.png'):
            p = os.path.join(MAPS_DIR, map_name + ext)
            if os.path.exists(p):
                img_path = p; break
        img = np.array(Image.open(img_path).convert('L'))
        xmin = origin[0]; xmax = origin[0] + w * res
        ymin = origin[1]; ymax = origin[1] + h * res

        fig, ax = plt.subplots(figsize=(13, 10))
        ax.imshow(img, origin='lower', extent=[xmin, xmax, ymin, ymax],
                  cmap='gray', alpha=0.75)

        # 안전 마진 등고선 (참고용)
        xs = np.linspace(xmin, xmax, dist_map.shape[1])
        ys = np.linspace(ymin, ymax, dist_map.shape[0])
        ax.contour(xs, ys, dist_map, levels=[0.3, 0.5, 1.0],
                   colors=['red','orange','yellow'], linewidths=0.5, alpha=0.4)

        pts = raceline
        segs = np.array([[[pts[i,0], pts[i,1]], [pts[i+1,0], pts[i+1,1]]]
                          for i in range(len(pts)-1)])
        lc = LineCollection(segs, cmap='plasma', linewidth=2.5, zorder=5)
        lc.set_array(v[:-1])
        lc.set_clim(v.min(), v.max())
        ax.add_collection(lc)
        fig.colorbar(lc, ax=ax, label='speed (m/s)', fraction=0.03, pad=0.02)

        ax.set_aspect('equal')
        ax.set_title(f'{map_name}  {len(raceline)}pts  '
                     f'{v.min():.1f}~{v.max():.1f} m/s', fontsize=10)
        out = os.path.splitext(wp_path)[0] + '.png'
        fig.tight_layout()
        fig.savefig(out, dpi=130, bbox_inches='tight')
        print(f'  오버레이 PNG: {out}')
        plt.close(fig)
    except Exception as e:
        print(f'  오버레이 저장 실패: {e}')


# ══════════════════════════════════════════════════════════════════════════════
# 6. GUI
# ══════════════════════════════════════════════════════════════════════════════
def gui_draw(map_name, dist_map, origin, res, img, h, w, init_closed):
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt

    xmin = origin[0]; xmax = origin[0] + w * res
    ymin = origin[1]; ymax = origin[1] + h * res

    # 기존 웨이포인트 후보 로드
    candidates = {}
    if os.path.isdir(WP_DIR):
        for fname in sorted(os.listdir(WP_DIR)):
            if fname.startswith(map_name) and fname.endswith('.csv'):
                try:
                    pts = []
                    with open(os.path.join(WP_DIR, fname)) as f:
                        for row in csv.DictReader(f):
                            pts.append([float(row['x']), float(row['y'])])
                    candidates[fname[:-4]] = np.array(pts)
                except Exception:
                    pass

    fig, ax = plt.subplots(figsize=(13, 10))
    ax.imshow(img, origin='lower', extent=[xmin, xmax, ymin, ymax],
              cmap='gray', alpha=0.75)
    # 거리변환 반투명 오버레이 — 안전마진 0.3/0.5m 등고선
    xs = np.linspace(xmin, xmax, dist_map.shape[1])
    ys = np.linspace(ymin, ymax, dist_map.shape[0])
    ax.contourf(xs, ys, dist_map, levels=[0, 0.3], colors=['red'], alpha=0.20, zorder=2)
    ax.contour (xs, ys, dist_map, levels=[0.3, 0.5, 1.0],
                colors=['red','orange','yellow'], linewidths=0.6, alpha=0.5, zorder=3)

    pal = ['lime', 'magenta', 'deepskyblue', 'yellow', 'orange']
    for i, (cname, cpts) in enumerate(candidates.items()):
        short = cname.replace(map_name + '_', '')
        ax.plot(cpts[:, 0], cpts[:, 1], '-', color=pal[i % len(pal)],
                lw=1.5, alpha=0.7, label=short, zorder=4)

    ax.set_aspect('equal')
    ax.legend(loc='upper right', fontsize=7, framealpha=0.6)

    state = {'pts': [], 'artists': [], 'closed': init_closed}

    def _title():
        mode = '닫힌' if state['closed'] else '열린'
        ax.set_title(
            f'{map_name} [{mode}]  —  L-click: 추가  R-click/u: 취소  '
            f'd: 초기화  c: 열림/닫힘 토글  Enter: 확정  q: 취소',
            fontsize=8)

    def _redraw():
        for a in state['artists']:
            try: a.remove()
            except Exception: pass
        state['artists'].clear()
        pts = state['pts']
        if not pts:
            _title(); fig.canvas.draw_idle(); return
        arr = np.array(pts)
        state['artists'].append(
            ax.scatter(arr[:, 0], arr[:, 1], c='white', edgecolors='red',
                       s=30, zorder=12, linewidths=1.2))
        if len(pts) >= 2:
            ln, = ax.plot(arr[:, 0], arr[:, 1], 'r--', lw=0.8, alpha=0.40, zorder=10)
            state['artists'].append(ln)
        if len(pts) >= 4:
            try:
                closed = state['closed']
                k = min(3, len(arr)-1)
                tck, _ = splprep([arr[:, 0], arr[:, 1]], s=0, per=closed, k=k)
                u_f = np.linspace(0, 1, max(400, len(pts)*15))
                sx, sy = splev(u_f, tck)
                if closed:
                    sx = np.r_[sx, sx[0]]; sy = np.r_[sy, sy[0]]
                prev, = ax.plot(sx, sy, 'r-', lw=2.5, zorder=11)
                state['artists'].append(prev)
            except Exception:
                pass
        _title(); fig.canvas.draw_idle()

    def on_click(ev):
        if ev.inaxes != ax: return
        if ev.button == 1:
            state['pts'].append([ev.xdata, ev.ydata])
        elif ev.button == 3 and state['pts']:
            state['pts'].pop()
        _redraw()

    def on_key(ev):
        if ev.key == 'u' and state['pts']:
            state['pts'].pop()
        elif ev.key == 'd':
            state['pts'].clear()
        elif ev.key == 'c':
            state['closed'] = not state['closed']
        elif ev.key in ('enter', 'return'):
            plt.close(fig)
        elif ev.key == 'q':
            state['pts'].clear()
            plt.close(fig)
        _redraw()

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    _title(); plt.tight_layout(); plt.show()

    return state['pts'], state['closed']


# ══════════════════════════════════════════════════════════════════════════════
# 7. main
# ══════════════════════════════════════════════════════════════════════════════
def main():
    p = argparse.ArgumentParser(description='클릭 경유점 → 장애물 회피 웨이포인트 생성')
    p.add_argument('--map',    default=None,  help='맵 이름 (생략 시 대화형 선택)')
    p.add_argument('--out',    default=None,  help='출력 웨이포인트 이름 (기본: {map}_free_wp)')
    p.add_argument('--closed', action='store_true', help='닫힌 루프 경로로 시작')
    p.add_argument('--vmax',   type=float, default=12.0, help='최대 속도 (m/s)')
    p.add_argument('--vmin',   type=float, default=1.0,  help='최소 속도 / 끝점 속도 (m/s)')
    p.add_argument('--mu',     type=float, default=0.90, help='횡가속도 한계 계산용 마찰계수')
    p.add_argument('--acc',    type=float, default=5.0,  help='가속도 (m/s²)')
    p.add_argument('--brk',    type=float, default=8.0,  help='제동 (m/s²)')
    p.add_argument('--step',   type=float, default=0.10, help='웨이포인트 간격 (m)')
    p.add_argument('--safety', type=float, default=0.40,
                   help='장애물 안전 마진 (m). 이보다 가까우면 밀어냄')
    p.add_argument('--iters',  type=int,   default=80,   help='최대 반복 횟수')
    p.add_argument('--alpha',  type=float, default=0.25, help='밀기 강도 (0~1)')
    p.add_argument('--sigma',  type=float, default=4.0,  help='스무딩 강도 (픽셀)')
    args = p.parse_args()

    # 맵 선택
    if args.map:
        map_name = args.map
    else:
        maps = sorted(f[:-4] for f in os.listdir(MAPS_DIR) if f.endswith('.pgm'))
        print('\n사용 가능한 맵:')
        for i, m in enumerate(maps, 1):
            print(f'  {i}. {m}')
        val = input('맵 이름 또는 번호: ').strip()
        map_name = maps[int(val)-1] if val.isdigit() else val

    out_name = args.out or f'{map_name}_free_wp'
    wp_path  = os.path.join(WP_DIR, out_name + '.csv')
    os.makedirs(WP_DIR, exist_ok=True)

    print(f'\n맵 로드: {map_name}')
    dist_map, origin, res, img, h, w = load_map(map_name)
    print(f'  해상도 {res:.3f}m/px  크기 {w}×{h}px  '
          f'({w*res:.1f}×{h*res:.1f}m)')
    print(f'  장애물 최소 여유 목표: {args.safety:.2f}m')

    # GUI
    ctrl_pts, closed = gui_draw(map_name, dist_map, origin, res, img, h, w,
                                 init_closed=args.closed)
    if len(ctrl_pts) < 4:
        print('경유점이 4개 미만 — 취소')
        return

    mode = '닫힌' if closed else '열린'
    print(f'\n[1] {len(ctrl_pts)}개 경유점 → {mode} 스플라인 피팅...')
    dense, total_len = fit_dense(ctrl_pts, closed, step=0.02)
    print(f'  길이≈{total_len:.1f}m  밀집점 {len(dense)}개')

    print(f'\n[2] 장애물 회피 스무딩 (safety={args.safety}m, iter≤{args.iters})...')
    smooth = push_from_obstacles(
        dense, dist_map, origin, res, h,
        safety_m=args.safety, closed=closed,
        max_iter=args.iters, alpha=args.alpha, sigma=args.sigma)

    print(f'\n[3] 균일 리샘플 (step={args.step}m)...')
    raceline = resample_uniform(smooth, args.step, closed)
    print(f'  웨이포인트 {len(raceline)}개')

    print(f'\n[4] 곡률·속도 프로파일...')
    seg = np.vstack([raceline, raceline[0]]) if closed else raceline
    el    = np.linalg.norm(np.diff(seg, axis=0), axis=1)
    kappa = compute_kappa(raceline, closed)
    v     = calc_vel(kappa, el, args.mu, args.vmax, args.vmin,
                     args.acc, args.brk, closed)

    # 차량 기하 체크
    r_act = 1.0 / float(kappa.max()) if kappa.max() > 1e-6 else float('inf')
    n_bad = int(np.sum(kappa > 1.0 / R_CAR_MIN))
    print(f'  R_min={r_act:.3f}m  (차량 한계 {R_CAR_MIN:.3f}m)', end='')
    if n_bad:
        print(f'\n  ★ 차량이 못 도는 점 {n_bad}/{len(kappa)}개 — 헤어핀을 더 크게 그릴 것')
    else:
        print(f'  여유 {r_act - R_CAR_MIN:+.3f}m ✓')

    print(f'\n[5] 저장...')
    save_csv(wp_path, raceline, v)
    save_overlay(map_name, raceline, v, wp_path, dist_map, origin, res, h, w)

    print(f'\n완료! race_script.yaml → waypoints: {out_name}')
    print(f'  {"닫힌 루프" if closed else "열린 경로 (시작→끝)"} '
          f'| {len(raceline)}pts | {el.sum():.1f}m | '
          f'{v.min():.1f}~{v.max():.1f} m/s')


if __name__ == '__main__':
    main()
