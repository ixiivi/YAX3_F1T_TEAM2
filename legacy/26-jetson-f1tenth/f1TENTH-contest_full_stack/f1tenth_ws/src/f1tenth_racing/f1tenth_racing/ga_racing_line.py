#!/usr/bin/env python3
"""
유전 알고리즘 기반 레이싱 라인 최적화 (ga_racing_line.py)

Usage:
    python3 ga_racing_line.py

기준 경로 복수 지원:
    waypoints_{맵}_route1.csv, waypoints_{맵}_route2.csv 가 있으면 자동 인식.
    없으면 waypoints_{맵}.csv 단일 경로 사용.

출력:
    waypoints_{맵}_ga.csv  (pure_pursuit_node.py 호환 x,y,speed 포맷)
"""

import numpy as np
from PIL import Image
import scipy.ndimage as ndimage
from scipy.interpolate import splprep, splev
import csv, yaml, os, sys
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

_THIS_DIR  = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
_MAPS_DIR  = os.path.join(_REPO_ROOT, 'sim_ws', 'src', 'f1tenth_gym_ros', 'maps')

# ══════════════════════════════════════════════════════════════════════════════
#  GA 하이퍼파라미터
# ══════════════════════════════════════════════════════════════════════════════
POP_SIZE   = 60     # 집단 크기
N_GEN      = 200    # 세대 수
N_CTRL     = 30     # 염색체 제어점 수 (경로당)
ELITE_N    = 6      # 엘리트 보존 수
P_CROSS    = 0.80   # 교차 확률
P_MUT      = 0.15   # 제어점당 돌연변이 확률
SIGMA_MUT  = 0.10   # 돌연변이 표준편차 (m)
MAX_OFF    = 0.50   # 최대 횡방향 오프셋 (m)

# ══════════════════════════════════════════════════════════════════════════════
#  물리 파라미터
# ══════════════════════════════════════════════════════════════════════════════
A_LAT   = 4.0   # 최대 횡가속도 (m/s²)
A_ACC   = 3.0   # 최대 가속도 (m/s²)
A_BRK   = 6.0   # 최대 제동 가속도 (m/s²)
V_MAX   = 4.5   # 최대 속도 (m/s)
V_MIN   = 0.5   # 최소 속도 (m/s)
SAFE_M  = 0.30  # 벽 안전 거리 (m) — 이 이내 접근 시 패널티
PEN_W   = 30.0  # 벽 침범 패널티 가중치 (s/m)
N_DENSE = 400   # 경로 평가용 밀집 샘플 수


# ══════════════════════════════════════════════════════════════════════════════
#  맵 선택
# ══════════════════════════════════════════════════════════════════════════════
def select_map():
    files = os.listdir(_MAPS_DIR)
    pngs  = {f[:-4] for f in files if f.endswith('.png')}
    yamls = {f[:-5] for f in files if f.endswith('.yaml') and '_route' not in f}
    maps  = sorted(pngs & yamls)
    if not maps:
        print('사용 가능한 맵 없음'); sys.exit(1)

    print('\n사용 가능한 맵:')
    for i, m in enumerate(maps, 1):
        print(f'  {i}. {m}')

    sel = input('\n맵 선택 (번호 또는 이름): ').strip()
    if sel.isdigit() and 1 <= int(sel) <= len(maps):
        return maps[int(sel) - 1]
    if sel in maps:
        return sel
    print('취소'); sys.exit(0)


# ══════════════════════════════════════════════════════════════════════════════
#  맵 로드 → 거리 변환 맵 생성
# ══════════════════════════════════════════════════════════════════════════════
def load_map(map_name):
    yaml_path = os.path.join(_MAPS_DIR, f'{map_name}.yaml')
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    res  = float(cfg.get('resolution', 0.05))
    orig = cfg.get('origin', [0.0, 0.0, 0.0])
    ox, oy = float(orig[0]), float(orig[1])

    img_path = os.path.join(_MAPS_DIR, f'{map_name}.png')
    img = np.array(Image.open(img_path).convert('L'))
    h, w = img.shape

    free   = (img > 200).astype(np.uint8)
    dist_m = ndimage.distance_transform_edt(free) * res   # 벽까지 거리 (m)

    return img, dist_m, ox, oy, res, h, w


# ══════════════════════════════════════════════════════════════════════════════
#  기준 경로 로드
# ══════════════════════════════════════════════════════════════════════════════
def load_routes(map_name):
    routes = []
    idx = 1
    while True:
        p = os.path.join(_MAPS_DIR, f'{map_name}_route{idx}.csv')
        if not os.path.exists(p):
            break
        pts = _read_csv(p)
        if pts is not None:
            routes.append(pts)
            print(f'  route{idx}: {len(pts)}pts  ({p})')
        idx += 1

    if not routes:
        p = os.path.join(_MAPS_DIR, f'{map_name}_waypoints.csv')
        pts = _read_csv(p)
        if pts is None:
            print(f'웨이포인트 없음: {p}'); sys.exit(1)
        routes.append(pts)
        print(f'  단일 경로: {len(pts)}pts  ({p})')

    return routes


def _read_csv(path):
    pts = []
    try:
        with open(path) as f:
            for row in csv.DictReader(f):
                pts.append([float(row['x']), float(row['y'])])
    except Exception as e:
        print(f'CSV 오류 ({path}): {e}'); return None
    return np.array(pts) if len(pts) > 3 else None


# ══════════════════════════════════════════════════════════════════════════════
#  기준 스플라인 구축
# ══════════════════════════════════════════════════════════════════════════════
def build_spline(pts):
    """pts (N,2) → tck, 제어점 위치, 법선 벡터 반환"""
    if np.linalg.norm(pts[-1] - pts[0]) < 1e-6:
        pts = pts[:-1]

    x, y = pts[:, 0], pts[:, 1]
    for s in [0.5, 1.0, 2.0]:
        try:
            tck, _ = splprep([x, y], per=True, s=s, k=3)
            break
        except Exception:
            continue
    else:
        raise RuntimeError('스플라인 생성 실패')

    u_ctrl = np.linspace(0, 1, N_CTRL, endpoint=False)
    cx, cy = splev(u_ctrl, tck)
    ctrl_pts = np.stack([cx, cy], axis=1)          # (N_CTRL, 2)

    dx, dy = splev(u_ctrl, tck, der=1)
    tang    = np.stack([dx, dy], axis=1)
    tang   /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-9
    normals = np.stack([-tang[:, 1], tang[:, 0]], axis=1)  # 왼쪽 법선

    return tck, ctrl_pts, normals


# ══════════════════════════════════════════════════════════════════════════════
#  개체 (Individual)
# ══════════════════════════════════════════════════════════════════════════════
class Individual:
    def __init__(self, base_idx, offsets):
        self.base_idx = int(base_idx)
        self.offsets  = np.clip(np.asarray(offsets, dtype=float), -MAX_OFF, MAX_OFF)
        self.fitness  = None
        self.lap_time = float('inf')

    def copy(self):
        c = Individual(self.base_idx, self.offsets.copy())
        c.fitness  = self.fitness
        c.lap_time = self.lap_time
        return c


# ══════════════════════════════════════════════════════════════════════════════
#  경로 전개: 염색체 → 밀집 경로 좌표
# ══════════════════════════════════════════════════════════════════════════════
def expand_path(ind, splines):
    _, ctrl_pts, normals = splines[ind.base_idx]
    pts = ctrl_pts + ind.offsets[:, None] * normals   # 오프셋 적용

    for s in [0.3, 0.8, 2.0]:
        try:
            new_tck, _ = splprep([pts[:, 0], pts[:, 1]], per=True, s=s, k=3)
            break
        except Exception:
            continue
    else:
        return None

    u = np.linspace(0, 1, N_DENSE, endpoint=False)
    ox, oy = splev(u, new_tck)
    return np.stack([ox, oy], axis=1)


# ══════════════════════════════════════════════════════════════════════════════
#  곡률 계산
# ══════════════════════════════════════════════════════════════════════════════
def compute_curvature(pts):
    p0 = np.roll(pts,  1, axis=0)
    p1 = pts
    p2 = np.roll(pts, -1, axis=0)
    dx  = (p2[:, 0] - p0[:, 0]) * 0.5
    dy  = (p2[:, 1] - p0[:, 1]) * 0.5
    ddx = p2[:, 0] - 2 * p1[:, 0] + p0[:, 0]
    ddy = p2[:, 1] - 2 * p1[:, 1] + p0[:, 1]
    return np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2 + 1e-9) ** 1.5


# ══════════════════════════════════════════════════════════════════════════════
#  속도 프로파일 계산 (전방-후방 통합)
# ══════════════════════════════════════════════════════════════════════════════
def speed_profile(pts):
    ds    = np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)
    kappa = compute_curvature(pts)

    v = np.clip(np.sqrt(A_LAT / (kappa + 1e-9)), V_MIN, V_MAX)

    for _ in range(3):
        # 후방 패스 — 제동 한계
        for i in range(len(v) - 1, -1, -1):
            j = (i + 1) % len(v)
            v[i] = min(v[i], (v[j]**2 + 2 * A_BRK * ds[i]) ** 0.5)
        # 전방 패스 — 가속 한계
        for i in range(len(v)):
            j = (i + 1) % len(v)
            v[j] = min(v[j], (v[i]**2 + 2 * A_ACC * ds[i]) ** 0.5)

    v = np.clip(v, V_MIN, V_MAX)
    return v, float(np.sum(ds / v))


# ══════════════════════════════════════════════════════════════════════════════
#  벽 거리 조회
# ══════════════════════════════════════════════════════════════════════════════
def get_wall_dists(pts, dist_m, ox, oy, res, h):
    px = np.clip(((pts[:, 0] - ox) / res).astype(int), 0, dist_m.shape[1] - 1)
    py = np.clip((h - (pts[:, 1] - oy) / res).astype(int), 0, dist_m.shape[0] - 1)
    return dist_m[py, px]


# ══════════════════════════════════════════════════════════════════════════════
#  적합도 평가
#  score = -(추정랩타임 + 벽침범패널티)   높을수록 좋음
# ══════════════════════════════════════════════════════════════════════════════
def evaluate(ind, splines, dist_m, ox, oy, res, h):
    pts = expand_path(ind, splines)
    if pts is None:
        ind.fitness = -1e6; return

    # 벽 침범 패널티: 안전 거리 이내 접근량 합산
    dists      = get_wall_dists(pts, dist_m, ox, oy, res, h)
    violations = np.maximum(0.0, SAFE_M - dists)
    wall_pen   = float(violations.sum()) * PEN_W

    v, lap_time = speed_profile(pts)
    ind.lap_time = lap_time
    ind.fitness  = -(lap_time + wall_pen)


# ══════════════════════════════════════════════════════════════════════════════
#  GA 연산자
# ══════════════════════════════════════════════════════════════════════════════
def tournament(pop, k=3):
    idxs = np.random.choice(len(pop), k, replace=False)
    return pop[int(max(idxs, key=lambda i: pop[i].fitness))].copy()


def crossover(p1, p2, n_routes):
    child_base = p1.base_idx if np.random.rand() < 0.5 else p2.base_idx
    if np.random.rand() > P_CROSS:
        src = p1 if child_base == p1.base_idx else p2
        return Individual(child_base, src.offsets.copy())
    mask = np.random.rand(N_CTRL) < 0.5
    off  = np.where(mask, p1.offsets, p2.offsets)
    return Individual(child_base, off)


def mutate(ind):
    mask = np.random.rand(N_CTRL) < P_MUT
    ind.offsets += mask * np.random.normal(0, SIGMA_MUT, N_CTRL)
    ind.offsets  = np.clip(ind.offsets, -MAX_OFF, MAX_OFF)
    ind.fitness  = None
    ind.lap_time = float('inf')


# ══════════════════════════════════════════════════════════════════════════════
#  ROS 좌표 → 픽셀 변환 (시각화용)
# ══════════════════════════════════════════════════════════════════════════════
def to_px(pts, ox, oy, res, h):
    px = (pts[:, 0] - ox) / res
    py = h - (pts[:, 1] - oy) / res
    return px, py


def draw_speed_path(ax, pts, ox, oy, res, h, lw=2.5, alpha=1.0, zorder=10):
    """연두(느림) → 빨강(빠름) 그라데이션으로 경로 그리기. artist 반환."""
    v, _ = speed_profile(pts)
    px, py = to_px(pts, ox, oy, res, h)
    px_c = np.append(px, px[0])
    py_c = np.append(py, py[0])
    v_c  = np.append(v, v[0])
    points   = np.array([px_c, py_c]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm = plt.Normalize(V_MIN, V_MAX)
    lc   = LineCollection(segments, cmap='RdYlGn_r', norm=norm,
                          linewidth=lw, alpha=alpha, zorder=zorder)
    lc.set_array(v_c)
    ax.add_collection(lc)
    return lc


# ══════════════════════════════════════════════════════════════════════════════
#  GA 메인 루프
# ══════════════════════════════════════════════════════════════════════════════
def ga_loop(splines, dist_m, ox, oy, res, h, img, n_routes):
    # 초기 집단 — 경로별 균등 분배
    pop = []
    for i in range(POP_SIZE):
        base = i % n_routes
        off  = np.random.uniform(-MAX_OFF * 0.2, MAX_OFF * 0.2, N_CTRL)
        pop.append(Individual(base, off))

    # 시각화 초기화
    plt.ion()
    fig, (ax_map, ax_fit) = plt.subplots(1, 2, figsize=(15, 7))
    fig.suptitle('유전 알고리즘 레이싱 라인 최적화', fontsize=13)

    ax_map.imshow(img, cmap='gray', origin='upper', alpha=0.75)
    ax_map.axis('off')

    # 속도 컬러바 (연두=느림, 빨강=빠름)
    sm = plt.cm.ScalarMappable(cmap='RdYlGn_r', norm=plt.Normalize(V_MIN, V_MAX))
    sm.set_array([])
    fig.colorbar(sm, ax=ax_map, label='속도 (m/s)', shrink=0.6, pad=0.01)

    ax_fit.set_xlabel('세대')
    ax_fit.set_ylabel('추정 랩타임 (s) — 낮을수록 우수')
    ax_fit.grid(True, alpha=0.3)

    route_colors  = ['#cc3333', '#3366cc', '#33aa33', '#cc8800']
    sub_size      = POP_SIZE // n_routes          # 경로별 서브 집단 크기
    elite_sub     = max(1, ELITE_N // n_routes)   # 경로별 엘리트 수

    # 경로별 서브 집단 초기화
    pops = []
    for r in range(n_routes):
        sub = []
        for _ in range(sub_size):
            off = np.random.uniform(-MAX_OFF * 0.2, MAX_OFF * 0.2, N_CTRL)
            sub.append(Individual(r, off))
        pops.append(sub)

    path_artists  = []
    best_by_route = [None] * n_routes
    hist_by_route = [[] for _ in range(n_routes)]

    for gen in range(N_GEN):
        # ── 경로별 서브 집단 독립 진화 ──
        for r, pop in enumerate(pops):
            # 평가
            for ind in pop:
                if ind.fitness is None:
                    evaluate(ind, splines, dist_m, ox, oy, res, h)

            pop.sort(key=lambda x: x.fitness, reverse=True)

            # 베스트 갱신
            if best_by_route[r] is None or pop[0].fitness > best_by_route[r].fitness:
                best_by_route[r] = pop[0].copy()

            # 다음 세대 생성 (서브 집단 내에서만 교차)
            next_sub = [pop[i].copy() for i in range(elite_sub)]
            while len(next_sub) < sub_size:
                p1    = tournament(pop)
                p2    = tournament(pop)
                child = crossover(p1, p2, n_routes)
                child.base_idx = r   # 서브 집단 경로 고정
                mutate(child)
                next_sub.append(child)
            pops[r] = next_sub

        # ── 통계 기록 ──
        for r in range(n_routes):
            t = best_by_route[r].lap_time if best_by_route[r] else float('inf')
            hist_by_route[r].append(t)

        status = '  '.join(
            f'r{r}={best_by_route[r].lap_time:.3f}s' if best_by_route[r] else f'r{r}=-'
            for r in range(n_routes))
        print(f'Gen {gen+1:3d}/{N_GEN}  {status}', end='\r', flush=True)

        # ── 시각화 업데이트 ──
        for a in path_artists:
            a.remove()
        path_artists.clear()

        # 서브 집단 경로 (희미한 회색)
        for r, pop in enumerate(pops):
            for ind in pop[:8]:
                pts = expand_path(ind, splines)
                if pts is None: continue
                px, py = to_px(pts, ox, oy, res, h)
                ln, = ax_map.plot(np.append(px, px[0]), np.append(py, py[0]),
                                  color='gray', alpha=0.08, linewidth=0.8)
                path_artists.append(ln)

        # 경로별 베스트 (속도 그라데이션)
        for r in range(n_routes):
            if best_by_route[r] is None: continue
            pts = expand_path(best_by_route[r], splines)
            if pts is None: continue
            lc = draw_speed_path(ax_map, pts, ox, oy, res, h, lw=2.5, zorder=10)
            path_artists.append(lc)
            # 경로 번호 레이블 (시작점)
            px, py = to_px(pts, ox, oy, res, h)
            ln, = ax_map.plot(px[0], py[0], 'o', markersize=7,
                              color=route_colors[r % len(route_colors)],
                              zorder=11, label=f'route{r} {best_by_route[r].lap_time:.3f}s')
            path_artists.append(ln)

        best_str = '  '.join(
            f'route{r}:{best_by_route[r].lap_time:.3f}s' if best_by_route[r] else ''
            for r in range(n_routes))
        ax_map.set_title(f'Gen {gen+1}/{N_GEN}   {best_str}', fontsize=10)

        gens = list(range(1, gen + 2))
        ax_fit.cla()
        for r in range(n_routes):
            col = route_colors[r % len(route_colors)]
            t   = best_by_route[r].lap_time if best_by_route[r] else float('inf')
            ax_fit.plot(gens, hist_by_route[r], color=col, linewidth=1.5,
                        label=f'route{r}  {t:.3f}s')
        ax_fit.set_xlabel('세대')
        ax_fit.set_ylabel('추정 랩타임 (s)')
        ax_fit.legend(fontsize=9)
        ax_fit.grid(True, alpha=0.3)
        ax_fit.invert_yaxis()

        fig.canvas.draw()
        plt.pause(0.001)

    print(f'\n\n완료!')
    for r in range(n_routes):
        if best_by_route[r]:
            print(f'  route{r}: {best_by_route[r].lap_time:.3f}s')

    best_str = '  '.join(
        f'route{r}:{best_by_route[r].lap_time:.3f}s' if best_by_route[r] else ''
        for r in range(n_routes))
    ax_map.set_title(f'[완료]  {best_str}', fontsize=11, color='darkred')
    fig.canvas.draw()

    best_ever = min((b for b in best_by_route if b), key=lambda x: x.lap_time)
    return best_ever, best_by_route, fig


# ══════════════════════════════════════════════════════════════════════════════
#  결과 저장
# ══════════════════════════════════════════════════════════════════════════════
def save_result(best_by_route, splines, map_name):
    for r, best in enumerate(best_by_route):
        if best is None: continue
        pts = expand_path(best, splines)
        if pts is None: continue
        v, lap_time = speed_profile(pts)
        out = os.path.join(_MAPS_DIR, f'{map_name}_ga_route{r}_waypoints.csv')
        with open(out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y', 'speed'])
            for i in range(len(pts)):
                w.writerow([round(float(pts[i, 0]), 4),
                            round(float(pts[i, 1]), 4),
                            round(float(v[i]),      4)])
        print(f'저장: {out}  ({lap_time:.3f}s, {len(pts)}pts)')


# ══════════════════════════════════════════════════════════════════════════════
#  시각화 최종 저장
# ══════════════════════════════════════════════════════════════════════════════
def save_figure(fig, map_name):
    out = os.path.join(_MAPS_DIR, f'{map_name}_ga.jpg')
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f'시각화 저장: {out}')


# ══════════════════════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════════════════════
def main():
    print('=== 유전 알고리즘 레이싱 라인 최적화 ===')
    print(f'설정: 개체수={POP_SIZE}, 세대={N_GEN}, 제어점={N_CTRL}, 최대오프셋=±{MAX_OFF}m\n')

    map_name = select_map()

    print(f'\n[1/3] 맵 로드: {map_name}')
    img, dist_m, ox, oy, res, h, w = load_map(map_name)
    print(f'  해상도={res}m/px  원점=({ox},{oy})  크기={w}x{h}px')

    print('\n[2/3] 기준 경로 로드...')
    routes   = load_routes(map_name)
    n_routes = len(routes)
    splines  = [build_spline(r) for r in routes]
    print(f'  경로 {n_routes}개 → 스플라인 구축 완료')

    print(f'\n[3/3] GA 시작 — 창이 열리면 학습 과정을 실시간 확인할 수 있습니다.\n')
    best, best_by_route, fig = ga_loop(splines, dist_m, ox, oy, res, h, img, n_routes)

    save_result(best_by_route, splines, map_name)
    save_figure(fig, map_name)

    print('\n창을 닫으면 종료됩니다.')
    plt.ioff()
    plt.show(block=True)


if __name__ == '__main__':
    main()
