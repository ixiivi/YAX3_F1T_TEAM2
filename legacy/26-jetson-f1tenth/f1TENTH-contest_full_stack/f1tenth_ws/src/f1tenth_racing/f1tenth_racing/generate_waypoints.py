#!/usr/bin/env python3
"""
F1TENTH 레이싱 라인 생성기 v3
==============================
핵심 수정:
  - 스폰 위치 시드(seed)로 flood-fill → 외벽 바깥 배경 완전 제거
  - 트랙 폭에 비례한 최적화 범위 (좁은 미로 = 중앙 유지, 넓은 코너 = 아웃-인-아웃)
  - 충돌 검증 + 자동 수정
알고리즘:
  - 최소 곡률 최적화 (Heilmeier et al. 2019)
  - 마찰원 + 전방-후방 적분 속도 프로파일
"""

import numpy as np
from PIL import Image
import scipy.ndimage as ndimage
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
from scipy.spatial import cKDTree
import csv
import argparse
import yaml
import os
import networkx as nx

# ── 커맨드라인 인자 파싱 ───────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description='F1TENTH 레이싱 라인 생성기',
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="""
사용 예시:
  # 기본 (현재 맵)
  python3 generate_waypoints.py

  # SLAM 맵 사용
  python3 generate_waypoints.py \\
      --map ~/slam_map \\
      --seed-x 1.0 --seed-y 0.5

  # 모든 옵션 지정
  python3 generate_waypoints.py \\
      --map ~/f1TENTH/sim_ws/src/f1tenth_gym_ros/maps/slam_map \\
      --out ~/f1TENTH/f1tenth_ws/src/f1tenth_racing/f1tenth_racing/waypoints.csv \\
      --seed-x 1.0 --seed-y 0.5 \\
      --vmax 4.5 --vmin 0.8
        """)
    p.add_argument('--map',
        default=None,
        help='맵 파일 경로 (확장자 제외). 미입력 시 목록에서 선택')
    p.add_argument('--out',
        default=os.path.join(_THIS_DIR, 'waypoints.csv'),
        help='웨이포인트 출력 경로')
    p.add_argument('--seed-x', type=float, default=None,
        help='트랙 내부 시드 X 좌표 (ROS m). 미입력 시 맵 중앙 자동 탐색')
    p.add_argument('--seed-y', type=float, default=None,
        help='트랙 내부 시드 Y 좌표 (ROS m)')
    p.add_argument('--vmax',   type=float, default=4.5,  help='최대 속도 (m/s)')
    p.add_argument('--vmin',   type=float, default=0.8,  help='최소 속도 (m/s)')
    p.add_argument('--safety', type=float, default=0.25, help='벽 안전 여유 (m)')
    p.add_argument('--acc',    type=float, default=3.0,  help='최대 가속도 (m/s²)')
    p.add_argument('--brk',    type=float, default=5.0,  help='최대 제동 가속도 (m/s²)')
    p.add_argument('--mu',     type=float, default=0.8,  help='타이어 마찰 계수')
    p.add_argument('--max-width', type=float, default=0.0,
        help='트랙 최대 폭 (m). 0=제한없음(기본). F1 서킷처럼 인필드가 넓은 맵에서\n'
             '인필드 영역 제외용. 트랙 가장 넓은 구간보다 약간 크게 설정 (예: 15.0)')
    p.add_argument('--opt-mode', choices=['laptime', 'curvature'],
        default='laptime',
        help='최적화 목적: laptime=최소 랩타임(기본), curvature=최소 곡률(구버전)')
    p.add_argument('--smooth-w', type=float, default=0.05,
        help='랩타임 최적화 평활화 가중치 λ (기본 0.05)')
    p.add_argument('--restarts', type=int, default=1,
        help='랜덤 재시작 횟수 (기본 1)')
    return p.parse_args()


def select_map():
    """maps/ 폴더에서 .png + .yaml 모두 있는 맵을 나열하고 선택받는다."""
    files = os.listdir(_MAPS_DIR)
    pngs  = {f[:-4] for f in files if f.endswith('.png')}
    yamls = {f[:-5] for f in files if f.endswith('.yaml')}
    maps  = sorted(pngs & yamls)

    if not maps:
        print('사용 가능한 맵이 없습니다.')
        return None

    print('\n사용 가능한 맵:')
    for i, m in enumerate(maps, 1):
        wp  = os.path.join(_MAPS_DIR, f'{m}_waypoints.csv')
        tag = ' [웨이포인트 있음]' if os.path.exists(wp) else ''
        print(f'  {i}. {m}{tag}')

    print()
    sel = input('맵 선택 (번호 또는 이름): ').strip()
    if sel.isdigit() and 1 <= int(sel) <= len(maps):
        return maps[int(sel) - 1]
    if sel in maps:
        return sel
    print(f'오류: "{sel}" 맵을 찾을 수 없습니다.')
    return None


def load_map_config(map_base):
    """yaml 파일에서 resolution, origin 자동 읽기."""
    yaml_path = map_base + '.yaml'
    if not os.path.exists(yaml_path):
        print(f'  ⚠ yaml 없음: {yaml_path} → 기본값 사용')
        return 0.05, (-10.0, -10.0)
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    resolution = float(cfg.get('resolution', 0.05))
    origin_raw = cfg.get('origin', [-10.0, -10.0, 0.0])
    origin = (float(origin_raw[0]), float(origin_raw[1]))
    print(f'  yaml 읽기: resolution={resolution}  origin={origin}')
    return resolution, origin


_THIS_DIR  = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
_MAPS_DIR  = os.path.join(_REPO_ROOT, 'sim_ws', 'src', 'f1tenth_gym_ros', 'maps')

# 전역 설정 (parse_args()에서 채워짐)
MAP_PATH    = None
OUTPUT_PATH = None
RESOLUTION  = 0.05
ORIGIN      = (-10.0, -10.0)
SEED_ROS_X  = None
SEED_ROS_Y  = None

MU          = 0.8
G           = 9.81
A_LAT_MAX   = MU * G
A_LON_ACC   = 3.0
A_LON_BRK   = 5.0
V_MAX       = 4.5
V_MIN       = 0.8
SAFETY_M    = 0.25
N_OPT       = 200
N_FINAL     = 400

MU          = 0.8
G           = 9.81
A_LAT_MAX   = MU * G
A_LON_ACC   = 3.0
A_LON_BRK   = 5.0
V_MAX       = 4.5
V_MIN       = 0.8
SAFETY_M    = 0.25   # 벽 안전 여유 (m)
MAX_TRACK_M = 0.0    # 최대 트랙 폭 (0=제한없음). F1 서킷 인필드 제거용
N_OPT       = 200
N_FINAL     = 400
EPS_KAPPA   = 1e-4   # 목적함수 분모 0 방지용 κ 하한
OPT_MODE    = 'laptime'
SMOOTH_W    = 0.05
N_RESTARTS  = 1


# ── 좌표 변환 ──────────────────────────────────────────────────────────────
def px_to_ros(px, py, h):
    return (np.asarray(px, float) * RESOLUTION + ORIGIN[0],
            (h - np.asarray(py, float)) * RESOLUTION + ORIGIN[1])

def ros_to_px(rx, ry, h):
    px = np.round((np.asarray(rx) - ORIGIN[0]) / RESOLUTION).astype(int)
    py = np.round(h - (np.asarray(ry) - ORIGIN[1]) / RESOLUTION).astype(int)
    return px, py


# ══════════════════════════════════════════════════════════════════════════
# STEP 1 : 트랙 영역 추출 (핵심 수정)
# ══════════════════════════════════════════════════════════════════════════
def extract_track_mask(free, h):
    """
    스폰 위치를 시드로 flood-fill하여 실제 트랙 영역만 추출.
    외벽 바깥 배경 흰색을 완전히 제거함.
    """
    labeled, _ = ndimage.label(free)

    # 시드가 주어진 경우
    if SEED_ROS_X is not None and SEED_ROS_Y is not None:
        seed_px = int((SEED_ROS_X - ORIGIN[0]) / RESOLUTION)
        seed_py = int(h - (SEED_ROS_Y - ORIGIN[1]) / RESOLUTION)
        seed_px = np.clip(seed_px, 0, free.shape[1]-1)
        seed_py = np.clip(seed_py, 0, free.shape[0]-1)
        track_label = labeled[seed_py, seed_px]
    else:
        track_label = 0

    if track_label == 0:
        print('  ⚠ 시드가 벽 위에 있음 → 가장 큰 내부 영역 사용')
        counts = np.bincount(labeled.ravel())
        counts[0] = 0
        # 이미지 테두리에 닿은 레이블(배경)은 제외
        edge = set(np.unique(np.concatenate([
            labeled[0,:], labeled[-1,:], labeled[:,0], labeled[:,-1]])))
        for el in edge:
            if el < len(counts):
                counts[el] = 0
        track_label = int(counts.argmax())

    mask = (labeled == track_label).astype(np.uint8)
    print(f'  트랙 마스크: {mask.sum()}px  '
          f'({mask.sum()*RESOLUTION**2:.1f} m²)')
    return mask


# ══════════════════════════════════════════════════════════════════════════
# 그래프 기반 다중 루트 처리
# ══════════════════════════════════════════════════════════════════════════
def _build_graph(skel, h):
    """스켈레톤 → networkx 그래프.
    노드: 분기점(이웃≥3) 또는 끝점(이웃=1)
    엣지: 노드 간 픽셀 경로 + 예상 통과 시간
    """
    H, W = skel.shape
    k3   = np.array([[1,1,1],[1,0,1],[1,1,1]])
    deg  = ndimage.convolve(skel.astype(int), k3, mode='constant') * skel
    nodes = set(map(tuple, np.argwhere(skel & ((deg >= 3) | (deg == 1)))))

    G = nx.Graph()
    G.add_nodes_from(nodes)
    seen_edges = set()

    for start in nodes:
        sr, sc = start
        for dr in range(-1, 2):
            for dc in range(-1, 2):
                if dr == 0 and dc == 0: continue
                nr, nc = sr+dr, sc+dc
                if not (0 <= nr < H and 0 <= nc < W): continue
                if not skel[nr, nc]: continue

                # 이 방향으로 다음 노드까지 트레이스
                path = [start, (nr, nc)]
                prev, cur = start, (nr, nc)
                while cur not in nodes:
                    cr2, cc2 = cur
                    nexts = []
                    for dr2 in range(-1, 2):
                        for dc2 in range(-1, 2):
                            if dr2 == 0 and dc2 == 0: continue
                            nr2, nc2 = cr2+dr2, cc2+dc2
                            if (nr2, nc2) == prev: continue
                            if not (0 <= nr2 < H and 0 <= nc2 < W): continue
                            if skel[nr2, nc2]:
                                nexts.append((nr2, nc2))
                    if not nexts: break
                    prev, cur = cur, nexts[0]
                    path.append(cur)

                if cur not in nodes or cur == start: continue
                eid = frozenset({start, cur, len(path)})  # 길이로 중복 구분
                if eid in seen_edges: continue
                seen_edges.add(eid)

                # 엣지 랩타임 추정 (마찰원)
                path_arr = np.array(path)
                rx, ry = px_to_ros(path_arr[:, 1], path_arr[:, 0], h)
                kappa   = np.abs(compute_curvature(rx, ry))
                v       = np.clip(np.sqrt(A_LAT_MAX / np.maximum(kappa, 0.01)),
                                  V_MIN, V_MAX)
                ds      = np.hypot(np.diff(np.append(rx, rx[-1])),
                                   np.diff(np.append(ry, ry[-1])))
                t_edge  = float(np.sum(ds / np.maximum(v, 1e-3)))
                G.add_edge(start, cur, path=path, path_rev=list(reversed(path)),
                           time=t_edge)

    return G


def _select_fastest_loop(skel, dist_track, track, h):
    """그래프에서 모든 단순 사이클을 열거하고 예상 랩타임 최소 루프 반환.
    분기점이 없으면 None 반환 (기존 방식 폴백).
    """
    G = _build_graph(skel, h)
    if G.number_of_nodes() == 0: return None

    # 분기점(차수≥3) 있어야 의미 있음
    if not any(G.degree(n) >= 3 for n in G.nodes()): return None

    # 단방향 그래프로 변환 (simple_cycles 사용)
    DG = nx.DiGraph()
    for u, v, data in G.edges(data=True):
        DG.add_edge(u, v, path=data['path'])
        DG.add_edge(v, u, path=data['path_rev'])

    # 1패스: 모든 사이클 수집 + 길이 계산
    candidates = []
    for cycle in nx.simple_cycles(DG):
        if len(cycle) < 2: continue
        full_py, full_px = [], []
        valid = True
        for i in range(len(cycle)):
            u, v = cycle[i], cycle[(i+1) % len(cycle)]
            if not DG.has_edge(u, v): valid = False; break
            seg = DG[u][v]['path']
            full_py.extend(p[0] for p in seg[:-1])
            full_px.extend(p[1] for p in seg[:-1])
        if not valid or len(full_py) < 20: continue
        full_py = np.array(full_py)
        full_px = np.array(full_px)
        rx, ry  = px_to_ros(full_px, full_py, h)
        ds = np.hypot(np.diff(np.append(rx, rx[0])), np.diff(np.append(ry, ry[0])))
        track_len = float(ds.sum())
        candidates.append((track_len, full_py, full_px, rx, ry, ds))

    if not candidates: return None

    # 필터 1: 최단 후보의 30% 미만 → 아티팩트 제거
    max_len = max(c[0] for c in candidates)
    candidates = [c for c in candidates if c[0] >= max_len * 0.9]
    if not candidates: return None

    # 필터 3: 스폰 위치(출발라인)를 지나는 루프만 유효
    # 레이싱 서킷은 출발선을 반드시 통과해야 한 바퀴
    SPAWN_THRESH = 3.0  # 스폰에서 3m 이내를 지나는 루프만 허용
    if SEED_ROS_X is not None and SEED_ROS_Y is not None:
        valid = []
        for c in candidates:
            _, _, _, rx, ry, _ = c
            if np.hypot(rx - SEED_ROS_X, ry - SEED_ROS_Y).min() <= SPAWN_THRESH:
                valid.append(c)
        if valid:
            candidates = valid
            print(f'  스폰({SEED_ROS_X:.1f},{SEED_ROS_Y:.1f}) 근접 필터: {len(candidates)}개 유효')
        else:
            print(f'  ⚠ 스폰 근접 루프 없음 → 필터 미적용')

    best_time = float('inf')
    best_py   = best_px = None

    for track_len, full_py, full_px, rx, ry, ds in candidates:
        kappa = np.abs(compute_curvature(rx, ry))
        v     = np.clip(np.sqrt(A_LAT_MAX / np.maximum(kappa, 0.01)), V_MIN, V_MAX)
        t     = float(np.sum(ds / np.maximum(v, 1e-3)))
        print(f'    루트 후보: {len(full_py)}pts  {track_len:.1f}m  예상 {t:.2f}s')

        # 최소 랩타임 선택
        if t < best_time:
            best_time = t
            best_py, best_px = full_py, full_px

    if best_py is None: return None

    widths   = dist_track[best_py, best_px] * RESOLUTION
    best_rx, best_ry = px_to_ros(best_px, best_py, h)
    return best_rx, best_ry, widths, best_time


# ══════════════════════════════════════════════════════════════════════════
# STEP 2 : 스켈레톤 추출 + 가지 제거
# ══════════════════════════════════════════════════════════════════════════
def prune_dead_ends(skel, steps=30):
    kernel = np.array([[1,1,1],[1,0,1],[1,1,1]])
    s = skel.copy()
    for _ in range(steps):
        n = ndimage.convolve(s.astype(int), kernel, mode='constant')
        ep = s & (n == 1)
        if not ep.any():
            break
        s = s & ~ep
    return s


def prune_dead_ends_fully(skel):
    """dead end를 완전 제거해 순수 루프만 남김.
    단순 트랙: 외곽 루프 하나만 남음.
    다중 루트: 루프를 형성하는 경로들만 남음.
    """
    kernel = np.array([[1,1,1],[1,0,1],[1,1,1]])
    s = skel.copy()
    for _ in range(10000):
        n = ndimage.convolve(s.astype(int), kernel, mode='constant') * s
        ep = s & (n == 1)
        if not ep.any():
            break
        s = s & ~ep
    # 빈 스켈레톤 방지: 완전 제거되면 원본 반환
    return s if s.any() else skel


def extract_centerline(map_path):
    from skimage.morphology import skeletonize
    from skimage.measure import label

    img  = np.array(Image.open(map_path).convert('L'))
    h, _ = img.shape
    free = (img > 200).astype(np.uint8)

    # ── 핵심: 스폰 시드로 실제 트랙만 추출 ────────────────────────────────
    track = extract_track_mask(free, h)

    # 트랙 거리 변환 (트랙 내부에서만)
    dist_track = ndimage.distance_transform_edt(track)

    # 안전 침식 + 최대 폭 필터 (F1 인필드 제거용)
    safety_px = max(1, int(SAFETY_M / RESOLUTION))
    if MAX_TRACK_M > 0:
        max_px = MAX_TRACK_M / RESOLUTION
        safe_zone = (dist_track > safety_px) & (dist_track < max_px)
        safe_zone = safe_zone.astype(np.uint8)
        print(f'  최대 폭 필터 적용: {MAX_TRACK_M}m ({max_px:.0f}px) — 인필드 제거')
    else:
        safe_zone = ndimage.binary_erosion(track, iterations=safety_px)

    skel = skeletonize(safe_zone)
    skel = prune_dead_ends_fully(skel)
    _labeled = label(skel); _counts = np.bincount(_labeled.ravel()); _counts[0] = 0
    skel = (_labeled == _counts.argmax())

    # order_points 시험: 마감 갭이 작으면 단일 경로, 크면 다중 경로
    py, px_ = np.where(skel)
    ros_x, ros_y = px_to_ros(px_, py, h)
    widths = dist_track[py, px_] * RESOLUTION

    pts = np.column_stack([ros_x, ros_y])
    used = np.zeros(len(pts), dtype=bool); used[0] = True; cur = pts[0]; order = [0]
    for _ in range(len(pts) - 1):
        d = np.linalg.norm(pts - cur, axis=1); d[used] = np.inf
        nxt = int(np.argmin(d)); dist = float(d[nxt])
        if dist > 2.0: break
        order.append(nxt); used[nxt] = True; cur = pts[nxt]
    close_gap = float(np.linalg.norm(pts[order[-1]] - pts[0]))

    # 모든 포인트를 방문하고 루프도 닫힌 경우만 확실한 단일 경로
    if close_gap <= 1.0 and len(order) == len(pts):
        print(f'  order_points 완전 루프 ({len(order)}pts, 마감갭:{close_gap:.2f}m) → 단일 경로')
        print(f'  스켈레톤: {len(ros_x)}pts  폭 {widths.min():.2f}~{widths.max():.2f} m')
        return ros_x, ros_y, widths, dist_track, track, h, False

    print(f'  미방문 포인트 존재 ({len(order)}/{len(pts)}pts, 마감갭:{close_gap:.2f}m) → 그래프 방식')
    result = _select_fastest_loop(skel, dist_track, track, h)
    if result is not None:
        return result[0], result[1], result[2], dist_track, track, h, True

    print('  ⚠ 그래프 경로 탐색 실패 → 기존 방식으로 폴백')
    print(f'  스켈레톤: {len(ros_x)}pts  폭 {widths.min():.2f}~{widths.max():.2f} m')
    return ros_x, ros_y, widths, dist_track, track, h, False


# ══════════════════════════════════════════════════════════════════════════
# STEP 3 : 정렬 & 스무딩
# ══════════════════════════════════════════════════════════════════════════
def arc_resample(x, y, n_pts):
    """호길이 기반 선형 보간 리샘플링.
    스플라인 대신 선형 보간 사용 → 벽 뚫기 원천 불가능.
    스플라인은 코너에서 벽을 가로질러 단축하지만,
    선형 보간은 양 끝점이 안전하면 중간도 반드시 안전.
    """
    from scipy.interpolate import interp1d
    n = len(x)
    dx = np.diff(np.append(x, x[0]))
    dy = np.diff(np.append(y, y[0]))
    ds = np.hypot(dx, dy)
    s  = np.cumsum(np.append(0.0, ds))       # 0 ~ total (n+1 개)
    total = s[-1]
    if total < 1e-6:
        return x[:n_pts], y[:n_pts]
    u_orig = s / total                        # 0 ~ 1.0
    u_new  = np.linspace(0, 1, n_pts, endpoint=False)
    x_ext  = np.append(x, x[0])
    y_ext  = np.append(y, y[0])
    cx = interp1d(u_orig, x_ext, kind='linear')(u_new)
    cy = interp1d(u_orig, y_ext, kind='linear')(u_new)
    return np.array(cx), np.array(cy)


def order_points(x, y):
    """단순 nearest-neighbor 정렬 (단일 루프 전용).
    prune_dead_ends_fully 이후 분기 없는 깨끗한 루프에서 사용.
    """
    pts  = np.column_stack([x, y])
    used = np.zeros(len(pts), dtype=bool)
    used[0] = True
    order = [0]
    for _ in range(len(pts) - 1):
        cur = pts[order[-1]]
        d   = np.linalg.norm(pts - cur, axis=1)
        d[used] = np.inf
        nxt = int(np.argmin(d))
        if d[nxt] > 2.0:
            break
        order.append(nxt)
        used[nxt] = True
    return x[order], y[order]


def smooth_resample(x, y, n_pts, s_factor=0.5):
    pts  = np.column_stack([x, y])
    mask = np.concatenate([[True],
           np.linalg.norm(np.diff(pts, axis=0), axis=1) > 0.03])
    pts  = pts[mask]
    if len(pts) < 4:
        return x[:n_pts], y[:n_pts]
    tck, _ = splprep([pts[:,0], pts[:,1]],
                     s=len(pts)*s_factor, per=True, k=3)
    u = np.linspace(0, 1, n_pts, endpoint=False)
    xs, ys = splev(u, tck)
    return np.array(xs), np.array(ys)


# ══════════════════════════════════════════════════════════════════════════
# 충돌 검증 + 수정
# ══════════════════════════════════════════════════════════════════════════
def snap_to_safe(x, y, dist_track, h, min_m=None):
    if min_m is None:
        min_m = SAFETY_M
    min_px = min_m / RESOLUTION
    pxs, pys = ros_to_px(x, y, h)
    pxs = np.clip(pxs, 0, dist_track.shape[1]-1)
    pys = np.clip(pys, 0, dist_track.shape[0]-1)
    bad = set(np.where(dist_track[pys, pxs] < min_px)[0].tolist())

    # 세그먼트 중간점 검사: 양 끝점이 안전해도 스플라인이 중간에서 벽을 관통할 수 있음
    n = len(x)
    for i in range(n):
        j = (i + 1) % n
        for t in (0.25, 0.5, 0.75):
            mx = x[i] * (1 - t) + x[j] * t
            my = y[i] * (1 - t) + y[j] * t
            mpx, mpy = ros_to_px(np.array([mx]), np.array([my]), h)
            mpx = int(np.clip(mpx[0], 0, dist_track.shape[1]-1))
            mpy = int(np.clip(mpy[0], 0, dist_track.shape[0]-1))
            if dist_track[mpy, mpx] < min_px:
                bad.add(i)
                bad.add(j)

    if not bad:
        return x.copy(), y.copy()
    print(f'  ⚠ {len(bad)}개 벽 근처 포인트 수정...')
    safe_ys, safe_xs = np.where(dist_track >= min_px)
    tree = cKDTree(np.column_stack([safe_xs, safe_ys]))
    xf, yf = x.copy(), y.copy()
    for i in sorted(bad):
        _, idx = tree.query([pxs[i], pys[i]])
        xf[i], yf[i] = px_to_ros(safe_xs[idx], safe_ys[idx], h)
    print(f'  수정 완료')
    return xf, yf


def smooth_constrained(x, y, dist_track, h, iters=5, alpha=0.3, min_m=None):
    """Laplacian 스무딩으로 snap_to_safe 꺾임(kink) 제거.
    snap_to_safe가 점을 강제 이동시킬 때 생기는 급격한 방향 변화를 완화.
    순수 스무딩 N회 → 최종 안전 검사 1회.
    """
    if min_m is None:
        min_m = SAFETY_M
    xf, yf = x.copy(), y.copy()
    for _ in range(iters):
        xf = xf + alpha * (0.5 * (np.roll(xf, 1) + np.roll(xf, -1)) - xf)
        yf = yf + alpha * (0.5 * (np.roll(yf, 1) + np.roll(yf, -1)) - yf)
    return snap_to_safe(xf, yf, dist_track, h, min_m=min_m)


# ══════════════════════════════════════════════════════════════════════════
# STEP 4 : 최소 곡률 최적화 (폭 비례 범위)
# ══════════════════════════════════════════════════════════════════════════
def compute_curvature(x, y):
    dx  = np.gradient(x);  dy  = np.gradient(y)
    ddx = np.gradient(dx); ddy = np.gradient(dy)
    return (dx*ddy - dy*ddx) / np.clip((dx**2+dy**2)**1.5, 1e-9, None)

def compute_normals(x, y):
    dx = np.gradient(x); dy = np.gradient(y)
    n  = np.clip(np.hypot(dx, dy), 1e-9, None)
    return -dy/n, dx/n


def minimum_curvature_opt(cx, cy, cw, dist_track, h):
    nx, ny = compute_normals(cx, cy)

    # 폭 비례 최적화 범위:
    #   좁은 구간(미로, < NARROW_THRESH) → 오프셋 0 (완전 중앙 고정)
    #   넓은 구간(코너, > NARROW_THRESH) → 아웃-인-아웃 허용
    NARROW_THRESH = 1.8   # m : 이 이하 폭에서는 중앙선 고정
    w_avail = np.clip(cw - SAFETY_M, 0.05, None)
    w_ratio  = np.clip((cw - NARROW_THRESH) / (cw.max() - NARROW_THRESH + 1e-6), 0, 1)
    w_lim    = w_avail * w_ratio

    def objective(d):
        px = cx + d * nx
        py = cy + d * ny
        return float(np.sum(compute_curvature(px, py)**2))

    iters = [0]
    def callback(d):
        iters[0] += 1
        if iters[0] % 50 == 0:
            print(f'    iter {iters[0]:3d}  Σκ²={objective(d):.5f}')

    bounds = [(-w, w) for w in w_lim]
    result = minimize(objective, np.zeros(len(cx)),
                      method='L-BFGS-B', bounds=bounds,
                      callback=callback,
                      options={'maxiter':300,'ftol':1e-10,'gtol':1e-7})

    rx = cx + result.x * nx
    ry = cy + result.x * ny

    kb = np.mean(np.abs(compute_curvature(cx, cy)))
    ka = np.mean(np.abs(compute_curvature(rx, ry)))
    print(f'  평균 곡률: {kb:.4f} → {ka:.4f} ({(1-ka/kb)*100:.1f}% 개선)')
    print(f'  최대 오프셋: {np.abs(result.x).max():.3f} m')

    rx, ry = snap_to_safe(rx, ry, dist_track, h)
    rx, ry = smooth_constrained(rx, ry, dist_track, h)
    return rx, ry


# ══════════════════════════════════════════════════════════════════════════
# STEP 4b : 최소 랩타임 최적화
#   목적: f(d) = Σ ds_i/v_i(κ_i)  +  λ Σ(d_{i+1}-d_i)²
#   v_i = clip(√(μg / max(|κ_i|, ε)), V_MIN, V_MAX)
# ══════════════════════════════════════════════════════════════════════════
def minimum_laptime_opt(cx, cy, cw, dist_track, h, lambda_smooth=0.05, n_restarts=1):
    nx, ny = compute_normals(cx, cy)
    N = len(cx)

    NARROW_THRESH = 1.8
    # 최적화기 오프셋 범위: 벽에서 최소 0.7m 확보 (배율 방식 대신 고정값)
    opt_safety = max(SAFETY_M, 0.7)
    w_avail = np.clip(cw - opt_safety, 0.05, None)
    w_ratio = np.clip((cw - NARROW_THRESH) / (cw.max() - NARROW_THRESH + 1e-6), 0, 1)
    w_lim   = w_avail * w_ratio
    bounds  = [(-w, w) for w in w_lim]

    def laptime_est(d):
        px = cx + d * nx
        py = cy + d * ny
        ds = np.hypot(np.diff(np.append(px, px[0])), np.diff(np.append(py, py[0])))
        kappa = np.abs(compute_curvature(px, py))
        v = np.clip(np.sqrt(A_LAT_MAX / np.maximum(kappa, EPS_KAPPA)), V_MIN, V_MAX)
        return float(np.sum(ds / v))

    def objective(d):
        d_diff = np.diff(np.append(d, d[0]))
        return laptime_est(d) + lambda_smooth * float(np.sum(d_diff ** 2))

    T_before = laptime_est(np.zeros(N))

    iters = [0]
    def callback(d):
        iters[0] += 1
        if iters[0] % 50 == 0:
            print(f'    iter {iters[0]:3d}  T≈{laptime_est(d):.4f}s')

    best_result, best_val = None, np.inf
    for restart in range(max(1, n_restarts)):
        if restart == 0:
            x0 = np.zeros(N)
        else:
            x0 = np.random.default_rng(restart).uniform(-0.3, 0.3, N) * w_lim
        res = minimize(objective, x0, method='L-BFGS-B', bounds=bounds,
                       callback=callback,
                       options={'maxiter': 2000, 'ftol': 1e-10, 'gtol': 1e-7,
                                'eps': 1e-6, 'maxfun': 500000})
        if not res.success:
            print(f'  ⚠ restart {restart}: 미수렴 ({res.message.strip()})')
        if res.fun < best_val:
            best_val, best_result = res.fun, res

    T_after = laptime_est(best_result.x)
    print(f'  예상 랩타임: {T_before:.2f}s → {T_after:.2f}s  ({(T_before-T_after)/T_before*100:.1f}% 개선)')
    print(f'  최대 오프셋: {np.abs(best_result.x).max():.3f} m  ({best_result.nit}회 반복)')

    rx = cx + best_result.x * nx
    ry = cy + best_result.x * ny
    rx, ry = snap_to_safe(rx, ry, dist_track, h)
    rx, ry = smooth_constrained(rx, ry, dist_track, h)
    return rx, ry


# ══════════════════════════════════════════════════════════════════════════
# STEP 5 : 마찰원 + 전방-후방 적분 속도 프로파일
# ══════════════════════════════════════════════════════════════════════════
def speed_profile(x, y):
    n  = len(x)
    dx = np.diff(np.append(x, x[0]))
    dy = np.diff(np.append(y, y[0]))
    ds = np.clip(np.hypot(dx, dy), 1e-6, None)

    kappa   = np.abs(compute_curvature(x, y))
    v_lim   = np.clip(np.sqrt(A_LAT_MAX / np.clip(kappa, 0.01, None)), V_MIN, V_MAX)

    v = v_lim.copy()
    for _ in range(3):
        for i in range(n):
            v[i] = min(v[i], np.sqrt(v[(i-1)%n]**2 + 2*A_LON_ACC*ds[(i-1)%n]))
    for _ in range(3):
        for i in range(n-1, -1, -1):
            v[i] = min(v[i], np.sqrt(v[(i+1)%n]**2 + 2*A_LON_BRK*ds[i]))
    v = np.clip(v, V_MIN, V_MAX)

    total = ds.sum()
    avg   = total / (ds / np.clip(v, 1e-3, None)).sum()
    print(f'  속도: {v.min():.2f}~{v.max():.2f} m/s  '
          f'랩타임 ≈ {total/avg:.1f}s  (트랙 {total:.1f}m)')
    return v


# ══════════════════════════════════════════════════════════════════════════
# 중심선 시각화 저장 (2단계 결과)
# ══════════════════════════════════════════════════════════════════════════
def save_centerline_image(raw_x, raw_y, raw_w, map_path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        img     = np.array(Image.open(map_path).convert('L'))
        h_img   = img.shape[0]
        px      = (np.asarray(raw_x) - ORIGIN[0]) / RESOLUTION
        py      = h_img - (np.asarray(raw_y) - ORIGIN[1]) / RESOLUTION

        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(img, cmap='gray', origin='upper', alpha=0.7)
        sc = ax.scatter(px, py, c=raw_w, cmap='RdYlGn', s=3, linewidths=0)
        plt.colorbar(sc, ax=ax, label='dist to wall (m)')
        map_name = os.path.splitext(os.path.basename(map_path))[0]
        ax.set_title(f'{map_name} centerline ({len(raw_x)} pts)', fontsize=12)
        ax.axis('off')
        plt.tight_layout()

        out = os.path.join(os.path.dirname(map_path), f'{map_name}_center.jpg')
        plt.savefig(out, dpi=150, bbox_inches='tight')
        plt.close()
        print(f'  중심선 시각화 저장: {out}')
    except Exception as e:
        print(f'  ⚠ 중심선 시각화 실패: {e}')


# ══════════════════════════════════════════════════════════════════════════
# 저장
# ══════════════════════════════════════════════════════════════════════════
def save_waypoints(x, y, v, path):
    import shutil
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg

    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'speed'])
        for xi, yi, vi in zip(x, y, v):
            w.writerow([f'{xi:.4f}', f'{yi:.4f}', f'{vi:.4f}'])
    print(f'  저장: {len(x)}개 → {path}')

    # 웨이포인트 시각화 이미지 저장
    try:
        map_dir  = os.path.dirname(path)
        map_name = os.path.splitext(os.path.basename(path))[0]  # waypoints_real_track
        map_base = map_name.replace('_waypoints.csv', '')             # real_track

        # png 또는 pgm
        img_path = None
        for ext in ['.png', '.pgm']:
            candidate = os.path.join(map_dir, map_base + ext)
            if os.path.exists(candidate):
                img_path = candidate
                break

        if img_path:
            # 픽셀 좌표로 변환
            px = (np.asarray(x) - ORIGIN[0]) / RESOLUTION
            py = []
            img = np.array(__import__('PIL').Image.open(img_path).convert('L'))
            h  = img.shape[0]
            py = h - (np.asarray(y) - ORIGIN[1]) / RESOLUTION

            fig, ax = plt.subplots(figsize=(10, 10))
            ax.imshow(img, cmap='gray', origin='upper')

            # 속도에 따라 색상 변화 (파랑=느림, 빨강=빠름)
            sc = ax.scatter(px, py, c=v, cmap='RdYlGn',
                           s=3, linewidths=0)
            plt.colorbar(sc, ax=ax, label='speed (m/s)')
            ax.set_title(f'{map_base} waypoints ({len(x)} pts)', fontsize=12)
            ax.axis('off')
            plt.tight_layout()

            img_out = os.path.join(map_dir, f'{map_base}_waypoints.jpg')
            plt.savefig(img_out, dpi=150, bbox_inches='tight')
            plt.close()
            print(f'  시각화 저장: {img_out}')
        else:
            print(f'  ⚠ 맵 이미지 없음 → 시각화 생략')

    except Exception as e:
        print(f'  ⚠ 시각화 실패: {e}')

    # build/ → src/ 심볼릭 링크 확인
    build_path = path.replace('/src/', '/build/')
    if os.path.islink(build_path):
        print(f'  심볼릭 링크 확인: install/ 자동 반영됨')


# ══════════════════════════════════════════════════════════════════════════
# 시작점·방향 클릭 & 재정렬
# ══════════════════════════════════════════════════════════════════════════
def click_start_direction(map_path, rx, ry, h):
    """맵 + 웨이포인트를 표시하고 시작점(1번)·주행 방향(2번)을 클릭받는다."""
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt

    img  = np.array(Image.open(map_path).convert('L'))
    px_wp = (np.asarray(rx) - ORIGIN[0]) / RESOLUTION
    py_wp = h - (np.asarray(ry) - ORIGIN[1]) / RESOLUTION

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(img, cmap='gray', origin='upper', alpha=0.8)
    ax.scatter(px_wp, py_wp, s=4, c='cyan', alpha=0.7, zorder=3, label='waypoints')
    ax.set_title('1: 시작점 클릭  →  2: 주행 방향 클릭', fontsize=13, fontweight='bold')
    ax.axis('off')
    ax.legend(loc='lower right', fontsize=9)
    plt.tight_layout()

    pts = plt.ginput(2, timeout=120)
    plt.close()

    if len(pts) < 2:
        return None, None, None, None

    px1, py1 = pts[0]
    sx =  px1 * RESOLUTION + ORIGIN[0]
    sy = (h - py1) * RESOLUTION + ORIGIN[1]

    px2, py2 = pts[1]
    ddx =  (px2 - px1) * RESOLUTION   # 픽셀 차이 → ROS 좌표계 방향벡터
    ddy = -(py2 - py1) * RESOLUTION   # 이미지 y축 반전

    print(f'  시작점  : ({sx:.3f}, {sy:.3f})')
    print(f'  방향벡터: ({ddx:.3f}, {ddy:.3f})  {np.degrees(np.arctan2(ddy, ddx)):.1f}°')
    return sx, sy, ddx, ddy


def reorder_waypoints(rx, ry, v, sx, sy, ddx, ddy):
    """시작점에서 가장 가까운 웨이포인트를 인덱스 0으로 설정하고,
    클릭 방향과 일치하도록 순서를 정렬한다."""
    i_start = int(np.argmin(np.hypot(rx - sx, ry - sy)))

    rx = np.roll(rx, -i_start)
    ry = np.roll(ry, -i_start)
    v  = np.roll(v,  -i_start)

    # index 1 방향이 클릭 방향과 반대이면 역순 (시작점 index 0 유지)
    v_fwd = np.array([rx[1] - rx[0], ry[1] - ry[0]])
    if ddx * v_fwd[0] + ddy * v_fwd[1] < 0:
        rx = np.roll(rx[::-1], 1)
        ry = np.roll(ry[::-1], 1)
        v  = np.roll(v[::-1],  1)
        print('  역방향 감지 → 순서 반전')

    d_start = float(np.hypot(rx[0] - sx, ry[0] - sy))
    print(f'  시작 인덱스: {i_start}  (클릭점까지 {d_start:.3f}m)')
    return rx, ry, v


# ══════════════════════════════════════════════════════════════════════════
# 메인
# ══════════════════════════════════════════════════════════════════════════
def main():
    global MAP_PATH, OUTPUT_PATH, RESOLUTION, ORIGIN
    global SEED_ROS_X, SEED_ROS_Y, V_MAX, V_MIN, SAFETY_M, MAX_TRACK_M, A_LON_ACC, A_LON_BRK, A_LAT_MAX
    global OPT_MODE, SMOOTH_W, N_RESTARTS

    args = parse_args()

    # 맵 선택: --map 미입력 시 대화식 선택
    if args.map is None:
        map_name = select_map()
        if map_name is None:
            return
        map_base = os.path.join(_MAPS_DIR, map_name)
        if args.out == os.path.join(_THIS_DIR, 'waypoints.csv'):
            args.out = os.path.join(_MAPS_DIR, f'{map_name}_waypoints.csv')
    else:
        map_base = args.map

    # 맵 파일 결정 (.png 또는 .pgm 자동 선택)
    if os.path.exists(map_base + '.png'):
        MAP_PATH = map_base + '.png'
    elif os.path.exists(map_base + '.pgm'):
        MAP_PATH = map_base + '.pgm'
    else:
        print(f'오류: 맵 파일을 찾을 수 없음 ({map_base}.png / .pgm)')
        return

    OUTPUT_PATH = args.out
    RESOLUTION, ORIGIN = load_map_config(map_base)
    SEED_ROS_X  = args.seed_x
    SEED_ROS_Y  = args.seed_y
    V_MAX       = args.vmax
    V_MIN       = args.vmin
    SAFETY_M    = args.safety
    A_LON_ACC   = args.acc
    A_LON_BRK   = args.brk
    A_LAT_MAX   = args.mu * G
    MAX_TRACK_M = args.max_width
    OPT_MODE    = args.opt_mode
    SMOOTH_W    = args.smooth_w
    N_RESTARTS  = args.restarts

    print('='*55)
    print(' F1TENTH 레이싱 라인 생성기 v3')
    print(' Flood-fill 트랙 추출 + 폭 비례 최적화')
    print('='*55)
    print(f'  맵    : {MAP_PATH}')
    print(f'  출력  : {OUTPUT_PATH}')
    print(f'  해상도: {RESOLUTION} m/px  원점: {ORIGIN}')
    if SEED_ROS_X is not None:
        print(f'  시드  : ({SEED_ROS_X}, {SEED_ROS_Y})')

    print('\n[1/5] 트랙 영역 추출 및 중심선 계산...')
    raw_x, raw_y, raw_w, dist_track, track, h, already_ordered = \
        extract_centerline(MAP_PATH)

    save_centerline_image(raw_x, raw_y, raw_w, MAP_PATH)

    print('[2/5] 정렬 및 스무딩...')
    if already_ordered:
        # 그래프 경로: 선형 보간 → 벽 뚫기 불가
        cx, cy = arc_resample(raw_x, raw_y, N_OPT)
    else:
        ox, oy = order_points(raw_x, raw_y)
        cx, cy = smooth_resample(ox, oy, N_OPT, s_factor=0.05)
    cx, cy = snap_to_safe(cx, cy, dist_track, h)

    # cw: smooth된 실제 경로 위치에서 직접 dist_track 조회
    # (nearest skeleton pixel 사용 시 스플라인 이탈로 cw 과대평가 → 벽 충돌 원인)
    pxs_cw, pys_cw = ros_to_px(cx, cy, h)
    pxs_cw = np.clip(pxs_cw, 0, dist_track.shape[1]-1)
    pys_cw = np.clip(pys_cw, 0, dist_track.shape[0]-1)
    cw = dist_track[pys_cw, pxs_cw] * RESOLUTION

    if OPT_MODE == 'laptime':
        print(f'[3/5] 최소 랩타임 최적화 ({N_OPT}pts, λ={SMOOTH_W}, restarts={N_RESTARTS})...')
        rx, ry = minimum_laptime_opt(cx, cy, cw, dist_track, h,
                                      lambda_smooth=SMOOTH_W, n_restarts=N_RESTARTS)
    else:
        print(f'[3/5] 최소 곡률 최적화 ({N_OPT}pts, 폭 비례 범위)...')
        rx, ry = minimum_curvature_opt(cx, cy, cw, dist_track, h)

    print('[4/5] 최종 리샘플링...')
    rx, ry = arc_resample(rx, ry, N_FINAL)
    rx, ry = snap_to_safe(rx, ry, dist_track, h, min_m=SAFETY_M * 1.5)
    rx, ry = smooth_constrained(rx, ry, dist_track, h, iters=8, alpha=0.3, min_m=SAFETY_M * 1.5)

    print('[5/5] 속도 프로파일 (마찰원 + 전방-후방 적분)...')
    v = speed_profile(rx, ry)

    print('\n[+] 시작점·주행 방향 설정 (창 닫거나 1번만 클릭하면 기본 순서 유지)...')
    try:
        sx_wp, sy_wp, ddx, ddy = click_start_direction(MAP_PATH, rx, ry, h)
        if sx_wp is not None:
            rx, ry, v = reorder_waypoints(rx, ry, v, sx_wp, sy_wp, ddx, ddy)
        else:
            print('  클릭 없음 → 기본 순서 유지')
    except Exception as e:
        print(f'  ⚠ 클릭 실패 ({e}) → 기본 순서 유지')

    print('\n결과 저장...')
    save_waypoints(rx, ry, v, OUTPUT_PATH)
    print('\n완료! colcon build 후 재시작하세요.')


if __name__ == '__main__':
    main()
