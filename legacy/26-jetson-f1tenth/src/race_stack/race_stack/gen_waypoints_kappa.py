#!/usr/bin/env python3
"""
gen_waypoints_kappa.py — SLAM 저장 맵에서 레이싱 라인 웨이포인트 생성
(gen_waypoints.py 의 곡률-제약 버전 — 랩타임 비교용으로 분리)

gen_waypoints.py 와 100% 동일한 흐름/조작이되, 최적화 목적함수에
조향각 물리 한계(wheelbase/max_steer로 계산한 최대곡률) 벌점이 추가됐다.
가장 곡률이 큰 코너에서 조향 포화로 넓게 돌고 복귀하는 문제를 완화하려는
시도 — 트랙 폭이 허용하는 한 코너를 아웃-인-아웃으로 넓혀 곡률을 낮춘다.
출력 파일명을 <map_name>_wp_kappa.csv 로 따로 둬서 원본 gen_waypoints.py
결과물과 나란히 두고 실차 랩타임을 비교할 수 있게 했다.

흐름:
  1) 맵 선택 (터미널)
  2) 자동 스켈레톤 후보 추출 → 맵에 표시
  3) 마우스 클릭으로 원하는 경로 스케치
  4) Enter → 최적 후보 매칭 → 랩타임 최적화 → 자동 저장
  5) r=재그리기, s=재저장

조작:
  좌클릭        — 포인트 추가
  우클릭 / z    — 마지막 포인트 취소
  Enter         — 매칭 + 최적화 + 저장
  s             — 재저장
  r             — 리셋
  Esc           — 종료

저장 위치:
  waypoints/<map_name>_wp_kappa.csv    (x, y, speed — stanley_controller용)
  maps/<map_name>/inner_bound.npy      (픽셀 좌표 int32 — Q2 장애물 회피용)
  maps/<map_name>/outer_bound.npy
"""

import itertools
import os
import csv
import sys
import yaml
import numpy as np
import networkx as nx
from PIL import Image
import scipy.ndimage as ndimage
from scipy.interpolate import splprep, splev, interp1d
from scipy.optimize import minimize
from scipy.spatial import cKDTree

import cv2
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

# ── 경로 (환경변수 RACE_WS 로 워크스페이스 지정, 기본은 이 파일 기준) ────────
# 이 파일: <ws>/src/race_stack/race_stack/gen_waypoints.py → 패키지 루트는 두 단계 위
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.environ.get('RACE_WS'):
    _PKG_DIR = os.path.join(
        os.path.expanduser(os.environ['RACE_WS']), 'src', 'race_stack')
MAPS_DIR  = os.path.join(_PKG_DIR, 'maps')
WP_DIR    = os.path.join(_PKG_DIR, 'waypoints')

# ── 물리 파라미터 (main에서 args로 덮어씀) ───────────────────────────────────
# A_LAT_MAX: curvature_probe 08/07 실측(좌회전 풀락 스윕, v=2.00~2.25㎧ 슬립
# 포화구간 a_lat=v²·κ_achieved 역산 평균) 기반. 0.9g(8.83)는 실측보다 훨씬
# 낙관적이라 랩타임 최적화 단계에서 곡률 비용을 과소평가했었다(set_speed_kappa.py
# 참고 — 그쪽은 이미 이 값으로 속도만 사후 재계산 중, 여긴 경로 형태 자체).
RESOLUTION = 0.05
ORIGIN     = (0.0, 0.0)
G_GRAV     = 9.81
A_LAT_MAX  = 3.7
A_LON_ACC  = 5.0
A_LON_BRK  = 8.0
V_MAX      = 6.0
V_MIN      = 0.5
SAFETY_M   = 0.4
N_OPT      = 200
N_FINAL    = 400
EPS_KAPPA  = 1e-4
SMOOTH_W   = 0.005

# race_script.yaml [pure_pursuit] 의 wheelbase/max_steer 와 동일하게 맞출 것.
# 조향각이 포화되면 이 곡률보다 급한 코너는 실제로 못 돌고 밖으로 밀린다.
WHEELBASE  = 0.33         # 축간거리 (m)
MAX_STEER  = 0.36         # 최대 조향각 (rad)
KAPPA_MAX  = np.tan(MAX_STEER) / WHEELBASE   # 물리적 최대 곡률 (1/m)
# 최적화는 조향 한계의 KAPPA_MARGIN 배만 쓴다. 1.0 이면 라인 R_min = 차 R_min
# 이라, 추종 오차로 안쪽에 1cm 만 들어가도 κ' = κ/(1−dκ) 가 불법이 된다.
KAPPA_MARGIN = 0.85


def _car_steer_geom(fallback_wb=0.33, fallback_ms=0.36):
    """apply_vehicle 이 만든 overlay 에서 wheelbase / max_steer.

    손으로 적으면 컨트롤러와 갈라진다 (gen_raceline._car_min_radius 과 같은 이유).
    overlay 가 없으면(노트북에서 분석만 할 때) 모듈 기본값.
    """
    for name in ('.vehicle_overlay.yaml',):
        p = os.path.join(_PKG_DIR, '..', '..', 'config', name)
        p = os.path.abspath(p)
        if not os.path.exists(p):
            continue
        try:
            o = yaml.safe_load(open(p))
            ms = float(o['pure_pursuit']['max_steer'])
            wb = float(o['pure_pursuit']['wheelbase'])
            return wb, ms
        except Exception:
            pass
    return fallback_wb, fallback_ms


# ══════════════════════════════════════════════════════════════════════════════
# 맵 선택
# ══════════════════════════════════════════════════════════════════════════════
def select_map_interactive():
    os.makedirs(MAPS_DIR, exist_ok=True)
    files = os.listdir(MAPS_DIR)
    maps  = sorted(set(
        f.rsplit('.', 1)[0]
        for f in files
        if f.endswith(('.png', '.pgm'))
    ))
    if not maps:
        sys.exit(f'맵 없음. SLAM으로 맵을 먼저 저장하세요.\n  ./race.sh savemap <이름>')

    wp_set = set()
    if os.path.isdir(WP_DIR):
        suffix = '_wp_kappa.csv'
        wp_set = {f[:-len(suffix)] for f in os.listdir(WP_DIR) if f.endswith(suffix)}

    print('\n' + '='*50)
    print('  사용 가능한 맵 목록')
    print('='*50)
    for i, name in enumerate(maps, 1):
        mark = '*' if name in wp_set else ' '
        print(f'  {i:3d}. [{mark}] {name}')
    print('='*50)
    print('  * = 웨이포인트 이미 존재\n')

    while True:
        val = input('맵 이름 또는 번호: ').strip()
        if not val:
            continue
        if val.isdigit():
            idx = int(val) - 1
            if 0 <= idx < len(maps):
                return maps[idx]
            print(f'  범위 초과 (1~{len(maps)})')
            continue
        if val in maps:
            return val
        matches = [n for n in maps if val.lower() in n.lower()]
        if len(matches) == 1:
            print(f'  -> {matches[0]}')
            return matches[0]
        if matches:
            print(f'  후보: {matches}')
        else:
            print(f'  맵 없음: {val}')


# ══════════════════════════════════════════════════════════════════════════════
# 좌표 변환
# ══════════════════════════════════════════════════════════════════════════════
def px_to_world(px, py, h):
    return (np.asarray(px, float) * RESOLUTION + ORIGIN[0],
            (h - np.asarray(py, float)) * RESOLUTION + ORIGIN[1])


def world_to_px(rx, ry, h):
    px = np.round((np.asarray(rx) - ORIGIN[0]) / RESOLUTION).astype(int)
    py = np.round(h - (np.asarray(ry) - ORIGIN[1]) / RESOLUTION).astype(int)
    return px, py


# ══════════════════════════════════════════════════════════════════════════════
# 트랙 마스크 + 거리 변환
# ══════════════════════════════════════════════════════════════════════════════
def build_dist_track(img_arr, seed_px, seed_py):
    h, w = img_arr.shape

    # 확정 자유공간(254)만 사용. > 200 이면 미탐지(205)도 포함돼
    # 실제 SLAM 맵에서 전체가 하나의 영역이 되어 스켈레톤이 의미없어짐.
    free = (img_arr > 250).astype(np.uint8)

    # 확정 자유공간이 너무 적으면 넓은 임계값으로 폴백
    if free.sum() < 100:
        print('  경고: 확정 자유공간 부족 → 임계값 완화 (> 200)')
        free = (img_arr > 200).astype(np.uint8)

    labeled, n_labels = ndimage.label(free)
    spx = int(np.clip(seed_px, 0, w - 1))
    spy = int(np.clip(seed_py, 0, h - 1))
    tl  = labeled[spy, spx]
    if tl == 0:
        # 시드가 벽 위에 있으면 가장 큰 자유 영역 선택
        counts = np.bincount(labeled.ravel()); counts[0] = 0
        tl = int(counts.argmax())
    mask = (labeled == tl).astype(np.uint8)
    dist = ndimage.distance_transform_edt(mask)
    print(f'  자유공간: {free.sum()}px (전체 {h*w}px의 {free.sum()*100/(h*w):.1f}%)  '
          f'연결 영역: {n_labels}개  트랙 영역: {mask.sum()}px')
    return dist, mask


# ══════════════════════════════════════════════════════════════════════════════
# 스켈레톤 유틸
# ══════════════════════════════════════════════════════════════════════════════
def prune_dead_ends_fully(skel):
    kernel = np.array([[1,1,1],[1,0,1],[1,1,1]])
    s = skel.copy()
    for _ in range(10000):
        n = ndimage.convolve(s.astype(int), kernel, mode='constant') * s
        ep = s & (n == 1)
        if not ep.any():
            break
        s = s & ~ep
    return s if s.any() else skel


def trace_skeleton_pixels(skel, h):
    py_arr, px_arr = np.where(skel)
    if len(py_arr) == 0:
        return None
    total    = len(py_arr)
    skel_set = set(zip(py_arr.tolist(), px_arr.tolist()))

    def unvisited_nbrs(r, c, visited):
        result = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0: continue
                pt = (r+dr, c+dc)
                if pt in skel_set and pt not in visited:
                    result.append(pt)
        return result

    start = (py_arr[0], px_arr[0])
    path  = [start]; visited = {start}; prev = None; cur = start

    while len(path) < total:
        nbrs = unvisited_nbrs(*cur, visited)
        if not nbrs: break
        if len(nbrs) == 1:
            nxt = nbrs[0]
        else:
            if prev is not None:
                dr_p = cur[0]-prev[0]; dc_p = cur[1]-prev[1]
                nbrs.sort(key=lambda n:
                    -((n[0]-cur[0])*dr_p + (n[1]-cur[1])*dc_p))
            nxt = nbrs[0]
        visited.add(nxt); path.append(nxt); prev = cur; cur = nxt

    path_arr = np.array(path)
    px_ = path_arr[:, 1]; py_ = path_arr[:, 0]
    ros_x, ros_y = px_to_world(px_, py_, h)
    close_gap = float(np.hypot(ros_x[-1]-ros_x[0], ros_y[-1]-ros_y[0]))
    return ros_x, ros_y, py_, px_, len(path), close_gap


def _build_graph(skel, h):
    H, W  = skel.shape
    k3    = np.array([[1,1,1],[1,0,1],[1,1,1]])
    deg   = ndimage.convolve(skel.astype(int), k3, mode='constant') * skel
    nodes = set(map(tuple, np.argwhere(skel & ((deg >= 3) | (deg == 1)))))
    G     = nx.Graph(); G.add_nodes_from(nodes)
    seen_edges = set()

    for start in nodes:
        sr, sc = start
        for dr in range(-1, 2):
            for dc in range(-1, 2):
                if dr == 0 and dc == 0: continue
                nr, nc = sr+dr, sc+dc
                if not (0 <= nr < H and 0 <= nc < W): continue
                if not skel[nr, nc]: continue
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
                            if skel[nr2, nc2]: nexts.append((nr2, nc2))
                    if not nexts: break
                    prev, cur = cur, nexts[0]; path.append(cur)
                if cur not in nodes or cur == start: continue
                eid = frozenset({start, cur, len(path)})
                if eid in seen_edges: continue
                seen_edges.add(eid)
                path_arr = np.array(path)
                rx, ry   = px_to_world(path_arr[:,1], path_arr[:,0], h)
                kappa    = np.abs(_curvature(rx, ry))
                v        = np.clip(np.sqrt(A_LAT_MAX / np.maximum(kappa, 0.01)),
                                   V_MIN, V_MAX)
                ds       = np.hypot(np.diff(np.append(rx, rx[-1])),
                                    np.diff(np.append(ry, ry[-1])))
                t_edge   = float(np.sum(ds / np.maximum(v, 1e-3)))
                G.add_edge(start, cur, path=path,
                           path_rev=list(reversed(path)), time=t_edge)
    return G


# ══════════════════════════════════════════════════════════════════════════════
# 후보 경로 추출
# ══════════════════════════════════════════════════════════════════════════════
def extract_candidates(img_arr, seed_px, seed_py):
    from skimage.morphology import skeletonize
    from skimage.measure import label as sk_label

    h, w = img_arr.shape
    dist_track, mask = build_dist_track(img_arr, seed_px, seed_py)

    # 안전 마진 침식 — 너무 많이 침식되면 자동으로 줄임
    safety_px = max(1, int(SAFETY_M / RESOLUTION))
    safe_zone = ndimage.binary_erosion(mask, iterations=safety_px)
    if safe_zone.sum() < 50:
        safety_px = max(1, safety_px // 2)
        safe_zone = ndimage.binary_erosion(mask, iterations=safety_px)
        print(f'  경고: 안전마진 축소 → {safety_px}px ({safety_px * RESOLUTION:.2f}m)')
    if safe_zone.sum() < 20:
        safe_zone = mask.astype(bool)
        print('  경고: 침식 건너뜀 (트랙이 너무 좁음)')
    print(f'  침식 후 자유공간: {safe_zone.sum()}px')

    skel = skeletonize(safe_zone)
    skel = prune_dead_ends_fully(skel)
    print(f'  스켈레톤: {skel.sum()}px')

    lbl  = sk_label(skel)
    cnts = np.bincount(lbl.ravel()); cnts[0] = 0
    if cnts.max() == 0:
        print('  오류: 스켈레톤 생성 실패 — 맵을 clean_map.py로 먼저 정제하세요')
        return [], dist_track, h
    skel = (lbl == cnts.argmax())

    total_pts  = int(skel.sum())
    candidates = []

    trace = trace_skeleton_pixels(skel, h)
    if trace is not None:
        tr_x, tr_y, tr_py, tr_px, n_vis, close_gap = trace
        coverage = n_vis / total_pts
        if coverage >= 0.90 and close_gap <= 3.0:
            widths = dist_track[tr_py, tr_px] * RESOLUTION
            ds = np.hypot(np.diff(np.append(tr_x, tr_x[0])),
                          np.diff(np.append(tr_y, tr_y[0])))
            print(f'  pixel-trace: {coverage*100:.0f}% / {ds.sum():.1f}m')
            return [(tr_x, tr_y, widths, float(ds.sum()))], dist_track, h

    Gg = _build_graph(skel, h)
    if Gg.number_of_nodes() == 0 or \
            not any(Gg.degree(n) >= 3 for n in Gg.nodes()):
        if trace is not None:
            tr_x, tr_y, tr_py, tr_px, n_vis, _ = trace
            widths = dist_track[tr_py, tr_px] * RESOLUTION
            ds = np.hypot(np.diff(np.append(tr_x, tr_x[0])),
                          np.diff(np.append(tr_y, tr_y[0])))
            return [(tr_x, tr_y, widths, float(ds.sum()))], dist_track, h
        return [], dist_track, h

    DG = nx.DiGraph()
    for u, v, data in Gg.edges(data=True):
        DG.add_edge(u, v, path=data['path'])
        DG.add_edge(v, u, path=data['path_rev'])

    raw = []
    for cycle in itertools.islice(nx.simple_cycles(DG), 500):
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
        full_py = np.array(full_py); full_px = np.array(full_px)
        rx, ry  = px_to_world(full_px, full_py, h)
        ds      = np.hypot(np.diff(np.append(rx, rx[0])),
                           np.diff(np.append(ry, ry[0])))
        tl      = float(ds.sum())
        widths  = dist_track[full_py, full_px] * RESOLUTION
        raw.append((rx, ry, widths, tl))

    if not raw:
        return candidates, dist_track, h

    max_len = max(c[3] for c in raw)
    raw     = [c for c in raw if c[3] >= max_len * 0.3]

    seen = []
    for c in sorted(raw, key=lambda x: x[3], reverse=True):
        tl = c[3]
        if not any(abs(tl - s) / (s + 1e-6) < 0.03 for s in seen):
            seen.append(tl)
            candidates.append(c)

    print(f'  graph 후보: {len(raw)} -> 중복 제거: {len(candidates)}')
    for c in candidates:
        print(f'    {c[3]:.1f}m')
    return candidates, dist_track, h


# ══════════════════════════════════════════════════════════════════════════════
# 최적화
# ══════════════════════════════════════════════════════════════════════════════
def ensure_ccw(x, y):
    signed_area = 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))
    if signed_area < 0:
        return x[::-1].copy(), y[::-1].copy()
    return x, y


def _curvature(x, y):
    dx  = np.gradient(x);  dy  = np.gradient(y)
    ddx = np.gradient(dx); ddy = np.gradient(dy)
    return (dx*ddy - dy*ddx) / np.clip((dx**2+dy**2)**1.5, 1e-9, None)


def _normals(x, y):
    dx = np.gradient(x); dy = np.gradient(y)
    n  = np.clip(np.hypot(dx, dy), 1e-9, None)
    return -dy/n, dx/n


def arc_resample(x, y, n_pts):
    dx = np.diff(np.append(x, x[0])); dy = np.diff(np.append(y, y[0]))
    ds = np.hypot(dx, dy)
    s  = np.cumsum(np.append(0.0, ds)); total = s[-1]
    if total < 1e-6: return x[:n_pts], y[:n_pts]
    u_o = s / total; u_n = np.linspace(0, 1, n_pts, endpoint=False)
    cx  = interp1d(u_o, np.append(x, x[0]), kind='linear')(u_n)
    cy  = interp1d(u_o, np.append(y, y[0]), kind='linear')(u_n)
    return np.array(cx), np.array(cy)


def smooth_resample(x, y, n_pts, s_factor=0.5):
    pts  = np.column_stack([x, y])
    mask = np.concatenate([[True],
           np.linalg.norm(np.diff(pts, axis=0), axis=1) > 0.03])
    pts  = pts[mask]
    if len(pts) < 4: return arc_resample(x, y, n_pts)
    try:
        tck, _ = splprep([pts[:,0], pts[:,1]],
                         s=len(pts)*s_factor, per=True, k=3)
        u = np.linspace(0, 1, n_pts, endpoint=False)
        xs, ys = splev(u, tck)
        return np.array(xs), np.array(ys)
    except Exception:
        return arc_resample(x, y, n_pts)


def snap_to_safe(x, y, dist_track, h, min_m=None):
    if min_m is None: min_m = SAFETY_M
    min_px = min_m / RESOLUTION
    H_, W_ = dist_track.shape
    pxs, pys = world_to_px(x, y, h)
    pxs = np.clip(pxs, 0, W_-1); pys = np.clip(pys, 0, H_-1)
    bad = np.where(dist_track[pys, pxs] < min_px)[0]
    if len(bad) == 0: return x.copy(), y.copy()
    safe_ys, safe_xs = np.where(dist_track >= min_px)
    tree = cKDTree(np.column_stack([safe_xs, safe_ys]))
    xf, yf = x.copy(), y.copy()
    for i in bad:
        _, idx = tree.query([pxs[i], pys[i]])
        xf[i], yf[i] = px_to_world(safe_xs[idx], safe_ys[idx], h)
    return xf, yf


def _refine_curvature_local(d, cx, cy, nx_, ny_, bounds, window=10, guard=4, max_rounds=3):
    """L-BFGS-B 뒤에 붙는 국소 곡률 보정 패스.

    curvature_penalty 는 _curvature()(np.gradient 로 2계 수치미분)를 다시
    scipy 가 유한차분으로 미분해서 쓴다 — 2단 수치미분이라 신호가 노이즈에
    묻혀 L-BFGS-B 가 좁은 코너에서 개선 방향을 못 찾는 경우가 있다.
    위반 구간 주변 몇 포인트(window)만 떼어 Powell(derivative-free)로
    국소 재탐색하면 노이즈에 안 흔들리고 안정적으로 수렴한다. 나머지
    포인트(랩타임 최적화 결과)는 그대로 둔다.

    두 가지 안전장치:
      - 평가범위(eval_idx)를 자유변수 범위(win)보다 넓게 잡는다 — 윈도우
        경계 바로 밖 포인트도 곡률이 이웃(win 안 점)에 의존하므로, 평가에서
        빠지면 Powell이 그 경계로 문제를 떠넘겨 새 스파이크를 만들 수 있다.
      - 그룹 적용 전후로 "전체" 최대곡률을 비교해, 실제로 나빠지면 되돌린다
        (윈도우 국소값만 보고 좋아졌다고 착각하는 걸 방지).
    """
    d = d.copy()
    N = len(d)

    def global_max_kappa(dd):
        px_ = cx + dd * nx_; py_ = cy + dd * ny_
        return float(np.abs(_curvature(px_, py_)).max())

    for _ in range(max_rounds):
        px_ = cx + d * nx_; py_ = cy + d * ny_
        kappa = np.abs(_curvature(px_, py_))
        over = kappa > KAPPA_MAX
        if not over.any():
            break

        idx = np.where(over)[0]
        groups = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
        improved_any = False

        for g in groups:
            lo = max(0, int(g[0]) - window)
            hi = min(N, int(g[-1]) + window + 1)
            win = np.arange(lo, hi)
            elo = max(0, lo - guard)
            ehi = min(N, hi + guard)
            eval_idx = np.arange(elo, ehi)

            def local_excess(sub, win=win, eval_idx=eval_idx):
                dd = d.copy()
                dd[win] = sub
                px_l = cx + dd * nx_; py_l = cy + dd * ny_
                kappa_l = np.abs(_curvature(px_l, py_l))
                excess = np.maximum(0.0, kappa_l[eval_idx] - KAPPA_MAX)
                smooth = np.sum(np.diff(dd[win]) ** 2)
                return float(np.sum(excess ** 2)) + 0.02 * float(smooth)

            base_val = local_excess(d[win])
            res_l = minimize(local_excess, d[win], method='Powell',
                             bounds=[bounds[i] for i in win],
                             options={'maxiter': 800, 'xtol': 1e-6, 'ftol': 1e-10})
            if res_l.fun >= base_val - 1e-9:
                continue

            trial = d.copy()
            trial[win] = res_l.x
            if global_max_kappa(trial) <= global_max_kappa(d) + 1e-9:
                d = trial
                improved_any = True

        if not improved_any:
            break

    return d


def postfix_curvature(rx, ry, dist_track, h):
    """minimum_laptime_opt 직후엔 곡률이 한계 안이어도, 그 뒤 smooth_resample
    (N_OPT->N_FINAL 재보간)이 곡률 제약을 전혀 모른 채 다시 매끄럽게만
    그리면서 헤어핀 곡률이 도로 튀는 경우가 있다. 이 최종 곡선 위에서
    한 번 더 국소보정을 돌려 잡는다 (오프셋 기준이 아니라 절대좌표 곡선에
    직접, normal/bound를 이 곡선 자체에서 새로 구해서 적용)."""
    kappa = np.abs(_curvature(rx, ry))
    if not (kappa > KAPPA_MAX).any():
        return rx, ry

    n_over = int((kappa > KAPPA_MAX).sum())
    print(f'  [최종보정] 재보간 후 곡률 재초과 {n_over}pt '
          f'(|κ|_max={kappa.max():.3f}) — 재보정 중...')

    nx_f, ny_f = _normals(rx, ry)
    pxs_f, pys_f = world_to_px(rx, ry, h)
    pxs_f = np.clip(pxs_f, 0, dist_track.shape[1] - 1)
    pys_f = np.clip(pys_f, 0, dist_track.shape[0] - 1)
    cw_f = dist_track[pys_f, pxs_f] * RESOLUTION
    bounds_f = [(-w, w) for w in np.clip(cw_f - SAFETY_M, 0.05, None)]

    d_fix = _refine_curvature_local(np.zeros(len(rx)), rx, ry, nx_f, ny_f, bounds_f)
    rx2 = rx + d_fix * nx_f
    ry2 = ry + d_fix * ny_f

    kappa2 = np.abs(_curvature(rx2, ry2))
    print(f'  [최종보정] |κ|_max {kappa.max():.3f} -> {kappa2.max():.3f}  '
          f'초과 {n_over} -> {int((kappa2 > KAPPA_MAX).sum())}pt')
    return rx2, ry2


def minimum_laptime_opt(cx, cy, cw, dist_track, h):
    nx_, ny_ = _normals(cx, cy)
    N = len(cx)
    # gen_waypoints.py 원본은 클리어런스 0.6m 미만 구간을 통째로 오프셋 0 으로
    # 잠갔다(NARROW_THRESH) — 옆으로 못 미니 곡률 페널티를 아무리 세게 줘도
    # 좁은 헤어핀에서 손을 못 썼다. 여기서는 안전마진을 제외한 실제 여유
    # (w_avail, 최소 0.05m 보장)를 그대로 바운드로 쓴다 — 곡률 페널티가
    # 좁은 구간에서도 라인을 실제로 밀어낼 수 있게.
    opt_safety = SAFETY_M
    w_avail = np.clip(cw - opt_safety, 0.05, None)
    bounds  = [(-w, w) for w in w_avail]

    coll_thresh = max(SAFETY_M * 0.8, 0.25)

    def laptime_est(d):
        px_ = cx + d * nx_; py_ = cy + d * ny_
        ds  = np.hypot(np.diff(np.append(px_, px_[0])),
                       np.diff(np.append(py_, py_[0])))
        kappa = np.abs(_curvature(px_, py_))
        v = np.clip(np.sqrt(A_LAT_MAX / np.maximum(kappa, EPS_KAPPA)), V_MIN, V_MAX)
        return float(np.sum(ds / v))

    def collision_penalty(d):
        px_ = cx + d * nx_; py_ = cy + d * ny_
        pxs_ = np.clip(((px_ - ORIGIN[0]) / RESOLUTION).astype(int), 0, dist_track.shape[1]-1)
        pys_ = np.clip((h - (py_ - ORIGIN[1]) / RESOLUTION).astype(int), 0, dist_track.shape[0]-1)
        dist_m = dist_track[pys_, pxs_] * RESOLUTION
        return float(np.sum(np.maximum(0.0, coll_thresh - dist_m) ** 2))

    def curvature_penalty(d):
        """조향각 물리 한계(KAPPA_MAX)를 넘는 곡률에 벌점 — 트랙 폭이 허용하는
        한 코너를 아웃-인-아웃으로 넓혀 곡률을 낮추도록 유도한다."""
        px_ = cx + d * nx_; py_ = cy + d * ny_
        kappa = np.abs(_curvature(px_, py_))
        return float(np.sum(np.maximum(0.0, kappa - KAPPA_MAX) ** 2))

    def objective(d):
        d_diff = np.diff(np.append(d, d[0]))
        return (laptime_est(d)
                + SMOOTH_W * float(np.sum(d_diff**2))
                + 300.0 * collision_penalty(d)
                + 1500.0 * curvature_penalty(d))

    T_before = laptime_est(np.zeros(N))
    iters = [0]
    def callback(d):
        iters[0] += 1
        if iters[0] % 100 == 0:
            print(f'    iter {iters[0]:4d}  T~{laptime_est(d):.3f}s')

    res = minimize(objective, np.zeros(N), method='L-BFGS-B', bounds=bounds,
                   callback=callback,
                   options={'maxiter':6000,'ftol':1e-12,'gtol':1e-9,
                            'eps':1e-6,'maxfun':1000000})

    d_opt = res.x
    kappa_pre = np.abs(_curvature(cx + d_opt * nx_, cy + d_opt * ny_))
    n_over_pre = int((kappa_pre > KAPPA_MAX).sum())
    if n_over_pre > 0:
        print(f'  [국소보정] L-BFGS-B 이후 {n_over_pre}개 포인트 곡률 초과 '
              f'(|κ|_max={kappa_pre.max():.3f}) — 위반구간 국소 재탐색 중...')
        d_opt = _refine_curvature_local(d_opt, cx, cy, nx_, ny_, bounds)

    T_after = laptime_est(d_opt)
    print(f'  랩타임: {T_before:.2f}s -> {T_after:.2f}s  '
          f'({(T_before-T_after)/T_before*100:.1f}% 단축)')
    rx = cx + d_opt * nx_; ry = cy + d_opt * ny_

    kappa_final = np.abs(_curvature(rx, ry))
    over_pct = float((kappa_final > KAPPA_MAX).mean() * 100)
    print(f'  |κ|_max={kappa_final.max():.3f} 1/m  '
          f'(한계 {KAPPA_MAX:.3f} = {KAPPA_MARGIN:.0%}·tan(max_steer)/L, '
          f'L={WHEELBASE}m·{np.degrees(MAX_STEER):.1f}°)  '
          f'초과구간={over_pct:.1f}%')
    if over_pct > 0:
        print('  → 국소보정 이후에도 남은 초과 구간 — 트랙 폭으로도 못 피하는 코너'
              '(물리적 최소회전반경보다 좁음). 실주행에서 감속 여유를 더 주거나'
              ' safety 마진을 줄여보세요.')

    return snap_to_safe(rx, ry, dist_track, h)


def speed_profile(x, y):
    n  = len(x)
    dx = np.diff(np.append(x, x[0])); dy = np.diff(np.append(y, y[0]))
    ds = np.clip(np.hypot(dx, dy), 1e-6, None)
    kappa = np.abs(_curvature(x, y))
    v_lim = np.clip(np.sqrt(A_LAT_MAX / np.clip(kappa, 0.01, None)), V_MIN, V_MAX)
    v = v_lim.copy()
    for _ in range(3):
        for i in range(n):
            v[i] = min(v[i], np.sqrt(v[(i-1)%n]**2 + 2*A_LON_ACC*ds[(i-1)%n]))
    for _ in range(3):
        for i in range(n-1, -1, -1):
            v[i] = min(v[i], np.sqrt(v[(i+1)%n]**2 + 2*A_LON_BRK*ds[i]))
    v = np.clip(v, V_MIN, V_MAX)
    total = ds.sum(); avg = total / (ds / np.clip(v, 1e-3, None)).sum()
    print(f'  속도: {v.min():.2f}~{v.max():.2f} m/s  '
          f'랩타임 ~{total/avg:.1f}s  (트랙 {total:.1f}m)')
    return v


# ══════════════════════════════════════════════════════════════════════════════
# 저장
# ══════════════════════════════════════════════════════════════════════════════
def save_overlay_png(map_name, rx, ry, v, wp_path, img_arr):
    """웨이포인트(속도 컬러) 를 맵 위에 올린 오버레이 PNG 저장.

    파일명:  waypoints/<wp_stem>.png  (CSV 와 확장자만 다른 짝, 예: track_2nd_clean_wp.png)

    전역 백엔드(TkAgg)는 디스플레이가 없으면 figure 생성이 실패하므로,
    여기서는 Agg 캔버스를 직접 만들어 headless 환경에서도 항상 저장되게 한다.
    """
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.colors import Normalize

    h = img_arr.shape[0]
    px, py = world_to_px(rx, ry, h)
    # 트랙은 순환 — 시작점을 끝에 이어 라인을 닫는다
    px = np.append(px, px[0]); py = np.append(py, py[0])
    v_ext = np.append(v, v[0])

    fig = Figure(figsize=(9, 9))
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    ax.imshow(img_arr, cmap='gray', origin='upper')
    ax.set_aspect('equal'); ax.axis('off')

    pts  = np.array([px, py]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc   = LineCollection(segs, cmap='RdYlGn',
                          norm=Normalize(V_MIN, V_MAX), linewidth=2.5, zorder=5)
    lc.set_array(v_ext)
    ax.add_collection(lc)
    ax.plot(px[0], py[0], 'b*', ms=15, zorder=6)   # 시작점
    fig.colorbar(lc, ax=ax, label='speed (m/s)', fraction=0.03, pad=0.02)
    ax.set_title(f'{map_name}  —  {len(rx)} waypoints '
                 f'({v.min():.1f}~{v.max():.1f} m/s)', fontsize=10, fontweight='bold')

    out = os.path.join(os.path.dirname(wp_path),
                       f'{os.path.splitext(os.path.basename(wp_path))[0]}.png')
    fig.tight_layout()
    fig.savefig(out, dpi=130, bbox_inches='tight')
    print(f'  오버레이 저장: {out}')


def save_all(map_name, rx, ry, v, wp_path, img_arr):
    """웨이포인트 CSV + 오버레이 PNG + inner/outer_bound.npy 저장"""
    os.makedirs(os.path.dirname(wp_path), exist_ok=True)
    with open(wp_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'speed'])
        for xi, yi, vi in zip(rx, ry, v):
            w.writerow([f'{xi:.4f}', f'{yi:.4f}', f'{vi:.4f}'])
    print(f'  웨이포인트 저장: {len(rx)} pts -> {wp_path}')

    # 웨이포인트를 맵 위에 올린 오버레이 PNG (속도 컬러맵) — 디스플레이 없이도 저장
    save_overlay_png(map_name, rx, ry, v, wp_path, img_arr)

    # 경계 추출 (Q2 장애물 회피용)
    bound_dir = os.path.join(MAPS_DIR, map_name)
    os.makedirs(bound_dir, exist_ok=True)

    h_img = img_arr.shape[0]
    free  = (img_arr > 128).astype(np.uint8)
    labeled, _ = ndimage.label(free)

    # 첫 웨이포인트를 시드로 트랙 라벨 찾기
    sx = int(np.clip(round((rx[0] - ORIGIN[0]) / RESOLUTION), 0, img_arr.shape[1]-1))
    sy = int(np.clip(h_img - round((ry[0] - ORIGIN[1]) / RESOLUTION), 0, h_img-1))
    track_label = labeled[sy, sx]
    if track_label == 0:
        for r in range(1, 10):
            found = False
            for dy in range(-r, r+1):
                for dx in range(-r, r+1):
                    lbl = labeled[max(0,sy+dy), max(0,sx+dx)]
                    if lbl != 0:
                        track_label = lbl; found = True; break
                if found: break
            if found: break

    if track_label == 0:
        print('  경계 추출 실패 (트랙 라벨 없음)')
        return

    track_mask = (labeled == track_label).astype(np.uint8) * 255
    contours, hierarchy = cv2.findContours(
        track_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        print('  경계 추출 실패 (컨투어 없음)')
        return

    outer_c = inner_c = None
    for c, hi in zip(contours, hierarchy[0]):
        area = cv2.contourArea(c)
        if hi[3] < 0:
            if outer_c is None or area > cv2.contourArea(outer_c):
                outer_c = c
        elif area > 200:
            if inner_c is None or area > cv2.contourArea(inner_c):
                inner_c = c

    outer_bound = cv2.approxPolyDP(outer_c, 2, True).reshape(-1, 2).astype(np.int32)
    inner_bound = (cv2.approxPolyDP(inner_c, 2, True).reshape(-1, 2).astype(np.int32)
                   if inner_c is not None
                   else np.array([[0,0],[1,0],[0,1]], dtype=np.int32))

    np.save(os.path.join(bound_dir, 'outer_bound.npy'), outer_bound)
    np.save(os.path.join(bound_dir, 'inner_bound.npy'), inner_bound)
    print(f'  경계 저장: outer={len(outer_bound)}pts, inner={len(inner_bound)}pts -> {bound_dir}')

    # 경계 미리보기 이미지
    vis = cv2.cvtColor((img_arr > 128).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(vis, [outer_bound.reshape(-1,1,2)], -1, (0,0,255), 2)
    cv2.drawContours(vis, [inner_bound.reshape(-1,1,2)], -1, (0,255,0), 2)
    cv2.imwrite(os.path.join(bound_dir, 'bounds_preview.png'), vis)


# ══════════════════════════════════════════════════════════════════════════════
# 스플라인 미리보기
# ══════════════════════════════════════════════════════════════════════════════
def spline_preview_px(clicks, n=400):
    if len(clicks) < 2: return None, None
    pts = np.array(clicks, float)
    pts_closed = np.vstack([pts, pts[0]])
    k = min(3, len(pts_closed) - 1)
    try:
        tck, _ = splprep([pts_closed[:,0], pts_closed[:,1]], s=0, per=False, k=k)
        u = np.linspace(0, 1, n)
        sx, sy = splev(u, tck)
        return sx, sy
    except Exception:
        xs = list(pts[:,0]) + [pts[0,0]]
        ys = list(pts[:,1]) + [pts[0,1]]
        return xs, ys


# ══════════════════════════════════════════════════════════════════════════════
# Drawing App
# ══════════════════════════════════════════════════════════════════════════════
class DrawingApp:
    def __init__(self, map_name, map_base, wp_path, candidates, dist_track):
        self.map_name   = map_name
        self.wp_path    = wp_path
        self.candidates = candidates
        self.dist_track = dist_track

        for ext in ['.pgm', '.png']:
            if os.path.exists(map_base + ext):
                img_path = map_base + ext; break
        else:
            sys.exit(f'이미지 없음: {map_base}')

        self.img_arr = np.array(Image.open(img_path).convert('L'))
        self.H, self.W = self.img_arr.shape

        self.clicks = []
        self.state  = 'drawing'
        self.result = None

        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        try:
            self.fig.canvas.manager.set_window_title(f'gen_waypoints — {map_name}')
        except Exception:
            pass

        self.ax.imshow(self.img_arr, cmap='gray', origin='upper')
        self.ax.set_aspect('equal')
        self.ax.axis('off')

        self.dots,      = self.ax.plot([], [], 'ro', ms=5,  zorder=6)
        self.start_dot, = self.ax.plot([], [], 'y*', ms=13, zorder=7)
        self.preview,   = self.ax.plot([], [], color='limegreen', lw=2, zorder=5, alpha=0.9)
        self.opt_coll   = None
        self.colorbar   = None
        self.cand_lines = []
        self._draw_candidate_lines()
        self._update_title()
        plt.tight_layout()

        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event',    self.on_key)
        self._print_help()

    def _draw_candidate_lines(self):
        n = len(self.candidates)
        if n == 0: return
        sorted_c = sorted(enumerate(self.candidates), key=lambda x: x[1][3])
        colors   = [plt.cm.cool(i / max(n-1,1)) for i in range(n)]
        for rank, (orig_i, (rx, ry, _, tl)) in enumerate(sorted_c):
            px_, py_ = world_to_px(rx, ry, self.H)
            line, = self.ax.plot(
                np.append(px_, px_[0]), np.append(py_, py_[0]),
                color=colors[rank], lw=1.5, alpha=0.55, zorder=3,
                label=f'{tl:.0f}m')
            self.cand_lines.append((orig_i, line))
        if self.cand_lines:
            self.ax.legend(loc='lower right', fontsize=7,
                           title='candidates', title_fontsize=7)

    def _highlight_candidate(self, best_orig_i):
        for orig_i, line in self.cand_lines:
            if orig_i == best_orig_i:
                line.set_linewidth(3.0); line.set_alpha(1.0); line.set_zorder(5)
            else:
                line.set_linewidth(0.8); line.set_alpha(0.15)

    def _print_help(self):
        print('\n' + '='*52)
        print(f'  map:   {self.map_name}')
        print(f'  save:  {self.wp_path}')
        print(f'  cands: {len(self.candidates)} (shown on map)')
        print('='*52)
        print('  L-click  — add point')
        print('  R-click/z — undo last point')
        print('  Enter   — optimize + save (auto-pick candidate)')
        print('  s       — save again')
        print('  r       — reset')
        print('  Esc     — quit')
        print('='*52)

    def _update_title(self):
        n = len(self.clicks)
        if self.state == 'drawing':
            if n == 0:   msg = 'click to draw the path'
            elif n < 3:  msg = f'{n}pts — need at least 3'
            else:        msg = f'{n}pts — Enter: optimize'
        else:
            msg = 'done — s: save again  r: redraw'
        self.ax.set_title(f'[{self.map_name}]  {msg}',
                          fontsize=10, fontweight='bold', pad=6)

    def on_click(self, event):
        if event.inaxes != self.ax: return
        if self.state == 'optimized': return
        if event.xdata is None: return
        px = int(np.clip(round(event.xdata), 0, self.W-1))
        py = int(np.clip(round(event.ydata), 0, self.H-1))
        if event.button == 1:
            self.clicks.append((px, py)); self._update_display()
        elif event.button == 3:
            if self.clicks: self.clicks.pop(); self._update_display()

    def on_key(self, event):
        if event.key == 'enter':
            self._finalize()
        elif event.key in ('z', 'backspace'):
            if self.clicks and self.state == 'drawing':
                self.clicks.pop(); self._update_display()
        elif event.key == 'r':
            self._reset()
        elif event.key == 's':
            self._save()
        elif event.key == 'escape':
            plt.close(self.fig)

    def _update_display(self):
        n = len(self.clicks); self._update_title()
        if n == 0:
            self.dots.set_data([], []); self.start_dot.set_data([], [])
            self.preview.set_data([], [])
        else:
            xs = [c[0] for c in self.clicks]; ys = [c[1] for c in self.clicks]
            self.dots.set_data(xs, ys)
            self.start_dot.set_data([xs[0]], [ys[0]])
            if n >= 2:
                sx, sy = spline_preview_px(self.clicks)
                if sx is not None: self.preview.set_data(sx, sy)
            else:
                self.preview.set_data(xs, ys)
        self.fig.canvas.draw_idle()

    def _reset(self):
        self.clicks = []; self.state = 'drawing'; self.result = None
        if self.opt_coll is not None:
            self.opt_coll.remove(); self.opt_coll = None
        if self.colorbar is not None:
            self.colorbar.remove(); self.colorbar = None
        for _, line in self.cand_lines:
            line.set_linewidth(1.5); line.set_alpha(0.55)
        self.preview.set_data([]); self._update_display()

    def _find_best_candidate(self, drawn_rx, drawn_ry):
        try:
            drx_d, dry_d = smooth_resample(drawn_rx, drawn_ry, 300, s_factor=0.1)
        except Exception:
            drx_d, dry_d = drawn_rx, drawn_ry
        drawn_pts  = np.column_stack([drx_d, dry_d])
        tree_drawn = cKDTree(drawn_pts)
        best_idx = 0; best_score = float('inf')
        for i, (cx, cy, _, tl) in enumerate(self.candidates):
            cand_pts  = np.column_stack([cx, cy])
            tree_cand = cKDTree(cand_pts)
            d1, _ = tree_cand.query(drawn_pts)
            d2, _ = tree_drawn.query(cand_pts)
            score = (d1.mean() + d2.mean()) / 2
            print(f'    cand {i} ({tl:.1f}m): score={score:.3f}m')
            if score < best_score:
                best_score = score; best_idx = i
        print(f'  -> picked cand {best_idx} ({self.candidates[best_idx][3]:.1f}m)')
        return best_idx

    def _finalize(self):
        if self.state == 'optimized': return
        if len(self.clicks) < 3:
            if len(self.candidates) == 1:
                print('  1 candidate → auto-select')
            else:
                print('  need at least 3 points'); return

        print('\n[match + optimize]')
        if len(self.clicks) >= 3:
            pxs = np.array([c[0] for c in self.clicks], float)
            pys = np.array([c[1] for c in self.clicks], float)
            drawn_rx, drawn_ry = px_to_world(pxs, pys, self.H)
        else:
            drawn_rx, drawn_ry = None, None

        if self.candidates:
            best_idx = (self._find_best_candidate(drawn_rx, drawn_ry)
                        if drawn_rx is not None else 0)
            self._highlight_candidate(best_idx)
            cx, cy, _, _ = self.candidates[best_idx]
            cx, cy = arc_resample(cx, cy, N_OPT)
        else:
            if drawn_rx is None:
                print('  no candidates — draw a path first'); return
            s_fac  = 0.0 if len(self.clicks) <= 20 else 0.5
            cx, cy = smooth_resample(drawn_rx, drawn_ry, N_OPT, s_factor=s_fac)

        cx, cy = ensure_ccw(cx, cy)
        cx, cy = snap_to_safe(cx, cy, self.dist_track, self.H)

        H_, W_ = self.dist_track.shape
        pxs2, pys2 = world_to_px(cx, cy, self.H)
        pxs2 = np.clip(pxs2, 0, W_-1); pys2 = np.clip(pys2, 0, H_-1)
        cw   = self.dist_track[pys2, pxs2] * RESOLUTION

        print(f'[optimize] {N_OPT}pts laptime optimization...')
        rx, ry = minimum_laptime_opt(cx, cy, cw, self.dist_track, self.H)
        rx, ry = smooth_resample(rx, ry, N_FINAL, s_factor=0.005)
        rx, ry = snap_to_safe(rx, ry, self.dist_track, self.H, min_m=SAFETY_M)
        rx, ry = postfix_curvature(rx, ry, self.dist_track, self.H)
        rx, ry = snap_to_safe(rx, ry, self.dist_track, self.H, min_m=SAFETY_M)
        v = speed_profile(rx, ry)

        self.result = (rx, ry, v)
        self.state  = 'optimized'
        self._draw_result(rx, ry, v)
        self._update_title()
        self.fig.canvas.draw()
        self._save()
        print('  done. close the window...')
        plt.close(self.fig)

    def _draw_result(self, rx, ry, v):
        if self.opt_coll is not None: self.opt_coll.remove()
        if self.colorbar is not None: self.colorbar.remove()
        opt_px, opt_py = world_to_px(rx, ry, self.H)
        opt_px = np.append(opt_px, opt_px[0])
        opt_py = np.append(opt_py, opt_py[0])
        v_ext  = np.append(v, v[0])
        pts    = np.array([opt_px, opt_py]).T.reshape(-1, 1, 2)
        segs   = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc     = LineCollection(segs, cmap='RdYlGn',
                                norm=plt.Normalize(V_MIN, V_MAX),
                                linewidth=3, zorder=8)
        lc.set_array(v_ext)
        self.opt_coll = self.ax.add_collection(lc)
        self.colorbar = self.fig.colorbar(lc, ax=self.ax,
                                          label='speed (m/s)',
                                          fraction=0.03, pad=0.02)

    def _save(self):
        if self.result is None:
            print('  press Enter first to optimize'); return
        rx, ry, v = self.result
        save_all(self.map_name, rx, ry, v, self.wp_path, self.img_arr)

    def run(self):
        plt.show()


# ══════════════════════════════════════════════════════════════════════════════
# main
# ══════════════════════════════════════════════════════════════════════════════
def _pick_seed_interactive(img_arr, w, h):
    """
    맵 이미지를 띄우고 트랙 내부를 클릭해 시드 픽셀 좌표를 받는다.
    30초 안에 클릭 없으면 이미지 중앙 사용.
    """
    print('\nMap image will open.')
    print('  Click inside the track (corridor) to set the seed point.')
    print('  (If you skip, image center is used.)\n')

    result = [w // 2, h // 2]
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(img_arr, cmap='gray', interpolation='nearest')
    ax.set_title('Click inside the track (seed point)', fontsize=11)
    ax.axis('off')
    dot, = ax.plot([], [], 'r+', ms=18, mew=3)

    def on_click(event):
        if event.inaxes != ax or event.xdata is None:
            return
        px = int(np.clip(round(event.xdata), 0, w - 1))
        py = int(np.clip(round(event.ydata), 0, h - 1))
        result[0], result[1] = px, py
        dot.set_data([px], [py])
        ax.set_title(f'seed pixel: ({px}, {py}) — Enter or close window to confirm', fontsize=10)
        fig.canvas.draw_idle()
        print(f'  seed click: pixel ({px}, {py})')

    def on_key(event):
        if event.key in ('enter', 'escape'):
            plt.close(fig)

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event',    on_key)
    plt.tight_layout()
    plt.show(block=True)

    seed_px, seed_py = result
    print(f'  confirmed seed: pixel ({seed_px}, {seed_py})')
    return seed_px, seed_py


def load_map_config(map_base):
    global RESOLUTION, ORIGIN
    yaml_path = map_base + '.yaml'
    if not os.path.exists(yaml_path):
        print(f'  경고: yaml 없음 ({yaml_path}), 기본값 사용')
        return
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    RESOLUTION  = float(cfg.get('resolution', 0.05))
    origin_raw  = cfg.get('origin', [0.0, 0.0, 0.0])
    ORIGIN      = (float(origin_raw[0]), float(origin_raw[1]))
    print(f'  yaml: resolution={RESOLUTION}  origin={ORIGIN}')


def main():
    global A_LAT_MAX, A_LON_ACC, A_LON_BRK, V_MAX, V_MIN, SAFETY_M
    global WHEELBASE, MAX_STEER, KAPPA_MAX, KAPPA_MARGIN

    import argparse
    wb0, ms0 = _car_steer_geom()
    p = argparse.ArgumentParser(
        description='SLAM 맵에서 레이싱 라인 웨이포인트 생성 (곡률-제약 버전)')
    p.add_argument('--map',    default=None,  help='맵 이름 (생략 시 대화형 선택)')
    p.add_argument('--vmax',   type=float, default=6.0,  help='최대 속도 (m/s)')
    p.add_argument('--vmin',   type=float, default=0.5,  help='최소 속도 (m/s)')
    p.add_argument('--safety', type=float, default=0.2,  help='벽 안전 마진 (m, 실차 복도는 0.2 권장)')
    p.add_argument('--mu',     type=float, default=0.377,
                    help='마찰 계수 — A_LAT_MAX=mu*9.81. 기본값은 curvature_probe '
                         '08/07 실측(A_LAT_MAX=3.7) 역산치, 0.9(=8.83) 아님')
    p.add_argument('--acc',    type=float, default=5.0,  help='가속도 (m/s²)')
    p.add_argument('--brk',    type=float, default=8.0,  help='감속도 (m/s²)')
    p.add_argument('--wheelbase', type=float, default=None,
                    help='축간거리 (m). 생략 시 vehicle overlay / 기본값')
    p.add_argument('--max-steer', type=float, default=None,
                    help='최대 조향각 (rad). 생략 시 vehicle overlay / 기본값')
    p.add_argument('--kappa-margin', type=float, default=KAPPA_MARGIN,
                    help='조향 한계 대비 곡률 여유 (기본 0.85). 1.0=한계까지 붙임')
    p.add_argument('--seed-px', type=int,  default=None, help='트랙 내부 시드 픽셀 X (미지정시 자동/클릭)')
    p.add_argument('--seed-py', type=int,  default=None, help='트랙 내부 시드 픽셀 Y')
    args = p.parse_args()

    WHEELBASE = float(args.wheelbase) if args.wheelbase is not None else wb0
    MAX_STEER = float(args.max_steer) if args.max_steer is not None else ms0
    KAPPA_MARGIN = float(np.clip(args.kappa_margin, 0.5, 1.0))
    KAPPA_MAX = np.tan(MAX_STEER) / WHEELBASE * KAPPA_MARGIN

    map_name = args.map or select_map_interactive()
    map_base = os.path.join(MAPS_DIR, map_name)

    for ext in ['.pgm', '.png']:
        if os.path.exists(map_base + ext):
            break
    else:
        sys.exit(f'맵 이미지 없음: {map_base}.pgm / .png')

    wp_path = os.path.join(WP_DIR, f'{map_name}_wp_kappa.csv')
    load_map_config(map_base)

    A_LAT_MAX = args.mu  * G_GRAV
    A_LON_ACC = args.acc
    A_LON_BRK = args.brk
    V_MAX     = args.vmax
    V_MIN     = args.vmin
    SAFETY_M  = args.safety

    print(f'\n맵     : {map_name}')
    print(f'저장   : {wp_path}')
    print(f'vmax={V_MAX}  vmin={V_MIN}  safety={SAFETY_M}  mu={args.mu}')
    print(f'wheelbase={WHEELBASE}m  max_steer={np.degrees(MAX_STEER):.1f}°  '
          f'margin={KAPPA_MARGIN:.2f}  → κ_max={KAPPA_MAX:.3f} 1/m '
          f'(최소회전반경 {1/KAPPA_MAX:.2f}m)')

    img_arr = np.array(Image.open(
        map_base + ('.pgm' if os.path.exists(map_base+'.pgm') else '.png')
    ).convert('L'))

    h, w = img_arr.shape

    # 시드 픽셀 결정: CLI > 클릭 선택 > 이미지 중앙
    if args.seed_px is not None and args.seed_py is not None:
        seed_px, seed_py = args.seed_px, args.seed_py
        print(f'시드: 픽셀 ({seed_px}, {seed_py})  [CLI 지정]')
    else:
        seed_px, seed_py = _pick_seed_interactive(img_arr, w, h)

    print('\n[전처리] 스켈레톤 후보 추출 중...')
    candidates, dist_track, _ = extract_candidates(img_arr, seed_px, seed_py)

    if len(candidates) == 0:
        print('\n  후보 경로가 없습니다.')
        print('  권장 해결 방법:')
        print('   1) clean_map.py 로 맵을 먼저 정제하고 track_2_clean 맵 사용')
        print('   2) --seed-px/--seed-py 로 트랙 안쪽 픽셀 좌표 직접 지정')
        print('   3) --safety 값을 더 줄여서 재시도 (예: --safety 0.1)')

    print(f'  -> {len(candidates)}개 후보 준비\n')

    app = DrawingApp(map_name, map_base, wp_path, candidates, dist_track)
    app.run()

    # race_script.yaml 안내
    print('\n=== 다음 단계 ===')
    print(f'race_script.yaml 에서:')
    print(f'  map_name:  {map_name}')
    print(f'  waypoints: {map_name}_wp_kappa   (기존 {map_name}_wp 와 랩타임 비교용)')


if __name__ == '__main__':
    main()
