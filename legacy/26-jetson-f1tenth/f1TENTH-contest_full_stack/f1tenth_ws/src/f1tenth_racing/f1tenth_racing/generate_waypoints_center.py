#!/usr/bin/env python3
"""
F1TENTH 중심선 추출기 (시각화용)
벽에서 멀수록 초록, 가까울수록 빨강으로 스켈레톤 표시
"""

import numpy as np
from PIL import Image
import scipy.ndimage as ndimage
import argparse
import yaml
import os
import csv

_THIS_DIR  = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
_MAPS_DIR  = os.path.join(_REPO_ROOT, 'sim_ws', 'src', 'f1tenth_gym_ros', 'maps')

RESOLUTION  = 0.05
ORIGIN      = (-10.0, -10.0)
SEED_ROS_X  = None
SEED_ROS_Y  = None
SEED_THETA  = None


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--map',    default=None)
    p.add_argument('--seed-x', type=float, default=None)
    p.add_argument('--seed-y', type=float, default=None)
    return p.parse_args()


def select_map():
    files = os.listdir(_MAPS_DIR)
    pngs  = {f[:-4] for f in files if f.endswith('.png')}
    yamls = {f[:-5] for f in files if f.endswith('.yaml')}
    maps  = sorted(pngs & yamls)

    if not maps:
        print('사용 가능한 맵이 없습니다.')
        return None

    print('\n사용 가능한 맵:')
    for i, m in enumerate(maps, 1):
        wp  = os.path.join(_MAPS_DIR, f'{m}_center.jpg')
        tag = ' [시각화 있음]' if os.path.exists(wp) else ''
        print(f'  {i}. {m}{tag}')

    print()
    sel = input('맵 선택 (번호 또는 이름): ').strip()
    if sel.isdigit() and 1 <= int(sel) <= len(maps):
        return maps[int(sel) - 1]
    if sel in maps:
        return sel
    print(f'오류: "{sel}" 맵을 찾을 수 없습니다.')
    return None


def click_seed(map_name, origin, res, img_h):
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg

    img_path = os.path.join(_MAPS_DIR, f'{map_name}.png')
    if not os.path.exists(img_path):
        for ext in ['.pgm']:
            p = os.path.join(_MAPS_DIR, f'{map_name}{ext}')
            if os.path.exists(p):
                img_path = p
                break

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(mpimg.imread(img_path), cmap='gray', origin='upper')
    ax.set_title(f'{map_name}  —  1: 시작점  2: 방향', fontsize=13, fontweight='bold')
    ax.set_xlabel('pixel x')
    ax.set_ylabel('pixel y')
    plt.tight_layout()

    print('  맵 이미지가 열립니다. 시작점 클릭(1번) → 주행 방향 클릭(2번)')
    pts = plt.ginput(2, timeout=120)
    plt.close()

    if len(pts) < 1:
        return None, None, None

    px1, py1 = pts[0]
    sx = round(px1 * res + origin[0], 3)
    sy = round((img_h - py1) * res + origin[1], 3)

    stheta = None
    if len(pts) >= 2:
        px2, py2 = pts[1]
        dx =  (px2 - px1)
        dy = -(py2 - py1)
        stheta = round(float(np.arctan2(dy, dx)), 4)

    print(f'  시작점: ROS ({sx}, {sy})')
    if stheta is not None:
        print(f'  방향  : {np.degrees(stheta):.1f}° ({stheta} rad)')
    return sx, sy, stheta


def extract_track_mask(img, h):
    free    = (img > 200).astype(np.uint8)
    labeled, _ = ndimage.label(free)

    label_id = 0
    if SEED_ROS_X is not None and SEED_ROS_Y is not None:
        spx = int(np.clip((SEED_ROS_X - ORIGIN[0]) / RESOLUTION, 0, free.shape[1]-1))
        spy = int(np.clip(h - (SEED_ROS_Y - ORIGIN[1]) / RESOLUTION, 0, free.shape[0]-1))
        label_id = int(labeled[spy, spx])

    if label_id == 0:
        counts = np.bincount(labeled.ravel()); counts[0] = 0
        edge = set(np.unique(np.concatenate([
            labeled[0,:], labeled[-1,:], labeled[:,0], labeled[:,-1]])))
        for e in edge:
            if e < len(counts): counts[e] = 0
        label_id = int(counts.argmax())

    return (labeled == label_id).astype(np.uint8)


# ── 루트 CSV 저장 헬퍼 ────────────────────────────────────────────────────────

def _find_routes(nodes, edges):
    """그래프에서 모든 단순 사이클 탐색 후 중복 제거"""
    adj = {nid: [] for nid in nodes}
    for eidx, (a, b, _) in enumerate(edges):
        adj[a].append((b, eidx))
        adj[b].append((a, eidx))

    start = list(nodes.keys())[0]
    found = []

    def dfs(cur, node_path, edge_path, used):
        for nb, eidx in adj[cur]:
            if eidx in used:
                continue
            if nb == start and len(node_path) > 1:
                found.append((node_path[:], edge_path + [eidx]))
            elif nb not in node_path:
                dfs(nb, node_path + [nb], edge_path + [eidx], used | {eidx})

    dfs(start, [start], [], set())

    seen, unique = set(), []
    for ns, es in found:
        key = frozenset(es)
        if key not in seen:
            seen.add(key)
            unique.append((ns, es))
    return unique


def _chain_route(node_seq, edge_seq, edges, origin, res, h):
    """노드/엣지 시퀀스 → ROS (x,y) 리스트"""
    ox, oy = origin
    pts = []
    for step, eidx in enumerate(edge_seq):
        _, _, path = edges[eidx]
        if not path:
            continue
        # 이전 마지막 점과의 거리로 방향 결정
        if pts:
            last_x, last_y = pts[-1]
            head_r, head_c = path[0]
            tail_r, tail_c = path[-1]
            head_x = head_c * res + ox
            head_y = (h - head_r) * res + oy
            tail_x = tail_c * res + ox
            tail_y = (h - tail_r) * res + oy
            dist_head = (head_x - last_x) ** 2 + (head_y - last_y) ** 2
            dist_tail = (tail_x - last_x) ** 2 + (tail_y - last_y) ** 2
            ordered = path if dist_head <= dist_tail else list(reversed(path))
        else:
            ordered = path
        for row, col in ordered:
            pts.append((col * res + ox, (h - row) * res + oy))
    return pts


def _resample(pts, spacing=0.20):
    """균등 간격으로 재샘플"""
    arr = np.array(pts)
    if len(arr) < 2:
        return pts
    dists = np.linalg.norm(np.diff(arr, axis=0), axis=1)
    cum = np.concatenate([[0], np.cumsum(dists)])
    total = cum[-1]
    if total < spacing:
        return pts
    s = np.arange(0, total, spacing)
    x = np.interp(s, cum, arr[:, 0])
    y = np.interp(s, cum, arr[:, 1])
    return list(zip(x.tolist(), y.tolist()))


def _reorder(pts, seed_x, seed_y):
    """시드 위치와 가장 가까운 점을 시작점으로 재정렬"""
    if seed_x is None or not pts:
        return pts
    arr = np.array(pts)
    dists = np.hypot(arr[:, 0] - seed_x, arr[:, 1] - seed_y)
    idx = int(np.argmin(dists))
    return pts[idx:] + pts[:idx]


def _save_csv(pts, path):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'speed'])
        for x, y in pts:
            w.writerow([round(float(x), 4), round(float(y), 4), 2.0])


def _trace_single_loop(pruned, seed_x, seed_y, origin, res, h):
    """분기 없는 단순 루프 → 시드에서 시작하는 순서 있는 픽셀 리스트"""
    ox, oy = origin
    py_, px_ = np.where(pruned)
    pixels = set(zip(py_.tolist(), px_.tolist()))

    seed_px = int((seed_x - ox) / res) if seed_x is not None else int(px_[0])
    seed_py = int(h - (seed_y - oy) / res) if seed_y is not None else int(py_[0])
    dists   = np.hypot(px_ - seed_px, py_ - seed_py)
    si      = int(np.argmin(dists))
    start   = (int(py_[si]), int(px_[si]))

    def nb(p):
        r, c = p
        return [(r+dr, c+dc) for dr in [-1,0,1] for dc in [-1,0,1]
                if (dr,dc) != (0,0) and (r+dr,c+dc) in pixels]

    path, visited, cur = [start], {start}, start
    while True:
        nbs = [n for n in nb(cur) if n not in visited]
        if not nbs:
            break
        cur = nbs[0]
        path.append(cur)
        visited.add(cur)

    pts = [(c * res + ox, (h - r) * res + oy) for r, c in path]
    return _resample(pts, 0.20)


def main():
    global RESOLUTION, ORIGIN, SEED_ROS_X, SEED_ROS_Y, SEED_THETA

    args = parse_args()

    if args.map:
        map_name = os.path.splitext(os.path.basename(args.map))[0]
        map_base = args.map
    else:
        print('=== 중심선 시각화 ===')
        map_name = select_map()
        if map_name is None:
            return
        map_base = os.path.join(_MAPS_DIR, map_name)

    map_path = None
    for ext in ['.png', '.pgm']:
        if os.path.exists(map_base + ext):
            map_path = map_base + ext
            break
    if map_path is None:
        print(f'오류: 맵 파일 없음 ({map_base})'); return

    yaml_path = map_base + '.yaml'
    if os.path.exists(yaml_path):
        with open(yaml_path) as f:
            cfg = yaml.safe_load(f)
        RESOLUTION = float(cfg.get('resolution', 0.05))
        orig = cfg.get('origin', [-10.0, -10.0, 0.0])
        ORIGIN = (float(orig[0]), float(orig[1]))

    img  = np.array(Image.open(map_path).convert('L'))
    h, w = img.shape

    if args.seed_x is not None and args.seed_y is not None:
        SEED_ROS_X = args.seed_x
        SEED_ROS_Y = args.seed_y
    else:
        sx, sy, stheta = click_seed(map_name, ORIGIN, RESOLUTION, h)
        if sx is None:
            print('시작점 클릭 없음 → 취소'); return
        SEED_ROS_X = sx
        SEED_ROS_Y = sy
        SEED_THETA = stheta

    # ── [1/5] 트랙 마스크 ────────────────────────────────────────────────────
    print('[1/5] 트랙 마스크 추출...')
    track = extract_track_mask(img, h)
    print(f'  트랙: {track.sum()}px ({track.sum()*RESOLUTION**2:.1f} m²)')

    # ── [2/5] 거리 변환 + 스켈레톤 ──────────────────────────────────────────
    print('[2/5] 거리 변환 + 스켈레톤...')
    dist  = ndimage.distance_transform_edt(track)
    from skimage.morphology import skeletonize
    skel  = skeletonize(track)
    py_, px_ = np.where(skel)
    dists_m  = dist[py_, px_] * RESOLUTION
    print(f'  스켈레톤: {skel.sum()}px  벽거리 {dists_m.min():.2f}~{dists_m.max():.2f} m')

    # ── [3/5] DFS 메인 루프 + 분기점 클러스터링 ─────────────────────────────
    print('[3/5] 메인 루프 탐색 (DFS) + 분기점 추출...')
    import sys
    sys.setrecursionlimit(10000)

    py_, px_ = np.where(skel)
    seed_px = int((SEED_ROS_X - ORIGIN[0]) / RESOLUTION)
    seed_py = int(h - (SEED_ROS_Y - ORIGIN[1]) / RESOLUTION)
    dists_to_seed = np.hypot(px_ - seed_px, py_ - seed_py)
    idx   = np.argmin(dists_to_seed)
    start = (int(py_[idx]), int(px_[idx]))
    print(f'  시작 픽셀: {start}  (시드에서 {dists_to_seed[idx]*RESOLUTION:.2f}m)')

    pixels = set(zip(py_.tolist(), px_.tolist()))

    def get_neighbors(p):
        r, c = p
        return [(r+dr, c+dc) for dr in [-1,0,1] for dc in [-1,0,1]
                if (dr, dc) != (0, 0) and (r+dr, c+dc) in pixels]

    kept = set()

    def dfs(current, path, in_path):
        found = False
        for nb in get_neighbors(current):
            if nb == start and len(path) > 2:
                kept.update(path)
                found = True
            elif nb not in in_path:
                in_path.add(nb)
                path.append(nb)
                if dfs(nb, path, in_path):
                    found = True
                path.pop()
                in_path.remove(nb)
        return found

    dfs(start, [start], {start})

    pruned = np.zeros_like(skel)
    kept.add(start)
    for r, c in kept:
        pruned[r, c] = 1

    py_p, px_p = np.where(pruned)
    if len(py_p) > 0:
        dists_p = dist[py_p, px_p] * RESOLUTION
        print(f'  제거: {skel.sum()-pruned.sum()}px → 남음: {pruned.sum()}px  '
              f'벽거리 {dists_p.min():.2f}~{dists_p.max():.2f} m')
    else:
        print('  ⚠ 루프를 찾지 못함')
        dists_p = np.array([0.0])

    kernel         = np.array([[1,1,1],[1,0,1],[1,1,1]], dtype=np.uint8)
    neighbor_count = ndimage.convolve(pruned.astype(np.uint8), kernel,
                                      mode='constant', cval=0)
    junction_mask             = (pruned == 1) & (neighbor_count >= 3)
    junction_labeled, n_junc  = ndimage.label(junction_mask)
    centers = ndimage.center_of_mass(junction_mask, junction_labeled,
                                     range(1, n_junc + 1))
    nodes = {cid: centers[cid - 1] for cid in range(1, n_junc + 1)}
    print(f'  분기점 클러스터: {n_junc}개')

    # ── [4/5] 그래프 구축 + 단순화 ──────────────────────────────────────────
    print('[4/5] 그래프 구축...')
    pixel_cluster  = junction_labeled.copy()
    regular_pixels = set(
        zip(*np.where((pruned == 1) & (pixel_cluster == 0)))
    )

    def trace_edge(start_px, from_cid):
        path, visited, current = [start_px], {start_px}, start_px
        while True:
            r, c = current
            to_cid = None
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0: continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w:
                        cid = pixel_cluster[nr, nc]
                        if cid > 0 and cid != from_cid:
                            to_cid = cid
            if to_cid is not None:
                return path, to_cid
            nxt = None
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0: continue
                    nb = (r + dr, c + dc)
                    if nb in regular_pixels and nb not in visited:
                        nxt = nb
                        break
                if nxt: break
            if nxt is None:
                return path, None
            path.append(nxt)
            visited.add(nxt)
            current = nxt

    edges, traced = [], set()
    for cid in range(1, n_junc + 1):
        ry, rx = np.where(junction_labeled == cid)
        for r, c in zip(ry, rx):
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0: continue
                    nb = (r + dr, c + dc)
                    if nb in regular_pixels and nb not in traced:
                        path, to_cid = trace_edge(nb, cid)
                        if to_cid is not None:
                            edges.append((cid, to_cid, path))
                        traced.update(path)

    # degree-2 노드 제거
    iters = 0
    while True:
        adj = {nid: [] for nid in nodes}
        for eidx, (a, b, _) in enumerate(edges):
            adj[a].append((b, eidx))
            adj[b].append((a, eidx))
        deg2 = next((nid for nid, nbrs in adj.items() if len(nbrs) == 2), None)
        if deg2 is None:
            break
        (n1, e1_idx), (n2, e2_idx) = adj[deg2]
        _, _, path1 = edges[e1_idx]
        _, _, path2 = edges[e2_idx]

        # deg2 중심 기준으로 각 path의 방향 정렬
        # path1: 끝점이 deg2 쪽 → tail이 deg2 근처가 되도록
        # path2: 시작점이 deg2 쪽 → head가 deg2 근처가 되도록
        deg2_cy, deg2_cx = nodes[deg2]
        def d2(pix):
            return (pix[0] - deg2_cy) ** 2 + (pix[1] - deg2_cx) ** 2

        if path1 and d2(path1[0]) < d2(path1[-1]):
            path1 = list(reversed(path1))
        if path2 and d2(path2[-1]) < d2(path2[0]):
            path2 = list(reversed(path2))

        edges.append((n1, n2, path1 + path2))
        for idx in sorted([e1_idx, e2_idx], reverse=True):
            edges.pop(idx)
        del nodes[deg2]
        iters += 1

    print(f'  단순화 후: 노드 {len(nodes)}개  엣지 {len(edges)}개  ({iters}회 병합)')
    for i, (a, b, path) in enumerate(edges):
        print(f'    E{i}: N{a} — N{b}  ({len(path)}px, {len(path)*RESOLUTION:.1f}m)')

    # ── [4.5/5] 루트 CSV 저장 ────────────────────────────────────────────────
    print('[4.5/5] 루트 CSV 저장...')
    saved_routes = []   # 시각화용: [(pts_ros, label), ...]

    if len(nodes) == 0:
        pts = _trace_single_loop(pruned, SEED_ROS_X, SEED_ROS_Y, ORIGIN, RESOLUTION, h)
        if pts:
            out = os.path.join(_MAPS_DIR, f'{map_name}_route1.csv')
            _save_csv(pts, out)
            saved_routes.append((pts, 'route1'))
            print(f'  route1 (단일 루프): {len(pts)}pts → {out}')
        else:
            print('  경로 추출 실패')
    else:
        routes = _find_routes(nodes, edges)
        if not routes:
            print('  루트 탐색 실패 (그래프 구조 확인 필요)')
        else:
            ox_s, oy_s = ORIGIN
            seed_px_s = int((SEED_ROS_X - ox_s) / RESOLUTION) if SEED_ROS_X is not None else None
            seed_py_s = int(h - (SEED_ROS_Y - oy_s) / RESOLUTION) if SEED_ROS_Y is not None else None

            saved = 0
            for ridx, (node_seq, edge_seq) in enumerate(routes):
                raw_pts = _chain_route(node_seq, edge_seq, edges, ORIGIN, RESOLUTION, h)

                if seed_px_s is not None:
                    arr = np.array(raw_pts)
                    seed_ros_x = seed_px_s * RESOLUTION + ox_s
                    seed_ros_y = (h - seed_py_s) * RESOLUTION + oy_s
                    min_dist = float(np.min(np.hypot(arr[:, 0] - seed_ros_x,
                                                     arr[:, 1] - seed_ros_y)))
                    if min_dist > RESOLUTION * 20:
                        print(f'  route{ridx+1} 제외 (시드와 최근접 거리 {min_dist:.2f}m)')
                        continue

                pts = _resample(raw_pts, 0.20)
                pts = _reorder(pts, SEED_ROS_X, SEED_ROS_Y)
                label = f'route{saved+1}'
                out = os.path.join(_MAPS_DIR, f'{map_name}_{label}.csv')
                _save_csv(pts, out)
                saved_routes.append((pts, label))
                print(f'  {label}: {len(pts)}pts → {out}')
                saved += 1
            print(f'  총 {saved}개 루트 저장 완료')

    # ── [5/5] 시각화 저장 ────────────────────────────────────────────────────
    print('[5/5] 시각화 저장...')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize

    ROUTE_COLORS = ['#e63946', '#2196f3', '#4caf50', '#ff9800', '#9c27b0']

    fig, axes = plt.subplots(1, 2, figsize=(18, 10))

    # ── 왼쪽: 최종 센터라인 (잔가지 제거 후) ──
    vmin = dists_p.min() if len(dists_p) else 0
    vmax = dists_p.max() if len(dists_p) else 1
    norm = Normalize(vmin=vmin, vmax=vmax)

    axes[0].imshow(img, cmap='gray', origin='upper', alpha=0.6)
    if len(py_p) > 0:
        sc = axes[0].scatter(px_p, py_p, c=dists_p, cmap='RdYlGn',
                             s=2, linewidths=0, norm=norm, zorder=3)
        plt.colorbar(sc, ax=axes[0], label='dist to wall (m)', shrink=0.7)

    if len(nodes) > 0:
        node_cy = np.array([nodes[c][0] for c in nodes])
        node_cx = np.array([nodes[c][1] for c in nodes])
        axes[0].scatter(node_cx, node_cy, c='red', s=80, marker='o',
                        zorder=6, label=f'junction ({len(nodes)})')
        for cid, (cy, cx) in nodes.items():
            axes[0].text(cx + 3, cy - 3, str(cid), color='red', fontsize=8, zorder=7)

    axes[0].scatter([seed_px], [seed_py], c='blue', s=100,
                    marker='*', zorder=7, label='seed')
    if SEED_THETA is not None:
        dx =  np.cos(SEED_THETA) / RESOLUTION * 10
        dy = -np.sin(SEED_THETA) / RESOLUTION * 10
        axes[0].annotate('', xy=(seed_px + dx, seed_py + dy),
                         xytext=(seed_px, seed_py),
                         arrowprops=dict(arrowstyle='->', color='blue', lw=2))
    axes[0].legend(loc='lower right', fontsize=9, markerscale=1.5, framealpha=0.7)
    axes[0].set_title(f'centerline  ({len(nodes)} junctions, {len(edges)} edges)', fontsize=12)
    axes[0].axis('off')

    # ── 오른쪽: 엣지(경로 조각)를 색별로 표시 ──
    axes[1].imshow(img, cmap='gray', origin='upper', alpha=0.6)

    ox_v, oy_v = ORIGIN
    MAX_STEP_PX = 5.0

    for ridx, (pts, label) in enumerate(saved_routes):
        if not pts:
            continue
        arr = np.array(pts)
        px_r = (arr[:, 0] - ox_v) / RESOLUTION
        py_r = h - (arr[:, 1] - oy_v) / RESOLUTION
        col  = ROUTE_COLORS[ridx % len(ROUTE_COLORS)]

        # 연속 점 간 거리 초과 시 NaN으로 끊음
        dists = np.hypot(np.diff(px_r), np.diff(py_r))
        px_plot = px_r.tolist()
        py_plot = py_r.tolist()
        for i in range(len(dists) - 1, -1, -1):
            if dists[i] > MAX_STEP_PX:
                px_plot.insert(i + 1, float('nan'))
                py_plot.insert(i + 1, float('nan'))

        # 루프 닫기 (마지막 점 → 첫 점)
        px_plot.append(px_r[0])
        py_plot.append(py_r[0])

        axes[1].plot(px_plot, py_plot, color=col, linewidth=2.0, zorder=4,
                     label=f'{label} ({len(pts)}pts)')
        axes[1].scatter(px_r[0], py_r[0], color=col, s=60, marker='o', zorder=5)

    if not saved_routes:
        axes[1].text(0.5, 0.5, '저장된 루트 없음', transform=axes[1].transAxes,
                     ha='center', va='center', fontsize=13, color='gray')

    axes[1].scatter([seed_px], [seed_py], c='blue', s=100,
                    marker='*', zorder=7, label='seed')
    axes[1].legend(loc='lower right', fontsize=9, markerscale=1.5, framealpha=0.7)
    axes[1].set_title(f'routes  ({len(saved_routes)}개)', fontsize=12)
    axes[1].axis('off')

    fig.suptitle(f'{map_name}  —  centerline & routes', fontsize=14)
    plt.tight_layout()

    out = os.path.join(_MAPS_DIR, f'{map_name}_center.jpg')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  저장: {out}')
    print('완료!')


if __name__ == '__main__':
    main()
